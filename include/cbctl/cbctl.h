/* cbctl — controller-side library for cbplc PLC boards.
 *
 * THE ONE RULE: nothing here blocks. There is no timeout parameter in this header, and no
 * call waits for a reply. You declare the state you want; cbctl_tick() reconciles it against
 * the board, on cadence, forever.
 *
 * That is not a style preference. A synchronous request costs up to 3 x (1000 ms timeout +
 * 100 ms backoff) = 3300 ms of retries, while every OTHER board on this controller is holding
 * it to a 1500 ms feed deadline (CBPLC_LIMITS_STALE_MS). One blocking call to one wedged
 * board therefore terminates the charging sessions on all the others. See
 * docs/03-concurrency-and-fleet.md section 1.
 *
 * Read docs/01-api-reference.md for what to call and in what order.
 */
#ifndef CBCTL_H
#define CBCTL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbctl/units.h"

#include "cbplc/protocol_generated.h"
#include "cbplc/safety.h"
#include "cbplc/session.h"
#include "cbplc/transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- tunables ------------------------------------------------------------------------ */

/* cbplc_poll() moves AT MOST one frame each way per call, and cbplc_request() is
 * stop-and-wait. So a tick that called poll once could not complete even a single
 * request/response, and the feed would never leave the controller. The tick therefore drains
 * up to this many polls and then yields unconditionally.
 *
 * This is the bounded quantum that gives the fleet its isolation property (F3): whatever a
 * board does, the work one link can demand of a tick has a ceiling. */
#ifndef CBCTL_POLL_PER_TICK
#define CBCTL_POLL_PER_TICK 16u
#endif

/* Session timing, deliberately tighter than the schema defaults — a controller on a 400 ms
 * cadence sharing a stop-and-wait wire with a relay dead-man cannot afford a 1000 ms x3 retry.
 * Derivation and the failure it prevents are in src/link.c. */
#ifndef CBCTL_REQUEST_TIMEOUT_MS
#define CBCTL_REQUEST_TIMEOUT_MS 250u
#endif
#ifndef CBCTL_RETRY_BACKOFF_MS
#define CBCTL_RETRY_BACKOFF_MS 60u
#endif
#ifndef CBCTL_RETRY_LIMIT
#define CBCTL_RETRY_LIMIT 2u
#endif
/* Longest one unanswered request can block every other message. */
#define CBCTL_WORST_BLOCK_MS \
    (CBCTL_RETRY_LIMIT * (CBCTL_REQUEST_TIMEOUT_MS + CBCTL_RETRY_BACKOFF_MS))
/* HOW MANY RENEWALS FIT INSIDE ONE HOLD.
 *
 * Six, not three. The dead-man exists to catch a controller that has DIED; a controller that
 * is merely LATE must never trip it. On a real charger the same process runs OCPP, an HMI and
 * the control loop, and it will be descheduled — for tens of milliseconds routinely, and for
 * far longer under memory pressure, TLS handshakes or log flushes. Every one of those stalls
 * is a renewal that did not happen on time.
 *
 * At hold/6 the lease survives losing FIVE consecutive renewal opportunities. At the old
 * hold/3 it survived two. The cost is a few extra 3-byte messages per second on a link that
 * is otherwise idle 99% of the time; the benefit is that a contactor does not open because a
 * Raspberry Pi was busy. */
#define CBCTL_RELAY_RENEW_DIVISOR 6u

/* Past this fraction of the hold with no CONFIRMED renewal, the lease is in danger and its
 * renewal PREEMPTS the feed. Normally the feed outranks it — the feed is what holds permission
 * to energise — but a lease about to lapse drops a contactor NOW, which is worse than a feed
 * repeat arriving one tick late against its own 1500 ms bound. */
#define CBCTL_RELAY_DANGER_DIVISOR 2u

/* Shortest relay hold that cannot be lapsed by one blocked request:
 * hold > WORST_BLOCK + hold/DIVISOR. With DIVISOR 6 that is hold > 1.2 x WORST_BLOCK; the
 * 2x + 400 below is deliberately more generous than the algebra requires. */
#define CBCTL_RELAY_HOLD_MIN_MS (CBCTL_WORST_BLOCK_MS * 2u + 400u)

/* What a production charger should actually ask for.
 *
 * Not a limit — a recommendation the examples use and the docs justify. Ten seconds gives
 * renewal every ~1.7 s and survives an eight-second stall of the whole controller process,
 * which is far beyond anything a healthy Pi does and still an order of magnitude below the
 * board's 30 s ceiling. A SHORTER hold buys nothing: the dead-man's job is to open the
 * contactor when the controller is gone, and a controller that has been gone ten seconds is
 * just as gone as one that has been gone three. */
#define CBCTL_RELAY_HOLD_RECOMMENDED_MS 10000u

/* How old the observed snapshot may get before the link is reported DEGRADED.
 *
 * Every cbctl_flag()/cbctl_counter()/cbctl_vehicle_mac() answer is LAST KNOWN GOOD: it is
 * whatever the most recent complete STATUS said. Without a bound on that, a board that stops
 * answering STATUS — or one whose STATUS is truncated every time — leaves a controller reading
 * "vehicle present, session active" indefinitely and acting on history. Four missed polls. */
#ifndef CBCTL_OBSERVATION_STALE_MS
#define CBCTL_OBSERVATION_STALE_MS 4000u
/* An EV target older than this is not a demand any more. Freshness is the honest test: cbplc
 * keeps echoing the last vehicle's request between sessions, and its `secc_active` flag is NOT a
 * reliable "a session is live" signal — it reads 0 while DC requests are still flowing. Gating on
 * that flag blocked the module command entirely and the DCDC never started. */
#define CBCTL_EV_TARGET_STALE_MS 6000u
#endif

/* Shortest AUTHORIZE lease that cannot be lapsed by one unanswered request, derived from the
 * same blocking budget as the relay hold. Refused rather than raised: unlike a relay hold —
 * where a longer dead-man is merely less safe and silently raising it keeps the contactor
 * governed — an authorisation lease is a policy decision the caller owns, and quietly
 * lengthening it would grant more permission than was asked for. */
#define CBCTL_AUTH_LEASE_MIN_SAFE_MS ((CBCTL_WORST_BLOCK_MS * 3u) / 2u + 200u)

/* How many times the board may undo a close we are holding before cbctl gives up on it.
 *
 * MEASURED, not guessed. Commanding a gun closed with the board's preconditions unmet produces
 * exactly one safety-open per renewal — observed on hardware climbing 23,25,27,30,32,34 against
 * renewals 3,5,7,10,12,14 — while EVERY command is ACKed with gate 0. There is no error to
 * notice: the controller is simply fighting the safety layer, and the relay actuates on every
 * cycle. That is audible chatter and real contactor wear, and it will run forever.
 *
 * Three is enough to distinguish a genuine fight from one transient open during a state
 * change. */
#ifndef CBCTL_RELAY_OVERRIDE_LIMIT
#define CBCTL_RELAY_OVERRIDE_LIMIT 3u
/* How many times a refused relay close may retry AT ONCE before falling back to the ordinary
 * renewal cadence. A transient gate clears within a retry or two; a standing one (authorisation
 * not yet granted) must not be allowed to retry at tick rate, because those retries starve the
 * AUTHORIZE renewal that would clear it. */
#define CBCTL_RELAY_REFUSAL_FAST_RETRIES 2u
#endif

/* Consecutive lost requests before a link is quarantined. */
/* A controller driving more than this many PLCs is unusual; raise it freely. */
#ifndef CBCTL_FLEET_MAX
#define CBCTL_FLEET_MAX 8u
#endif

#ifndef CBCTL_LOSS_LIMIT
#define CBCTL_LOSS_LIMIT 5u
#endif

/* ---- results ------------------------------------------------------------------------- */

typedef enum {
    CBCTL_OK = 0,
    CBCTL_E_ARG,            /* a nonsense argument — programming error, not a charger state */
    CBCTL_E_NOT_IDENTIFIED, /* the board has not said who it is yet */
    CBCTL_E_WRONG_STATE,    /* legal call, illegal right now; the state is named in the link */
    CBCTL_E_CONTRADICTORY,  /* power_ready while stopping, and similar self-cancelling asks  */
    CBCTL_E_WRONG_BOARD,    /* identified, but not the board this link was pinned to */
    CBCTL_E_QUARANTINED,    /* the link is wedged and is being re-probed on a backoff        */
    CBCTL_E_TRANSPORT       /* the adapter itself failed                                     */
} cbctl_status_t;

/* ---- link lifecycle ------------------------------------------------------------------- */

typedef enum {
    CBCTL_LINK_CLOSED = 0,
    CBCTL_LINK_IDENTIFYING, /* asking WHICH board; nothing else is legal yet */
    CBCTL_LINK_READY,
    /* Answering, but what we BELIEVE about the charger has gone stale — STATUS stopped
     * arriving or keeps arriving truncated. The link is alive; our picture of it is not. */
    CBCTL_LINK_DEGRADED,
    CBCTL_LINK_QUARANTINED  /* not answering; contained so it cannot consume the loop        */
} cbctl_link_state_t;

/* ---- intents and their observation ----------------------------------------------------- */

/* An operation handle. Zero is "never submitted". */
typedef struct { uint32_t id; } cbctl_op_t;

typedef enum {
    CBCTL_OP_NONE = 0,
    CBCTL_OP_PENDING,  /* in flight, or waiting its turn — NOT applied */
    CBCTL_OP_APPLIED,  /* the board acked it */
    CBCTL_OP_REFUSED,  /* the board said no, and named the gate */
    CBCTL_OP_LOST      /* retries exhausted; the link is suspect */
} cbctl_op_state_t;

typedef enum {
    CBCTL_AUTH_DENY = 0,
    CBCTL_AUTH_PENDING,  /* still deciding — keeps the session alive; NOT the same as DENY */
    CBCTL_AUTH_GRANTED
} cbctl_auth_t;

/* UNKNOWN is zero on purpose: a zeroed measurement means "I have not measured", which is the
 * safe default. Mapping "not measured yet" onto VALID is an accidental claim that an
 * insulation test passed. */
typedef enum {
    CBCTL_ISO_UNKNOWN = 0,
    CBCTL_ISO_INVALID,
    CBCTL_ISO_VALID,
    CBCTL_ISO_WARNING,
    CBCTL_ISO_FAULT
} cbctl_isolation_t;

typedef struct {
    cbctl_mv_t max_voltage, min_voltage;
    cbctl_ma_t max_current, min_current;
    cbctl_w_t  max_power;
    bool       power_ready;
} cbctl_envelope_t;

typedef struct {
    cbctl_mv_t        present_voltage;
    cbctl_ma_t        present_current;
    cbctl_isolation_t isolation;
} cbctl_measurement_t;

/* ---- observed state and events --------------------------------------------------------- */

/* LIVE FLAGS AND CUMULATIVE COUNTERS ARE SEPARATE VOCABULARIES, and that is deliberate.
 * Reading a cumulative count as a live flag is a mistake this project made four times with
 * one field (`secc_sess`, a total, read repeatedly as "a session is open"). Two disjoint
 * enums and two different accessors make it inexpressible. */
typedef enum {
    CBCTL_FLAG_VEHICLE_PRESENT = 0, /* pilot has left state A */
    CBCTL_FLAG_SLAC_MATCHED,
    CBCTL_FLAG_SESSION_ACTIVE,      /* a V2G session is open RIGHT NOW */
    CBCTL_FLAG_STOP_LATCHED,
    CBCTL_FLAG_VEHICLE_MAC_KNOWN,
    CBCTL_FLAG_SAFETY_ASSERTED,   /* the board's external safety chain is asserted */
    CBCTL_FLAG_ENERGISED,
    CBCTL_FLAG_EV_READY,          /* the EV says it is ready for energy      */
    CBCTL_FLAG_EV_PAUSED,         /* the EV has asked to pause               */
    CBCTL_FLAG_EV_SOC_KNOWN,      /* an EVRESSSOC has actually been reported */         /* the board reports the gun contactor closed   */
    CBCTL_FLAG__COUNT
} cbctl_flag_t;

typedef enum {
    CBCTL_CTR_SESSIONS = 0,         /* TOTAL sessions ever — never "is one open" */
    CBCTL_CTR_RELAY_SAFETY_OPENS,   /* a precondition went away; can legitimately be non-zero */
    CBCTL_CTR_RELAY_LEASE_OPENS,    /* the dead-man LAPSED — audible relay flake; want zero   */
    CBCTL_CTR_GUN_OPEN_TIMEOUTS,    /* contactor broke current instead of waiting for decay   */
    CBCTL_CTR_UPTIME_MS,
    /* STATUS lines the board reported as OVERFLOWING its payload cap. Non-zero means the
     * diagnostic line no longer fits and the tail of it is gone. */
    CBCTL_CTR_STATUS_OVERFLOWS,
    /* STATUS replies WE received truncated (the sentinel key was missing). Distinct from the
     * above: the board can be fine while the wire lost the tail. */
    CBCTL_CTR_STATUS_TRUNCATED,
    /* THE EV'S OWN STATE OF CHARGE, percent, straight from EVRESSSOC in every DC request.
     *
     * cbplc publishes this (`ev_soc` in STATUS); plc_firmware never extracted it, so nothing
     * built on that firmware could display, log or bill against SoC. It is a counter rather
     * than a gauge only because the accessor split is by vocabulary, not by physics — read it
     * with cbctl_ev_soc_pct(), which tells you whether it is known at all. */
    CBCTL_CTR_EV_SOC_PCT,
    CBCTL_CTR__COUNT
} cbctl_counter_t;

typedef enum {
    CBCTL_EV_NONE = 0,
    CBCTL_EV_VEHICLE_PLUGGED,
    CBCTL_EV_VEHICLE_UNPLUGGED,
    CBCTL_EV_SLAC_MATCHED,
    CBCTL_EV_VEHICLE_MAC_KNOWN,     /* THE AUTOCHARGE TRIGGER — see cbctl_vehicle_mac() */
    CBCTL_EV_SESSION_STARTED,
    CBCTL_EV_SESSION_ENDED,
    CBCTL_EV_BOARD_RESET,
    CBCTL_EV_LINK_QUARANTINED,
    /* We commanded a relay closed, the board accepted it, and its own safety layer opened it
     * again — repeatedly. cbctl has abandoned the close rather than keep re-commanding it. */
    CBCTL_EV_RELAY_OVERRIDDEN
} cbctl_event_kind_t;

typedef struct {
    cbctl_event_kind_t kind;
    uint32_t           at_ms;
} cbctl_event_t;

/* Who this board IS. Filled from the 20-byte IDENTITY reply.
 *
 * The MAC is the identity that cannot be misconfigured — unlike a stored id, which is exactly
 * what has gone wrong on this bench before. The build fingerprint hashes __DATE__/__TIME__
 * rather than a git SHA, because this firmware is flashed dirty by design and a commit id
 * would be a confident lie about what is running. */
typedef struct {
    uint8_t  mac[6];
    uint8_t  wire_version;
    uint8_t  reset_reason;
    uint32_t up_ms;      /* at the moment it answered */
    char     build[9];   /* 8 chars + NUL */
    bool     valid;
} cbctl_identity_t;

/* ---- the link ------------------------------------------------------------------------- */

/* Caller-provided storage. No allocation anywhere in cbctl: an embedded controller must see
 * its whole memory cost at compile time, and a charger is not a place to meet an allocator
 * failure. The fields are public only so the caller can place one; treat them as private. */
/* Set by cbctl_open(). Nothing else writes it, so a struct that was never opened — or was
 * memset by a caller "to be safe" — is detectable instead of being run as a state machine
 * over uninitialised memory. */
#define CBCTL_LINK_MAGIC 0x43424354u  /* "CBCT" */

typedef struct {
    uint32_t           magic;
    cbplc_session_t    session;
    cbctl_link_state_t state;
    cbctl_identity_t   identity;

    /* Declared intent. The tick's job is to make the board match this. */
    cbctl_envelope_t    envelope;
    cbctl_measurement_t measurement;
    cbctl_auth_t        auth;
    uint32_t            auth_lease_ms;
    bool                stop_requested;
    bool                stop_dirty;
    uint8_t             slac_pending;  /* 0 none, 1 arm+1, 2 replug+1 */   /* the latch state has not yet reached the board */
    bool                have_envelope, have_measurement;
    /* DIRTY FLAGS, not zeroed timestamps.
     *
     * These used to be expressed by setting last_limits/last_feedback back to 0 to mean "due
     * now". On real hardware that was two bugs at once: the staleness accounting read a zero
     * stamp as an age of `now` (6.4 MILLION late-feed ticks reported while the feed was in
     * fact perfect), and because a caller may legitimately re-declare every pass, LIMITS was
     * PERMANENTLY due — which starved every lower-priority message, including the relay lease
     * renewal, and the board opened the contactor on a lapsed dead-man. */
    bool                envelope_dirty, measurement_dirty, auth_dirty;

    /* When each declaration was last put on the wire, so the tick knows what is due. */
    cbctl_instant_t last_limits, last_feedback, last_heartbeat, last_auth;

    /* One outstanding op at a time, because the session underneath is stop-and-wait. */
    cbctl_op_t       op;
    cbctl_op_state_t op_state;
    uint8_t          op_gate;
    /* True only while the op's OWN request is on the wire. Without it, the completion of an
     * unrelated in-flight message (a heartbeat, an auth re-assertion) was attributed to the
     * caller's op and reported it LOST. */
    bool             op_on_wire;

    /* Relay lease: what we are holding closed, and on what dead-man. */
    uint8_t  relay_enable, relay_state;
    uint32_t relay_hold_ms;
    cbctl_instant_t last_relay;
    bool     relay_dirty;   /* a change the tick has not yet put on the wire */

    /* SEND ACCOUNTING. Without these a message that never reaches the wire is invisible:
     * the board starves, the controller reports a healthy link, and nothing names the gap.
     * Learned the hard way — cbplc refused energy with limits_stale while cbctl believed its
     * feed was perfect, because nothing counted what was actually sent. */
    uint32_t limits_sent, feedback_sent, auth_sent, status_sent;
    uint32_t issue_blocked_busy;   /* issue_one had work but the wire was occupied */
    uint32_t relay_renewals; /* renewal requests SENT */
    uint32_t relay_acks;     /* renewals the board CONFIRMED — the number that matters */
    uint32_t relay_refusals; /* renewals the board refused (a gate held) */
    uint32_t relay_refusal_streak; /* consecutive refusals; retry-at-once is capped by this */
    bool     relay_ack_pending;
    uint32_t relay_safety_base;   /* safety-open count when the close intent was established */
    bool     relay_base_valid;
    uint32_t ev_target_mv, ev_target_ma;
    bool     ev_target_valid;
    uint32_t lease_released_no_session; /* leases dropped because the session ended */
    /* Which relays the BOARD reports closed, not what we asked for. Exposed because a charge
     * that delivers no current with a closed gun is usually a DIFFERENT contactor still open —
     * the power path (RELAY_POWER) is a separate branch the controller closes on its own, and
     * commanding only the gun leaves module current with nowhere to go. */
    uint32_t ev_target_ms;   /* when the board last ANSWERED a DC request — the liveness stamp */
    uint32_t dc_answers;     /* cbplc's dc_ans: rises only while the vehicle is asking */
    uint8_t  relay_closed_mask;

    /* Raw STATUS observer. NULL by default; set it and every status payload the board sends is
     * handed over verbatim before any parsing.
     *
     * WHY THIS EXISTS: cbctl deliberately parses a SUBSET of the board's status line, so a field
     * that matters to a diagnosis but not to the library is invisible — and "the controller
     * cannot see what the board actually said" turns a five-minute answer into a wrong one.
     * The callback fires even on a payload cbctl REJECTS as truncated, because a truncated line
     * is precisely when you most want to look at it. */
    void (*status_observer)(const char *line, size_t len, void *ud);
    void *status_observer_ud;
    uint32_t relay_overridden;    /* times we gave up on a close the board kept undoing */
    uint32_t relay_danger_preempts; /* renewals that jumped the feed to save a lease */
    uint32_t lease_min_headroom_ms; /* closest the lease ever came to lapsing */
    cbctl_instant_t last_tick;
    uint32_t worst_tick_gap_ms;   /* longest the CALLER left between ticks */

    uint32_t next_op_id;
    uint32_t consecutive_losses;
    cbctl_instant_t quarantined_until;
    /* PERMANENT quarantine is a POLICY STATE, NOT A DEADLINE.
     *
     * The obvious encoding — quarantined_until = 0xFFFFFFFF — is wrong, and wrong in the
     * quietest way. Deadline comparison here is wrap-safe by necessity, so "the future" is
     * only ever the next half of the clock: cbctl_reached(now, 0xFFFFFFFF) is TRUE for most
     * values of now, and a containment meant to last forever released on the very next tick.
     * There is no infinitely-distant instant in modular arithmetic, so do not try to name one. */
    bool     quarantine_permanent;
    uint8_t  quarantine_count;  /* successive quarantines, for the backoff */

    /* Observability. These exist because "it stopped working" is not a diagnosis. */
    /* Observed state, parsed from STATUS. */
    bool     flags[CBCTL_FLAG__COUNT];
    bool     have_flags;
    uint32_t counters[CBCTL_CTR__COUNT];
    uint8_t  ev_mac[6];
    /* TWO stamps, deliberately. `last_status_req` is when we ASKED and drives the request
     * cadence; `last_status` is when a reply was PARSED and answers "how current is what we
     * believe". Driving the cadence from the reply made each round trip's latency compound
     * into the period — measured at ~2400 ms for a 1000 ms setting — while stamping only on
     * send would let a board that never answers look like one being observed. */
    cbctl_instant_t last_status_req;
    cbctl_instant_t last_status;

    /* A tiny event ring. Bounded like everything else; overflow drops the OLDEST, because
     * the newest state is the one a controller must act on. */
    cbctl_event_t ev[16];
    uint8_t ev_head, ev_tail;
    uint32_t ev_dropped;

    uint32_t feed_late;      /* TIMES the feed went past CBPLC_LIMITS_STALE_MS (edges, not ticks) */
    bool     feed_is_late;   /* is it late right now */
    bool     reidentify_pending;
    uint32_t reestablishes;    /* times once-only state was re-asserted after a board reset */
    uint32_t identity_changed; /* the board on this port became a DIFFERENT board */
    uint32_t transport_failures; /* ticks where the transport itself reported a hard failure */
    uint32_t observation_stalls; /* times the snapshot aged past the bound */
    uint32_t status_promotions; /* times STATUS had to jump the queue to avoid starvation */
    uint32_t polls_capped;   /* ticks that hit CBCTL_POLL_PER_TICK with work still queued   */
} cbctl_link_t;

/* Open a link over `tx`. Does not block and does not wait for the board: the link starts in
 * IDENTIFYING and reports its own progress. Start ticking immediately. */
cbctl_status_t cbctl_open(cbctl_link_t *link, const cbplc_transport_t *tx);

/* The one call you must never skip. Bounded work, no waiting. Call it at >= 5 Hz.
 * `now_ms` must be MONOTONIC — not wall clock, not anything NTP can step. */
cbctl_status_t cbctl_tick(cbctl_link_t *link, uint32_t now_ms);

cbctl_link_state_t cbctl_link_state(const cbctl_link_t *link);

/* Which board is this? False until it has identified.
 *
 * READ THIS AND PIN IT. On a multi-gun charger the port a board appears on is NOT stable —
 * USB enumeration order can differ across reboots, so /dev/ttyACM0 is not reliably gun 1. A
 * controller that trusts port order will one day authorise one gun and close the OTHER gun's
 * contactor, and nothing in the protocol will object. Bind your gun numbers to MACs. */
bool cbctl_identity(const cbctl_link_t *link, cbctl_identity_t *out);

/* Refuse to proceed unless this link is the board you expect. Returns CBCTL_E_WRONG_BOARD if
 * it has identified as something else — which is the moment to stop, not to carry on. */
cbctl_status_t cbctl_expect_mac(const cbctl_link_t *link, const uint8_t mac[6]);

/* ---- declarations (all return immediately) --------------------------------------------- */

cbctl_status_t cbctl_set_envelope(cbctl_link_t *link, const cbctl_envelope_t *env);
cbctl_status_t cbctl_set_measurement(cbctl_link_t *link, const cbctl_measurement_t *m);
cbctl_status_t cbctl_authorize(cbctl_link_t *link, cbctl_auth_t decision, uint32_t lease_ms);

/* Raise the stop AND clear power_ready, together, because they are one decision.
 *
 * Not a convenience. An envelope that advertises ready while the controller asks to stop
 * tells the vehicle two opposite things and it resolves them by resetting TCP; and a stop
 * that leaves demand commanded makes the PLC break the contactor under load on the decay
 * budget, which is how contacts weld on hardware with no weld sensor. */
cbctl_status_t cbctl_begin_stop(cbctl_link_t *link);

/* Clear the latched stop.
 *
 * THE PLC LATCHES A STOP UNTIL A ZERO BYTE CLEARS IT — deliberately, because a stop that
 * evaporates when the next tick fails to repeat it is not a stop. The consequence is that a
 * stop raised by a previous controller, INCLUDING A PREVIOUS RUN OF YOURS, is still in force
 * at startup and will refuse every session until cleared.
 *
 * cbctl_open() does NOT clear it for you: silently releasing a safety latch on connect is
 * exactly the wrong default. You have to ask.
 *
 * Clearing the stop does not re-arm power delivery. power_ready stays false until you set an
 * envelope again, so releasing the latch cannot by itself energise anything. */
cbctl_status_t cbctl_stop_clear(cbctl_link_t *link);

/* ---- SLAC control ------------------------------------------------------------------------ */

/* Start SLAC matching now. Refused by the PLC if a match already exists. */
cbctl_status_t cbctl_slac_arm(cbctl_link_t *link, cbctl_op_t *out_op);

/* Drive the pilot to E for a dwell, then release, so a vehicle that is ALREADY PLUGGED IN
 * re-initiates SLAC. No physical replug needed.
 *
 * A controller genuinely needs this, and cbctl was missing it: after a firmware restart the
 * vehicle is sitting there matched to a stack that no longer exists, and nothing but a pilot
 * transition will make it try again. It is also how any bench retries matching without
 * touching the cable — cbplc's own Python harness opens every cycle with it.
 *
 * It deliberately does NOT arm anything itself. Driving E makes CP leave B/C/D, which retires
 * the SLAC and charging sessions through the paths that already exist. */
cbctl_status_t cbctl_slac_replug(cbctl_link_t *link, cbctl_op_t *out_op);

/* Close relays as a LEASE. `hold_ms` is the dead-man: the PLC opens them ITSELF if the lease
 * is not renewed, which is what makes a crashed controller safe rather than merely detectable.
 *
 * You do not renew it. cbctl_tick() does, at hold/3 — three chances before it could lapse.
 * That ratio is not arbitrary: on this project's own bench a 5 s hold was renewed by hand
 * every 2.0 s, a 2.5x margin, and one slow pass around a vehicle phase change let the lease
 * lapse, dropped the contactor mid-charge and re-closed it on the next renewal. That is what
 * an audible relay "flake" is. A caller cannot make that mistake here because a caller does
 * not set the renewal period.
 *
 * hold_ms is REQUIRED and must be non-zero for the energy relays: a close with no dead-man is
 * the one relay mistake this hardware cannot absorb. */
cbctl_status_t cbctl_relay_close(cbctl_link_t *link, uint8_t mask, uint32_t hold_ms,
                                 cbctl_op_t *out_op);

/* Drop the lease and open now. Always legal — opening is never the dangerous direction. */
cbctl_status_t cbctl_relay_open(cbctl_link_t *link, uint8_t mask, cbctl_op_t *out_op);

cbctl_op_state_t cbctl_op_state(const cbctl_link_t *link, cbctl_op_t op);

/* The gate the PLC named when it refused an op. Zero if it did not refuse. */
uint8_t cbctl_op_gate(const cbctl_link_t *link, cbctl_op_t op);

/* How long the caller may sleep before cbctl needs servicing. 0 = now.
 * MANDATORY on every port — unlike a pollable fd, a deadline is meaningful on bare metal. */
uint32_t cbctl_next_deadline_ms(const cbctl_link_t *link, uint32_t now_ms);

bool     cbctl_flag(const cbctl_link_t *link, cbctl_flag_t f);
uint32_t cbctl_counter(const cbctl_link_t *link, cbctl_counter_t c);

/* The EV's HomePlug MAC, known at SLAC MATCH — before any V2G session exists and therefore
 * long before an EVCCID. That timing is the whole point for Autocharge: a controller
 * recognising a returning vehicle needs it BEFORE it has to decide whether to authorise.
 *
 * Returns false until it is known. Deliberately NOT the same call or the same type as an
 * EVCCID: a 6-byte MAC and an 8-byte contract identifier are different identities, and
 * conflating them is how a car gets billed to the wrong account. */
bool cbctl_vehicle_mac(const cbctl_link_t *link, uint8_t out[6]);

/* The EV's state of charge in percent, or -1 when the vehicle has not reported one.
 *
 * -1 rather than 0, because 0 %% is a real and alarming reading and must never be what
 * "unknown" looks like — a charge controller that stops at a target SoC would otherwise treat
 * a silent vehicle as an empty one. */
int cbctl_ev_soc_pct(const cbctl_link_t *link);

/* What the EV is ASKING FOR right now, in mV and mA, as cbplc reports it (`ev_tgt_mv` /
 * `ev_tgt_ma`). False until the vehicle has requested something.
 *
 * A controller needs this from PRECHARGE onward, not just during delivery: the EV will not
 * close its own contactor until the bus is already near its requested voltage, so a charger
 * that waits to be told "energised" before producing volts deadlocks itself and the session
 * times out with nothing on the wire. */
bool cbctl_ev_target(const cbctl_link_t *link, uint32_t *mv, uint32_t *ma);

/* Pull the next synthesised event; false when the queue is empty. Non-blocking, no callback,
 * never called from an interrupt — you always pull, the library never calls you back.
 *
 * Events are DERIVED by diffing STATUS snapshots. There is no push channel: CBPLC_T_EVENT
 * exists in the protocol's type table and the firmware never emits one, so a library that
 * offered a callback for it would have callers waiting for something that never arrives. */
bool cbctl_next_event(cbctl_link_t *link, cbctl_event_t *out);


/* ---- driver-facing presentation (LED / display) ----------------------------------------- */

/* What the connector should be SHOWING, derived from the same snapshot everything else uses.
 *
 * WHY THIS LIVES IN THE LIBRARY. This project has a recorded failure class of exactly one
 * shape: the OCPP state, the session state, the charge state and the LED preset are four
 * parallel state machines, they are updated in four places, and they drift. A driver then
 * stands in front of a connector whose light disagrees with what the charger is doing.
 * Deriving the preset HERE, from one observation, in one function, makes that drift
 * impossible rather than merely discouraged.
 *
 * The names match the vocabulary the rest of this codebase already uses, so the value drops
 * straight into an existing LED command without a translation table nobody maintains.
 *
 * cbctl does NOT drive pixels: the cbplc wire protocol has no LED message. This tells you what
 * to show; your display, LED driver or existing LED command shows it. */
typedef enum {
    CBCTL_LED_BOOTING = 0, /* not identified yet — we do not know what this connector is    */
    CBCTL_LED_AVAILABLE,   /* healthy, nothing plugged                                      */
    CBCTL_LED_PREPARING,   /* plugged; matching, negotiating, or waiting for authorisation   */
    CBCTL_LED_CHARGING,    /* energy is actually being delivered                             */
    CBCTL_LED_FINISHING,   /* winding down — stop raised, or the session is closing          */
    CBCTL_LED_FAULTED,     /* this connector is not usable right now                         */
    CBCTL_LED_EMERGENCY    /* the safety chain is asserted                                   */
} cbctl_led_t;

/* Derive the preset. Never returns CHARGING on a stale observation — see the note in led.c:
 * showing a driver "charging" from a picture that may be a minute old is the one error here
 * with a real cost. */
cbctl_led_t cbctl_led_preset(const cbctl_link_t *link, uint32_t now_ms);

/* Lower-case name, matching the existing LED command vocabulary ("booting", "available",
 * "preparing", "charging", "finishing", "faulted", "emergency"). Never NULL. */
const char *cbctl_led_name(cbctl_led_t p);

/* ---- the fleet: N PLCs, one loop, no interference ------------------------------------- */

/* The fleet is deliberately NOT an abstraction that owns the links' behaviour. It is a loop
 * over independent state machines plus a rotation counter. There is no shared buffer, no
 * shared request-id space and no shared clock stamp, so isolation is STRUCTURAL rather than
 * a property maintained by careful coding — the kind that decays.
 *
 * Note what isolation actually rests on. Non-blocking I/O is necessary but NOT sufficient:
 * the claim "non-blocking state machines mean a stalled peer cannot block its siblings" was
 * put to independent verifiers against libcurl's own documentation and refuted. What delivers
 * it here is three things designed in on purpose — per-link deadlines, a bounded quantum per
 * link per tick (CBCTL_POLL_PER_TICK), and explicit quarantine. */
typedef struct {
    cbctl_link_t *links[CBCTL_FLEET_MAX];
    unsigned      n;
    unsigned      rr;   /* rotation start, so link 0 is not always serviced first */
    uint32_t      duplicate_boards; /* links found to be the SAME physical board */
} cbctl_fleet_t;

void cbctl_fleet_init(cbctl_fleet_t *fleet);

/* Add a link the caller owns. Returns CBCTL_E_ARG when full or already present. */
cbctl_status_t cbctl_fleet_add(cbctl_fleet_t *fleet, cbctl_link_t *link);

/* Service EVERY link, bounded. One wedged board cannot delay the others: there is no call
 * here that waits, each link gets at most CBCTL_POLL_PER_TICK polls, and a link that stops
 * answering is quarantined rather than retried forever. */
cbctl_status_t cbctl_fleet_tick(cbctl_fleet_t *fleet, uint32_t now_ms);

/* The NEAREST deadline across all links — one value, not N timers, which is the shape a host
 * event loop can actually use. */
uint32_t cbctl_fleet_next_deadline_ms(const cbctl_fleet_t *fleet, uint32_t now_ms);

/* How stale is this link's feed right now? The fleet's isolation guarantee is stated in terms
 * of this: wedge one link and every other link's value must stay under CBPLC_LIMITS_STALE_MS. */
/* Returned by cbctl_feed_age_ms() when the envelope has been declared but NEVER put on the
 * wire. Distinct from 0, which means "sent just now" — conflating them hid a live fault where
 * no LIMITS was being sent at all and the age read as perfectly fresh. */
#define CBCTL_FEED_NEVER_SENT 0xFFFFFFFFu
uint32_t cbctl_feed_age_ms(const cbctl_link_t *link, uint32_t now_ms);

/* How old the OBSERVED state is — the age of the last complete STATUS.
 *
 * Read this before acting on cbctl_flag() or cbctl_vehicle_mac(). Those return last known
 * good, which is the right default (silently flipping them to false on a lost poll would
 * manufacture a phantom "vehicle unplugged"), but last-known-good is only meaningful next to
 * its age. Returns CBCTL_NEVER_OBSERVED if no complete snapshot has ever arrived. */
#define CBCTL_NEVER_OBSERVED 0xFFFFFFFFu
uint32_t cbctl_observation_age_ms(const cbctl_link_t *link, uint32_t now_ms);

/* How long until the relay lease would lapse if no further renewal were confirmed.
 *
 * This is the number to watch on a loaded controller. It should never approach zero; if it
 * trends downward your loop is being starved and a contactor will eventually open for no
 * reason other than scheduling. `link->lease_min_headroom_ms` records the worst it ever got,
 * and `link->worst_tick_gap_ms` records the longest the caller went between ticks — together
 * they say whether the margin is comfortable or whether you got lucky.
 *
 * Returns 0 when no lease is held. */
uint32_t cbctl_lease_headroom_ms(const cbctl_link_t *link, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif /* CBCTL_H */
