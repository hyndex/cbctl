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
#include "cbplc/provision.h"
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
/* Longest one unanswered request can block every other message.
 *
 * MEASURED FROM THE SESSION, NOT ASSUMED. The obvious reading of "2 retries at 250 ms plus
 * 60 ms of backoff" gives 620, and it is wrong twice:
 *
 *   1. RETRY_LIMIT is a retry count, not a transmission count. cbplc retries while
 *      `attempts <= retry_limit` with attempts starting at 1 (session.c), so a limit of 2 sends
 *      the request THREE times and therefore waits out THREE timeouts.
 *   2. The backoff is never applied. restage_for_retry() sets pending_since = now_ms with no
 *      backoff term, and `retry_backoff_ms` is written (here and in session.c's defaults) but
 *      read nowhere in either tree. Adding it to the budget counts time that is never spent.
 *
 * The two errors point in opposite directions and very nearly cancelled, which is why 620
 * survived: the real figure is 750. Everything derived from this constant was sized against a
 * budget 130 ms short of the truth. */
#define CBCTL_WORST_BLOCK_MS \
    ((CBCTL_RETRY_LIMIT + 1u) * CBCTL_REQUEST_TIMEOUT_MS)
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
/* THE LAST KEY IN cbplc's STATUS FORMAT STRING, and therefore the proof a line arrived whole.
 *
 * Must be kept in step with cbplc's build_status() and with tools/probe.py's TAIL_KEY. A
 * sentinel that is no longer last does not fail loudly — it simply stops detecting truncation,
 * and the keys that fell off read as absent while every flag derived from them reads false. */
#ifndef CBCTL_STATUS_TAIL_KEY
#define CBCTL_STATUS_TAIL_KEY "prov_timeout"
#endif

#ifndef CBCTL_OBSERVATION_STALE_MS
#define CBCTL_OBSERVATION_STALE_MS 4000u
/* An EV target older than this is not a demand any more. Freshness is the honest test: cbplc
 * keeps echoing the last vehicle's request between sessions, and its `secc_active` flag is NOT a
 * reliable "a session is live" signal — it reads 0 while DC requests are still flowing. Gating on
 * that flag blocked the module command entirely and the DCDC never started. */
#define CBCTL_EV_TARGET_STALE_MS 6000u
#endif

/* The floor auth_period() clamps the AUTHORIZE re-assert period to. Named here because
 * CBCTL_AUTH_LEASE_MIN_SAFE_MS is derived from it, and a floor that drifts away from the
 * constant derived from it is how the old derivation silently stopped being true. */
#ifndef CBCTL_AUTH_PERIOD_FLOOR_MS
#define CBCTL_AUTH_PERIOD_FLOOR_MS 500u
#endif

/* Shortest AUTHORIZE lease that cannot be lapsed by one unanswered request, derived from the
 * same blocking budget as the relay hold. Refused rather than raised: unlike a relay hold —
 * where a longer dead-man is merely less safe and silently raising it keeps the contactor
 * governed — an authorisation lease is a policy decision the caller owns, and quietly
 * lengthening it would grant more permission than was asked for. */
/* Derived from the RE-ASSERT PERIOD ACTUALLY USED, which is not lease/3 for a short lease.
 *
 * The old form assumed AUTHORIZE re-asserts at lease/3, making (3/2)x the block budget the
 * requirement. But auth_period() floors the period at CBCTL_AUTH_PERIOD_FLOOR_MS, so every
 * lease below 3x the floor re-asserts at the floor instead — and the constant was accepting
 * leases in exactly that range. Worked through at the old value: a 1130 ms lease re-asserts at
 * 500 ms, one blocked request adds the worst block, and the gap reaches 500 + 750 = 1250 ms
 * against an 1130 ms lease. The board demotes GRANTED to PENDING on expiry and a live charge
 * ends with auth_withdrawn — from a lease the library had certified as safe.
 *
 * State the real requirement instead of a ratio that only holds above the floor: one re-assert
 * period, plus one worst-case block, plus margin. */
#define CBCTL_AUTH_LEASE_MIN_SAFE_MS \
    (CBCTL_AUTH_PERIOD_FLOOR_MS + CBCTL_WORST_BLOCK_MS + 200u)

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

/* DIAG drain cadence. The floor is how fast a backlog is cleared; the ceiling is what a quiet
 * board costs. Geometric backoff between them means an idle link settles at one round trip
 * every eight seconds while a board with a session's worth of trace to hand over drains at the
 * floor until it is empty. */
#ifndef CBCTL_DIAG_PERIOD_MIN_MS
#define CBCTL_DIAG_PERIOD_MIN_MS 250u
#endif
#ifndef CBCTL_DIAG_PERIOD_MAX_MS
#define CBCTL_DIAG_PERIOD_MAX_MS 8000u
#endif
/* cbplc caps one drain reply at 1024 bytes, so a reply THIS big proves there is more waiting —
 * the only backlog signal available from a firmware too old to report `pending`. */
#define CBCTL_DIAG_CHUNK_FULL 1024u
/* How many times one control intent may be re-asserted before cbctl stops spending round trips
 * on it. Three covers a lost frame or two; beyond that the board is answering something we did
 * not ask for and repeating will not change its mind. */
/* How many times one provisioning declaration is re-sent before cbctl stops. Three covers a
 * lost frame or two; beyond that the board is refusing a value and repeating cannot change its
 * mind. The intent stays in link->prov either way, and a board reset re-arms it. */
#ifndef CBCTL_PROV_TRIES
#define CBCTL_PROV_TRIES 3u
#endif

#ifndef CBCTL_DIAG_CTL_TRIES
#define CBCTL_DIAG_CTL_TRIES 3u
#endif
/* How often the board's ring state is re-read while diagnostics are armed.
 *
 * `dropped` and `pending` only ever arrive in a control reply, so without a refresh they are
 * frozen at whatever they were when the last setter was answered. Observed on hardware: after
 * draining 520 bytes the tool still reported "pending=520", because nothing had asked again —
 * a stale number presented as a live one, which is the exact failure mode this library treats
 * as a defect everywhere else. Three seconds is far cheaper than the drain it accompanies, and
 * it is suppressed during a charge like every other diagnostic request. */
#ifndef CBCTL_DIAG_STATE_PERIOD_MS
#define CBCTL_DIAG_STATE_PERIOD_MS 3000u
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

/* Minimum level the BOARD should record. Mirrors cbplc's wire values; OFF records nothing. */
typedef enum {
    CBCTL_DIAG_DEBUG = 0,
    CBCTL_DIAG_INFO = 1,
    CBCTL_DIAG_WARN = 2,
    CBCTL_DIAG_ERROR = 3,
    CBCTL_DIAG_OFF = 4,
} cbctl_diag_level_t;

/* Category bits, mirroring cbplc's. V2G is the per-message decoded HLC trace and is the reason
 * categories exist at all: it is INFO-level and enormous, so it must be separable from the
 * INFO lines that are rare and always wanted. */
#define CBCTL_DIAG_CAT_CORE  0x00000001u
#define CBCTL_DIAG_CAT_V2G   0x00000002u
#define CBCTL_DIAG_CAT_SLAC  0x00000004u
#define CBCTL_DIAG_CAT_CP    0x00000008u
#define CBCTL_DIAG_CAT_RELAY 0x00000010u

/* Where drained text goes. Called from inside cbctl_tick(), with the bytes exactly as the board
 * produced them — never NUL-terminated, because the ring holds a byte stream and a chunk
 * boundary can fall mid-line. A sink that wants whole lines must buffer; cbctl deliberately does
 * not, since a controller writing to a file wants the bytes and one that parses wants to choose
 * its own framing.
 *
 * MUST NOT BLOCK. It runs on the control loop, and this project has already lost a fleet's
 * PreCharge to a synchronous read on that thread. Append to memory or write to an fd opened
 * non-blocking; do not fsync, and do not call back into cbctl. */
typedef void (*cbctl_diag_sink_fn)(const uint8_t *bytes, uint16_t len, void *user);

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
    /* Which cbplc message type the outstanding op is waiting for. Without it, any branch that
     * transmitted while an op was pending could claim the wire for it — and a RELAY ack then
     * resolved a queued RESTART as applied by a board that never saw one. */
    uint8_t          op_type;
    /* True only while the op's OWN request is on the wire. Without it, the completion of an
     * unrelated in-flight message (a heartbeat, an auth re-assertion) was attributed to the
     * caller's op and reported it LOST. */
    bool             op_on_wire;

    /* Relay lease: what we are holding closed, and on what dead-man. */
    uint8_t  relay_enable, relay_state;
    uint32_t relay_hold_ms;
    cbctl_instant_t last_relay;
    bool     relay_dirty;   /* a change the tick has not yet put on the wire */

    /* Per-charger identity and per-board calibration. Inert until cbctl_provision_*() is
     * called: an un-declaring controller puts nothing extra on the wire and the board keeps the
     * compiled defaults it always had. */
    cbplc_provision_t prov;
    bool     prov_dirty;        /* a declaration the tick has not yet put on the wire */
    bool     prov_supported;    /* the board ACKed a declaration rather than CBPLC_E_BAD_TYPE  */
    bool     prov_probed;       /* we have asked at least once, so "false" above is a verdict  */
    bool     prov_inflight;
    uint8_t  prov_tries;
    uint32_t prov_accepted, prov_refused;
    cbctl_op_t restart_op;
    bool     restart_pending, restart_inflight;

    /* Board diagnostics. Everything here is inert until cbctl_diag_enable(). */
    cbctl_diag_sink_fn diag_sink;
    void              *diag_user;
    bool     diag_on;
    bool     diag_ctl_supported;  /* the board has answered a control request as a control reply */
    bool     diag_ctl_probed;     /* we have asked at least once, so "false" above is a verdict */
    bool     diag_state_valid;    /* the board has reported its ring state at least once */
    uint8_t  diag_want_level;     /* what we intend the board to record */
    uint32_t diag_want_cats;
    /* DIRTY UNTIL THE BOARD CONFIRMS IT, not until we transmit it. A setter whose value the
     * board clamped or ignored must keep being re-asserted, and the only evidence of what is
     * actually in force is the state the board sends back. Same rule the envelope follows. */
    bool     diag_level_dirty, diag_cats_dirty, diag_clear_dirty;
    /* Arming sends one GET_STATE. Without it a controller that only wants to READ the log
     * would never learn whether control is supported, nor the board's drop count — and the
     * drop count is what separates "nothing was logged" from "we lost it". */
    bool     diag_probe_dirty;
    /* Bounded re-assertion. If a board keeps reporting something other than what was asked —
     * a firmware that clamps differently, a value we cannot express — re-sending forever would
     * spend a round trip per attempt for the life of the controller. Give up, leave the intent
     * visible in diag_want_*, and let a board reset re-arm it. */
    uint8_t  diag_ctl_tries;
    uint8_t  diag_board_level;    /* what the board last SAID it is recording */
    uint32_t diag_board_cats;
    uint32_t diag_dropped;        /* board's own count of bytes it overwrote — silence vs loss */
    uint32_t diag_pending;
    cbctl_instant_t last_diag;
    cbctl_instant_t last_diag_state;  /* when the board last reported its ring */
    /* ADAPTIVE BACKOFF, so an idle board costs almost nothing and a backlog clears fast. An
     * empty drain doubles the interval up to a ceiling; a non-empty one collapses it to the
     * floor. Without this, polling a quiet board at the fast rate would spend a round trip a
     * second on "nothing new" for the entire life of the controller. */
    uint32_t diag_period_ms;
    /* Which kind of reply the outstanding DIAG request expects. A drain reply is bytes; a
     * control reply is a state struct; and on an old firmware a control REQUEST comes back as
     * bytes — which is exactly how support is detected. */
    bool     diag_req_was_control;
    bool     diag_req_pending;
    uint32_t diag_drains;         /* observability: what actually reached the wire */
    uint32_t diag_bytes;
    uint32_t diag_suppressed;     /* drains skipped because energy was live */

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
    uint32_t auth_danger_preempts; /* AUTHORIZE re-asserts that jumped the feed to save a lease */
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
 * You do not renew it. cbctl_tick() does, at hold/CBCTL_RELAY_RENEW_DIVISOR (hold/6) — five
 * chances to lose a renewal before it could lapse.
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

/* ---- board diagnostics: the log ring --------------------------------------------------------
 *
 * cbplc keeps every line of its diagnostic text in a RAM ring rather than printing it, because
 * the control wire carries framed binary and one text line mid-stream reads as a garbage length
 * prefix. CBPLC_T_DIAG drains that ring, and (on firmware that supports the subcommands) sets
 * what goes into it — including the per-message decoded V2G trace, which is the thing worth
 * having when a particular vehicle refuses a particular charger.
 *
 * DIAGNOSTICS ARE THE ONE MESSAGE THAT IS ALLOWED TO STARVE, and that is a deliberate inversion.
 *
 *   STATUS earned its aging and its queue-jump because a controller that goes blind mid-charge
 *   is dangerous — measured: limits=60, feedback=60, heartbeat=33, relay=20, STATUS=1 across
 *   24 s, until it was given the right to age past CBCTL_STATUS_STARVE_MS. The note on that fix
 *   says not to add another low-priority message without the same treatment.
 *
 *   DIAG is the case where that warning does not apply, and applying it would be actively wrong.
 *   cbplc is stop-and-wait: one outstanding request, and a request occupies the wire for a full
 *   round trip. Spending that on log text during a charge that is WORKING buys nothing and
 *   costs feed and lease headroom. So DIAG is issued strictly last, never ages, never jumps the
 *   queue, and is suppressed outright while energy is live. Logs matter for the sessions that
 *   FAILED, and a vehicle that has refused you has already stopped talking — there is no
 *   contention left to lose by then.
 *
 * All of this is OFF by default. A controller that never calls cbctl_diag_enable() behaves
 * exactly as one built before this existed. */

/* Arm or disarm draining. Disarming does NOT change what the board records — a controller that
 * stops listening should not silently reconfigure a board another tool may be watching. Use
 * cbctl_diag_set_level(CBCTL_DIAG_OFF) for that, explicitly. */
cbctl_status_t cbctl_diag_enable(cbctl_link_t *link, bool on, cbctl_diag_sink_fn sink,
                                 void *user);

/* Ask the board to record at this level / these categories. Intents, like every other setter
 * here: they are transmitted by the tick and re-asserted after a board reset, because cbplc
 * deliberately does not persist them across a reboot (a charger silently retaining a debug
 * firehose through a power cycle is how a diagnostic becomes an incident). */
cbctl_status_t cbctl_diag_set_level(cbctl_link_t *link, cbctl_diag_level_t level);
cbctl_status_t cbctl_diag_set_categories(cbctl_link_t *link, uint32_t mask);

/* Discard whatever the board has buffered. Used to get a clean slate before arming a trace, so
 * one session's log cannot be prefixed with the tail of the last one. */
cbctl_status_t cbctl_diag_clear(cbctl_link_t *link);

/* What the board last reported about its own ring.
 *
 * `dropped` is the field that matters: it counts bytes the ring overwrote because nobody drained
 * fast enough, and it is what separates "the board said nothing" from "the board said something
 * and we lost it". A trace whose drop count is climbing is not evidence, and a controller that
 * cannot tell will quietly draw conclusions from a log with holes in it.
 *
 * Returns false before the board has ever answered a control request — never zeros, for the same
 * reason cbctl_ev_soc_pct() returns -1 rather than 0. */
bool cbctl_diag_state(const cbctl_link_t *link, uint8_t *level, uint32_t *categories,
                      uint32_t *dropped, uint32_t *pending);

/* True once the board has proved it understands the control subcommands. False either because
 * nothing has been asked yet, or because this is an older firmware that answers a control
 * request by draining — in which case draining still works and only the level/category setters
 * are inert. Checking this is how a caller tells "logging is off" from "I cannot turn it on". */
bool cbctl_diag_control_supported(const cbctl_link_t *link);

/* ---- provisioning: what this CHARGER is, and what this BOARD needs trimmed -----------------
 *
 * cbplc stores nothing. It has no NVS, no config file and no board identity, because a stored
 * id that can corrupt is how a dead board and a misconfigured board became indistinguishable.
 * So the three facts that belong to a charger rather than to firmware are DECLARED here, held
 * in the board's RAM while it is powered, and re-asserted by cbctl after a reset — the same
 * shape as the envelope and the authorisation.
 *
 * The controller is the config store. It has a filesystem and already knows all of this.
 *
 * Every setter is an intent: nothing blocks, and cbctl_tick() puts it on the wire. All of it is
 * OFF by default — a controller that never calls these behaves exactly as one built before they
 * existed, and the board keeps the values that shipped with its firmware. */

/* The charger's identity on the V2G wire. ISO 15118-2 evseIDType is 7..37 characters; a string
 * outside that is REFUSED here rather than sent, because a vehicle rejects an out-of-schema
 * EVSEID on every message of the charge loop and that presents as "the car stopped talking".
 *
 * There is no fleet-wide default worth having: shipping one is what made every charger we build
 * announce the same identity. */
cbctl_status_t cbctl_provision_evse_id(cbctl_link_t *link, const char *id);

/* Which HLC protocols the board may negotiate. Mask of CBCTL_PROTO_*; zero is refused.
 *
 * This is an interop instrument. When one vehicle fails on ISO 15118-2 and charges on DIN
 * 70121, restricting the offer answers "is it the protocol?" in a single session. It takes
 * effect at SupportedAppProtocol, so a car that offered both simply falls back rather than
 * failing. */
#define CBCTL_PROTO_DIN70121 0x01u
#define CBCTL_PROTO_ISO2     0x02u
#define CBCTL_PROTO_ALL      (CBCTL_PROTO_DIN70121 | CBCTL_PROTO_ISO2)
cbctl_status_t cbctl_provision_protocols(cbctl_link_t *link, uint8_t mask);

/* Per-board SLAC receive-path trim in dB, ADDED to the firmware's fixed front-end correction.
 *
 * The fixed part models the front end and stays fixed on purpose — an adaptive correction that
 * solves toward a target attenuation makes a distant vehicle look adjacent, which is the
 * cross-gun mismatch SLAC exists to prevent. What it cannot model is the rest of the physical
 * path: cable, connector, coupling, unit-to-unit spread. ISO 15118-3 expects that measured per
 * device. Store the measured value against the board's MAC and declare it at link-up.
 *
 * Bounded to +/-30 dB. This is a calibration knob, never a control loop. */
cbctl_status_t cbctl_provision_attn_trim_db(cbctl_link_t *link, int8_t db);

/* The board's feed deadline in ms — how long an envelope and measurement stay valid before it
 * withdraws permission to energise. 0 restores the compiled default.
 *
 * BENCH ONLY. The default is correct for production and this exists because interop work needed
 * the number moved and the only way was a rebuild. The board publishes it in STATUS, so a
 * charger running a non-default deadline cannot do so invisibly. */
cbctl_status_t cbctl_provision_feed_deadline_ms(cbctl_link_t *link, uint16_t ms);

/* True once a board has ACKed a declaration. False either because nothing has been declared
 * yet, or because this firmware predates provisioning and answered CBPLC_E_BAD_TYPE — in which
 * case the board is running its compiled defaults and cannot be told otherwise. Checking this
 * is how a caller tells "not declared" from "cannot declare". */
bool cbctl_provision_supported(const cbctl_link_t *link);

/* Ask the board to reboot.
 *
 * REFUSED BY THE BOARD unless the connector is genuinely idle — no vehicle on the pilot, no
 * session, every relay proven open. It restarts the thing holding the gun contactor, so a
 * refusal here is the design working, not a failure. Observe the result with cbctl_op_state();
 * a board that accepts it will stop answering shortly afterwards and come back re-identifying,
 * at which point cbctl re-asserts every declaration automatically. */
cbctl_status_t cbctl_request_restart(cbctl_link_t *link, cbctl_op_t *out_op);

#ifdef __cplusplus
}
#endif
#endif /* CBCTL_H */
