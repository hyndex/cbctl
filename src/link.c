/* One PLC link: declared intent in, reconciliation on tick, never a wait.
 *
 * The whole file is written around one constraint. cbplc_request() is stop-and-wait — one
 * outstanding request per link — and cbplc_poll() moves at most one frame each way per call.
 * So reconciliation is necessarily incremental: each tick drains a BOUNDED number of polls
 * and, if the wire is free, puts exactly one declaration on it, chosen by priority.
 *
 * Nothing here waits for a reply. A reply that has not arrived is simply not there yet, and
 * the next tick looks again.
 */
#include "cbctl/cbctl.h"

#include <string.h>

#include "cbplc/auth.h"
#include "cbplc/diag.h"
#include "cbplc/provision.h"
#include "cbplc/limits.h"

void cbctl_status_apply(cbctl_link_t *link, const uint8_t *payload, size_t len,
                        uint32_t now_ms);

/* Called by status.c the moment a board reset is detected. */
void cbctl_link_reestablish(cbctl_link_t *link);
void push_quarantine_event(cbctl_link_t *link, uint32_t now_ms);

/* How often each declaration is re-asserted. 400 ms against the PLC's 1500 ms staleness
 * bound gives THREE attempts before a feed goes stale, so one lost frame cannot cost a
 * session. That margin is the reason for the number; do not raise it without redoing the
 * arithmetic in docs/03-concurrency-and-fleet.md section 5. */
#define CBCTL_FEED_PERIOD_MS 400u
#define CBCTL_HEARTBEAT_PERIOD_MS 400u
/* Observation, not liveness. Slower than the feed on purpose: STATUS is a large message and
 * must never delay LIMITS/FEEDBACK, which are what hold permission to energise. */
#define CBCTL_STATUS_PERIOD_MS 1000u
/* ANTI-STARVATION BOUND for STATUS.
 *
 * Priority ordering here is a SAFETY ordering: the feed holds permission to energise, so it
 * outranks observation. But under a real charging load the high-priority set — LIMITS and
 * FEEDBACK due every tick, plus the heartbeat, the relay lease and auth — consumes the tick's
 * bounded poll budget, and the lowest-priority message never gets a slot. Measured on a fully
 * loaded link: limits=60, feedback=60, heartbeat=33, relay=20, and STATUS=1 across 24 s.
 *
 * That is not a cosmetic loss. STATUS is where plug/unplug edges, the SLAC-match vehicle MAC
 * (Autocharge) and the relay diagnostic counters come from, so starving it blinds the
 * controller precisely while a car is attached.
 *
 * So STATUS ages: past this bound it is promoted above the feed for ONE message. The feed
 * still wins every ordinary tick, and one deferred feed repeat is harmless against a 1500 ms
 * staleness bound with a 400 ms period. */
#define CBCTL_STATUS_STARVE_MS 3000u

/* Re-probe backoff ceiling for a quarantined link. A dead USB endpoint that errors instantly
 * turns an unbounded retry into a busy-wait, which is the thing quarantine exists to stop. */
#define CBCTL_REPROBE_MS 2000u
#define CBCTL_REPROBE_MAX_MS 30000u

/* ---- THE BLOCKING BUDGET, and why the defaults were wrong here ---------------------------
 *
 * cbplc is stop-and-wait: ONE request outstanding, and cbplc_request() returns NOT_READY while
 * one is in flight. So a request the board never answers blocks EVERY other message — including
 * the relay-lease renewal — for the whole retry sequence.
 *
 * With the schema defaults (1000 ms timeout, 3 attempts, 100 ms backoff) that is
 *     3 x (1000 + 100) = 3300 ms
 * of dead air. A 3000 ms relay hold therefore EXPIRES while we are still retrying something
 * else, and the contactor drops with nobody having asked for it. The generic safety condition
 * is `hold > worst_block + renewal_period`, and with renewal = hold/3 that means
 *     hold > 3300 + hold/3  ->  hold > 4950 ms
 * i.e. every hold under ~5 s was unsafe. The live run used 3 s and passed only because the
 * link happened to be clean.
 *
 * A controller ticking at 400 ms has no use for a 1000 ms timeout anyway: a board that has not
 * answered a 3-byte request in 250 ms over 115200 baud is not about to. Tightening it bounds
 * the blocking at
 *     2 x (250 + 60) = 620 ms
 * which makes the safe-hold floor ~930 ms and leaves a 3 s hold with real margin. The retry
 * still exists; it is simply sized to the deadline it shares a wire with. */

/* ---- intent bookkeeping ---------------------------------------------------------------- */

static bool due(cbctl_instant_t now, cbctl_instant_t last, uint32_t period_ms)
{
    if (last.ms == 0u) {
        return true; /* never sent */
    }
    return cbctl_older_than(cbctl_age(now, last), cbctl_dur(period_ms));
}

static void on_message(void *user, const cbplc_msg_t *msg, cbplc_err_t status)
{
    cbctl_link_t *link = (cbctl_link_t *)user;
    if (link == NULL) {
        return;
    }

    /* STATUS IS CHECKED BEFORE msg, and that order is load-bearing: the session reports a
     * timed-out request as deliver(NULL, CBPLC_E_TIMEOUT) — there is no message, because
     * none arrived. Checking msg first swallowed every loss, so a board that answered
     * nothing was never quarantined and was retried forever. */
    if (status != CBPLC_E_OK) {
        /* A LOST DIAG REPLY MUST NOT WEDGE DIAGNOSTICS FOREVER.
         *
         * diag_req_pending is what stops a second drain being issued while one is outstanding,
         * and a request that times out never reaches the reply path that clears it. Without
         * this line the flag latches on the first lost frame and the controller silently stops
         * draining for the rest of its life — the exact shape of the "one outstanding request
         * that never completes" bugs this project has already paid for elsewhere. A control
         * subcommand that was lost is re-armed too, so an intent is not quietly abandoned. */
        if (link->diag_req_pending) {
            link->diag_req_pending = false;
            link->diag_req_was_control = false;
        }
        /* Same rule as the diag drain: an in-flight flag that a timeout never clears wedges the
         * path for the life of the controller. The dirty intent survives, so the tick retries
         * within its budget. */
        if (link->prov_inflight) {
            link->prov_inflight = false;
        }
        if (link->restart_inflight) {
            link->restart_inflight = false;
        }
        /* A LOST RENEWAL IS NEITHER AN ACK NOR A REFUSAL, AND THERE IS NO MESSAGE TO READ.
         *
         * `status != CBPLC_E_OK` means the session had nothing to hand us: every non-OK delivery
         * in cbplc/src/session.c passes NULL (:211 transport, :233/:348 timeout, :253, :306), and
         * the only deliveries carrying a message are CBPLC_E_OK. So classifying the ACK here by
         * reading msg->payload — which the block on the OK path below does, correctly — is an
         * unconditional NULL dereference on the one path whose whole job is to handle loss.
         *
         * relay_ack_pending is armed by every lease renewal, so the trigger is "a gun is holding
         * its contactor closed and the board goes quiet": a USB re-enumeration, a WDT reset, a
         * CDC wedge. Those are the most common real faults on this hardware, and this crashed the
         * process — which on a two-link host takes the OTHER gun's link down with it and drops a
         * contactor that was carrying current.
         *
         * Clear the flag so the path cannot wedge, and account nothing, because we did not learn
         * whether the lease was extended. Deliberately NOT re-stamping last_relay is the safe
         * direction: the lease stays unproven, relay_in_danger() keeps pressing, and the renewal
         * is retried rather than the clock being advanced by one that may never have landed. The
         * loss itself is already counted by consecutive_losses below. */
        if (link->relay_ack_pending) {
            link->relay_ack_pending = false;
        }
        if (link->op_on_wire && link->op_state == CBCTL_OP_PENDING) {
            link->op_state = CBCTL_OP_LOST;
            link->op_on_wire = false;
        }
        link->consecutive_losses++;
        return;
    }
    if (msg == NULL) {
        return;
    }

    switch (msg->type) {
    case CBPLC_T_IDENTITY:
        /* 20 bytes: mac[6] wire reset up_ms[4] build[8]. A short reply is not an identity —
         * accepting one would let a truncated frame name the board. */
        if (msg->payload_len == 20u) {
            link->identity.wire_version = msg->payload[6];
            link->identity.reset_reason = msg->payload[7];
            link->identity.up_ms = (uint32_t)msg->payload[8] |
                                   ((uint32_t)msg->payload[9] << 8) |
                                   ((uint32_t)msg->payload[10] << 16) |
                                   ((uint32_t)msg->payload[11] << 24);
            if (link->identity.valid &&
                memcmp(link->identity.mac, msg->payload, 6u) != 0) {
                /* The board on this port is not the board that was here before. Do NOT quietly
                 * adopt the new identity — a controller that renumbers its guns underneath
                 * itself is how the wrong contactor gets closed. Contain it and say so. */
                link->identity_changed++;
                link->state = CBCTL_LINK_QUARANTINED;
                link->quarantine_permanent = true;   /* not a deadline — see the field's note */
                push_quarantine_event(link, link->session.last_now_ms);
                break;
            }
            memcpy(link->identity.mac, msg->payload, 6u);
            memcpy(link->identity.build, &msg->payload[12], 8u);
            link->identity.build[8] = '\0';
            link->identity.valid = true;
        }
        /* A board that names itself is a board we may talk to. Until then every other
         * declaration is refused — a run that cannot say which board and which firmware
         * produced it is not evidence, and this project has lost days to exactly that. */
        if (link->state == CBCTL_LINK_IDENTIFYING || link->state == CBCTL_LINK_QUARANTINED) {
            link->state = CBCTL_LINK_READY;
            link->quarantine_count = 0u; /* it recovered; forget the backoff history */
        }
        break;

    case CBPLC_T_STATUS:
        cbctl_status_apply(link, msg->payload, msg->payload_len, link->session.last_now_ms);
        break;

    case CBPLC_T_RESTART:
        /* The board answers a restart with its own type rather than an ACK, so the gate verdict
         * is unambiguous: byte 0 is CBPLC_E_OK when it accepted and will reboot, or the reason
         * it refused. A refusal is INFORMATION — the connector was not idle — not a transport
         * failure, and must not be retried in a loop. */
        link->restart_inflight = false;
        if (link->op_state == CBCTL_OP_PENDING) {
            link->op_gate = (msg->payload_len > 0u) ? msg->payload[0] : 0u;
            link->op_state = (link->op_gate == 0u) ? CBCTL_OP_APPLIED : CBCTL_OP_REFUSED;
            link->op_on_wire = false;
        }
        break;

    case CBPLC_T_DIAG: {
        /* ONE REPLY TYPE, TWO MEANINGS, DISAMBIGUATED BY THE PAYLOAD ITSELF.
         *
         * A control reply begins with a NUL and carries a magic byte, a version and an exact
         * length. Drained log text is printf output, which contains no NUL, so the two cannot
         * be confused — and that is what lets a controller talk to a board built before the
         * subcommands existed. Such a board answers a control REQUEST by draining, so the reply
         * arrives as text; seeing text where a control reply was expected is therefore not an
         * error, it is the capability answer. */
        cbplc_diag_state_t st;
        const bool was_control = link->diag_req_was_control;
        link->diag_req_pending = false;
        link->diag_req_was_control = false;

        if (cbplc_diag_decode_reply(msg->payload, msg->payload_len, &st)) {
            link->diag_ctl_supported = true;
            link->diag_state_valid = true;
            link->last_diag_state = cbctl_instant(link->session.last_now_ms);
            link->diag_board_level = st.level;
            link->diag_board_cats = st.categories;
            link->diag_dropped = st.dropped;
            link->diag_pending = st.pending;
            /* THE BOARD'S ANSWER IS THE TRUTH, NOT OUR REQUEST. A setter whose value was
             * clamped or refused leaves the intent dirty so the tick re-asserts it; without
             * this, a caller would believe a level the board never adopted. */
            if (st.level == link->diag_want_level) {
                link->diag_level_dirty = false;
            }
            if (st.categories == link->diag_want_cats) {
                link->diag_cats_dirty = false;
            }
            /* A backlog on the board is a reason to hurry, and the only one we get. */
            if (st.pending > 0u) {
                link->diag_period_ms = CBCTL_DIAG_PERIOD_MIN_MS;
            }
            break;
        }

        if (was_control) {
            /* Text where a control reply was expected: an older firmware. Record the verdict
             * once and stop asking — but KEEP the bytes, because they are real log content that
             * this board drained on our behalf, and discarding them would silently lose the
             * only diagnostics such a board can give. */
            link->diag_ctl_supported = false;
            link->diag_level_dirty = false;
            link->diag_cats_dirty = false;
        }
        if (msg->payload_len > 0u) {
            link->diag_bytes += msg->payload_len;
            /* CONTENT MEANS KEEP LISTENING. Any non-empty reply collapses the interval to the
             * floor: the board caps one drain at CBCTL_DIAG_CHUNK_FULL, so a session's worth of
             * trace is several replies, and backing off between them would stretch a 4 KB log
             * over half a minute. Only silence backs off. */
            link->diag_period_ms = CBCTL_DIAG_PERIOD_MIN_MS;
            if (link->diag_sink != NULL) {
                link->diag_sink(msg->payload, msg->payload_len, link->diag_user);
            }
        } else if (!was_control) {
            /* Nothing new. Back off geometrically so a quiet board costs almost nothing —
             * without this, a controller would spend a round trip a second on "no" forever. */
            uint32_t next = link->diag_period_ms * 2u;
            if (next > CBCTL_DIAG_PERIOD_MAX_MS || next < link->diag_period_ms) {
                next = CBCTL_DIAG_PERIOD_MAX_MS;
            }
            link->diag_period_ms = next;
        }
        break;
    }

    case CBPLC_T_ACK:
        /* The board answered the outstanding op. Payload byte 0 is the result code: zero is
         * applied, anything else is a REFUSAL that names its gate. A refusal is information,
         * not a transient failure — retrying it in a loop is how a controller argues with an
         * interlock forever. */
        if (link->prov_inflight) {
            const uint8_t rc = (msg->payload_len > 0u) ? msg->payload[0] : 0u;
            link->prov_inflight = false;
            if (rc == 0u) {
                /* CLEARED ONLY ON THE BOARD'S OWN ACK, never on transmission. A declaration
                 * that was sent and lost must be re-sent; believing it because it left is how a
                 * charger ends up announcing an identity nobody applied. */
                link->prov_dirty = false;
                link->prov_supported = true;
                link->prov_tries = 0u;
                link->prov_accepted++;
            } else if (rc == (uint8_t)CBPLC_E_BAD_TYPE) {
                /* A firmware that predates provisioning. Stop asking — it is running its
                 * compiled defaults and no amount of repeating changes that — but keep the
                 * intent so a later board reset (or a reflash) re-asserts it. */
                link->prov_supported = false;
                link->prov_dirty = false;
                link->prov_refused++;
            } else {
                /* The board understood and REFUSED: a value it considers invalid. Repeating it
                 * cannot help, so the bounded try budget stops us arguing, and the intent stays
                 * visible in link->prov for a caller to inspect. */
                link->prov_refused++;
            }
        }
        /* A relay ACK is the only proof the lease was actually extended. Re-stamp from it so
         * the renewal clock measures APPLIED renewals, not attempted ones. */
        if (link->relay_ack_pending) {
            link->relay_ack_pending = false;
            if ((msg->payload_len == 0u) || (msg->payload[0] == 0u)) {
                link->last_relay = cbctl_instant(link->session.last_now_ms);
                link->relay_acks++;
                link->relay_refusal_streak = 0u;
            } else {
                /* Refused: the lease was NOT extended.
                 *
                 * Retrying AT ONCE is right for a transient gate and ruinous for a standing one.
                 * cbplc refuses a gun close while authorisation is not yet live (`auth=0`), which
                 * can persist for seconds — and this path retried every tick for as long as it
                 * lasted. Measured on hardware: 2509 refusals in one run, 125 in a 10 s window.
                 * On a stop-and-wait wire those retries crowd out the AUTHORIZE renewal that is
                 * the very thing that would clear the refusal, so the loop sustains itself.
                 *
                 * So: retry immediately a couple of times — a genuinely transient gate clears
                 * there — then fall back to the ordinary renewal cadence, which still re-attempts
                 * the close forever, just without starving the link that has to carry the fix.
                 * Any ACK clears the streak. */
                link->relay_refusals++;
                if (link->relay_refusal_streak < CBCTL_RELAY_REFUSAL_FAST_RETRIES) {
                    link->last_relay = cbctl_instant(0u);
                }
                if (link->relay_refusal_streak < 0xFFFFFFFFu) {
                    link->relay_refusal_streak++;
                }
            }
        }
        if (link->op_on_wire && link->op_state == CBCTL_OP_PENDING) {
            const uint8_t code = (msg->payload_len > 0u) ? msg->payload[0] : 0u;
            link->op_gate = code;
            link->op_state = (code == 0u) ? CBCTL_OP_APPLIED : CBCTL_OP_REFUSED;
            link->op_on_wire = false;
        }
        link->consecutive_losses = 0u;
        break;

    default:
        break;
    }
}

/* ---- public ---------------------------------------------------------------------------- */

cbctl_status_t cbctl_open(cbctl_link_t *link, const cbplc_transport_t *tx)
{
    if (link == NULL || tx == NULL) {
        return CBCTL_E_ARG;
    }
    memset(link, 0, sizeof *link);

    {
        /* NOT the schema defaults — see the blocking-budget note above. */
        cbplc_timing_t timing;
        cbplc_timing_defaults(&timing);
        timing.request_timeout_ms = CBCTL_REQUEST_TIMEOUT_MS;
        timing.retry_backoff_ms = CBCTL_RETRY_BACKOFF_MS;
        timing.retry_limit = (uint8_t)CBCTL_RETRY_LIMIT;
        if (cbplc_open(&link->session, tx, &timing, on_message, link) != CBPLC_E_OK) {
            return CBCTL_E_TRANSPORT;
        }
    }

    /* Zeroed intent is the SAFE intent: no envelope, no measurement, no authorisation,
     * isolation UNKNOWN. A caller who ticks without declaring anything energises nothing.
     *
     * "No authorisation" IS PENDING, NOT DENY, and the difference is the whole session.
     * This header already says so one enum over — "still deciding — keeps the session alive;
     * NOT the same as DENY" — and this line used to contradict it.
     *
     * On the cbplc wire DENY is not a quiet withholding, it is a TERMINAL REFUSAL: the SECC
     * answers AuthorizationRes FAILED/Finished, the vehicle treats that as final and tears down,
     * and the board latches the refusal for CBPLC_AUTH_DENY_TTL_MS (30 s). The firmware is so
     * careful about this that a LAPSED lease demotes to PENDING rather than DENY, with its own
     * comment explaining that saying FAILED to a car on the strength of a merely-busy controller
     * makes it stop asking.
     *
     * cbctl then did exactly what the firmware refuses to do. Opening a link put a terminal
     * refusal on the wire before the application had formed any opinion, and re-asserted it at
     * ~1 Hz until a grant arrived. Measured against a real vehicle: auth_ok=3 against
     * auth_no=104, slac_matches=5 against slac_rearms=33 — a self-sustaining loop where the car
     * matched, asked, was failed, retired the stack, re-matched, and asked again.
     *
     * PENDING withholds energy exactly as completely. It just does not lie about it being final. */
    link->state = CBCTL_LINK_IDENTIFYING;
    link->auth = CBCTL_AUTH_PENDING;
    link->next_op_id = 1u;
    link->magic = CBCTL_LINK_MAGIC;
    return CBCTL_OK;
}

bool cbctl_identity(const cbctl_link_t *link, cbctl_identity_t *out)
{
    if (link == NULL || out == NULL || !link->identity.valid) {
        return false;
    }
    *out = link->identity;
    return true;
}

cbctl_status_t cbctl_expect_mac(const cbctl_link_t *link, const uint8_t mac[6])
{
    if (link == NULL || mac == NULL) {
        return CBCTL_E_ARG;
    }
    if (!link->identity.valid) {
        return CBCTL_E_NOT_IDENTIFIED;
    }
    return (memcmp(link->identity.mac, mac, 6u) == 0) ? CBCTL_OK : CBCTL_E_WRONG_BOARD;
}

void cbctl_link_reestablish(cbctl_link_t *link)
{
    if (link == NULL) {
        return;
    }
    /* A REBOOTED BOARD HAS FORGOTTEN EVERYTHING IT WAS TOLD.
     *
     * The periodic declarations — LIMITS, FEEDBACK, AUTHORIZE, RELAY, HEARTBEAT — repair
     * themselves within one period, so they were never the problem. The STOP LATCH is the
     * problem: it is sent once when it changes, so a controller that had raised a stop and
     * whose board then rebooted would sit there believing the charger was stopped while the
     * board came up perfectly willing to energise. Nothing would ever correct it.
     *
     * Re-assert every once-only piece of state, and re-verify WHO the board is: a reset is
     * also the window in which hardware gets swapped, and coming back as a different board on
     * the same port is exactly the mix-up cbctl_expect_mac() exists to catch. */
    link->stop_dirty = true;
    link->envelope_dirty = true;
    link->measurement_dirty = true;
    link->auth_dirty = true;
    /* cbplc deliberately does NOT persist its log level or category mask across a reboot — a
     * charger silently retaining a debug firehose through a power cycle is how a diagnostic
     * becomes an incident. That makes re-asserting them our job, and it is the same once-only
     * class of state as the stop latch: a controller that armed a V2G trace, whose board then
     * reset, would otherwise sit believing the trace was running against a board recording
     * nothing, and would draw conclusions from the silence. */
    if (link->diag_ctl_supported || !link->diag_ctl_probed) {
        link->diag_level_dirty = true;
        link->diag_cats_dirty = true;
        link->diag_probe_dirty = link->diag_on;
        link->diag_ctl_tries = 0u;
    }
    link->diag_state_valid = false;
    link->diag_req_pending = false;
    link->diag_req_was_control = false;
    /* PROVISIONING IS ONCE-ONLY STATE AND THE BOARD DOES NOT KEEP IT.
     *
     * cbplc holds a declaration in RAM only, so a reset returns it to compiled defaults — which
     * for the EVSE ID means the fleet-wide fallback. A controller that did not re-assert would
     * leave a charger quietly announcing the wrong identity for the rest of its uptime, with
     * nothing anywhere saying so. Same class as the stop latch. */
    if (link->prov.evse_id_set || link->prov.protocol_mask_set || link->prov.attn_trim_set ||
        link->prov.session_timeout_set) {
        link->prov_dirty = true;
        link->prov_tries = 0u;
    }
    link->prov_inflight = false;
    link->restart_inflight = false;
    link->relay_dirty = true;
    link->reidentify_pending = true;
    link->reestablishes++;
}

cbctl_link_state_t cbctl_link_state(const cbctl_link_t *link)
{
    return (link != NULL) ? link->state : CBCTL_LINK_CLOSED;
}

cbctl_status_t cbctl_set_envelope(cbctl_link_t *link, const cbctl_envelope_t *env)
{
    if (link == NULL || env == NULL) {
        return CBCTL_E_ARG;
    }
    /* Ready-while-stopping is self-cancelling and the vehicle resolves it by resetting TCP.
     * Refused at the call so it never reaches the wire. */
    if (env->power_ready && link->stop_requested) {
        return CBCTL_E_CONTRADICTORY;
    }
    /* Only a REAL change is dirty. A caller that re-declares the same envelope every pass —
     * which the API explicitly permits — must not thereby force a send every tick. */
    if (!link->have_envelope || memcmp(&link->envelope, env, sizeof *env) != 0) {
        link->envelope = *env;
        link->envelope_dirty = true;
    }
    link->have_envelope = true;
    return CBCTL_OK;
}

cbctl_status_t cbctl_set_measurement(cbctl_link_t *link, const cbctl_measurement_t *m)
{
    if (link == NULL || m == NULL) {
        return CBCTL_E_ARG;
    }
    /* NOISE MUST NOT PREEMPT THE LINK.
     *
     * This used to mark the intent dirty on ANY difference. Against real hardware every read
     * jitters by a few millivolts, so the comparison never matched and FEEDBACK was sent on every
     * single tick. On a stop-and-wait wire that is not merely wasteful — it STARVES the other
     * intents. Measured during a live session: 618 FEEDBACK frames against 7 AUTHORIZE frames in
     * 23 s. The 8 s authorisation lease renews every lease/3, could not get on the wire, lapsed,
     * and the board then refused the gun close 1136 times with `auth=0` while the vehicle sat
     * mid-charge. The feed flooded out the very message that keeps the contactor legal.
     *
     * The freshness guarantee does NOT come from dirty — it comes from the periodic send at
     * CBCTL_FEED_PERIOD_MS, which carries whatever value is stored at that moment. So a numeric
     * update always reaches the board within one feed period; it just no longer jumps the queue.
     *
     * ISOLATION is different and stays edge-urgent: it is a discrete safety verdict, not a
     * measurement, and a change from VALID to FAULT must not wait for a timer. */
    if (!link->have_measurement || link->measurement.isolation != m->isolation) {
        link->measurement_dirty = true;
    }
    link->measurement = *m;
    link->have_measurement = true;
    return CBCTL_OK;
}

cbctl_status_t cbctl_authorize(cbctl_link_t *link, cbctl_auth_t decision, uint32_t lease_ms)
{
    if (link == NULL) {
        return CBCTL_E_ARG;
    }
    if (decision == CBCTL_AUTH_GRANTED) {
        /* A lease shorter than the tick period cannot survive one missed tick, so granting
         * it is a way of writing "revoke this almost immediately" that reads like "grant". */
        /* The SAME blocking budget that bounds the relay hold bounds this. An authorisation
         * lease that cannot outlast one unanswered request plus its own re-assert period will
         * lapse on a busy wire, and a lapsed authorisation ends a live session — quieter than
         * a dropped contactor, and just as much a charge the driver did not get. */
        if (lease_ms < CBPLC_AUTH_LEASE_MIN_MS || lease_ms > CBPLC_AUTH_LEASE_MAX_MS ||
            lease_ms <= CBCTL_FEED_PERIOD_MS || lease_ms < CBCTL_AUTH_LEASE_MIN_SAFE_MS) {
            return CBCTL_E_ARG;
        }
        link->auth_lease_ms = lease_ms;
    }
    if (link->auth != decision) {
        link->auth_dirty = true;
    }
    link->auth = decision;
    return CBCTL_OK;
}

cbctl_status_t cbctl_begin_stop(cbctl_link_t *link)
{
    if (link == NULL) {
        return CBCTL_E_ARG;
    }
    /* Both halves, together. Clearing power_ready is not tidiness: a stop that leaves demand
     * commanded makes the PLC break the contactor on its decay budget, i.e. under load, and
     * this hardware has no weld sensor. See cbplc/docs/07-close-path-vs-reference.md. */
    link->stop_requested = true;
    link->stop_dirty = true;
    link->envelope.power_ready = false;
    link->envelope_dirty = true;   /* the cleared power_ready must reach the board at once */
    return CBCTL_OK;
}

cbctl_status_t cbctl_stop_clear(cbctl_link_t *link)
{
    if (link == NULL) {
        return CBCTL_E_ARG;
    }
    link->stop_requested = false;
    link->stop_dirty = true;
    /* power_ready is deliberately NOT restored. Releasing the latch must not, by itself,
     * re-arm the power path — the caller declares an envelope when it actually intends to
     * deliver again. */
    return CBCTL_OK;
}

cbctl_op_state_t cbctl_op_state(const cbctl_link_t *link, cbctl_op_t op)
{
    if (link == NULL || op.id == 0u || link->op.id != op.id) {
        return CBCTL_OP_NONE;
    }
    return link->op_state;
}

/* ---- the tick --------------------------------------------------------------------------- */


/* ---- relay leases ------------------------------------------------------------------------ */

/* AN OP BELONGS TO ONE MESSAGE TYPE, and recording which one is not bookkeeping.
 *
 * There is a single op slot, and several branches of issue_one() marked "the outstanding op is
 * on the wire" whenever ANY pending op existed — not only when they were sending that op's own
 * message. So a RELAY frame going out while a restart was queued set op_on_wire, and the relay's
 * ACK resolved the RESTART op as applied, from a board that had never seen a restart request.
 *
 * This is the same defect the op_on_wire flag was introduced to fix, one level up: it stopped an
 * unrelated message's completion resolving an op, and this stops an unrelated message's
 * TRANSMISSION claiming one. Caught by a test, not on hardware. */
static cbctl_op_t new_op(cbctl_link_t *link, uint8_t owner_type)
{
    const cbctl_op_t op = { link->next_op_id++ };
    link->op = op;
    link->op_state = CBCTL_OP_PENDING;
    link->op_gate = 0u;
    link->op_type = owner_type;
    link->op_on_wire = false;
    return op;
}

/* Is the message about to go out the one the outstanding op is waiting for? */
static bool op_owns(const cbctl_link_t *link, uint8_t type)
{
    return link->op_state == CBCTL_OP_PENDING && link->op_type == type;
}

cbctl_status_t cbctl_relay_close(cbctl_link_t *link, uint8_t mask, uint32_t hold_ms,
                                 cbctl_op_t *out_op)
{
    if (link == NULL || (mask & ~(uint8_t)CBPLC_RELAY_ALL) != 0u || mask == 0u) {
        return CBCTL_E_ARG;
    }
    /* A CLOSE WITH NO DEAD-MAN IS THE ONE RELAY MISTAKE THIS HARDWARE CANNOT ABSORB.
     * There is no contactor feedback and no weld sensor, so a contactor left closed by a
     * controller that died is only detectable by someone noticing. Refused at the call. */
    if (hold_ms == 0u) {
        return CBCTL_E_ARG;
    }
    if (hold_ms > CBPLC_RELAY_HOLD_MAX_MS) {
        hold_ms = CBPLC_RELAY_HOLD_MAX_MS;  /* clamped, and the caller can see it below */
    }
    /* RAISED, not refused. A hold this short cannot survive one unanswered request on a
     * stop-and-wait wire, so honouring it literally would hand the caller a lease that opens
     * the contactor for no reason. Raising it is the safe direction — the dead-man still
     * exists, it is merely long enough to be renewable. */
    if (hold_ms < CBCTL_RELAY_HOLD_MIN_MS) {
        hold_ms = CBCTL_RELAY_HOLD_MIN_MS;
    }
    /* QUANTISE TO THE WIRE, UPWARD, AND STORE WHAT THE BOARD WILL ACTUALLY HOLD.
     *
     * RELAY carries the hold in 100 ms units, so the board truncates. We were storing the
     * caller's raw value and computing the renewal period from it, which meant our arithmetic
     * used a lease up to 99 ms LONGER than the one the board was enforcing — renewing against
     * a deadline that had already passed. Rounding UP rather than down keeps the board's lease
     * >= our belief, so the error can only ever make us renew early. */
    hold_ms = ((hold_ms + 99u) / 100u) * 100u;
    if (hold_ms > CBPLC_RELAY_HOLD_MAX_MS) {
        hold_ms = CBPLC_RELAY_HOLD_MAX_MS;   /* re-clamp: rounding may have pushed it over */
    }
    if (link->state != CBCTL_LINK_READY) {
        return (link->state == CBCTL_LINK_QUARANTINED) ? CBCTL_E_QUARANTINED
                                                       : CBCTL_E_NOT_IDENTIFIED;
    }
    /* CLOSING AN ENERGY RELAY WHILE A STOP IS LATCHED IS SELF-CANCELLING.
     *
     * It is the same contradiction cbctl_set_envelope() already refuses for power_ready, and
     * it deserves the same answer: the controller is asking the charger to stop and to connect
     * the gun at once. The PLC would refuse it too — but a refusal that arrives 250 ms later
     * as a gate code is far harder to attribute than an error at the call. Opening is of
     * course still allowed; see cbctl_relay_open(). */
    if (link->stop_requested && (mask & (CBPLC_RELAY_GUN | CBPLC_RELAY_POWER)) != 0u) {
        return CBCTL_E_CONTRADICTORY;
    }

    link->relay_enable = (uint8_t)(link->relay_enable | mask);
    link->relay_state = (uint8_t)(link->relay_state | mask);
    link->relay_hold_ms = hold_ms;
    link->relay_dirty = true;
    /* Re-baseline the override detector: this is a NEW intent, so the board's past safety
     * opens are not evidence against it. */
    link->relay_safety_base = link->counters[CBCTL_CTR_RELAY_SAFETY_OPENS];
    link->relay_base_valid = link->have_flags;
    if (out_op != NULL) {
        *out_op = new_op(link, (uint8_t)CBPLC_T_RELAY);
    }
    return CBCTL_OK;
}

cbctl_status_t cbctl_relay_open(cbctl_link_t *link, uint8_t mask, cbctl_op_t *out_op)
{
    if (link == NULL || (mask & ~(uint8_t)CBPLC_RELAY_ALL) != 0u) {
        return CBCTL_E_ARG;
    }
    /* Opening is never the dangerous direction, so it is legal in every state — including
     * QUARANTINED, where the attempt may not reach the board but must still be expressible. */
    link->relay_enable = (uint8_t)(link->relay_enable | mask);
    link->relay_state = (uint8_t)(link->relay_state & ~mask);
    link->relay_dirty = true;
    if (out_op != NULL) {
        *out_op = new_op(link, (uint8_t)CBPLC_T_RELAY);
    }
    return CBCTL_OK;
}

/* Subcommand values are the firmware's: 0 = arm, 1 = replug. Encoded here rather than exposed
 * as a raw byte, so a caller cannot invent a third one. */
static cbctl_status_t slac_cmd(cbctl_link_t *link, uint8_t sub, cbctl_op_t *out_op)
{
    if (link == NULL) {
        return CBCTL_E_ARG;
    }
    if (link->state != CBCTL_LINK_READY) {
        return (link->state == CBCTL_LINK_QUARANTINED) ? CBCTL_E_QUARANTINED
                                                       : CBCTL_E_NOT_IDENTIFIED;
    }
    link->slac_pending = (uint8_t)(sub + 1u);
    if (out_op != NULL) {
        *out_op = new_op(link, (uint8_t)CBPLC_T_SLAC);
    }
    return CBCTL_OK;
}

cbctl_status_t cbctl_slac_arm(cbctl_link_t *link, cbctl_op_t *out_op)
{
    return slac_cmd(link, 0u, out_op);
}

cbctl_status_t cbctl_slac_replug(cbctl_link_t *link, cbctl_op_t *out_op)
{
    return slac_cmd(link, 1u, out_op);
}

uint8_t cbctl_op_gate(const cbctl_link_t *link, cbctl_op_t op)
{
    if (link == NULL || op.id == 0u || link->op.id != op.id) {
        return 0u;
    }
    return link->op_gate;
}

/* Renewal period for a held relay lease: a THIRD of the hold, so three renewals may be lost
 * before a contactor drops. The caller never picks this. See the note on cbctl_relay_close. */
static uint32_t relay_period(const cbctl_link_t *link)
{
    uint32_t p = link->relay_hold_ms / CBCTL_RELAY_RENEW_DIVISOR;
    if (p < 100u) {
        p = 100u;
    }
    return p;
}

/* Is the lease close enough to lapsing that it should jump the queue?
 *
 * Measured against the last CONFIRMED renewal, not the last attempt — an attempt that was
 * never applied is exactly the case this is here to survive. */
static bool relay_in_danger(const cbctl_link_t *link, cbctl_instant_t now)
{
    uint32_t age;
    if (link->relay_enable == 0u || (link->relay_state & CBPLC_RELAY_ALL) == 0u ||
        link->last_relay.ms == 0u) {
        return false;
    }
    age = (uint32_t)cbctl_age(now, link->last_relay).ms;
    return age >= (link->relay_hold_ms / CBCTL_RELAY_DANGER_DIVISOR);
}

uint32_t cbctl_lease_headroom_ms(const cbctl_link_t *link, uint32_t now_ms)
{
    uint32_t age;
    if (link == NULL || link->relay_enable == 0u ||
        (link->relay_state & CBPLC_RELAY_ALL) == 0u || link->last_relay.ms == 0u) {
        return 0u;
    }
    age = (uint32_t)cbctl_age(cbctl_instant(now_ms), link->last_relay).ms;
    return (age >= link->relay_hold_ms) ? 0u : (link->relay_hold_ms - age);
}

/* Re-assertion period for AUTHORIZE: a third of the lease, floored so a short lease cannot
 * turn into a busy-wait and ceilinged so a long one is still refreshed regularly. DENY and
 * PENDING carry no lease, so they use the floor. */
static uint32_t auth_period(const cbctl_link_t *link)
{
    uint32_t p;
    if (link->auth != CBCTL_AUTH_GRANTED || link->auth_lease_ms == 0u) {
        return 1000u;
    }
    p = link->auth_lease_ms / 3u;
    if (p < CBCTL_AUTH_PERIOD_FLOOR_MS) {
        p = CBCTL_AUTH_PERIOD_FLOOR_MS;
    }
    if (p > 5000u) {
        p = 5000u;
    }
    return p;
}

/* Put ONE due declaration on the wire. Returns true if something was sent.
 *
 * Order is priority order, and it is a safety ordering rather than a fairness one: the feed
 * is what keeps permission to energise alive, so it outranks everything that merely changes
 * what the charger would do. */
/* Everything the tick can put on the wire must fit. Checked here rather than trusted, so a
 * future field added to the envelope fails the build instead of smashing the stack. */
_Static_assert(CBPLC_LIMITS_PAYLOAD_LEN <= 64u, "issue_one buffer too small for LIMITS");

/* MAY WE SPEND A ROUND TRIP ON LOG TEXT RIGHT NOW?
 *
 * No, while energy is live. cbplc allows ONE outstanding request, so a drain in flight is
 * ~620 ms of worst-case wire against a 1500 ms feed bound and a lease renewing at hold/6 — and
 * it buys nothing, because a charge that is DELIVERING is not the one anybody needs a trace of.
 * The sessions worth reading are the ones that failed, and by then the vehicle has stopped
 * talking and there is no contention left to lose.
 *
 * "Live" is deliberately the union of two independent signals rather than either alone. A close
 * intent means we are holding a contactor, which is the safety-critical state regardless of what
 * the board reports; a board-reported active session covers the window before our intent exists
 * and after it is dropped. Erring toward suppression is free — the ring is 16 KB and holds the
 * whole session, so anything suppressed here is drained a moment later, intact.
 *
 * Suppressions are COUNTED. A trace that is quiet because the tool declined to ask looks exactly
 * like a board that had nothing to say, and those must not be confusable. */
static bool diag_drain_allowed(const cbctl_link_t *link)
{
    if ((link->relay_state & CBPLC_RELAY_ALL) != 0u) {
        return false;
    }
    return !cbctl_flag(link, CBCTL_FLAG_SESSION_ACTIVE);
}

static bool issue_one(cbctl_link_t *link, cbctl_instant_t now)
{
    uint8_t buf[64];
    size_t n;
    uint16_t id = 0u;   /* MUST be non-NULL: passing NULL makes the request fire-and-forget,
                         * which un-gates stop-and-wait and lets one tick flood the board. */

    if (link->state == CBCTL_LINK_IDENTIFYING || link->state == CBCTL_LINK_QUARANTINED ||
        link->reidentify_pending) {
        if (cbplc_request(&link->session, CBPLC_T_IDENTIFY, NULL, 0u, now.ms, &id) ==
            CBPLC_E_OK) {
            link->reidentify_pending = false;
            return true;
        }
        return false;
    }

    /* THE LATCH, FIRST. Only sent when it has actually changed, so it costs nothing on a
     * normal tick — but when it changes it must not wait behind a feed repeat. */
    if (link->stop_dirty) {
        buf[0] = link->stop_requested ? 1u : 0u;
        if (cbplc_request(&link->session, CBPLC_T_STOP, buf, 1u, now.ms, &id) == CBPLC_E_OK) {
            link->stop_dirty = false;
            return true;
        }
        return false;
    }

    if (link->slac_pending != 0u) {
        buf[0] = (uint8_t)(link->slac_pending - 1u);
        if (cbplc_request(&link->session, CBPLC_T_SLAC, buf, 1u, now.ms, &id) == CBPLC_E_OK) {
            link->slac_pending = 0u;
            if (op_owns(link, (uint8_t)CBPLC_T_SLAC)) {
                link->op_on_wire = true;
            }
            return true;
        }
        return false;
    }

    /* A LEASE ABOUT TO LAPSE OUTRANKS EVERYTHING, INCLUDING THE FEED.
     *
     * The normal order puts the feed first, because the feed holds permission to energise. But
     * a lapsed lease drops a contactor immediately, whereas a feed repeat arriving one tick
     * late is still comfortably inside its own 1500 ms bound. When the controller has been
     * starved and is catching up, the lease is what it must save first. */
    if (relay_in_danger(link, now)) {
        buf[0] = (uint8_t)(link->relay_enable & CBPLC_RELAY_ALL);
        buf[1] = (uint8_t)(link->relay_state & buf[0]);
        buf[2] = (uint8_t)(link->relay_hold_ms / 100u);
        if (cbplc_request(&link->session, CBPLC_T_RELAY, buf, 3u, now.ms, &id) == CBPLC_E_OK) {
            link->relay_ack_pending = true;
            link->relay_dirty = false;
            link->relay_renewals++;
            link->relay_danger_preempts++;
            return true;
        }
        return false;
    }

    /* Aged-out STATUS jumps the queue, once. See CBCTL_STATUS_STARVE_MS. */
    if (link->state == CBCTL_LINK_READY &&
        due(now, link->last_status_req, CBCTL_STATUS_STARVE_MS)) {
        if (cbplc_request(&link->session, CBPLC_T_STATUS_REQUEST, NULL, 0u, now.ms, &id) ==
            CBPLC_E_OK) {
            link->last_status_req = now;
            link->status_promotions++;
            link->status_sent++;
            return true;
        }
        return false;
    }

    if (link->have_envelope &&
        (link->envelope_dirty || due(now, link->last_limits, CBCTL_FEED_PERIOD_MS))) {
        cbplc_envelope_t e;
        memset(&e, 0, sizeof e);
        e.max_voltage_mv = cbctl_mv(link->envelope.max_voltage);
        e.min_voltage_mv = cbctl_mv(link->envelope.min_voltage);
        e.max_current_ma = cbctl_ma(link->envelope.max_current);
        e.min_current_ma = cbctl_ma(link->envelope.min_current);
        e.max_power_w = cbctl_w(link->envelope.max_power);
        e.power_ready = link->envelope.power_ready;
        n = cbplc_limits_encode_envelope(&e, buf, sizeof buf);
        if (n > 0u && cbplc_request(&link->session, CBPLC_T_LIMITS, buf, (uint16_t)n, now.ms,
                                    NULL) == CBPLC_E_OK) {
            link->last_limits = now;
            link->envelope_dirty = false;
            link->limits_sent++;
            return true;
        }
        return false;
    }

    if (link->have_measurement &&
        (link->measurement_dirty || due(now, link->last_feedback, CBCTL_FEED_PERIOD_MS))) {
        cbplc_measurement_t m;
        memset(&m, 0, sizeof m);
        m.present_voltage_mv = cbctl_mv(link->measurement.present_voltage);
        m.present_current_ma = cbctl_ma(link->measurement.present_current);
        m.isolation = (uint8_t)link->measurement.isolation;
        /* THE CONTACTOR IS DERIVED, NOT SUPPLIED — and it used to be neither.
         *
         * cbplc's measurement carries a contactor state and the SECC reads it as one input to
         * "is energy actually expected". This builder memset the struct and never set the field,
         * so every FEEDBACK cbctl has ever sent said CONTACTOR_UNKNOWN(0). Observed on hardware:
         * `dcreq: type=18 finished=1 fresh=1 iso=2 contactor=0` repeating while the gun was
         * physically closed, with the vehicle re-asking instead of advancing to PreCharge.
         *
         * It is derived from the BOARD'S REPORTED relay mask rather than from our own commanded
         * intent, because those two disagree exactly when it matters — the board's safety layer
         * can open a contactor we asked to close, and telling the SECC a circuit exists when it
         * does not is the dangerous direction of this lie.
         *
         * Deriving it also makes it unforgettable: there is no caller-supplied field to leave
         * unset, which is the bug that was here. Before the first STATUS we genuinely do not
         * know, and UNKNOWN is the honest answer — not OPEN, which is a claim. */
        if (!link->have_flags) {
            m.contactor = CBPLC_CONTACTOR_UNKNOWN;
        } else {
            m.contactor = ((link->relay_closed_mask & CBPLC_RELAY_GUN) != 0u)
                              ? CBPLC_CONTACTOR_CLOSED
                              : CBPLC_CONTACTOR_OPEN;
        }
        n = cbplc_limits_encode_measurement(&m, buf, sizeof buf);
        if (n > 0u && cbplc_request(&link->session, CBPLC_T_FEEDBACK, buf, (uint16_t)n,
                                    now.ms, &id) == CBPLC_E_OK) {
            link->last_feedback = now;
            link->measurement_dirty = false;
            link->feedback_sent++;
            return true;
        }
        return false;
    }

    /* AUTH IS RE-ASSERTED ON THE LEASE'S OWN CLOCK, NOT THE FEED'S.
     *
     * A third of the lease gives three attempts before it could lapse — the same margin the
     * feed gets against its 1500 ms bound — while not spamming an 8 s grant at 2.5 Hz.
     *
     * ON STARVATION, WHICH THIS COMMENT HAS NOW BEEN WRONG ABOUT IN BOTH DIRECTIONS.
     *
     * On an IDLE link there is no starvation: issue_one() is called repeatedly within a tick
     * while the wire is free, so a lower-priority declaration still gets a slot (measured
     * auth=4, heartbeat=9 over ten ticks).
     *
     * Under a real CHARGING load it is entirely different. LIMITS and FEEDBACK fall due every
     * tick, and with the heartbeat, the relay lease and auth they consume the tick's bounded
     * poll budget, so the lowest-priority message is starved outright — measured limits=60,
     * feedback=60, heartbeat=33, relay=20, STATUS=1 across 24 s.
     *
     * So priority decides ORDER within a tick until the budget binds, and after that it
     * decides whether a message is sent AT ALL. That is why STATUS ages
     * (CBCTL_STATUS_STARVE_MS) and why the soak asserts the controller can still observe the
     * board mid-charge. Do not add another low-priority message without the same treatment. */
    /* The relay lease outranks auth and the heartbeat: letting it lapse physically drops a
     * contactor mid-charge, which is louder and worse than a late authorisation. */
    /* A LEASE IS A CLOSE. AN OPEN IS AN INSTRUCTION, AND IT MUST NOT BE "RENEWED".
     *
     * This gate was `relay_enable != 0`, i.e. "have we ever touched a relay" — and
     * cbctl_relay_open() SETS relay_enable (it has to; the mask says which relays the frame
     * addresses). So the first open-all a caller issues put the link into a state where it
     * retransmitted that open forever on the lease cadence, incrementing relay_renewals and,
     * on each ACK, relay_acks.
     *
     * The board cannot object: cbplc's RELAY handler only runs cbplc_relay_may_close() when
     * `want != 0`, so an all-open is accepted with gate 0 by design. Measured on the charger,
     * against a live vehicle: renewals 50, acks 49, refusals 0, lease headroom reported — and
     * `relay_closed=0x00` on the board for every single STATUS of the run, with
     * `limits_refusal=contactor_open` while everything else was green. cbctl reported a
     * healthy dead-man lease the whole time it was commanding the gun OPEN once a second.
     *
     * That is the same class of defect as the feed_age-of-0-when-never-sent lie: an
     * instrumentation counter that reads healthy for the failure it is supposed to expose.
     *
     * So: a DIRTY intent always goes out once, open or close — an open must reach the board
     * promptly. Only a genuinely HELD close is renewed, and only that increments the renewal
     * counters. `held` is the same expression the danger/headroom helpers already use, which
     * is why cbctl_lease_headroom_ms() correctly read 0 ms throughout — it was the one place
     * that already knew nothing was being held. */
    const uint8_t held = (uint8_t)(link->relay_state & link->relay_enable & CBPLC_RELAY_ALL);
    if (link->relay_enable != 0u &&
        (link->relay_dirty ||
         (held != 0u && due(now, link->last_relay, relay_period(link))))) {
        buf[0] = (uint8_t)(link->relay_enable & CBPLC_RELAY_ALL);
        buf[1] = (uint8_t)(link->relay_state & buf[0]);
        buf[2] = (uint8_t)(link->relay_hold_ms / 100u); /* the wire carries 100 ms units */
        if (cbplc_request(&link->session, CBPLC_T_RELAY, buf, 3u, now.ms, &id) == CBPLC_E_OK) {
            /* Provisional. The authoritative stamp is applied when the board ACKS — see
             * relay_ack_pending below. Stamping only here would let a renewal that was queued
             * and never applied reset our timer, so a silent board would look renewed while
             * its lease quietly ran out. */
            link->last_relay = now;
            link->relay_dirty = false;
            /* Only a CLOSE has a lease, so only a close may report one. Counting an open as a
             * renewal is what let "renewals 50 / acks 49" describe a run whose gun was open
             * from end to end. */
            if (held != 0u) {
                link->relay_ack_pending = true;
                link->relay_renewals++;
            }
            if (op_owns(link, (uint8_t)CBPLC_T_RELAY)) {
                link->op_on_wire = true;
            }
            return true;
        }
        return false;
    }

    if (link->auth_dirty || due(now, link->last_auth, auth_period(link))) {
        const cbplc_auth_state_t st = (link->auth == CBCTL_AUTH_GRANTED)
                                          ? CBPLC_AUTH_GRANTED
                                          : (link->auth == CBCTL_AUTH_PENDING
                                                 ? CBPLC_AUTH_PENDING
                                                 : CBPLC_AUTH_DENY);
        n = cbplc_auth_encode(st, link->auth_lease_ms, buf, sizeof buf);
        if (n > 0u && cbplc_request(&link->session, CBPLC_T_AUTHORIZE, buf, (uint16_t)n,
                                    now.ms, &id) == CBPLC_E_OK) {
            link->last_auth = now;
            link->auth_dirty = false;
            link->auth_sent++;
            return true;
        }
        return false;
    }

    if (due(now, link->last_heartbeat, CBCTL_HEARTBEAT_PERIOD_MS)) {
        if (cbplc_request(&link->session, CBPLC_T_HEARTBEAT, NULL, 0u, now.ms, &id) ==
            CBPLC_E_OK) {
            link->last_heartbeat = now;
            return true;
        }
        return false;
    }

    if (due(now, link->last_status_req, CBCTL_STATUS_PERIOD_MS)) {
        if (cbplc_request(&link->session, CBPLC_T_STATUS_REQUEST, NULL, 0u, now.ms, &id) ==
            CBPLC_E_OK) {
            link->last_status_req = now;   /* cadence */
            link->status_sent++;
            return true;
        }
    }

    /* ---- PROVISIONING: rare, small, and above diagnostics ----------------------------------
     *
     * Below every operational message, because nothing here holds a contactor or permission to
     * energise — but ABOVE the log drain, because a board running the wrong EVSE ID is
     * announcing the wrong charger to a vehicle right now, while a log can wait.
     *
     * It is not periodic. A declaration goes out when it CHANGES and after a board reset, and
     * then never again: re-sending an identity every second would be pure noise on a
     * stop-and-wait wire. `prov_dirty` is cleared only when the board ACKs, so a lost frame
     * retries rather than being silently believed. */
    if (link->prov_dirty && !link->prov_inflight && link->prov_tries < CBCTL_PROV_TRIES) {
        n = cbplc_provision_encode(&link->prov, buf, sizeof buf);
        if (n > 0u && cbplc_request(&link->session, CBPLC_T_PROVISION, buf, (uint16_t)n,
                                    now.ms, &id) == CBPLC_E_OK) {
            link->prov_inflight = true;
            link->prov_probed = true;
            link->prov_tries++;
            return true;
        }
        return false;
    }

    if (link->restart_pending && !link->restart_inflight) {
        if (cbplc_request(&link->session, CBPLC_T_RESTART, NULL, 0u, now.ms, &id) ==
            CBPLC_E_OK) {
            link->restart_pending = false;
            link->restart_inflight = true;
            if (op_owns(link, (uint8_t)CBPLC_T_RESTART)) {
                link->op_on_wire = true;
            }
            return true;
        }
        return false;
    }

    /* ---- DIAGNOSTICS: LAST, ALWAYS, AND HAPPY TO BE STARVED --------------------------------
     *
     * Everything above this line either holds permission to energise, holds a contactor, or is
     * the controller's only view of the board. This does none of those things, so it goes after
     * all of them and it does NOT age its way forward the way STATUS does.
     *
     * That asymmetry is the whole design. STATUS was given the right to jump the queue because a
     * controller going blind mid-charge is dangerous. DIAG is the opposite case: on a
     * stop-and-wait wire one request is a full round trip, and spending that on log text during
     * a charge that is working costs feed and lease headroom for no benefit at all. Under load
     * the messages above will consume the tick's poll budget and this will simply not run —
     * which is the correct outcome, not a bug to fix later with an aging counter.
     *
     * A control subcommand outranks a drain, because a controller waiting for its trace to
     * actually turn on should not queue behind kilobytes of the log it does not want yet. */
    {
    /* An INTENT is bounded by the try budget — arguing with a board that keeps answering
     * something else is pointless. A periodic state READ is not an intent and must not be
     * governed by that budget: it is the only thing that keeps `dropped` and `pending` from
     * going stale, and staleness there is what makes a drained log look complete when it is
     * not. */
    const bool ctl_intent = (link->diag_clear_dirty || link->diag_level_dirty ||
                             link->diag_cats_dirty || link->diag_probe_dirty) &&
                            link->diag_ctl_tries < CBCTL_DIAG_CTL_TRIES;
    const bool state_due = link->diag_on && link->diag_ctl_supported &&
                           due(now, link->last_diag_state, CBCTL_DIAG_STATE_PERIOD_MS);
    if (!link->diag_req_pending && (ctl_intent || state_due)) {
        /* Derived at SEND time from the intent, never queued as a command. A queue can hold a
         * value the caller has since changed its mind about; the intent cannot. Order matters:
         * CLEAR first so an armed trace starts from an empty ring rather than the tail of the
         * previous vehicle's session, then the level, then the categories. */
        cbplc_diag_cmd_t cmd;
        uint32_t arg;
        if (link->diag_clear_dirty) {
            cmd = CBPLC_DIAG_CLEAR;
            arg = 0u;
        } else if (link->diag_level_dirty) {
            cmd = CBPLC_DIAG_SET_LEVEL;
            arg = link->diag_want_level;
        } else if (link->diag_cats_dirty) {
            cmd = CBPLC_DIAG_SET_CATS;
            arg = link->diag_want_cats;
        } else {
            cmd = CBPLC_DIAG_GET_STATE;   /* the probe, and the periodic refresh */
            arg = 0u;
        }
        n = cbplc_diag_encode_request(cmd, arg, buf, sizeof buf);
        if (n > 0u && cbplc_request(&link->session, CBPLC_T_DIAG, buf, (uint16_t)n, now.ms,
                                    &id) == CBPLC_E_OK) {
            link->diag_req_pending = true;
            link->diag_req_was_control = true;
            link->diag_ctl_probed = true;
            /* Only an INTENT spends the budget. The periodic refresh must be able to run
             * forever without exhausting the thing that bounds re-assertion. */
            if (ctl_intent) {
                link->diag_ctl_tries++;
            }
            /* CLEAR has no readback to confirm it — the reply reports pending==0, which is also
             * what an already-empty ring reports — so it is retired on transmission. The level
             * and categories are NOT: they are confirmed against what the board sends back. */
            if (cmd == CBPLC_DIAG_CLEAR) {
                link->diag_clear_dirty = false;
            }
            /* The probe is answered either way — a control reply proves support, log text
             * proves the opposite — so it is retired on transmission and never repeats. */
            if (cmd == CBPLC_DIAG_GET_STATE) {
                link->diag_probe_dirty = false;
            }
            return true;
        }
        return false;
    }
    }

    if (link->diag_on && !link->diag_req_pending &&
        due(now, link->last_diag, link->diag_period_ms)) {
        if (!diag_drain_allowed(link)) {
            /* Counted, and the cadence clock is STAMPED so a suppressed pass does not leave a
             * drain permanently "due" — otherwise every tick of a long charge would re-evaluate
             * this branch and fall through, and the moment the session ended a burst of drains
             * would be owed all at once. */
            link->last_diag = now;
            link->diag_suppressed++;
            return false;
        }
        /* An EMPTY payload, not an explicit DRAIN byte. Every firmware ever built answers this
         * by draining, so the read path works against a board that has never heard of the
         * subcommands — and detecting that is what the control probe is for, separately. */
        if (cbplc_request(&link->session, CBPLC_T_DIAG, NULL, 0u, now.ms, &id) == CBPLC_E_OK) {
            link->last_diag = now;
            link->diag_req_pending = true;
            link->diag_req_was_control = false;
            link->diag_drains++;
            return true;
        }
        return false;
    }
    return false;
}

cbctl_status_t cbctl_tick(cbctl_link_t *link, uint32_t now_ms)
{
    const cbctl_instant_t now = cbctl_instant(now_ms);
    bool transport_dead = false;
    unsigned i;

    /* Refuse to drive a link that was never opened rather than dereferencing whatever the
     * caller's stack happened to contain. */
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }

    /* HOW LATE IS THE CALLER? A stalled control loop is the thing most likely to lapse a
     * lease on a busy charger, and it is invisible unless measured here. */
    if (link->last_tick.ms != 0u) {
        const uint32_t gap = (uint32_t)cbctl_age(now, link->last_tick).ms;
        if (gap > link->worst_tick_gap_ms) {
            link->worst_tick_gap_ms = gap;
        }
    }
    link->last_tick = now;

    {
        const uint32_t head = cbctl_lease_headroom_ms(link, now_ms);
        if (link->last_relay.ms != 0u && (link->relay_state & CBPLC_RELAY_ALL) != 0u) {
            if (link->lease_min_headroom_ms == 0u || head < link->lease_min_headroom_ms) {
                link->lease_min_headroom_ms = head;
            }
        }
    }

    if (link->state == CBCTL_LINK_QUARANTINED) {
        if (link->quarantine_permanent) {
            return CBCTL_E_QUARANTINED; /* a swapped or duplicated board is never re-probed */
        }
        if (!cbctl_reached(now, link->quarantined_until)) {
            return CBCTL_E_QUARANTINED; /* contained: costs this tick nothing */
        }
        /* THE STATE DOES NOT CHANGE HERE, and that is the fix.
         *
         * It used to drop back to IDENTIFYING to re-probe, so a permanently dead board spent
         * most of its life reporting IDENTIFYING and only briefly QUARANTINED — the answer to
         * "is this board bad?" flapped depending on when you asked. A caller cannot act on a
         * signal like that. Now the link stays QUARANTINED until an IDENTITY actually comes
         * back, and only then becomes READY. Re-probing is something a quarantined link does,
         * not a state it enters. */
        link->consecutive_losses = 0u;
    }

    /* Drain, BOUNDED. This ceiling is what stops one talkative or one broken board from
     * consuming a tick that N-1 other boards are also depending on. */
    if (cbplc_busy(&link->session)) {
        link->issue_blocked_busy++;
    }
    for (i = 0u; i < CBCTL_POLL_PER_TICK; i++) {
        const bool was_busy = cbplc_busy(&link->session);
        const cbplc_err_t prc = cbplc_poll(&link->session, now_ms);

        /* A HARD TRANSPORT FAILURE MUST COUNT AS A LOSS.
         *
         * Quarantine used to be driven only by request TIMEOUTS, so a link whose fd had gone
         * away never reached it: with a dead transport no request is ever in flight, nothing
         * times out, and the link sat in DEGRADED forever being retried at full rate every
         * tick. Observed on hardware by killing one board's transport mid-run — exactly the
         * USB drop this project has had in the field. Counted once per tick, not once per
         * poll, so the quarantine threshold still means "N consecutive ticks". */
        if (prc == CBPLC_E_TRANSPORT) {
            transport_dead = true;
        }

        if (!cbplc_busy(&link->session)) {
            if (was_busy && link->op_on_wire && link->op_state == CBCTL_OP_PENDING) {
                /* The op's OWN request finished without an ACK reaching us. Gated on
                 * op_on_wire: an unrelated heartbeat completing here used to be recorded as
                 * the caller's relay close having been lost. */
                link->op_state = CBCTL_OP_LOST;
                link->op_on_wire = false;
                link->consecutive_losses++;
            }
            if (!issue_one(link, now)) {
                break; /* nothing due — the normal idle exit */
            }
        }
    }
    if (i == CBCTL_POLL_PER_TICK) {
        link->polls_capped++;
    }
    if (transport_dead) {
        link->consecutive_losses++;
        link->transport_failures++;
    }

    if (link->consecutive_losses >= CBCTL_LOSS_LIMIT) {
        /* Exponential, capped. A board that has failed ten times running is not worth
         * probing at the same rate as one that just missed a reply, and an unbounded retry
         * against a dead USB endpoint that errors instantly is a busy-wait. */
        uint32_t backoff = CBCTL_REPROBE_MS;
        uint8_t k = link->quarantine_count;
        while (k-- > 0u && backoff < CBCTL_REPROBE_MAX_MS) {
            backoff *= 2u;
        }
        if (backoff > CBCTL_REPROBE_MAX_MS) {
            backoff = CBCTL_REPROBE_MAX_MS;
        }
        if (link->quarantine_count < 255u) {
            link->quarantine_count++;
        }
        if (link->state != CBCTL_LINK_QUARANTINED) {
            push_quarantine_event(link, now_ms);
        }
        link->state = CBCTL_LINK_QUARANTINED;
        link->consecutive_losses = 0u;
        link->quarantined_until = cbctl_add(now, cbctl_dur(backoff));
    }

    /* Did we let the feed go stale? Counted rather than merely avoided, because a controller
     * that is silently late is a controller nobody knows is late. */
    /* OBSERVATION STALENESS drives READY <-> DEGRADED.
     *
     * The link can be perfectly healthy on the wire while our picture of the charger is
     * minutes old — a board that keeps ACKing commands but has stopped answering STATUS, or
     * whose STATUS is truncated every time, produces exactly that. DEGRADED existed in the
     * enum and nothing ever set it, so this condition had no way to be reported at all. */
    if (link->state == CBCTL_LINK_READY || link->state == CBCTL_LINK_DEGRADED) {
        const uint32_t obs = cbctl_observation_age_ms(link, now_ms);
        const bool stale = (obs != CBCTL_NEVER_OBSERVED) &&
                           (obs >= CBCTL_OBSERVATION_STALE_MS);
        if (stale && link->state == CBCTL_LINK_READY) {
            link->state = CBCTL_LINK_DEGRADED;
            link->observation_stalls++;
        } else if (!stale && link->state == CBCTL_LINK_DEGRADED) {
            link->state = CBCTL_LINK_READY;
        }
    }

    /* Guarded on last_limits.ms: a stamp of zero means "not sent yet", NOT "infinitely old".
     * Without this guard the counter reported 6.4 million late ticks on a healthy link. */
    /* COUNT THE EDGE, NOT THE TICK.
     *
     * This used to increment on every tick the feed was stale, so one dead link reported
     * 6,024,729 "late feeds" — a number that carries no information and buries the case that
     * matters (a feed that went late once, briefly, on a healthy link). What an operator needs
     * is HOW MANY TIMES it went late, so count transitions and keep the current state. */
    if (link->have_envelope && link->last_limits.ms != 0u) {
        const bool late =
            cbctl_older_than(cbctl_age(now, link->last_limits), cbctl_dur(CBPLC_LIMITS_STALE_MS));
        if (late && !link->feed_is_late) {
            link->feed_late++;   /* one per excursion */
        }
        link->feed_is_late = late;
    }
    return CBCTL_OK;
}

uint32_t cbctl_observation_age_ms(const cbctl_link_t *link, uint32_t now_ms)
{
    if (link == NULL || !link->have_flags || link->last_status.ms == 0u) {
        return CBCTL_NEVER_OBSERVED;
    }
    return (uint32_t)cbctl_age(cbctl_instant(now_ms), link->last_status).ms;
}

uint32_t cbctl_next_deadline_ms(const cbctl_link_t *link, uint32_t now_ms)
{
    const cbctl_instant_t now = cbctl_instant(now_ms);
    uint32_t best = CBCTL_FEED_PERIOD_MS;
    uint32_t age;

    if (link == NULL) {
        return CBCTL_FEED_PERIOD_MS;
    }
    if (link->state == CBCTL_LINK_QUARANTINED) {
        cbctl_age_t a;
        /* PERMANENT QUARANTINE HAS NO DEADLINE, AND ASKING FOR ONE RETURNS ZERO FOREVER.
         *
         * quarantine_permanent is a policy state, so nothing ever stamps quarantined_until on
         * those paths (link.c identity-changed, fleet.c duplicate-board). The reversed-argument
         * idiom below reads "time remaining", so an unstamped 0 computes 0 - now, clamps to 0,
         * and this link asks to be woken immediately for the rest of the process's life. There
         * is nothing to do for a board that is contained forever; say so with a long period. */
        if (link->quarantine_permanent) {
            return CBCTL_REPROBE_MS;
        }
        a = cbctl_age(link->quarantined_until, now);
        return (a.ms > 0) ? (uint32_t)a.ms : 0u;
    }
    if (cbplc_busy(&link->session)) {
        return 0u; /* work in flight: come straight back */
    }

    /* STATUS IS DUE ON EVERY LINK. THE FEED IS DUE ONLY ON ONE THAT DECLARED AN ENVELOPE.
     *
     * This used to schedule purely from last_limits, which is stamped only when a LIMITS feed
     * actually goes out. On a link that has never declared an envelope — an IDLE GUN, the most
     * ordinary state a second connector is ever in — last_limits stays 0, the computed age is
     * therefore enormous, and the function returned 0. Nothing then stamps it, because there is
     * no envelope to send, so the answer stays 0 forever and the caller stops sleeping.
     *
     * Measured on evse-h85fw: one link 2.4% CPU, two links with the second one idle 25% — a
     * tenfold cost for a board that was doing nothing. The direction is safe, since a spinning
     * loop services BOTH guns more often rather than less, which is exactly why no test caught
     * it: cbctl_next_deadline_ms is asserted only from above (CHECK(d <= 400u)).
     *
     * The two sibling functions already carry this guard — cbctl_feed_age_ms() returns early on
     * !have_envelope, and the feed-late counter tests last_limits.ms != 0u — so the omission was
     * here alone.
     *
     * A link that HAS declared an envelope but not yet sent it keeps returning 0, which is
     * correct and self-limiting: the very next tick sends the feed and stamps the clock. */
    if (!link->have_envelope) {
        /* Nothing to feed, so there is no feed clock to schedule from — but this function is the
         * ONLY wake-up signal a caller has, and every other periodic obligation on the link
         * (STATUS, heartbeat, identity retry, provisioning and AUTHORIZE re-assert) still has to
         * be reached. Do not try to compute the earliest of them here: their cadences live in
         * the tick, each behind its own due() test, and a second copy of that arithmetic in this
         * function is a second thing to keep in step.
         *
         * Return the library's base cadence instead. It is shorter than every one of those
         * obligations and shorter than nothing at all was, so no schedule tightens or slips —
         * the caller simply sleeps between passes rather than spinning through them. In-flight
         * requests are already handled above by cbplc_busy(), which is what keeps the 250 ms
         * retry ladder from being paced by this number. */
        return CBCTL_FEED_PERIOD_MS;
    }

    age = (uint32_t)cbctl_age(now, link->last_limits).ms;
    if (age >= CBCTL_FEED_PERIOD_MS) {
        return 0u;
    }
    best = CBCTL_FEED_PERIOD_MS - age;
    return best;
}

/* ---- board diagnostics --------------------------------------------------------------------
 *
 * Every function here is an INTENT, like the rest of the library: nothing waits, nothing blocks,
 * and cbctl_tick() is what touches the wire. See the header for why diagnostics are the one
 * message class deliberately allowed to starve.
 */
cbctl_status_t cbctl_diag_enable(cbctl_link_t *link, bool on, cbctl_diag_sink_fn sink,
                                 void *user)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }
    /* A SINK IS REQUIRED TO ARM. Draining is DESTRUCTIVE — the board's ring hands the bytes over
     * and forgets them — so enabling without somewhere to put them would quietly consume the
     * evidence it was turned on to collect. Refusing here makes that impossible to write. */
    if (on && sink == NULL) {
        return CBCTL_E_ARG;
    }
    link->diag_sink = sink;
    link->diag_user = user;
    if (on && !link->diag_on) {
        /* Start at the floor so an armed trace is picked up promptly, and let the backoff find
         * the quiet rate by itself. */
        link->diag_period_ms = CBCTL_DIAG_PERIOD_MIN_MS;
        link->last_diag = cbctl_instant(0u);
        link->diag_probe_dirty = true;
        link->diag_ctl_tries = 0u;
    }
    link->diag_on = on;
    return CBCTL_OK;
}

cbctl_status_t cbctl_diag_set_level(cbctl_link_t *link, cbctl_diag_level_t level)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }
    if ((uint32_t)level > (uint32_t)CBCTL_DIAG_OFF) {
        return CBCTL_E_ARG;
    }
    /* Only an ACTUAL change re-arms the try budget. Re-stating a level that is already in force
     * must be free, because a caller may legitimately assert its policy every pass — the same
     * lesson the feed learned when re-declaring it every tick starved every other message. */
    if (link->diag_want_level != (uint8_t)level || !link->diag_state_valid) {
        link->diag_want_level = (uint8_t)level;
        link->diag_level_dirty = true;
        link->diag_ctl_tries = 0u;
    }
    return CBCTL_OK;
}

cbctl_status_t cbctl_diag_set_categories(cbctl_link_t *link, uint32_t mask)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }
    if (link->diag_want_cats != mask || !link->diag_state_valid) {
        link->diag_want_cats = mask;
        link->diag_cats_dirty = true;
        link->diag_ctl_tries = 0u;
    }
    return CBCTL_OK;
}

cbctl_status_t cbctl_diag_clear(cbctl_link_t *link)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }
    link->diag_clear_dirty = true;
    link->diag_ctl_tries = 0u;
    return CBCTL_OK;
}

bool cbctl_diag_state(const cbctl_link_t *link, uint8_t *level, uint32_t *categories,
                      uint32_t *dropped, uint32_t *pending)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC || !link->diag_state_valid) {
        /* NOT zeros. "The board has never told us" and "the board reports nothing dropped" are
         * different facts, and rendering the first as the second is how a controller concludes a
         * trace is complete when it has never spoken to the ring at all. */
        return false;
    }
    if (level != NULL) { *level = link->diag_board_level; }
    if (categories != NULL) { *categories = link->diag_board_cats; }
    if (dropped != NULL) { *dropped = link->diag_dropped; }
    if (pending != NULL) { *pending = link->diag_pending; }
    return true;
}

bool cbctl_diag_control_supported(const cbctl_link_t *link)
{
    return link != NULL && link->magic == CBCTL_LINK_MAGIC && link->diag_ctl_supported;
}

/* ---- provisioning -------------------------------------------------------------------------
 *
 * Intents, like everything else here. Each marks the declaration dirty ONLY on an actual change
 * — a caller may legitimately re-state its whole configuration every pass, and making that
 * re-transmit would be the same defect the feed had when re-declaring it starved every other
 * message. */
static cbctl_status_t prov_touch(cbctl_link_t *link)
{
    link->prov_dirty = true;
    link->prov_tries = 0u;
    return CBCTL_OK;
}

cbctl_status_t cbctl_provision_evse_id(cbctl_link_t *link, const char *id)
{
    size_t n;

    if (link == NULL || link->magic != CBCTL_LINK_MAGIC || id == NULL) {
        return CBCTL_E_ARG;
    }
    n = strlen(id);
    /* REFUSED AT THE CALL, not one round trip later. An EVSEID outside ISO 15118-2's 7..37 is
     * rejected by strict vehicles on every message of the charge loop, and that surfaces as
     * "the car stopped talking" rather than as a configuration error anybody can act on. */
    if (n < CBPLC_EVSE_ID_MIN || n > CBPLC_EVSE_ID_MAX) {
        return CBCTL_E_ARG;
    }
    if (link->prov.evse_id_set && link->prov.evse_id_len == (uint8_t)n &&
        memcmp(link->prov.evse_id, id, n) == 0) {
        return CBCTL_OK;   /* unchanged: free */
    }
    memcpy(link->prov.evse_id, id, n);
    link->prov.evse_id[n] = '\0';
    link->prov.evse_id_len = (uint8_t)n;
    link->prov.evse_id_set = true;
    return prov_touch(link);
}

cbctl_status_t cbctl_provision_protocols(cbctl_link_t *link, uint8_t mask)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }
    /* Zero would mean "negotiate nothing", i.e. a charger that answers no vehicle. Never what a
     * caller meant, and not expressible. */
    if (mask == 0u || (mask & (uint8_t)~CBCTL_PROTO_ALL) != 0u) {
        return CBCTL_E_ARG;
    }
    if (link->prov.protocol_mask_set && link->prov.protocol_mask == mask) {
        return CBCTL_OK;
    }
    link->prov.protocol_mask = mask;
    link->prov.protocol_mask_set = true;
    return prov_touch(link);
}

cbctl_status_t cbctl_provision_attn_trim_db(cbctl_link_t *link, int8_t db)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }
    if (db < CBPLC_ATTN_TRIM_MIN || db > CBPLC_ATTN_TRIM_MAX) {
        return CBCTL_E_ARG;
    }
    if (link->prov.attn_trim_set && link->prov.attn_trim_db == db) {
        return CBCTL_OK;
    }
    link->prov.attn_trim_db = db;
    link->prov.attn_trim_set = true;
    return prov_touch(link);
}

cbctl_status_t cbctl_provision_feed_deadline_ms(cbctl_link_t *link, uint16_t ms)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }
    if (ms > CBPLC_SESSION_PARAM_TIMEOUT_MAX_MS) {
        return CBCTL_E_ARG;
    }
    if (link->prov.session_timeout_set && link->prov.session_timeout_ms == ms) {
        return CBCTL_OK;
    }
    link->prov.session_timeout_ms = ms;
    link->prov.session_timeout_set = true;
    return prov_touch(link);
}

bool cbctl_provision_supported(const cbctl_link_t *link)
{
    return link != NULL && link->magic == CBCTL_LINK_MAGIC && link->prov_supported;
}

cbctl_status_t cbctl_request_restart(cbctl_link_t *link, cbctl_op_t *out_op)
{
    if (link == NULL || link->magic != CBCTL_LINK_MAGIC) {
        return CBCTL_E_ARG;
    }
    if (link->state != CBCTL_LINK_READY) {
        return (link->state == CBCTL_LINK_QUARANTINED) ? CBCTL_E_QUARANTINED
                                                       : CBCTL_E_NOT_IDENTIFIED;
    }
    /* REFUSED WHILE WE ARE HOLDING A CONTACTOR. The board has its own gate and it is the
     * authoritative one, but asking it to reboot while this controller is actively leasing a
     * relay closed is a self-cancelling instruction — the same class as power_ready-while-
     * stopping, and refused at the call for the same reason. */
    if ((link->relay_state & CBPLC_RELAY_ALL) != 0u) {
        return CBCTL_E_CONTRADICTORY;
    }
    link->restart_pending = true;
    if (out_op != NULL) {
        *out_op = new_op(link, (uint8_t)CBPLC_T_RESTART);
    }
    return CBCTL_OK;
}
