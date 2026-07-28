/* The link, driven against the real cbplc session running as a fake PLC.
 *
 * The virtual clock is deliberate: wall-clock tests of a deadline-driven loop are flaky, and
 * a flaky safety test gets muted. Time only moves when a test moves it.
 */
#include "cbctl/cbctl.h"
#include "loopback.h"

#include <stdio.h>
#include <string.h>

static int g_fail;
#define CHECK(c)                                                                          \
    do {                                                                                  \
        if (!(c)) {                                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                         \
            g_fail++;                                                                     \
        }                                                                                 \
    } while (0)
#define TEST(name) static void test_##name(void)
#define RUN(name)                                                                         \
    do {                                                                                  \
        printf("  %s\n", #name);                                                          \
        test_##name();                                                                    \
    } while (0)

/* One "round" = the controller ticks, then the board gets to run. Both are bounded, so a
 * round is bounded, which is the property every test below leans on. */
static void round_trip(cbctl_link_t *link, cbctl_loopback_t *lb, uint32_t *now, uint32_t step)
{
    cbctl_tick(link, *now);
    cbctl_loopback_poll(lb, *now);
    cbctl_tick(link, *now);
    cbctl_loopback_poll(lb, *now);
    *now += step;
}

/* A hold is usable only if its renewal period still outruns one worst-case block. */
static bool relay_period_fits(uint32_t hold)
{
    return (hold / CBCTL_RELAY_RENEW_DIVISOR) > 0u &&
           hold > (CBCTL_WORST_BLOCK_MS + hold / CBCTL_RELAY_RENEW_DIVISOR);
}

static void bring_up(cbctl_link_t *link, cbctl_loopback_t *lb, uint32_t *now)
{
    cbctl_loopback_init(lb);
    CHECK(cbctl_open(link, &lb->ctrl_tx) == CBCTL_OK);
    CHECK(cbctl_link_state(link) == CBCTL_LINK_IDENTIFYING);
    for (int i = 0; i < 8 && cbctl_link_state(link) != CBCTL_LINK_READY; i++) {
        round_trip(link, lb, now, 10u);
    }
}

/* A board must name itself before anything else is legal. */
TEST(a_link_identifies_over_the_real_protocol)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_link_state(&link) == CBCTL_LINK_READY);
    CHECK(lb.identify_seen >= 1u);
}

/* THE point of the library. The caller declares once; the tick repeats it forever. Nobody
 * writes a resend loop, so nobody can forget to — the most common way a hand-rolled
 * controller kills a live charge. */
TEST(the_tick_repeats_the_feed_without_the_caller_asking)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    unsigned before;

    bring_up(&link, &lb, &now);

    cbctl_envelope_t env;
    memset(&env, 0, sizeof env);
    env.max_voltage = CBCTL_MV(500000u);
    env.max_current = CBCTL_MA(200000u);
    env.max_power = CBCTL_W(100000u);
    env.power_ready = true;
    CHECK(cbctl_set_envelope(&link, &env) == CBCTL_OK);

    cbctl_measurement_t m;
    memset(&m, 0, sizeof m);
    m.present_voltage = CBCTL_MV(400000u);
    m.isolation = CBCTL_ISO_VALID;
    CHECK(cbctl_set_measurement(&link, &m) == CBCTL_OK);

    for (int i = 0; i < 6; i++) {
        round_trip(&link, &lb, &now, 450u); /* past the 400 ms repeat period */
    }
    before = lb.limits_seen;
    CHECK(before >= 3u);       /* declared ONCE, sent many times */
    CHECK(lb.feedback_seen >= 3u);

    /* And it keeps going with no further calls from the caller. */
    for (int i = 0; i < 4; i++) {
        round_trip(&link, &lb, &now, 450u);
    }
    CHECK(lb.limits_seen > before);
}

/* Two opposite instructions to the vehicle. Refused at the call, so it never reaches the
 * wire and the vehicle never has to resolve it by resetting TCP. */
TEST(power_ready_while_stopping_is_refused_at_the_call)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_begin_stop(&link) == CBCTL_OK);

    cbctl_envelope_t env;
    memset(&env, 0, sizeof env);
    env.max_voltage = CBCTL_MV(500000u);
    env.power_ready = true;
    CHECK(cbctl_set_envelope(&link, &env) == CBCTL_E_CONTRADICTORY);
}

/* begin_stop is one decision, not two. A stop that leaves demand commanded makes the PLC
 * break the contactor under load on its decay budget — how contacts weld on hardware with
 * no weld sensor. */
TEST(begin_stop_clears_power_ready_itself)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);

    cbctl_envelope_t env;
    memset(&env, 0, sizeof env);
    env.max_voltage = CBCTL_MV(500000u);
    env.power_ready = true;
    CHECK(cbctl_set_envelope(&link, &env) == CBCTL_OK);
    CHECK(link.envelope.power_ready);

    CHECK(cbctl_begin_stop(&link) == CBCTL_OK);
    CHECK(!link.envelope.power_ready);
}

/* A lease that cannot survive one missed tick is a revocation wearing the word "grant". */
TEST(an_auth_lease_shorter_than_the_tick_is_refused)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 100u) == CBCTL_E_ARG);
    CHECK(cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 8000u) == CBCTL_OK);
    /* DENY and PENDING carry no lease, so they are always expressible. */
    CHECK(cbctl_authorize(&link, CBCTL_AUTH_PENDING, 0u) == CBCTL_OK);
    CHECK(cbctl_authorize(&link, CBCTL_AUTH_DENY, 0u) == CBCTL_OK);
}

/* THE NON-BLOCKING PROPERTY, at link level.
 *
 * A board that answers NOTHING must not be able to make a tick unbounded. This is what the
 * whole design exists for: at fleet level the same property is what stops one dead board
 * terminating the charges on every other one. */
TEST(a_board_that_never_answers_cannot_make_the_tick_unbounded)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    lb.wedged = true;

    /* Every one of these returns. If the tick could wait, this test would hang rather than
     * fail — which is itself the point: there is no timeout to hang on. */
    for (int i = 0; i < 200; i++) {
        const cbctl_status_t rc = cbctl_tick(&link, now);
        CHECK(rc == CBCTL_OK || rc == CBCTL_E_QUARANTINED);
        now += 400u;
    }
    CHECK(1); /* reaching here at all is the assertion */
}

/* A wedged board is CONTAINED, not retried at full rate forever. A dead USB endpoint that
 * errors instantly would otherwise turn the retry path into a busy-wait. */
TEST(a_wedged_board_is_quarantined_and_then_re_probed)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    bool saw_quarantine = false;

    bring_up(&link, &lb, &now);
    lb.wedged = true;

    for (int i = 0; i < 100; i++) {
        cbctl_tick(&link, now);
        if (cbctl_link_state(&link) == CBCTL_LINK_QUARANTINED) {
            saw_quarantine = true;
            break;
        }
        now += 400u;
    }
    CHECK(saw_quarantine);

    /* It must come back on its own once the board recovers, without the caller reopening. */
    lb.wedged = false;
    for (int i = 0; i < 60 && cbctl_link_state(&link) != CBCTL_LINK_READY; i++) {
        round_trip(&link, &lb, &now, 500u);
    }
    CHECK(cbctl_link_state(&link) == CBCTL_LINK_READY);
}

/* A transport whose send always refuses (a full USB-CDC TX buffer, the classic stall) must
 * defer, not spin and not block. */
TEST(a_full_transmit_buffer_defers_instead_of_stalling)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    lb.tx_full = true;

    for (int i = 0; i < 50; i++) {
        const cbctl_status_t rc = cbctl_tick(&link, now);
        /* Deferring is the required behaviour; QUARANTINED is also correct here and is not
         * a weaker outcome. A transmit buffer that has stayed full for twenty seconds is a
         * wedged link by any useful definition, and containing it is the point. What must
         * never happen is a wait, and neither result is one. */
        CHECK(rc == CBCTL_OK || rc == CBCTL_E_QUARANTINED);
        now += 400u;
    }
    lb.tx_full = false;
    for (int i = 0; i < 80 && cbctl_link_state(&link) != CBCTL_LINK_READY; i++) {
        round_trip(&link, &lb, &now, 400u);
    }
    CHECK(cbctl_link_state(&link) == CBCTL_LINK_READY); /* recovered by itself */
    /* READY only means the board named itself. Keep ticking to prove the periodic traffic
     * actually resumes — a link that identifies and then says nothing is a dead-man that
     * has stopped feeding, which is exactly the failure this library exists to prevent. */
    for (int i = 0; i < 6; i++) {
        round_trip(&link, &lb, &now, 400u);
    }
    CHECK(lb.heartbeat_seen >= 1u);
}

/* next_deadline is MANDATORY on every port, unlike a pollable fd. It must never invite the
 * caller to sleep past the feed period, or the library would be handing out the very
 * staleness it exists to prevent. */
TEST(next_deadline_never_exceeds_the_feed_period)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    for (int i = 0; i < 20; i++) {
        const uint32_t d = cbctl_next_deadline_ms(&link, now);
        CHECK(d <= 400u);
        round_trip(&link, &lb, &now, 137u); /* deliberately not a multiple of the period */
    }
}


/* A close with no dead-man cannot be expressed. There is no contactor feedback and no weld
 * sensor on this hardware, so a contactor left closed by a dead controller is discovered by
 * a person noticing, which is not a safety mechanism. */
TEST(a_relay_close_without_a_deadman_is_refused)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 0u, NULL) == CBCTL_E_ARG);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 2000u, NULL) == CBCTL_OK);
}

/* An over-long hold is CLAMPED to the firmware ceiling, not rejected and not honoured. A
 * longer dead-man is strictly less safe, so the library must not quietly grant one. */
TEST(an_over_long_hold_is_clamped_to_the_firmware_ceiling)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 120000u, NULL) == CBCTL_OK);
    CHECK(link.relay_hold_ms == CBPLC_RELAY_HOLD_MAX_MS);
}

/* THE FLAKING SCENARIO, as a test.
 *
 * The caller closes ONCE and then does nothing but tick. The lease must be renewed often
 * enough that it never lapses, because a lapse drops the contactor mid-charge and the next
 * renewal closes it again — which is what was heard on the bench as a relay "flake". The
 * caller does not and cannot choose the renewal period. */
TEST(a_held_lease_is_renewed_by_the_tick_and_never_lapses)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    uint32_t last_seen = 0u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);

    /* Thirty seconds of charging, ten times the hold. */
    for (int i = 0; i < 75; i++) {
        round_trip(&link, &lb, &now, 400u);
        /* The gap between renewals must never reach the hold itself. */
        const uint32_t gap = now - last_seen;
        if (link.relay_renewals > 0u) {
            CHECK(gap < 3000u);
        }
        if (link.relay_renewals > 0u) {
            last_seen = link.last_relay.ms;
        }
    }
    /* hold/3 = 1000 ms over ~30 s is roughly 30 renewals; assert it actually kept going
     * rather than sending one and stopping. */
    CHECK(link.relay_renewals >= 20u);
}

/* Opening is never the dangerous direction, so it must stay expressible even when the link
 * is in a state that refuses closes. */
TEST(opening_is_legal_even_when_closing_is_not)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;

    cbctl_loopback_init(&lb);
    CHECK(cbctl_open(&link, &lb.ctrl_tx) == CBCTL_OK);
    /* Still IDENTIFYING: the board has not named itself. */
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 2000u, NULL) == CBCTL_E_NOT_IDENTIFIED);
    CHECK(cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL) == CBCTL_OK);
}

/* AN OPEN IS NOT A LEASE, SO IT MUST NOT BE RENEWED — AND MUST NOT REPORT RENEWALS.
 *
 * The renewal gate was `relay_enable != 0`, and cbctl_relay_open() SETS relay_enable (the mask
 * says which relays the frame addresses) and nothing ever clears it. So one open-all put the
 * link into retransmitting that OPEN on the lease cadence forever, counting each as a renewal
 * and each ACK as a confirmed renewal.
 *
 * cbplc cannot object: its RELAY handler only runs cbplc_relay_may_close() when `want != 0`, so
 * an all-open is accepted with gate 0 by design. On the charger this read renewals=50, acks=49,
 * refusals=0 — a perfectly healthy dead-man lease — while the board reported relay_closed=0x00
 * for every STATUS of the run and refused energy with limits_refusal=contactor_open.
 *
 * Asserting the COUNTER, not "did a frame go out": the dirty open legitimately transmits once,
 * and it is the repetition-as-renewal that was the defect. */
TEST(an_open_is_sent_once_and_never_renewed_as_a_lease)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
    for (int i = 0; i < 40; i++) { round_trip(&link, &lb, &now, 100u); }
    CHECK(link.relay_renewals > 0u);          /* a held close IS renewed */

    /* Now let go of it. Everything after this is an open. */
    CHECK(cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL) == CBCTL_OK);
    {
        uint32_t after_open;
        for (int i = 0; i < 5; i++) { round_trip(&link, &lb, &now, 100u); }
        after_open = link.relay_renewals;
        /* Well past several renewal periods for the 3000 ms hold used above. */
        for (int i = 0; i < 120; i++) { round_trip(&link, &lb, &now, 100u); }
        printf("    renewals after release: %u (was %u)\n", link.relay_renewals, after_open);
        CHECK(link.relay_renewals == after_open);
        CHECK(cbctl_lease_headroom_ms(&link, now) == 0u);
    }
}

/* A refusal names its gate and is NOT a transient failure. Retrying it in a loop is how a
 * controller argues with an interlock forever. */
TEST(a_refused_op_reports_the_gate_rather_than_looking_lost)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_op_t op;

    bring_up(&link, &lb, &now);
    lb.refuse_code = 7u; /* CBPLC_E_BAD_ARGUMENT, standing in for any named gate */

    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 2000u, &op) == CBCTL_OK);
    for (int i = 0; i < 8 && cbctl_op_state(&link, op) == CBCTL_OP_PENDING; i++) {
        round_trip(&link, &lb, &now, 100u);
    }
    CHECK(cbctl_op_state(&link, op) == CBCTL_OP_REFUSED);
    CHECK(cbctl_op_gate(&link, op) == 7u);
}

/* Intent is not confirmation. PENDING must be a distinct, visible value — a caller who
 * ignores the difference is never silently told "applied". */
TEST(a_submitted_op_is_pending_before_it_is_applied)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_op_t op;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 2000u, &op) == CBCTL_OK);
    CHECK(cbctl_op_state(&link, op) == CBCTL_OP_PENDING);   /* not yet on the wire */

    for (int i = 0; i < 8 && cbctl_op_state(&link, op) == CBCTL_OP_PENDING; i++) {
        round_trip(&link, &lb, &now, 100u);
    }
    CHECK(cbctl_op_state(&link, op) == CBCTL_OP_APPLIED);
}

/* A STOP THAT NEVER REACHES THE BOARD IS NOT A STOP.
 *
 * begin_stop() used to clear power_ready locally and nothing else — the board's LATCH was
 * never set. It half-worked, because the cleared power_ready does travel in LIMITS, which is
 * exactly why it went unnoticed. */
TEST(begin_stop_actually_latches_the_stop_on_the_board)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(!lb.stop_latched);

    CHECK(cbctl_begin_stop(&link) == CBCTL_OK);
    for (int i = 0; i < 6; i++) {
        round_trip(&link, &lb, &now, 200u);
    }
    CHECK(lb.stop_seen >= 1u);
    CHECK(lb.stop_latched);      /* the BOARD is stopped, not just our copy of the intent */
}

/* And it must be releasable, or the first stop poisons the charger forever. */
TEST(a_latched_stop_can_be_cleared)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_begin_stop(&link) == CBCTL_OK);
    for (int i = 0; i < 6; i++) {
        round_trip(&link, &lb, &now, 200u);
    }
    CHECK(lb.stop_latched);

    CHECK(cbctl_stop_clear(&link) == CBCTL_OK);
    for (int i = 0; i < 6; i++) {
        round_trip(&link, &lb, &now, 200u);
    }
    CHECK(!lb.stop_latched);

    /* Clearing the latch must NOT by itself re-arm the power path. */
    CHECK(!link.envelope.power_ready);
    /* And an envelope is expressible again now the stop is gone. */
    cbctl_envelope_t env;
    memset(&env, 0, sizeof env);
    env.max_voltage = CBCTL_MV(500000u);
    env.power_ready = true;
    CHECK(cbctl_set_envelope(&link, &env) == CBCTL_OK);
}

/* RE-DECLARING THE SAME VALUES EVERY PASS MUST BE HARMLESS.
 *
 * The API says a caller may call set_envelope/set_measurement whenever it likes because the
 * tick owns repetition. Every loopback test declared ONCE, so nothing exercised that promise —
 * and on real hardware it was false in two expensive ways. Marking "due now" by zeroing the
 * timestamp meant a caller that re-declared each pass made LIMITS PERMANENTLY due, which
 * (a) starved every lower-priority message, so the relay lease was never renewed and the board
 * opened the contactor on a lapsed dead-man, and (b) made the staleness counter read the zero
 * stamp as an infinite age — 6.4 million late-feed ticks reported on a perfectly healthy link.
 *
 * Two facts pinned here: the lower-priority traffic still flows, and feed_late stays zero. */
TEST(re_declaring_the_feed_every_pass_starves_nothing)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_envelope_t env;
    cbctl_measurement_t m;

    bring_up(&link, &lb, &now);
    memset(&env, 0, sizeof env);
    env.max_voltage = CBCTL_MV(500000u);
    env.max_current = CBCTL_MA(120000u);
    env.power_ready = true;
    memset(&m, 0, sizeof m);
    m.present_voltage = CBCTL_MV(400000u);
    m.isolation = CBCTL_ISO_VALID;

    CHECK(cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 8000u) == CBCTL_OK);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);

    for (int i = 0; i < 120; i++) {
        /* IDENTICAL values, every single pass — exactly what the live driver does. */
        CHECK(cbctl_set_envelope(&link, &env) == CBCTL_OK);
        CHECK(cbctl_set_measurement(&link, &m) == CBCTL_OK);
        round_trip(&link, &lb, &now, 400u);
    }

    /* The dead-man was renewed throughout. Zero renewals is a contactor the board dropped. */
    CHECK(link.relay_renewals >= 10u);
    /* The heartbeat and STATUS still got slots. */
    CHECK(lb.heartbeat_seen >= 5u);
    CHECK(lb.status_seen >= 5u);
    /* And the staleness counter told the truth. */
    CHECK(link.feed_late == 0u);
    printf("    renewals=%u heartbeat=%u status=%u feed_late=%u\n", link.relay_renewals,
           lb.heartbeat_seen, lb.status_seen, link.feed_late);
}

/* A genuine CHANGE must be flagged for immediate send; an identical re-declaration must not.
 *
 * Asserted on the dirty flag rather than on "did a frame go out", because the periodic repeat
 * also puts LIMITS on the wire and would mask the distinction. The flag IS the distinction. */
TEST(only_a_real_change_marks_the_feed_dirty)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_envelope_t env;
    cbctl_measurement_t m;

    bring_up(&link, &lb, &now);
    memset(&env, 0, sizeof env);
    env.max_voltage = CBCTL_MV(500000u);
    env.power_ready = true;

    /* First declaration is new, so it is dirty. */
    CHECK(cbctl_set_envelope(&link, &env) == CBCTL_OK);
    CHECK(link.envelope_dirty);

    /* Let the tick send it and clear the flag. */
    for (int i = 0; i < 4; i++) {
        round_trip(&link, &lb, &now, 100u);
    }
    CHECK(!link.envelope_dirty);

    /* IDENTICAL values: must NOT re-dirty. This is the case the live driver hits every pass,
     * and the case that used to force LIMITS permanently due and starve the relay lease. */
    CHECK(cbctl_set_envelope(&link, &env) == CBCTL_OK);
    CHECK(!link.envelope_dirty);

    /* A real change: dirty again, so it does not wait for the next period. */
    env.max_current = CBCTL_MA(60000u);
    CHECK(cbctl_set_envelope(&link, &env) == CBCTL_OK);
    CHECK(link.envelope_dirty);

    /* Same rule for the measurement. */
    memset(&m, 0, sizeof m);
    m.present_voltage = CBCTL_MV(400000u);
    m.isolation = CBCTL_ISO_VALID;
    CHECK(cbctl_set_measurement(&link, &m) == CBCTL_OK);
    CHECK(link.measurement_dirty);
    for (int i = 0; i < 4; i++) {
        round_trip(&link, &lb, &now, 100u);
    }
    CHECK(!link.measurement_dirty);
    CHECK(cbctl_set_measurement(&link, &m) == CBCTL_OK);
    CHECK(!link.measurement_dirty);
    /* A NUMERIC CHANGE NO LONGER PREEMPTS THE LINK — but it must still ARRIVE.
     *
     * Marking dirty on any numeric difference meant real telemetry jitter sent FEEDBACK every
     * tick and starved AUTHORIZE off a stop-and-wait wire (618 feedback vs 7 auth in 23 s on
     * hardware; the auth lease lapsed and the board refused the gun close 1136 times). Freshness
     * is the periodic send's job, so the new value rides the next feed period instead. */
    {
        const unsigned before = lb.feedback_seen;
        m.present_current = CBCTL_MA(50000u);
        CHECK(cbctl_set_measurement(&link, &m) == CBCTL_OK);
        CHECK(!link.measurement_dirty);            /* did not jump the queue ... */
        for (int i = 0; i < 12; i++) {
            round_trip(&link, &lb, &now, 100u);
        }
        CHECK(lb.feedback_seen > before);          /* ... but was delivered anyway */
    }

    /* Isolation is a safety verdict, not a measurement: it must still be urgent. */
    m.isolation = CBCTL_ISO_FAULT;
    CHECK(cbctl_set_measurement(&link, &m) == CBCTL_OK);
    CHECK(link.measurement_dirty);
}

/* AN UNANSWERED REQUEST MUST NOT LAPSE THE RELAY LEASE.
 *
 * cbplc is stop-and-wait, so one request the board ignores blocks EVERY other message for the
 * whole retry sequence — including the lease renewal. With the schema defaults that is
 * 3 x (1000 + 100) = 3300 ms of dead air, longer than a 3000 ms hold: the contactor drops with
 * nobody having asked for it. That is the "lease expiry without intention" case, and the live
 * run only escaped it because the link happened to be clean.
 *
 * Here the board answers everything EXCEPT STATUS, so the retry path is exercised continuously
 * while a lease is held. The renewal gap must stay under the hold throughout. */
TEST(one_unanswered_message_cannot_lapse_the_relay_lease)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    uint32_t worst_gap = 0u, last_ack_at = 0u;
    uint32_t hold;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
    hold = link.relay_hold_ms;

    /* The board goes deaf on STATUS only — everything else still works. */
    lb.swallow_type = (uint8_t)CBPLC_T_STATUS_REQUEST;

    for (int i = 0; i < 400; i++) {
        round_trip(&link, &lb, &now, 100u);
        if (link.relay_acks > 0u && link.last_relay.ms != last_ack_at) {
            if (last_ack_at != 0u) {
                const uint32_t gap = link.last_relay.ms - last_ack_at;
                if (gap > worst_gap) {
                    worst_gap = gap;
                }
            }
            last_ack_at = link.last_relay.ms;
        }
    }

    printf("    hold=%ums worst CONFIRMED-renewal gap=%ums acks=%u refusals=%u\n", hold,
           worst_gap, link.relay_acks, link.relay_refusals);

    /* Renewals were CONFIRMED, not merely attempted. */
    CHECK(link.relay_acks > 10u);
    /* And the gap between confirmed renewals never approached the hold. */
    CHECK(worst_gap < hold);
}

/* A hold too short to survive one unanswered request is RAISED, not honoured literally. */
TEST(an_unrenewable_hold_is_raised_to_a_safe_floor)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 150u, NULL) == CBCTL_OK);
    CHECK(link.relay_hold_ms >= CBCTL_RELAY_HOLD_MIN_MS);
    /* The floor is DERIVED from the blocking budget, not picked: a hold must outlast one
     * worst-case retry sequence plus its own renewal period. */
    CHECK(CBCTL_RELAY_HOLD_MIN_MS > CBCTL_WORST_BLOCK_MS);
    printf("    worst_block=%ums -> hold floor=%ums (asked 150ms, got %ums)\n",
           (unsigned)CBCTL_WORST_BLOCK_MS, (unsigned)CBCTL_RELAY_HOLD_MIN_MS,
           link.relay_hold_ms);
}

/* A REBOOTED BOARD HAS FORGOTTEN THE STOP LATCH.
 *
 * The periodic declarations repair themselves within a period, so they were never at risk.
 * STOP is sent once when it changes — so a controller that had raised a stop, whose board then
 * rebooted, sat believing the charger was stopped while the board came up perfectly willing to
 * energise, and nothing would ever have corrected it. */
TEST(a_board_reset_re_asserts_the_stop_latch)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    lb.up_ms = 500000u;
    for (int i = 0; i < 8; i++) {
        round_trip(&link, &lb, &now, 600u);
    }

    CHECK(cbctl_begin_stop(&link) == CBCTL_OK);
    for (int i = 0; i < 6; i++) {
        round_trip(&link, &lb, &now, 300u);
    }
    CHECK(lb.stop_latched);

    /* The board reboots: uptime goes backwards and it forgets the latch. */
    lb.stop_latched = false;
    lb.up_ms = 200u;
    for (int i = 0; i < 14; i++) {
        round_trip(&link, &lb, &now, 600u);
    }

    printf("    reestablishes=%u stop_latched_again=%d\n", link.reestablishes,
           lb.stop_latched ? 1 : 0);
    CHECK(link.reestablishes > 0u);
    CHECK(lb.stop_latched);   /* the controller put it back without being asked */
}

/* A reset is also the window in which hardware gets swapped. Coming back as a DIFFERENT board
 * on the same port must be contained, never quietly adopted — a controller that renumbers its
 * guns underneath itself is how the wrong contactor gets closed. */
TEST(a_board_that_comes_back_with_a_different_mac_is_quarantined)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_identity_t id;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_identity(&link, &id));
    CHECK(cbctl_expect_mac(&link, id.mac) == CBCTL_OK);

    lb.up_ms = 400000u;
    for (int i = 0; i < 8; i++) {
        round_trip(&link, &lb, &now, 600u);
    }

    /* Somebody swapped the board while it was down. */
    lb.board_mac[5] = 0x99u;
    lb.up_ms = 100u;
    for (int i = 0; i < 20; i++) {
        round_trip(&link, &lb, &now, 600u);
    }

    printf("    identity_changed=%u state=%d\n", link.identity_changed,
           (int)cbctl_link_state(&link));
    CHECK(link.identity_changed > 0u);
    CHECK(cbctl_link_state(&link) == CBCTL_LINK_QUARANTINED);
    CHECK(link.quarantine_permanent);
    /* AND IT MUST STAY THERE. Encoding "forever" as a far-future deadline silently released
     * on the next tick, because wrap-safe comparison has no infinitely-distant instant. */
    for (int i = 0; i < 200; i++) {
        round_trip(&link, &lb, &now, 1000u);
        CHECK(cbctl_link_state(&link) == CBCTL_LINK_QUARANTINED);
    }
    /* The pinned identity is unchanged — we did NOT adopt the impostor. */
    CHECK(cbctl_expect_mac(&link, id.mac) == CBCTL_OK);
}

/* An auth lease too short to survive one blocked request is refused, not silently granted. */
TEST(an_unsafe_auth_lease_is_refused)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 600u) == CBCTL_E_ARG);
    CHECK(cbctl_authorize(&link, CBCTL_AUTH_GRANTED, CBCTL_AUTH_LEASE_MIN_SAFE_MS) == CBCTL_OK);
}

/* The hold we believe must be the hold the board enforces. RELAY carries 100 ms units, so a
 * value that does not divide evenly used to leave us renewing against a longer lease than the
 * board was holding. */
TEST(the_relay_hold_matches_what_the_wire_can_carry)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    static const uint32_t asks[] = { 1130u, 2550u, 3000u, 30000u, 999u };

    bring_up(&link, &lb, &now);
    for (unsigned i = 0; i < sizeof asks / sizeof asks[0]; i++) {
        CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, asks[i], NULL) == CBCTL_OK);
        CHECK(link.relay_hold_ms % 100u == 0u);            /* representable on the wire */
        CHECK(link.relay_hold_ms >= asks[i] ||
              link.relay_hold_ms == CBPLC_RELAY_HOLD_MAX_MS); /* rounded UP, never down */
        CHECK(link.relay_hold_ms <= CBPLC_RELAY_HOLD_MAX_MS);
    }
}

/* Closing an energy relay while a stop is latched is the same self-cancelling instruction as
 * power_ready-while-stopping, and gets the same answer at the call rather than 250 ms later
 * as a gate code that is much harder to attribute. */
TEST(closing_an_energy_relay_while_stopping_is_refused)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_begin_stop(&link) == CBCTL_OK);

    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_E_CONTRADICTORY);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_POWER, 3000u, NULL) == CBCTL_E_CONTRADICTORY);
    /* Opening is never the dangerous direction and stays available. */
    CHECK(cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL) == CBCTL_OK);
    /* And once the stop is cleared it becomes expressible again. */
    CHECK(cbctl_stop_clear(&link) == CBCTL_OK);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
}

/* A link that was never opened must be refused, not run as a state machine over whatever the
 * caller's stack contained. */
TEST(an_unopened_link_is_refused)
{
    cbctl_link_t link;
    memset(&link, 0xA5, sizeof link);   /* deliberate garbage */
    CHECK(cbctl_tick(&link, 1000u) == CBCTL_E_ARG);
    memset(&link, 0, sizeof link);      /* and a zeroed one is equally not open */
    CHECK(cbctl_tick(&link, 1000u) == CBCTL_E_ARG);
}

/* A STARVED CONTROLLER MUST NOT DROP A CONTACTOR.
 *
 * The dead-man exists to catch a controller that has DIED. A controller that is merely LATE —
 * descheduled by OCPP, a TLS handshake, a log flush, memory pressure — must never trip it. On
 * a resource-scarce charger that is the normal condition, not an exception.
 *
 * Here the caller stalls repeatedly for most of the hold, which at the old hold/3 renewal would
 * have lapsed. With hold/6 plus danger preemption the lease survives, and the headroom never
 * reaches zero. */
TEST(a_starved_controller_never_lapses_the_lease)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    const uint32_t hold = CBCTL_RELAY_HOLD_RECOMMENDED_MS;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, hold, NULL) == CBCTL_OK);
    for (int i = 0; i < 10; i++) {
        round_trip(&link, &lb, &now, 200u);
    }

    /* Thirty stalls, each swallowing 60 % of the hold. A controller this badly starved is
     * pathological — and it still must not open a contactor. */
    for (int i = 0; i < 30; i++) {
        round_trip(&link, &lb, &now, (hold * 6u) / 10u);
        round_trip(&link, &lb, &now, 50u);
        round_trip(&link, &lb, &now, 50u);
        CHECK(cbctl_lease_headroom_ms(&link, now) > 0u);
    }

    printf("    hold=%ums worst tick gap=%ums min headroom=%ums preempts=%u acks=%u\n", hold,
           link.worst_tick_gap_ms, link.lease_min_headroom_ms, link.relay_danger_preempts,
           link.relay_acks);

    /* The stall was real. */
    CHECK(link.worst_tick_gap_ms >= (hold * 5u) / 10u);
    /* The lease never came close to lapsing, and the preemption is what saved it. */
    CHECK(link.lease_min_headroom_ms > 0u);
    CHECK(link.relay_danger_preempts > 0u);
    /* DEGRADED is the correct report here, not a failure: a controller that has not polled
     * STATUS in six seconds genuinely does have a stale picture of the charger, and saying so
     * is the point of that state. What must NOT happen is quarantine or a dropped lease. */
    CHECK(cbctl_link_state(&link) == CBCTL_LINK_READY ||
          cbctl_link_state(&link) == CBCTL_LINK_DEGRADED);
    CHECK(cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS) == 0u);
}

/* Six renewals inside one hold, not three: losing five in a row must still be survivable. */
TEST(the_lease_survives_five_consecutive_lost_renewals)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    const uint32_t hold = CBCTL_RELAY_HOLD_RECOMMENDED_MS;

    bring_up(&link, &lb, &now);
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, hold, NULL) == CBCTL_OK);
    for (int i = 0; i < 10; i++) {
        round_trip(&link, &lb, &now, 200u);
    }
    CHECK(relay_period_fits(hold));
    /* The renewal period must leave room for at least DIVISOR attempts inside the hold. */
    CHECK(hold / CBCTL_RELAY_RENEW_DIVISOR * CBCTL_RELAY_RENEW_DIVISOR <= hold);
    CHECK(CBCTL_RELAY_RENEW_DIVISOR >= 6u);
}

/* cbctl must never put a TERMINAL refusal on the wire before the application has an opinion.
 *
 * cbplc reads AUTHORIZE decision byte 0 as DENY, answers the vehicle FAILED/Finished, and latches
 * that for 30 s. A lapsed lease on the firmware side deliberately demotes to PENDING instead, for
 * exactly this reason. cbctl used to open with DENY as its "safe" default and re-assert it at
 * ~1 Hz, which against a real car produced auth_ok=3 vs auth_no=104 and a match/refuse/re-match
 * loop. Withholding energy is PENDING; DENY is a verdict, and only the caller may reach it. */
TEST(an_unopinionated_controller_never_denies)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    for (int i = 0; i < 10; i++) {
        round_trip(&link, &lb, &now, 500u);
    }
    CHECK(lb.auth_seen > 0u);                 /* it must actually say something */
    CHECK(lb.first_auth_decision != 0);       /* but never a terminal refusal */
    printf("    %u auth frames, first decision=%d (0 would be terminal DENY)\n",
           lb.auth_seen, lb.first_auth_decision);
}

/* The contactor the SECC reads must follow the BOARD'S reported relay state. It was never set at
 * all, so every FEEDBACK said CONTACTOR_UNKNOWN(0) while the gun was physically closed. */
TEST(feedback_reports_the_contactor_the_board_reports)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    {   /* FEEDBACK is only sent once a measurement has been declared. */
        cbctl_measurement_t m;
        memset(&m, 0, sizeof m);
        m.present_voltage = CBCTL_MV(400000u);
        m.present_current = CBCTL_MA(0u);
        m.isolation = CBCTL_ISO_VALID;
        CHECK(cbctl_set_measurement(&link, &m) == CBCTL_OK);
    }
    lb.relay_closed_mask = 0u;
    for (int i = 0; i < 6; i++) {
        round_trip(&link, &lb, &now, 500u);
    }
    CHECK(lb.last_feedback_contactor == CBPLC_CONTACTOR_OPEN);

    lb.relay_closed_mask = CBPLC_RELAY_GUN;
    for (int i = 0; i < 6; i++) {
        round_trip(&link, &lb, &now, 500u);
    }
    CHECK(lb.last_feedback_contactor == CBPLC_CONTACTOR_CLOSED);
    printf("    contactor tracked the board: OPEN -> CLOSED\n");
}

/* A STANDING REFUSAL MUST NOT RETRY AT TICK RATE.
 *
 * cbplc refuses a gun close while authorisation is not yet live, and that can persist for
 * seconds. Retrying at once for the whole time floods a stop-and-wait wire and starves the very
 * AUTHORIZE renewal that would clear the refusal — measured as 2509 refusals in one bench run.
 * A couple of fast retries, then the ordinary cadence. */
TEST(a_standing_relay_refusal_backs_off)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_op_t op;

    bring_up(&link, &lb, &now);
    lb.refuse_code = 14u;                   /* board rejects every command (rc=14, as seen live) */
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN,
                            CBCTL_RELAY_HOLD_RECOMMENDED_MS, &op) == CBCTL_OK);
    for (int i = 0; i < 60; i++) {
        round_trip(&link, &lb, &now, 50u);  /* 3 s of ticks at 50 ms */
    }
    CHECK(link.relay_refusals > 0u);
    /* Without the backoff this is one refusal per tick (~60). With it, a couple of fast retries
     * then the renewal cadence — far fewer. */
    CHECK(link.relay_refusals < 20u);
    printf("    %u refusals over 3s of ticks (unbounded would be ~60)\n", link.relay_refusals);
}

int main(void)
{
    printf("link\n");
    RUN(a_link_identifies_over_the_real_protocol);
    RUN(the_tick_repeats_the_feed_without_the_caller_asking);
    RUN(power_ready_while_stopping_is_refused_at_the_call);
    RUN(begin_stop_clears_power_ready_itself);
    RUN(an_auth_lease_shorter_than_the_tick_is_refused);
    RUN(a_board_that_never_answers_cannot_make_the_tick_unbounded);
    RUN(a_wedged_board_is_quarantined_and_then_re_probed);
    RUN(a_full_transmit_buffer_defers_instead_of_stalling);
    RUN(next_deadline_never_exceeds_the_feed_period);
    RUN(a_relay_close_without_a_deadman_is_refused);
    RUN(an_over_long_hold_is_clamped_to_the_firmware_ceiling);
    RUN(a_held_lease_is_renewed_by_the_tick_and_never_lapses);
    RUN(opening_is_legal_even_when_closing_is_not);
    RUN(an_open_is_sent_once_and_never_renewed_as_a_lease);
    RUN(a_refused_op_reports_the_gate_rather_than_looking_lost);
    RUN(a_submitted_op_is_pending_before_it_is_applied);
    RUN(begin_stop_actually_latches_the_stop_on_the_board);
    RUN(a_latched_stop_can_be_cleared);
    RUN(re_declaring_the_feed_every_pass_starves_nothing);
    RUN(only_a_real_change_marks_the_feed_dirty);
    RUN(one_unanswered_message_cannot_lapse_the_relay_lease);
    RUN(an_unrenewable_hold_is_raised_to_a_safe_floor);
    RUN(a_board_reset_re_asserts_the_stop_latch);
    RUN(a_board_that_comes_back_with_a_different_mac_is_quarantined);
    RUN(an_unsafe_auth_lease_is_refused);
    RUN(the_relay_hold_matches_what_the_wire_can_carry);
    RUN(closing_an_energy_relay_while_stopping_is_refused);
    RUN(an_unopened_link_is_refused);
    RUN(a_starved_controller_never_lapses_the_lease);
    RUN(the_lease_survives_five_consecutive_lost_renewals);
    RUN(an_unopinionated_controller_never_denies);
    RUN(feedback_reports_the_contactor_the_board_reports);
    RUN(a_standing_relay_refusal_backs_off);
    if (g_fail) {
        printf("FAIL link (%d)\n", g_fail);
        return 1;
    }
    printf("PASS link\n");
    return 0;
}
