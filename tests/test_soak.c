/* Long-run behaviour, including the one moment you cannot wait for on hardware.
 *
 * `now_ms` is a uint32 of milliseconds, so it wraps every 49.7 days. A charger runs for
 * months. This project has already shipped a defect where an age computed across that
 * boundary read the freshest possible feed as expired and killed every live session while
 * every idle test passed — so the wrap is not an edge case here, it is the case.
 *
 * You cannot test it on a bench: you would wait seven weeks. In simulation it costs a second,
 * which is the entire argument for a virtual clock.
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

typedef struct {
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now;
} rig_t;

static void round_trip(rig_t *r, uint32_t step)
{
    cbctl_tick(&r->link, r->now);
    cbctl_loopback_poll(&r->lb, r->now);
    cbctl_tick(&r->link, r->now);
    cbctl_loopback_poll(&r->lb, r->now);
    r->now += step;
}

static void bring_up(rig_t *r, uint32_t start)
{
    cbctl_envelope_t env;
    cbctl_measurement_t m;
    int i;

    memset(r, 0, sizeof *r);
    r->now = start;
    cbctl_loopback_init(&r->lb);
    r->lb.cp = 'B';
    CHECK(cbctl_open(&r->link, &r->lb.ctrl_tx) == CBCTL_OK);
    for (i = 0; i < 12 && cbctl_link_state(&r->link) != CBCTL_LINK_READY; i++) {
        round_trip(r, 10u);
    }
    CHECK(cbctl_link_state(&r->link) == CBCTL_LINK_READY);

    memset(&env, 0, sizeof env);
    env.max_voltage = CBCTL_MV(500000u);
    env.max_current = CBCTL_MA(200000u);
    env.max_power = CBCTL_W(100000u);
    env.power_ready = true;
    CHECK(cbctl_set_envelope(&r->link, &env) == CBCTL_OK);

    memset(&m, 0, sizeof m);
    m.present_voltage = CBCTL_MV(400000u);
    m.present_current = CBCTL_MA(100000u);
    m.isolation = CBCTL_ISO_VALID;
    CHECK(cbctl_set_measurement(&r->link, &m) == CBCTL_OK);

    CHECK(cbctl_authorize(&r->link, CBCTL_AUTH_GRANTED, 8000u) == CBCTL_OK);
    CHECK(cbctl_relay_close(&r->link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
}

/* THE WRAP.
 *
 * Start ~30 s before the uint32 millisecond rollover and charge straight through it. The feed
 * must never go stale and the relay lease must keep being renewed — a lapse here would open a
 * contactor mid-charge on a charger that has simply been powered on for seven weeks. */
TEST(a_charge_survives_the_49_day_millisecond_wrap)
{
    static rig_t r;
    uint32_t worst_feed = 0u;
    uint32_t renewals_before, renewals_after;
    int i;
    bool crossed = false;

    bring_up(&r, 0xFFFFFFFFu - 30000u);
    for (i = 0; i < 20; i++) {
        round_trip(&r, 400u);
    }
    renewals_before = r.link.relay_renewals;

    /* 60 s of charging straddling the boundary. */
    for (i = 0; i < 150; i++) {
        const uint32_t prev = r.now;
        const uint32_t age = cbctl_feed_age_ms(&r.link, r.now);
        if (age > worst_feed) {
            worst_feed = age;
        }
        round_trip(&r, 400u);
        if (r.now < prev) {
            crossed = true; /* the clock rolled over during this pass */
        }
    }
    renewals_after = r.link.relay_renewals;

    printf("    crossed=%d worst feed age across the wrap: %u ms (bound %u)\n",
           crossed ? 1 : 0, worst_feed, (unsigned)CBPLC_LIMITS_STALE_MS);

    CHECK(crossed);                                   /* the test actually reached the wrap */
    CHECK(worst_feed < CBPLC_LIMITS_STALE_MS);        /* no false staleness */
    CHECK(cbctl_link_state(&r.link) == CBCTL_LINK_READY);
    CHECK(renewals_after > renewals_before + 10u);    /* the contactor was held throughout */
}

/* A long, ordinary run. Nothing exotic — the point is that nothing DRIFTS: no counter creeps,
 * no state machine wanders, no lease slowly loses its margin over thousands of cycles. */
TEST(a_long_run_holds_the_feed_and_the_lease)
{
    static rig_t r;
    uint32_t worst_feed = 0u;
    uint32_t worst_gap = 0u;
    uint32_t last_renewal_at;
    int i;

    bring_up(&r, 1000u);
    for (i = 0; i < 20; i++) {
        round_trip(&r, 400u);
    }
    last_renewal_at = r.link.last_relay.ms;

    /* ~40 minutes of simulated charging at the real cadence. */
    for (i = 0; i < 6000; i++) {
        const uint32_t age = cbctl_feed_age_ms(&r.link, r.now);
        if (age > worst_feed) {
            worst_feed = age;
        }
        round_trip(&r, 400u);
        if (r.link.last_relay.ms != last_renewal_at) {
            const uint32_t gap = r.link.last_relay.ms - last_renewal_at;
            if (gap > worst_gap) {
                worst_gap = gap;
            }
            last_renewal_at = r.link.last_relay.ms;
        }
    }

    printf("    40 min: worst feed age %u ms, worst lease gap %u ms (hold 3000)\n",
           worst_feed, worst_gap);

    CHECK(worst_feed < CBPLC_LIMITS_STALE_MS);
    /* THE FLAKING BOUND. The gap between renewals must never approach the hold, or the
     * contactor drops and the next renewal closes it again — which is what was heard on the
     * bench. hold/3 should keep this near 1000 ms. */
    CHECK(worst_gap < 3000u);
    CHECK(cbctl_link_state(&r.link) == CBCTL_LINK_READY);
    CHECK(r.link.feed_late == 0u);
    /* Nothing pathological accumulated: no event backlog, no capped ticks with work stuck. */
    CHECK(r.link.ev_dropped == 0u);
}

/* Plug/unplug thousands of times. Session churn is where state machines leak. */
TEST(repeated_plug_cycles_do_not_accumulate_state)
{
    static rig_t r;
    int i, c;

    bring_up(&r, 1000u);
    for (c = 0; c < 200; c++) {
        r.lb.cp = 'B';
        r.lb.slac_matched = true;
        r.lb.secc_active = true;
        r.lb.secc_streams++;
        for (i = 0; i < 4; i++) {
            round_trip(&r, 400u);
        }
        r.lb.cp = 'A';
        r.lb.slac_matched = false;
        r.lb.secc_active = false;
        for (i = 0; i < 4; i++) {
            round_trip(&r, 400u);
        }
        /* Drain events so the ring is not the thing being tested. */
        {
            cbctl_event_t ev;
            while (cbctl_next_event(&r.link, &ev)) {
            }
        }
    }
    CHECK(cbctl_link_state(&r.link) == CBCTL_LINK_READY);
    CHECK(r.link.feed_late == 0u);
    CHECK(r.lb.to_plc.dropped == 0u);
    CHECK(r.lb.to_ctrl.dropped == 0u);
    printf("    200 plug cycles: sessions seen %u, feed_late %u, status promotions %u\n",
           cbctl_counter(&r.link, CBCTL_CTR_SESSIONS), r.link.feed_late,
           r.link.status_promotions);

    /* THE STARVATION REGRESSION.
     *
     * This assertion is why the soak exists. Under a full charging load the high-priority
     * traffic (LIMITS and FEEDBACK due every tick, plus heartbeat, relay lease and auth)
     * consumed the tick's bounded poll budget and STATUS — lowest priority — ran once in
     * 24 s. Everything still "worked": the feed was perfect, the lease held, the tests were
     * green. But the controller was BLIND: no plug edges, no SLAC-match vehicle MAC for
     * Autocharge, no relay diagnostics, for the entire duration of a charge.
     *
     * The board's session count moved 200 times here. If the controller cannot see that, it
     * cannot see anything. */
    CHECK(cbctl_counter(&r.link, CBCTL_CTR_SESSIONS) > 0u);
}

int main(void)
{
    printf("soak\n");
    RUN(a_charge_survives_the_49_day_millisecond_wrap);
    RUN(a_long_run_holds_the_feed_and_the_lease);
    RUN(repeated_plug_cycles_do_not_accumulate_state);
    if (g_fail) {
        printf("FAIL soak (%d)\n", g_fail);
        return 1;
    }
    printf("PASS soak\n");
    return 0;
}
