/* Measure what N simultaneously-CHARGING links actually cost, on the machine that will pay it.
 *
 * WHY THIS EXISTS
 *   Every scaling number in docs/04-failure-modes.md stops at 2 links, because the bench has
 *   two boards. The 4-8 question was answered by linear extrapolation, and an extrapolated
 *   safety margin is a guess wearing a number. The loopback removes the hardware limit: it runs
 *   cbplc's REAL session/framing/CRC code on the far side of a byte queue, so everything except
 *   USB latency is the true cost — and USB latency is per-link parallel, which is precisely the
 *   part that does not aggregate.
 *
 * WHAT ONE RUN MEASURES, per N in {1,2,4,8}:
 *   - the wall-clock cost of cbctl_fleet_tick() (p50/p90/p99/max) with every link holding a
 *     closed relay, a granted lease and a live feed — the dual-gun steady state, times four
 *   - the realized worst feed age, confirmed-renewal gap and AUTHORIZE gap on link 0, which is
 *     the caller-visible answer to "did adding links cost the first gun anything"
 *   - reaction time under full load: cbctl_relay_close() intent -> board-confirmed ACK, and a
 *     STATUS-carried edge (SoC change) -> caller-visible, both measured on the LAST link so the
 *     round-robin position is the unfavourable one
 *   - a correlated storm: half the boards wedge at once (the shared-hub-reset shape), and the
 *     survivors' numbers are tracked separately from that instant
 *
 *   ./fleet_bench [seconds-per-N]     (default 20)
 */
#include "cbctl/cbctl.h"
#include "loopback.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAXN 8u

static uint64_t now_ns(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static uint32_t now_ms(void) { return (uint32_t)(now_ns() / 1000000ull); }

static int cmp_u64(const void *a, const void *b)
{
    const uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

/* One sample per pass; 20 s at a free-running loopback is bounded by this, not by time. */
#define MAX_SAMPLES 2000000u
static uint64_t g_samples[MAX_SAMPLES];

static void run_n(unsigned n, uint32_t run_s)
{
    static cbctl_link_t link[MAXN];
    static cbctl_loopback_t lb[MAXN];
    cbctl_fleet_t fleet;
    unsigned i, ns = 0u;
    uint32_t t0, t;
    uint32_t worst_feed = 0u, worst_renew_gap = 0u, worst_auth_gap = 0u;
    uint32_t last_ack_at = 0u, last_auth_at = 0u;
    uint32_t storm_at, worst_feed_after_storm = 0u;
    int stormed = 0;
    uint32_t react_close_ms = 0u, react_status_ms = 0u;
    int react_phase = 0;
    uint32_t react_started = 0u, react_soc_target = 0u;
    uint32_t auth_sent_mark = 0u;

    cbctl_fleet_init(&fleet);
    for (i = 0; i < n; i++) {
        cbctl_loopback_init(&lb[i]);
        lb[i].board_mac[5] = (uint8_t)(0x40 + i); /* distinct boards, or the guard quarantines */
        if (cbctl_open(&link[i], &lb[i].ctrl_tx) != CBCTL_OK ||
            cbctl_fleet_add(&fleet, &link[i]) != CBCTL_OK) {
            fprintf(stderr, "setup failed at link %u\n", i);
            exit(2);
        }
    }

    /* Bring-up, then put EVERY link in the charging steady state. */
    t0 = now_ms();
    while ((now_ms() - t0) < 1500u) {
        t = now_ms();
        (void)cbctl_fleet_tick(&fleet, t);
        for (i = 0; i < n; i++) { cbctl_loopback_poll(&lb[i], t); }
    }
    for (i = 0; i < n; i++) {
        cbctl_envelope_t env;
        cbctl_measurement_t m;
        if (cbctl_link_state(&link[i]) != CBCTL_LINK_READY) {
            fprintf(stderr, "link %u not READY after bring-up\n", i);
            exit(2);
        }
        memset(&env, 0, sizeof env);
        env.max_voltage = CBCTL_MV(500000u);
        env.max_current = CBCTL_MA(200000u);
        env.max_power = CBCTL_W(100000u);
        env.power_ready = true;
        (void)cbctl_set_envelope(&link[i], &env);
        memset(&m, 0, sizeof m);
        m.present_voltage = CBCTL_MV(380000u);
        m.present_current = CBCTL_MA(60000u);
        m.isolation = CBCTL_ISO_VALID;
        (void)cbctl_set_measurement(&link[i], &m);
        (void)cbctl_authorize(&link[i], CBCTL_AUTH_GRANTED, 8000u);
        (void)cbctl_relay_close(&link[i], CBPLC_RELAY_GUN, 10000u, NULL);
        lb[i].secc_active = true;
    }

    t0 = now_ms();
    last_ack_at = last_auth_at = t0;
    storm_at = t0 + (run_s * 1000u) / 2u;

    for (;;) {
        uint64_t a, b;
        t = now_ms();
        if ((t - t0) >= run_s * 1000u) { break; }

        /* A real caller re-states measurements every pass; that is what loads the wire. */
        for (i = 0; i < n; i++) {
            cbctl_measurement_t m;
            memset(&m, 0, sizeof m);
            m.present_voltage = CBCTL_MV(380000u + (t % 977u));
            m.present_current = CBCTL_MA(60000u + (t % 613u));
            m.isolation = CBCTL_ISO_VALID;
            (void)cbctl_set_measurement(&link[i], &m);
        }

        a = now_ns();
        (void)cbctl_fleet_tick(&fleet, t);
        b = now_ns();
        if (ns < MAX_SAMPLES) { g_samples[ns++] = b - a; }
        for (i = 0; i < n; i++) { cbctl_loopback_poll(&lb[i], t); }

        /* Pace the loop. Free-running fills the sample buffer with a million no-op passes in
         * the first second and the storm phase — the expensive part, four links mid-ladder —
         * never gets sampled. 500 us is still 800x faster than any deadline here, so it costs
         * the margins nothing while making every phase of the run land in the histogram. */
        {
            struct timespec ts = { 0, 500000 };
            (void)nanosleep(&ts, NULL);
        }

        /* Link 0's realized margins — the "did the other guns cost me anything" numbers. */
        {
            const uint32_t age = cbctl_feed_age_ms(&link[0], t);
            if (age != CBCTL_FEED_NEVER_SENT && age > worst_feed) { worst_feed = age; }
            if (stormed && age != CBCTL_FEED_NEVER_SENT && age > worst_feed_after_storm) {
                worst_feed_after_storm = age;
            }
        }
        if (link[0].relay_acks > 0u && link[0].last_relay.ms != last_ack_at) {
            if (last_ack_at > t0) {
                const uint32_t g = link[0].last_relay.ms - last_ack_at;
                if (g > worst_renew_gap) { worst_renew_gap = g; }
            }
            last_ack_at = link[0].last_relay.ms;
        }
        if (link[0].last_auth.ms != last_auth_at) {
            if (last_auth_at > t0) {
                const uint32_t g = link[0].last_auth.ms - last_auth_at;
                if (g > worst_auth_gap) { worst_auth_gap = g; }
            }
            last_auth_at = link[0].last_auth.ms;
        }

        /* THE STORM: half the boards wedge in the same millisecond — the hub-reset shape.
         * Wedged, not transport-dead: the fd stays alive and every request runs its full
         * 750 ms ladder, which is the more expensive of the two failure modes. */
        if (!stormed && t >= storm_at && n >= 2u) {
            stormed = 1;
            for (i = n / 2u; i < n; i++) { lb[i].wedged = true; }
        }

        /* Reaction, measured after the storm on a surviving link so the load is the worst
         * realistic one: re-close intent -> confirmed ACK, then a SoC edge -> visible. */
        if (stormed && react_phase == 0) {
            react_phase = 1;
            react_started = t;
            auth_sent_mark = link[0].relay_acks;
            /* Re-assert the same close: dirty flag forces a fresh RELAY exchange. */
            (void)cbctl_relay_close(&link[0], CBPLC_RELAY_GUN, 10000u, NULL);
        } else if (react_phase == 1 && link[0].relay_acks > auth_sent_mark) {
            react_close_ms = t - react_started;
            react_phase = 2;
            react_started = t;
            lb[0].ev_soc = 77u;
            react_soc_target = 77u;
        } else if (react_phase == 2 &&
                   cbctl_counter(&link[0], CBCTL_CTR_EV_SOC_PCT) == react_soc_target) {
            react_status_ms = t - react_started;
            react_phase = 3;
        }
    }

    /* Percentiles. */
    qsort(g_samples, ns, sizeof g_samples[0], cmp_u64);
    {
        const uint64_t p50 = ns ? g_samples[ns / 2u] : 0u;
        const uint64_t p90 = ns ? g_samples[(ns * 9u) / 10u] : 0u;
        const uint64_t p99 = ns ? g_samples[(ns * 99u) / 100u] : 0u;
        const uint64_t mx = ns ? g_samples[ns - 1u] : 0u;
        unsigned survivors_ready = 0u, storm_quarantined = 0u;
        for (i = 0; i < n / 2u || (n == 1u && i < 1u); i++) {
            if (cbctl_link_state(&link[i]) == CBCTL_LINK_READY) { survivors_ready++; }
        }
        for (i = n / 2u; i < n; i++) {
            if (cbctl_link_state(&link[i]) == CBCTL_LINK_QUARANTINED) { storm_quarantined++; }
        }
        printf("N=%u  passes=%u\n", n, ns);
        printf("  tick cost      : p50=%" PRIu64 "us p90=%" PRIu64 "us p99=%" PRIu64
               "us max=%" PRIu64 "us\n",
               p50 / 1000u, p90 / 1000u, p99 / 1000u, mx / 1000u);
        printf("  link0 margins  : worst feed=%ums (stale %u)  renew gap=%ums  auth gap=%ums\n",
               worst_feed, (unsigned)CBPLC_LIMITS_STALE_MS, worst_renew_gap, worst_auth_gap);
        if (n >= 2u) {
            printf("  storm (%u wedge): survivors READY=%u/%u  wedged quarantined=%u/%u  "
                   "link0 feed after=%ums\n",
                   n - n / 2u, survivors_ready, n / 2u, storm_quarantined, n - n / 2u,
                   worst_feed_after_storm);
        }
        printf("  reaction (loaded, post-storm): relay close->ACK=%ums  SoC edge->visible=%ums\n",
               react_close_ms, react_status_ms);
        printf("  link0 counters : feeds=%u acks=%u refusals=%u lease_opens=%u feed_late=%s\n\n",
               link[0].limits_sent, link[0].relay_acks, link[0].relay_refusals,
               cbctl_counter(&link[0], CBCTL_CTR_RELAY_LEASE_OPENS),
               link[0].feed_is_late ? "YES" : "0");
    }
}

int main(int argc, char **argv)
{
    const uint32_t run_s = (argc > 1) ? (uint32_t)atoi(argv[1]) : 20u;
    unsigned n;
    setvbuf(stdout, NULL, _IOLBF, 0);
    printf("fleet_bench: %us per N, loopback boards (real cbplc session code, no USB)\n\n",
           run_s);
    for (n = 1u; n <= MAXN; n *= 2u) {
        run_n(n, run_s);
    }
    return 0;
}
