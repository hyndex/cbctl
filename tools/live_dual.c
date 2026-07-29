/* cbctl driving TWO real PLC boards at once, and trying to break the isolation between them.
 *
 * The loopback proves F1-F6 against a simulated peer. This proves it against two real boards
 * on real USB, where the failure modes are not simulated: enumeration order, shared bus
 * bandwidth, one board rebooting, one board being physically unplugged mid-charge.
 *
 * It JUDGES. Gate:
 *   1. both boards identify, with DISTINCT MACs
 *   2. neither link's feed ever goes stale, at any point, including while the other is abused
 *   3. no lease lapses, no opens under load
 *   4. when one board is quarantined, the other stays READY and keeps its lease
 *
 *   make live-dual
 *   ./build/live_dual /dev/cu.usbmodem101 /dev/cu.usbmodem2101 [seconds]
 */
#include "cbctl/cbctl.h"
#include "posix_serial.h"

#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define NLINK 2

static volatile sig_atomic_t g_stop;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static uint32_t now_ms(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000));
}

static const char *state_name(cbctl_link_state_t s)
{
    switch (s) {
    case CBCTL_LINK_CLOSED:      return "closed";
    case CBCTL_LINK_IDENTIFYING: return "identifying";
    case CBCTL_LINK_READY:       return "ready";
    case CBCTL_LINK_DEGRADED:    return "DEGRADED";
    case CBCTL_LINK_QUARANTINED: return "QUARANTINED";
    default:                     return "?";
    }
}

int main(int argc, char **argv)
{
    cbctl_serial_t ser[NLINK];
    cbctl_link_t link[NLINK];
    cbctl_fleet_t fleet;
    cbctl_identity_t id[NLINK];
    uint32_t t0, t, last_report = 0u;
    uint32_t run_s = (argc > 3) ? (uint32_t)atoi(argv[3]) : 120u;
    /* Seconds after start at which gun1's transport is KILLED, or 0 to leave it alone.
     *
     * Closing the fd underneath the link models the failure this project has actually had:
     * a USB drop. On the Pi 4B hub, over-current took both PLCs down together; the question
     * a dual-gun charger has to answer is whether ONE going away takes the other with it. */
    uint32_t kill_at_s = (argc > 4) ? (uint32_t)atoi(argv[4]) : 0u;
    bool killed = false;
    uint32_t worst_feed_after_kill = 0u;
    uint32_t worst_feed[NLINK] = { 0u, 0u };
    uint32_t base_lease[NLINK] = { 0u, 0u }, base_load[NLINK] = { 0u, 0u };
    bool baselined = false, identified = false;
    int i, errors = 0;

    if (argc < 3) {
        fprintf(stderr, "usage: %s <portA> <portB> [seconds]\n", argv[0]);
        return 2;
    }
    signal(SIGINT, on_sigint);
    cbctl_fleet_init(&fleet);

    for (i = 0; i < NLINK; i++) {
        if (!cbctl_serial_open(&ser[i], argv[i + 1], 115200u)) {
            fprintf(stderr, "cannot open %s\n", argv[i + 1]);
            return 1;
        }
        if (cbctl_open(&link[i], &ser[i].tx) != CBCTL_OK ||
            cbctl_fleet_add(&fleet, &link[i]) != CBCTL_OK) {
            fprintf(stderr, "cannot attach %s\n", argv[i + 1]);
            return 1;
        }
        printf("gun%d: %s\n", i, argv[i + 1]);
    }

    t0 = now_ms();
    while (!g_stop) {
        struct pollfd fds[NLINK];
        int nfds = 0;

        t = now_ms();
        if ((t - t0) / 1000u >= run_s) {
            break;
        }

        for (i = 0; i < NLINK; i++) {
            cbctl_envelope_t env;
            cbctl_measurement_t m;
            cbctl_event_t ev;

            memset(&env, 0, sizeof env);
            env.max_voltage = CBCTL_MV(500000u);
            env.min_voltage = CBCTL_MV(150000u);
            env.max_current = CBCTL_MA(120000u);
            env.max_power = CBCTL_W(60000u);
            env.power_ready = true;
            (void)cbctl_set_envelope(&link[i], &env);

            memset(&m, 0, sizeof m);
            m.present_voltage = CBCTL_MV(400000u);
            m.isolation = CBCTL_ISO_VALID;
            (void)cbctl_set_measurement(&link[i], &m);

            while (cbctl_next_event(&link[i], &ev)) {
                if (ev.kind == CBCTL_EV_SESSION_STARTED) {
                    printf("[%6ums] gun%d: session started, closing gun relay\n", t - t0, i);
                    (void)cbctl_relay_close(&link[i], CBPLC_RELAY_GUN, CBCTL_RELAY_HOLD_RECOMMENDED_MS, NULL);
                } else if (ev.kind == CBCTL_EV_LINK_QUARANTINED) {
                    printf("[%6ums] gun%d: QUARANTINED\n", t - t0, i);
                } else if (ev.kind == CBCTL_EV_BOARD_RESET) {
                    printf("[%6ums] gun%d: board reset\n", t - t0, i);
                }
            }
        }

        cbctl_fleet_tick(&fleet, t);

        /* Once both have identified, pin them and prove they are different boards. */
        if (!identified && cbctl_identity(&link[0], &id[0]) && cbctl_identity(&link[1], &id[1])) {
            identified = true;
            for (i = 0; i < NLINK; i++) {
                printf("gun%d identity: %02X:%02X:%02X:%02X:%02X:%02X build=%s wire=%u\n", i,
                       id[i].mac[0], id[i].mac[1], id[i].mac[2], id[i].mac[3], id[i].mac[4],
                       id[i].mac[5], id[i].build, id[i].wire_version);
            }
            if (memcmp(id[0].mac, id[1].mac, 6u) == 0) {
                printf("  ERROR: both links are the SAME board\n");
                errors++;
            }
            /* Pin them: from here on a swap is an error, not a surprise. */
            for (i = 0; i < NLINK; i++) {
                if (cbctl_expect_mac(&link[i], id[i].mac) != CBCTL_OK) {
                    printf("  ERROR: gun%d failed its own pin\n", i);
                    errors++;
                }
            }
        }

        if (!baselined && link[0].have_flags && link[1].have_flags) {
            baselined = true;
            for (i = 0; i < NLINK; i++) {
                base_lease[i] = cbctl_counter(&link[i], CBCTL_CTR_RELAY_LEASE_OPENS);
                base_load[i] = cbctl_counter(&link[i], CBCTL_CTR_GUN_OPEN_TIMEOUTS);
            }
        }

        /* CBCTL_FEED_NEVER_SENT IS NOT AN AGE, AND FOLDING IT INTO A MAXIMUM IS THE LIE THE
         * SENTINEL EXISTS TO PREVENT.
         *
         * cbctl_feed_age_ms() returns 0xFFFFFFFF until a feed has actually gone out — precisely
         * so "never declared" cannot be mistaken for "perfectly fresh", which is a bug this
         * library already shipped once. Taking a max over it makes the opposite mistake: this
         * tool reported worst_feed=4294967295 and failed its own gate with "feed went stale" on
         * a run where both links were healthy and the summary lines said feed=0ms and 396ms.
         * A sentinel has to be excluded from arithmetic at every site, not just defined. */
        for (i = 0; i < NLINK; i++) {
            if (cbctl_link_state(&link[i]) == CBCTL_LINK_READY) {
                const uint32_t age = cbctl_feed_age_ms(&link[i], t);
                if (age != CBCTL_FEED_NEVER_SENT && age > worst_feed[i]) {
                    worst_feed[i] = age;
                }
            }
        }

        /* THE ISOLATION EVENT. Pull gun1's transport out from under it, mid-run. */
        if (kill_at_s != 0u && !killed && (t - t0) / 1000u >= kill_at_s) {
            killed = true;
            printf("[%6ums] *** KILLING gun1's transport (models a USB drop) ***\n", t - t0);
            cbctl_serial_close(&ser[1]);
        }
        /* After the kill, gun0's feed age is the whole question. */
        if (killed && cbctl_link_state(&link[0]) == CBCTL_LINK_READY) {
            const uint32_t a = cbctl_feed_age_ms(&link[0], t);
            if (a != CBCTL_FEED_NEVER_SENT && a > worst_feed_after_kill) {
                worst_feed_after_kill = a;
            }
        }

        if (t - last_report >= 5000u) {
            for (i = 0; i < NLINK; i++) {
                const uint32_t obs = cbctl_observation_age_ms(&link[i], t);
                char obsbuf[16];
                if (obs == CBCTL_NEVER_OBSERVED) {
                    (void)snprintf(obsbuf, sizeof obsbuf, "never");
                } else {
                    (void)snprintf(obsbuf, sizeof obsbuf, "%ums", obs);
                }
                printf("  gun%d %-12s feed=%4ums obs=%7s plug=%d sess=%d renew=%u acks=%u\n",
                       i, state_name(cbctl_link_state(&link[i])),
                       cbctl_feed_age_ms(&link[i], t), obsbuf,
                       cbctl_flag(&link[i], CBCTL_FLAG_VEHICLE_PRESENT) ? 1 : 0,
                       cbctl_flag(&link[i], CBCTL_FLAG_SESSION_ACTIVE) ? 1 : 0,
                       link[i].relay_renewals, link[i].relay_acks);
            }
            last_report = t;
        }

        for (i = 0; i < NLINK; i++) {
            const int fd = cbctl_serial_fd(&ser[i]);
            if (fd >= 0) {
                fds[nfds].fd = fd;
                fds[nfds].events = POLLIN;
                nfds++;
            }
        }
        (void)poll(fds, (nfds_t)nfds, (int)cbctl_fleet_next_deadline_ms(&fleet, now_ms()));
    }

    printf("\n--- wind-down: both guns ---\n");
    for (i = 0; i < NLINK; i++) {
        (void)cbctl_begin_stop(&link[i]);
        (void)cbctl_relay_open(&link[i], CBPLC_RELAY_ALL, NULL);
        (void)cbctl_authorize(&link[i], CBCTL_AUTH_PENDING, 0u);
    }
    {
        const uint32_t w0 = now_ms();
        while (now_ms() - w0 < 6000u) {
            for (i = 0; i < NLINK; i++) {
                cbctl_measurement_t m;
                memset(&m, 0, sizeof m);
                m.present_voltage = CBCTL_MV(0u);
                m.present_current = CBCTL_MA(0u);
                m.isolation = CBCTL_ISO_VALID;
                (void)cbctl_set_measurement(&link[i], &m);
            }
            cbctl_fleet_tick(&fleet, now_ms());
            (void)poll(NULL, 0, 40);
        }
    }

    printf("\n=== dual-PLC result ===\n");
    if (killed) {
        printf("  gun1 transport was KILLED mid-run\n");
        printf("  gun0 worst feed age AFTER the kill : %u ms  (bound %u)\n",
               worst_feed_after_kill, (unsigned)CBPLC_LIMITS_STALE_MS);
        if (worst_feed_after_kill >= CBPLC_LIMITS_STALE_MS) {
            printf("  GATE: gun1 dying took gun0's feed past the bound\n");
            errors++;
        }
        if (cbctl_link_state(&link[0]) != CBCTL_LINK_READY) {
            printf("  GATE: gun0 did not stay READY while gun1 died\n");
            errors++;
        }
        if (cbctl_link_state(&link[1]) == CBCTL_LINK_READY) {
            printf("  GATE: gun1 still reports READY after its transport died\n");
            errors++;
        }
    }
    printf("  duplicate boards detected : %u  (must be 0)\n", fleet.duplicate_boards);
    for (i = 0; i < NLINK; i++) {
        printf("  gun%d: %-12s worst_feed=%4ums late=%u renewals=%u acks=%u refusals=%u "
               "lease_lapses=%u load_opens=%u stalls=%u tx(def=%u part=%u rd=%u)\n",
               i, state_name(cbctl_link_state(&link[i])), worst_feed[i], link[i].feed_late,
               link[i].relay_renewals, link[i].relay_acks, link[i].relay_refusals,
               cbctl_counter(&link[i], CBCTL_CTR_RELAY_LEASE_OPENS) - base_lease[i],
               cbctl_counter(&link[i], CBCTL_CTR_GUN_OPEN_TIMEOUTS) - base_load[i],
               link[i].observation_stalls, ser[i].write_deferred, ser[i].partial_writes,
               ser[i].read_errors);
        {
            /* "0 ms of headroom" and "no lease was ever held" are different facts and must not
             * print the same way — the first is an emergency, the second is Tuesday. */
            char head[24];
            if (link[i].relay_acks == 0u) {
                (void)snprintf(head, sizeof head, "n/a (no lease)");
            } else {
                (void)snprintf(head, sizeof head, "%ums", link[i].lease_min_headroom_ms);
            }
            printf("        led=%-9s min_headroom=%-14s worst_tick_gap=%ums preempts=%u\n",
                   cbctl_led_name(cbctl_led_preset(&link[i], now_ms())), head,
                   link[i].worst_tick_gap_ms, link[i].relay_danger_preempts);
        }

        if (killed && i == 1) {
            continue; /* deliberately destroyed; it is the SURVIVOR that is on trial */
        }
        if (worst_feed[i] == 0u && link[i].limits_sent == 0u) {
            /* Nothing was ever declared on this link, so there is no feed to judge. Reporting
             * that as a pass would be the same sentinel confusion in the other direction. */
            printf("  note: gun%d never had an envelope declared — feed not exercised\n", i);
        } else if (worst_feed[i] >= CBPLC_LIMITS_STALE_MS) {
            printf("  GATE: gun%d feed went stale (%u)\n", i, worst_feed[i]);
            errors++;
        }
        if (link[i].feed_late != 0u) {
            printf("  GATE: gun%d had %u late-feed ticks\n", i, link[i].feed_late);
            errors++;
        }
        if (cbctl_counter(&link[i], CBCTL_CTR_RELAY_LEASE_OPENS) != base_lease[i]) {
            printf("  GATE: gun%d dead-man LAPSED\n", i);
            errors++;
        }
        if (cbctl_counter(&link[i], CBCTL_CTR_GUN_OPEN_TIMEOUTS) != base_load[i]) {
            printf("  GATE: gun%d opened a contactor under load\n", i);
            errors++;
        }
    }
    if (fleet.duplicate_boards != 0u) {
        printf("  GATE: two links resolved to the same physical board\n");
        errors++;
    }
    if (!identified) {
        printf("  GATE: both boards never identified\n");
        errors++;
    }

    for (i = 0; i < NLINK; i++) {
        cbctl_serial_close(&ser[i]);
    }
    printf("\n%s (%d problem%s)\n", errors ? "FAIL" : "PASS", errors, errors == 1 ? "" : "s");
    return errors ? 1 : 0;
}
