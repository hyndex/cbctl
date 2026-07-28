/* cbctl driving a REAL PLC board and a REAL vehicle, end to end.
 *
 * This is the hardware counterpart of the loopback suite, and it is deliberately shaped like
 * cbplc's Python harness (tools/bench.py): it JUDGES. It asserts along the way, it asserts the
 * close, and it exits non-zero when the run did not satisfy the gate. A live tool that only
 * prints is a tool nobody can put in CI.
 *
 * The gate mirrors the reference harness's, because "works" has to mean the same thing on both
 * sides of the wire:
 *
 *   1. the link identifies                              (which board, which firmware)
 *   2. the feed never goes stale                        (< CBPLC_LIMITS_STALE_MS, always)
 *   3. SLAC matches and the vehicle MAC becomes readable (the Autocharge trigger)
 *   4. a V2G session opens
 *   5. on stop: the relays end OPEN, and the dead-man never lapsed
 *
 *   cc -std=c11 -Iinclude -Iports -Ithird_party/cbplc/include \
 *      tools/live_session.c src/[a-z]*.c ports/posix_serial.c \
 *      third_party/cbplc/src/[a-z]*.c -o build/live_session
 *   ./build/live_session /dev/cu.usbmodem101 [seconds]
 */
#include "cbctl/cbctl.h"
#include "posix_serial.h"

#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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
    case CBCTL_LINK_DEGRADED:    return "degraded";
    case CBCTL_LINK_QUARANTINED: return "QUARANTINED";
    default:                     return "?";
    }
}

int main(int argc, char **argv)
{
    cbctl_serial_t ser;
    cbctl_link_t link;
    uint32_t t0, t, last_report = 0u;
    uint32_t run_s = (argc > 2) ? (uint32_t)atoi(argv[2]) : 180u;
    uint32_t worst_feed = 0u;
    uint32_t ready_at = 0u;
    uint32_t match_at = 0u, session_at = 0u, mac_at = 0u;
    uint8_t  mac[6];
    bool     have_mac = false, saw_session = false, saw_match = false;
    bool     relay_closed_once = false;
    int      errors = 0;
    /* BASELINE THE CUMULATIVE COUNTERS.
     *
     * relay_lease_opens and friends are TOTALS since the board booted, not this-run values.
     * Judging the raw total makes a previous run's fault fail this one — which is exactly the
     * live-flag-vs-cumulative-counter confusion cbctl separates by type, and which I then made
     * anyway in the judging tool. Assert on the DELTA. */
    uint32_t base_lease_opens = 0u, base_safety_opens = 0u, base_load_opens = 0u;
    bool     baselined = false;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <port> [seconds]\n", argv[0]);
        return 2;
    }
    signal(SIGINT, on_sigint);

    if (!cbctl_serial_open(&ser, argv[1], 115200u)) {
        fprintf(stderr, "cannot open %s\n", argv[1]);
        return 1;
    }
    if (cbctl_open(&link, &ser.tx) != CBCTL_OK) {
        fprintf(stderr, "cbctl_open failed\n");
        return 1;
    }
    printf("cbctl live: %s, budget %us\n", argv[1], run_s);

    t0 = now_ms();
    while (!g_stop) {
        cbctl_envelope_t env;
        cbctl_measurement_t m;
        cbctl_event_t ev;
        struct pollfd fd1;

        t = now_ms();
        if ((t - t0) / 1000u >= run_s) {
            break;
        }

        /* THE FEED. A real controller reads these from its meter and modules; on this bench
         * the supply is simulated, so they are synthesised — exactly as cbplc's Python harness
         * does. What is NOT synthesised is the timing, which is the thing under test. */
        memset(&env, 0, sizeof env);
        env.max_voltage = CBCTL_MV(500000u);
        env.min_voltage = CBCTL_MV(150000u);
        env.max_current = CBCTL_MA(120000u);
        env.max_power = CBCTL_W(60000u);
        env.power_ready = true;
        (void)cbctl_set_envelope(&link, &env);

        memset(&m, 0, sizeof m);
        m.present_voltage = CBCTL_MV(400000u);
        m.present_current = CBCTL_MA(0u);
        /* VALID because the bench has no real insulation measurement and the firmware needs a
         * verdict to leave CableCheck. On a real charger this comes from the IMD. */
        m.isolation = CBCTL_ISO_VALID;
        (void)cbctl_set_measurement(&link, &m);

        cbctl_tick(&link, t);

        if (cbctl_link_state(&link) == CBCTL_LINK_READY && ready_at == 0u) {
            ready_at = t - t0;
            printf("[%5ums] link READY\n", ready_at);
            /* Authorise as soon as the board is talking. A real controller would wait for its
             * CSMS; PENDING would be the honest answer meanwhile. */
            if (cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 8000u) != CBCTL_OK) {
                printf("  ERROR: authorize refused\n");
                errors++;
            }
            /* The vehicle is already plugged in and, after a firmware flash, is matched to a
             * stack that no longer exists. Nothing but a pilot transition makes it try again —
             * the same move cbplc's Python harness opens every cycle with. */
            {
                const cbctl_status_t rc = cbctl_slac_replug(&link, NULL);
                printf("           slac_replug -> %d\n", (int)rc);
            }
        }

        /* Feed staleness is the safety property, so it is sampled every pass rather than
         * reported at the end. */
        if (cbctl_link_state(&link) == CBCTL_LINK_READY) {
            const uint32_t age = cbctl_feed_age_ms(&link, t);
            if (age > worst_feed) {
                worst_feed = age;
            }
        }

        /* Take the baseline on the first snapshot that actually arrived. */
        if (!baselined && link.have_flags) {
            base_lease_opens = cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS);
            base_safety_opens = cbctl_counter(&link, CBCTL_CTR_RELAY_SAFETY_OPENS);
            base_load_opens = cbctl_counter(&link, CBCTL_CTR_GUN_OPEN_TIMEOUTS);
            baselined = true;
            printf("[%5ums] baseline: lease_opens=%u safety_opens=%u load_opens=%u\n", t - t0,
                   base_lease_opens, base_safety_opens, base_load_opens);
        }

        /* Seed from the FLAGS as well as the events. A vehicle that was already matched
         * before our first STATUS produces no edge, and reporting that as "SLAC: NO" while
         * the very same line prints slac=1 is the tool lying, not the library failing. */
        if (!saw_match && cbctl_flag(&link, CBCTL_FLAG_SLAC_MATCHED)) {
            saw_match = true;
            match_at = t - t0;
            printf("[%5ums] SLAC already matched at first snapshot\n", match_at);
        }
        if (!have_mac && cbctl_vehicle_mac(&link, mac)) {
            have_mac = true;
            mac_at = t - t0;
            printf("[%5ums] VEHICLE MAC %02X:%02X:%02X:%02X:%02X:%02X  (Autocharge)\n", mac_at,
                   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        }
        if (!saw_session && cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE)) {
            saw_session = true;
            session_at = t - t0;
        }

        while (cbctl_next_event(&link, &ev)) {
            switch (ev.kind) {
            case CBCTL_EV_VEHICLE_PLUGGED:
                printf("[%5ums] vehicle plugged\n", t - t0);
                break;
            case CBCTL_EV_VEHICLE_UNPLUGGED:
                printf("[%5ums] vehicle unplugged\n", t - t0);
                break;
            case CBCTL_EV_SLAC_MATCHED:
                saw_match = true;
                match_at = t - t0;
                printf("[%5ums] SLAC MATCHED\n", match_at);
                break;
            case CBCTL_EV_VEHICLE_MAC_KNOWN:
                if (cbctl_vehicle_mac(&link, mac)) {
                    have_mac = true;
                    mac_at = t - t0;
                    printf("[%5ums] VEHICLE MAC %02X:%02X:%02X:%02X:%02X:%02X  (Autocharge)\n",
                           mac_at, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                }
                break;
            case CBCTL_EV_SESSION_STARTED:
                saw_session = true;
                session_at = t - t0;
                printf("[%5ums] V2G SESSION STARTED\n", session_at);
                /* Close the gun on a dead-man. cbctl renews it at hold/3; we never touch it
                 * again. The firmware still enforces its own interlocks on top. */
                if (!relay_closed_once) {
                    const cbctl_status_t rc =
                        cbctl_relay_close(&link, CBPLC_RELAY_GUN, CBCTL_RELAY_HOLD_RECOMMENDED_MS, NULL);
                    printf("           relay_close(GUN, 3000ms) -> %d\n", (int)rc);
                    relay_closed_once = (rc == CBCTL_OK);
                }
                break;
            case CBCTL_EV_SESSION_ENDED:
                printf("[%5ums] session ended\n", t - t0);
                break;
            case CBCTL_EV_BOARD_RESET:
                printf("[%5ums] ERROR: BOARD RESET mid-run\n", t - t0);
                errors++;
                break;
            case CBCTL_EV_LINK_QUARANTINED:
                printf("[%5ums] ERROR: LINK QUARANTINED\n", t - t0);
                errors++;
                break;
            default:
                break;
            }
        }

        if (t - last_report >= 5000u) {
            printf("      %-12s feed_age=%4ums  plug=%d slac=%d sess=%d mac=%d  "
                   "lease_opens=%u safety_opens=%u promotions=%u\n",
                   state_name(cbctl_link_state(&link)), cbctl_feed_age_ms(&link, t),
                   cbctl_flag(&link, CBCTL_FLAG_VEHICLE_PRESENT) ? 1 : 0,
                   cbctl_flag(&link, CBCTL_FLAG_SLAC_MATCHED) ? 1 : 0,
                   cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE) ? 1 : 0,
                   cbctl_flag(&link, CBCTL_FLAG_VEHICLE_MAC_KNOWN) ? 1 : 0,
                   cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS),
                   cbctl_counter(&link, CBCTL_CTR_RELAY_SAFETY_OPENS),
                   link.status_promotions);
            last_report = t;
        }

        fd1.fd = cbctl_serial_fd(&ser);
        fd1.events = POLLIN;
        (void)poll(&fd1, 1, (int)cbctl_next_deadline_ms(&link, now_ms()));
    }

    /* ---- THE CLOSE, asserted rather than assumed ---------------------------------------- */
    printf("\n--- wind-down ---\n");
    (void)cbctl_begin_stop(&link);
    (void)cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL);
    (void)cbctl_authorize(&link, CBCTL_AUTH_PENDING, 0u);

    /* KEEP FEEDING THROUGH THE WIND-DOWN. PowerDelivery(Stop) and WeldingDetection are still
     * controller-fed exchanges: if the feed dies here the vehicle's close runs against a stale
     * envelope, which is how a session ends by TCP reset instead of SessionStop. There is no
     * blocking wait to call, which is exactly why this is a tick loop. */
    {
        const uint32_t w0 = now_ms();
        while (now_ms() - w0 < 8000u) {
            cbctl_measurement_t m;
            struct pollfd fd1;

            memset(&m, 0, sizeof m);
            m.present_voltage = CBCTL_MV(0u);   /* 0 V / 0 A is the weld evidence */
            m.present_current = CBCTL_MA(0u);
            m.isolation = CBCTL_ISO_VALID;
            (void)cbctl_set_measurement(&link, &m);

            cbctl_tick(&link, now_ms());
            fd1.fd = cbctl_serial_fd(&ser);
            fd1.events = POLLIN;
            (void)poll(&fd1, 1, 50);
        }
    }

    printf("\n=== cbctl live result ===\n");
    printf("  link state        : %s\n", state_name(cbctl_link_state(&link)));
    printf("  ready at          : %u ms\n", ready_at);
    printf("  worst feed age    : %u ms   (bound %u)\n", worst_feed,
           (unsigned)CBPLC_LIMITS_STALE_MS);
    printf("  SLAC matched      : %s%s\n", saw_match ? "yes" : "NO",
           saw_match ? "" : "  (vehicle may not have initiated)");
    if (saw_match) {
        printf("  match at          : %u ms\n", match_at);
    }
    printf("  vehicle MAC       : ");
    if (have_mac) {
        printf("%02X:%02X:%02X:%02X:%02X:%02X  at %u ms\n", mac[0], mac[1], mac[2], mac[3],
               mac[4], mac[5], mac_at);
    } else {
        printf("not obtained\n");
    }
    printf("  V2G session       : %s", saw_session ? "yes" : "no");
    if (saw_session) {
        printf("  at %u ms", session_at);
    }
    printf("\n");
    printf("  sessions (total)  : %u\n", cbctl_counter(&link, CBCTL_CTR_SESSIONS));
    printf("  relays open now   : %s\n",
           cbctl_flag(&link, CBCTL_FLAG_STOP_LATCHED) ? "stop latched" : "-");
    printf("  lease lapses      : %u this run (total %u)   (must be 0 — IS the relay flake)\n",
           cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS) - base_lease_opens,
           cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS));
    printf("  safety opens      : %u this run (total %u)\n",
           cbctl_counter(&link, CBCTL_CTR_RELAY_SAFETY_OPENS) - base_safety_opens,
           cbctl_counter(&link, CBCTL_CTR_RELAY_SAFETY_OPENS));
    printf("  opens under load  : %u this run (total %u)   (must be 0)\n",
           cbctl_counter(&link, CBCTL_CTR_GUN_OPEN_TIMEOUTS) - base_load_opens,
           cbctl_counter(&link, CBCTL_CTR_GUN_OPEN_TIMEOUTS));
    printf("  relay renewals    : %u (acks %u, danger-preempts %u)\n", link.relay_renewals,
           link.relay_acks, link.relay_danger_preempts);
    if (link.relay_acks == 0u) {
        printf("  min lease headroom: n/a — no lease was ever confirmed this run\n");
    } else {
        printf("  min lease headroom: %u ms of %u ms hold\n", link.lease_min_headroom_ms,
               link.relay_hold_ms);
    }
    printf("  worst tick gap    : %u ms  (caller's own lateness)\n", link.worst_tick_gap_ms);
    printf("  LED preset now    : %s\n", cbctl_led_name(cbctl_led_preset(&link, now_ms())));
    printf("  feed_late ticks   : %u\n", link.feed_late);
    printf("  status promotions : %u\n", link.status_promotions);
    printf("  transport         : deferred=%u partial=%u read_err=%u\n", ser.write_deferred,
           ser.partial_writes, ser.read_errors);

    /* ---- the gate ------------------------------------------------------------------------ */
    if (ready_at == 0u) {
        printf("  GATE: link never identified\n");
        errors++;
    }
    if (worst_feed >= CBPLC_LIMITS_STALE_MS) {
        printf("  GATE: feed went stale (%u >= %u)\n", worst_feed,
               (unsigned)CBPLC_LIMITS_STALE_MS);
        errors++;
    }
    if (link.feed_late != 0u) {
        printf("  GATE: %u late-feed ticks\n", link.feed_late);
        errors++;
    }
    if (cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS) != base_lease_opens) {
        printf("  GATE: the dead-man LAPSED — this is the audible relay flake\n");
        errors++;
    }
    if (cbctl_counter(&link, CBCTL_CTR_GUN_OPEN_TIMEOUTS) != base_load_opens) {
        printf("  GATE: a contactor opened under load\n");
        errors++;
    }

    cbctl_serial_close(&ser);
    printf("\n%s (%d problem%s)\n", errors ? "FAIL" : "PASS", errors, errors == 1 ? "" : "s");
    return errors ? 1 : 0;
}
