/* A working two-gun controller in one file.
 *
 *   cc -std=c11 -Iinclude -Iports -Ithird_party/cbplc/include \
 *      examples/pi_two_guns.c src/[a-z]*.c ports/posix_serial.c \
 *      third_party/cbplc/src/[a-z]*.c \
 *      -o pi_two_guns
 *   ./pi_two_guns /dev/ttyACM0 /dev/ttyACM1
 *
 * This is the shape a real controller has. Note what is NOT here: no resend loop, no lease
 * renewal, no timeout handling, no thread, and no retry-on-refusal. Those are the things a
 * hand-rolled controller gets wrong, and they are all inside cbctl_tick().
 *
 * The measurements below are FAKE — a real controller reads them from its meter and its power
 * modules. Everything else is exactly what you would ship.
 */
#include "cbctl/cbctl.h"
#include "posix_serial.h"

#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t g_stop;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* MONOTONIC. Not gettimeofday, not anything NTP can step: this project has an incident where
 * a clock step made a watchdog compute a 921-second silence and kill a healthy controller. */
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

static void handle_events(cbctl_link_t *link, int gun)
{
    cbctl_event_t ev;
    uint8_t mac[6];

    while (cbctl_next_event(link, &ev)) {
        switch (ev.kind) {
        case CBCTL_EV_VEHICLE_PLUGGED:
            printf("gun%d: vehicle plugged\n", gun);
            /* PENDING, not DENY. The driver has not tapped their card yet, and the honest
             * answer for the next few seconds is "still deciding" — which keeps the session
             * alive. Collapsing this to a bool makes every slow CSMS lookup look like a
             * refusal and ends the session before the driver has finished paying. */
            (void)cbctl_authorize(link, CBCTL_AUTH_PENDING, 0u);
            break;

        case CBCTL_EV_VEHICLE_MAC_KNOWN:
            /* AUTOCHARGE HAPPENS HERE, at SLAC match — before any V2G session exists and long
             * before an EVCCID. This is the moment a returning vehicle can be recognised
             * without the driver doing anything. */
            if (cbctl_vehicle_mac(link, mac)) {
                printf("gun%d: vehicle MAC %02X:%02X:%02X:%02X:%02X:%02X\n", gun,
                       mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                /* if (autocharge_lookup(mac)) ... */
                (void)cbctl_authorize(link, CBCTL_AUTH_GRANTED, 8000u);
            }
            break;

        case CBCTL_EV_SESSION_STARTED:
            printf("gun%d: V2G session started\n", gun);
            break;

        case CBCTL_EV_SESSION_ENDED:
            printf("gun%d: session ended\n", gun);
            break;

        case CBCTL_EV_VEHICLE_UNPLUGGED:
            printf("gun%d: unplugged\n", gun);
            (void)cbctl_authorize(link, CBCTL_AUTH_PENDING, 0u);
            (void)cbctl_relay_open(link, CBPLC_RELAY_ALL, NULL);
            break;

        case CBCTL_EV_BOARD_RESET:
            /* Leases and authorisation did NOT survive the reset. cbctl re-asserts the
             * declared intent on its own, but a controller that does not notice is one
             * commanding a board that has forgotten everything it was told. */
            printf("gun%d: BOARD RESET — re-establishing\n", gun);
            break;

        case CBCTL_EV_LINK_QUARANTINED:
            /* Loud on purpose: a charger that silently demotes a connector is worse than one
             * that fails, because the operator hears about it from a customer. */
            printf("gun%d: LINK QUARANTINED — connector out of service\n", gun);
            break;

        default:
            break;
        }
    }
}

int main(int argc, char **argv)
{
    cbctl_serial_t ser[2];
    cbctl_link_t link[2];
    cbctl_fleet_t fleet;
    int n = 0, i;
    uint32_t next_report = 0u;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <port> [port2]\n", argv[0]);
        return 2;
    }
    signal(SIGINT, on_sigint);

    cbctl_fleet_init(&fleet);
    for (i = 1; i < argc && i <= 2; i++) {
        if (!cbctl_serial_open(&ser[n], argv[i], 115200u)) {
            fprintf(stderr, "cannot open %s\n", argv[i]);
            return 1;
        }
        if (cbctl_open(&link[n], &ser[n].tx) != CBCTL_OK ||
            cbctl_fleet_add(&fleet, &link[n]) != CBCTL_OK) {
            fprintf(stderr, "cannot attach %s\n", argv[i]);
            return 1;
        }
        printf("gun%d: %s\n", n, argv[i]);
        n++;
    }

    while (!g_stop) {
        struct pollfd fds[2];
        int nfds = 0;
        uint32_t t = now_ms();

        /* Declare what the power path can do and what it is measuring. Called every pass here
         * for clarity; a real controller calls them only when its values CHANGE, because
         * cbctl_tick() owns the repetition either way. */
        for (i = 0; i < n; i++) {
            cbctl_envelope_t env;
            cbctl_measurement_t m;

            memset(&env, 0, sizeof env);
            env.max_voltage = CBCTL_MV(500000u);
            env.min_voltage = CBCTL_MV(150000u);
            env.max_current = CBCTL_MA(200000u);
            env.max_power = CBCTL_W(100000u);
            env.power_ready = true;
            (void)cbctl_set_envelope(&link[i], &env);

            memset(&m, 0, sizeof m);
            m.present_voltage = CBCTL_MV(0u);
            m.present_current = CBCTL_MA(0u);
            /* UNKNOWN until the insulation test has a verdict. Mapping "not measured" onto
             * VALID is an accidental claim that CableCheck passed. */
            m.isolation = CBCTL_ISO_UNKNOWN;
            (void)cbctl_set_measurement(&link[i], &m);
        }

        cbctl_fleet_tick(&fleet, t);
        for (i = 0; i < n; i++) {
            handle_events(&link[i], i);
        }

        if (t >= next_report) {
            for (i = 0; i < n; i++) {
                printf("gun%d: %-12s feed_age=%4u ms  lease_opens=%u  budget_opens=%u\n", i,
                       state_name(cbctl_link_state(&link[i])),
                       cbctl_feed_age_ms(&link[i], t),
                       cbctl_counter(&link[i], CBCTL_CTR_RELAY_LEASE_OPENS),
                       cbctl_counter(&link[i], CBCTL_CTR_GUN_OPEN_TIMEOUTS));
            }
            next_report = t + 5000u;
        }

        /* Sleep exactly as long as cbctl says is safe, waking early on data. The deadline is
         * mandatory; the descriptors are optional and this loop works without them. */
        for (i = 0; i < n; i++) {
            const int fd = cbctl_serial_fd(&ser[i]);
            if (fd >= 0) {
                fds[nfds].fd = fd;
                fds[nfds].events = POLLIN;
                nfds++;
            }
        }
        (void)poll(fds, (nfds_t)nfds, (int)cbctl_fleet_next_deadline_ms(&fleet, now_ms()));
    }

    printf("\nshutting down: opening relays and revoking authorisation\n");
    for (i = 0; i < n; i++) {
        (void)cbctl_begin_stop(&link[i]);
        (void)cbctl_relay_open(&link[i], CBPLC_RELAY_ALL, NULL);
        (void)cbctl_authorize(&link[i], CBCTL_AUTH_PENDING, 0u);
    }
    /* Keep ticking so the wind-down actually reaches the board. There is no blocking wait to
     * call here, and that is deliberate: a call that waited could not keep the feed alive,
     * and the wind-down needs the feed. */
    for (i = 0; i < 40; i++) {
        cbctl_fleet_tick(&fleet, now_ms());
        (void)poll(NULL, 0, 25);
    }
    for (i = 0; i < n; i++) {
        cbctl_serial_close(&ser[i]);
    }
    return 0;
}
