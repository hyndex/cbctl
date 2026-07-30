/* A working two-gun controller in one file.
 *
 *   cc -std=c11 -Iinclude -Iports -Ithird_party/cbplc/include \
 *      examples/pi_two_guns.c src/[a-z]*.c ports/posix_serial.c \
 *      third_party/cbplc/src/[a-z]*.c \
 *      -o pi_two_guns
 *   ./pi_two_guns /dev/serial/by-id/usb-Espressif_..._D0:CF:13:71:FF:54-if00 \
 *                 /dev/serial/by-id/usb-Espressif_..._D0:CF:13:73:0A:68-if00
 *
 * OPEN BY /dev/serial/by-id/, NEVER BY ttyACM NUMBER. Port order changes across reboots and
 * reflashes — and, measured on this project's own bench, within a single session. Passing
 * /dev/ttyACM0 here is how gun 1's commands end up on gun 2's board.
 *
 * This is the shape a real controller has. Note what is NOT here: no resend loop, no lease
 * renewal, no timeout handling, no thread, and no retry-on-refusal. Those are the things a
 * hand-rolled controller gets wrong, and they are all inside cbctl_tick().
 *
 * The measurements below are FAKE — a real controller reads them from its power modules (NOT
 * its meter: the meter reads through a CT downstream of the contactor and is blind during
 * PreCharge, which cost real charging sessions). Everything else is what you would ship,
 * including the two patterns a controller cannot omit: pinning each gun to a board MAC, and
 * reopening a port after the link is quarantined.
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

static void recover_link(int gun);   /* defined below; the quarantine handler calls it */

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
            recover_link(gun);
            break;

        default:
            break;
        }
    }
}

/* Gun -> board MAC. A real controller loads this from its configuration; all-zero disables the
 * check for a bench run with unknown boards. */
static const uint8_t kGunMac[2][6] = {
    { 0, 0, 0, 0, 0, 0 },
    { 0, 0, 0, 0, 0, 0 },
};
static const char *port[2];
static cbctl_serial_t ser[2];
static cbctl_link_t   link[2];

/* RECOVERY IS THE CALLER'S JOB, AND OMITTING IT IS A WHOLE-CHARGER OUTAGE.
 *
 * cbctl contains a dead transport perfectly — the link quarantines, re-probes on a capped
 * backoff, and its siblings never notice. What it will never do is reopen the port: the fd
 * belongs to whoever opened it. A controller without this function turns any USB flap into a
 * permanent outage for that gun, whose only cure is a process restart that kills the OTHER
 * gun's live session too.
 *
 * Reopening IN PLACE matters: the fleet slot keeps its pointer, so nothing is removed or
 * re-added and this works even at CBCTL_FLEET_MAX where an open-new-then-retire shape would be
 * refused. cbctl_open() memsets the link, so EVERYTHING must be re-declared afterwards — that
 * is the difference from a board reset, where the library re-asserts for you. */
static void recover_link(int gun)
{
    printf("gun%d: reopening %s in place\n", gun, port[gun]);
    cbctl_serial_close(&ser[gun]);
    if (!cbctl_serial_open(&ser[gun], port[gun], 115200u) ||
        cbctl_open(&link[gun], &ser[gun].tx) != CBCTL_OK) {
        printf("gun%d: reopen FAILED — will retry on the next quarantine\n", gun);
        return;
    }
    if (kGunMac[gun][0] != 0u) {
        (void)cbctl_expect_mac(&link[gun], kGunMac[gun]);
    }
    /* Re-declare here what your controller had declared: envelope, measurement, authorisation,
     * relay intent, provisioning and any diag arming. The steady-state loop below re-states the
     * envelope and measurement every pass, so this example needs nothing more. */
}

int main(int argc, char **argv)
{
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
        /* PIN THE GUN TO THE BOARD, not to the cable. cbctl_expect_mac() refuses once the
         * board has identified itself as something else, which is the only defence against a
         * swapped cable or a replaced board silently becoming "gun 1". Real controllers read
         * this table from config; it is inline here to keep the example one file. */
        if (kGunMac[n][0] != 0u) {
            (void)cbctl_expect_mac(&link[n], kGunMac[n]);
        }
        port[n] = argv[i];   /* remembered: recovery must reopen the SAME path */
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
