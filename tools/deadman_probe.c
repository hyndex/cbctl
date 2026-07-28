/* Hold a relay lease, then get killed, so the BOARD's dead-man can be observed doing its job.
 *
 * The lease is the reason a crashed controller is safe rather than merely detectable, and it
 * had never been verified end to end on hardware — only its renewal had. This closes a relay,
 * renews it correctly, prints the moment it is holding, and then waits to be SIGKILLed. What
 * happens next is the board's business, and the point: nothing tidy runs on the way out.
 *
 *   ./build/deadman_probe <port> <relay-mask-hex> <hold_ms>
 */
#include "cbctl/cbctl.h"
#include "posix_serial.h"

#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static uint32_t now_ms(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000));
}

int main(int argc, char **argv)
{
    cbctl_serial_t ser;
    cbctl_link_t link;
    cbctl_op_t op;
    const uint8_t mask = (argc > 2) ? (uint8_t)strtoul(argv[2], NULL, 16) : CBPLC_RELAY_AUX;
    const uint32_t hold = (argc > 3) ? (uint32_t)atoi(argv[3]) : 3000u;
    uint32_t t0;
    bool asked = false;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <port> [mask-hex] [hold_ms]\n", argv[0]);
        return 2;
    }
    if (!cbctl_serial_open(&ser, argv[1], 115200u) || cbctl_open(&link, &ser.tx) != CBCTL_OK) {
        fprintf(stderr, "cannot attach %s\n", argv[1]);
        return 1;
    }

    t0 = now_ms();
    for (;;) {
        cbctl_envelope_t e;
        cbctl_measurement_t m;
        struct pollfd fd1;
        const uint32_t t = now_ms();

        memset(&e, 0, sizeof e);
        e.max_voltage = CBCTL_MV(500000u);
        e.max_current = CBCTL_MA(120000u);
        e.power_ready = true;
        (void)cbctl_set_envelope(&link, &e);
        memset(&m, 0, sizeof m);
        m.present_voltage = CBCTL_MV(400000u);
        m.isolation = CBCTL_ISO_VALID;
        (void)cbctl_set_measurement(&link, &m);

        cbctl_tick(&link, t);

        if (!asked && cbctl_link_state(&link) == CBCTL_LINK_READY) {
            (void)cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 8000u);
            if (cbctl_relay_close(&link, mask, hold, &op) == CBCTL_OK) {
                asked = true;
                printf("asked for relay 0x%02X, hold %u ms (stored %u)\n", mask, hold,
                       link.relay_hold_ms);
                fflush(stdout);
            }
        }
        if (asked && (t - t0) > 2500u) {
            printf("HOLDING: renewals=%u acks=%u refusals=%u gate=%u | board: closed_mask "
                   "safety_opens=%u lease_opens=%u — kill me now\n",
                   link.relay_renewals, link.relay_acks, link.relay_refusals,
                   cbctl_op_gate(&link, op),
                   cbctl_counter(&link, CBCTL_CTR_RELAY_SAFETY_OPENS),
                   cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS));
            fflush(stdout);
            t0 = t;
        }

        fd1.fd = cbctl_serial_fd(&ser);
        fd1.events = POLLIN;
        (void)poll(&fd1, 1, (int)cbctl_next_deadline_ms(&link, now_ms()));
    }
}
