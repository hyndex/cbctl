/* How much wall-clock does a tick actually cost, and how many guns does that allow?
 *
 * The 400 ms cadence against a 1500 ms deadline is the whole safety margin, and until now
 * nobody had measured what one tick costs on real hardware. Everything below is measured on
 * live boards, not modelled: the answer decides how many connectors one controller can drive
 * before the feed it is protecting starts going stale.
 *
 * Reports the DISTRIBUTION, not the mean. A mean tick cost is useless for a deadline — what
 * matters is the worst one, because that is the one that misses.
 */
#include "cbctl/cbctl.h"
#include "posix_serial.h"

#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAXLINK 4
#define SAMPLES 2000

static uint64_t mono_us(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

static int cmp_u64(const void *a, const void *b)
{
    const uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

int main(int argc, char **argv)
{
    cbctl_serial_t ser[MAXLINK];
    cbctl_link_t link[MAXLINK];
    cbctl_fleet_t fleet;
    static uint64_t cost[SAMPLES];
    int n = 0, i, s = 0;

    if (argc < 2) {
        fprintf(stderr, "usage: %s <port> [port...]\n", argv[0]);
        return 2;
    }
    cbctl_fleet_init(&fleet);
    for (i = 1; i < argc && n < MAXLINK; i++) {
        if (!cbctl_serial_open(&ser[n], argv[i], 115200u)) {
            fprintf(stderr, "cannot open %s\n", argv[i]);
            return 1;
        }
        (void)cbctl_open(&link[n], &ser[n].tx);
        (void)cbctl_fleet_add(&fleet, &link[n]);
        n++;
    }

    /* Settle: let both identify and get a feed going, so we measure steady state rather
     * than start-up. */
    for (i = 0; i < 200; i++) {
        for (int k = 0; k < n; k++) {
            cbctl_envelope_t e;
            cbctl_measurement_t m;
            memset(&e, 0, sizeof e);
            e.max_voltage = CBCTL_MV(500000u);
            e.max_current = CBCTL_MA(120000u);
            e.power_ready = true;
            (void)cbctl_set_envelope(&link[k], &e);
            memset(&m, 0, sizeof m);
            m.present_voltage = CBCTL_MV(400000u);
            m.isolation = CBCTL_ISO_VALID;
            (void)cbctl_set_measurement(&link[k], &m);
            (void)cbctl_authorize(&link[k], CBCTL_AUTH_GRANTED, 8000u);
        }
        cbctl_fleet_tick(&fleet, (uint32_t)(mono_us() / 1000u));
        (void)poll(NULL, 0, 5);
    }
    for (i = 0; i < n; i++) {
        (void)cbctl_relay_close(&link[i], CBPLC_RELAY_GUN, 3000u, NULL);
    }

    while (s < SAMPLES) {
        const uint32_t t = (uint32_t)(mono_us() / 1000u);
        uint64_t a, b;

        for (i = 0; i < n; i++) {
            cbctl_measurement_t m;
            memset(&m, 0, sizeof m);
            m.present_voltage = CBCTL_MV(400000u + (uint32_t)(s % 7) * 1000u); /* keep it dirty */
            m.isolation = CBCTL_ISO_VALID;
            (void)cbctl_set_measurement(&link[i], &m);
        }

        a = mono_us();
        cbctl_fleet_tick(&fleet, t);
        b = mono_us();
        cost[s++] = b - a;

        (void)poll(NULL, 0, 2);
    }

    qsort(cost, SAMPLES, sizeof cost[0], cmp_u64);
    printf("\n=== tick cost over %d samples, %d link%s ===\n", SAMPLES, n, n == 1 ? "" : "s");
    printf("  p50 %6llu us\n", (unsigned long long)cost[SAMPLES / 2]);
    printf("  p90 %6llu us\n", (unsigned long long)cost[(SAMPLES * 90) / 100]);
    printf("  p99 %6llu us\n", (unsigned long long)cost[(SAMPLES * 99) / 100]);
    printf("  max %6llu us\n", (unsigned long long)cost[SAMPLES - 1]);

    {
        /* The budget question. A tick must fit inside the cadence with room for the caller's
         * own work; the deadline that actually matters is CBPLC_LIMITS_STALE_MS, and the
         * design gives itself three attempts inside it. */
        const uint64_t worst = cost[SAMPLES - 1];
        const uint64_t per_link = (n > 0) ? (worst / (uint64_t)n) : worst;
        printf("\n  worst per link      : %llu us\n", (unsigned long long)per_link);
        printf("  cadence             : 400000 us\n");
        if (per_link > 0u) {
            printf("  links that fit in ONE cadence period : %llu\n",
                   (unsigned long long)(400000u / per_link));
            printf("  links before the 1500 ms deadline    : %llu\n",
                   (unsigned long long)(1500000u / per_link));
        }
        printf("\n  NOTE: this is the LIBRARY's cost only. Your control loop, OCPP client and\n"
               "  HMI share the same wall clock, so treat the figure above as a ceiling that\n"
               "  the rest of the application eats into, not as a budget you may spend.\n");
    }

    for (i = 0; i < n; i++) {
        (void)cbctl_relay_open(&link[i], CBPLC_RELAY_ALL, NULL);
        (void)cbctl_authorize(&link[i], CBCTL_AUTH_PENDING, 0u);
    }
    for (i = 0; i < 40; i++) {
        cbctl_fleet_tick(&fleet, (uint32_t)(mono_us() / 1000u));
        (void)poll(NULL, 0, 25);
    }
    for (i = 0; i < n; i++) {
        cbctl_serial_close(&ser[i]);
    }
    return 0;
}
