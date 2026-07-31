/* weld_timing — measure how fast the gun-side meter can prove a contactor opened.
 *
 * WHAT THIS NUMBER IS FOR. Weld detection asks one question: after we command
 * the gun contactor open, is the VEHICLE side still live? Only the meter can
 * answer it, because only the meter is on that side of the switch:
 *
 *     [modules] --- (gun contactor) --- [METER] --- (EV contactor) --- [pack]
 *
 * The controller serves a discharge dwell after commanding open and only then
 * draws a conclusion. That dwell has to exceed the meter's own settling time —
 * bleed-down plus one poll period plus the worst sample age — or every clean
 * open reads as a timeout. Calibrating it by guess is how a charger either
 * condemns healthy connectors or waits pointlessly on every session end.
 *
 * So: energise, close, confirm the meter sees it, open, and time the fall.
 * Repeated, because a single sample of a physical bleed-down is an anecdote.
 *
 * WHY THIS USES THE SHIPPING DRIVERS. Same argument power_cbm.cpp makes: the
 * first bench version of anything like this drove the modules from an encoder
 * written for the occasion, never sent the output-enable, and reported a
 * perfect setpoint into 0 V. cbmodules and cbmeter are the code the fleet runs.
 * A bench that talks to hardware its own way is testing the bench.
 *
 * SAFETY, AND IT IS NOT OPTIONAL
 *   - The evse-controller MUST be stopped. Two writers on one CAN bus and one
 *     PLC serial link will fight, and the loser is whichever one was mid-charge.
 *   - REFUSES if the pilot is not state A. This closes a DC contactor; doing
 *     that with a vehicle attached is not a bench test.
 *   - Every exit path drops the relay and commands the modules off, including
 *     the signal handler. A cleanup path that only runs on success is not one.
 *
 * USAGE
 *     weld_timing --port /dev/serial/by-id/...-if00 --can can0 --meter /dev/serial0 \
 *                 --volts 350 --runs 5
 */
#include "cbctl/cbctl.h"
#include "posix_serial.h"

#include <cbplc/limits.h>

#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* power_cbm.cpp — the shipping cbmodules + cbmeter carriage. */
extern int  power_open(const char *can_iface, const char *meter_dev, int meter_unit,
                       const int *module_addrs, int nmodules);
extern void power_set(double volts, double amps);
extern void power_off(void);
extern void power_tick(void);
extern int  power_read(double *v, double *a);
extern int  power_meter_read(int channel, double *v, double *a);
extern void power_close(void);

static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s) { (void)s; g_stop = 1; }

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static void nap_ms(unsigned ms)
{
    struct timespec ts = { ms / 1000u, (long)(ms % 1000u) * 1000000L };
    nanosleep(&ts, NULL);
}

/* Service both links for `ms`, so cbctl renews the relay lease and cbmodules
 * keeps re-sending the setpoint. Modules turn OFF if they stop hearing from us
 * — that is their own dead-man and it is the reason this cannot just sleep. */
static void pump(cbctl_link_t *link, unsigned ms)
{
    const uint32_t until = now_ms() + ms;
    while (!g_stop && (int32_t)(until - now_ms()) > 0) {
        cbctl_tick(link, now_ms());
        power_tick();
        nap_ms(20);
    }
}

static int cmp_u32(const void *a, const void *b)
{
    const uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return (x > y) - (x < y);
}

int main(int argc, char **argv)
{
    const char *port   = "/dev/ttyACM0";
    const char *canif  = "can0";
    const char *meter  = "/dev/serial0";
    int   meter_unit   = 2;
    int   meter_ch     = 1;
    int   module_addr  = 1;
    double volts       = 350.0;
    double amps        = 2.0;    /* a trickle: enough to charge the bus, not to weld */
    int   runs         = 5;
    unsigned settle_ms = 8000;
    unsigned poll_ms   = 50;
    double zero_v      = 20.0;
    unsigned timeout_ms = 60000;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--port") && i + 1 < argc)        port = argv[++i];
        else if (!strcmp(argv[i], "--can") && i + 1 < argc)    canif = argv[++i];
        else if (!strcmp(argv[i], "--meter") && i + 1 < argc)  meter = argv[++i];
        else if (!strcmp(argv[i], "--meter-unit") && i+1<argc) meter_unit = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--meter-ch") && i + 1<argc) meter_ch = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--module") && i + 1 < argc) module_addr = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--volts") && i + 1 < argc)  volts = atof(argv[++i]);
        else if (!strcmp(argv[i], "--amps") && i + 1 < argc)   amps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--runs") && i + 1 < argc)   runs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--settle-ms") && i+1<argc)  settle_ms = (unsigned)atoi(argv[++i]);
        else if (!strcmp(argv[i], "--zero-v") && i + 1 < argc) zero_v = atof(argv[++i]);
        else {
            fprintf(stderr,
                "usage: %s [--port DEV] [--can IF] [--meter DEV] [--meter-unit N]\n"
                "          [--meter-ch N] [--module ADDR] [--volts V] [--amps A]\n"
                "          [--runs N] [--settle-ms MS] [--zero-v V]\n", argv[0]);
            return 2;
        }
    }
    if (volts < 100.0 || volts > 500.0) {
        fprintf(stderr, "refusing --volts %.0f: this bench is for 100-500 V\n", volts);
        return 2;
    }

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    cbctl_serial_t ser;
    cbctl_link_t link;
    memset(&link, 0, sizeof link);

    const int addrs[1] = { module_addr };
    if (power_open(canif, meter, meter_unit, addrs, 1) != 0) {
        fprintf(stderr, "power_open failed (can=%s meter=%s)\n", canif, meter);
        return 1;
    }
    if (!cbctl_serial_open(&ser, port, 115200u) || cbctl_open(&link, &ser.tx) != CBCTL_OK) {
        fprintf(stderr, "cannot open PLC link on %s\n", port);
        power_close();
        return 1;
    }

    /* Let the board identify and the modules report before deciding anything. */
    printf("waiting for the PLC link and the modules…\n");
    pump(&link, 4000);
    if (cbctl_link_state(&link) != CBCTL_LINK_READY) {
        fprintf(stderr, "PLC link is not READY (state=%d); refusing\n",
                (int)cbctl_link_state(&link));
        goto done;
    }

    /* THE PILOT GATE. This closes a DC contactor. */
    if (cbctl_flag(&link, CBCTL_FLAG_VEHICLE_PRESENT)) {
        fprintf(stderr,
            "REFUSING: the pilot has left state A — a vehicle is attached.\n"
            "This bench closes and opens a DC gun contactor at %.0f V.\n", volts);
        goto done;
    }

    /* BASELINE PROBE, NOT A GATE — and the difference cost a run.
     *
     * power_meter_read() demands voltage AND current both present AND valid,
     * which is the right rule for billing and the wrong one for a pre-flight on
     * a DEAD bus: with nothing energised there is no current to report, so a
     * perfectly healthy DCM3366 fails the check and the bench refuses on a meter
     * that is working. Observed exactly that on the first run here, against a
     * controller that was simultaneously reporting `meter: ok`.
     *
     * The real gate is further down and is the honest one: after the contactor
     * closes, does the meter SEE the bus? A run that cannot answer that is
     * skipped there, with the reason printed. */
    double mv = 0.0, ma = 0.0;
    if (power_meter_read(meter_ch, &mv, &ma) == 0) {
        printf("meter baseline: %.1f V / %.2f A with everything off\n", mv, ma);
    } else {
        printf("meter baseline: no fully-valid sample at rest (no current flowing —\n"
               "  expected on a dead bus). Proceeding; the close step below is what\n"
               "  actually proves the meter can see the vehicle side.\n");
    }

    uint32_t *falls = calloc((size_t)(runs > 0 ? runs : 1), sizeof(uint32_t));
    int measured = 0;

    for (int r = 1; r <= runs && !g_stop; ++r) {
        printf("\n--- run %d/%d ------------------------------------------\n", r, runs);

        /* 1. ENERGISE, then CLOSE. Order matters: closing onto a dead bus and
         *    then raising it is the inrush the cross-tie rules exist to avoid,
         *    and it is not what a real session does either. */
        printf("  energising to %.0f V @ %.1f A…\n", volts, amps);
        power_set(volts, amps);
        pump(&link, 2000);

        printf("  closing the gun contactor…\n");
        cbctl_op_t op;
        if (cbctl_relay_close(&link, CBPLC_RELAY_GUN,
                              CBCTL_RELAY_HOLD_RECOMMENDED_MS, &op) != CBCTL_OK) {
            fprintf(stderr, "  relay close REFUSED by cbctl; skipping this run\n");
            power_off();
            pump(&link, 500);
            continue;
        }

        /* 2. CONFIRM THE VEHICLE SIDE IS LIVE. Without this the timing below is
         *    measuring a bus that was never charged. */
        bool live = false;
        const uint32_t settle_until = now_ms() + settle_ms;
        while (!g_stop && (int32_t)(settle_until - now_ms()) > 0) {
            cbctl_tick(&link, now_ms());
            power_tick();
            if (power_meter_read(meter_ch, &mv, &ma) == 0 && mv > zero_v) {
                live = true;
                break;
            }
            nap_ms(poll_ms);
        }
        if (!live) {
            /* SPLIT THE TWO FAILURES. "The meter sees nothing" has two very
             * different causes and reporting them as one wastes a bench trip:
             * either the MODULE never produced the voltage (no AC input, an
             * interlock, a fault), or it did and the meter or the wiring between
             * them cannot see it. power_read() is the module's own account of
             * its output, so printing both sides names which. */
            double bv = 0.0, ba = 0.0;
            const int rr = power_read(&bv, &ba);
            printf("  meter never rose above %.0f V — nothing to time.\n", zero_v);
            if (rr == 0 && bv > zero_v) {
                printf("    but the MODULE reports %.1f V / %.2f A on its own output.\n"
                       "    So the module is delivering and the vehicle side is not seeing it:\n"
                       "    look at the gun contactor, the meter's CT/shunt, or the wiring\n"
                       "    between them — NOT at the module.\n", bv, ba);
            } else if (rr == 0) {
                printf("    and the MODULE reports only %.1f V on its own output, so it is not\n"
                       "    delivering. The meter is not the problem. Check AC input to the\n"
                       "    module, its enable/interlock, and any module fault it is reporting.\n",
                       bv);
            } else {
                printf("    and the module gave no readable output telemetry either.\n");
            }
            (void)cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL);
            power_off();
            pump(&link, 1000);
            continue;
        }
        printf("  meter sees %.1f V on the vehicle side\n", mv);

        /* 3. OPEN, AND TIME THE FALL. Modules OFF first, then the contactor —
         *    breaking current is what welds contacts, and it is also what the
         *    controller's own Curtail path exists to avoid. */
        power_off();
        const uint32_t t_open = now_ms();
        (void)cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL);

        uint32_t fell = 0;
        while (!g_stop && now_ms() - t_open < timeout_ms) {
            cbctl_tick(&link, now_ms());
            power_tick();
            if (power_meter_read(meter_ch, &mv, &ma) == 0 && mv <= zero_v) {
                fell = now_ms() - t_open;
                break;
            }
            nap_ms(poll_ms);
        }

        if (fell == 0) {
            printf("  STILL ABOVE %.0f V after %u ms (last %.1f V).\n"
                   "  On a real charger this is what a WELD looks like.\n",
                   zero_v, timeout_ms, mv);
        } else {
            printf("  vehicle side fell below %.0f V in %u ms\n", zero_v, fell);
            falls[measured++] = fell;
        }
        pump(&link, 1500);   /* settle before the next run */
    }

    printf("\n============================================================\n");
    printf("RESULT\n");
    if (measured == 0) {
        printf("  no completed measurements.\n");
    } else {
        qsort(falls, (size_t)measured, sizeof(uint32_t), cmp_u32);
        const uint32_t p95 = falls[(measured * 95) / 100 >= measured
                                       ? measured - 1
                                       : (measured * 95) / 100];
        printf("  runs measured        : %d\n", measured);
        printf("  open -> below %.0f V  : min=%u  median=%u  max=%u  (ms)\n",
               zero_v, falls[0], falls[measured / 2], falls[measured - 1]);
        printf("  p95                  : %u ms\n", p95);
        printf("\n");
        printf("  SafetyLimits::discharge_dwell_ms must EXCEED this plus the worst\n");
        printf("  meter sample age, with margin: this is a physical bleed-down and it\n");
        printf("  varies with pack voltage, cable length and temperature.\n");
    }
    free(falls);

done:
    /* Every path, including the signal handler's. */
    power_off();
    (void)cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL);
    for (int i = 0; i < 40; ++i) { cbctl_tick(&link, now_ms()); power_tick(); nap_ms(25); }
    power_close();
    cbctl_serial_close(&ser);
    printf("\nrelays open, modules off.\n");
    return 0;
}
