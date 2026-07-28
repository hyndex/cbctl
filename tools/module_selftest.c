/* Does cbmodules actually make these modules produce voltage? Nothing else in the loop.
 *
 * WHY THIS EXISTS
 *   On the bench cbmodules reports a group with active=2, confirmed_on=2, requested 356 V / 2 A —
 *   and the measured output stays at zero. Meanwhile a hand-written ENR encoder driving the same
 *   two modules over raw CAN commanded 100 V and read 151.3 V back, so the hardware is fine and
 *   the wire is fine. Those two facts cannot both be about the vehicle, the PLC, or the V2G stack,
 *   because neither was involved in the second one.
 *
 *   This tool removes everything else: no PLC link, no session, no contactor, no vehicle. It asks
 *   cbmodules for volts and reports what the modules say. If it produces voltage, the module drive
 *   is sound and the fault is upstream in the charge loop. If it does not, the fault is in how
 *   cbmodules is configured or driven, and every V2G-layer theory is a distraction.
 *
 * SAFETY
 *   The gun contactor is NOT touched and must be open — this tool never talks to the PLC, so it
 *   cannot close anything. Output therefore terminates at an open contactor exactly as it does
 *   during a normal precharge. Modules are commanded OFF on every exit path including signals, and
 *   the default setpoint is deliberately low.
 *
 *   ./module_selftest can0 200        # 200 V, ~1 A, 12 s
 */
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

extern int  power_open(const char *can_iface, const char *meter_dev, int meter_unit,
                       const unsigned char *addrs, int naddr);
extern void power_set(double volts, double amps);
extern void power_off(void);
extern void power_tick(void);
extern int  power_read(double *v, double *a);
extern int  power_modules_on(void);
extern int  power_available(void);
extern void power_group_debug(int *assigned, int *active, int *confirmed_on, int *faulted,
                              double *req_v, double *req_a);
extern void power_close(void);

static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

static const unsigned char kAddr[] = { 1u, 2u };

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + (uint32_t)(ts.tv_nsec / 1000000));
}

int main(int argc, char **argv)
{
    const char *canif = (argc > 1) ? argv[1] : "can0";
    const double volts = (argc > 2) ? atof(argv[2]) : 200.0;
    const double amps  = (argc > 3) ? atof(argv[3]) : 1.0;
    const uint32_t run_ms = (argc > 4) ? (uint32_t)(atof(argv[4]) * 1000.0) : 12000u;
    uint32_t t0, last = 0u;
    double peak = 0.0;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    printf("cbmodules self-test: can=%s target=%.0fV %.1fA for %.0fs "
           "(gun contactor untouched — this tool never talks to the PLC)\n",
           canif, volts, amps, run_ms / 1000.0);

    if (power_open(canif, "", 2, kAddr, (int)(sizeof kAddr / sizeof kAddr[0])) != 0) {
        fprintf(stderr, "power_open failed\n");
        return 1;
    }

    t0 = now_ms();
    for (;;) {
        const uint32_t t = now_ms();
        double v = 0.0, a = 0.0;
        int ok, assigned = 0, active = 0, on = 0, faulted = 0;
        double rv = 0.0, ra = 0.0;

        if (g_stop || (t - t0) > run_ms) {
            break;
        }

        power_tick();
        power_set(volts, amps);
        ok = (power_read(&v, &a) == 0);
        power_group_debug(&assigned, &active, &on, &faulted, &rv, &ra);
        if (ok && v > peak) {
            peak = v;
        }

        if (t - last >= 2000u) {
            printf("[%5.1fs] telemetry=%-6s V=%-10.1f A=%-6.1f | grp a=%d act=%d on=%d flt=%d "
                   "req=%.0fV/%.1fA | avail=%d\n",
                   (t - t0) / 1000.0, ok ? "fresh" : "STALE", v, a,
                   assigned, active, on, faulted, rv, ra, power_available());
            last = t;
        }
        usleep(50000);
    }

    printf("\ncommanding OFF\n");
    power_off();
    for (int i = 0; i < 10; i++) { power_tick(); usleep(50000); }
    power_close();

    printf("peak measured %.1f V\n", peak);
    printf("%s\n", peak > 20.0
        ? "RESULT: cbmodules DOES drive these modules — the fault is upstream in the charge loop."
        : "RESULT: cbmodules produced NO voltage — the fault is in the module setup/drive itself.");
    return 0;
}
