/* Drive ONE module address at a chosen voltage, so a bench can prove which meter
 * loop is physically wired to which gun.
 *
 * WHY THIS EXISTS. module_selftest drives every configured module at the SAME
 * setpoint, which cannot distinguish two guns: both meters then read the same
 * number and any mapping looks self-consistent. Running module 1 at 200 V and
 * module 2 at 320 V with the merge tie (relay 2) OPEN makes the two buses
 * genuinely different, so whichever Modbus unit reports 200 V is, by
 * construction, the meter on module 1's gun. There is no inference left.
 *
 * SAFETY. This tool does not talk to the PLC and cannot move a contactor. The
 * operator closes the gun contactor separately and must leave relay 2 open --
 * closing it merges the buses and destroys the whole point of the test. Modules
 * are commanded OFF on every exit path including signals.
 *
 *   ./module_one can0 1 200 1.0 30      # addr 1, 200 V, 1 A, 30 s
 */
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

extern int  power_open(const char *can_iface, const char *meter_dev, int meter_unit,
                       const unsigned char *addrs, int naddr);
extern void power_set(double volts, double amps);
extern void power_off(void);
extern void power_tick(void);
extern int  power_read(double *v, double *a, int *assigned, int *active, int *on, int *faulted,
                       double *req_v, double *req_a);
extern void power_close(void);

static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + (uint32_t)(ts.tv_nsec / 1000000));
}

int main(int argc, char **argv)
{
    const char *canif = (argc > 1) ? argv[1] : "can0";
    const unsigned char addr = (argc > 2) ? (unsigned char)atoi(argv[2]) : 1u;
    const double volts = (argc > 3) ? atof(argv[3]) : 200.0;
    const double amps  = (argc > 4) ? atof(argv[4]) : 1.0;
    const uint32_t run_ms = (argc > 5) ? (uint32_t)(atof(argv[5]) * 1000.0) : 30000u;
    uint32_t t0, last = 0u;

    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);
    printf("module_one: can=%s addr=%u target=%.0fV %.1fA for %.0fs\n",
           canif, addr, volts, amps, run_ms / 1000.0);

    if (power_open(canif, "", 2, &addr, 1) != 0) {
        fprintf(stderr, "power_open failed\n");
        return 1;
    }
    t0 = now_ms();
    for (;;) {
        const uint32_t t = now_ms();
        double v = 0.0, a = 0.0, rv = 0.0, ra = 0.0;
        int assigned = 0, active = 0, on = 0, faulted = 0;
        if (g_stop || (t - t0) >= run_ms) break;
        power_set(volts, amps);
        power_tick();
        if ((t - last) >= 1000u) {
            last = t;
            if (power_read(&v, &a, &assigned, &active, &on, &faulted, &rv, &ra) == 0) {
                printf("  addr=%u t=%2us measured=%6.1fV/%5.2fA assigned=%d active=%d on=%d faulted=%d\n",
                       addr, (t - t0) / 1000u, v, a, assigned, active, on, faulted);
                fflush(stdout);
            }
        }
        struct timespec ns = {0, 20 * 1000 * 1000};
        nanosleep(&ns, NULL);
    }
    power_off();
    power_tick();
    power_off();
    power_close();
    printf("module_one: addr=%u commanded OFF\n", addr);
    return 0;
}
