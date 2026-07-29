/* A real DC charge driven by cbctl against cbplc, stopping at a target SoC.
 *
 * This is the end-to-end test of OUR stack: cbctl (this library) talking to cbplc (our
 * firmware), driving the real Maxwell ENR modules over SocketCAN and reading the real DCM3366
 * meter, with the stop condition taken from the vehicle's OWN reported state of charge.
 *
 * WHY THE SOC STOP IS THE POINT
 *   cbplc publishes `ev_soc` — the EVRESSSOC the car sends in every DC request. plc_firmware
 *   never extracted it, so nothing built on that firmware could stop at a SoC target, display
 *   one, or bill against one. Here the target is enforced from the vehicle's own number.
 *
 *   cbctl_ev_soc_pct() returns -1 when the car has not reported, and this program refuses to
 *   start delivering until it has: a controller that treats "unknown" as 0 %% will happily
 *   charge a full battery forever.
 *
 * SAFETY
 *   Modules are commanded off before anything else on every exit path — normal, error, or
 *   signal. Measurements fed to cbplc are MEASURED, never synthesised: if module telemetry
 *   goes stale the envelope drops power_ready rather than guessing, because a fabricated
 *   voltage is how a charger energises into a fault it cannot see.
 *
 *   make charge && ./build/charge_soc <plc-port> <can-if> <max_amps> <soc_delta_pct>
 */
#include "cbctl/cbctl.h"
#include "posix_serial.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* ---- power: the SHIPPING drivers, not a bench re-implementation -------------------------
 *
 * This used to be a hand-written ENR encoder. It ACKed, it reported a perfect setpoint, and it
 * never sent the output-enable — the modules held 0 V while every log line looked healthy, and
 * the failure surfaced as "the vehicle will not charge". cbmodules cannot lose the enable,
 * because enable is a FIELD of the target rather than a separate message, and it is the code
 * the fleet actually runs. cbmeter reads the DCM3366 the same way production does.
 *
 * See tools/power_cbm.cpp. Declared here rather than in a header: five functions, one caller. */
extern int  power_open(const char *can_iface, const char *meter_dev, int meter_unit,
                       const unsigned char *addrs, int naddr);
extern void power_set(double volts, double amps);
extern void power_off(void);
extern void power_tick(void);
extern int  power_read(double *v, double *a);        /* 0 = trustworthy; -1 = withhold, NOT 0 V */
extern int  power_modules_on(void);
extern int  power_available(void);   /* plant can supply, regardless of idle telemetry */
extern void power_group_debug(int *assigned, int *active, int *confirmed_on, int *faulted,
                              double *req_v, double *req_a);
extern int  power_meter_read(int channel, double *v, double *a);
extern void power_close(void);

/* Monotonic. Never wall clock: a charge integrates energy over elapsed time, and an NTP step
 * mid-session would invent or erase kilowatt-hours. */
static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + (uint32_t)(ts.tv_nsec / 1000000));
}

/* WHICH MODULES FEED *THIS* GUN — and it is not "all of them".
 *
 * This bench has one 30 kW ENR per gun: module 1 is wired to gun 1, module 2 to gun 2. Both
 * were being put in one group, and cbmodules SPLITS a group's current target evenly across its
 * members — so a 20 A ask became 10 A per module, and module 2's half went into gun 2's open
 * contactor. Measured mid-charge with a read-only per-module poll:
 *
 *     module 1: V=360.0  A=10.0     <- in circuit, sagged to the pack
 *     module 2: V=389.1  A=0.002    <- at its setpoint, no load at all
 *
 * So the charger delivered exactly HALF of what it advertised, and the missing half was not
 * lost anywhere — it was allocated to a module that has no path to this vehicle. On a real
 * charger the group is per-gun for this reason; the bench had quietly become a two-gun group
 * driving one gun.
 *
 * Default stays {1,2} so nothing changes for a single-gun bus; pass the list to scope it. */
static unsigned char kModuleAddr[8] = { 1u, 2u };

/* WHICH DCM3366 LOOP MEASURES THIS GUN, and getting it wrong is silent.
 *
 * The meter is two-loop — ch1 voltage at 0x203C, ch2 at 0x203E — and the loops are per gun.
 * This was hardcoded to 1, which was right on gun 1 and catastrophically wrong on gun 2: the
 * meter reported the OTHER gun's idle loop, so the vehicle was told the bus sat at 0 V for the
 * whole of PreCharge, never saw it converge, and dropped the session. The contactor then opened
 * and re-closed on every retry — audible relay chatter whose real cause was a meter channel. */
static int g_meter_ch = 1;

/* Fleet mode (CBCTL_SIBLING_PORT). File scope because the wind-down loops below need them too;
 * a charge that stopped servicing the sibling the moment it finished would not be modelling
 * anything real. */
static cbctl_serial_t g_sib_ser;
static cbctl_link_t   g_sib_link;
static cbctl_fleet_t  g_fleet;
static int            g_fleet_on;
static uint32_t       g_sib_worst_feed_gap;   /* worst gap between sibling STATUS observations */
static uint32_t       g_sib_last_obs;
static uint32_t       g_sib_quarantines;
static uint32_t       g_sib_kill_s;          /* CBCTL_SIBLING_KILL_S: yank its USB mid-charge */
static int            g_sib_killed;
static uint32_t       g_worst_gap_after_kill; /* the charging link's worst tick gap AFTER it */
static int g_meter_ok;
static double g_meter_v, g_meter_a;
static unsigned g_meter_mismatch;
static int g_nmod = 2;
#define NMOD (g_nmod)

/* "1" or "1,2" — anything else is refused rather than silently reinterpreted, because getting
 * this wrong drives the WRONG GUN's module, which is the one mistake here with a safety edge. */
static int parse_modules(const char *s)
{
    int n = 0;
    if (s == NULL || *s == '\0') { return 0; }
    while (*s != '\0' && n < (int)(sizeof kModuleAddr)) {
        char *end = NULL;
        long v = strtol(s, &end, 10);
        if (end == s || v < 1 || v > 127) { return -1; }
        kModuleAddr[n++] = (unsigned char)v;
        s = end;
        if (*s == ',') { s++; } else if (*s != '\0') { return -1; }
    }
    g_nmod = n;
    return n;
}

/* WHICH BOARD IS THIS, REALLY.
 *
 * USB enumeration order is not stable, and reflashing two boards is enough to swap them:
 * measured on this bench within one session, ttyACM0 went from D0:CF:13:73:0A:68 to
 * D0:CF:13:71:FF:54. Everything inferred from the old mapping — which gun, therefore which
 * module, therefore which meter loop — was then wrong, and the failure presented as "the
 * modules produce nothing" rather than as "you are talking to the other gun".
 *
 * cbctl has carried cbctl_expect_mac() for exactly this since the dual-board work; this tool
 * simply was not using it. Set CBCTL_EXPECT_MAC and a port swap becomes a refusal at start-up
 * instead of a charge delivered to the gun nobody asked for. */
static int parse_mac(const char *s, uint8_t out[6])
{
    int i;
    unsigned b[6];
    if (s == NULL) { return -1; }
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) {
        return -1;
    }
    for (i = 0; i < 6; i++) {
        if (b[i] > 0xFFu) { return -1; }
        out[i] = (uint8_t)b[i];
    }
    return 0;
}

/* ---- main ------------------------------------------------------------------------------- */

static volatile sig_atomic_t g_stop;
static void on_sig(int s) { (void)s; g_stop = 1; }

/* CBCTL_TRACE_STATUS=<substr> prints the board's own status line, filtered. A charge that
 * stalls is nearly always a field cbctl does not parse. */
/* ---- session recorder: every wire line, every decision, one ordered file --------------------
 *
 * WHY
 *   Post-mortem on a failed session has repeatedly been impossible because the evidence was gone:
 *   the board's STATUS is ~2.5 KB of state at poll cadence, cbctl parses a deliberate subset, and
 *   the console kept only what someone thought to print. Every hard bug in this bring-up was
 *   already visible on the wire before it was diagnosed — `limits_refusal=contactor_open`,
 *   `auth=0`, ALL_SET `byte1=0x01` — and each cost hours because nothing recorded it.
 *
 *   So: append the RAW status line, verbatim, with a monotonic timestamp, plus our own decisions
 *   interleaved in the same stream. Raw rather than parsed, because the field that explains the
 *   NEXT vehicle is one nobody has thought to parse yet. Plain text so it needs no decoder and
 *   greps and diffs across vehicles.
 *
 * COST
 *   ~2.5 KB per poll, ~1 poll/s => ~9 MB/hour, on a Pi with a disk. Off unless CBCTL_SESSION_LOG
 *   names a file, because a bench tool that silently fills a production disk is its own incident. */
/* ---- the board's own log, pulled over the protocol ------------------------------------------
 *
 * CBCTL_DIAG_LOG=<file> arms it; CBCTL_DIAG_V2G=1 additionally turns on cbplc's per-message
 * decoded HLC trace, which is the thing to reach for when a PARTICULAR vehicle refuses a
 * charger that works with every other one.
 *
 * Off by default and deliberately so. The trace is roughly 100 bytes a message and 40-60
 * messages a session, and cbctl only drains between sessions — never while a contactor is held
 * — so arming it costs nothing during delivery and everything is still there afterwards,
 * because the board's ring holds the whole session.
 *
 * The sink appends raw bytes and nothing else. No line assembly: a drain boundary can fall
 * mid-line, and re-framing here would mean buffering on the control thread for no benefit that
 * `tail -f` does not already provide. */
static FILE *g_diaglog;

static void diag_sink(const uint8_t *bytes, uint16_t len, void *user)
{
    (void)user;
    if (g_diaglog != NULL && len != 0u) {
        (void)fwrite(bytes, 1u, len, g_diaglog);
    }
}

static FILE *g_rec;
static uint32_t g_rec_t0;

static void rec_open(const char *path)
{
    if (path == NULL || path[0] == '\0') { return; }
    g_rec = fopen(path, "we");
    if (g_rec == NULL) {
        fprintf(stderr, "session log: cannot open %s: %s\n", path, strerror(errno));
        return;
    }
    g_rec_t0 = now_ms();
    setvbuf(g_rec, NULL, _IOLBF, 0);   /* line-buffered: a crash must not eat the last seconds */
}

static void rec_event(const char *kind, const char *fmt, ...)
{
    va_list ap;
    if (g_rec == NULL) { return; }
    fprintf(g_rec, "%8u EVT %-10s ", now_ms() - g_rec_t0, kind);
    va_start(ap, fmt);
    vfprintf(g_rec, fmt, ap);
    va_end(ap);
    fputc('\n', g_rec);
}

static void rec_status(const char *line, size_t len)
{
    if (g_rec == NULL) { return; }
    fprintf(g_rec, "%8u STS %.*s\n", now_ms() - g_rec_t0, (int)len, line);
}

/* ---- phase watchdog: cbplc's own trail, with a deadline on every phase ----------------------
 *
 * WHY THIS EXISTS
 *   Three separate hunts in this bring-up ended the same way: a session died, and the only
 *   evidence was "no volts" twenty seconds later. cbplc actually says exactly what it is stuck
 *   on — `substate=`, `furthest=`, and above all `limits_refusal=`, which names the ONE gate in
 *   its energy chain that is failing (limits_stale -> power_not_ready -> isolation_unknown ->
 *   contactor_open). None of it was being read, so every diagnosis cost a bench cycle.
 *
 *   cbctl deliberately parses only the subset of STATUS it needs, which is right for a library
 *   and useless for a bench. So the raw observer scrapes the three fields that attribute a
 *   failure, and a per-phase deadline turns "it hung" into "it hung in PreCharge for 30 s with
 *   limits_refusal=contactor_open" — printed at the moment it happens, not inferred afterwards. */
static char g_substate[32], g_furthest[32], g_refusal[32], g_chg_reason[32];
static uint32_t g_substate_since;

static void scrape(const char *line, size_t len, const char *key, char *out, size_t cap)
{
    const size_t kl = strlen(key);
    size_t i, n = 0u;
    out[0] = '\0';
    if (len < kl) { return; }
    for (i = 0; i + kl <= len; i++) {
        if (memcmp(line + i, key, kl) != 0) { continue; }
        if (i != 0 && line[i - 1] != ' ') { continue; }   /* anchored: no substring hits */
        i += kl;
        while (i < len && line[i] != ' ' && n + 1u < cap) { out[n++] = line[i++]; }
        out[n] = '\0';
        return;
    }
}

static void trace_status(const char *line, size_t len, void *ud)
{
    char sub[32];
    const char *want = (const char *)ud;

    rec_status(line, len);
    scrape(line, len, "substate=", sub, sizeof sub);
    if (sub[0] != '\0' && strcmp(sub, g_substate) != 0) {
        snprintf(g_substate, sizeof g_substate, "%s", sub);
        g_substate_since = now_ms();
    }
    scrape(line, len, "furthest=", g_furthest, sizeof g_furthest);
    scrape(line, len, "limits_refusal=", g_refusal, sizeof g_refusal);
    scrape(line, len, "chg_reason=", g_chg_reason, sizeof g_chg_reason);

    if (want == NULL || want[0] == '\0') { return; }
    {
        const size_t wl = strlen(want);
        size_t i;
        int hit = 0;
        for (i = 0; wl <= len && i + wl <= len; i++) {
            if (memcmp(line + i, want, wl) == 0) { hit = 1; break; }
        }
        if (!hit) { return; }
    }
    printf("  STATUS: %.*s\n", (int)len, line);
}

static unsigned bad_readings;
static int announced_close, announced_wait, announced_idle;
static int g_ga, g_gact, g_gon, g_gflt;
static double g_grv, g_gra;

static uint8_t g_relay_mask = CBPLC_RELAY_GUN; /* see the close site for why not RELAY_ALL */

int main(int argc, char **argv)
{
    cbctl_serial_t ser;
    cbctl_link_t link;
    const char *port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const char *canif = (argc > 2) ? argv[2] : "can0";
    const double max_a = (argc > 3) ? atof(argv[3]) : 20.0;
    const int soc_delta = (argc > 4) ? atoi(argv[4]) : 5;
    /* Empty means "no meter": the bench still charges from module telemetry, it just cannot
     * claim METER_MEASURED. A missing meter is not a reason to refuse to charge. */
    const char *meterdev = (argc > 5) ? argv[5] : "";
    /* argv[6]: the module addresses wired to THIS gun. See kModuleAddr. */
    /* argv[7]: the meter loop. Defaults to the first module address, which is the per-gun
     * convention on this bench (module 1 -> gun 1, module 2 -> gun 2). */
    if (argc > 6 && parse_modules(argv[6]) <= 0) {
        fprintf(stderr, "bad module list '%s' — expected e.g. 1 or 1,2\n", argv[6]);
        return 2;
    }
    g_meter_ch = (argc > 7) ? atoi(argv[7]) : (int)kModuleAddr[0];
    if (g_meter_ch < 1 || g_meter_ch > 2) {
        fprintf(stderr, "meter channel must be 1 or 2 (the DCM3366 has two loops)\n");
        return 2;
    }

    int start_soc = -1, last_soc = -1, target_soc = -1;
    double peak_v = 0.0, peak_a = 0.0, wh = 0.0;
    uint32_t t0, last_report = 0u, last_wh = 0u;
    int rc = 1;
    /* FAST-FAIL TIMEOUTS. A session test that only ends on the operator's patience burns
     * vehicle time and cycles contactors for nothing. Each of these names a specific thing
     * that should have happened by now, so a failure reports WHAT did not arrive. */
    uint32_t t_session = 0u, t_volts = 0u, t_demand = 0u, t_gun_open = 0u;
    uint8_t expect_mac[6];
    int want_mac = 0, mac_checked = 0;
    int bailed = 0;
    const uint32_t kNoSessionMs = 180000u;  /* room for a few replug retries before giving up */
    /* A vehicle that has tried and failed backs off for MINUTES before trying again, so a bench
     * that only waits is measuring the vehicle's patience, not the charger. cbplc's own Python
     * harness opens every cycle with a pilot replug for exactly this reason; driving CP to E and
     * releasing makes an already-plugged car re-initiate SLAC immediately. */
    const uint32_t kReplugEveryMs = 40000u;
    /* One phase should never take this long. The reference's own per-phase budgets are seconds;
     * 30 s is generous enough not to false-fire and short enough to free the bench. */
    const uint32_t kPhaseStallMs = 90000u;
    /* A close intent may be (re)created at most this often. Slower than the board's own gate
     * changes, fast enough that a genuine session never waits noticeably for its contactor. */
    const uint32_t kRearmFloorMs = 2000u;
    uint32_t t_rearm = 0u;
    uint32_t last_mod_rec = 0u;
    uint32_t t_replug = 0u;
    unsigned replugs = 0u;
    const uint32_t kNoVoltsMs = 20000u;     /* modules must answer a setpoint quickly */
    const uint32_t kNoSocMs = 120000u;      /* a delivering EV reports SoC continuously */

    /* Line-buffer stdout: redirected to a file it is block-buffered by default, and a charge
     * log that only appears after the process dies is useless exactly when it is needed. */
    {
        const char *em = getenv("CBCTL_EXPECT_MAC");
        if (em != NULL && em[0] != '\0') {
            if (parse_mac(em, expect_mac) != 0) {
                fprintf(stderr, "bad CBCTL_EXPECT_MAC '%s' (want aa:bb:cc:dd:ee:ff)\n", em);
                return 2;
            }
            want_mac = 1;
        }
    }

    setvbuf(stdout, NULL, _IOLBF, 0);
    /* AFTER setvbuf, never before. setvbuf is only valid before any other operation on the
     * stream, so a printf above it silently leaves stdout fully buffered — and a charge whose
     * output appears only when the process dies is invisible exactly while it is being watched.
     * Cost me one bench cycle. */
    {
        int mi;
        printf("meter loop: ch%d   modules:", g_meter_ch);
        for (mi = 0; mi < NMOD; mi++) { printf("%s%u", mi ? "," : " ", kModuleAddr[mi]); }
        printf("\n");
    }
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    if (power_open(canif, meterdev, 2, kModuleAddr, NMOD) != 0) {
        fprintf(stderr, "power_open failed (can=%s meter=%s)\n", canif, meterdev);
        return 2;
    }
    power_off();

    if (!cbctl_serial_open(&ser, port, 115200u) || cbctl_open(&link, &ser.tx) != CBCTL_OK) {
        fprintf(stderr, "cannot attach %s\n", port);
        power_off();
        return 2;
    }

    /* FLEET MODE — the other gun's board sharing this one control loop.
     *
     * The question this exists to answer is not "does the library have a fleet API" but "does a
     * REAL charge degrade when a second PLC is being serviced by the same thread". Everything
     * else here is unchanged, so the run is directly comparable against a single-link baseline:
     * same tool, same charge, same counters, one extra board on the loop.
     *
     * The sibling is deliberately given NO vehicle and NO relay. That is the case the operator
     * actually has most of the time — one gun charging, one gun sitting idle — and it is the one
     * where an idle link's STATUS polling could quietly eat the charging link's wire time. */
    {
        const char *sib = getenv("CBCTL_SIBLING_PORT");
        if (sib != NULL && sib[0] != '\0') {
            if (!cbctl_serial_open(&g_sib_ser, sib, 115200u) ||
                cbctl_open(&g_sib_link, &g_sib_ser.tx) != CBCTL_OK) {
                fprintf(stderr, "cannot attach sibling %s\n", sib);
                power_off();
                return 2;
            }
            cbctl_fleet_init(&g_fleet);
            if (cbctl_fleet_add(&g_fleet, &link) != CBCTL_OK ||
                cbctl_fleet_add(&g_fleet, &g_sib_link) != CBCTL_OK) {
                fprintf(stderr, "cannot build fleet\n");
                power_off();
                return 2;
            }
            g_fleet_on = 1;
            g_sib_kill_s = (uint32_t)atoi(getenv("CBCTL_SIBLING_KILL_S") ?
                                          getenv("CBCTL_SIBLING_KILL_S") : "0");
            printf("FLEET MODE: charging link + sibling %s on ONE loop%s\n", sib,
                   g_sib_kill_s ? " (sibling transport will be KILLED mid-charge)" : "");
        }
    }
    /* AFTER cbctl_open — it zeroes the link, so a hook installed before it would vanish. */
    {
        const char *tr = getenv("CBCTL_TRACE_STATUS");
        rec_open(getenv("CBCTL_SESSION_LOG"));
        {
            /* PROVISIONING, declared by this process exactly as a real controller does at
             * link-up. Putting it here rather than in a separate tool is what makes one run an
             * end-to-end test of the whole stack: the identity the vehicle is told, the trim the
             * SLAC front end uses, and the charge itself all come from the same link, and the
             * declaration is re-asserted automatically if the board resets mid-run. */
            {
                const char *id = getenv("CBCTL_EVSE_ID");
                const char *trim_s = getenv("CBCTL_ATTN_TRIM");
                const char *pr = getenv("CBCTL_PROTOCOLS");
                if (id != NULL && id[0] != '\0') {
                    if (cbctl_provision_evse_id(&link, id) != CBCTL_OK) {
                        fprintf(stderr, "bad CBCTL_EVSE_ID '%s' (need %u..%u printable chars)\n",
                                id, (unsigned)CBPLC_EVSE_ID_MIN, (unsigned)CBPLC_EVSE_ID_MAX);
                        power_off();
                        return 2;
                    }
                    printf("provisioning: evse_id=%s\n", id);
                }
                if (trim_s != NULL && trim_s[0] != '\0') {
                    if (cbctl_provision_attn_trim_db(&link, (int8_t)atoi(trim_s)) != CBCTL_OK) {
                        fprintf(stderr, "bad CBCTL_ATTN_TRIM '%s' (need %d..%d dB)\n", trim_s,
                                CBPLC_ATTN_TRIM_MIN, CBPLC_ATTN_TRIM_MAX);
                        power_off();
                        return 2;
                    }
                    printf("provisioning: attn_trim=%s dB\n", trim_s);
                }
                if (pr != NULL && pr[0] != '\0') {
                    uint8_t m = 0u;
                    if (strstr(pr, "din") != NULL) { m |= CBCTL_PROTO_DIN70121; }
                    if (strstr(pr, "iso") != NULL) { m |= CBCTL_PROTO_ISO2; }
                    if (m == 0u || cbctl_provision_protocols(&link, m) != CBCTL_OK) {
                        fprintf(stderr, "bad CBCTL_PROTOCOLS '%s' (want din, iso2 or din,iso2)\n",
                                pr);
                        power_off();
                        return 2;
                    }
                    printf("provisioning: protocols=%s\n", pr);
                }
            }
            const char *dl = getenv("CBCTL_DIAG_LOG");
            if (dl != NULL && dl[0] != '\0') {
                g_diaglog = fopen(dl, "we");
                if (g_diaglog == NULL) {
                    fprintf(stderr, "diag log: cannot open %s: %s\n", dl, strerror(errno));
                } else {
                    const char *v2g = getenv("CBCTL_DIAG_V2G");
                    const int want_v2g = (v2g != NULL && v2g[0] == '1');
                    /* Line-buffered for the same reason as the session recorder: a log that
                     * only appears after the process dies is useless exactly when it matters. */
                    setvbuf(g_diaglog, NULL, _IOLBF, 0);
                    (void)cbctl_diag_enable(&link, true, diag_sink, NULL);
                    /* Start from an empty ring so this session's trace cannot be prefixed with
                     * the tail of the previous vehicle's. */
                    (void)cbctl_diag_clear(&link);
                    (void)cbctl_diag_set_level(&link, CBCTL_DIAG_INFO);
                    (void)cbctl_diag_set_categories(
                        &link, want_v2g ? (CBCTL_DIAG_CAT_CORE | CBCTL_DIAG_CAT_V2G |
                                           CBCTL_DIAG_CAT_SLAC | CBCTL_DIAG_CAT_CP |
                                           CBCTL_DIAG_CAT_RELAY)
                                        : (CBCTL_DIAG_CAT_CORE | CBCTL_DIAG_CAT_SLAC |
                                           CBCTL_DIAG_CAT_CP | CBCTL_DIAG_CAT_RELAY));
                    printf("board log -> %s%s\n", dl,
                           want_v2g ? "  (V2G trace armed)" : "");
                }
            }
        }
        /* The observer is what feeds BOTH the phase watchdog and the recorder, so it must be
         * installed whenever either is wanted — not only when a human asked for console tracing. */
        if (tr != NULL || g_rec != NULL) {
            link.status_observer = trace_status;
            link.status_observer_ud = (void *)(tr ? tr : "");
        }
        if (g_rec != NULL) {
            rec_event("start", "port=%s can=%s ceiling=%.0fA target=+%d%%",
                      port, canif, max_a, soc_delta);
        }
    }

    printf("cbctl -> cbplc charge: port=%s can=%s ceiling=%.0fA target=+%d%% SoC\n",
           port, canif, max_a, soc_delta);

    t0 = now_ms();
    while (!g_stop) {
        cbctl_envelope_t env;
        cbctl_measurement_t meas;
        cbctl_event_t ev;
        struct pollfd pfd;
        double v = 0.0, a = 0.0;
        int ok = 0, soc;
        const uint32_t t = now_ms();

        power_tick();
        power_group_debug(&g_ga, &g_gact, &g_gon, &g_gflt, &g_grv, &g_gra);
        /* RECORD THE MODULE SIDE TOO. The recorder captured the PLC wire and nothing else, so a
         * session log could show us commanding 355 V @ 2 A and still not answer "did the modules
         * do anything". Both halves or neither: the whole point is that the file explains the
         * failure without a rerun. Once a second is plenty and keeps the file readable. */
        if (g_rec != NULL && (last_mod_rec == 0u || (t - last_mod_rec) >= 1000u)) {
            double mv = 0.0, ma = 0.0;
            const int mok = (power_read(&mv, &ma) == 0);
            rec_event("modules", "telemetry=%s V=%.1f A=%.2f | assigned=%d active=%d on=%d "
                                 "faulted=%d req=%.1fV/%.2fA",
                      mok ? "fresh" : "STALE", mv, ma, g_ga, g_gact, g_gon, g_gflt, g_grv, g_gra);
            last_mod_rec = t;
        }
        /* power_read() writes its outputs ONLY on success, so they must be reset every pass. They were
         * not, and on the first iteration that read uninitialised stack — CBCTL_MV()'s range
         * assertion fired on a garbage millivolt value. That assertion is the only reason this
         * did not become an invented measurement sent to a vehicle, which is the exact failure
         * the withhold-don't-fabricate rule exists to prevent.
         *
         * Zero is the honest value here rather than a fabrication: when telemetry is unavailable
         * the modules are off and not producing. `ok` still carries "did we actually measure
         * this", and power_ready/isolation are derived from their own sources. */
        v = 0.0;
        a = 0.0;
        /* THE MODULES ARE THE CONTROL SOURCE. THE METER IS THE WITNESS.
         *
         * This corrects my own earlier change, and both halves are worth recording because each
         * looked right at the time.
         *
         * I made the METER primary after watching cbmodules report 389.1 V / 0.0 A — its own
         * commanded setpoint — through a whole CurrentDemand while the DCM3366 measured
         * 359 V / 9.9 A. That was real, but the CAUSE was that cbmodules' runtime telemetry
         * probing defaults to disabled (runtime_probe_interval_ms = 0) and this bench had never
         * enabled it, so the manager never asked the modules anything after the startup scan.
         * Enabling it was the fix. Promoting the meter on top of that treated the symptom.
         *
         * And it broke the thing it was meant to protect. The meter sits downstream of the
         * contactor and reads through a CT: during PRECHARGE there is no current, so it reports
         * 0 V while the module already sits at the vehicle's requested 355.0 V. We sent that
         * zero to the car as EVSEPresentVoltage, the car waited for a bus that as far as it could
         * tell never came up, and dropped the session. Measured, verbatim:
         *
         *     src=meter 0.0V 0.0A | mod ok 355.0V 0.0A   req=355V/2A
         *
         * The contactor then opened and re-closed on every retry — the audible relay chatter,
         * whose real cause was a measurement source that cannot see the phase it was asked about.
         *
         * So: the MODULES answer the vehicle. They are the converter, they know their own output
         * within one poll, and they can see it with no current flowing, which is exactly what
         * PreCharge is. The METER stays as an independent instrument on the other side of the
         * contactor and does what it is good for: proving the two agree once current IS flowing.
         * A disagreement is REPORTED rather than silently resolved, because "modules 355 V, meter
         * 0 V" means something different in PreCharge (normal) than in delivery (a broken
         * circuit), and only a human knows which. */
        v = 0.0;
        a = 0.0;
        ok = (power_read(&v, &a) == 0);

        /* The meter, throttled: a synchronous 9600-baud Modbus read on the control tick is how
         * this project once starved a fleet's PreCharge. It never drives the loop now. */
        {
            static uint32_t mtr_at;
            double nv = 0.0, na = 0.0;
            if (mtr_at == 0u || (t - mtr_at) >= 400u) {
                g_meter_ok = (power_meter_read(g_meter_ch, &nv, &na) == 0);
                if (g_meter_ok) { g_meter_v = nv; g_meter_a = na; }
                mtr_at = (t == 0u) ? 1u : t;
            }
            if (ok && g_meter_ok && a > 2.0 &&
                (g_meter_v < v * 0.8 || g_meter_v > v * 1.2)) {
                if (g_meter_mismatch++ < 3u) {
                    printf("  WARN: modules %.1fV vs meter ch%d %.1fV at %.1fA — the two "
                           "instruments disagree with current flowing; check the meter loop\n",
                           v, g_meter_ch, g_meter_v, a);
                }
            }
        }
        if (ok) {
            if (v > peak_v) { peak_v = v; }
            if (a > peak_a) { peak_a = a; }
            if (last_wh != 0u) {
                wh += v * a * (double)(t - last_wh) / 3600000.0;
            }
            last_wh = t;
        } else {
            last_wh = 0u;
        }

        /* THE FEED. Measured values only — and if the modules have gone quiet, power_ready
         * drops rather than a number being invented.
         *
         * ENVELOPE readiness and ISOLATION are two different questions, and conflating them
         * deadlocks the session. This tool used to send
         *
         *     meas.isolation = ok ? CBCTL_ISO_VALID : CBCTL_ISO_UNKNOWN;
         *
         * where `ok` means "module telemetry is fresh". That is circular and it never resolves:
         * isolation stays UNKNOWN until modules are live -> modules only go live once commanded
         * a target -> the target comes from the vehicle -> the vehicle only sends one after
         * CableCheck -> CableCheck needs isolation. Observed on hardware as `dcreq ... fresh=1
         * iso=0` repeating until the vehicle gave up, with the session never leaving Negotiating.
         *
         * Isolation is the verdict of an INSULATION MONITOR, not a statement about how recently
         * a power module answered a poll. There is no IMD on this bench, so the controller
         * asserts it as its own decision — exactly what the reference harness does
         * (plc_real_charge.py sends isolation_result=2 with provenance ISOLATION_CONTROLLER_PROXY
         * on every PreCharge and Delivery sample, ungated). Saying so in one place, loudly, is
         * honest; deriving it from an unrelated signal is not.
         *
         * power_ready KEEPS its gate: that one really is "can we deliver right now", and a stale
         * module is a genuine reason to answer no. */
        memset(&env, 0, sizeof env);
        env.max_voltage = CBCTL_MV(500000u);
        env.min_voltage = CBCTL_MV(150000u);
        env.max_current = CBCTL_MA((uint32_t)(max_a * 1000.0));
        env.max_power = CBCTL_W(60000u);
        /* THE SAME CIRCULAR TRAP AS ISOLATION, AND IT WAS STILL HERE.
         *
         * power_ready was `ok` — "module telemetry is fresh". But this field is advertised to the
         * vehicle during ChargeParameterDiscovery, which happens BEFORE any module has been asked
         * to produce anything, so it is always stale at exactly the moment it is read. The car
         * then sits re-asking ChargeParameterDiscoveryReq forever, waiting for an EVSE that keeps
         * saying it cannot supply power. Measured: `dcreq: type=18` (CHARGE_PARAMETER_DISCOVERY_REQ)
         * repeating for the whole session with finished=1 iso=2 contactor=2 — every other input
         * correct, and this one telling the vehicle not to bother.
         *
         * "Ready" is a property of the PLANT: are there modules allocated and unfaulted. That is
         * knowable at start-up and true while idle, which is the whole point. Whether the last
         * telemetry poll was fresh belongs to the measurement, and the measurement already says
         * so — that is not this field's job. */
        env.power_ready = (power_available() != 0);
        (void)cbctl_set_envelope(&link, &env);

        memset(&meas, 0, sizeof meas);
        /* CLAMP, AND SAY SO. cbmodules' combined_voltage_v came back past the protocol ceiling
         * and CBCTL_MV()'s assertion killed the process mid-session. Both halves of that were
         * wrong for a bench: a wild reading must not abort a run that is holding a contactor
         * closed, and it certainly must not be forwarded to a vehicle as a measurement. Clamp to
         * the wire's range, count it, and let `ok` keep carrying whether we truly measured. */
        {
            double mv = v * 1000.0, ma = a * 1000.0;
            if (!(mv >= 0.0)) { mv = 0.0; }          /* also catches NaN */
            if (!(ma >= 0.0)) { ma = 0.0; }
            /* AN OUT-OF-RANGE READING COLLAPSES TO ZERO, NEVER TO THE CEILING.
             *
             * The first version of this clamped UP to the protocol maximum, and the board duly
             * reported present_mv=1000000 to a vehicle that had asked for 356 V. Telling a car
             * the bus is at 1000 V is not a smaller error than crashing — it is a fabricated
             * measurement in the most dangerous direction, and it is exactly what the
             * withhold-don't-invent rule exists to stop.
             *
             * Zero is the safe collapse: it reads as "no bus voltage", which is what an
             * unreadable module is indistinguishable from, and it cannot invite an EV to close
             * its contactor onto something. `ok` goes false so nothing downstream treats it as
             * measured. */
            if (mv > 1000000.0 || ma > 1000000.0) {
                bad_readings++;
                if (bad_readings <= 3u) {
                    printf("  WARN: module telemetry out of range (%.1fV %.1fA) — reporting 0, "
                           "not clamping to the ceiling\n", v, a);
                }
                mv = 0.0;
                ma = 0.0;
                ok = 0;                               /* not measured, so not trustworthy */
            }
            meas.present_voltage = CBCTL_MV((uint32_t)mv);
            meas.present_current = CBCTL_MA((uint32_t)ma);
        }
        meas.isolation = CBCTL_ISO_VALID;   /* controller proxy — no IMD on this bench */
        (void)cbctl_set_measurement(&link, &meas);

        /* ONE tick call drives both boards when a sibling is attached. cbctl_fleet_tick services
         * every link in round-robin; it is the same code path a real controller would use. */
        /* THE DECISIVE EVENT: the idle board's transport is pulled out from under it while THIS
         * gun is drawing current. Everything after this point is the answer to "does cbctl ditch
         * one PLC when someone is charging on the other" — except the roles are the harder way
         * round, because the survivor is the one with a contactor closed and a car attached. */
        if (g_fleet_on && g_sib_kill_s != 0u && !g_sib_killed &&
            (t - t0) / 1000u >= g_sib_kill_s) {
            g_sib_killed = 1;
            printf("[%6us] *** KILLING the idle sibling's transport (models a USB drop) ***\n",
                   (t - t0) / 1000u);
            cbctl_serial_close(&g_sib_ser);
        }
        if (g_fleet_on) {
            (void)cbctl_fleet_tick(&g_fleet, t);
            if (g_sib_killed && link.worst_tick_gap_ms > g_worst_gap_after_kill) {
                g_worst_gap_after_kill = link.worst_tick_gap_ms;
            }
            {
                /* Watch the SIBLING's health from the charging link's own loop. If servicing a
                 * charge ever starved the idle board, this gap is where it would show. */
                const uint32_t obs = cbctl_observation_age_ms(&g_sib_link, t);
                if (obs != 0xFFFFFFFFu && obs > g_sib_worst_feed_gap) {
                    g_sib_worst_feed_gap = obs;
                }
                g_sib_last_obs = obs;
                if (cbctl_link_state(&g_sib_link) == CBCTL_LINK_QUARANTINED) {
                    g_sib_quarantines++;
                }
            }
        } else {
            cbctl_tick(&link, t);
        }

        while (cbctl_next_event(&link, &ev)) {
            if (ev.kind == CBCTL_EV_VEHICLE_MAC_KNOWN) {
                uint8_t mac[6];
                if (cbctl_vehicle_mac(&link, mac)) {
                    printf("[%6us] vehicle %02X:%02X:%02X:%02X:%02X:%02X — authorising\n",
                           (t - t0) / 1000u, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                }
                (void)cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 8000u);
            } else if (ev.kind == CBCTL_EV_SESSION_STARTED) {
                printf("[%6us] V2G session started\n", (t - t0) / 1000u);
                /* WHICH CONTACTORS, AND WHY NOT ALL OF THEM.
                 *
                 * RELAY_GUN is the DC gun contactor and is the only one this bench needs: the
                 * modules here are fed AC directly, and the reference harness that delivered a
                 * real 191 Wh charge on this same hardware closes exactly this and nothing
                 * else ("CTRL RELAY 1 1 <lease>").
                 *
                 * RELAY_POWER (relay3) is the AC input contactor on a production charger —
                 * charger.json calls it acContactorRelay3Mode and cbcontroller closes it from
                 * a separate owner. It is NOT wired on this bench, and commanding a contactor
                 * that may be wired to something else on the next board is how a bench tool
                 * becomes an incident. So it is an explicit --relays choice, defaulting to the
                 * mask that is known to work here, and the board's REPORTED mask is printed
                 * every line so a wrong choice is visible rather than inferred. */
                /* Assert the INTENT, do not re-command the relay.
                 *
                 * cbctl_relay_close() marks the intent dirty unconditionally and re-baselines the
                 * override detector, so calling it every tick sends a relay frame every tick —
                 * 826 renewals in 140 s on the first run of this, which is precisely the audible
                 * contactor chatter and real wear we are trying to avoid. Level-driven means the
                 * CONDITION is re-evaluated every tick; the transmission is cbctl's periodic
                 * renewal, which already runs at hold/6. Re-arm only when the intent is actually
                 * missing — after a session flap wiped it, or when it was never set. */
                /* RE-ARMING IS ALSO A TRANSMISSION, AND IT BYPASSES EVERY BACKOFF.
                 *
                 * cbctl_relay_close() sets relay_dirty, which skips the renewal period gate — so
                 * re-arming on "intent missing" sends immediately, every time. That is fine once.
                 * It is not fine when the session FLAPS: SESSION_ENDED zeroes relay_state, this
                 * re-arms, the board refuses (auth not yet live), the session flaps again, and the
                 * loop runs at tick rate. Measured: 7458 refusals in 155 s — 48/s — with the
                 * refusal-side backoff already in place and unable to see any of it, because the
                 * traffic was coming from the arm side, not the retry side.
                 *
                 * So the re-arm gets its own floor. cbctl still renews a live intent on its own
                 * cadence; this only bounds how often a NEW intent may be created. */
                if ((link.relay_state & g_relay_mask) != g_relay_mask &&
                    (t_rearm == 0u || (t - t_rearm) >= kRearmFloorMs)) {
                    (void)cbctl_relay_close(&link, g_relay_mask,
                                            CBCTL_RELAY_HOLD_RECOMMENDED_MS, NULL);
                    t_rearm = t;
                }
            }
        }
        /* THE BOARD MUST BE THE ONE WE WERE POINTED AT, and this is checked the moment it names
         * itself — before any authorisation, any relay, any energy. Refusing here costs a
         * restart; not refusing costs a charge delivered to the wrong gun. */
        if (want_mac && !mac_checked && cbctl_link_state(&link) == CBCTL_LINK_READY) {
            cbctl_identity_t id;
            if (cbctl_identity(&link, &id)) {
                mac_checked = 1;
                if (cbctl_expect_mac(&link, expect_mac) != CBCTL_OK) {
                    printf("\nWRONG BOARD on %s: expected %02X:%02X:%02X:%02X:%02X:%02X, "
                           "found %02X:%02X:%02X:%02X:%02X:%02X\n"
                           "  USB order is not stable — reflashing can swap the ports. Bind the "
                           "gun to the MAC, not the device path.\n",
                           port, expect_mac[0], expect_mac[1], expect_mac[2], expect_mac[3],
                           expect_mac[4], expect_mac[5], id.mac[0], id.mac[1], id.mac[2],
                           id.mac[3], id.mac[4], id.mac[5]);
                    bailed = 1;
                    break;
                }
                printf("board confirmed: %02X:%02X:%02X:%02X:%02X:%02X\n", id.mac[0], id.mac[1],
                       id.mac[2], id.mac[3], id.mac[4], id.mac[5]);
            }
        }

        /* AUTHORISE AS SOON AS THE LINK IS UP, AND KEEP SAYING SO.
         *
         * cbctl opens with auth = DENY and marks it dirty, because a zeroed intent must be the
         * SAFE intent — so the first thing any controller tells the board is "not authorised".
         * That is correct for the library and a trap for the caller: the vehicle may already be
         * plugged, and cbplc asks the controller for a decision within a few hundred
         * milliseconds of `secc: vehicle connected`. Measured on the bench:
         *
         *     [3188007] I secc: vehicle connected
         *     [3188268] W secc: controller DENIED authorisation, failing session
         *
         * 261 ms. Granting only on the VEHICLE_MAC_KNOWN edge loses that race whenever the MAC
         * is learned late or the event was already consumed, and the old `start_soc < 0` guard
         * made it worse by silently retiring the re-assert the moment a SoC arrived — so a
         * session that got that far could never be re-authorised afterwards.
         *
         * Restating a decision that has not changed is free: cbctl_authorize() only marks the
         * intent dirty on an actual change, and the lease renewal runs off its own period. */
        if (cbctl_link_state(&link) == CBCTL_LINK_READY) {
            (void)cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 8000u);
        }

        /* DRIVE THE MODULES FROM THE EV'S OWN TARGET, THE MOMENT IT ASKS.
         *
         * The first version gated this on CBCTL_FLAG_ENERGISED and deadlocked: the EV will not
         * close its contactor until the bus is already near its requested voltage, so waiting
         * to be told "energised" means never producing volts, PreCharge never completing, and
         * the session timing out with 0 V on the wire. Observed exactly that — session up at
         * 7 s, gone by 20 s, 0 Wh.
         *
         * The target arrives during PreCharge, which is precisely when the bus must start
         * ramping toward it. */
        {
            uint32_t tgt_mv = 0u, tgt_ma = 0u;
            const bool want_power = cbctl_ev_target(&link, &tgt_mv, &tgt_ma) && tgt_mv > 0u;
            const bool gun_shut = (link.relay_closed_mask & CBPLC_RELAY_GUN) != 0u;
            /* Session liveness for the CONTACTOR is deliberately broader than demand liveness for
             * the MODULES: secc_active alone proved unreliable mid-PreCharge, so accept either it
             * or a recent DC answer. Erring early here is safe — the bus is dead. */
            const bool session_live = cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE) || want_power;

            /* CLOSE FIRST, ENERGISE SECOND — and hold the close, do not pulse it.
             *
             * cbplc will not grant energy at all until the contactor is shut. Its refusal chain
             * (cbplc/src/limits.c:252) is ordered and it names the step it is stuck on; on this
             * bench it read `limits_refusal=contactor_open` with everything else already green.
             * So the contactor is not something to close once a charge is under way — it is a
             * PRECONDITION of the charge, and it must be shut while the bus is still dead.
             *
             * The close used to be a one-shot on the SESSION_STARTED edge. That edge fired once,
             * early, sometimes before authorisation was live, and was never re-asserted — so the
             * board sat with relay_verified_open=0x07 (readback-PROVEN open) while the vehicle sat
             * in C2 asking for 356 V. A physical state that must be HELD cannot be driven by an
             * edge. cbctl_relay_close() only re-dirties on change, so restating it is free.
             *
             * The ordering is the safety half: modules are commanded ONLY once the board reports
             * the gun actually shut. Enabling first would ramp the bus and then close a contactor
             * onto live DC, which is how contactors weld. Board-reported, not commanded — we
             * energise on evidence, not on intent. */
            /* LEVEL, NOT EDGE — AND EARLY, NOT ON DEMAND.
             *
             * The reference harness closes the gun whenever the session is in the energy phase
             * (plc_bench.py:1036: in_energy = CABLE_CHECK | PRECHARGE | DELIVERY) and re-sends
             * that close every ~2 s for as long as the phase holds. It deliberately ANTICIPATES
             * CableCheck from the earlier substates, because the isolation test runs through a
             * contactor that must already be in. In plc_firmware relay1_set(true, ...) has exactly
             * one call site: the controller's repeating lease is the only thing that ever closes
             * the gun.
             *
             * Waiting for an EV target inverts that. The target arrives at PreCharge, which is
             * AFTER CableCheck, which is after the point the contactor was needed — so gating on
             * it guarantees we are always one phase late. A live session is the right level
             * condition, and it is if anything earlier than the reference's, which both firmwares
             * accept (no gate anywhere checks the V2G phase). */
            /* THE LIVE SIGNAL IS THE VEHICLE'S OWN DEMAND, NOT A FLAG.
             *
             * This branch used to be gated on CBCTL_FLAG_SESSION_ACTIVE, i.e. cbplc's
             * `secc_active`. That flag reads 0 while the EV is actively in PreCharge sending DC
             * requests, so every pass fell into the else-branch and called power_off(). Captured
             * on the CAN wire across a whole session: the ONLY frames we ever sent the modules
             * were 17 x ALL_SET with byte1=0x01 — OnOff "Shut Down (DC, not PFC)" — and an
             * all-zero payload. The modules were not refusing to start. We were holding them off
             * and then timing out because they produced nothing.
             *
             * `want_power` is the honest condition: the vehicle asked for a voltage and that ask
             * is FRESH (cbctl_ev_target() now ages out at CBCTL_EV_TARGET_STALE_MS). A demand
             * that is current is exactly what "the car wants energy" means; a session flag that
             * disagrees with the car's own messages is not. */
            /* TWO TRIGGERS, NOT ONE — the sequence doc says so and I merged them anyway.
             *
             * The gun closes at CableCheck; the modules enable at PreCharge. Those are different
             * phases with different conditions, and collapsing both into "the EV asked for a
             * voltage" deadlocks: the target only exists at PreCharge, PreCharge is only reached
             * once cbplc grants energy, and cbplc refuses energy while the contactor is open.
             * Captured verbatim in a session log — substate=ServiceDiscovery, ev_tgt_mv=0,
             * relay_verified_open=0x07, limits_refusal=contactor_open — the charger waiting for
             * the car and the car waiting for the charger.
             *
             * So: a LIVE SESSION closes the gun (early, onto a dead bus, which is the whole point
             * of closing at CableCheck), and a LIVE DEMAND energises. */
            if (session_live) {
                /* LEVEL-DRIVEN CONDITION, EDGE-DRIVEN TRANSMISSION.
                 *
                 * This called cbctl_relay_close() on EVERY pass, and that function marks the
                 * intent dirty unconditionally — so it put a RELAY frame on the wire every
                 * tick. Measured on the charger: 874 relay frames across a ~6 s session, ~145
                 * per second. On a stop-and-wait link with a bounded poll budget that is not
                 * merely contactor wear, it is starvation: the relay outranks AUTHORIZE in the
                 * priority order, so re-commanding it at tick rate crowds out the auth lease
                 * renewal, the board drops to auth=0, and cbplc then refuses the very close
                 * this loop is spamming. The session dies of the traffic meant to hold it up.
                 *
                 * cbctl already renews a held lease on its own cadence (hold/6). So re-arm
                 * only when the intent is genuinely absent — never established, or wiped by a
                 * session flap — and let the library carry it the rest of the time. */
                if ((link.relay_state & g_relay_mask) != g_relay_mask &&
                    (t_rearm == 0u || (t - t_rearm) >= kRearmFloorMs)) {
                    (void)cbctl_relay_close(&link, g_relay_mask,
                                            CBCTL_RELAY_HOLD_RECOMMENDED_MS, NULL);
                    t_rearm = t;
                }
                if (gun_shut && want_power) {
                    double want_a = (double)tgt_ma / 1000.0;
                    if (want_a > max_a) {
                        want_a = max_a;
                    }
                    /* PreCharge asks for volts at ~0 A; delivery asks for current too. Following
                     * the vehicle's own number in both phases is the whole contract. */
                    power_set((double)tgt_mv / 1000.0, want_a);
                    announced_idle = 0;
                    if (!announced_close) {
                        rec_event("energise", "gun closed (readback), target=%umV want_a=%.1f",
                                  tgt_mv, want_a);
                        printf("[%6us] gun CLOSED (board readback) — energising to %umV\n",
                               (t - t0) / 1000u, tgt_mv);
                        announced_close = 1;
                    }
                } else if (!gun_shut && !announced_wait) {
                    printf("[%6us] session live — closing gun (bus dead) before any energy\n",
                           (t - t0) / 1000u);
                    announced_wait = 1;
                }
            } else {
                /* SAY WHY WE ARE NOT DRIVING. This branch commands the modules OFF, and for a
                 * whole afternoon it did so silently while a vehicle sat in PreCharge asking for
                 * power — the only trace was 17 ALL_SET frames carrying "Shut Down" that nobody
                 * looked at. A refusal to energise must explain itself at the moment it happens. */
                if (!announced_idle) {
                    rec_event("idle", "not driving: target %s raw=%umV dc_ans=%u",
                              link.ev_target_valid ? "stale" : "absent",
                              link.ev_target_mv, link.dc_answers);
                    printf("[%6us] NOT driving modules: ev_target %s (raw=%umV, dc_ans=%u, "
                           "last DC answer %ums ago) — commanding OFF\n",
                           (t - t0) / 1000u,
                           link.ev_target_valid ? "stale" : "absent",
                           link.ev_target_mv, link.dc_answers,
                           link.ev_target_ms ? (t - link.ev_target_ms) : 0u);
                    announced_idle = 1;
                }
                if (power_modules_on() > 0) {
                    power_off();
                }
                /* Session gone: drop the intent so the dead-man is not renewed against nothing. */
                if ((link.relay_state & CBPLC_RELAY_ALL) != 0u) {
                    (void)cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL);
                }
                announced_close = 0;
                announced_wait = 0;
            }
        }

        /* THE SOC GATE. -1 means the car has not told us; it must never read as 0. */
        soc = cbctl_ev_soc_pct(&link);
        if (soc >= 0) {
            if (start_soc < 0) {
                start_soc = soc;
                target_soc = soc + soc_delta;
                printf("[%6us] EV reports SoC %d%% — target %d%%\n",
                       (t - t0) / 1000u, start_soc, target_soc);
            }
            if (soc != last_soc) {
                printf("[%6us] SoC %d%% (%+d)  %.1fV %.1fA  %.0fWh\n",
                       (t - t0) / 1000u, soc, soc - start_soc, v, a, wh);
                last_soc = soc;
            }
            if (target_soc > 0 && soc >= target_soc) {
                printf("\n*** TARGET REACHED: %d%% -> %d%% (+%d) ***\n",
                       start_soc, soc, soc - start_soc);
                rc = 0;
                break;
            }
        }

        /* --- fast-fail gates, checked before the periodic report so a bail is immediate --- */
        if (cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE)) {
            if (t_session == 0u) { t_session = t; }
        } else {
            t_session = 0u;
        }
        if (v > 5.0) {
            if (t_volts == 0u) { t_volts = t; }
        }
        /* No session yet and the vehicle has gone quiet — kick it rather than wait out its
         * back-off. Only while nothing is in progress: a replug during a live session is a
         * disconnect. */
        if (t_session == 0u && !bailed &&
            cbctl_flag(&link, CBCTL_FLAG_VEHICLE_PRESENT) &&
            (t - t0) > 12000u &&
            (t_replug == 0u || (t - t_replug) > kReplugEveryMs)) {
            cbctl_op_t op;
            if (cbctl_slac_replug(&link, &op) == CBCTL_OK) {
                replugs++;
                printf("[%6us] no session — driving a pilot replug (#%u)\n",
                       (t - t0) / 1000u, replugs);
            }
            t_replug = t;
        }

        /* THE PHASE WATCHDOG. A phase that stops advancing is the failure; waiting out a
         * global timeout only tells you the run failed, not where. Print the phase, how long it
         * has been stuck, and cbplc's OWN refusal reason — that last field is the whole
         * diagnosis and it is one scrape away. */
        /* The stamp is written inside the STATUS observer, which runs DURING cbctl_tick() — i.e.
         * after `t` was sampled at the top of this pass. So g_substate_since can legitimately be
         * a few milliseconds AHEAD of t, and an unsigned subtraction then wraps to ~49 days. It
         * did exactly that on the first run and false-fired a bail at 20 s, reporting
         * "stuck=4294967s". A watchdog that cries wolf is worse than none: it ends good sessions
         * and teaches you to ignore it. Compare signed, and treat "ahead of us" as zero. */
        /* CurrentDemand IS THE DESTINATION, SO DWELLING IN IT IS NOT A STALL.
         *
         * Every other substate is a step on the way somewhere and a long stay means something is
         * wrong. CurrentDemand is where a charge LIVES — a 40 kWh session sits in it for an hour.
         * This watchdog killed the first genuinely working charge at exactly 90 s: 359.8 V,
         * 9.9 A, 89 Wh accumulating, SoC climbing, `limits_refusal='-'` (the gate wide open) —
         * and it printed "stuck". The stop conditions for delivery are the SoC target and the
         * no-current bail below, both of which measure the thing that would actually be wrong. */
        if (!bailed && g_substate[0] != '\0' && g_substate_since != 0u &&
            strcmp(g_substate, "Terminated") != 0 && strcmp(g_substate, "Idle") != 0 &&
            strcmp(g_substate, "CurrentDemand") != 0 &&
            (int32_t)(t - g_substate_since) > (int32_t)kPhaseStallMs) {
            {
                const int32_t stuck = (int32_t)(t - g_substate_since);
                rec_event("bail", "phase=%s stuck=%ds furthest=%s refusal=%s",
                          g_substate, (stuck > 0 ? stuck : 0) / 1000, g_furthest, g_refusal);
            }
            printf("\nBAIL: stuck in phase '%s' for %us (furthest=%s)\n"
                   "      cbplc energy gate says: limits_refusal='%s'%s%s\n",
                   g_substate, (unsigned)(((int32_t)(t - g_substate_since) > 0)
                                          ? (int32_t)(t - g_substate_since) / 1000 : 0),
                   g_furthest[0] ? g_furthest : "?",
                   g_refusal[0] ? g_refusal : "(none — gate is open)",
                   g_chg_reason[0] ? "  chg_reason=" : "", g_chg_reason);
            bailed = 1;
            break;
        }

        if (!bailed && t - t0 > kNoSessionMs && t_session == 0u &&
            !cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE)) {
            printf("\nBAIL: no V2G session in %us after %u replugs (slac=%d plug=%d led=%s)\n",
                   kNoSessionMs / 1000u, replugs,
                   cbctl_flag(&link, CBCTL_FLAG_SLAC_MATCHED) ? 1 : 0,
                   cbctl_flag(&link, CBCTL_FLAG_VEHICLE_PRESENT) ? 1 : 0,
                   cbctl_led_name(cbctl_led_preset(&link, t)));
            bailed = 1;
            break;
        }
        /* THE BAIL MUST USE THE SAME FRESHNESS TEST AS THE DRIVE, OR IT KILLS THE RUN.
         *
         * This read `link.ev_target_valid` + `link.ev_target_mv` RAW, while the branch that
         * actually energises uses cbctl_ev_target(), which ages the demand out at
         * CBCTL_EV_TARGET_STALE_MS. cbplc keeps publishing the LAST session's ev_tgt_mv after
         * that session has Terminated, so the two disagreed exactly when it mattered: the
         * driver correctly refused to drive an absent target ("NOT driving modules: ev_target
         * absent"), and twenty seconds later this bail fired on the corpse of the previous
         * session and aborted the run — measured three times, always at ~32 s, always before
         * the vehicle had finished backing off and re-initiated. A fast-fail that fires on
         * stale evidence does not save bench time, it prevents the test from ever passing. */
        /* AND THE CLOCK STARTS WHEN THE CAR ASKS, NOT WHEN THE PROCESS DID.
         *
         * `t - t0 > kNoVoltsMs` measures the run, not the demand. The first session opened at
         * 73 s, the gun closed at 79 s, the modules were commanded — and the bail fired in the
         * same second, because 79 s had already passed. It reported "the modules never
         * produced volts in 20 s" about a setpoint that was under one second old. A deadline
         * has to start at the event it is about. */
        if (t_demand == 0u) {
            uint32_t d_mv = 0u, d_ma = 0u;
            if (cbctl_ev_target(&link, &d_mv, &d_ma) && d_mv > 0u) { t_demand = t; }
        }
        {
            uint32_t bail_mv = 0u, bail_ma = 0u;
            if (!bailed && t_volts == 0u && t_demand != 0u && t - t_demand > kNoVoltsMs &&
                cbctl_ev_target(&link, &bail_mv, &bail_ma) && bail_mv > 0u) {
                printf("\nBAIL: EV asked for %umV but the modules never produced volts in %us "
                       "— check the OUTPUT ENABLE, not the setpoint\n",
                       bail_mv, kNoVoltsMs / 1000u);
                bailed = 1;
                break;
            }
        }
        /* Volts at the modules, no current, and the board says the contactor we asked for is
         * NOT closed. That is a circuit that was never completed, and it is worth naming
         * rather than waiting out the SoC timeout: asked-for and actually-closed differ. */
        /* A BLIP IS NOT A BROKEN CIRCUIT. This sampled the relay mask at one instant, so a
         * momentary open — a session flap, a restart landing mid-session — read as "the circuit
         * was never completed" and killed a run that was otherwise fine. The condition it is
         * really about is the relay being open CONTINUOUSLY while we hold volts and see no
         * current, so it now measures how long the gun has actually been open. */
        if ((link.relay_closed_mask & g_relay_mask) == g_relay_mask) {
            t_gun_open = 0u;
        } else if (t_gun_open == 0u) {
            t_gun_open = (t == 0u) ? 1u : t;
        }
        if (!bailed && t_volts != 0u && (t - t_volts) > 20000u && a < 0.5 &&
            t_gun_open != 0u && (t - t_gun_open) > 20000u &&
            (link.relay_closed_mask & g_relay_mask) != g_relay_mask) {
            printf("\nBAIL: %.0fV at the modules, %.1fA, but the board reports relays=0x%02X "
                   "when we asked for 0x%02X — the circuit was never completed\n",
                   v, a, link.relay_closed_mask, g_relay_mask);
            bailed = 1;
            break;
        }
        if (!bailed && t_session != 0u && (t - t_session) > kNoSocMs && start_soc < 0) {
            printf("\nBAIL: session up %us with no SoC reported — cannot enforce a target\n",
                   kNoSocMs / 1000u);
            bailed = 1;
            break;
        }

        /* THE METER IS THE ONLY WITNESS DOWNSTREAM OF THE MODULES.
         *
         * The modules reported 389.1 V at 0.0 A for a whole CurrentDemand phase — which is a
         * current-limited supply sitting at its exact setpoint, i.e. NO LOAD, while the vehicle
         * asked for 20 A with ev_ready=1 and the board reported the gun closed. Module telemetry
         * cannot distinguish "the pack is drawing nothing" from "the circuit past me is open",
         * because both look identical at the module's own terminals. The DCM3366 sits after the
         * contactor, so it can: volts there and none at the modules is one fault, volts at the
         * modules and none there is a different one. Printing both is what makes the next run
         * decisive instead of another round of inference. */
        if (t - last_report >= 5000u) {
            {
                const uint32_t fa = cbctl_feed_age_ms(&link, t);
                /* NAME THE SOURCE. This line used to say "modules=" for a value that now comes
                 * from the meter, and that mislabel cost a diagnosis: a meter reading the wrong
                 * gun's loop was read as "the modules produced nothing". Say which it is, and
                 * print the module group's own numbers beside it so the two can disagree
                 * visibly. */
                printf("       %-11s feed=%s soc=%d tgt=%umV %s %.1fV %.1fA %.0fWh "
                       "| meter %s %.1fV %.1fA "
                       "led=%s relays=0x%02X | grp a=%d/act=%d/on=%d/flt=%d req=%.0fV/%.0fA | L=%u F=%u A=%u S=%u\n",
                       cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE) ? "session" : "waiting",
                       fa == CBCTL_FEED_NEVER_SENT ? "NEVER" : "ok",
                       soc, link.ev_target_mv,
                       ok ? "mod" : "STALE", v, a, wh,
                       g_meter_ok ? "ok" : "--", g_meter_v, g_meter_a,
                       cbctl_led_name(cbctl_led_preset(&link, t)), link.relay_closed_mask,
                       g_ga, g_gact, g_gon, g_gflt, g_grv, g_gra,
                       link.limits_sent, link.feedback_sent, link.auth_sent,
                       link.status_sent);
                last_report = t;
            }
            if (0) printf("       %-11s feed=%4ums soc=%d modules=%s %.1fV %.1fA %.0fWh led=%s\n",
                   cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE) ? "session" : "waiting",
                   cbctl_feed_age_ms(&link, t), soc, ok ? "fresh" : "STALE", v, a, wh,
                   cbctl_led_name(cbctl_led_preset(&link, t)));
            last_report = t;
        }

        /* Waiting on ONE link's fd while a second link needs servicing is how a caller ditches a
         * PLC even though the library did nothing wrong — so poll BOTH, and wake on whichever
         * link's deadline comes first. */
        if (g_fleet_on) {
            struct pollfd pf[2];
            pf[0].fd = cbctl_serial_fd(&ser);      pf[0].events = POLLIN; pf[0].revents = 0;
            pf[1].fd = cbctl_serial_fd(&g_sib_ser); pf[1].events = POLLIN; pf[1].revents = 0;
            (void)poll(pf, 2, (int)cbctl_fleet_next_deadline_ms(&g_fleet, now_ms()));
        } else {
            pfd.fd = cbctl_serial_fd(&ser);
            pfd.events = POLLIN;
            (void)poll(&pfd, 1, (int)cbctl_next_deadline_ms(&link, now_ms()));
        }
    }

    /* Wind down: modules off FIRST, then the session, then keep ticking so the close reaches
     * the board. There is no blocking wait to call, which is exactly why this is a loop. */
    printf("\nwind-down\n");
    power_off();
    (void)cbctl_begin_stop(&link);
    (void)cbctl_relay_open(&link, CBPLC_RELAY_ALL, NULL);
    (void)cbctl_authorize(&link, CBCTL_AUTH_PENDING, 0u);
    {
        const uint32_t w0 = now_ms();
        while (now_ms() - w0 < 8000u) {
            cbctl_measurement_t z;
            memset(&z, 0, sizeof z);
            z.isolation = CBCTL_ISO_VALID;   /* 0 V / 0 A is the weld evidence */
            (void)cbctl_set_measurement(&link, &z);
            cbctl_tick(&link, now_ms());
            power_tick();
        power_group_debug(&g_ga, &g_gact, &g_gon, &g_gflt, &g_grv, &g_gra);
            (void)poll(NULL, 0, 40);
        }
    }

    /* DRAIN THE BOARD'S LOG BEFORE EXITING, or the run that most needed it has none.
     *
     * cbctl suppresses draining for the whole time a contactor is held or a session is live —
     * correctly, because a round trip spent on log text during delivery comes out of the feed
     * and lease budget. The consequence is that a charge which runs to completion has its ENTIRE
     * trace still sitting in the board's ring at the moment this process would otherwise exit,
     * and it would be lost to the next CLEAR or simply overwritten.
     *
     * By here the relays are open, the stop is latched and there is no session, so the wire is
     * ours and the drain costs nothing. Bounded, because a bench tool may not hang on a board
     * that has stopped answering: `pending` reaching zero ends it early, and the deadline ends
     * it regardless. What could not be drained is REPORTED rather than passed over — a truncated
     * trace that looks complete is worse than no trace. */
    if (g_diaglog != NULL) {
        const uint32_t d0 = now_ms();
        uint32_t pending = 0u;
        printf("draining the board's log...\n");
        while (now_ms() - d0 < 15000u) {
            if (g_fleet_on) { (void)cbctl_fleet_tick(&g_fleet, now_ms()); }
            else { cbctl_tick(&link, now_ms()); }
            (void)poll(NULL, 0, 20);
            if (cbctl_diag_state(&link, NULL, NULL, NULL, &pending) && pending == 0u &&
                now_ms() - d0 > 2000u) {
                break;
            }
        }
        {
            uint32_t dropped = 0u;
            (void)cbctl_diag_state(&link, NULL, NULL, &dropped, &pending);
            printf("  board log: %u bytes drained, %u still pending, %u dropped by the board\n",
                   link.diag_bytes, pending, dropped);
            if (dropped != 0u) {
                printf("  WARNING: the board's ring lapped — this trace has HOLES and is not "
                       "complete evidence\n");
            }
        }
        (void)fflush(g_diaglog);
        (void)fclose(g_diaglog);
        g_diaglog = NULL;
    }

    /* WHAT THE BOARD CONFIRMED, not what we sent. An unconfirmed declaration means the charge
     * ran under an identity nobody applied, and that must be visible in the result rather than
     * assumed from the fact that a setter returned OK. */
    if (link.prov.evse_id_set || link.prov.attn_trim_set || link.prov.protocol_mask_set) {
        printf("\n=== provisioning ===\n");
        printf("  supported       : %s\n", cbctl_provision_supported(&link) ? "yes" : "NO");
        printf("  confirmed       : %s\n", link.prov_dirty ? "NOT CONFIRMED BY THE BOARD" : "yes");
        printf("  accepted/refused: %u/%u\n", link.prov_accepted, link.prov_refused);
    }

    printf("\n=== result ===\n");
    printf("  SoC        : %d%% -> %d%%\n", start_soc, last_soc);
    printf("  peak       : %.1f V  %.1f A\n", peak_v, peak_a);
    printf("  energy     : %.0f Wh\n", wh);
    printf("  feed_late  : %u   lease lapses: %u   renewals: %u (acks %u)\n",
           link.feed_late, cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS),
           link.relay_renewals, link.relay_acks);
    /* A close that is commanded, refused, and silently retried forever looks identical to one
     * that was never asked for — unless the refusals are printed. */
    printf("  relay      : refusals=%u overridden=%u safety_opens=%u board_mask=0x%02X\n",
           link.relay_refusals, link.relay_overridden,
           cbctl_counter(&link, CBCTL_CTR_RELAY_SAFETY_OPENS), link.relay_closed_mask);
    printf("  min lease headroom: %ums   worst tick gap: %ums\n",
           link.lease_min_headroom_ms, link.worst_tick_gap_ms);
    if (g_fleet_on) {
        /* The sibling never charged, so it has no lease to report. What matters is that it stayed
         * READY and kept being observed while this gun drew 50 A — and, symmetrically, that the
         * numbers above are no worse than a single-link run. */
        printf("  sibling    : state=%s worst STATUS gap=%ums (bound %u) last=%ums "
               "quarantined_ticks=%u\n",
               cbctl_link_state(&g_sib_link) == CBCTL_LINK_READY ? "READY" :
               cbctl_link_state(&g_sib_link) == CBCTL_LINK_QUARANTINED ? "QUARANTINED" : "other",
               g_sib_worst_feed_gap, (unsigned)CBPLC_LIMITS_STALE_MS, g_sib_last_obs,
               g_sib_quarantines);
        if (g_sib_killed) {
            printf("  sibling transport was KILLED at %us — this gun's worst tick gap after "
                   "the kill: %ums\n", g_sib_kill_s, g_worst_gap_after_kill);
        }
    }
    printf("  transport  : deferred=%u partial=%u read_err=%u\n",
           ser.write_deferred, ser.partial_writes, ser.read_errors);

    power_off();
    power_close();
    cbctl_serial_close(&ser);
    return rc;
}
