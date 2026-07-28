/* Pull a cbplc board's diagnostic log over the control protocol, and arm what it records.
 *
 * WHY A SEPARATE TOOL
 *   The board's log has always been reachable in principle — CBPLC_T_DIAG has drained it since
 *   text stopped being printed onto the control wire — and in practice nobody could get at it
 *   without writing a program. That is the wrong friction for the thing you reach for when a
 *   particular vehicle refuses a charger that works with every other one.
 *
 *   It is also the tool that PROVES the control path, which is why it prints the board's own
 *   reported state rather than assuming the settings took. A setter that is not read back is a
 *   setter you believe rather than one you know.
 *
 * THE V2G TRACE
 *   --v2g arms cbplc's per-message decoded HLC trace: one line per request with the message,
 *   the stack (ISO2 or DIN), how long the session has sat in this phase, what the car asked
 *   for, whether it declared itself ready, and what we answered. That is roughly 100 bytes a
 *   message and 40-60 messages a session, which is why it is off by default and why the board
 *   does not persist it across a reboot.
 *
 * SAFETY AND COST
 *   This tool never authorises, never closes a relay and never commands a module — it cannot
 *   start or stop a charge. It is safe to run against a charger that is working, because cbctl
 *   suppresses draining entirely while a contactor is held or a session is live; anything the
 *   board records during a charge is handed over afterwards, intact, from its 16 KB ring.
 *
 *   ./board_log /dev/ttyACM1 [seconds] [--v2g] [--quiet] [--level=debug|info|warn|error|off]
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
static void on_sig(int s) { (void)s; g_stop = 1; }

static uint32_t now_ms(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000));
}

static uint64_t g_bytes;

/* Straight to stdout, unmodified. The ring is a byte stream and a drain boundary can fall
 * mid-line, so anything that tried to emit whole lines would have to buffer — on the control
 * thread, for no benefit that piping into a file does not already give. */
static void sink(const uint8_t *bytes, uint16_t len, void *user)
{
    (void)user;
    g_bytes += len;
    (void)fwrite(bytes, 1u, len, stdout);
}

static const char *level_name(uint8_t l)
{
    switch (l) {
    case CBCTL_DIAG_DEBUG: return "debug";
    case CBCTL_DIAG_INFO:  return "info";
    case CBCTL_DIAG_WARN:  return "warn";
    case CBCTL_DIAG_ERROR: return "error";
    case CBCTL_DIAG_OFF:   return "off";
    default:               return "?";
    }
}

static int parse_level(const char *s, cbctl_diag_level_t *out)
{
    if (strcmp(s, "debug") == 0) { *out = CBCTL_DIAG_DEBUG; return 0; }
    if (strcmp(s, "info") == 0)  { *out = CBCTL_DIAG_INFO;  return 0; }
    if (strcmp(s, "warn") == 0)  { *out = CBCTL_DIAG_WARN;  return 0; }
    if (strcmp(s, "error") == 0) { *out = CBCTL_DIAG_ERROR; return 0; }
    if (strcmp(s, "off") == 0)   { *out = CBCTL_DIAG_OFF;   return 0; }
    return -1;
}

int main(int argc, char **argv)
{
    cbctl_serial_t ser;
    cbctl_link_t link;
    const char *port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    uint32_t run_s = 30u;
    int want_v2g = 0, quiet = 0, i;
    cbctl_diag_level_t level = CBCTL_DIAG_INFO;
    uint32_t cats;
    uint32_t t0, t, last_report = 0u;
    int reported_support = 0;

    for (i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--v2g") == 0) {
            want_v2g = 1;
        } else if (strcmp(argv[i], "--quiet") == 0) {
            quiet = 1;
        } else if (strncmp(argv[i], "--level=", 8) == 0) {
            if (parse_level(argv[i] + 8, &level) != 0) {
                fprintf(stderr, "bad level '%s'\n", argv[i] + 8);
                return 2;
            }
        } else {
            run_s = (uint32_t)atoi(argv[i]);
        }
    }

    /* Line-buffered: a log that only appears when the process exits is useless exactly when it
     * is being watched. */
    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    if (!cbctl_serial_open(&ser, port, 115200u) || cbctl_open(&link, &ser.tx) != CBCTL_OK) {
        fprintf(stderr, "cannot attach %s\n", port);
        return 2;
    }

    cats = CBCTL_DIAG_CAT_CORE | CBCTL_DIAG_CAT_SLAC | CBCTL_DIAG_CAT_CP | CBCTL_DIAG_CAT_RELAY;
    if (want_v2g) {
        cats |= CBCTL_DIAG_CAT_V2G;
    }
    if (cbctl_diag_enable(&link, true, sink, NULL) != CBCTL_OK) {
        fprintf(stderr, "cannot arm diagnostics\n");
        return 2;
    }
    (void)cbctl_diag_set_level(&link, level);
    (void)cbctl_diag_set_categories(&link, cats);

    fprintf(stderr, "board_log: %s for %us, level=%s%s\n", port, run_s, level_name((uint8_t)level),
            want_v2g ? ", V2G trace ARMED" : "");

    t0 = now_ms();
    for (;;) {
        uint8_t blevel = 0u;
        uint32_t bcats = 0u, dropped = 0u, pending = 0u;

        t = now_ms();
        if (g_stop || (t - t0) >= run_s * 1000u) {
            break;
        }
        (void)cbctl_tick(&link, t);

        /* Say ONCE whether the board can be configured at all. An older firmware answers a
         * control request by draining, so its settings cannot be changed — draining still
         * works, and a caller that could not tell the two apart would report "logging is off"
         * when the truth is "I cannot turn it on". */
        if (!reported_support && (t - t0) > 2000u) {
            reported_support = 1;
            if (!cbctl_diag_control_supported(&link)) {
                fprintf(stderr, "board_log: this firmware predates diag control — draining "
                                "works, level/categories cannot be set\n");
            }
        }

        if (!quiet && (t - last_report) >= 5000u) {
            last_report = t;
            if (cbctl_diag_state(&link, &blevel, &bcats, &dropped, &pending)) {
                /* The BOARD's numbers, not ours. `dropped` is the one that decides whether a
                 * trace is evidence: non-zero means the ring lapped and the log has holes in
                 * it, and a conclusion drawn from a log with holes is not a conclusion. */
                fprintf(stderr,
                        "  [%3us] board: level=%s cats=0x%08X dropped=%u pending=%u | "
                        "got %llu B\n",
                        (t - t0) / 1000u, level_name(blevel), bcats, dropped, pending,
                        (unsigned long long)g_bytes);
            } else {
                fprintf(stderr, "  [%3us] board has not reported its ring state yet | got %llu B\n",
                        (t - t0) / 1000u, (unsigned long long)g_bytes);
            }
        }
        /* Sleep until the link's own next deadline, capped so the reporting line still ticks.
         * Waiting on the fd rather than spinning is what keeps this cheap enough to leave
         * running next to a charger. */
        {
            struct pollfd pfd;
            uint32_t wait_ms = cbctl_next_deadline_ms(&link, now_ms());
            if (wait_ms > 200u) { wait_ms = 200u; }
            pfd.fd = cbctl_serial_fd(&ser);
            pfd.events = POLLIN;
            pfd.revents = 0;
            (void)poll(&pfd, 1u, (int)wait_ms);
        }
    }

    /* LEAVE THE BOARD AS WE FOUND IT — almost.
     *
     * The V2G trace is disarmed on the way out, because leaving a firehose running on a charger
     * nobody is watching is the failure this whole design is shaped to avoid. The level is left
     * alone: another tool may be watching, and silently quietening a board on somebody else's
     * behalf is worse than leaving it slightly chattier than default. */
    if (want_v2g) {
        uint32_t deadline;
        (void)cbctl_diag_set_categories(&link, cats & ~(uint32_t)CBCTL_DIAG_CAT_V2G);
        deadline = now_ms() + 1500u;
        while (now_ms() < deadline) {
            struct pollfd pfd;
            (void)cbctl_tick(&link, now_ms());
            pfd.fd = cbctl_serial_fd(&ser);
            pfd.events = POLLIN;
            pfd.revents = 0;
            (void)poll(&pfd, 1u, 50);
        }
    }

    {
        uint8_t blevel = 0u;
        uint32_t bcats = 0u, dropped = 0u, pending = 0u;
        fflush(stdout);
        fprintf(stderr, "\nboard_log: %llu bytes", (unsigned long long)g_bytes);
        if (cbctl_diag_state(&link, &blevel, &bcats, &dropped, &pending)) {
            fprintf(stderr, ", board level=%s cats=0x%08X dropped=%u pending=%u",
                    level_name(blevel), bcats, dropped, pending);
            if (dropped != 0u) {
                fprintf(stderr, "\n  WARNING: the board's ring lapped — this log has HOLES in "
                                "it and is not complete evidence");
            }
        }
        fprintf(stderr, "\n  control=%s drains=%u suppressed=%u (suppressed = declined while "
                        "energy was live, by design)\n",
                cbctl_diag_control_supported(&link) ? "supported" : "UNSUPPORTED",
                link.diag_drains, link.diag_suppressed);
    }
    cbctl_serial_close(&ser);
    return 0;
}
