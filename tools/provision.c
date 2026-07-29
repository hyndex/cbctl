/* Declare a charger's identity and a board's calibration, and read back what took.
 *
 * WHY THIS IS A TOOL AND NOT A CONFIG FILE
 *   cbplc stores nothing — no NVS, no config file, no board identity — because a stored id that
 *   can corrupt is how a dead board and a misconfigured board became indistinguishable. So
 *   provisioning is a DECLARATION the controller repeats, and this is the smallest thing that
 *   can make one. On a production charger cbcontroller does exactly this at link-up, keyed by
 *   the board's MAC; here it is a person at a bench.
 *
 *   It exists mostly because of one number. `kEvseId[] = "IN*JPE*E000100010001"` was compiled
 *   into every binary, so every charger we build announced the SAME identity to every vehicle.
 *
 * WHY IT READS BACK
 *   A setter that is not read back is a setter you believe rather than one you know. This waits
 *   for the board's ack, then prints what the BOARD reports in its own STATUS — prov_id,
 *   prov_proto, prov_trim, prov_ok/prov_bad — so "I sent it" and "it is in force" stay
 *   distinguishable.
 *
 * SAFETY
 *   Nothing here authorises, closes a relay or commands a module. --restart is refused by the
 *   BOARD unless the connector is genuinely idle, and that refusal is reported rather than
 *   retried.
 *
 *   ./provision /dev/ttyACM1 --evse-id DE*ABC*E000100010001 --protocols din,iso2 --trim -6
 *   ./provision /dev/ttyACM1 --show
 *   ./provision /dev/ttyACM1 --restart
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

/* The board's own STATUS is the authority on what is in force, so the tool scrapes the
 * provisioning tail rather than reporting back what it sent. */
static char g_prov_id[64], g_prov_proto[16], g_prov_trim[16], g_prov_ok[16], g_prov_bad[16];

static void scrape(const char *line, size_t len, const char *key, char *out, size_t cap)
{
    const size_t kl = strlen(key);
    size_t i, n = 0u;
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

static void on_status(const char *line, size_t len, void *ud)
{
    (void)ud;
    scrape(line, len, "prov_id=", g_prov_id, sizeof g_prov_id);
    scrape(line, len, "prov_proto=", g_prov_proto, sizeof g_prov_proto);
    scrape(line, len, "prov_trim=", g_prov_trim, sizeof g_prov_trim);
    scrape(line, len, "prov_ok=", g_prov_ok, sizeof g_prov_ok);
    scrape(line, len, "prov_bad=", g_prov_bad, sizeof g_prov_bad);
}

static int parse_protocols(const char *s, uint8_t *out)
{
    uint8_t m = 0u;
    if (strstr(s, "din") != NULL) { m |= CBCTL_PROTO_DIN70121; }
    if (strstr(s, "iso2") != NULL || strstr(s, "iso") != NULL) { m |= CBCTL_PROTO_ISO2; }
    if (m == 0u) { return -1; }
    *out = m;
    return 0;
}

int main(int argc, char **argv)
{
    cbctl_serial_t ser;
    cbctl_link_t link;
    const char *port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    const char *evse_id = NULL;
    uint8_t protocols = 0u;
    int trim = 0, have_trim = 0, want_restart = 0, i;
    long deadline = -1;
    uint32_t t0;
    cbctl_op_t restart_op = { 0 };

    for (i = 2; i < argc; i++) {
        if (strcmp(argv[i], "--evse-id") == 0 && i + 1 < argc) {
            evse_id = argv[++i];
        } else if (strcmp(argv[i], "--protocols") == 0 && i + 1 < argc) {
            if (parse_protocols(argv[++i], &protocols) != 0) {
                fprintf(stderr, "bad --protocols (want din, iso2, or din,iso2)\n");
                return 2;
            }
        } else if (strcmp(argv[i], "--trim") == 0 && i + 1 < argc) {
            trim = atoi(argv[++i]);
            have_trim = 1;
        } else if (strcmp(argv[i], "--feed-deadline") == 0 && i + 1 < argc) {
            deadline = atol(argv[++i]);
        } else if (strcmp(argv[i], "--restart") == 0) {
            want_restart = 1;
        } else if (strcmp(argv[i], "--show") == 0) {
            /* read-only */
        } else {
            fprintf(stderr, "unknown argument '%s'\n", argv[i]);
            return 2;
        }
    }

    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, on_sig);
    signal(SIGTERM, on_sig);

    if (!cbctl_serial_open(&ser, port, 115200u) || cbctl_open(&link, &ser.tx) != CBCTL_OK) {
        fprintf(stderr, "cannot attach %s\n", port);
        return 2;
    }
    /* Installed AFTER cbctl_open — it zeroes the link, so a hook set before would vanish. */
    link.status_observer = on_status;
    link.status_observer_ud = NULL;

    /* REFUSED AT THE CALL, before anything reaches the wire, so a bad EVSEID is a message here
     * rather than a vehicle that stops talking three phases later. */
    if (evse_id != NULL && cbctl_provision_evse_id(&link, evse_id) != CBCTL_OK) {
        fprintf(stderr, "evse-id must be %u..%u printable characters (ISO 15118-2 evseIDType)\n",
                (unsigned)CBPLC_EVSE_ID_MIN, (unsigned)CBPLC_EVSE_ID_MAX);
        return 2;
    }
    if (protocols != 0u && cbctl_provision_protocols(&link, protocols) != CBCTL_OK) {
        fprintf(stderr, "bad protocol mask\n");
        return 2;
    }
    if (have_trim && cbctl_provision_attn_trim_db(&link, (int8_t)trim) != CBCTL_OK) {
        fprintf(stderr, "trim must be %d..%d dB\n", CBPLC_ATTN_TRIM_MIN, CBPLC_ATTN_TRIM_MAX);
        return 2;
    }
    if (deadline >= 0 &&
        cbctl_provision_feed_deadline_ms(&link, (uint16_t)deadline) != CBCTL_OK) {
        fprintf(stderr, "feed deadline must be 0..%u ms (0 = the board's default)\n",
                (unsigned)CBPLC_SESSION_PARAM_TIMEOUT_MAX_MS);
        return 2;
    }

    t0 = now_ms();
    for (;;) {
        const uint32_t t = now_ms();
        struct pollfd pfd;
        if (g_stop || (t - t0) > 12000u) { break; }
        (void)cbctl_tick(&link, t);

        /* The restart is issued only after the declarations have been ACKed — rebooting the
         * board first would simply discard them. */
        if (want_restart == 1 && !link.prov_dirty && cbctl_link_state(&link) == CBCTL_LINK_READY &&
            (t - t0) > 1500u) {
            const cbctl_status_t rc = cbctl_request_restart(&link, &restart_op);
            want_restart = (rc == CBCTL_OK) ? 2 : -1;
            if (rc != CBCTL_OK) {
                printf("restart refused locally: we are holding a relay closed\n");
            }
        }
        if (want_restart == 2 && cbctl_op_state(&link, restart_op) != CBCTL_OP_PENDING) {
            const cbctl_op_state_t st = cbctl_op_state(&link, restart_op);
            printf("restart: %s%s\n",
                   st == CBCTL_OP_APPLIED ? "accepted — the board is rebooting"
                                          : "REFUSED by the board",
                   st == CBCTL_OP_REFUSED ? " (the connector was not idle: vehicle, session or a"
                                            " relay not proven open)" : "");
            break;
        }
        pfd.fd = cbctl_serial_fd(&ser);
        pfd.events = POLLIN;
        pfd.revents = 0;
        (void)poll(&pfd, 1u, 50);
    }

    printf("\n=== the BOARD's own report ===\n");
    printf("  link            : %s\n",
           cbctl_link_state(&link) == CBCTL_LINK_READY ? "ready" : "NOT READY");
    printf("  provisioning    : %s\n",
           cbctl_provision_supported(&link)
               ? "supported"
               : (link.prov_probed ? "UNSUPPORTED — this firmware predates it; the board is on "
                                     "its compiled defaults"
                                   : "not asked"));
    printf("  evse_id         : %s\n", g_prov_id[0] ? g_prov_id : "(none reported)");
    printf("  protocols       : %s\n", g_prov_proto[0] ? g_prov_proto : "?");
    printf("  attn trim (dB)  : %s\n", g_prov_trim[0] ? g_prov_trim : "?");
    printf("  accepted/refused: %s/%s\n", g_prov_ok[0] ? g_prov_ok : "?",
           g_prov_bad[0] ? g_prov_bad : "?");
    printf("  still pending   : %s\n", link.prov_dirty ? "YES — not confirmed by the board" : "no");

    cbctl_serial_close(&ser);
    /* Non-zero when a declaration was made and the board did not confirm it. A provisioning
     * tool that exits 0 on an unconfirmed identity is worse than one that fails. */
    return link.prov_dirty ? 1 : 0;
}
