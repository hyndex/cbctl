/* The POSIX serial port, against a REAL file descriptor.
 *
 * The loopback cannot test this: it fakes the transport, and the whole hazard here is what
 * the kernel does to a real one. A pty pair gives a genuine tty with a genuine finite buffer,
 * and nobody reading the far end is exactly what a wedged PLC looks like.
 */
#include "cbctl/cbctl.h"
#include "posix_serial.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#if defined(__APPLE__)
#include <util.h>
#else
#include <pty.h>
#endif

static int g_fail;
#define CHECK(c)                                                                          \
    do {                                                                                  \
        if (!(c)) {                                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                         \
            g_fail++;                                                                     \
        }                                                                                 \
    } while (0)
#define TEST(name) static void test_##name(void)
#define RUN(name)                                                                         \
    do {                                                                                  \
        printf("  %s\n", #name);                                                          \
        test_##name();                                                                    \
    } while (0)

static uint64_t mono_us(void)
{
    struct timespec ts;
    (void)clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

/* THE ONE THAT MATTERS.
 *
 * Fill a real tty's transmit buffer with nobody draining the far end, then keep writing. Every
 * call must RETURN — deferring, counting, never waiting. A port that blocks here re-creates
 * the 3300 ms-vs-1500 ms defect underneath an API that promises it cannot happen, and every
 * other PLC on the controller loses its charge.
 *
 * Asserted on WALL CLOCK, because "did not block" is a claim about real time and nothing else
 * can measure it. */
TEST(a_full_transmit_buffer_never_blocks_the_caller)
{
    int master = -1, slave = -1;
    cbctl_serial_t s;
    char name[256];
    uint8_t frame[256];
    uint64_t t0, elapsed;
    int i, deferred_seen = 0;

    if (openpty(&master, &slave, name, NULL, NULL) != 0) {
        printf("    (no pty available; skipping)\n");
        return;
    }
    /* The controller talks to the SLAVE side; nobody ever reads the master. */
    CHECK(cbctl_serial_open(&s, name, 115200u));
    if (s.fd < 0) {
        (void)close(master);
        (void)close(slave);
        return;
    }
    (void)close(slave); /* our cbctl_serial_open holds its own fd on the same tty */

    memset(frame, 0xA5, sizeof frame);

    t0 = mono_us();
    for (i = 0; i < 4000; i++) {
        const cbplc_err_t rc = s.tx.send(s.tx.ctx, frame, sizeof frame);
        if (rc == CBPLC_E_TIMEOUT) {
            deferred_seen = 1;
        }
        if (rc == CBPLC_E_TRANSPORT) {
            break; /* the pty went away; not what we are testing */
        }
    }
    elapsed = mono_us() - t0;

    printf("    4000 sends into an undrained tty took %llu us, deferred=%u\n",
           (unsigned long long)elapsed, s.write_deferred);

    /* The buffer MUST have filled — otherwise this test proved nothing about blocking. */
    CHECK(deferred_seen == 1);
    CHECK(s.write_deferred > 0u);
    /* 4000 attempts against a full buffer, if each blocked even briefly, would take seconds.
     * A generous ceiling: the claim is "does not wait", not "is fast". */
    CHECK(elapsed < 2000000u);

    cbctl_serial_close(&s);
    (void)close(master);
}

/* recv on an idle port reports "nothing waiting" as TIMEOUT, not as an error, and returns
 * immediately. Mapping idle onto an error would make a healthy quiet link look broken. */
TEST(an_idle_port_reports_nothing_waiting_without_waiting)
{
    int master = -1, slave = -1;
    cbctl_serial_t s;
    char name[256];
    uint8_t frame[512];
    size_t len = 99u;
    uint64_t t0, elapsed;

    if (openpty(&master, &slave, name, NULL, NULL) != 0) {
        printf("    (no pty available; skipping)\n");
        return;
    }
    CHECK(cbctl_serial_open(&s, name, 115200u));
    if (s.fd < 0) {
        (void)close(master);
        (void)close(slave);
        return;
    }
    (void)close(slave);

    t0 = mono_us();
    CHECK(s.tx.recv(s.tx.ctx, frame, sizeof frame, &len) == CBPLC_E_TIMEOUT);
    elapsed = mono_us() - t0;
    CHECK(len == 0u);          /* and it says so explicitly, rather than leaving stale length */
    CHECK(elapsed < 100000u);  /* returned promptly; VMIN=0/VTIME=0 plus O_NONBLOCK */

    cbctl_serial_close(&s);
    (void)close(master);
}

/* A whole frame written by the port must be readable back through the same framing. This is
 * the round trip the length prefix exists for. */
TEST(a_framed_write_is_recovered_by_the_reader)
{
    int master = -1, slave = -1;
    cbctl_serial_t s;
    char name[256];
    uint8_t out[64], in[512], scratch[512];
    size_t len = 0u;
    ssize_t n;
    unsigned i;

    if (openpty(&master, &slave, name, NULL, NULL) != 0) {
        printf("    (no pty available; skipping)\n");
        return;
    }
    CHECK(cbctl_serial_open(&s, name, 115200u));
    if (s.fd < 0) {
        (void)close(master);
        (void)close(slave);
        return;
    }
    (void)close(slave);

    for (i = 0u; i < sizeof out; i++) {
        out[i] = (uint8_t)i;
    }
    CHECK(s.tx.send(s.tx.ctx, out, sizeof out) == CBPLC_E_OK);

    /* Read the framed bytes off the master and push them back in, so the port's own reader
     * has to recover the frame from the length prefix. */
    n = read(master, scratch, sizeof scratch);
    CHECK(n > 0);
    if (n > 0) {
        CHECK(write(master, scratch, (size_t)n) == n);
        /* A tty round trip is not instant; give the reader a few attempts. */
        for (i = 0u; i < 50u && s.tx.recv(s.tx.ctx, in, sizeof in, &len) != CBPLC_E_OK; i++) {
            struct timespec ts = { 0, 2000000 }; /* 2 ms */
            (void)nanosleep(&ts, NULL);
        }
        CHECK(len == sizeof out);
        CHECK(memcmp(in, out, sizeof out) == 0);
    }

    cbctl_serial_close(&s);
    (void)close(master);
}

/* THE CRASH ON THREE CHARGERS, AS A TEST.
 *
 * A USB hub drops; the port is closed for rebuild; the reopen races the hub's
 * re-enumeration and loses — open(2) fails on a path that is briefly gone. The
 * cbplc session still holds &s.tx from before the drop and polls through it two
 * seconds later. The failed open must therefore leave a transport that is
 * CALLABLE and answers "dead", never one whose function pointers were zeroed.
 * Seeded with garbage so it is open() itself, not luck, that restores the
 * invariant. */
TEST(a_failed_open_leaves_a_callable_transport)
{
    cbctl_serial_t s;
    uint8_t buf[8];
    size_t len = 0u;

    memset(&s, 0xAA, sizeof s);
    CHECK(!cbctl_serial_open(&s, "/nonexistent/tty-gone-mid-reenumeration", 115200u));
    CHECK(s.fd < 0);
    CHECK(s.tx.send != NULL);
    CHECK(s.tx.recv != NULL);
    CHECK(s.tx.recv(s.tx.ctx, buf, sizeof buf, &len) == CBPLC_E_TRANSPORT);
    CHECK(s.tx.send(s.tx.ctx, buf, 1u) == CBPLC_E_TRANSPORT);
}

/* Second, independent layer of the same fix: even if a future transport backend
 * reintroduces a NULL vtable underneath a live session, cbplc_poll must report
 * the transport dead rather than jump through it. */
static cbplc_err_t dummy_send(void *ctx, const uint8_t *f, size_t l)
{
    (void)ctx; (void)f; (void)l;
    return CBPLC_E_OK;
}
static cbplc_err_t dummy_recv(void *ctx, uint8_t *f, size_t cap, size_t *len)
{
    (void)ctx; (void)f; (void)cap;
    *len = 0u;
    return CBPLC_E_TIMEOUT;
}
TEST(poll_through_a_nulled_vtable_reports_transport_dead)
{
    cbplc_session_t p;
    cbplc_transport_t tx;

    memset(&tx, 0, sizeof tx);
    tx.send = dummy_send;
    tx.recv = dummy_recv;
    tx.mtu = CBPLC_STREAM_MAX;
    CHECK(cbplc_open(&p, &tx, NULL, NULL, NULL) == CBPLC_E_OK);
    (void)cbplc_poll(&p, 0u); /* healthy baseline: reaching the next line is the check */
    tx.send = NULL;
    tx.recv = NULL;
    CHECK(cbplc_poll(&p, 1u) == CBPLC_E_TRANSPORT);
}

int main(void)
{
    printf("serial\n");
    RUN(a_full_transmit_buffer_never_blocks_the_caller);
    RUN(an_idle_port_reports_nothing_waiting_without_waiting);
    RUN(a_framed_write_is_recovered_by_the_reader);
    RUN(a_failed_open_leaves_a_callable_transport);
    RUN(poll_through_a_nulled_vtable_reports_transport_dead);
    if (g_fail) {
        printf("FAIL serial (%d)\n", g_fail);
        return 1;
    }
    printf("PASS serial\n");
    return 0;
}
