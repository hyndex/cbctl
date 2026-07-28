/* FEATURE-TEST MACRO, BEFORE ANY INCLUDE.
 *
 * cfmakeraw() and CRTSCTS are BSD extensions. macOS exposes them by default; glibc does not
 * unless _DEFAULT_SOURCE is set, so this file compiled clean on the development Mac and failed
 * outright on the Raspberry Pi it is actually written for — the one target that matters. It
 * has to precede every #include, because the first header pulled in fixes the feature set. */
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE 1
#endif

#include "posix_serial.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifndef CBCTL_SERIAL_CHUNK
#define CBCTL_SERIAL_CHUNK 512u
#endif

static speed_t baud_of(unsigned baud)
{
    switch (baud) {
    case 9600u:   return B9600;
    case 19200u:  return B19200;
    case 38400u:  return B38400;
    case 57600u:  return B57600;
    case 230400u: return B230400;
    case 115200u:
    default:      return B115200;
    }
}

/* ---- the vtable ------------------------------------------------------------------------- */

static cbplc_err_t ser_send(void *ctx, const uint8_t *frame, size_t len)
{
    cbctl_serial_t *s = (cbctl_serial_t *)ctx;
    uint8_t wire[CBPLC_STREAM_MAX + CBPLC_STREAM_PREFIX];
    size_t wlen = 0u;
    ssize_t n;

    if (s == NULL || s->fd < 0) {
        return CBPLC_E_TRANSPORT;
    }
    if (cbplc_stream_wrap(frame, len, wire, sizeof wire, &wlen) != CBPLC_E_OK) {
        return CBPLC_E_NO_SPACE;
    }

    n = write(s->fd, wire, wlen);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            /* THE WHOLE POINT OF THIS FILE. The kernel buffer is full because the peer has
             * stopped reading — a wedged board. We report "try later" and RETURN. Blocking
             * here would stall the tick, which stalls the feed, which withdraws permission to
             * energise on every OTHER board this controller is driving. */
            s->write_deferred++;
            return CBPLC_E_TIMEOUT;
        }
        return CBPLC_E_TRANSPORT;
    }
    if ((size_t)n != wlen) {
        /* A partial write would leave half a frame on the wire and desynchronise the length
         * prefix. Rather than loop (which is blocking by another name), report it as not
         * sent: the session's retry will re-send the whole frame, and the receiver's bound
         * check discards the fragment. Counted, because a link doing this often is sick. */
        s->partial_writes++;
        return CBPLC_E_TIMEOUT;
    }
    return CBPLC_E_OK;
}

static cbplc_err_t ser_recv(void *ctx, uint8_t *frame, size_t cap, size_t *len)
{
    cbctl_serial_t *s = (cbctl_serial_t *)ctx;
    uint8_t chunk[CBCTL_SERIAL_CHUNK];
    ssize_t n;

    *len = 0u;
    if (s == NULL || s->fd < 0) {
        return CBPLC_E_TRANSPORT;
    }

    /* Drain what is already buffered FIRST. A single read() can carry several frames, and
     * returning only one per call while leaving the rest in the accumulator would make the
     * tick's bounded poll count the limiting factor rather than the data. */
    if (cbplc_stream_feed(&s->rx, NULL, 0u, frame, cap, len)) {
        return CBPLC_E_OK;
    }

    n = read(s->fd, chunk, sizeof chunk);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return CBPLC_E_TIMEOUT; /* nothing waiting — the normal idle case, not an error */
        }
        s->read_errors++;
        return CBPLC_E_TRANSPORT;
    }
    if (n == 0) {
        return CBPLC_E_TIMEOUT; /* EOF on a serial port: treat as idle, let the link decide */
    }
    if (cbplc_stream_feed(&s->rx, chunk, (size_t)n, frame, cap, len)) {
        return CBPLC_E_OK;
    }
    return CBPLC_E_TIMEOUT;
}

/* ---- open / close ------------------------------------------------------------------------ */

bool cbctl_serial_open(cbctl_serial_t *s, const char *path, unsigned baud)
{
    struct termios t;

    if (s == NULL || path == NULL) {
        return false;
    }
    memset(s, 0, sizeof *s);
    s->fd = -1;
    cbplc_stream_reset(&s->rx);

    /* O_NONBLOCK on the OPEN as well as the fd: without it, open() itself blocks waiting for
     * carrier on a port with modem-control lines. O_NOCTTY so a serial console cannot make
     * this process a session leader and take signals meant for a terminal. */
    s->fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (s->fd < 0) {
        return false;
    }

#ifdef TIOCEXCL
    /* Exclusive: a second opener gets EBUSY instead of silently stealing our replies. */
    (void)ioctl(s->fd, TIOCEXCL);
#endif

    if (tcgetattr(s->fd, &t) != 0) {
        cbctl_serial_close(s);
        return false;
    }
    cfmakeraw(&t);
    (void)cfsetispeed(&t, baud_of(baud));
    (void)cfsetospeed(&t, baud_of(baud));
    t.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
    t.c_cflag &= (tcflag_t)~CRTSCTS;   /* no hardware flow control: the PLC does not wire it */
    /* Fully non-blocking reads. VMIN=0/VTIME=0 means read() returns immediately with whatever
     * is there, which together with O_NONBLOCK is what makes recv honour "never blocks". */
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    if (tcsetattr(s->fd, TCSANOW, &t) != 0) {
        cbctl_serial_close(s);
        return false;
    }
    /* ASSERT DTR (AND RTS) EXPLICITLY. THIS IS NOT OPTIONAL ON THIS HARDWARE.
     *
     * The ESP32-S3 USB-Serial-JTAG bridge closes its CDC endpoint when DTR is DEASSERTED, so a
     * port opened without touching the modem-control lines enumerates perfectly, accepts
     * writes, and delivers nothing. macOS happens to leave the lines in a state that works;
     * Linux does not — both boards on the Raspberry Pi identified as dead and quarantined,
     * with a clean transport and zero read errors, which is exactly what "the endpoint is
     * closed" looks like from up here.
     *
     * cbplc's own Python harness has always done this and says why in one line: "DTR must stay
     * asserted in all of them — a deasserted DTR closes the CDC endpoint on this hardware."
     * The C port simply never learned it.
     *
     * RTS is asserted too because that is the combination the harness finds working first on
     * this hardware; its fallback is DTR-only (RTS low), which is worth trying by hand if a
     * future board refuses to talk. */
    {
        const int lines = TIOCM_DTR | TIOCM_RTS;
        (void)ioctl(s->fd, TIOCMBIS, &lines);
    }
    /* Let the endpoint settle before the first frame. This runs once at open, never in the
     * tick, so it costs nothing that matters and it saves a first-request timeout. */
    {
        struct timespec ts = { 0, 150 * 1000 * 1000 };  /* 150 ms */
        (void)nanosleep(&ts, NULL);
    }
    (void)tcflush(s->fd, TCIOFLUSH);

    s->tx.ctx = s;
    s->tx.send = ser_send;
    s->tx.recv = ser_recv;
    s->tx.mtu = CBPLC_STREAM_MAX;
    return true;
}

void cbctl_serial_close(cbctl_serial_t *s)
{
    if (s != NULL && s->fd >= 0) {
        (void)close(s->fd);
        s->fd = -1;
    }
}

int cbctl_serial_fd(const cbctl_serial_t *s)
{
    return (s != NULL) ? s->fd : -1;
}
