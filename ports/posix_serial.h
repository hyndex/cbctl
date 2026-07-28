/* POSIX serial transport (Linux, macOS) for USB-CDC and UART.
 *
 * THIS FILE CARRIES THE LARGEST RESIDUAL HAZARD IN THE WHOLE DESIGN, so it is worth stating
 * plainly. cbctl's send/recv are callbacks invoked INSIDE cbctl_tick(). A sans-IO core does
 * not save you from a blocking callback — nghttp2 documents the same trap for its callback
 * API — and on POSIX a serial write() WILL block once the kernel TX buffer fills and the peer
 * stops reading, which is exactly what a wedged PLC looks like from here.
 *
 * A port that blocks re-creates the 3300 ms-vs-1500 ms defect underneath an API that promises
 * it cannot happen, which is worse than the original because the promise stops anyone looking.
 * So: O_NONBLOCK, and EAGAIN is reported as CBPLC_E_TIMEOUT (try later), never waited on.
 * Every new transport must be reviewed for exactly this.
 */
#ifndef CBCTL_POSIX_SERIAL_H
#define CBCTL_POSIX_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

#include "cbplc/stream_frame.h"
#include "cbplc/transport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int fd;
    cbplc_transport_t tx;      /* hand this to cbctl_open() */
    cbplc_stream_rx_t rx;      /* the 2-byte length-prefix accumulator */

    /* Counters, because "it stopped working" is not a diagnosis. */
    uint32_t write_deferred;   /* EAGAIN: the TX buffer was full and we did NOT wait */
    uint32_t read_errors;
    uint32_t partial_writes;   /* a frame that did not leave in one write() */
} cbctl_serial_t;

/* Open exclusively at `baud` (B115200 etc. are handled internally for common rates).
 *
 * EXCLUSIVE IS NOT OPTIONAL. Two readers on one port silently steal each other's replies —
 * this project lost a debugging session to exactly that on the Python side — so a second
 * opener is refused rather than allowed to interleave.
 *
 * Returns false and leaves fd < 0 on failure. */
bool cbctl_serial_open(cbctl_serial_t *s, const char *path, unsigned baud);

void cbctl_serial_close(cbctl_serial_t *s);

/* The pollable descriptor, for hosts that have one. Returns -1 when there is none — which is
 * not an error, it is the honest answer, and it is why cbctl_next_deadline_ms() is the
 * mandatory half of the loop-integration contract and this is the optional half. */
int cbctl_serial_fd(const cbctl_serial_t *s);

#ifdef __cplusplus
}
#endif
#endif /* CBCTL_POSIX_SERIAL_H */
