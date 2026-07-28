/* Frame delimiting for STREAM transports (UART, USB-CDC).
 *
 * A packet link (CAN) has frame boundaries; a stream does not. This adds them with
 * a 2-byte little-endian length prefix.
 *
 * WHY A LENGTH PREFIX AND NOT A MAGIC/SENTINEL SCAN
 *   plc_firmware scanned for a magic word, which needs a resynchroniser and a
 *   discard-evidence buffer to stay diagnosable after loss. A length prefix cannot
 *   half-consume a frame: a bogus length is caught by one bound check, immediately,
 *   and the accumulator resets. Smaller code and a better failure mode.
 *
 * WHY IT LIVES IN THE CORE
 *   Two adapters need it (UART and USB-CDC), and a host test must be able to feed
 *   it byte-at-a-time and in bursts. Keeping it inside one adapter would make it
 *   untestable and then duplicated.
 *
 * WHAT IT IS NOT
 *   Not integrity. A length prefix says where a frame ends, not that its contents
 *   are intact — the message CRC-32 owns that, once. Do not add a checksum here.
 */
#ifndef CBPLC_STREAM_FRAME_H
#define CBPLC_STREAM_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CBPLC_STREAM_PREFIX 2u
#define CBPLC_STREAM_MAX (CBPLC_MAX_MESSAGE + CBPLC_SEG_HEADER_SIZE)

typedef struct {
    uint8_t buf[CBPLC_STREAM_MAX + CBPLC_STREAM_PREFIX];
    size_t len;
    /* Counters so a desynchronising peer is diagnosable rather than merely
     * ignored. Both can actually move. */
    uint32_t bad_length;
    uint32_t overflows;
} cbplc_stream_rx_t;

void cbplc_stream_reset(cbplc_stream_rx_t *rx);

/* Write `len` bytes framed into `out`. Returns the total written via `*out_len`. */
cbplc_err_t cbplc_stream_wrap(const uint8_t *frame, size_t len,
                              uint8_t *out, size_t out_cap, size_t *out_len);

/* Feed received bytes. Returns true and fills `frame`/`frame_len` when a whole
 * frame is available; call again with 0 bytes to drain further complete frames from
 * what was already buffered (a burst can carry several). */
bool cbplc_stream_feed(cbplc_stream_rx_t *rx,
                       const uint8_t *in, size_t in_len,
                       uint8_t *frame, size_t frame_cap, size_t *frame_len);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_STREAM_FRAME_H */
