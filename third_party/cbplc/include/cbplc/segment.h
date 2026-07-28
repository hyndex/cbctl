/* Segmentation: carry one message across N transport frames.
 *
 * WHY SEGMENTATION IS ALWAYS ON
 *   plc_firmware assumed a byte stream, then needed to carry something larger
 *   than its internal frame, and solved that by splitting the data across a frame
 *   AND a parallel array re-paired afterwards by ring index. Two channels for one
 *   message is a bug generator. Here, "larger than one frame" is the normal case
 *   and N==1 is just the common one. A stream transport pays 4 bytes per frame for
 *   that; in exchange there is one code path instead of two.
 *
 * WHAT THIS LAYER IS NOT
 *   It is NOT an integrity mechanism. It does not look inside the message, has no
 *   checksum of its own, and never inspects a type. Reassembly hands bytes to
 *   cbplc_msg_decode(), which owns version/length/CRC. A weaker link therefore
 *   cannot earn a weaker guarantee.
 *
 *   Say this out loud because it is the assumption a reader gets backwards: the
 *   segment index detects LOSS cheaply. It does not detect corruption, and it
 *   cannot detect every loss pattern. The message CRC-32 is what rejects a bad
 *   reassembly, and a test asserts exactly that.
 *
 * HEADER, 4 bytes
 *   0  message_id  wraps at 256; separates interleaved and abandoned messages
 *   1  index       0-based
 *   2  count       total segments in this message
 *   3  flags       reserved, must be 0
 *
 * `count` is carried explicitly rather than inferred from a more-follows bit so a
 * receiver knows the full size from the FIRST segment and can reject an
 * over-length message before buffering any of it.
 *
 * WHY message_id EXISTS
 *   A message abandoned midway (peer reset, bus-off) leaves segments in flight
 *   that otherwise look exactly like the continuation of whatever starts next.
 *   The id turns silent contamination into a detected mismatch.
 */
#ifndef CBPLC_SEGMENT_H
#define CBPLC_SEGMENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

/* How many frames a message of `msg_len` needs at this MTU. 0 if impossible
 * (MTU too small, or the message needs more than CBPLC_MAX_SEGMENTS). Callers use
 * it to decide affordability before committing to a send. */
uint16_t cbplc_seg_count(size_t msg_len, size_t mtu);

/* --- sender ----------------------------------------------------------- */
/* Holds a pointer to the caller's message buffer and does not copy it, so that
 * buffer must outlive the splitter. Pull-based: the caller emits frames at
 * whatever rate its egress budget allows, so one large message cannot monopolise
 * a shared bus in a single burst. */
typedef struct {
    const uint8_t *msg;
    size_t msg_len;
    size_t mtu;
    uint16_t total;
    uint16_t next;
    uint8_t message_id;
} cbplc_splitter_t;

cbplc_err_t cbplc_split_begin(cbplc_splitter_t *s,
                              const uint8_t *msg,
                              size_t msg_len,
                              size_t mtu,
                              uint8_t message_id);

bool cbplc_split_done(const cbplc_splitter_t *s);

/* Write exactly one frame. A short final frame is NOT padded: padding would make
 * the reassembled length depend on the header being right, and an unpadded length
 * is self-evident. */
cbplc_err_t cbplc_split_next(cbplc_splitter_t *s,
                             uint8_t *frame,
                             size_t frame_cap,
                             size_t *frame_len);

/* --- receiver --------------------------------------------------------- */
typedef enum {
    CBPLC_RX_ACCUMULATING = 0, /* accepted, waiting for more */
    CBPLC_RX_COMPLETE,         /* a message is ready: read msg/msg_len now */
    CBPLC_RX_DISCARDED,        /* partial dropped: gap, id change, or timeout */
    CBPLC_RX_REJECTED,         /* frame itself unusable; state unchanged */
} cbplc_rx_t;

typedef struct {
    uint8_t buf[CBPLC_MAX_MESSAGE];
    size_t len;
    uint32_t started_ms;
    uint16_t expect_index;
    uint16_t total;
    uint8_t message_id;
    bool active;
    /* Counters a peer can actually act on. Every one CAN move; a counter that
     * structurally cannot change reads as evidence of health rather than as
     * absence of measurement, which is why plc_firmware's frozen queue counters
     * were deleted rather than pinned at zero. */
    uint32_t discards;
    uint32_t rejects;
} cbplc_joiner_t;

void cbplc_join_reset(cbplc_joiner_t *j);

/* Feed one frame. `now_ms` is the caller's monotonic clock — this layer never
 * reads a clock itself, which is what lets a test drive reassembly timeouts with
 * a virtual clock and no sleeping. */
cbplc_rx_t cbplc_join_feed(cbplc_joiner_t *j,
                           const uint8_t *frame,
                           size_t frame_len,
                           uint32_t now_ms,
                           uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_SEGMENT_H */
