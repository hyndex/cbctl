#include "cbplc/segment.h"

#include <string.h>

#define H_ID    0u
#define H_INDEX 1u
#define H_COUNT 2u
#define H_FLAGS 3u

/* Wrap-safe elapsed. Never compare absolute stamps with `<`: a 32-bit
 * millisecond clock wraps every 49.7 days and the field will reach it. Unsigned
 * subtraction is correct across the wrap by construction. */
static bool expired(uint32_t now_ms, uint32_t since_ms, uint32_t timeout_ms)
{
    return (uint32_t)(now_ms - since_ms) > timeout_ms;
}

uint16_t cbplc_seg_count(size_t msg_len, size_t mtu)
{
    if (mtu < CBPLC_MIN_TRANSPORT_MTU || msg_len == 0u) {
        return 0u;
    }
    const size_t per = mtu - CBPLC_SEG_HEADER_SIZE;
    const size_t n = (msg_len + per - 1u) / per;
    if (n > CBPLC_MAX_SEGMENTS) {
        return 0u;
    }
    return (uint16_t)n;
}

cbplc_err_t cbplc_split_begin(cbplc_splitter_t *s,
                              const uint8_t *msg,
                              size_t msg_len,
                              size_t mtu,
                              uint8_t message_id)
{
    if (s == NULL || msg == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* An empty message is refused rather than encoded as one empty segment:
     * there is no valid zero-length cbplc message, so accepting it here would let
     * a bug travel one layer further before failing. */
    if (msg_len == 0u || msg_len > CBPLC_MAX_MESSAGE) {
        return CBPLC_E_BAD_LENGTH;
    }
    if (mtu < CBPLC_MIN_TRANSPORT_MTU) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    const uint16_t total = cbplc_seg_count(msg_len, mtu);
    if (total == 0u) {
        return CBPLC_E_NO_SPACE;
    }
    s->msg = msg;
    s->msg_len = msg_len;
    s->mtu = mtu;
    s->total = total;
    s->next = 0u;
    s->message_id = message_id;
    return CBPLC_E_OK;
}

bool cbplc_split_done(const cbplc_splitter_t *s)
{
    return s == NULL || s->next >= s->total;
}

cbplc_err_t cbplc_split_next(cbplc_splitter_t *s,
                             uint8_t *frame,
                             size_t frame_cap,
                             size_t *frame_len)
{
    if (s == NULL || frame == NULL || frame_len == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    if (cbplc_split_done(s)) {
        return CBPLC_E_SEQUENCE;
    }
    if (frame_cap < CBPLC_SEG_HEADER_SIZE + 1u) {
        return CBPLC_E_SHORT_BUFFER;
    }
    const size_t per = s->mtu - CBPLC_SEG_HEADER_SIZE;
    const size_t offset = (size_t)s->next * per;
    size_t chunk = s->msg_len - offset;
    if (chunk > per) {
        chunk = per;
    }
    if (frame_cap < CBPLC_SEG_HEADER_SIZE + chunk) {
        return CBPLC_E_SHORT_BUFFER;
    }

    frame[H_ID] = s->message_id;
    frame[H_INDEX] = (uint8_t)s->next;
    frame[H_COUNT] = (uint8_t)s->total;
    frame[H_FLAGS] = 0u;
    memcpy(&frame[CBPLC_SEG_HEADER_SIZE], &s->msg[offset], chunk);
    *frame_len = CBPLC_SEG_HEADER_SIZE + chunk;
    s->next++;
    return CBPLC_E_OK;
}

void cbplc_join_reset(cbplc_joiner_t *j)
{
    if (j == NULL) {
        return;
    }
    j->len = 0u;
    j->expect_index = 0u;
    j->total = 0u;
    j->active = false;
}

cbplc_rx_t cbplc_join_feed(cbplc_joiner_t *j,
                           const uint8_t *frame,
                           size_t frame_len,
                           uint32_t now_ms,
                           uint32_t timeout_ms)
{
    if (j == NULL) {
        return CBPLC_RX_REJECTED;
    }
    /* A frame must carry a header plus at least one payload byte, and cannot
     * exceed a whole message. Rejected frames change no state at all, so a
     * flood of noise cannot disturb a healthy in-progress reassembly. */
    if (frame == NULL || frame_len <= CBPLC_SEG_HEADER_SIZE ||
        frame_len > CBPLC_MAX_MESSAGE + CBPLC_SEG_HEADER_SIZE) {
        j->rejects++;
        return CBPLC_RX_REJECTED;
    }
    if (frame[H_FLAGS] != 0u) {
        j->rejects++;
        return CBPLC_RX_REJECTED;
    }

    const uint8_t id = frame[H_ID];
    const uint16_t index = frame[H_INDEX];
    const uint16_t count = frame[H_COUNT];
    const size_t chunk = frame_len - CBPLC_SEG_HEADER_SIZE;

    if (count == 0u || index >= count || count > CBPLC_MAX_SEGMENTS) {
        j->rejects++;
        return CBPLC_RX_REJECTED;
    }

    /* Age out a stale partial BEFORE looking at this frame, so a peer that
     * vanished mid-message cannot make the next message inherit its tail. */
    bool timed_out = false;
    if (j->active && expired(now_ms, j->started_ms, timeout_ms)) {
        cbplc_join_reset(j);
        j->discards++;
        timed_out = true;
    }

    const bool is_first = (index == 0u);

    if (!j->active) {
        /* Only a first segment may open a message. A middle segment arriving with
         * nothing in progress is the tail of something already abandoned; dropping
         * it is the entire point of carrying an index. */
        if (!is_first) {
            j->discards++;
            return CBPLC_RX_DISCARDED;
        }
        /* Reject an over-length message from its FIRST segment, before buffering
         * anything. This is why `count` is explicit rather than inferred. */
        if ((size_t)count * chunk > CBPLC_MAX_MESSAGE + chunk) {
            j->rejects++;
            return CBPLC_RX_REJECTED;
        }
        j->active = true;
        j->len = 0u;
        j->expect_index = 0u;
        j->total = count;
        j->message_id = id;
        j->started_ms = now_ms;
    } else if (id != j->message_id || count != j->total ||
               index != j->expect_index) {
        /* Either the peer moved on without finishing, or a segment was lost.
         * Both mean this partial can never complete correctly. */
        cbplc_join_reset(j);
        j->discards++;
        if (!is_first) {
            return CBPLC_RX_DISCARDED;
        }
        /* The offending frame is itself a valid start, so adopt it rather than
         * throwing away two messages instead of one. */
        j->active = true;
        j->total = count;
        j->message_id = id;
        j->started_ms = now_ms;
    }

    if (j->len + chunk > CBPLC_MAX_MESSAGE) {
        cbplc_join_reset(j);
        j->discards++;
        return CBPLC_RX_DISCARDED;
    }
    memcpy(&j->buf[j->len], &frame[CBPLC_SEG_HEADER_SIZE], chunk);
    j->len += chunk;
    j->expect_index = (uint16_t)(index + 1u);

    if (j->expect_index >= j->total) {
        j->active = false; /* buf/len stay valid until the next feed */
        return CBPLC_RX_COMPLETE;
    }
    return timed_out ? CBPLC_RX_DISCARDED : CBPLC_RX_ACCUMULATING;
}
