#include "cbplc/stream_frame.h"

#include <string.h>

void cbplc_stream_reset(cbplc_stream_rx_t *rx)
{
    if (rx != NULL) {
        rx->len = 0u;
    }
}

cbplc_err_t cbplc_stream_wrap(const uint8_t *frame, size_t len,
                              uint8_t *out, size_t out_cap, size_t *out_len)
{
    if (frame == NULL || out == NULL || out_len == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    if (len == 0u || len > CBPLC_STREAM_MAX) {
        return CBPLC_E_BAD_LENGTH;
    }
    if (out_cap < len + CBPLC_STREAM_PREFIX) {
        return CBPLC_E_SHORT_BUFFER;
    }
    out[0] = (uint8_t)(len & 0xFFu);
    out[1] = (uint8_t)((len >> 8) & 0xFFu);
    memcpy(&out[CBPLC_STREAM_PREFIX], frame, len);
    *out_len = len + CBPLC_STREAM_PREFIX;
    return CBPLC_E_OK;
}

static size_t declared(const cbplc_stream_rx_t *rx)
{
    return (size_t)rx->buf[0] | ((size_t)rx->buf[1] << 8);
}

bool cbplc_stream_feed(cbplc_stream_rx_t *rx,
                       const uint8_t *in, size_t in_len,
                       uint8_t *frame, size_t frame_cap, size_t *frame_len)
{
    if (rx == NULL || frame == NULL || frame_len == NULL) {
        return false;
    }
    *frame_len = 0u;

    for (size_t i = 0; i < in_len; ++i) {
        if (rx->len >= sizeof rx->buf) {
            /* Cannot happen while the length check below runs, but resetting here
             * rather than trusting that keeps a future edit from turning a bug into
             * an overflow. */
            rx->overflows++;
            rx->len = 0u;
        }
        rx->buf[rx->len++] = in[i];
        if (rx->len == CBPLC_STREAM_PREFIX) {
            const size_t want = declared(rx);
            /* A bogus prefix is discarded IMMEDIATELY. This is the whole advantage
             * over a sentinel scan: a bad length can never silently consume the
             * next real frame's bytes. */
            if (want == 0u || want > CBPLC_STREAM_MAX) {
                rx->bad_length++;
                rx->len = 0u;
            }
        }
    }

    if (rx->len < CBPLC_STREAM_PREFIX) {
        return false;
    }
    const size_t want = declared(rx);
    if (rx->len < CBPLC_STREAM_PREFIX + want) {
        return false; /* still arriving */
    }
    if (frame_cap < want) {
        rx->len = 0u;
        return false;
    }
    memcpy(frame, &rx->buf[CBPLC_STREAM_PREFIX], want);
    *frame_len = want;

    /* Carry over trailing bytes: one USB packet can hold several frames, and
     * dropping the remainder would lose them. */
    const size_t consumed = CBPLC_STREAM_PREFIX + want;
    const size_t remain = rx->len - consumed;
    if (remain > 0u) {
        memmove(rx->buf, &rx->buf[consumed], remain);
    }
    rx->len = remain;
    return true;
}
