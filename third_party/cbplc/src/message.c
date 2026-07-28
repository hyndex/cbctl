#include "cbplc/message.h"

#include <string.h>

/* Header layout (little-endian), 12 bytes:
 *   0     version
 *   1     type
 *   2..3  request_id
 *   4..5  payload_len
 *   6..7  reserved, must be 0
 *   8..11 crc32 over the whole message with these 4 bytes treated as zero
 *
 * The CRC covers the header too, so a corrupted length or type is caught by the
 * same check as a corrupted payload. Zeroing the CRC field during computation is
 * what makes encode and decode able to use one helper.
 */
#define OFF_VERSION 0u
#define OFF_TYPE    1u
#define OFF_REQID   2u
#define OFF_LEN     4u
#define OFF_FLAGS   6u
#define OFF_CRC     8u

/* Bitwise CRC-32 rather than a 1 KiB table.
 *
 * ponytail: a table would be ~8x faster and cost 1 KiB of flash plus a
 * generator. Measured need first: the largest message is 524 bytes and the link
 * is idle most of the time, so this is microseconds on the slowest target we
 * support. If profiling ever shows this on a hot path, add the table then — the
 * function boundary is already here for it.
 */
uint32_t cbplc_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    if (data == NULL) {
        return crc ^ 0xFFFFFFFFu;
    }
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            /* 0xEDB88320 is the reflected IEEE 802.3 polynomial. Must match the
             * harness's zlib.crc32 exactly; a test pins a known vector. */
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

bool cbplc_type_known(uint8_t t)
{
    switch ((cbplc_type_t)t) {
    /* Deliberately exhaustive with no `default`: adding a type to the schema
     * makes this a compile error until it is listed, which is the cheapest
     * possible reminder that a new message needs a decision here. */
    case CBPLC_T_PING:
    case CBPLC_T_PONG:
    case CBPLC_T_IDENTIFY:
    case CBPLC_T_IDENTITY:
    case CBPLC_T_HEARTBEAT:
    case CBPLC_T_AUTHORIZE:
    case CBPLC_T_SLAC:
    case CBPLC_T_RELAY:
    case CBPLC_T_LIMITS:
    case CBPLC_T_STOP:
    case CBPLC_T_FEEDBACK:
    case CBPLC_T_STATUS_REQUEST:
    case CBPLC_T_STATUS:
    case CBPLC_T_ACK:
    case CBPLC_T_EVENT:
    case CBPLC_T_DIAG:
        return true;
    }
    return false;
}

cbplc_err_t cbplc_msg_encode(cbplc_type_t type,
                             uint16_t request_id,
                             bool is_response,
                             const uint8_t *payload,
                             uint16_t payload_len,
                             uint8_t *out,
                             size_t out_cap,
                             size_t *out_len)
{
    if (out == NULL || out_len == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    if (payload_len > 0u && payload == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    if (payload_len > CBPLC_MAX_PAYLOAD) {
        return CBPLC_E_BAD_LENGTH;
    }
    if (!cbplc_type_known((uint8_t)type)) {
        return CBPLC_E_BAD_TYPE;
    }
    const size_t total = (size_t)CBPLC_MSG_HEADER_SIZE + payload_len;
    if (out_cap < total) {
        /* Write nothing on failure. A partially-filled buffer is the kind of
         * thing a caller forgets to check the error for. */
        return CBPLC_E_SHORT_BUFFER;
    }

    out[OFF_VERSION] = (uint8_t)CBPLC_WIRE_VERSION;
    out[OFF_TYPE] = (uint8_t)type;
    put_u16(&out[OFF_REQID], request_id);
    put_u16(&out[OFF_LEN], payload_len);
    put_u16(&out[OFF_FLAGS], is_response ? CBPLC_FLAG_RESPONSE : 0u);
    put_u32(&out[OFF_CRC], 0u); /* zero while computing */
    if (payload_len > 0u) {
        memcpy(&out[CBPLC_MSG_HEADER_SIZE], payload, payload_len);
    }
    put_u32(&out[OFF_CRC], cbplc_crc32(out, total));
    *out_len = total;
    return CBPLC_E_OK;
}

cbplc_err_t cbplc_msg_decode(const uint8_t *in, size_t in_len, cbplc_msg_t *msg)
{
    if (in == NULL || msg == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    if (in_len < CBPLC_MSG_HEADER_SIZE) {
        return CBPLC_E_INCOMPLETE;
    }
    if (in[OFF_VERSION] != (uint8_t)CBPLC_WIRE_VERSION) {
        /* Distinct from a parse failure on purpose: "we speak different
         * versions" is actionable (upgrade one end), "malformed" is not. */
        return CBPLC_E_BAD_VERSION;
    }
    const uint16_t flags = get_u16(&in[OFF_FLAGS]);
    /* Only bit 0 (RESPONSE) is defined. Every other bit is still rejected, which
     * is what keeps them genuinely reservable: if we ignored an unknown bit now,
     * we could never assign it meaning later without silently changing how an
     * older peer behaves. */
    if ((flags & (uint16_t)~(uint16_t)CBPLC_FLAG_RESPONSE) != 0u) {
        return CBPLC_E_BAD_RESERVED;
    }

    const uint16_t payload_len = get_u16(&in[OFF_LEN]);
    if (payload_len > CBPLC_MAX_PAYLOAD) {
        return CBPLC_E_BAD_LENGTH;
    }
    const size_t total = (size_t)CBPLC_MSG_HEADER_SIZE + payload_len;
    if (in_len < total) {
        return CBPLC_E_INCOMPLETE;
    }

    /* Verify the CRC before trusting the type, so a corrupted type byte is
     * reported as corruption rather than as an unsupported operation. */
    const uint32_t claimed = get_u32(&in[OFF_CRC]);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < total; ++i) {
        const uint8_t b = (i >= OFF_CRC && i < OFF_CRC + 4u) ? 0u : in[i];
        crc ^= b;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) ? ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
        }
    }
    if ((crc ^ 0xFFFFFFFFu) != claimed) {
        return CBPLC_E_BAD_CRC;
    }
    if (!cbplc_type_known(in[OFF_TYPE])) {
        return CBPLC_E_BAD_TYPE;
    }

    msg->type = (cbplc_type_t)in[OFF_TYPE];
    msg->request_id = get_u16(&in[OFF_REQID]);
    msg->is_response = (flags & CBPLC_FLAG_RESPONSE) != 0u;
    msg->payload = (payload_len > 0u) ? &in[CBPLC_MSG_HEADER_SIZE] : NULL;
    msg->payload_len = payload_len;
    return CBPLC_E_OK;
}
