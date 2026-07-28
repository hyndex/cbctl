/* The DIAG control codec. Portable C, no platform, so a host test can reach every branch.
 *
 * This file exists so that the firmware and every controller speak ONE encoding. cbctl vendors
 * it verbatim rather than reimplementing it, for the same reason it vendors the session core: a
 * second implementation is free to disagree, and the disagreement surfaces on a charger. */
#include "cbplc/diag.h"

#include <string.h>

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

size_t cbplc_diag_encode_request(cbplc_diag_cmd_t cmd, uint32_t arg, uint8_t *out, size_t cap)
{
    if (out == NULL) {
        return 0u;
    }
    switch (cmd) {
    case CBPLC_DIAG_DRAIN:
    case CBPLC_DIAG_GET_STATE:
    case CBPLC_DIAG_CLEAR:
        if (cap < 1u) {
            return 0u;
        }
        out[0] = (uint8_t)cmd;
        return 1u;
    case CBPLC_DIAG_SET_LEVEL:
        /* Refused at the call. A level the board would reject is a level the caller should be
         * told about now, not one round trip later via a status byte it may not check. */
        if (arg > CBPLC_DIAG_LEVEL_OFF || cap < 2u) {
            return 0u;
        }
        out[0] = (uint8_t)cmd;
        out[1] = (uint8_t)arg;
        return 2u;
    case CBPLC_DIAG_SET_CATS:
        if (cap < 5u) {
            return 0u;
        }
        out[0] = (uint8_t)cmd;
        put_u32(&out[1], arg);
        return 5u;
    default:
        return 0u;
    }
}

cbplc_err_t cbplc_diag_decode_request(const uint8_t *payload, uint16_t len,
                                      cbplc_diag_cmd_t *cmd, uint32_t *arg)
{
    if (cmd == NULL || arg == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    *arg = 0u;
    /* THE EMPTY PAYLOAD IS DRAIN, FOREVER. Every controller built before subcommands existed
     * sends exactly this, and the fleet contains such controllers. */
    if (len == 0u) {
        *cmd = CBPLC_DIAG_DRAIN;
        return CBPLC_E_OK;
    }
    if (payload == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    switch (payload[0]) {
    case (uint8_t)CBPLC_DIAG_DRAIN:
    case (uint8_t)CBPLC_DIAG_GET_STATE:
    case (uint8_t)CBPLC_DIAG_CLEAR:
        *cmd = (cbplc_diag_cmd_t)payload[0];
        return CBPLC_E_OK;
    case (uint8_t)CBPLC_DIAG_SET_LEVEL:
        if (len < 2u || payload[1] > CBPLC_DIAG_LEVEL_OFF) {
            return CBPLC_E_BAD_ARGUMENT;
        }
        *cmd = CBPLC_DIAG_SET_LEVEL;
        *arg = payload[1];
        return CBPLC_E_OK;
    case (uint8_t)CBPLC_DIAG_SET_CATS:
        if (len < 5u) {
            return CBPLC_E_BAD_ARGUMENT;
        }
        *cmd = CBPLC_DIAG_SET_CATS;
        *arg = get_u32(&payload[1]);
        return CBPLC_E_OK;
    default:
        /* An unknown subcommand is REFUSED, not treated as a drain. Silently draining on a
         * value we do not understand would let a future controller believe it had disabled
         * logging on a board that carried on recording at full rate. */
        return CBPLC_E_BAD_ARGUMENT;
    }
}

size_t cbplc_diag_encode_reply(const cbplc_diag_state_t *st, uint8_t *out, size_t cap)
{
    if (st == NULL || out == NULL || cap < CBPLC_DIAG_REPLY_SIZE) {
        return 0u;
    }
    memset(out, 0, CBPLC_DIAG_REPLY_SIZE);
    /* Byte 0 is a NUL BY CONSTRUCTION. cbplc_diag_read() returns printf output, which cannot
     * contain one, so this single byte is what makes "control reply" and "log text"
     * distinguishable without any version negotiation. Do not repurpose it. */
    out[0] = 0x00u;
    out[1] = CBPLC_DIAG_REPLY_MAGIC;
    out[2] = CBPLC_DIAG_REPLY_VERSION;
    out[3] = st->status;
    out[4] = st->level;
    out[5] = 0u; /* reserved; must stay zero so it remains genuinely reservable */
    put_u32(&out[6], st->categories);
    put_u32(&out[10], st->dropped);
    out[14] = (uint8_t)(st->pending & 0xFFu);
    out[15] = (uint8_t)((st->pending >> 8) & 0xFFu);
    return CBPLC_DIAG_REPLY_SIZE;
}

bool cbplc_diag_is_reply(const uint8_t *payload, uint16_t len)
{
    return payload != NULL && len == CBPLC_DIAG_REPLY_SIZE && payload[0] == 0x00u &&
           payload[1] == CBPLC_DIAG_REPLY_MAGIC && payload[2] == CBPLC_DIAG_REPLY_VERSION;
}

bool cbplc_diag_decode_reply(const uint8_t *payload, uint16_t len, cbplc_diag_state_t *st)
{
    if (st == NULL || !cbplc_diag_is_reply(payload, len)) {
        return false;
    }
    st->status = payload[3];
    st->level = payload[4];
    st->categories = get_u32(&payload[6]);
    st->dropped = get_u32(&payload[10]);
    st->pending = (uint32_t)payload[14] | ((uint32_t)payload[15] << 8);
    return true;
}
