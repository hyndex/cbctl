/* The PROVISION codec. Portable C, no platform, so a host test can reach every branch.
 *
 * cbctl vendors this file verbatim rather than reimplementing it — the same rule the session
 * core follows. A second implementation is free to disagree, and the disagreement surfaces as a
 * charger announcing an identity its controller did not send. */
#include "cbplc/provision.h"

#include <string.h>

/* THE COMPILED DEFAULTS ARE THE VALUES THAT SHIPPED BEFORE THIS EXISTED.
 *
 * That is the compatibility contract: a board nobody provisions behaves exactly as it always
 * has, so rolling this firmware onto a fleet with un-updated controllers changes nothing. The
 * EVSE ID default deliberately stays empty here — secc_dc.cpp owns the fallback string, and
 * duplicating it would create two literals free to drift apart. */
void cbplc_provision_defaults(cbplc_provision_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof *p);
    p->protocol_mask = CBPLC_PROTO_ALL;
    p->attn_trim_db = 0;
    p->session_timeout_ms = 0u;
}

static bool evse_id_plausible(const uint8_t *v, uint8_t len)
{
    uint8_t i;
    if (len < CBPLC_EVSE_ID_MIN || len > CBPLC_EVSE_ID_MAX) {
        return false;
    }
    /* PRINTABLE ASCII ONLY. The EVSEID goes onto the wire inside an EXI string and is compared
     * by vehicles; a stray control byte or NUL there produces a schema-invalid message that a
     * strict EV rejects on every message of the charge loop. Rejecting it here turns a silent
     * interop failure into a refused declaration the controller can see. */
    for (i = 0u; i < len; i++) {
        if (v[i] < 0x20u || v[i] > 0x7Eu) {
            return false;
        }
    }
    return true;
}

/* ONE PASS TO VALIDATE, A SECOND TO APPLY.
 *
 * The two-pass shape is the whole reason this is all-or-nothing. Validating as we go and
 * writing as we go would leave a frame whose last TLV is malformed with its earlier fields
 * already applied — a charger running half a declaration, announcing a new EVSE ID with an old
 * protocol mask, and nothing anywhere saying so. */
static cbplc_err_t walk(const uint8_t *payload, uint16_t len, cbplc_provision_t *out)
{
    uint16_t i = 0u;

    while (i < len) {
        uint8_t tag;
        uint8_t vlen;
        const uint8_t *v;

        /* A tag needs its length byte; a header split across the end of the payload is a
         * truncated frame, not an empty field. */
        if ((uint16_t)(len - i) < 2u) {
            return CBPLC_E_BAD_LENGTH;
        }
        tag = payload[i];
        vlen = payload[i + 1u];
        v = &payload[i + 2u];
        if ((uint16_t)(len - i - 2u) < (uint16_t)vlen) {
            return CBPLC_E_BAD_LENGTH;
        }

        switch (tag) {
        case CBPLC_PROV_EVSE_ID:
            if (!evse_id_plausible(v, vlen)) {
                return CBPLC_E_BAD_ARGUMENT;
            }
            if (out != NULL) {
                memcpy(out->evse_id, v, vlen);
                out->evse_id[vlen] = '\0';
                out->evse_id_len = vlen;
                out->evse_id_set = true;
            }
            break;

        case CBPLC_PROV_PROTOCOL_MASK:
            /* A zero mask would mean "accept no protocol", i.e. a charger that answers no
             * vehicle. That is never what a caller meant, and a corrupt byte must not be able
             * to express it. Bits outside the known set are refused rather than masked off, so
             * a newer controller learns its field was not understood. */
            if (vlen != 1u || v[0] == 0u || (v[0] & (uint8_t)~CBPLC_PROTO_ALL) != 0u) {
                return CBPLC_E_BAD_ARGUMENT;
            }
            if (out != NULL) {
                out->protocol_mask = v[0];
                out->protocol_mask_set = true;
            }
            break;

        case CBPLC_PROV_ATTN_TRIM_DB: {
            int8_t trim;
            if (vlen != 1u) {
                return CBPLC_E_BAD_ARGUMENT;
            }
            trim = (int8_t)v[0];
            if (trim < CBPLC_ATTN_TRIM_MIN || trim > CBPLC_ATTN_TRIM_MAX) {
                return CBPLC_E_BAD_ARGUMENT;
            }
            if (out != NULL) {
                out->attn_trim_db = trim;
                out->attn_trim_set = true;
            }
            break;
        }

        case CBPLC_PROV_SESSION_TIMEOUT: {
            uint16_t ms;
            if (vlen != 2u) {
                return CBPLC_E_BAD_ARGUMENT;
            }
            ms = (uint16_t)((uint16_t)v[0] | ((uint16_t)v[1] << 8));
            if (ms > CBPLC_SESSION_PARAM_TIMEOUT_MAX_MS) {
                return CBPLC_E_BAD_ARGUMENT;
            }
            if (out != NULL) {
                out->session_timeout_ms = ms;
                out->session_timeout_set = true;
            }
            break;
        }

        default:
            /* UNKNOWN TAGS ARE SKIPPED, NOT REFUSED. This is the forward-compatibility half:
             * a controller newer than the firmware may declare a field this build has never
             * heard of, and refusing the whole frame would mean the fields it DOES understand
             * never take effect. The length prefix is what makes skipping safe. */
            break;
        }
        i = (uint16_t)(i + 2u + vlen);
    }
    return CBPLC_E_OK;
}

cbplc_err_t cbplc_provision_apply(cbplc_provision_t *p, const uint8_t *payload, uint16_t len)
{
    cbplc_err_t rc;

    if (p == NULL || (payload == NULL && len != 0u)) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* Validate against a scratch copy so a rejection cannot have touched the live state. */
    rc = walk(payload, len, NULL);
    if (rc != CBPLC_E_OK) {
        p->rejected++;
        return rc;
    }
    (void)walk(payload, len, p);
    p->applied++;
    return CBPLC_E_OK;
}

static bool put(uint8_t *out, size_t cap, size_t *n, uint8_t tag, const uint8_t *v, uint8_t vlen)
{
    if (*n + 2u + (size_t)vlen > cap) {
        return false;
    }
    out[*n] = tag;
    out[*n + 1u] = vlen;
    if (vlen != 0u) {
        memcpy(&out[*n + 2u], v, vlen);
    }
    *n += 2u + (size_t)vlen;
    return true;
}

size_t cbplc_provision_encode(const cbplc_provision_t *p, uint8_t *out, size_t cap)
{
    size_t n = 0u;

    if (p == NULL || out == NULL) {
        return 0u;
    }
    if (p->evse_id_set) {
        if (!evse_id_plausible((const uint8_t *)p->evse_id, p->evse_id_len) ||
            !put(out, cap, &n, (uint8_t)CBPLC_PROV_EVSE_ID, (const uint8_t *)p->evse_id,
                 p->evse_id_len)) {
            return 0u;
        }
    }
    if (p->protocol_mask_set) {
        const uint8_t m = p->protocol_mask;
        if (m == 0u || (m & (uint8_t)~CBPLC_PROTO_ALL) != 0u ||
            !put(out, cap, &n, (uint8_t)CBPLC_PROV_PROTOCOL_MASK, &m, 1u)) {
            return 0u;
        }
    }
    if (p->attn_trim_set) {
        const uint8_t t = (uint8_t)p->attn_trim_db;
        if (p->attn_trim_db < CBPLC_ATTN_TRIM_MIN || p->attn_trim_db > CBPLC_ATTN_TRIM_MAX ||
            !put(out, cap, &n, (uint8_t)CBPLC_PROV_ATTN_TRIM_DB, &t, 1u)) {
            return 0u;
        }
    }
    if (p->session_timeout_set) {
        uint8_t b[2];
        if (p->session_timeout_ms > CBPLC_SESSION_PARAM_TIMEOUT_MAX_MS) {
            return 0u;
        }
        b[0] = (uint8_t)(p->session_timeout_ms & 0xFFu);
        b[1] = (uint8_t)((p->session_timeout_ms >> 8) & 0xFFu);
        if (!put(out, cap, &n, (uint8_t)CBPLC_PROV_SESSION_TIMEOUT, b, 2u)) {
            return 0u;
        }
    }
    return n;
}

bool cbplc_provision_protocol_allowed(const cbplc_provision_t *p, uint8_t proto_bit)
{
    /* INERT UNTIL DECLARED. A board nobody has provisioned must accept everything it accepted
     * before this existed, or rolling the firmware out would quietly refuse vehicles. */
    if (p == NULL || !p->protocol_mask_set) {
        return true;
    }
    return (p->protocol_mask & proto_bit) != 0u;
}
