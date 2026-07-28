#include "cbplc/auth.h"

#include <stddef.h>

uint32_t cbplc_hb_timeout_from_period(uint32_t period_ms)
{
    uint32_t timeout_ms;
    if (period_ms == 0u) {
        return CBPLC_HB_TIMEOUT_DEFAULT_MS;
    }
    /* Multiply in a wider type: a controller announcing 0xFFFF units (6,553,500 ms) times
     * three overflows 32 bits, and an overflowed timeout is a SHORT one — which would tear
     * down every session on the one input most likely to be a controller bug. */
    {
        const uint64_t wide = (uint64_t)period_ms * (uint64_t)CBPLC_HB_TOLERATED_MISSES;
        timeout_ms = (wide > (uint64_t)CBPLC_HB_TIMEOUT_MAX_MS)
                         ? CBPLC_HB_TIMEOUT_MAX_MS
                         : (uint32_t)wide;
    }
    if (timeout_ms < CBPLC_HB_TIMEOUT_MIN_MS) {
        timeout_ms = CBPLC_HB_TIMEOUT_MIN_MS;
    }
    return timeout_ms;
}

void cbplc_auth_init(cbplc_auth_t *a)
{
    if (a == NULL) {
        return;
    }
    const cbplc_auth_t zero = {0};
    *a = zero;
    /* PENDING, not DENY, as the boot state.
     *
     * DENY would be terminal: the first vehicle to ask before the controller has spoken
     * would be told its authorisation FAILED and would stop asking. PENDING says "not yet",
     * which is exactly true of a charger that has just started. Neither grants energy. */
    a->state = CBPLC_AUTH_PENDING;
    a->initialised = true;
}

cbplc_err_t cbplc_auth_apply(cbplc_auth_t *a, const uint8_t *payload, uint16_t len,
                             uint32_t now_ms)
{
    uint32_t lease_ms;
    cbplc_auth_state_t state;

    if (a == NULL || payload == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* A decision arriving before init is still a real decision — adopt it, and mark the
     * struct usable, rather than discarding the controller's word. */
    a->initialised = true;

    if (len == CBPLC_AUTH_LEGACY_PAYLOAD_LEN) {
        /* Legacy: one bit, no lease. Cannot express PENDING — see the header. */
        if (payload[0] > 1u) {
            a->rejects++;
            return CBPLC_E_BAD_ARGUMENT;
        }
        state = (payload[0] != 0u) ? CBPLC_AUTH_GRANTED : CBPLC_AUTH_DENY;
        lease_ms = CBPLC_AUTH_LEASE_DEFAULT_MS;
        a->legacy++;
    } else if (len == CBPLC_AUTH_PAYLOAD_LEN) {
        if (payload[0] > (uint8_t)CBPLC_AUTH_GRANTED) {
            /* An unknown decision is REFUSED, not mapped to a default. A newer controller's
             * fourth state must not read as one of ours — silently treating it as DENY would
             * end sessions, and as GRANTED would energise on a value we do not understand. */
            a->rejects++;
            return CBPLC_E_BAD_ARGUMENT;
        }
        state = (cbplc_auth_state_t)payload[0];
        lease_ms = ((uint32_t)payload[1] | ((uint32_t)payload[2] << 8)) * 100u;
        if (lease_ms == 0u) {
            lease_ms = CBPLC_AUTH_LEASE_DEFAULT_MS;
            a->clamped++;
        } else if (lease_ms < CBPLC_AUTH_LEASE_MIN_MS) {
            lease_ms = CBPLC_AUTH_LEASE_MIN_MS;
            a->clamped++;
        } else if (lease_ms > CBPLC_AUTH_LEASE_MAX_MS) {
            lease_ms = CBPLC_AUTH_LEASE_MAX_MS;
            a->clamped++;
        }
    } else {
        a->rejects++;
        return CBPLC_E_BAD_LENGTH;
    }

    /* EXPIRY FIRST, STATE SECOND, and the order is load-bearing.
     *
     * cbplc_auth_state() is a reader that MUTATES on lapse (it demotes a lapsed GRANTED to
     * PENDING) and it is called off the loop task by the SECC layer. With `state` stored
     * first, a reader landing between the two stores saw GRANTED paired with the PREVIOUS
     * decision's expiry — usually already past — and demoted the brand-new grant to
     * PENDING. The grant was simply lost until the next AUTHORIZE arrived, which on a 400 ms
     * feed is a 400 ms hole in authorisation at exactly the moment a session is starting.
     *
     * Writing the expiry first means the worst a reader can observe is the OLD state with
     * the NEW expiry, which is a slightly generous lease on a decision that is about to be
     * replaced anyway — the safe direction. This is not a substitute for a lock; it removes
     * the need for one, on a struct whose fields are single words. */
    uint32_t new_expiry;
    if (state == CBPLC_AUTH_GRANTED) {
        new_expiry = now_ms + lease_ms;
    } else if (state == CBPLC_AUTH_DENY) {
        /* A refusal expires too — see CBPLC_AUTH_DENY_TTL_MS. It decays to PENDING, not to
         * GRANTED: forgetting a refusal must never become permission. */
        new_expiry = now_ms + CBPLC_AUTH_DENY_TTL_MS;
    } else {
        new_expiry = 0u;
    }
    /* now_ms + lease can land on 0 across the millisecond rollover, and 0 is the sentinel
     * for "no lease". One millisecond of error is not worth a second field. */
    if (new_expiry == 0u && state != CBPLC_AUTH_PENDING) {
        new_expiry = 1u;
    }
    a->expiry_ms = new_expiry;
    a->state = state;
    if (state == CBPLC_AUTH_GRANTED) {
        a->grants++;
    } else if (state == CBPLC_AUTH_DENY) {
        a->denials++;
    } else {
        a->pendings++;
    }
    return CBPLC_E_OK;
}

size_t cbplc_auth_encode(cbplc_auth_state_t state, uint32_t lease_ms, uint8_t *out,
                         size_t cap)
{
    uint32_t units;
    if (out == NULL || cap < CBPLC_AUTH_PAYLOAD_LEN) {
        return 0u;
    }
    units = lease_ms / 100u;
    if (units > 0xFFFFu) {
        units = 0xFFFFu;
    }
    out[0] = (uint8_t)state;
    out[1] = (uint8_t)(units & 0xFFu);
    out[2] = (uint8_t)((units >> 8) & 0xFFu);
    return CBPLC_AUTH_PAYLOAD_LEN;
}

cbplc_auth_state_t cbplc_auth_state(cbplc_auth_t *a, uint32_t now_ms)
{
    if (a == NULL || !a->initialised) {
        /* Never initialised: PENDING, never the zero-valued DENY. See the field's comment. */
        return CBPLC_AUTH_PENDING;
    }
    if (a->state == CBPLC_AUTH_PENDING) {
        return CBPLC_AUTH_PENDING;
    }
    if (a->expiry_ms == 0u || (int32_t)(now_ms - a->expiry_ms) > 0) {
        /* A LAPSED LEASE IS PENDING, NOT DENY.
         *
         * The distinction is the whole reason this module exists. A lapsed lease means the
         * controller stopped talking; a DENY means it said no. Reporting a lapse as DENY
         * would tell the vehicle its authorisation FAILED — terminal, so it stops asking —
         * on the strength of a controller that is merely busy or restarting. PENDING keeps
         * the session alive and lets the next grant resume it, while still refusing energy
         * for exactly as long as the silence lasts. */
        a->state = CBPLC_AUTH_PENDING;
        a->expiry_ms = 0u;
        a->lease_expiries++;
        return CBPLC_AUTH_PENDING;
    }
    return a->state;
}

const char *cbplc_auth_name(cbplc_auth_state_t s)
{
    switch (s) {
    case CBPLC_AUTH_DENY: return "deny";
    case CBPLC_AUTH_PENDING: return "pending";
    case CBPLC_AUTH_GRANTED: return "granted";
    }
    return "?";
}

bool cbplc_hb_alive(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms)
{
    if (last_ms == 0u) {
        return false;
    }
    const int32_t age = (int32_t)(now_ms - last_ms);
    if (age < 0) {
        return true; /* stamped after `now` was captured: this very pass, maximally alive */
    }
    return (uint32_t)age < timeout_ms;
}
