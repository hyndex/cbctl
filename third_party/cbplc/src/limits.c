#include "cbplc/limits.h"

#include <string.h>

/* Age with the SIGN of the difference respected, the idiom auth.c already uses.
 *
 * The first version reasoned "unsigned subtraction is correct across the 49.7-day
 * rollover" — true for stamps in the past, and catastrophically wrong for stamps a few
 * milliseconds in the FUTURE. Those exist in production: `now` is captured at the top of
 * the loop pass, the serial poll then ingests a LIMITS message that stamps with live
 * millis(), and this check runs later in the same pass with the older `now`. Unsigned math
 * turns that -3 ms into ~4.29 billion ms and the freshest possible feed reads as expired.
 *
 * At idle a pass is ~1 ms and the collision window is negligible, which is why every bench
 * experiment passed. During SLAC/SECC a pass is long, the window widens to the whole pass,
 * and a 400 ms controller feed collided constantly: limits_expiry climbed ~1.5/s, the
 * charge machine churned Linking->Stopping:controller_lost on the same math in main.cpp,
 * the pilot left 5%, and the vehicle reset TCP at ChargeParameterDiscovery. One signed
 * cast, three days of symptoms.
 *
 * Signed is still rollover-safe for every real age: it misreads only at ±24.8 days, and
 * both real quantities here are bounded — ages by the stale threshold (seconds), future
 * skew by one loop pass (milliseconds). */
static bool stale(uint32_t now_ms, uint32_t since_ms)
{
    if (since_ms == 0u) {
        return true; /* never received is not fresh */
    }
    const int32_t age = (int32_t)(now_ms - since_ms);
    if (age < 0) {
        return false; /* stamped after `now` was captured: this very pass, maximally fresh */
    }
    return (uint32_t)age >= CBPLC_LIMITS_STALE_MS;
}

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

void cbplc_limits_init(cbplc_limits_t *l)
{
    if (l == NULL) {
        return;
    }
    const cbplc_limits_t zero = {0};
    *l = zero;
    /* Zero already means UNKNOWN for both enums, which is the correct starting belief:
     * nothing has been measured, and neither "isolation is fine" nor "the contactor is
     * open" may be assumed. */
}

cbplc_err_t cbplc_limits_apply_envelope(cbplc_limits_t *l, const uint8_t *payload,
                                        uint16_t len, uint32_t now_ms)
{
    if (l == NULL || payload == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    if (len != CBPLC_LIMITS_PAYLOAD_LEN) {
        l->rejects++;
        return CBPLC_E_BAD_LENGTH;
    }

    cbplc_envelope_t e;
    e.max_voltage_mv = rd32(&payload[0]);
    e.min_voltage_mv = rd32(&payload[4]);
    e.max_current_ma = rd32(&payload[8]);
    e.min_current_ma = rd32(&payload[12]);
    e.max_power_w = rd32(&payload[16]);
    const uint16_t flags = (uint16_t)((uint16_t)payload[20] |
                                     (uint16_t)((uint16_t)payload[21] << 8));
    e.power_ready = (flags & 0x0001u) != 0u;

    /* Range-checked, not trusted. A corrupted limit that is accepted becomes a number
     * the vehicle is told it may draw. */
    if (e.max_voltage_mv > CBPLC_LIMITS_MAX_VOLTAGE_MV ||
        e.min_voltage_mv > CBPLC_LIMITS_MAX_VOLTAGE_MV ||
        e.max_current_ma > CBPLC_LIMITS_MAX_CURRENT_MA ||
        e.min_current_ma > CBPLC_LIMITS_MAX_CURRENT_MA ||
        e.max_power_w > CBPLC_LIMITS_MAX_POWER_W) {
        l->rejects++;
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* An inverted envelope is not a narrow one, it is nonsense — and a vehicle handed a
     * minimum above the maximum has no legal operating point. */
    if (e.min_voltage_mv > e.max_voltage_mv || e.min_current_ma > e.max_current_ma) {
        l->rejects++;
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* Undefined flag bits are rejected so they stay reservable. */
    if ((flags & (uint16_t)~(uint16_t)0x0001u) != 0u) {
        l->rejects++;
        return CBPLC_E_BAD_RESERVED;
    }

    l->envelope = e;
    l->envelope_ms = (now_ms == 0u) ? 1u : now_ms; /* 0 is the "never" sentinel */
    l->envelope_updates++;
    return CBPLC_E_OK;
}

cbplc_err_t cbplc_limits_apply_measurement(cbplc_limits_t *l, const uint8_t *payload,
                                           uint16_t len, uint32_t now_ms)
{
    if (l == NULL || payload == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    if (len != CBPLC_FEEDBACK_PAYLOAD_LEN) {
        l->rejects++;
        return CBPLC_E_BAD_LENGTH;
    }

    cbplc_measurement_t m;
    m.present_voltage_mv = rd32(&payload[0]);
    m.present_current_ma = rd32(&payload[4]);
    const uint8_t iso = payload[8];
    const uint8_t con = payload[9];

    if (m.present_voltage_mv > CBPLC_LIMITS_MAX_VOLTAGE_MV ||
        m.present_current_ma > CBPLC_LIMITS_MAX_CURRENT_MA) {
        l->rejects++;
        return CBPLC_E_BAD_ARGUMENT;
    }
    if (iso > (uint8_t)CBPLC_ISO_FAULT || con > (uint8_t)CBPLC_CONTACTOR_CLOSED) {
        /* An unknown enumerant is refused rather than mapped to a default: silently
         * treating an unrecognised isolation verdict as UNKNOWN would let a newer
         * controller's "fault" read as "not measured yet". */
        l->rejects++;
        return CBPLC_E_BAD_ARGUMENT;
    }
    m.isolation = (cbplc_isolation_t)iso;
    m.contactor = (cbplc_contactor_t)con;

    l->measurement = m;
    l->measurement_ms = (now_ms == 0u) ? 1u : now_ms;
    l->measurement_updates++;
    return CBPLC_E_OK;
}

size_t cbplc_limits_encode_envelope(const cbplc_envelope_t *e, uint8_t *out, size_t cap)
{
    if (e == NULL || out == NULL || cap < CBPLC_LIMITS_PAYLOAD_LEN) {
        return 0u;
    }
    wr32(&out[0], e->max_voltage_mv);
    wr32(&out[4], e->min_voltage_mv);
    wr32(&out[8], e->max_current_ma);
    wr32(&out[12], e->min_current_ma);
    wr32(&out[16], e->max_power_w);
    const uint16_t flags = e->power_ready ? 0x0001u : 0x0000u;
    out[20] = (uint8_t)(flags & 0xFFu);
    out[21] = (uint8_t)((flags >> 8) & 0xFFu);
    return CBPLC_LIMITS_PAYLOAD_LEN;
}

size_t cbplc_limits_encode_measurement(const cbplc_measurement_t *m, uint8_t *out,
                                       size_t cap)
{
    if (m == NULL || out == NULL || cap < CBPLC_FEEDBACK_PAYLOAD_LEN) {
        return 0u;
    }
    wr32(&out[0], m->present_voltage_mv);
    wr32(&out[4], m->present_current_ma);
    out[8] = (uint8_t)m->isolation;
    out[9] = (uint8_t)m->contactor;
    out[10] = 0u;
    out[11] = 0u;
    return CBPLC_FEEDBACK_PAYLOAD_LEN;
}

bool cbplc_limits_fresh(const cbplc_limits_t *l, uint32_t now_ms)
{
    if (l == NULL) {
        return false;
    }
    /* BOTH. A DC response needs limits and measurements together; one without the other
     * cannot fill a CableCheckRes or a CurrentDemandRes. */
    return !stale(now_ms, l->envelope_ms) && !stale(now_ms, l->measurement_ms);
}

bool cbplc_limits_in_grace(const cbplc_limits_t *l, uint32_t now_ms)
{
    if (l == NULL) {
        return false;
    }
    /* Never received is not "briefly busy" — there is nothing to be graceful about. */
    if (l->envelope_ms == 0u || l->measurement_ms == 0u) {
        return false;
    }
    /* Same signed-age rule as stale(): a same-pass stamp is age 0, not 49.7 days. */
    const int32_t env_d = (int32_t)(now_ms - l->envelope_ms);
    const int32_t meas_d = (int32_t)(now_ms - l->measurement_ms);
    const uint32_t env_age = (env_d < 0) ? 0u : (uint32_t)env_d;
    const uint32_t meas_age = (meas_d < 0) ? 0u : (uint32_t)meas_d;
    const uint32_t age = (env_age > meas_age) ? env_age : meas_age;
    return age < CBPLC_LIMITS_GRACE_MS;
}

bool cbplc_limits_tick(cbplc_limits_t *l, uint32_t now_ms)
{
    if (l == NULL) {
        return false;
    }
    const bool fresh = cbplc_limits_fresh(l, now_ms);
    if (l->was_fresh && !fresh) {
        /* Counted on the EDGE, not per tick: this is "the controller stopped talking N
         * times", which is a diagnosis, rather than "it has been quiet for N ticks",
         * which is just the loop rate. */
        l->expiries++;
    }
    l->was_fresh = fresh;
    return fresh;
}

bool cbplc_limits_permit_energy(const cbplc_limits_t *l, uint32_t now_ms)
{
    if (!cbplc_limits_fresh(l, now_ms)) {
        return false;
    }
    if (!l->envelope.power_ready) {
        return false;
    }
    /* A zero envelope is not permission to draw zero, it is an absent envelope. */
    if (l->envelope.max_voltage_mv == 0u || l->envelope.max_current_ma == 0u) {
        return false;
    }
    /* Only VALID permits energy. UNKNOWN is refused because "not measured yet" is not a
     * pass, and WARNING is refused because a degrading insulation path is exactly the
     * condition not to push current through. */
    if (l->measurement.isolation != CBPLC_ISO_VALID) {
        return false;
    }
    /* An explicitly OPEN contactor while energy is being offered means the controller
     * and the PLC disagree about the power path. Refuse rather than resolve it: pushing
     * current at an open contactor is the failure this check exists for. UNKNOWN is
     * tolerated because not every install reports contactor position. */
    if (l->measurement.contactor == CBPLC_CONTACTOR_OPEN) {
        return false;
    }
    return true;
}

const char *cbplc_limits_refusal(const cbplc_limits_t *l, uint32_t now_ms)
{
    if (l == NULL) {
        return "no_state";
    }
    if (l->envelope_ms == 0u) {
        return "no_limits";
    }
    if (l->measurement_ms == 0u) {
        return "no_feedback";
    }
    if (stale(now_ms, l->envelope_ms)) {
        return "limits_stale";
    }
    if (stale(now_ms, l->measurement_ms)) {
        return "feedback_stale";
    }
    if (!l->envelope.power_ready) {
        return "power_not_ready";
    }
    if (l->envelope.max_voltage_mv == 0u || l->envelope.max_current_ma == 0u) {
        return "empty_envelope";
    }
    if (l->measurement.isolation != CBPLC_ISO_VALID) {
        switch (l->measurement.isolation) {
        case CBPLC_ISO_UNKNOWN: return "isolation_unknown";
        case CBPLC_ISO_INVALID: return "isolation_invalid";
        case CBPLC_ISO_WARNING: return "isolation_warning";
        case CBPLC_ISO_FAULT: return "isolation_fault";
        default: return "isolation";
        }
    }
    if (l->measurement.contactor == CBPLC_CONTACTOR_OPEN) {
        return "contactor_open";
    }
    return "";
}
