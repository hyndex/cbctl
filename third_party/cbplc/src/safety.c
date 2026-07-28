#include "cbplc/safety.h"

#include <string.h>


/* Input order is the REPORTING order too: most urgent first, so the reason string
 * names the worst active cause rather than whichever bit happened to be lowest. */
enum {
    IDX_ESTOP = 0,
    IDX_EARTH,
    IDX_INSULATION,
    IDX_FUSE_SPD,
    IDX_AC_VOLTAGE,
};

static const char *const kInputNames[CBPLC_SAFETY_INPUT_COUNT] = {
    "estop", "earth", "insulation", "fuse_spd", "ac_voltage",
};

/* Wrap-safe: the 32-bit ms clock wraps every 49.7 days and unsigned subtraction is
 * correct across it. A signed comparison would make every input decide it had been
 * stable forever at the wrap. */
static bool held_for(uint32_t now_ms, uint32_t since_ms, uint32_t ms)
{
    return (uint32_t)(now_ms - since_ms) >= ms;
}

void cbplc_safety_init(cbplc_safety_t *s, uint32_t now_ms)
{
    if (s == NULL) {
        return;
    }
    memset(s, 0, sizeof *s);
    for (size_t i = 0; i < CBPLC_SAFETY_INPUT_COUNT; ++i) {
        /* ASSERTED, not clear. "Nothing sampled yet" must never permit energy. */
        s->in[i].sampled_active = true;
        s->in[i].active = true;
        s->in[i].last_change_ms = now_ms;
    }
}

/* One debouncer step. Symmetric in time and asymmetric in consequence: the caller
 * reads `sampled_active` to de-energise immediately and `active` to pass judgement. */
static void observe(cbplc_safety_input_t *d, bool sampled, uint32_t now_ms)
{
    if (sampled != d->sampled_active) {
        d->sampled_active = sampled;
        d->last_change_ms = now_ms;
    }
    if (d->active != d->sampled_active &&
        held_for(now_ms, d->last_change_ms, CBPLC_SAFETY_DEBOUNCE_MS)) {
        d->active = d->sampled_active;
    }
}

bool cbplc_safety_update(cbplc_safety_t *s, const cbplc_safety_inputs_t *in,
                         uint32_t now_ms)
{
    if (s == NULL) {
        return true;
    }
    if (in == NULL) {
        /* The read FAILED, so the chain's state is unknown. Unknown asserts
         * immediately and WITHOUT the debounce window, and it also destroys any
         * standing inactive evidence: recovery has to earn a fresh full debounce.
         * A dead I2C bus must not read as a healthy safety chain. */
        for (size_t i = 0; i < CBPLC_SAFETY_INPUT_COUNT; ++i) {
            if (!s->in[i].active || !s->in[i].sampled_active) {
                s->in[i].last_change_ms = now_ms;
            }
            s->in[i].sampled_active = true;
            s->in[i].active = true;
        }
        s->ever_faulted = true;
        /* The proof is voided as well. Evidence gathered before the bus died says
         * nothing about the chain now, and inheriting it would let a fault latch on a
         * chain that has not been re-verified. */
        s->proven_mask = 0u;
        return true;
    }

    const bool sampled[CBPLC_SAFETY_INPUT_COUNT] = {
        in->emergency_stop, in->earth_fault, in->insulation,
        in->fuse_or_spd, in->ac_voltage,
    };
    bool confirmed = false;
    for (size_t i = 0; i < CBPLC_SAFETY_INPUT_COUNT; ++i) {
        observe(&s->in[i], sampled[i], now_ms);
        if (s->in[i].active) {
            confirmed = true;
        } else {
            /* Believed inactive: this input has now been proven at least once. */
            s->proven_mask |= (uint8_t)(1u << i);
        }
    }
    if (confirmed) {
        if (!s->ever_faulted) {
            s->ever_faulted = true;
        }
        s->confirmations++;
    }
    return confirmed;
}

bool cbplc_safety_faulted(const cbplc_safety_t *s)
{
    if (s == NULL) {
        return true;
    }
    for (size_t i = 0; i < CBPLC_SAFETY_INPUT_COUNT; ++i) {
        if (s->in[i].active) {
            return true;
        }
    }
    return false;
}

bool cbplc_safety_proven(const cbplc_safety_t *s)
{
    if (s == NULL) {
        return false;
    }
    const uint8_t all = (uint8_t)((1u << CBPLC_SAFETY_INPUT_COUNT) - 1u);
    return (s->proven_mask & all) == all;
}

bool cbplc_safety_raw_asserted(const cbplc_safety_t *s)
{
    if (s == NULL) {
        return true;
    }
    for (size_t i = 0; i < CBPLC_SAFETY_INPUT_COUNT; ++i) {
        if (s->in[i].sampled_active) {
            return true;
        }
    }
    return false;
}

const char *cbplc_safety_reason(const cbplc_safety_t *s)
{
    if (s == NULL) {
        return "no_sample";
    }
    /* Reports the CONFIRMED cause, matching cbplc_safety_faulted(). A raw-only
     * assertion is deliberately not named here: it has not been believed yet, and
     * naming it would put unconfirmed causes in operator-visible logs. */
    for (size_t i = 0; i < CBPLC_SAFETY_INPUT_COUNT; ++i) {
        if (s->in[i].active) {
            return kInputNames[i];
        }
    }
    return "";
}

cbplc_err_t cbplc_relay_may_close(const cbplc_relay_t *r,
                                  uint8_t mask,
                                  const cbplc_close_preconditions_t *pre)
{
    if (r == NULL || pre == NULL || mask == 0u || (mask & ~CBPLC_RELAY_ALL) != 0u) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* A drifted expander means its state is UNKNOWN. Refuse every close until a
     * verified write proves otherwise; guessing is what makes drift dangerous. */
    if (r->drifted) {
        return CBPLC_E_NOT_READY;
    }
    if (!pre->safety_clear) {
        return CBPLC_E_NOT_READY;
    }
    /* THE unconditional rule. Closing the gun contactor onto a connector with no
     * vehicle is the one mistake with no safe outcome, so this is checked in every
     * mode with no exception path. plc_firmware had a mode flag that skipped it. */
    if ((mask & CBPLC_RELAY_GUN) != 0u && !pre->vehicle_present) {
        return CBPLC_E_NOT_READY;
    }
    /* Energy-bearing relays additionally need the full controller contract. All
     * three are required together: a live heartbeat proves the controller exists,
     * authorization proves it consents, and hlc_permits proves the vehicle
     * negotiation has not asked to stop. */
    if ((mask & (CBPLC_RELAY_GUN | CBPLC_RELAY_POWER)) != 0u) {
        if (!pre->controller_alive || !pre->authorized || !pre->hlc_permits) {
            return CBPLC_E_NOT_READY;
        }
    }
    return CBPLC_E_OK;
}

uint32_t cbplc_relay_bound_hold(uint32_t requested_ms, bool *clamped)
{
    if (clamped != NULL) {
        *clamped = requested_ms > CBPLC_RELAY_HOLD_MAX_MS;
    }
    return requested_ms > CBPLC_RELAY_HOLD_MAX_MS ? CBPLC_RELAY_HOLD_MAX_MS
                                                  : requested_ms;
}

bool cbplc_relay_note_write(cbplc_relay_t *r,
                            uint8_t commanded,
                            uint8_t readback,
                            uint32_t now_ms,
                            uint32_t hold_ms)
{
    if (r == NULL) {
        return false;
    }
    commanded &= CBPLC_RELAY_ALL;
    readback &= CBPLC_RELAY_ALL;

    if (commanded != readback) {
        /* The expander did not do what it was told, or cannot be read. Either way
         * its state is unknown, so collapse the belief to all-open and latch the
         * drift. NOT retried here: a caller that retries in a loop against a dead
         * bus just delays the safe outcome. */
        r->drifted = true;
        r->drift_events++;
        r->closed = 0u;
        r->verified_open = 0u;
        memset(r->deadline_ms, 0, sizeof r->deadline_ms);
        return false;
    }

    r->drifted = false;
    /* The PREVIOUS closed mask, captured before it is overwritten: the deadline loop below
     * needs to know which bits were ALREADY closed to tell "preserving this relay" from
     * "closing it fresh". */
    const uint8_t was_closed = r->closed;
    r->closed = commanded;
    /* Only bits proven low by readback count as verified open. This is the only
     * evidence of an open contactor this hardware offers — there is no contactor
     * feedback line — so it must come from a read, never from having commanded it. */
    r->verified_open = (uint8_t)(CBPLC_RELAY_ALL & ~readback);

    for (unsigned bit = 0; bit < 3u; ++bit) {
        const uint8_t m = (uint8_t)(1u << bit);
        if ((commanded & m) == 0u) {
            /* Not commanded closed: no dead-man to run. */
            r->deadline_ms[bit] = 0u;
            continue;
        }
        /* ONE WRITE MUST NOT REARM OR DISARM A RELAY IT DID NOT MEAN TO TOUCH.
         *
         * hold_ms applies to the whole mask, and hold_ms == 0 means "no expiry". Applying
         * that to every commanded bit meant a write that merely PRESERVED an already-closed
         * relay silently changed its dead-man:
         *
         *   main.cpp's expiry sweep calls apply_relays(closed & ~must_open, 0) — so the
         *   first lease to lapse rewrote every OTHER still-closed relay's deadline to
         *   "never". One relay timing out disarmed the dead-man on the rest, which is the
         *   exact opposite of what a dead-man is for, and it is the gun contactor that
         *   would have been left armed-forever.
         *
         *   And forward: renewing relay 1's lease rewrote relay 3's deadline to relay 1's.
         *
         * So a hold of 0 on a bit that is ALREADY closed leaves its deadline alone — the
         * caller is preserving that relay, not re-licensing it. An explicit
         * no-expiry close is still available: command a bit that is currently open. */
        if (hold_ms > 0u) {
            r->deadline_ms[bit] = now_ms + hold_ms;
            if (r->deadline_ms[bit] == 0u) {
                r->deadline_ms[bit] = 1u;   /* 0 is the "no expiry" sentinel */
            }
        } else if ((was_closed & m) == 0u) {
            r->deadline_ms[bit] = 0u;       /* newly closed with no expiry: as asked */
        }
        /* else: already closed and hold_ms == 0 — preserve, do not disturb. */
    }
    return true;
}

uint8_t cbplc_relay_expired(const cbplc_relay_t *r, uint32_t now_ms)
{
    if (r == NULL) {
        return CBPLC_RELAY_ALL;
    }
    /* Unknown expander state means open everything, every poll, until proven. */
    if (r->drifted) {
        return CBPLC_RELAY_ALL;
    }
    uint8_t open_now = 0u;
    for (unsigned bit = 0; bit < 3u; ++bit) {
        const uint8_t m = (uint8_t)(1u << bit);
        if ((r->closed & m) == 0u || r->deadline_ms[bit] == 0u) {
            continue;
        }
        /* Wrap-safe: a 32-bit ms clock wraps every 49.7 days and an EVSE runs for
         * months. Never compare absolute stamps. */
        if ((uint32_t)(now_ms - r->deadline_ms[bit]) < 0x80000000u) {
            open_now |= m;
        }
    }
    return open_now;
}

void cbplc_relay_note_all_open(cbplc_relay_t *r)
{
    if (r == NULL) {
        return;
    }
    r->closed = 0u;
    r->verified_open = CBPLC_RELAY_ALL;
    r->drifted = false;
    memset(r->deadline_ms, 0, sizeof r->deadline_ms);
}
