#include "cbplc/safety.h"

#include <string.h>


/* Bit order of raw_mask, which is also the controller-side reporting order:
 * most urgent first. The controller (cbcontroller-v2 supervisor) owns the
 * names, the debounce and every policy built on these bits — this file only
 * promises the ORDER stays stable, because the wire carries positions. */
_Static_assert(CBPLC_SAFETY_INPUT_COUNT == 5,
               "adding a safety input means updating the mask fill in "
               "cbplc_safety_update(), cbplc_safety_inputs_t, and the "
               "controller-side classifier that names each bit");

void cbplc_safety_init(cbplc_safety_t *s, uint32_t now_ms)
{
    (void)now_ms;
    if (s == NULL) {
        return;
    }
    /* ASSERTED, not clear. "Nothing sampled yet" must never permit energy —
     * the controller, which owns all fault policy now, sees the fail-safe fill
     * until the first clean sample replaces it. */
    s->raw_mask = (uint8_t)((1u << CBPLC_SAFETY_INPUT_COUNT) - 1u);
    s->bus_unreadable = true;
}

bool cbplc_safety_update(cbplc_safety_t *s, const cbplc_safety_inputs_t *in,
                         uint32_t now_ms)
{
    (void)now_ms;
    if (s == NULL) {
        return true;
    }
    if (in == NULL) {
        /* The read FAILED. Publish the fail-safe fill and say so: a dead I2C
         * bus must never read as a healthy chain, and naming it lets the
         * controller report "bus" instead of blaming the first input. */
        s->raw_mask = (uint8_t)((1u << CBPLC_SAFETY_INPUT_COUNT) - 1u);
        s->bus_unreadable = true;
        return true;
    }
    /* Bit order is the reporting order the controller's classifier mirrors:
     * estop, earth, insulation, fuse_spd, ac_voltage. */
    uint8_t m = 0u;
    if (in->emergency_stop) { m |= 1u << 0; }
    if (in->earth_fault)    { m |= 1u << 1; }
    if (in->insulation)     { m |= 1u << 2; }
    if (in->fuse_or_spd)    { m |= 1u << 3; }
    if (in->ac_voltage)     { m |= 1u << 4; }
    s->raw_mask = m;
    s->bus_unreadable = false;
    return m != 0u;
}

uint8_t cbplc_safety_raw_mask(const cbplc_safety_t *s)
{
    if (s == NULL) {
        return (uint8_t)((1u << CBPLC_SAFETY_INPUT_COUNT) - 1u);
    }
    return s->raw_mask;
}

bool cbplc_safety_bus_unreadable(const cbplc_safety_t *s)
{
    return s == NULL || s->bus_unreadable;
}

cbplc_err_t cbplc_relay_may_close(const cbplc_relay_t *r,
                                  uint8_t mask,
                                  const cbplc_close_preconditions_t *pre)
{
    if (r == NULL || pre == NULL || mask == 0u || (mask & ~CBPLC_RELAY_ALL) != 0u) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* No fault terms here, by decision (2026-08-14): the safety chain and the
     * drift latch are FAULTS, and all fault policy — including whether either
     * refuses a close — belongs to the controller. What remains below is the
     * session/interlock contract, which is not fault handling: a vehicle on CP
     * before the gun contactor, a live controller, and a driver's
     * authorisation. The dead-man lease bounds a dead controller. */
    /* THE unconditional rule. Closing the gun contactor onto a connector with no
     * vehicle is the one mistake with no safe outcome, so this is checked in every
     * mode with no exception path. plc_firmware had a mode flag that skipped it. */
    if ((mask & CBPLC_RELAY_GUN) != 0u && !pre->vehicle_present) {
        return CBPLC_E_NOT_READY;
    }
    /* THE CONTROLLER CONTRACT, ON EVERY RELAY — with authorisation scoped to the one
     * relay a driver is involved in. A live heartbeat proves the controller exists and
     * hlc_permits proves no stop is in force; both are required for every close,
     * because there is no relay on this board whose close is none of the controller's
     * business.
     *
     * This clause used to read `(mask & (GUN|POWER)) != 0`, which left relay 2 — the tie
     * that borrows the OTHER gun's live power modules onto this gun's DC bus — closable
     * with a dead controller, no authorisation and a stop in force. Meanwhile main.cpp's
     * open paths treated relay 2 as energy-bearing and dropped it on an unplug. One gate
     * said relay 2 carries energy, the other said it does not, and the disagreement was
     * only ever going to be resolved in the permissive direction, at a connector. */
    if (!pre->controller_alive || !pre->hlc_permits) {
        return CBPLC_E_NOT_READY;
    }
    /* AUTHORISATION IS A STATEMENT ABOUT A DRIVER AT THIS GUN, so it gates the gun
     * contactor and nothing else. Requiring it on every relay closed the previous hole
     * by opening a new one: relay 2's close arrives on the DONOR's board, and a donor
     * is BY CONSTRUCTION an idle, vehicle-free, unauthorised connector — its bank is
     * being lent precisely because nobody is using it. With the blanket clause the
     * cross-tie close was NACKed on every attempt, the controller's borrow machine
     * timed out every attach, and cross-PLC power sharing could not complete at all —
     * silently, at one journal line per 18 s retry cycle. Relay 3 (shared AC plant) is
     * likewise a plant operation with no driver: the controller closes it on physical
     * presence, BEFORE any authorisation decision exists. The lend and the plant stay
     * supervised through the terms above plus the dead-man lease; what they do not
     * need is a driver's consent on a connector that has no driver. */
    if ((mask & CBPLC_RELAY_GUN) != 0u && !pre->authorized) {
        return CBPLC_E_NOT_READY;
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
        r->drift_readback = readback;
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
