/* What the connector should be showing a driver, derived once from one observation.
 *
 * THE FAILURE THIS PREVENTS
 *   The OCPP status, the session state, the charge state and the LED preset are four parallel
 *   state machines. Updated in four places they drift, and a driver ends up standing in front
 *   of a connector whose light disagrees with what the charger is doing. Deriving the preset
 *   here — from the same snapshot cbctl_flag() reads, in one ordered decision — makes that
 *   drift impossible instead of merely discouraged.
 *
 * THE ORDER IS THE DESIGN
 *   Each rule below is checked before the ones under it, and the order runs from "most
 *   dangerous to get wrong" to "least". A connector that is both faulted and plugged shows
 *   FAULTED, because a driver who is told PREPARING waits for a charge that is not coming.
 */
#include "cbctl/cbctl.h"

const char *cbctl_led_name(cbctl_led_t p)
{
    switch (p) {
    case CBCTL_LED_BOOTING:   return "booting";
    case CBCTL_LED_AVAILABLE: return "available";
    case CBCTL_LED_PREPARING: return "preparing";
    case CBCTL_LED_CHARGING:  return "charging";
    case CBCTL_LED_FINISHING: return "finishing";
    case CBCTL_LED_FAULTED:   return "faulted";
    case CBCTL_LED_EMERGENCY: return "emergency";
    default:                  return "faulted"; /* an unknown preset is not a safe one */
    }
}

cbctl_led_t cbctl_led_preset(const cbctl_link_t *link, uint32_t now_ms)
{
    uint32_t obs;

    /* No link at all. Not "available" — we cannot claim a connector is usable when we have
     * nothing to base that on. */
    if (link == NULL) {
        return CBCTL_LED_FAULTED;
    }

    /* 1. THE SAFETY CHAIN OUTRANKS EVERYTHING. It is the one condition where the light must
     *    tell a person to step back regardless of what the session was doing. */
    if (cbctl_flag(link, CBCTL_FLAG_SAFETY_ASSERTED)) {
        return CBCTL_LED_EMERGENCY;
    }

    /* 2. A connector we have LOST is not available, whatever it was doing a moment ago. */
    if (link->state == CBCTL_LINK_QUARANTINED) {
        return CBCTL_LED_FAULTED;
    }

    /* 3. Not started, or not yet named.
     *
     *    CLOSED belongs here rather than with FAULTED, and the distinction is worth stating:
     *    cbctl_open() moves a link straight to IDENTIFYING, so CLOSED only ever means "never
     *    successfully opened" — which at controller start-up is every connector, briefly.
     *    Lighting a whole charger up as FAULTED for the first second of its life would be
     *    alarming and wrong. A link that was working and then failed does not come back
     *    here; it goes to QUARANTINED above. */
    if (link->state == CBCTL_LINK_CLOSED || link->state == CBCTL_LINK_IDENTIFYING ||
        !link->identity.valid) {
        return CBCTL_LED_BOOTING;
    }

    /* 4. NEVER CLAIM CHARGING FROM A STALE PICTURE.
     *
     *    Every flag is last-known-good. If STATUS has gone quiet, the honest answer is that we
     *    do not know what this connector is doing — and of all the ways to be wrong here, the
     *    expensive one is showing a driver CHARGING while nothing is happening: they walk away
     *    and come back to an empty battery. Showing FAULTED when the charger is secretly fine
     *    costs an unnecessary unplug. That asymmetry decides this rule.
     *
     *    The threshold is the same one that puts the link into DEGRADED, so the light and the
     *    link state cannot disagree about whether our picture is trustworthy. */
    obs = cbctl_observation_age_ms(link, now_ms);
    if (obs == CBCTL_NEVER_OBSERVED) {
        return CBCTL_LED_BOOTING;
    }
    if (obs >= CBCTL_OBSERVATION_STALE_MS) {
        return CBCTL_LED_FAULTED;
    }

    /* 5. Winding down. Checked before CHARGING because a stop that has been raised while
     *    current still decays is FINISHING, not CHARGING — the driver should see it ending. */
    if (link->stop_requested || cbctl_flag(link, CBCTL_FLAG_STOP_LATCHED)) {
        return CBCTL_LED_FINISHING;
    }

    /* 6. Actually delivering. Both halves are required: the board reports the contactor closed
     *    AND a session is open. Either alone is a connector that is not yet charging — a closed
     *    contactor with no session is precharge or a fault, and a session with the contactor
     *    open has not got there yet. */
    if (cbctl_flag(link, CBCTL_FLAG_ENERGISED) && cbctl_flag(link, CBCTL_FLAG_SESSION_ACTIVE)) {
        return CBCTL_LED_CHARGING;
    }

    /* 7. Plugged and working towards a charge: matching, negotiating, or waiting on
     *    authorisation. Deliberately one state — a driver does not need SLAC explained, they
     *    need to know the charger has noticed them and is busy. */
    if (cbctl_flag(link, CBCTL_FLAG_VEHICLE_PRESENT)) {
        return CBCTL_LED_PREPARING;
    }

    /* 8. Nothing plugged, and we are sure of it. */
    return CBCTL_LED_AVAILABLE;
}
