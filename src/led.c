/* What the connector should be showing a driver, derived once from one observation.
 *
 * THE FAILURE THIS PREVENTS
 *   The OCPP status, the session state, the charge state and the LED preset are four parallel
 *   state machines. Updated in four places they drift, and a driver ends up standing in front
 *   of a connector whose light disagrees with what the charger is doing. Deriving the preset
 *   here — from the same snapshot cbctl_flag() reads, in one ordered decision — keeps every
 *   controller-side surface on one answer.
 *
 * THE CABINET STRIP IS THE BOARD'S, AND THIS RANKING MUST MATCH IT
 *   Firmware from 2026-08-14 renders its own WS2812B strip from the board's local state
 *   (cbplc main.cpp led_preset()), so this function is no longer the only derivation — it is
 *   the controller-side twin of the one driving the physical light. The two rank the same
 *   evidence in the same order: safety-bus-unreadable before the e-stop bit (the fail-safe
 *   fill asserts every bit, including e-stop, and pessimism about the contactors is not
 *   evidence a human pressed the button), only the e-stop bit reaches EMERGENCY, protection
 *   trips and expander drift are FAULTED, and the board's session= machine names CHARGING /
 *   FINISHING / FAULTED. Any divergence between this file and that one is a bug in one of
 *   them. The one honest difference is staleness: the board is never stale to itself, while
 *   this picture can be — those rules exist only here, by design.
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
     *    tell a person to step back regardless of what the session was doing.
     *
     *    On raw-publishing firmware the chain arrives as pins, and the ranking is the
     *    board's own, in the board's own order: an unreadable bus FIRST, and FAULTED — the
     *    fail-safe fill behind it asserts every bit including e-stop, and pessimism about
     *    the contactors is not evidence a human pressed the button. Then the e-stop bit
     *    (bit 0, the order safety.c pins by static assert) is the ONE thing that earns
     *    EMERGENCY; any other asserted pin is protection tripping, which is FAULTED. A
     *    technician reading "emergency" on the HMI in front of a strip blinking orange-red
     *    FAULTED was the drift this split closes.
     *
     *    Legacy firmware publishes only the classified flag, in which the e-stop is not
     *    distinguishable — that keeps EMERGENCY, because downgrading a real e-stop is the
     *    worse misread. */
    {
        uint8_t raw = 0u;
        bool bus = false;
        if (cbctl_safety_raw(link, &raw, &bus)) {
            if (bus) {
                return CBCTL_LED_FAULTED;
            }
            if ((raw & 0x01u) != 0u) {
                return CBCTL_LED_EMERGENCY;
            }
            if (raw != 0u) {
                return CBCTL_LED_FAULTED;
            }
        } else if (cbctl_flag(link, CBCTL_FLAG_SAFETY_ASSERTED)) {
            return CBCTL_LED_EMERGENCY;
        }
    }

    /* 2. A connector we have LOST is not available, whatever it was doing a moment ago. */
    if (link->state == CBCTL_LINK_QUARANTINED) {
        return CBCTL_LED_FAULTED;
    }

    /* 2b. A DRIFTED EXPANDER IS A CONNECTOR THAT CANNOT BE DRIVEN. The board's latch
     *     collapses every relay to all-open and refuses closes until a verified write
     *     agrees — indefinitely on genuinely broken hardware — and its strip blinks
     *     FAULTED for as long as it holds. Without this rank the collapse also zeroes
     *     relay_closed, so the ranking fell through to PREPARING or AVAILABLE: an
     *     invitation to plug into a connector that cannot close a contactor. */
    if (link->relay_drifted) {
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

    /* 5. THE BOARD'S OWN CHARGE MACHINE, when it is published (session=). The strip is lit
     *    from this machine, and the proxies below disagreed with it on EVERY session: the
     *    gun contactor deliberately closes for CableCheck — isolation testing needs it —
     *    tens of seconds before energy flows, and a fault is terminal for the plug-in.
     *
     *    FAULT with the vehicle still attached used to fall through to PREPARING: a driver
     *    told "busy, wait" by the HMI in front of a strip blinking FAULTED, for a charge
     *    that was never coming. */
    if (link->board_chg == (uint8_t)CBCTL_BOARD_CHG_FAULT) {
        return CBCTL_LED_FAULTED;
    }

    /* 6. Winding down. Checked before CHARGING because a stop that has been raised while
     *    current still decays is FINISHING, not CHARGING — the driver should see it ending.
     *
     *    Three doors in: the controller's own raised stop (shown one poll before the board
     *    latches it — inherent propagation, and the honest direction), the board's stop
     *    latch, and the board's machine ENTERING Stopping on its own. The last one is how a
     *    VEHICLE-initiated stop arrives — it never passes through the controller's stop
     *    latch at all, so before session= was read here, the end of every charge the car
     *    terminated was simply missed: CHARGING, then PREPARING, no FINISHING between. */
    if (link->stop_requested || cbctl_flag(link, CBCTL_FLAG_STOP_LATCHED) ||
        link->board_chg == (uint8_t)CBCTL_BOARD_CHG_STOPPING) {
        return CBCTL_LED_FINISHING;
    }

    /* 7. Actually delivering — the board's word when it gives one: Energised is entered at
     *    PowerDelivery, exactly when the strip starts its battery-fill sweep. The proxy is
     *    kept only for firmware that never publishes session=: contactor closed AND session
     *    open. Both halves are required there, and it still claims CHARGING through all of
     *    CableCheck/PreCharge — that inaccuracy is the reason the board's machine wins when
     *    present, and UNKNOWN never blocks the fallback for boards that predate the key. */
    if (link->board_chg == (uint8_t)CBCTL_BOARD_CHG_ENERGISED) {
        return CBCTL_LED_CHARGING;
    }
    if (link->board_chg == (uint8_t)CBCTL_BOARD_CHG_UNKNOWN &&
        cbctl_flag(link, CBCTL_FLAG_ENERGISED) && cbctl_flag(link, CBCTL_FLAG_SESSION_ACTIVE)) {
        return CBCTL_LED_CHARGING;
    }

    /* 8. Plugged and working towards a charge: matching, negotiating, or waiting on
     *    authorisation. Deliberately one state — a driver does not need SLAC explained, they
     *    need to know the charger has noticed them and is busy. */
    if (cbctl_flag(link, CBCTL_FLAG_VEHICLE_PRESENT)) {
        return CBCTL_LED_PREPARING;
    }

    /* 9. Nothing plugged, and we are sure of it. */
    return CBCTL_LED_AVAILABLE;
}
