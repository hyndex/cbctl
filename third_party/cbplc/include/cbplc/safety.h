/* Relay intent and the safety chain. Pure policy, host-testable, no hardware.
 *
 * This is the module that decides whether a contactor may be closed. Everything in
 * it is deliberately conservative: every unknown answers "no".
 *
 * WHY VERIFIED WRITES, NOT FIRE-AND-FORGET
 *   The relays hang off an I2C port expander. plc_firmware learned the hard way
 *   that an expander can lose its output register (brownout, bus glitch, a held
 *   SDA) while still ACKing — and that a commanded-but-not-actual open leaves a
 *   contactor energised with nothing downstream able to notice, because this
 *   hardware has NO contactor feedback line. So a write is not complete until it is
 *   read back and matches.
 *
 *   A readback that disagrees is not a retry-and-hope situation: it means the
 *   expander's state is unknown, and unknown must collapse to all-open.
 *
 * WHY A DEAD-MAN LEASE
 *   A closed relay with no expiry survives the death of whatever decided to close
 *   it. Every close carries a deadline; if it is not refreshed the relay opens by
 *   itself. That is what makes a crashed controller safe rather than merely
 *   detectable, and it is why hold_ms is a REQUIRED part of a close, not an option.
 *
 * WHERE THE POLICY WENT (2026-08-14)
 *   Every fault POLICY that used to be documented here — the raw/debounced
 *   split, cause classification, session latching, proven-clear tracking and
 *   the close refusals built on them — lives in the controller now
 *   (cbcontroller-v2 src/safety/supervisor.cpp), which reads safety_in /
 *   safety_bus from STATUS and commands every fault response through the
 *   normal relay path. The dead-man lease stays: it is a liveness contract,
 *   not fault handling.
 *
 * WHY EVERY INPUT STARTS ASSERTED
 *   Carried over from plc_firmware's FailSafeInputDebouncer: sampled AND published
 *   state begin ACTIVE, and a verified stable inactive run is the only way to clear.
 *   A charger that has not yet PROVEN its safety chain is clear must not be able to
 *   close a contactor, and "no evidence yet" is not evidence of safety. This is why
 *   a zeroed cbplc_safety_t is not usable — use cbplc_safety_init().
 *
 * WHAT THIS MUST NOT DO
 *   No I2C, no GPIO, no logging, no clock of its own. The caller supplies time and
 *   performs the writes. That is what lets every rule here be tested on a host with
 *   no board attached, which is the single most valuable pattern carried over from
 *   plc_firmware.
 */
#ifndef CBPLC_SAFETY_H
#define CBPLC_SAFETY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bit positions, and they are a contract: the controller sends these masks on the
 * wire, so renumbering them silently changes which contactor moves.
 *
 * WHAT EACH BIT PHYSICALLY IS. Spelled out because two gates in this file disagreed
 * about it and neither said which relay it meant:
 *   relay 1  this gun's DC contactor
 *   relay 2  the tie that BORROWS the other gun's power modules onto this gun's DC bus
 *   relay 3  the AC contactors feeding the module bank */
#define CBPLC_RELAY_GUN   0x01u /* relay 1 — DC gun contactor, the dangerous one */
#define CBPLC_RELAY_AUX   0x02u /* relay 2 — module-borrow tie from the other gun */
#define CBPLC_RELAY_POWER 0x04u /* relay 3 — AC feed to the modules, shared plant */
#define CBPLC_RELAY_ALL   0x07u

/* THE ENERGY-BEARING SET, DEFINED ONCE.
 *
 * Relay 1 and relay 2 both put DC on this gun's bus — 1 is the contactor at the
 * connector, 2 ties the neighbouring gun's live modules onto the same bus. Relay 3 is
 * upstream AC into a module bank that several connectors share, which is why the local
 * open paths deliberately leave it alone: dropping it because THIS vehicle unplugged
 * would take the neighbouring gun's charge down with it.
 *
 * There used to be two answers to "which relays are energy-bearing". main.cpp's open
 * paths used GUN|AUX; cbplc_relay_may_close() applied the controller contract to
 * GUN|POWER. The gap was relay 2: a close of the module-borrow tie ALONE required
 * neither a live controller, nor an authorisation, nor the vehicle negotiation's
 * consent — only a clear chain and an undrifted expander. So a stale RELAY request
 * could tie the other gun's live modules onto an unsupervised bus. Two gates
 * disagreeing about which relay carries energy is exactly how something closes that
 * should have stayed open, so there is now one definition and both consult it. */
#define CBPLC_RELAY_ENERGY (CBPLC_RELAY_GUN | CBPLC_RELAY_AUX)

/* Dead-man ceiling. A LONGER value is strictly less safe, so this is the longest
 * we accept and a request beyond it is clamped AND reported — never silently
 * shortened, which is how two ends end up disagreeing about when a relay opens.
 * Sized to outlast a supervising controller's restart, matching the 30 s lease
 * plc_firmware settled on for exactly that reason. */
#define CBPLC_RELAY_HOLD_MAX_MS 30000u

/* The external safety chain. Every one is active-low in hardware and normalised to
 * "true = asserted = bad" here, so no reader has to remember the polarity. */
typedef struct {
    bool emergency_stop;
    bool earth_fault;
    bool fuse_or_spd;
    bool ac_voltage;
    bool insulation;
} cbplc_safety_inputs_t;

#define CBPLC_SAFETY_INPUT_COUNT 5

/* RAW STATE ONLY — see WHERE THE POLICY WENT above. Bit i of raw_mask follows
 * the order safety.c's _Static_assert documents: estop, earth, insulation,
 * fuse_spd, ac_voltage. bus_unreadable means the last sample FAILED and
 * raw_mask holds the fail-safe all-asserted fill — a dead I2C bus must never
 * read as a healthy chain, and the controller names that cause "bus". */
typedef struct {
    uint8_t raw_mask;
    bool bus_unreadable;
} cbplc_safety_t;

/* Start with the fail-safe fill: all inputs asserted, bus unproven. */
void cbplc_safety_init(cbplc_safety_t *s, uint32_t now_ms);

/* Feed one sample. `in` NULL means the read FAILED: raw_mask takes the
 * all-asserted fill and bus_unreadable is set until a clean sample lands.
 * Returns true when any bit is asserted, for callers that only branch. */
bool cbplc_safety_update(cbplc_safety_t *s, const cbplc_safety_inputs_t *in,
                         uint32_t now_ms);

/* The pins as last sampled, bit i per the order above. */
uint8_t cbplc_safety_raw_mask(const cbplc_safety_t *s);

bool cbplc_safety_bus_unreadable(const cbplc_safety_t *s);

/* ---------------------------------------------------------------------------
 * Relay state
 * ------------------------------------------------------------------------- */
typedef struct {
    uint8_t closed;          /* bitmask of relays believed closed */
    uint8_t verified_open;   /* relays PROVEN open by readback */
    uint32_t deadline_ms[3]; /* per-relay dead-man expiry; 0 = none armed */
    /* Set when a readback disagreed with what was commanded. The expander's state
     * is then unknown, and unknown collapses to all-open until a verified write
     * succeeds. Latched, because a bus that lied once may lie again. */
    bool drifted;
    uint32_t drift_events;
    uint32_t denied_closes;
    /* The readback that disagreed, kept for diagnosis only.
     *
     * The collapse below zeroes `verified_open`, which is correct — "proven open" must
     * mean "a read we currently trust said so" — but it also destroyed the only record of
     * WHICH pins disagreed, so a drifted board could report that it was drifted and never
     * what it saw. A field investigation spent days unable to tell one stuck relay pin from
     * three. This field is written on the mismatch and read by nothing except STATUS. */
    uint8_t drift_readback;
} cbplc_relay_t;

/* Everything that must be true before a close is permitted. Assembled by the
 * caller so the decision is one auditable expression instead of scattered checks. */
typedef struct {
    bool vehicle_present;   /* from the CP decoder — untrusted CP answers false */
    bool authorized;        /* controller AUTHORIZE is live and granted */
    bool controller_alive;  /* heartbeat within its window */
    bool hlc_permits;       /* the V2G layer has not asked for a stop */
} cbplc_close_preconditions_t;

/* May `mask` be closed right now? Returns CBPLC_E_OK, or the specific reason.
 *
 * The GUN relay is held to a stricter rule than the others: closing it onto a
 * connector with no vehicle is the one mistake with no safe outcome, so
 * vehicle_present is checked UNCONDITIONALLY for it, in every ownership mode.
 * plc_firmware had a bug where a mode flag skipped that check.
 *
 * Everything ELSE required here applies to EVERY relay this board drives — see
 * CBPLC_RELAY_ENERGY for why the old GUN|POWER carve-out was a hole rather than a
 * concession. */
cbplc_err_t cbplc_relay_may_close(const cbplc_relay_t *r,
                                  uint8_t mask,
                                  const cbplc_close_preconditions_t *pre);

/* Bound a requested dead-man hold. `clamped` reports an adjustment rather than
 * applying it silently. 0 is legal and means "no dead-man". */
uint32_t cbplc_relay_bound_hold(uint32_t requested_ms, bool *clamped);

/* Record the outcome of a verified write.
 *
 * `readback` is what the expander actually reports. If it disagrees with
 * `commanded`, the relay state is marked drifted and everything collapses to
 * all-open — this function does not retry, because a readback mismatch means the
 * expander's state is unknown and guessing is what makes it dangerous.
 *
 * Returns true when the write is verified good. */
bool cbplc_relay_note_write(cbplc_relay_t *r,
                            uint8_t commanded,
                            uint8_t readback,
                            uint32_t now_ms,
                            uint32_t hold_ms);

/* Which relays must be opened NOW: dead-man expiries plus everything if drifted.
 * Call every poll; it is the mechanism that makes a lease real. */
uint8_t cbplc_relay_expired(const cbplc_relay_t *r, uint32_t now_ms);

/* Force the belief to all-open, e.g. after a successful emergency open. */
void cbplc_relay_note_all_open(cbplc_relay_t *r);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_SAFETY_H */
