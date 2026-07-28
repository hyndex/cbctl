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
 * HOW THE TWO ANSWERS DIFFER, AND WHY THERE ARE TWO
 *   A safety input has to drive two different decisions with different costs:
 *
 *     de-energise NOW      One raw sample is enough. Waiting to be sure before
 *                          opening a contactor is not a trade worth making.
 *
 *     latch the session    Requires a DEBOUNCED assertion — the input held for
 *                          kDebounceMs. A single glitch that then proves clean must
 *                          not end a charge that has to be restarted by hand.
 *                          Measured: a transient on this bench board latched the
 *                          session in Fault while the chain read clear, and only an
 *                          unplug could clear it.
 *
 *   So cbplc_safety_raw_asserted() gates energy and cbplc_safety_faulted() is the
 *   verdict. Both directions are needed; collapsing them into one loses either the
 *   fail-fast open or the tolerance of a glitch.
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
 * wire, so renumbering them silently changes which contactor moves. */
#define CBPLC_RELAY_GUN   0x01u /* DC gun contactor — the dangerous one */
#define CBPLC_RELAY_AUX   0x02u
#define CBPLC_RELAY_POWER 0x04u /* upstream/module power path */
#define CBPLC_RELAY_ALL   0x07u

/* Dead-man ceiling. A LONGER value is strictly less safe, so this is the longest
 * we accept and a request beyond it is clamped AND reported — never silently
 * shortened, which is how two ends end up disagreeing about when a relay opens.
 * Sized to outlast a supervising controller's restart, matching the 30 s lease
 * plc_firmware settled on for exactly that reason. */
#define CBPLC_RELAY_HOLD_MAX_MS 30000u

/* How long an input must hold a state before that state is believed.
 *
 * 120 ms is long enough to ride out the contact bounce and the brownout-induced I2C
 * glitches this carrier produces (measured in the field at 100-1000 ms on
 * power-fragile boards, so a longer window would swallow real faults), and short
 * enough that a genuine fault latches well inside a human reaction time. The relays
 * do not wait for it — see raw_asserted above. */
#define CBPLC_SAFETY_DEBOUNCE_MS 120u

/* The external safety chain. Every one is active-low in hardware and normalised to
 * "true = asserted = bad" here, so no reader has to remember the polarity. */
typedef struct {
    bool emergency_stop;
    bool earth_fault;
    bool fuse_or_spd;
    bool ac_voltage;
    bool insulation;
} cbplc_safety_inputs_t;

/* One debouncer per input. Both fields start true: fail-safe by construction. */
typedef struct {
    bool sampled_active;
    bool active;
    uint32_t last_change_ms;
} cbplc_safety_input_t;

#define CBPLC_SAFETY_INPUT_COUNT 5

typedef struct {
    cbplc_safety_input_t in[CBPLC_SAFETY_INPUT_COUNT];
    /* Set once a fault has ever been confirmed this boot. Diagnostics only — it must
     * never gate anything, because "was faulted" is not "is faulted". */
    bool ever_faulted;
    uint32_t confirmations;
    /* Bit per input, set the first time that input is believed inactive. */
    uint8_t proven_mask;
} cbplc_safety_t;

/* Start every input ASSERTED. A zeroed struct would start them clear, which is the
 * one starting belief a safety chain must not have. */
void cbplc_safety_init(cbplc_safety_t *s, uint32_t now_ms);

/* Feed one sample of the chain. `in` NULL means the read FAILED, which is treated as
 * every input asserted with no evidence — a missing sample is not a clear one, and a
 * dead I2C bus must not read as a healthy safety chain.
 *
 * Returns true while any input is CONFIRMED (debounced) asserted. */
bool cbplc_safety_update(cbplc_safety_t *s, const cbplc_safety_inputs_t *in,
                         uint32_t now_ms);

/* The VERDICT: some input has held asserted long enough to be believed. This is what
 * latches a session fault. */
bool cbplc_safety_faulted(const cbplc_safety_t *s);

/* Has the chain ever been PROVEN clear?
 *
 * Every input starts asserted (fail-safe), so "not clear" at boot means "not yet
 * proven", which is a completely different thing from "confirmed faulted". Without
 * this distinction a charger that boots with a vehicle already plugged in latches a
 * safety fault before it has taken its first stable sample — measured on the bench,
 * and it needed an unplug to clear.
 *
 * True once every input has been believed INACTIVE at least once since init. Energy
 * still requires the chain to be clear NOW; this only says a fault is a real
 * transition rather than a startup condition. */
bool cbplc_safety_proven(const cbplc_safety_t *s);

/* The RAW answer: some input is asserted in the most recent sample, believed or not.
 * This is what gates energy, because opening a contactor is cheap and being wrong
 * about it in the other direction is not. */
bool cbplc_safety_raw_asserted(const cbplc_safety_t *s);

/* A short stable token naming the FIRST asserted input, for the wire. Callers get
 * "why", not just "no" — a refusal without a cause costs a site visit. */
const char *cbplc_safety_reason(const cbplc_safety_t *s);

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
} cbplc_relay_t;

/* Everything that must be true before a close is permitted. Assembled by the
 * caller so the decision is one auditable expression instead of scattered checks. */
typedef struct {
    bool vehicle_present;   /* from the CP decoder — untrusted CP answers false */
    bool authorized;        /* controller AUTHORIZE is live and granted */
    bool controller_alive;  /* heartbeat within its window */
    bool safety_clear;      /* no latched fault */
    bool hlc_permits;       /* the V2G layer has not asked for a stop */
} cbplc_close_preconditions_t;

/* May `mask` be closed right now? Returns CBPLC_E_OK, or the specific reason.
 *
 * The GUN relay is held to a stricter rule than the others: closing it onto a
 * connector with no vehicle is the one mistake with no safe outcome, so
 * vehicle_present is checked UNCONDITIONALLY for it, in every ownership mode.
 * plc_firmware had a bug where a mode flag skipped that check. */
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
