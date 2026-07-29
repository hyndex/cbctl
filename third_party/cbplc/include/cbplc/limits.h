/* The controller's charging envelope and live measurements. Pure policy, host-testable.
 *
 * WHY THIS EXISTS
 *   Every DC response the PLC sends a vehicle is mostly made of numbers the PLC does not
 *   measure: EVSEPresentVoltage, EVSEPresentCurrent, the maximum/minimum limits, the
 *   isolation verdict. Those come from the power modules and the insulation monitor,
 *   which are on the controller's side of the wire. Without a defined feed for them a
 *   fully working SECC completes SDP and ServiceDiscovery and then stalls at CableCheck
 *   with nothing to report and no diagnosable cause — the same shape as the isolation
 *   omission that stalled DIN vehicles fleet-wide.
 *
 * WHY IT IS A PERIODIC FEED AND NOT A REQUEST/RESPONSE
 *   Copied from the reference commercial stack (Sevenstax WHITE beet), whose manual is
 *   explicit: "Update DC Charging Parameters must be sent at least every 500 ms from now
 *   on, or when values change", starting at CableCheck and continuing through PreCharge,
 *   the CurrentDemand loop and WeldingDetection. The vehicle's charge loop runs
 *   continuously and every iteration needs current numbers; asking for them per message
 *   would put a controller round trip inside an EV message deadline.
 *
 * THE RULE THAT MATTERS: STALE IS NOT VALID
 *   A feed that stops is the dangerous case, because the last values remain perfectly
 *   plausible. Reporting a stale present-voltage to a vehicle is how a precharge
 *   convergence test passes on numbers that are no longer true. So the data carries its
 *   arrival time and everything derived from it expires.
 *
 *   Expiry does NOT open contactors — that is the relay dead-man lease's job, and it
 *   works whether or not this module exists. Expiry withdraws the PERMISSION to
 *   energise, so the two mechanisms are independent and neither is load-bearing alone.
 *
 * WHY FIXED-POINT INTEGERS ON THE WIRE
 *   No floats. Their endianness is not guaranteed across the controller/PLC boundary,
 *   NaN and infinity have no defined meaning for a current limit, and a corrupted
 *   exponent turns a small number into an enormous one. Millivolts, milliamps and watts
 *   are exact, and their ranges are checked here rather than trusted.
 *
 * WHAT THIS MUST NOT DO
 *   No I/O, no clock of its own. The caller supplies both the bytes and the time, which
 *   is what lets every rule here be tested with no board and no vehicle.
 */
#ifndef CBPLC_LIMITS_H
#define CBPLC_LIMITS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"
#include "cbplc/provision.h"   /* CBPLC_SESSION_PARAM_TIMEOUT_MAX_MS bounds the override */

#ifdef __cplusplus
extern "C" {
#endif

/* How long a feed remains usable after its last update.
 *
 * Three times the mandated 500 ms cadence. Tolerating one missed update plus scheduling
 * jitter avoids nuisance stops on a busy controller; three consecutive misses is not
 * jitter, it is a controller that has stopped talking. Shorter would trade real session
 * failures for a safety margin the relay dead-man already provides. */
#define CBPLC_LIMITS_STALE_MS 1500u

/* How long a session may CONTINUE on a feed that has gone stale, before it is failed.
 *
 * 10 s, from the reference's CC_FEEDBACK_STALE_GRACE_MS. The distinction between this and
 * CBPLC_LIMITS_STALE_MS above is the whole point, and getting it wrong cost a session:
 *
 *   STALE (1.5 s)  the numbers are no longer trustworthy, so stop ASSERTING them —
 *                  answer Ongoing, report NotReady, offer no energy.
 *   GRACE (10 s)   but do NOT tear the session down yet. A controller that is briefly
 *                  busy is not a controller that has died, and a vehicle whose stream is
 *                  reset mid-negotiation treats it as a supply fault and starts over.
 *
 * cbplc originally returned -EIO the moment the feed went stale, which killed the TCP
 * stream at 1.5 s. The reference holds the exchange open for 10 s and only then gives up —
 * that is nearly seven times more tolerance, and it is the difference between riding out a
 * busy controller and failing every session that meets one. */
#define CBPLC_LIMITS_GRACE_MS 10000u

/* Plausibility ceilings. A value beyond these is a decode or units error, not a
 * charger: no CCS DC connector runs at 2000 V or 1000 A, and accepting a corrupted
 * limit is worse than rejecting the message — the vehicle would be told it may draw it. */
#define CBPLC_LIMITS_MAX_VOLTAGE_MV 1000000u /* 1000 V */
#define CBPLC_LIMITS_MAX_CURRENT_MA 1000000u /* 1000 A */
#define CBPLC_LIMITS_MAX_POWER_W 500000u     /* 500 kW */

/* Wire sizes. Fixed layouts, little-endian, no padding — the encoder and decoder are
 * both here and both tested against these constants. */
#define CBPLC_LIMITS_PAYLOAD_LEN 22u
#define CBPLC_FEEDBACK_PAYLOAD_LEN 12u

/* The insulation monitor's verdict, as the V2G layer needs it.
 *
 * UNKNOWN is distinct from INVALID on purpose: "not measured yet" and "measured and
 * bad" are different facts, and a DIN CableCheckRes that reports an absent verdict as a
 * valid one is how vehicles get stranded at CableCheck. There is no default that is safe
 * for both, so the caller must handle UNKNOWN explicitly. */
typedef enum {
    CBPLC_ISO_UNKNOWN = 0,
    CBPLC_ISO_INVALID = 1,
    CBPLC_ISO_VALID = 2,
    CBPLC_ISO_WARNING = 3,
    CBPLC_ISO_FAULT = 4,
} cbplc_isolation_t;

typedef enum {
    CBPLC_CONTACTOR_UNKNOWN = 0,
    CBPLC_CONTACTOR_OPEN = 1,
    CBPLC_CONTACTOR_CLOSED = 2,
} cbplc_contactor_t;

/* The controller's charging envelope: what the vehicle may be offered. */
typedef struct {
    uint32_t max_voltage_mv;
    uint32_t min_voltage_mv;
    uint32_t max_current_ma;
    uint32_t min_current_ma;
    uint32_t max_power_w;
    /* The controller's own permission bit. Distinct from every other gate: this is the
     * power path saying it is ready, not the session saying it is allowed. */
    bool power_ready;
} cbplc_envelope_t;

/* Live measurements from the power path. */
typedef struct {
    uint32_t present_voltage_mv;
    uint32_t present_current_ma;
    cbplc_isolation_t isolation;
    cbplc_contactor_t contactor;
} cbplc_measurement_t;

typedef struct {
    cbplc_envelope_t envelope;
    cbplc_measurement_t measurement;
    uint32_t envelope_ms;    /* arrival time; 0 = never received */
    uint32_t measurement_ms;
    /* Counters that can all actually move, for the STATUS reply. */
    uint32_t envelope_updates;
    uint32_t measurement_updates;
    uint32_t rejects;        /* malformed or implausible, per message */
    uint32_t expiries;       /* transitions from fresh to stale */
    bool was_fresh;          /* edge detection for `expiries` */
    /* THE FEED DEADLINE, runtime-overridable for the bench and ONLY for the bench.
     *
     * 0 means "use CBPLC_LIMITS_STALE_MS", which is what production runs and what every board
     * comes up with — this does not persist. It exists because interop work repeatedly needed
     * this number moved and the only way was a rebuild and a reflash. It is bounded by the
     * PROVISION codec and published in STATUS, so a charger running a non-default deadline
     * cannot do so invisibly. */
    uint32_t stale_ms;
} cbplc_limits_t;

void cbplc_limits_init(cbplc_limits_t *l);

/* Decode a LIMITS payload and record it.
 *
 * Returns CBPLC_E_OK, CBPLC_E_BAD_LENGTH for the wrong size, or CBPLC_E_BAD_ARGUMENT
 * for a value outside its plausibility range or a min above its max. A rejected message
 * leaves the previous envelope UNTOUCHED and does not refresh its timestamp: a corrupt
 * update must not extend the life of a good one. */
cbplc_err_t cbplc_limits_apply_envelope(cbplc_limits_t *l, const uint8_t *payload,
                                        uint16_t len, uint32_t now_ms);

/* Decode a FEEDBACK payload and record it. Same rejection contract. */
cbplc_err_t cbplc_limits_apply_measurement(cbplc_limits_t *l, const uint8_t *payload,
                                           uint16_t len, uint32_t now_ms);

/* Encoders, so the Python harness and the controller have one authoritative layout to
 * agree with. Return the bytes written, or 0 if the buffer is too small. */
size_t cbplc_limits_encode_envelope(const cbplc_envelope_t *e, uint8_t *out, size_t cap);
size_t cbplc_limits_encode_measurement(const cbplc_measurement_t *m, uint8_t *out,
                                       size_t cap);

/* Are BOTH feeds fresh? Both, because a DC response needs limits and measurements
 * together — one without the other cannot fill a CableCheckRes or a CurrentDemandRes. */
/* Override the feed deadline. 0 restores the compiled default. Values outside
 * 1..CBPLC_SESSION_PARAM_TIMEOUT_MAX_MS are clamped rather than refused: the caller has already
 * been bounds-checked by the PROVISION codec, and a clamp here is the belt to that braces. */
void cbplc_limits_set_stale_ms(cbplc_limits_t *l, uint32_t ms);
uint32_t cbplc_limits_stale_ms(const cbplc_limits_t *l);

bool cbplc_limits_fresh(const cbplc_limits_t *l, uint32_t now_ms);

/* Is the feed stale but still inside the grace window — i.e. answer the vehicle, but
 * without asserting numbers or offering energy?
 *
 * False once the grace expires, which is the point at which a session genuinely has to
 * fail rather than keep promising. */
bool cbplc_limits_in_grace(const cbplc_limits_t *l, uint32_t now_ms);

/* Call every tick so the fresh->stale transition can be counted. Returns freshness, so
 * a caller can use it directly in place of cbplc_limits_fresh(). */
bool cbplc_limits_tick(cbplc_limits_t *l, uint32_t now_ms);

/* May the vehicle be offered energy on the strength of the controller's feed alone?
 *
 * Requires: both feeds fresh, power_ready, a non-degenerate envelope, isolation VALID,
 * and the contactor not reported OPEN while energy is expected. This is ONE input to
 * the session machine's decision, not the decision — the safety interlocks and the
 * session state are ANDed on top by the caller. */
bool cbplc_limits_permit_energy(const cbplc_limits_t *l, uint32_t now_ms);

/* Short stable token naming why energy is refused, "" when it is not. */
const char *cbplc_limits_refusal(const cbplc_limits_t *l, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_LIMITS_H */
