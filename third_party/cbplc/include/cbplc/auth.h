/* The controller's authorisation decision, as a THREE-state lease.
 *
 * WHY THIS EXISTS AS ITS OWN CONTRACT
 *   Authorisation used to be one bit on the wire: granted, or not. That cannot express the
 *   ordinary case. A driver plugs in and then taps their card, so for a few seconds the
 *   honest answer is "I am still deciding" — and a single bit forces that to be reported as
 *   the same thing as "refused". The two demand opposite behaviour from a vehicle:
 *
 *     PENDING  answer AuthorizationRes(OK, Ongoing) and keep the session alive. ISO even has
 *              a dedicated Ongoing_WaitingForCustomerInteraction for exactly this, which
 *              lets the car display "waiting for authorisation" instead of an error.
 *     DENY     answer FAILED and close. A vehicle told this stops asking, which is correct
 *              and kind: it can show the driver something true.
 *
 *   With one bit cbplc had to pick one behaviour for both, and picking Ongoing meant a
 *   refused driver waited out a 60 s cap staring at nothing. Picking FAILED meant every
 *   tap-after-plug failed. Neither is acceptable, and no amount of tuning fixes a missing
 *   distinction — the information was never on the wire.
 *
 * WHY THE LEASE IS THE CONTROLLER'S TO SET
 *   A grant must expire, or a charger whose supervisor has died still believes it may start
 *   a new session for the rest of its uptime. But the right expiry depends on how often the
 *   controller actually speaks, which only the controller knows. A figure hardcoded in the
 *   PLC is either too short (it revokes mid-session on a busy controller) or too long (it
 *   outlives the supervisor). So the controller sends it, and the PLC BOUNDS it — because a
 *   controller bug that sends 0 or 49 days must not become the PLC's problem.
 */
#ifndef CBPLC_AUTH_H
#define CBPLC_AUTH_H

#include <stdbool.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /* Refused. Terminal: the vehicle is told FAILED and the session ends. */
    CBPLC_AUTH_DENY = 0,
    /* Not yet. The vehicle is told Ongoing and the session stays open. */
    CBPLC_AUTH_PENDING = 1,
    /* Authorised, for as long as the lease holds. */
    CBPLC_AUTH_GRANTED = 2,
} cbplc_auth_state_t;

/* Wire payload: [0] decision, [1..2] lease in 100 ms units, little-endian.
 *
 * A 1-byte payload is still accepted as the LEGACY form — 0 = DENY, non-zero = GRANTED with
 * the default lease — so a PLC flashed ahead of its controller keeps working. Legacy cannot
 * express PENDING, which is exactly the capability being added; it is not a supported way to
 * write new controller code, and it is rejected for anything but those two values. */
#define CBPLC_AUTH_PAYLOAD_LEN 3u
#define CBPLC_AUTH_LEGACY_PAYLOAD_LEN 1u

/* Lease bounds, in milliseconds. From the reference's [100 ms, 600 s].
 *
 * The floor stops a controller bug from revoking authorisation between two of its own
 * heartbeats; the ceiling stops one from granting for a week. A value outside the range is
 * CLAMPED rather than rejected: refusing the message entirely would leave the previous
 * decision standing, and silently keeping an old grant is worse than honouring a clamped
 * new one. */
#define CBPLC_AUTH_LEASE_MIN_MS 100u
#define CBPLC_AUTH_LEASE_MAX_MS 600000u
/* Used for the legacy 1-byte form, and when a controller sends 0. Long enough to survive a
 * busy controller, short enough that a dead one stops mattering within a session. */
#define CBPLC_AUTH_LEASE_DEFAULT_MS 8000u

/* HOW LONG A REFUSAL STANDS BEFORE IT DECAYS TO "still deciding".
 *
 * DENY was the one decision with no expiry, which made it the most durable thing on the wire:
 * a controller that refused one driver and then died left the charger refusing EVERY vehicle
 * for the rest of its uptime, with no way back short of a reboot. That is a worse outage than
 * the one the lease was introduced to prevent.
 *
 * A refusal is about a PARTICULAR authorisation attempt. Once the controller has stopped
 * speaking, it is no longer refusing anything — it is absent — and absent is PENDING. 30 s is
 * long enough that a refused driver sees a stable answer rather than a flicker, and short
 * enough that the next driver is not punished for the last one. */
#define CBPLC_AUTH_DENY_TTL_MS 30000u

typedef struct {
    /* MUST BE TRUE FOR ANY DECISION TO BE BELIEVED, and it is false in a zero-initialised
     * struct — which is the whole point.
     *
     * CBPLC_AUTH_DENY is 0 because that is its wire value, so a static cbplc_auth_t that
     * nobody initialised reads as a TERMINAL REFUSAL: every vehicle plugging in before the
     * controller's first AUTHORIZE would be told FAILED and would stop asking. That is
     * exactly what happened — cbplc_auth_init() was written, tested, and never called, and
     * the test passed because it tested the function rather than the path.
     *
     * So forgetting the init now fails SAFE: cbplc_auth_state() returns PENDING until this
     * is set. The init is still required and still called; this makes the consequence of
     * forgetting it a delay rather than an outage. */
    bool initialised;
    cbplc_auth_state_t state;
    /* When the lease expires. 0 means "no lease held" — which is also what DENY and PENDING
     * leave behind, so there is exactly one thing to check. */
    uint32_t expiry_ms;
    /* Counters that separate the three ways authorisation can be absent. Guessing between
     * them cost a bench session. */
    uint32_t grants;
    uint32_t denials;
    uint32_t pendings;
    uint32_t lease_expiries;
    uint32_t clamped;      /* leases outside the bounds */
    uint32_t rejects;      /* malformed payloads */
    uint32_t legacy;       /* 1-byte payloads seen */
} cbplc_auth_t;

/* THE SUPERVISORY TIMEOUT, DERIVED FROM THE CONTROLLER'S OWN PERIOD.
 *
 * The last hardcoded guess in this firmware. The heartbeat timeout was 32 s, chosen because
 * cbcontroller's actual cadence was not written down anywhere — too short and it tears down
 * sessions on a busy controller, too long and a dead supervisor goes unnoticed for half a
 * minute while the relay dead-man is the only thing still watching.
 *
 * A guess is not needed: the controller knows its own period, so it sends it, exactly as it
 * now sends the authorisation lease. Three periods of tolerance, because one missed message
 * plus scheduling jitter is not a dead controller and three consecutive misses is — the same
 * reasoning as CBPLC_LIMITS_STALE_MS, and deliberately the same factor.
 *
 * BOUNDED, for the same reason the lease is: a controller that announces a 10 ms period must
 * not be able to make the PLC tear down every session, and one announcing an hour must not
 * be able to disable supervision. An absent period keeps the legacy default, so a controller
 * that has not been updated behaves exactly as it does today. */
#define CBPLC_HB_TIMEOUT_MIN_MS 1000u
#define CBPLC_HB_TIMEOUT_MAX_MS 60000u
#define CBPLC_HB_TIMEOUT_DEFAULT_MS 32000u
#define CBPLC_HB_TOLERATED_MISSES 3u

/* Turn a controller-announced period into a supervisory timeout. `period_ms` of 0 means the
 * controller did not say, and yields the legacy default. Always returns a value inside the
 * bounds, so no caller has to re-check. */
uint32_t cbplc_hb_timeout_from_period(uint32_t period_ms);

/* Is the controller alive as of `now_ms`, given the last time any of its messages arrived?
 *
 * The subtlety is the SIGN. `last_ms` is stamped with live millis() inside a message
 * handler; `now_ms` is typically captured at the top of the loop pass that then runs the
 * handler — so `last_ms` can legitimately be a few milliseconds AFTER `now_ms`. Unsigned
 * age math turns that into ~49.7 days and declares a controller dead at the exact moment
 * it spoke. That shipped: sessions churned Linking->Stopping:controller_lost on a 400 ms
 * feed, but only under SLAC/SECC load where loop passes are long — at idle the collision
 * window is a microsecond and every bench run passed. A stamp from the future is age zero.
 *
 * `last_ms == 0` means no message has EVER arrived, and that is dead — a charger that has
 * never heard from its controller must not energise. Handlers guarantee a real stamp is
 * never 0 by mapping 0 to 1. */
bool cbplc_hb_alive(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms);

void cbplc_auth_init(cbplc_auth_t *a);

/* Decode an AUTHORIZE payload and record the decision.
 *
 * Returns CBPLC_E_OK, CBPLC_E_BAD_LENGTH for a size that is neither form, or
 * CBPLC_E_BAD_ARGUMENT for an unknown decision byte. A rejected message leaves the previous
 * decision UNTOUCHED — a corrupt frame must not be able to revoke or grant. */
cbplc_err_t cbplc_auth_apply(cbplc_auth_t *a, const uint8_t *payload, uint16_t len,
                             uint32_t now_ms);

/* Encode the canonical 3-byte form. Returns bytes written, or 0 if the buffer is too
 * small. Here so the harness and the controller have one authoritative layout. */
size_t cbplc_auth_encode(cbplc_auth_state_t state, uint32_t lease_ms, uint8_t *out,
                         size_t cap);

/* The decision AS OF now, with the lease applied. GRANTED becomes PENDING when the lease
 * runs out — not DENY, because a lapsed lease means the controller went quiet, which is not
 * the same as the controller refusing. Call every tick so lease_expiries is counted. */
cbplc_auth_state_t cbplc_auth_state(cbplc_auth_t *a, uint32_t now_ms);

/* Short stable token for the wire: "deny", "pending", "granted". */
const char *cbplc_auth_name(cbplc_auth_state_t s);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_AUTH_H */
