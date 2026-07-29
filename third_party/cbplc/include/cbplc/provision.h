/* Per-charger identity and per-board calibration, DECLARED by the controller.
 *
 * WHY THIS EXISTS AT ALL, GIVEN THAT cbplc DELIBERATELY STORES NOTHING
 *   Three facts that belong to a charger rather than to firmware were compiled in:
 *
 *     - the EVSE ID. `kEvseId[] = "IN*JPE*E000100010001"` shipped in every binary, so every
 *       charger in the fleet announced the same identity to every vehicle. That is wrong for
 *       OCPP, wrong for billing, and wrong in a way nobody notices until two chargers are on
 *       one site.
 *     - the SLAC receive-path correction. A single 18 dB constant stands in for cable length,
 *       connector and coupling — physical properties that vary per unit. ISO 15118-3 expects
 *       this to be measured, and Sevenstax's WhiteBeet manual is blunt about it: the
 *       transmission path "must be configured once for each device".
 *     - which HLC protocols to accept. Forcing DIN on a vehicle that fails ISO is a one-run
 *       experiment on any charger that can be told; on ours it was a firmware rebuild.
 *
 * WHY IT IS A DECLARATION AND NOT A STORED SETTING
 *   cbplc has no NVS, no config file and no board identity, and that is the design: a stored
 *   plc_id that can corrupt is how a dead board and a misconfigured board became
 *   indistinguishable, and MAC-as-identity exists to make that unrepresentable. Adding a
 *   config store to hold three fields would give the whole class of bug a home again.
 *
 *   The controller already has a filesystem and already knows all three. So this is shaped
 *   like LIMITS and AUTHORIZE: the controller declares, the board holds it in RAM for as long
 *   as it is powered, and cbctl re-asserts it after a reset. A board that reboots comes back
 *   at compiled defaults — which are the values that shipped before this existed, so an
 *   un-updated controller changes nothing.
 *
 * WHY TLV AND NOT A FIXED STRUCT
 *   Every other payload here is a fixed layout, and that is right for messages whose shape is
 *   settled. This one is explicitly the place where per-site facts accumulate, so it is
 *   tag-length-value: a field added next year is skipped by an older peer instead of shifting
 *   every offset after it. Unknown tags are IGNORED, not rejected — that is what makes the
 *   forward direction work — but a MALFORMED frame is rejected whole, because a partially
 *   applied identity is worse than none.
 *
 *   Tag values are never re-used. Retiring one leaves a gap.
 */
#ifndef CBPLC_PROVISION_H
#define CBPLC_PROVISION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ISO 15118-2 evseIDType is minLength 7 / maxLength 37. DIN 70121's EVSEID is shorter still,
 * so one buffer serves both and the length check is the ISO one. A shorter string is refused
 * rather than padded: an out-of-schema EVSEID is rejected by strict vehicles on every message
 * of the charge loop, which presents as "the car stopped talking" and not as a config error. */
#define CBPLC_EVSE_ID_MIN 7u
#define CBPLC_EVSE_ID_MAX 37u

/* Protocol mask. Bit values are wire values. */
#define CBPLC_PROTO_DIN70121 0x01u
#define CBPLC_PROTO_ISO2     0x02u
#define CBPLC_PROTO_ALL      (CBPLC_PROTO_DIN70121 | CBPLC_PROTO_ISO2)

/* The receive-path correction is bounded because it is a physical trim, not a free parameter.
 * ±30 dB is far wider than any real cable and narrow enough that a corrupt byte cannot make a
 * distant vehicle look adjacent — which is the cross-gun theft SLAC exists to prevent. */
#define CBPLC_ATTN_TRIM_MIN (-30)
#define CBPLC_ATTN_TRIM_MAX 30

/* Bench knob, deliberately bounded and deliberately not persisted. 0 means "use the compiled
 * default", so clearing it is expressible. */
#define CBPLC_SESSION_PARAM_TIMEOUT_MAX_MS 10000u

/* TLV tags. NEVER re-use a value. */
typedef enum {
    CBPLC_PROV_EVSE_ID = 1,        /* ASCII, 7..37 bytes                        */
    CBPLC_PROV_PROTOCOL_MASK = 2,  /* u8, CBPLC_PROTO_*; must be non-zero       */
    CBPLC_PROV_ATTN_TRIM_DB = 3,   /* int8, dB, CBPLC_ATTN_TRIM_MIN..MAX        */
    CBPLC_PROV_SESSION_TIMEOUT = 4 /* u16 LE ms, 0 = compiled default           */
} cbplc_prov_tag_t;

/* What the board currently believes. `*_set` says whether the controller has declared it, so
 * "the default is 18 dB" and "the controller asked for 18 dB" stay distinguishable — the same
 * reason cbctl_ev_soc_pct() returns -1 rather than 0. */
typedef struct cbplc_provision_s {
    char     evse_id[CBPLC_EVSE_ID_MAX + 1u];
    uint8_t  evse_id_len;
    bool     evse_id_set;

    uint8_t  protocol_mask;
    bool     protocol_mask_set;

    int8_t   attn_trim_db;
    bool     attn_trim_set;

    uint16_t session_timeout_ms;
    bool     session_timeout_set;

    /* Accepted declarations since boot. Non-zero proves the controller reached us, which is
     * the difference between "running on defaults because nobody asked" and "running on
     * defaults because the declaration was refused". */
    uint32_t applied;
    uint32_t rejected;
} cbplc_provision_t;

/* Compiled defaults. Called at boot and after any reset; leaves every *_set false. */
void cbplc_provision_defaults(cbplc_provision_t *p);

/* Decode and apply one PROVISION payload.
 *
 * ALL-OR-NOTHING. Every TLV is validated before any field is written, so a frame whose third
 * field is malformed cannot leave the first two applied — a half-applied identity is a charger
 * announcing one thing and behaving as another. Returns CBPLC_E_OK on success and leaves `p`
 * untouched on any error.
 *
 * An EMPTY payload is legal and means "no change": it is how a controller probes support
 * without asserting anything. */
cbplc_err_t cbplc_provision_apply(cbplc_provision_t *p, const uint8_t *payload, uint16_t len);

/* Encode a declaration. Only the fields whose `*_set` is true are emitted, so a caller can
 * declare a subset. Returns bytes written, or 0 if the arguments are not encodable or the
 * buffer is short — a truncated declaration is never put on the wire. */
size_t cbplc_provision_encode(const cbplc_provision_t *p, uint8_t *out, size_t cap);

/* Longest encodable declaration: 4 tags, each with a 2-byte header. */
#define CBPLC_PROVISION_MAX_PAYLOAD ((2u + CBPLC_EVSE_ID_MAX) + 3u + 3u + 4u)

/* Is this protocol permitted right now? Answers TRUE when nothing has been declared, so the
 * gate is inert until a controller opts in. */
bool cbplc_provision_protocol_allowed(const cbplc_provision_t *p, uint8_t proto_bit);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_PROVISION_H */
