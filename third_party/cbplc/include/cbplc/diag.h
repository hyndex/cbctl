/* Diagnostic log CONTROL: turning the board's log ring on, off, and down to a category.
 *
 * WHY THIS IS A CONTRACT AND NOT TWO printf CALLS
 *   The log ring already had a transport — CBPLC_T_DIAG drains it, framed and CRC'd, because
 *   printing text onto the control stream once cost three debugging sessions (a "v2g: matched"
 *   line read as a 12918-byte length prefix and swallowed thirteen kilobytes of real frames).
 *   What it did NOT have was a way to say WHAT to log. Everything the firmware chose to emit
 *   went in, unconditionally, and the only control was a recompile.
 *
 *   That is the wrong shape for the job it is now being asked to do. Vehicle interop forensics
 *   wants a per-message decoded V2G trace — roughly 100 bytes per message, 40-60 messages a
 *   session — and nobody wants that firehose running on a production charger that is working.
 *   So the controller must be able to arm it for one vehicle, read it, and disarm it.
 *
 * WHY THE SUBCOMMAND LIVES ON CBPLC_T_DIAG RATHER THAN A NEW MESSAGE TYPE
 *   The DIAG handler has always ignored its request payload, so payload_len == 0 keeps meaning
 *   DRAIN and byte 0 becomes the subcommand. Same shape as SLAC, whose arm/replug are byte 0 of
 *   its payload. A new message type would need a schema bump, a generator run, and a Python
 *   constant, to express something the existing type can carry.
 *
 * THE COMPATIBILITY PROBLEM, AND WHY THE REPLY CARRIES A SENTINEL
 *   An OLD firmware answers ANY DIAG request by draining. So a controller that sends SET_LEVEL
 *   to a board built before this existed gets back log TEXT, and if it parsed that as a control
 *   reply it would read four arbitrary ASCII bytes as a status. The reply therefore begins with
 *   a NUL: cbplc_diag_read() returns printf output, which never contains a NUL, so a leading
 *   zero byte cannot be produced by a drain. Combined with a magic byte, a version and an exact
 *   length, "is this a control reply?" is decidable with no version negotiation at all — and a
 *   controller talking to an old board simply observes that control is unsupported and keeps
 *   draining, which is the useful half anyway.
 *
 *   Wire values are never re-used here either. Retiring a subcommand leaves a gap.
 */
#ifndef CBPLC_DIAG_H
#define CBPLC_DIAG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Request subcommand, payload byte 0. An EMPTY payload is DRAIN, which is what every
 * firmware ever built has done and what an unmodified controller still sends. */
typedef enum {
    CBPLC_DIAG_DRAIN = 0,      /* reply: log bytes (possibly none) */
    CBPLC_DIAG_SET_LEVEL = 1,  /* payload[1] = minimum level to record */
    CBPLC_DIAG_SET_CATS = 2,   /* payload[1..4] = category bitmask, little-endian */
    CBPLC_DIAG_GET_STATE = 3,  /* no arguments; reply carries the whole state */
    CBPLC_DIAG_CLEAR = 4,      /* discard buffered text; reply carries the state */
} cbplc_diag_cmd_t;

/* Minimum level to RECORD. Values 0..3 match CBPLC_LOG_DEBUG..CBPLC_LOG_ERROR in platform.h;
 * OFF is one past ERROR so "record nothing" is expressible without a separate flag that could
 * disagree with the level. */
#define CBPLC_DIAG_LEVEL_DEBUG 0u
#define CBPLC_DIAG_LEVEL_INFO  1u
#define CBPLC_DIAG_LEVEL_WARN  2u
#define CBPLC_DIAG_LEVEL_ERROR 3u
#define CBPLC_DIAG_LEVEL_OFF   4u

/* Categories. A bitmask rather than a second level, because the interesting question is not
 * "how important" but "which subsystem" — a V2G trace is INFO-level and enormous, while an
 * INFO line from the relay path is rare and always wanted.
 *
 * CORE is everything that predates categorisation: every existing cbplc_log/cbplc_logf call
 * site. It defaults ON, so a board that is never configured behaves exactly as it always has.
 * V2G defaults OFF, because that is the firehose. */
#define CBPLC_DIAG_CAT_CORE  0x00000001u
#define CBPLC_DIAG_CAT_V2G   0x00000002u  /* per-message decoded HLC trace */
#define CBPLC_DIAG_CAT_SLAC  0x00000004u
#define CBPLC_DIAG_CAT_CP    0x00000008u
#define CBPLC_DIAG_CAT_RELAY 0x00000010u
#define CBPLC_DIAG_CAT_ALL   0xFFFFFFFFu
#define CBPLC_DIAG_CAT_DEFAULT (CBPLC_DIAG_CAT_CORE | CBPLC_DIAG_CAT_SLAC | \
                                CBPLC_DIAG_CAT_CP | CBPLC_DIAG_CAT_RELAY)

/* The control reply. Fixed size so length alone is most of the discrimination. */
#define CBPLC_DIAG_REPLY_SIZE 16u
#define CBPLC_DIAG_REPLY_MAGIC 0xDBu
#define CBPLC_DIAG_REPLY_VERSION 1u

/* Longest control REQUEST: subcommand + u32 category mask. */
#define CBPLC_DIAG_REQ_MAX 5u

typedef struct {
    uint8_t  level;        /* CBPLC_DIAG_LEVEL_* currently in force */
    uint8_t  status;       /* CBPLC_E_OK, or why the request was refused */
    uint32_t categories;   /* bitmask currently in force */
    uint32_t dropped;      /* bytes the ring has overwritten since boot — silence vs loss */
    uint32_t pending;      /* bytes waiting to be drained right now */
} cbplc_diag_state_t;

/* ---- request ---------------------------------------------------------------------------- */

/* Encode a control request. Returns bytes written, or 0 if the arguments are not encodable —
 * an unknown subcommand or an out-of-range level is refused HERE rather than being put on the
 * wire for the board to reject, because a caller cannot fix what it never learns about. */
size_t cbplc_diag_encode_request(cbplc_diag_cmd_t cmd, uint32_t arg, uint8_t *out, size_t cap);

/* Decode a control request. An EMPTY payload decodes as DRAIN with arg 0, which is the
 * backward-compatible reading and must stay that way. Returns CBPLC_E_BAD_ARGUMENT for a
 * malformed or unknown request; the caller must then leave existing settings ALONE, because a
 * corrupt frame may neither enable nor silence logging. */
cbplc_err_t cbplc_diag_decode_request(const uint8_t *payload, uint16_t len,
                                      cbplc_diag_cmd_t *cmd, uint32_t *arg);

/* ---- reply ------------------------------------------------------------------------------ */

/* Encode the control reply. Always writes exactly CBPLC_DIAG_REPLY_SIZE bytes when cap allows,
 * and 0 otherwise — a short reply is not a reply, and accepting one would let a truncated frame
 * masquerade as a state report. */
size_t cbplc_diag_encode_reply(const cbplc_diag_state_t *st, uint8_t *out, size_t cap);

/* True only if this payload IS a control reply. Everything else — including log text of any
 * length, and a truncated control reply — is not one, and the caller should treat the payload
 * as drained log bytes. */
bool cbplc_diag_is_reply(const uint8_t *payload, uint16_t len);

/* Decode a control reply. Returns false unless cbplc_diag_is_reply() would have returned true,
 * so a caller that ignores the discriminator still cannot read text as state. */
bool cbplc_diag_decode_reply(const uint8_t *payload, uint16_t len, cbplc_diag_state_t *st);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_DIAG_H */
