/* Message layer: typed struct <-> canonical bytes. Pure functions.
 *
 * WHAT THIS OWNS
 *   The only place in cbplc that knows the byte layout of a message. Encode and
 *   decode are exact inverses and neither allocates, blocks, or logs.
 *
 * WHAT IT DOES NOT OWN
 *   Transport, segmentation, retries, sequencing, time. A message does not know
 *   how it will travel, and that is what makes the same message work over UART
 *   and over 8-byte CAN frames without a second representation.
 *
 * WHY THERE IS EXACTLY ONE ENCODING
 *   plc_firmware re-encoded every command on its way in: text -> tokens -> an
 *   8-byte internal frame -> CRC-8 -> ring -> back out again, all inside one
 *   task. That second encoding quantised every duration to 100 ms and truncated
 *   it to one byte, so a 45 000 ms lease silently became 25 500 ms. The lesson is
 *   not "be careful with encodings" but "have one". If you find yourself packing
 *   a decoded message into another layout, that is a bug.
 *
 * VALIDATION HAPPENS HERE, ONCE
 *   Version, type, length, reserved bits and CRC-32 are checked at decode. Code
 *   downstream trusts the result. Re-validating inward is how you end up with two
 *   authorities that disagree, and the narrower one always wraps first.
 */
#ifndef CBPLC_MESSAGE_H
#define CBPLC_MESSAGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A decoded message. `payload` points into the caller's buffer — decode does not
 * copy, because on an MCU the copy is the expensive part and the caller already
 * owns a buffer that outlives the call. */
typedef struct {
    cbplc_type_t type;
    uint16_t request_id;   /* 0 = unsolicited event; never awaited */
    /* True when the RESPONSE flag is set, i.e. this is a reply to a request WE
     * sent. False with a non-zero request_id means the PEER is asking US
     * something — a PLC is a server, so that is the common case.
     *
     * request_id alone cannot express this, and a session that guesses is wrong
     * either way. Found on hardware: a correlatable PING from the peer was
     * silently dropped as an unmatched response. */
    bool is_response;
    const uint8_t *payload;
    uint16_t payload_len;
} cbplc_msg_t;

/* CRC-32 (IEEE 802.3, reflected, init 0xFFFFFFFF, final xor 0xFFFFFFFF).
 * Exposed because the Python harness must compute the identical value and a
 * test asserts they agree on a known vector. */
uint32_t cbplc_crc32(const uint8_t *data, size_t len);

/* Encode into `out`. Returns CBPLC_E_SHORT_BUFFER if `out_cap` is too small and
 * writes nothing. `*out_len` is set only on success. */
cbplc_err_t cbplc_msg_encode(cbplc_type_t type,
                             uint16_t request_id,
                             bool is_response,
                             const uint8_t *payload,
                             uint16_t payload_len,
                             uint8_t *out,
                             size_t out_cap,
                             size_t *out_len);

/* Decode `in`. On success `msg->payload` aliases `in`, so `in` must outlive
 * `msg`. Distinct errors for each failure mode: a caller that wants to know
 * whether it is talking to the wrong protocol version or merely seeing line
 * noise can tell the difference. */
cbplc_err_t cbplc_msg_decode(const uint8_t *in, size_t in_len, cbplc_msg_t *msg);

/* True if `t` is a type this build knows. Kept separate from decode so a peer
 * can be told "unsupported operation" rather than "malformed", which is the
 * difference between "upgrade me" and "your link is broken". */
bool cbplc_type_known(uint8_t t);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_MESSAGE_H */
