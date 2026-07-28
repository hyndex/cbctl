/* Transport adapter: the only place that touches hardware.
 *
 * A transport is a struct of function pointers. There is no base class, no
 * registry, no init order, and nothing to register with.
 *
 * WHY FUNCTION POINTERS AND NOT INHERITANCE
 *   Four real adapters exist (loopback, UART, USB-CDC, CAN) and a test needs to
 *   substitute a fake that drops and reorders frames. A struct of pointers gives
 *   that for free, is trivially faked in a test, and has no vtable or
 *   construction order to get wrong. A class hierarchy would add ceremony and
 *   buy nothing here.
 *
 * THE CONTRACT — three properties, and no more
 *   1. `mtu` is the largest frame this link can carry, and the segment layer
 *      obeys it. The transport reports it; nothing guesses. A CAN adapter says 8,
 *      a UART adapter can say whatever its buffer holds.
 *   2. `send` and `recv` are NON-BLOCKING. They move what they can and return.
 *      Returning CBPLC_E_TIMEOUT with nothing moved is normal and not an error
 *      condition — it is how an idle link looks.
 *   3. Frame boundaries are the transport's problem. A packet link (CAN) has them
 *      naturally; a stream link (UART/USB) must add its own delimiting. The layers
 *      above are given whole frames or nothing.
 *
 * WHAT A TRANSPORT MUST NOT DO
 *   No retries, no reassembly, no CRC, no sequencing, no logging of protocol
 *   state. Those belong to exactly one layer each, above. A transport that
 *   retries silently makes the session layer's deadline accounting wrong, and a
 *   transport that checks a CRC creates a second integrity authority that will
 *   eventually disagree with the first.
 *
 * WHY recv REPORTS ELAPSED TIME
 *   On Linux a `read()` can block for far longer than asked, and scheduler jitter
 *   is real. Rather than assume promptness, the adapter reports how long it
 *   actually took, and the session layer charges that against the deadline. This
 *   is what makes the same timing code correct on a bare-metal MCU and on a
 *   loaded Raspberry Pi.
 */
#ifndef CBPLC_TRANSPORT_H
#define CBPLC_TRANSPORT_H

#include <stddef.h>
#include <stdint.h>

#include "cbplc/protocol_generated.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cbplc_transport cbplc_transport_t;

struct cbplc_transport {
    /* Opaque adapter state. The core never dereferences it. */
    void *ctx;

    /* Largest frame this link carries, >= CBPLC_MIN_TRANSPORT_MTU. */
    size_t mtu;

    /* Send one frame. CBPLC_E_OK if it left, CBPLC_E_TIMEOUT if the link is
     * momentarily full (caller retries later), CBPLC_E_TRANSPORT if the link is
     * broken. Must not block. */
    cbplc_err_t (*send)(void *ctx, const uint8_t *frame, size_t len);

    /* Receive at most one frame. Sets *len to 0 and returns CBPLC_E_TIMEOUT when
     * nothing is waiting — the normal idle case. Never blocks. */
    cbplc_err_t (*recv)(void *ctx, uint8_t *frame, size_t cap, size_t *len);
};

/* True if this adapter satisfies the contract well enough to use. Checked once at
 * session open so a mis-wired adapter fails loudly at startup rather than
 * mysteriously under load. */
cbplc_err_t cbplc_transport_validate(const cbplc_transport_t *t);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_TRANSPORT_H */
