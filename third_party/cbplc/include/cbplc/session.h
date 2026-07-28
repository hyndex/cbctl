/* Session layer: the public Controller API.
 *
 * This is the whole application-facing surface. It is identical over UART, USB
 * and CAN — application code cannot tell which transport it has, and that is the
 * point.
 *
 * WHAT THIS OWNS, EXCLUSIVELY
 *   Request/response pairing, sequencing, retries and deadlines. Each of those
 *   has exactly ONE owner in cbplc and this is it. Nothing above re-pairs
 *   responses; nothing below retries.
 *
 * HOW TIME WORKS HERE — the part that matters most for portability
 *   Every call takes an explicit deadline in milliseconds. There is deliberately
 *   no "wait forever" variant to reach for, because one of those in a control loop
 *   is a hung charger.
 *
 *   The core POLLS. It never sleeps, spins, or owns a thread. `cbplc_poll()` does
 *   one bounded unit of work and returns; the caller decides when to call it
 *   again. On an MCU that is a task loop, on Linux it is whatever the supervisor
 *   already has. This is why the same code runs under FreeRTOS and under systemd
 *   without a threading abstraction.
 *
 *   Nothing assumes the poll is prompt. A late poll observes a passed deadline and
 *   reports it. That is the whole accommodation for OS jitter, and it is enough.
 *
 * SEQUENCE SAFETY
 *   `request_id` is allocated here, monotonically, skipping 0 (0 means
 *   "unsolicited event, never awaited"). A response whose id matches no
 *   outstanding request is dropped and counted, not guessed at. A retry reuses its
 *   id, so a duplicated response is detectable rather than being mistaken for an
 *   answer to the next question.
 *
 * WHAT THIS DOES NOT DO
 *   No allocation. No thread safety (single-owner by contract — call it from one
 *   task). No hidden state machine for charging: that is the application's job,
 *   and burying it here is how plc_firmware ended up with a 32,000-line file.
 */
#ifndef CBPLC_SESSION_H
#define CBPLC_SESSION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cbplc/message.h"
#include "cbplc/segment.h"
#include "cbplc/transport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Timing is configuration, not code.
 *
 * plc_firmware hard-coded limits that turned out to be wire artefacts (a 25 500 ms
 * ceiling because 255 x 100 ms is what one byte held) and then silently overrode
 * the controller's deliberately-chosen values with them. Every number here is
 * named, defaulted from the schema, and overridable at runtime — because a loaded
 * Raspberry Pi and a bare-metal MCU do not deserve the same numbers, and
 * hard-coding either makes the other wrong. */
typedef struct {
    uint32_t request_timeout_ms;
    uint32_t retry_backoff_ms;
    uint32_t reassembly_timeout_ms;
    uint8_t retry_limit;
} cbplc_timing_t;

/* Fills `t` with the schema defaults. Start here, then override what a platform
 * actually needs, so a new field never silently defaults to zero. */
void cbplc_timing_defaults(cbplc_timing_t *t);

/* Delivered to the application for anything not awaited: peer events, and the
 * final outcome of a request. Returning from this callback must be cheap — it is
 * called from inside cbplc_poll(). */
typedef void (*cbplc_on_message_fn)(void *user,
                                    const cbplc_msg_t *msg,
                                    cbplc_err_t status);

typedef struct {
    const cbplc_transport_t *tx;
    cbplc_timing_t timing;
    cbplc_on_message_fn on_message;
    void *user;

    /* --- internal below: read for diagnostics, never write --------------- */
    cbplc_joiner_t joiner;
    /* Replies that could not be staged because a send was already in flight. Should stay
     * zero: cbplc_poll() stops consuming requests once the slot is busy. Non-zero means a
     * peer's request was answered with silence. */
    uint32_t responses_refused;
    uint8_t out_msg[CBPLC_MAX_MESSAGE];
    size_t out_len;
    cbplc_splitter_t split;
    bool sending;
    /* When the current outbound message started. A transmit that cannot make
     * progress must eventually give up: without this, a transport that reports "link
     * momentarily full" forever leaves `sending` latched true and the session never
     * sends anything again. Measured on hardware — a reply larger than the USB TX
     * ring could never be accepted, and every later message queued behind it. */
    uint32_t sending_since;
    /* The most recent time the application handed us. Every entry point that has a
     * clock records it here so the ones that do not — a reply staged from inside the
     * receive callback — can still be given a deadline, without threading a
     * timestamp through an API where the caller has nothing sensible to pass. */
    uint32_t last_now_ms;

    uint16_t next_request_id;
    uint16_t pending_id;      /* 0 = nothing outstanding */
    uint32_t pending_since;
    uint8_t attempts;
    uint8_t next_message_id;

    /* Counters that can all actually move. */
    uint32_t sent;
    uint32_t received;
    uint32_t retries;
    uint32_t timeouts;
    uint32_t dropped_unmatched;
    uint32_t requests_in;   /* inbound requests from the peer (server role) */
    /* Outbound messages abandoned because the transport never accepted them. A
     * non-zero value here means the link is congested or the peer is not reading —
     * distinct from `timeouts`, which counts unanswered requests. */
    uint32_t send_abandoned;
} cbplc_session_t;

/* Open a session over `tx`. Validates the adapter immediately so a mis-wired
 * transport fails at startup, not mysteriously under load. `timing` may be NULL
 * to take the schema defaults. */
cbplc_err_t cbplc_open(cbplc_session_t *s,
                       const cbplc_transport_t *tx,
                       const cbplc_timing_t *timing,
                       cbplc_on_message_fn on_message,
                       void *user);

/* Reply to a request the PEER sent us. `request_id` must be the id from the
 * inbound message, which is what lets the peer correlate our answer.
 *
 * This exists because a PLC is a SERVER: the controller sends it requests. It is
 * NOT stop-and-wait-gated, because answering an inbound request has nothing to do
 * with whether we happen to have a request of our own outstanding — coupling them
 * would let our own traffic block our replies. */
cbplc_err_t cbplc_respond(cbplc_session_t *s,
                          cbplc_type_t type,
                          uint16_t request_id,
                          const uint8_t *payload,
                          uint16_t payload_len);

/* Queue a request. Returns the allocated request id via `out_id`.
 *
 * Stop-and-wait by design: one outstanding request at a time, so ordering needs
 * no reorder buffer and a duplicate response is unambiguous. Returns
 * CBPLC_E_NOT_READY if one is already outstanding — an explicit, checkable result
 * rather than a queue that can silently grow.
 *
 * Fire-and-forget: pass NULL for out_id to send an event that is never awaited. */
cbplc_err_t cbplc_request(cbplc_session_t *s,
                          cbplc_type_t type,
                          const uint8_t *payload,
                          uint16_t payload_len,
                          uint32_t now_ms,
                          uint16_t *out_id);

/* Do one bounded unit of work: push at most one outbound frame, take at most one
 * inbound frame, and check the deadline. Call it as often as you like; calling it
 * rarely costs latency, never correctness. */
cbplc_err_t cbplc_poll(cbplc_session_t *s, uint32_t now_ms);

/* True while a request is outstanding. */
bool cbplc_busy(const cbplc_session_t *s);

#ifdef __cplusplus
}
#endif
#endif /* CBPLC_SESSION_H */
