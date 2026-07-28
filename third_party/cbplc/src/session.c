#include "cbplc/session.h"

#include <string.h>

/* Wrap-safe. See segment.c — never compare absolute 32-bit ms stamps. */
static bool elapsed_at_least(uint32_t now_ms, uint32_t since_ms, uint32_t ms)
{
    return (uint32_t)(now_ms - since_ms) >= ms;
}

void cbplc_timing_defaults(cbplc_timing_t *t)
{
    if (t == NULL) {
        return;
    }
    t->request_timeout_ms = CBPLC_DEF_REQUEST_TIMEOUT_MS;
    t->retry_backoff_ms = CBPLC_DEF_RETRY_BACKOFF_MS;
    t->reassembly_timeout_ms = CBPLC_REASSEMBLY_TIMEOUT_MS;
    t->retry_limit = (uint8_t)CBPLC_DEF_RETRY_LIMIT;
}

cbplc_err_t cbplc_transport_validate(const cbplc_transport_t *t)
{
    if (t == NULL || t->send == NULL || t->recv == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* An MTU below the floor cannot carry a segment header plus a payload byte,
     * so the segment layer could never make progress. Fail at open() rather than
     * discovering it on the first send. */
    if (t->mtu < CBPLC_MIN_TRANSPORT_MTU) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* A message must be reachable at this MTU at all. */
    if (cbplc_seg_count(CBPLC_MAX_MESSAGE, t->mtu) == 0u) {
        return CBPLC_E_NO_SPACE;
    }
    return CBPLC_E_OK;
}

cbplc_err_t cbplc_open(cbplc_session_t *s,
                       const cbplc_transport_t *tx,
                       const cbplc_timing_t *timing,
                       cbplc_on_message_fn on_message,
                       void *user)
{
    if (s == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    const cbplc_err_t rc = cbplc_transport_validate(tx);
    if (rc != CBPLC_E_OK) {
        return rc;
    }
    memset(s, 0, sizeof *s);
    s->tx = tx;
    s->on_message = on_message;
    s->user = user;
    if (timing != NULL) {
        s->timing = *timing;
    } else {
        cbplc_timing_defaults(&s->timing);
    }
    cbplc_join_reset(&s->joiner);
    s->next_request_id = 1u; /* 0 is reserved for unsolicited events */
    return CBPLC_E_OK;
}

bool cbplc_busy(const cbplc_session_t *s)
{
    return s != NULL && (s->pending_id != 0u || s->sending);
}

static uint16_t alloc_request_id(cbplc_session_t *s)
{
    uint16_t id = s->next_request_id++;
    if (s->next_request_id == 0u) {
        s->next_request_id = 1u; /* skip 0 on wrap: it means "not a reply" */
    }
    return id;
}

/* Stage a message for transmission. Encoding happens once, here, and the splitter
 * then walks the SAME bytes — a retry re-sends identical bytes rather than
 * re-encoding and risking a different result. */
static cbplc_err_t stage(cbplc_session_t *s,
                         cbplc_type_t type,
                         uint16_t request_id,
                         bool is_response,
                         const uint8_t *payload,
                         uint16_t payload_len)
{
    const cbplc_err_t rc = cbplc_msg_encode(type, request_id, is_response,
                                            payload, payload_len, s->out_msg,
                                            sizeof s->out_msg, &s->out_len);
    if (rc != CBPLC_E_OK) {
        return rc;
    }
    const cbplc_err_t sc = cbplc_split_begin(&s->split, s->out_msg, s->out_len,
                                             s->tx->mtu, s->next_message_id++);
    if (sc != CBPLC_E_OK) {
        return sc;
    }
    s->sending = true;
    s->sending_since = s->last_now_ms;
    return CBPLC_E_OK;
}

cbplc_err_t cbplc_request(cbplc_session_t *s,
                          cbplc_type_t type,
                          const uint8_t *payload,
                          uint16_t payload_len,
                          uint32_t now_ms,
                          uint16_t *out_id)
{
    if (s == NULL || s->tx == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    s->last_now_ms = now_ms;
    /* Stop-and-wait: refuse rather than queue. An explicit NOT_READY the caller
     * must handle beats a queue that can grow without anyone noticing. */
    if (cbplc_busy(s)) {
        return CBPLC_E_NOT_READY;
    }

    const uint16_t id = (out_id != NULL) ? alloc_request_id(s) : 0u;
    const cbplc_err_t rc = stage(s, type, id, false, payload, payload_len);
    if (rc != CBPLC_E_OK) {
        s->sending = false;
        return rc;
    }
    if (out_id != NULL) {
        s->pending_id = id;
        s->pending_since = now_ms;
        s->attempts = 1u;
        *out_id = id;
    }
    return CBPLC_E_OK;
}

/* Re-send the already-staged bytes. The request id is unchanged, which is what
 * makes a duplicated response detectable instead of being mistaken for the answer
 * to the next question. */
static void restage_for_retry(cbplc_session_t *s, uint32_t now_ms)
{
    /* Re-send the ALREADY-ENCODED bytes: identical content, identical id, and
     * identical flags. Nothing is re-encoded, so a retry cannot differ. */
    (void)cbplc_split_begin(&s->split, s->out_msg, s->out_len, s->tx->mtu,
                            s->next_message_id++);
    s->sending = true;
    s->pending_since = now_ms;
    s->attempts++;
    s->retries++;
}

static void deliver(cbplc_session_t *s, const cbplc_msg_t *m, cbplc_err_t st)
{
    if (s->on_message != NULL) {
        s->on_message(s->user, m, st);
    }
}

cbplc_err_t cbplc_respond(cbplc_session_t *s,
                          cbplc_type_t type,
                          uint16_t request_id,
                          const uint8_t *payload,
                          uint16_t payload_len)
{
    if (s == NULL || s->tx == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* A response must be correlatable, so id 0 is refused — that value means
     * "unsolicited event" and would leave the peer unable to match the answer. */
    if (request_id == 0u) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    /* Only the transmitter is a shared resource, not the request slot: a reply is
     * not gated on our own outstanding request. */
    if (s->sending) {
        /* Counted, because this was silent. Every caller discards the return, so a refused
         * reply looked exactly like a delivered one from the application's side. A non-zero
         * value in STATUS now says "a reply was dropped" — and with the intake gate in
         * cbplc_poll() it should stay at zero. */
        s->responses_refused++;
        return CBPLC_E_NOT_READY;
    }
    const cbplc_err_t rc = stage(s, type, request_id, true, payload, payload_len);
    if (rc != CBPLC_E_OK) {
        s->sending = false;
    }
    return rc;
}

cbplc_err_t cbplc_poll(cbplc_session_t *s, uint32_t now_ms)
{
    if (s == NULL || s->tx == NULL) {
        return CBPLC_E_BAD_ARGUMENT;
    }
    s->last_now_ms = now_ms;

    /* 1. Push at most one outbound frame. One per poll keeps a large message from
     *    monopolising a shared bus and keeps this function's cost bounded. */
    if (s->sending && !cbplc_split_done(&s->split)) {
        uint8_t frame[CBPLC_MAX_MESSAGE + CBPLC_SEG_HEADER_SIZE];
        size_t fl = 0;
        const cbplc_err_t pc = cbplc_split_next(&s->split, frame, sizeof frame, &fl);
        if (pc == CBPLC_E_OK) {
            const cbplc_err_t sc = s->tx->send(s->tx->ctx, frame, fl);
            if (sc == CBPLC_E_TRANSPORT) {
                s->sending = false;
                if (s->pending_id != 0u) {
                    s->pending_id = 0u;
                    deliver(s, NULL, CBPLC_E_TRANSPORT);
                }
                return CBPLC_E_TRANSPORT;
            }
            if (sc == CBPLC_E_TIMEOUT) {
                /* "The link is momentarily full." The frame was already consumed
                 * from the splitter, so rewind by one and retry next poll.
                 *
                 * BOUNDED, and that bound is the whole point. An unbounded retry
                 * here is a LIVELOCK: a transport that can never accept this frame
                 * — because it is larger than the peripheral's TX ring, or because
                 * the peer stopped reading — leaves `sending` latched true forever,
                 * and every later message silently queues behind a reply that will
                 * never leave. That is not hypothetical: it wedged the link on real
                 * hardware, and the session looked healthy the whole time because
                 * nothing counted it. Abandon the message, say so, move on. */
                if (elapsed_at_least(now_ms, s->sending_since,
                                    s->timing.request_timeout_ms)) {
                    s->sending = false;
                    s->send_abandoned++;
                    if (s->pending_id != 0u) {
                        s->pending_id = 0u;
                        deliver(s, NULL, CBPLC_E_TIMEOUT);
                    }
                    return CBPLC_E_TIMEOUT;
                }
                /* Never rewind past the start: an unsigned underflow here would
                 * turn a full link into a wild read. */
                if (s->split.next > 0u) {
                    s->split.next--;
                }
            } else if (cbplc_split_done(&s->split)) {
                s->sending = false;
                s->sent++;
            }
        } else {
            /* The splitter refused to produce a frame at all, so no amount of
             * retrying will help. Abandoning is the only outcome that cannot hang. */
            s->sending = false;
            s->send_abandoned++;
            if (s->pending_id != 0u) {
                s->pending_id = 0u;
                deliver(s, NULL, pc);
            }
            return pc;
        }
    }

    /* 2. Take inbound frames — up to a BOUND, not exactly one.
     *
     * One per poll tied the inbound rate to the loop rate, so a peer that sent two
     * messages back to back had the second sitting in the transport until the next pass.
     * Combined with a 64-byte read throttle that was enough to lose it entirely.
     *
     * The bound still exists: this loop shares its task with the control-pilot sampling
     * and the relay dead-man, and an unbounded drain would let a chatty peer starve them.
     * Eight is several whole messages and still a tiny slice of a poll. */
    for (int taken = 0; taken < 8; ++taken) {
        /* KEEP DRAINING THE TRANSPORT EVEN WHILE A REPLY IS IN FLIGHT.
         *
         * I briefly did the opposite — `if (s->sending) break;` — to avoid consuming a
         * request whose reply could not be staged. It read well and it was wrong, measured on
         * a real vehicle: not reading the CDC while sending applies BACKPRESSURE TO THE HOST,
         * whose write() then blocks, which stalls the controller feed thread that shares the
         * port. The feed went quiet for ten seconds (meas_age climbed 328 -> 10202 ms), the
         * authorisation lease lapsed, and a healthy charging session died with
         * Stopping:auth_withdrawn. A dropped ACK is an inconvenience the peer retries; a
         * starved controller feed stops the charge.
         *
         * So: always drain. A request that cannot be answered right now is counted in
         * responses_refused and the peer's own retry covers it — cbplc_respond()'s callers
         * are all idempotent and the harness retries three times. The 8-frame bound below
         * still protects the pilot sampler and the dead-man from a chatty peer.
         *
         * The general rule this cost me: on a shared full-duplex link, never make progress in
         * one direction depend on completing the other. */
        uint8_t frame[CBPLC_MAX_MESSAGE + CBPLC_SEG_HEADER_SIZE];
        size_t fl = 0;
        const cbplc_err_t rc = s->tx->recv(s->tx->ctx, frame, sizeof frame, &fl);
        if (rc == CBPLC_E_TRANSPORT) {
            return CBPLC_E_TRANSPORT;
        }
        if (rc != CBPLC_E_OK || fl == 0u) {
            break; /* nothing more waiting */
        }
        {
            const cbplc_rx_t st = cbplc_join_feed(&s->joiner, frame, fl, now_ms,
                                                 s->timing.reassembly_timeout_ms);
            if (st == CBPLC_RX_COMPLETE) {
                cbplc_msg_t m;
                const cbplc_err_t dc = cbplc_msg_decode(s->joiner.buf,
                                                        s->joiner.len, &m);
                if (dc != CBPLC_E_OK) {
                    /* A bad message is reported, never silently dropped: the peer
                     * needs to learn its frames are not arriving intact. */
                    deliver(s, NULL, dc);
                } else {
                    s->received++;
                    /* THREE cases, distinguished by the RESPONSE flag rather than
                     * guessed from the id. Before the flag existed this had only
                     * two, and an inbound REQUEST from the peer fell into the
                     * "unmatched response" bucket and was silently dropped —
                     * found on hardware, where a correlatable PING got no reply. */
                    if (m.request_id == 0u) {
                        deliver(s, &m, CBPLC_E_OK); /* unsolicited event */
                    } else if (!m.is_response) {
                        /* The peer is asking US something. A PLC is a server, so
                         * this is the common case. The application replies with
                         * cbplc_respond() using this same id. */
                        s->requests_in++;
                        deliver(s, &m, CBPLC_E_OK);
                    } else if (m.request_id == s->pending_id) {
                        s->pending_id = 0u;
                        deliver(s, &m, CBPLC_E_OK);
                    } else {
                        /* A response matching no outstanding request: a late reply
                         * to a timed-out request, or a duplicate. Counted, never
                         * guessed at — guessing is how a stale answer authorises a
                         * fresh action. This safety property is why the flag was
                         * added instead of simply delivering everything. */
                        s->dropped_unmatched++;
                    }
                }
            }
        }
    }

    /* 3. Deadline. Checked last so a response that arrived in this same poll wins
     *    over a timeout that would otherwise fire in the same millisecond. */
    if (s->pending_id != 0u && !s->sending &&
        elapsed_at_least(now_ms, s->pending_since, s->timing.request_timeout_ms)) {
        if (s->attempts <= s->timing.retry_limit) {
            restage_for_retry(s, now_ms);
            return CBPLC_E_OK;
        }
        s->pending_id = 0u;
        s->timeouts++;
        deliver(s, NULL, CBPLC_E_TIMEOUT);
        return CBPLC_E_TIMEOUT;
    }
    return CBPLC_E_OK;
}
