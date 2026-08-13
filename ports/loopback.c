#include "loopback.h"

#include "cbctl/cbctl.h"
#include "cbplc/diag.h"
#include "cbplc/provision.h"

#include "cbplc/limits.h"

#include <string.h>

#include "cbplc/message.h"

#include <stdio.h>

/* ---- byte queues ------------------------------------------------------------------------ */

static bool q_push(cbctl_lb_queue_t *q, const uint8_t *b, size_t n)
{
    const unsigned next = (q->head + 1u) % CBCTL_LB_QUEUE;
    if (n > CBCTL_LB_FRAME || next == q->tail) {
        q->dropped++;
        return false;
    }
    memcpy(q->f[q->head].buf, b, n);
    q->f[q->head].len = (uint16_t)n;
    q->head = next;
    return true;
}

static bool q_pop(cbctl_lb_queue_t *q, uint8_t *out, size_t cap, size_t *len)
{
    if (q->tail == q->head) {
        return false;
    }
    if (q->f[q->tail].len > cap) {
        q->tail = (q->tail + 1u) % CBCTL_LB_QUEUE; /* undeliverable; drop rather than lie */
        q->dropped++;
        return false;
    }
    memcpy(out, q->f[q->tail].buf, q->f[q->tail].len);
    *len = q->f[q->tail].len;
    q->tail = (q->tail + 1u) % CBCTL_LB_QUEUE;
    return true;
}

/* ---- transport adapters ------------------------------------------------------------------ */
/* Both honour cbplc's contract to the letter: send MUST NOT block, recv NEVER blocks and
 * reports "nothing waiting" as CBPLC_E_TIMEOUT rather than as an error. */

static cbplc_err_t ctrl_send(void *ctx, const uint8_t *frame, size_t len)
{
    cbctl_loopback_t *lb = (cbctl_loopback_t *)ctx;
    if (lb->transport_dead) {
        return CBPLC_E_TRANSPORT; /* the device is gone, not merely busy */
    }
    if (lb->tx_full) {
        return CBPLC_E_TIMEOUT; /* momentarily full: the caller retries later, never waits */
    }
    lb->ctrl_frames_in++;
    return q_push(&lb->to_plc, frame, len) ? CBPLC_E_OK : CBPLC_E_TIMEOUT;
}

static cbplc_err_t ctrl_recv(void *ctx, uint8_t *frame, size_t cap, size_t *len)
{
    cbctl_loopback_t *lb = (cbctl_loopback_t *)ctx;
    *len = 0u;
    if (lb->transport_dead) {
        return CBPLC_E_TRANSPORT;
    }
    return q_pop(&lb->to_ctrl, frame, cap, len) ? CBPLC_E_OK : CBPLC_E_TIMEOUT;
}

static cbplc_err_t plc_send(void *ctx, const uint8_t *frame, size_t len)
{
    cbctl_loopback_t *lb = (cbctl_loopback_t *)ctx;
    if (lb->wedged) {
        return CBPLC_E_OK; /* swallowed: the board is not talking, but the wire is fine */
    }
    return q_push(&lb->to_ctrl, frame, len) ? CBPLC_E_OK : CBPLC_E_TIMEOUT;
}

static cbplc_err_t plc_recv(void *ctx, uint8_t *frame, size_t cap, size_t *len)
{
    cbctl_loopback_t *lb = (cbctl_loopback_t *)ctx;
    *len = 0u;
    if (lb->wedged) {
        return CBPLC_E_TIMEOUT; /* a hung board stops reading too */
    }
    if (!q_pop(&lb->to_plc, frame, cap, len)) {
        return CBPLC_E_TIMEOUT;
    }
    lb->plc_frames_in++;
    return CBPLC_E_OK;
}

/* ---- the fake PLC's behaviour -------------------------------------------------------------- */

static void plc_on_message(void *user, const cbplc_msg_t *msg, cbplc_err_t status)
{
    cbctl_loopback_t *lb = (cbctl_loopback_t *)user;
    uint8_t ack[1];

    if (lb == NULL || msg == NULL || status != CBPLC_E_OK) {
        return;
    }

    if (lb->swallow_type != 0u && msg->type == (cbplc_type_t)lb->swallow_type) {
        return; /* received, never answered: the caller now pays the full retry sequence */
    }

    switch (msg->type) {
    case CBPLC_T_IDENTIFY: {
        /* A real 20-byte IDENTITY: mac[6] wire reset up_ms[4] build[8]. */
        uint8_t idp[20];
        memset(idp, 0, sizeof idp);
        memcpy(idp, lb->board_mac, 6u);
        idp[6] = 1u;                 /* wire version */
        idp[7] = 0u;                 /* reset reason */
        idp[8] = (uint8_t)(lb->up_ms & 0xFFu);
        idp[9] = (uint8_t)((lb->up_ms >> 8) & 0xFFu);
        idp[10] = (uint8_t)((lb->up_ms >> 16) & 0xFFu);
        idp[11] = (uint8_t)((lb->up_ms >> 24) & 0xFFu);
        memcpy(&idp[12], "62a77ad2", 8u);
        lb->identify_seen++;
        (void)cbplc_respond(&lb->plc, CBPLC_T_IDENTITY, msg->request_id, idp, 20u);
        return;
    }
    case CBPLC_T_STATUS_REQUEST: {
        /* dc_ans rises only while the simulated EV is actually sending DC requests — the same
         * property the real board has, and the one cbctl uses to tell a live demand from the
         * stale value cbplc keeps echoing between sessions. */
        if (lb->ev_tgt_mv > 0u && !lb->ev_demand_idle) {
            lb->dc_answers++;
        }
        /* The real board emits ~165 keys; the fake emits the ones a controller acts on, in
         * the same space-separated key=value shape. Keeping the SHAPE honest is what makes
         * the parser test meaningful — including neighbours whose names share a tail, so a
         * substring match would be caught. */
        /* The EVCCID, rendered to the length the fake was told to report — the same
         * length-driven shape the firmware emits, because a fixed-width fake would let a
         * consumer that assumes 6 bytes pass a test the real DIN wire would fail. */
        char evcc_hex[2u * sizeof lb->evcc_id + 1u];
        {
            unsigned el = lb->evcc_id_len;
            if (el > sizeof lb->evcc_id) { el = (unsigned)sizeof lb->evcc_id; }
            for (unsigned i = 0u; i < el; ++i) {
                snprintf(evcc_hex + 2u * i, 3u, "%02X", lb->evcc_id[i]);
            }
            evcc_hex[2u * el] = '\0';
        }
        char safety_kv[40];
        if (lb->safety_raw_mode) {
            snprintf(safety_kv, sizeof safety_kv, "safety_in=0x%02X safety_bus=%d",
                     lb->safety_raw_mask, lb->safety_bus_unreadable ? 1 : 0);
        } else {
            snprintf(safety_kv, sizeof safety_kv, "safety=%d", lb->safety_asserted ? 1 : 0);
        }
        char body[448];
        const int n = snprintf(body, sizeof body,
            "cp=%c slac_matched=%d secc_active=%d secc_streams=%lu last_stop=1 stop=%d "
            "evcc_len=%u evcc=%s "
            "ev_mac=%02X%02X%02X%02X%02X%02X ev_mac_ok=%d up_ms=%lu "
            "relay_safety_opens=%lu relay_lease_opens=%lu gun_open_timeouts=%lu "
            "%s relay_closed=0x%02X relay_drifted=%d relay_rb=0x%02X ev_soc=%u ev_ready=%d ev_pause=%d "
            "ev_tgt_mv=%lu ev_tgt_ma=%lu dc_ans=%lu "
            /* The TAIL KEY must match the firmware's real last field, or every parse here
             * reads as truncated — which is the truncation guard working, and exactly what
             * caught this when the firmware's STATUS grew. Keep it last. */
            "status_overflow=0 b1_dwell=1500 " CBCTL_STATUS_TAIL_KEY "=0",
            lb->cp ? lb->cp : 'A', lb->slac_matched ? 1 : 0, lb->secc_active ? 1 : 0,
            (unsigned long)lb->secc_streams, lb->stop_latched ? 1 : 0,
            (unsigned)lb->evcc_id_len, evcc_hex,
            lb->ev_mac[0], lb->ev_mac[1], lb->ev_mac[2], lb->ev_mac[3], lb->ev_mac[4],
            lb->ev_mac[5], lb->ev_mac_ok ? 1 : 0, (unsigned long)lb->up_ms,
            (unsigned long)lb->relay_safety_opens, (unsigned long)lb->relay_lease_opens,
            (unsigned long)lb->gun_open_timeouts,
            safety_kv, lb->relay_closed_mask,
            lb->relay_drifted ? 1 : 0, lb->relay_drift_readback,
            (unsigned)lb->ev_soc, lb->ev_ready ? 1 : 0, lb->ev_paused ? 1 : 0,
            (unsigned long)lb->ev_tgt_mv, (unsigned long)lb->ev_tgt_ma,
            (unsigned long)lb->dc_answers);
        lb->status_seen++;
        /* Chop the tail, exactly as an over-long STATUS would: the keys are well-formed, the
         * line simply ends early and the sentinel is gone. */
        if (lb->truncate_status && n > 40) {
            body[40] = '\0';
        }
        if (n > 0) {
            (void)cbplc_respond(&lb->plc, CBPLC_T_STATUS, msg->request_id,
                                (const uint8_t *)body, (uint16_t)strlen(body));
        }
        return;
    }
    case CBPLC_T_STOP:
        /* MODEL THE LATCH, not just the acknowledgement. The real PLC holds a stop until a
         * zero byte clears it; a fake that merely ACKed would have let a library that never
         * sent STOP at all pass every test. */
        if (msg->payload_len >= 1u) {
            lb->stop_latched = (msg->payload[0] != 0u);
            lb->stop_seen++;
        }
        break;
    case CBPLC_T_RELAY:
        /* The board ACKs the close (below) and then its safety layer undoes it — which is
         * precisely why this is invisible to the controller without the counter. */
        if (lb->safety_undoes_closes && msg->payload_len >= 2u && msg->payload[1] != 0u) {
            lb->relay_safety_opens++;
        }
        break;
    case CBPLC_T_LIMITS:    lb->limits_seen++;    break;
    case CBPLC_T_FEEDBACK:
        /* Contactor is the last byte of the encoded measurement; decode rather than guess. */
        {
            cbplc_limits_t dl;
            cbplc_limits_init(&dl);
            if (cbplc_limits_apply_measurement(&dl, msg->payload, msg->payload_len, 1u)
                    == CBPLC_E_OK) {
                lb->last_feedback_contactor = (uint8_t)dl.measurement.contactor;
                lb->last_feedback_voltage_mv = dl.measurement.present_voltage_mv;
                lb->last_feedback_current_ma = dl.measurement.present_current_ma;
            }
        }
        lb->feedback_seen++;
        break;
    case CBPLC_T_AUTHORIZE:
        if (lb->first_auth_decision < 0 && msg->payload_len >= 1u) {
            lb->first_auth_decision = (int)msg->payload[0];
        }
        lb->auth_seen++;
        break;
    case CBPLC_T_HEARTBEAT: lb->heartbeat_seen++; break;

    case CBPLC_T_PROVISION: {
        if (lb->provision_unsupported) {
            /* A board built before provisioning existed. cbplc's default arm answers an
             * unknown type with BAD_TYPE rather than silence — "a feature gap, not a dead
             * link" — and that is exactly what a controller must be able to detect. */
            ack[0] = (uint8_t)CBPLC_E_BAD_TYPE;
        } else {
            const cbplc_err_t rc =
                cbplc_provision_apply(&lb->prov, msg->payload, msg->payload_len);
            ack[0] = (uint8_t)rc;
            lb->provision_seen++;
        }
        if (lb->provision_unsupported) {
            lb->provision_seen++;
        }
        (void)cbplc_respond(&lb->plc, CBPLC_T_ACK, msg->request_id, ack, 1u);
        return;
    }

    case CBPLC_T_RESTART: {
        lb->restart_seen++;
        ack[0] = lb->restart_refuse_code;
        /* Answered with its OWN type, not an ACK, so the gate verdict is unambiguous. */
        (void)cbplc_respond(&lb->plc, CBPLC_T_RESTART, msg->request_id, ack, 1u);
        return;
    }

    case CBPLC_T_DIAG: {
        cbplc_diag_cmd_t cmd = CBPLC_DIAG_DRAIN;
        uint32_t arg = 0u;
        const cbplc_err_t drc = cbplc_diag_decode_request(msg->payload, msg->payload_len,
                                                          &cmd, &arg);
        uint8_t reply[CBPLC_DIAG_REPLY_SIZE];
        cbplc_diag_state_t st;
        size_t rn;

        /* AN OLDER FIRMWARE DRAINS WHATEVER IT IS ASKED. This is the whole compatibility story
         * and it is modelled rather than assumed: with diag_control false, a control request
         * comes back as log text, which is exactly what cbctl must be able to survive. */
        if (!lb->diag_control) {
            cmd = CBPLC_DIAG_DRAIN;
        } else if (drc != CBPLC_E_OK) {
            /* Refused: change nothing, drain nothing, and say why. */
            st.status = (uint8_t)drc;
            st.level = lb->diag_level;
            st.categories = lb->diag_categories;
            st.dropped = lb->diag_dropped;
            st.pending = lb->diag_len - lb->diag_taken;
            rn = cbplc_diag_encode_reply(&st, reply, sizeof reply);
            (void)cbplc_respond(&lb->plc, CBPLC_T_DIAG, msg->request_id, reply, (uint16_t)rn);
            return;
        }
        lb->diag_last_cmd = (uint8_t)cmd;

        if (cmd == CBPLC_DIAG_DRAIN) {
            uint32_t avail = lb->diag_len - lb->diag_taken;
            if (avail > CBCTL_DIAG_CHUNK_FULL) {
                avail = CBCTL_DIAG_CHUNK_FULL;   /* the board's per-reply cap */
            }
            lb->diag_drains_seen++;
            (void)cbplc_respond(&lb->plc, CBPLC_T_DIAG, msg->request_id,
                                &lb->diag_text[lb->diag_taken], (uint16_t)avail);
            lb->diag_taken += avail;
            return;
        }

        lb->diag_ctl_seen++;
        switch (cmd) {
        case CBPLC_DIAG_SET_LEVEL: lb->diag_level = (uint8_t)arg; break;
        case CBPLC_DIAG_SET_CATS:  lb->diag_categories = arg; break;
        case CBPLC_DIAG_CLEAR:
            lb->diag_taken = lb->diag_len;
            lb->diag_clear_seen++;
            break;
        default: break;   /* GET_STATE is the reply itself */
        }
        st.status = 0u;
        st.level = lb->diag_level;
        st.categories = lb->diag_categories;
        st.dropped = lb->diag_dropped;
        st.pending = lb->diag_len - lb->diag_taken;
        rn = cbplc_diag_encode_reply(&st, reply, sizeof reply);
        (void)cbplc_respond(&lb->plc, CBPLC_T_DIAG, msg->request_id, reply, (uint16_t)rn);
        return;
    }
    default: break;
    }

    /* Every command is acknowledged with a result code. Zero is applied; anything else is a
     * named refusal, which is what refuse_code injects. */
    ack[0] = lb->refuse_code;
    (void)cbplc_respond(&lb->plc, CBPLC_T_ACK, msg->request_id, ack, 1u);
}

void cbctl_loopback_init(cbctl_loopback_t *lb)
{
    static const uint8_t kDefaultMac[6] = { 0xD0u, 0xCFu, 0x13u, 0x71u, 0xFFu, 0x54u };
    memset(lb, 0, sizeof *lb);
    lb->first_auth_decision = -1;   /* 0 is a real (and dangerous) decision — not "unset" */
    memcpy(lb->board_mac, kDefaultMac, 6u);

    lb->ctrl_tx.ctx = lb;
    lb->ctrl_tx.send = ctrl_send;
    lb->ctrl_tx.recv = ctrl_recv;
    lb->ctrl_tx.mtu = CBCTL_LB_FRAME;

    lb->plc_tx.ctx = lb;
    lb->plc_tx.send = plc_send;
    lb->plc_tx.recv = plc_recv;
    lb->plc_tx.mtu = CBCTL_LB_FRAME;

    (void)cbplc_open(&lb->plc, &lb->plc_tx, NULL, plc_on_message, lb);
}

void cbctl_loopback_poll(cbctl_loopback_t *lb, uint32_t now_ms)
{
    unsigned i;
    /* Bounded, like everything else. A board that is flooded still yields. */
    for (i = 0u; i < 16u; i++) {
        (void)cbplc_poll(&lb->plc, now_ms);
    }
}
