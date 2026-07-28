/* An in-process fake PLC, for tests that must run with no hardware.
 *
 * It is not a mock of the protocol — it runs cbplc's REAL session, segment and message code
 * on the far side of a pair of byte queues. A test that passes here has exercised the same
 * framing, CRC, segmentation and stop-and-wait correlation that runs on the board.
 *
 * The point of the wedge/refuse controls is that isolation and refusal handling cannot be
 * tested by a peer that always behaves.
 */
#ifndef CBCTL_LOOPBACK_H
#define CBCTL_LOOPBACK_H

#include <stdbool.h>
#include <stdint.h>

#include "cbplc/session.h"
#include "cbplc/transport.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CBCTL_LB_QUEUE
#define CBCTL_LB_QUEUE 32u
#endif
#ifndef CBCTL_LB_FRAME
#define CBCTL_LB_FRAME 1040u
#endif

typedef struct {
    uint8_t buf[CBCTL_LB_FRAME];
    uint16_t len;
} cbctl_lb_frame_t;

typedef struct {
    cbctl_lb_frame_t f[CBCTL_LB_QUEUE];
    unsigned head, tail;
    unsigned dropped; /* queue overflow — a real link loses frames too */
} cbctl_lb_queue_t;

typedef struct {
    cbctl_lb_queue_t to_plc;   /* controller -> PLC */
    cbctl_lb_queue_t to_ctrl;  /* PLC -> controller */

    cbplc_transport_t ctrl_tx; /* hand this to cbctl_open() */
    cbplc_transport_t plc_tx;

    cbplc_session_t plc; /* the far side, running the real session code */

    /* The state the fake board reports in STATUS. Tests drive these to make the controller
     * observe plugs, matches and sessions. */
    char     cp;              /* 'A'..'F'; 'A' = nothing plugged */
    bool     slac_matched;
    bool     secc_active;
    bool     stop_latched;
    bool     ev_mac_ok;
    bool     safety_asserted;
    uint8_t  relay_closed_mask;
    uint32_t dc_answers;    /* mirrors cbplc's dc_ans */
    bool     ev_demand_idle;/* set to stop answering: models a vehicle that has gone quiet */
    uint8_t  ev_soc;
    uint32_t ev_tgt_mv, ev_tgt_ma;
    bool     ev_ready;
    bool     ev_paused;
    uint8_t  ev_mac[6];
    uint8_t  board_mac[6];   /* what IDENTITY reports; change it to model a swapped board */
    uint32_t secc_streams;
    uint32_t up_ms;
    uint32_t relay_lease_opens, gun_open_timeouts, relay_safety_opens;
    /* Accept every RELAY close and then immediately "open it again" in the reported counters,
     * exactly as the board's safety layer does when a precondition is unmet. */
    bool     safety_undoes_closes;

    /* Fault injection. */
    bool wedged;       /* answers nothing — models a hung or unplugged board            */
    bool transport_dead; /* send/recv fail HARD — models a closed fd / vanished USB device */
    bool tx_full;      /* send() always refuses — models a full USB-CDC TX buffer       */
    uint8_t refuse_code; /* non-zero: ACK every command with this gate code             */
    bool truncate_status; /* drop the tail sentinel — models a STATUS past the payload cap */
    uint8_t swallow_type; /* answer nothing for this message type — models a board that has
                           * stopped servicing ONE request while the rest of it is alive, which
                           * is what forces the stop-and-wait retry path */

    /* --- the diagnostic log ring, modelled ---------------------------------------------
     *
     * `diag_text` is what the board would hand over on the next drain; the fake ring is a
     * simple buffer rather than a real one because what is under test here is the CONTROLLER's
     * behaviour — when it asks, how it backs off, and what it does with the answer.
     *
     * `diag_control` is the switch that matters most: with it FALSE the fake board behaves like
     * a firmware built before subcommands existed and answers EVERY diag request, control ones
     * included, by draining. That is the compatibility case, and it is only a test away from
     * being wrong in the field. */
    bool     diag_control;      /* false models an older firmware: control requests drain */
    uint8_t  diag_level;
    uint32_t diag_categories;
    uint32_t diag_dropped;
    uint8_t  diag_text[4096];
    uint32_t diag_len;          /* bytes currently buffered */
    uint32_t diag_taken;        /* read cursor into diag_text */
    unsigned diag_drains_seen, diag_ctl_seen, diag_clear_seen;
    uint8_t  diag_last_cmd;

    /* Observability for assertions. */
    unsigned ctrl_frames_in, plc_frames_in;
    unsigned identify_seen, limits_seen, feedback_seen, auth_seen, heartbeat_seen;
    /* Decision byte of the FIRST AUTHORIZE ever seen, -1 if none. On this wire byte 0 == 0 is a
     * TERMINAL refusal with a 30 s latch, so "what did the controller say before the application
     * had an opinion" is a safety question, not a curiosity. */
    int      first_auth_decision;
    uint8_t  last_feedback_contactor;
    unsigned status_seen;
    unsigned stop_seen;
} cbctl_loopback_t;

/* Wire up both sides. After this, `lb->ctrl_tx` is a transport cbctl_open() accepts. */
void cbctl_loopback_init(cbctl_loopback_t *lb);

/* Let the fake PLC do its bounded unit of work. Call it in the test loop alongside
 * cbctl_tick(), exactly as a real board runs independently of the controller. */
void cbctl_loopback_poll(cbctl_loopback_t *lb, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif /* CBCTL_LOOPBACK_H */
