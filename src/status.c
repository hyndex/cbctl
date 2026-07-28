/* STATUS parsing and event synthesis.
 *
 * The board publishes ~165 space-separated key=value pairs on request. cbctl parses the ones
 * a controller must act on into typed state, and DERIVES events by diffing consecutive
 * snapshots — there is no push channel to subscribe to.
 *
 * Allocation-free and destructive-free: the payload is parsed in place from a const buffer.
 */
#include "cbctl/cbctl.h"

#include <string.h>

void cbctl_link_reestablish(cbctl_link_t *link);

/* ---- tiny scanners ---------------------------------------------------------------------- */

/* Find `key=` in a space-separated buffer and return a pointer just past the '='.
 *
 * Matching is anchored on a preceding space (or the very start) so that looking for "stop"
 * cannot match inside "gun_open_timeouts=..." or any other key with the same tail. A
 * substring search here would silently read the wrong field, which is the kind of bug that
 * produces a plausible number and no error at all. */
static const char *field(const char *buf, size_t len, const char *key, size_t *out_len)
{
    const size_t klen = strlen(key);
    size_t i = 0u;

    while (i < len) {
        const bool at_start = (i == 0u) || (buf[i - 1u] == ' ');
        if (at_start && (len - i) > klen && memcmp(&buf[i], key, klen) == 0 &&
            buf[i + klen] == '=') {
            const size_t vs = i + klen + 1u;
            size_t ve = vs;
            while (ve < len && buf[ve] != ' ') {
                ve++;
            }
            *out_len = ve - vs;
            return &buf[vs];
        }
        i++;
    }
    *out_len = 0u;
    return NULL;
}

static bool u32_field(const char *buf, size_t len, const char *key, uint32_t *out)
{
    size_t vlen = 0u;
    const char *v = field(buf, len, key, &vlen);
    uint32_t acc = 0u;
    size_t i;

    if (v == NULL || vlen == 0u) {
        return false;
    }
    for (i = 0u; i < vlen; i++) {
        if (v[i] < '0' || v[i] > '9') {
            return false; /* not a number: report absence rather than a partial value */
        }
        acc = (acc * 10u) + (uint32_t)(v[i] - '0');
    }
    *out = acc;
    return true;
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'A' && c <= 'F') { return 10 + (c - 'A'); }
    if (c >= 'a' && c <= 'f') { return 10 + (c - 'a'); }
    return -1;
}

/* ---- event queue -------------------------------------------------------------------------- */

static void push_event(cbctl_link_t *link, cbctl_event_kind_t kind, uint32_t at_ms)
{
    const uint8_t next = (uint8_t)((link->ev_head + 1u) % (uint8_t)(sizeof link->ev / sizeof link->ev[0]));
    if (next == link->ev_tail) {
        /* Full: drop the OLDEST. A controller that has fallen behind needs the newest state,
         * not the state it already failed to act on. Counted so the fall-behind is visible. */
        link->ev_tail = (uint8_t)((link->ev_tail + 1u) % (uint8_t)(sizeof link->ev / sizeof link->ev[0]));
        link->ev_dropped++;
    }
    link->ev[link->ev_head].kind = kind;
    link->ev[link->ev_head].at_ms = at_ms;
    link->ev_head = next;
}

void push_quarantine_event(cbctl_link_t *link, uint32_t now_ms);

void push_quarantine_event(cbctl_link_t *link, uint32_t now_ms)
{
    /* Loud on purpose. A charger that silently demotes a connector is worse than one that
     * fails: the operator finds out from a customer. */
    push_event(link, CBCTL_EV_LINK_QUARANTINED, now_ms);
}

bool cbctl_next_event(cbctl_link_t *link, cbctl_event_t *out)
{
    if (link == NULL || out == NULL || link->ev_tail == link->ev_head) {
        return false;
    }
    *out = link->ev[link->ev_tail];
    link->ev_tail = (uint8_t)((link->ev_tail + 1u) % (uint8_t)(sizeof link->ev / sizeof link->ev[0]));
    return true;
}

/* ---- accessors ----------------------------------------------------------------------------- */

bool cbctl_flag(const cbctl_link_t *link, cbctl_flag_t f)
{
    if (link == NULL || (unsigned)f >= (unsigned)CBCTL_FLAG__COUNT) {
        return false;
    }
    return link->flags[f];
}

uint32_t cbctl_counter(const cbctl_link_t *link, cbctl_counter_t c)
{
    if (link == NULL || (unsigned)c >= (unsigned)CBCTL_CTR__COUNT) {
        return 0u;
    }
    return link->counters[c];
}

bool cbctl_ev_target(const cbctl_link_t *link, uint32_t *mv, uint32_t *ma)
{
    /* A TARGET WITHOUT A SESSION IS NOT A TARGET.
     *
     * cbplc clears its own EV target when a session STARTS, never when one ends, so between
     * sessions its STATUS keeps echoing the last vehicle's request — and every poll re-parses it,
     * which is why clearing the cached copy once is not enough. Observed on the bench as
     * ev_tgt_mv=356000 alongside substate=Terminated and secc_active=0: a controller reading this
     * would drive real power modules to a demand from a car that had already gone.
     *
     * Gating on the live session here rather than in the parser keeps one rule in one place and
     * cannot be defeated by the next STATUS arriving. */
    if (link == NULL || !link->ev_target_valid || link->ev_target_ms == 0u) {
        return false;
    }
    if ((uint32_t)(link->last_status.ms - link->ev_target_ms) >= CBCTL_EV_TARGET_STALE_MS) {
        return false;
    }
    if (mv != NULL) { *mv = link->ev_target_mv; }
    if (ma != NULL) { *ma = link->ev_target_ma; }
    return true;
}

int cbctl_ev_soc_pct(const cbctl_link_t *link)
{
    /* Same rule as cbctl_ev_target(): cbplc clears the SoC at session START, so between
     * sessions its STATUS keeps echoing the last vehicle's percentage. A controller that
     * believes it is a live reading will bill, display, or gate on a car that has gone. */
    if (link == NULL || !link->flags[CBCTL_FLAG_EV_SOC_KNOWN] ||
        !link->flags[CBCTL_FLAG_SESSION_ACTIVE]) {
        return -1;
    }
    return (int)link->counters[CBCTL_CTR_EV_SOC_PCT];
}

bool cbctl_vehicle_mac(const cbctl_link_t *link, uint8_t out[6])
{
    if (link == NULL || out == NULL || !link->flags[CBCTL_FLAG_VEHICLE_MAC_KNOWN]) {
        return false;
    }
    memcpy(out, link->ev_mac, 6u);
    return true;
}

/* ---- the diff ------------------------------------------------------------------------------ */

void cbctl_status_apply(cbctl_link_t *link, const uint8_t *payload, size_t len, uint32_t now_ms);

void cbctl_status_apply(cbctl_link_t *link, const uint8_t *payload, size_t len, uint32_t now_ms)
{
    const char *buf = (const char *)payload;
    const bool had = link->have_flags;
    bool now_flags[CBCTL_FLAG__COUNT];
    uint32_t v;
    size_t vlen = 0u;
    const char *p;
    unsigned i;

    if (link == NULL || payload == NULL || len == 0u) {
        return;
    }
    memset(now_flags, 0, sizeof now_flags);

    /* Hand the caller the raw line BEFORE any gate — see cbctl.h. A rejected line is exactly
     * when you most want to look at it. */
    if (link->status_observer != NULL) {
        link->status_observer(buf, len, link->status_observer_ud);
    }

    /* TRUNCATION DETECTION — AND IT MUST COME FIRST.
     *
     * The board's STATUS is one big formatted line with a payload cap, and `b1_dwell` is the
     * LAST key in that format string — so its presence proves the whole line arrived. Without
     * this check a truncated reply is indistinguishable from a healthy one: the keys that fell
     * off simply read as absent, every flag derived from them silently reads false, and a
     * controller concludes "no vehicle, no session, no match" while a car is plugged in.
     *
     * This is not hypothetical. It shipped in this project once: new fields pushed STATUS past
     * the cap and every downstream assertion quietly read a missing key.
     *
     * On truncation we return WITHOUT applying anything. That is the contract, and it used to
     * be a LIE: this block sat at the BOTTOM of the function, so by the time it ran, every
     * scalar written straight through to the link — ev_target_mv, relay_closed_mask, the whole
     * counter block — had already been applied from the partial line. Only the flags, staged in
     * now_flags[] and applied at the end, were actually protected. A truncated line could
     * therefore park a stale EV target and a wrong relay mask in believed state while the
     * comment promised it could not. Parsing nothing until the line is proven whole is the only
     * version of this rule that is true.
     *
     * The board's overflow MARKER is a different animal and is handled first: when STATUS grows
     * past the payload cap, cbplc deliberately sends a short "status_overflow=N cp=… session=…"
     * line instead of going quiet. It legitimately has no b1_dwell. Counting that as truncation
     * would throw away the one message whose entire purpose is to report the overflow. */
    if (u32_field(buf, len, "status_overflow", &v)) {
        link->counters[CBCTL_CTR_STATUS_OVERFLOWS] = v;
    }
    {
        size_t tl = 0u;
        if (field(buf, len, "b1_dwell", &tl) == NULL) {
            link->counters[CBCTL_CTR_STATUS_TRUNCATED]++;
            return;
        }
    }

    /* --- live flags --- */
    p = field(buf, len, "cp", &vlen);
    if (p != NULL && vlen >= 1u) {
        now_flags[CBCTL_FLAG_VEHICLE_PRESENT] = (p[0] != 'A');
    }
    if (u32_field(buf, len, "slac_matched", &v)) {
        now_flags[CBCTL_FLAG_SLAC_MATCHED] = (v != 0u);
    }
    if (u32_field(buf, len, "secc_active", &v)) {
        now_flags[CBCTL_FLAG_SESSION_ACTIVE] = (v != 0u);
    }
    if (u32_field(buf, len, "stop", &v)) {
        now_flags[CBCTL_FLAG_STOP_LATCHED] = (v != 0u);
    }
    if (u32_field(buf, len, "ev_mac_ok", &v)) {
        now_flags[CBCTL_FLAG_VEHICLE_MAC_KNOWN] = (v != 0u);
    }
    if (u32_field(buf, len, "safety", &v)) {
        now_flags[CBCTL_FLAG_SAFETY_ASSERTED] = (v != 0u);
    }
    /* relay_closed is a hex mask; a closed GUN bit is the board saying the contactor is in. */
    {
        size_t rl = 0u;
        const char *rp = field(buf, len, "relay_closed", &rl);
        if (rp != NULL && rl >= 3u && rp[0] == '0' && (rp[1] == 'x' || rp[1] == 'X')) {
            unsigned mask = 0u;
            for (size_t k = 2u; k < rl; k++) {
                const int nib = hex_nibble(rp[k]);
                if (nib < 0) {
                    mask = 0u;
                    break;
                }
                mask = (mask << 4) | (unsigned)nib;
            }
            now_flags[CBCTL_FLAG_ENERGISED] = ((mask & CBPLC_RELAY_GUN) != 0u);
            link->relay_closed_mask = (uint8_t)mask;
        }
    }

    /* --- the vehicle MAC, 12 hex digits --- */
    p = field(buf, len, "ev_mac", &vlen);
    if (p != NULL && vlen == 12u) {
        uint8_t mac[6];
        bool ok = true;
        for (i = 0u; i < 6u; i++) {
            const int hi = hex_nibble(p[i * 2u]);
            const int lo = hex_nibble(p[(i * 2u) + 1u]);
            if (hi < 0 || lo < 0) {
                ok = false;
                break;
            }
            mac[i] = (uint8_t)((hi << 4) | lo);
        }
        if (ok) {
            memcpy(link->ev_mac, mac, 6u);
        }
    }

    /* --- cumulative counters, kept in a SEPARATE vocabulary from the flags above --- */
    /* LIVENESS COMES FROM dc_ans, NOT FROM THE TARGET VALUE.
     *
     * Two wrong rules were tried here first. Gating on `secc_active` failed because that flag
     * reads 0 while the EV is actively in PreCharge. Stamping when the target VALUE changes
     * failed because a vehicle in steady PreCharge asks for the SAME millivolts every time — a
     * live demand aged out in six seconds and we commanded the modules off underneath a car that
     * was still asking.
     *
     * `dc_ans` is cbplc's count of DC requests it has ANSWERED. It rises only while the vehicle
     * is actually sending them, and it cannot rise between sessions. "Is this demand live" and
     * "is the vehicle still asking" are then the same question, which is what they should have
     * been from the start. */
    if (u32_field(buf, len, "dc_ans", &v)) {
        if (link->have_flags && v != link->dc_answers) {
            link->ev_target_ms = now_ms;
        }
        link->dc_answers = v;
    }
    if (u32_field(buf, len, "ev_tgt_mv", &v)) {
        link->ev_target_mv = v;
        link->ev_target_valid = (v > 0u);
    }
    if (u32_field(buf, len, "ev_tgt_ma", &v)) {
        link->ev_target_ma = v;
    }
    if (u32_field(buf, len, "ev_soc", &v)) {
        /* cbplc reports 0 until the EV has actually sent one, so a zero is only believed
         * once the session is live enough to have carried a DC request. */
        link->counters[CBCTL_CTR_EV_SOC_PCT] = (v <= 100u) ? v : 0u;
        now_flags[CBCTL_FLAG_EV_SOC_KNOWN] = (v > 0u && v <= 100u);
    }
    if (u32_field(buf, len, "ev_ready", &v)) {
        now_flags[CBCTL_FLAG_EV_READY] = (v != 0u);
    }
    if (u32_field(buf, len, "ev_pause", &v)) {
        now_flags[CBCTL_FLAG_EV_PAUSED] = (v != 0u);
    }
    if (u32_field(buf, len, "secc_streams", &v)) {
        link->counters[CBCTL_CTR_SESSIONS] = v;
    }
    if (u32_field(buf, len, "relay_safety_opens", &v)) {
        link->counters[CBCTL_CTR_RELAY_SAFETY_OPENS] = v;

        /* ARE WE FIGHTING THE BOARD'S SAFETY LAYER?
         *
         * A close we hold, that the board keeps opening, produces one safety-open per renewal
         * — with every command ACKed, so nothing else in the system can tell. The relay
         * actuates on every cycle: audible chatter and contactor wear, forever. Give up on the
         * intent instead, once, and say so loudly. The caller can re-establish the close when
         * the precondition it was missing is actually met. */
        if (!link->relay_base_valid) {
            link->relay_safety_base = v;
            link->relay_base_valid = true;
        } else if ((link->relay_state & CBPLC_RELAY_ALL) != 0u &&
                   v >= link->relay_safety_base + CBCTL_RELAY_OVERRIDE_LIMIT) {
            link->relay_state = 0u;      /* stop asking for a close we cannot have */
            link->relay_dirty = true;    /* and tell the board so, once */
            link->relay_overridden++;
            link->relay_safety_base = v;
            push_event(link, CBCTL_EV_RELAY_OVERRIDDEN, now_ms);
        }
    }
    if (u32_field(buf, len, "relay_lease_opens", &v)) {
        link->counters[CBCTL_CTR_RELAY_LEASE_OPENS] = v;
    }
    if (u32_field(buf, len, "gun_open_timeouts", &v)) {
        link->counters[CBCTL_CTR_GUN_OPEN_TIMEOUTS] = v;
    }
    if (u32_field(buf, len, "up_ms", &v)) {
        /* A board that has been up for LESS time than we last saw has rebooted. Delivered as
         * an event rather than silently absorbed: leases and authorisation do not survive a
         * reset, and a controller that does not notice is one commanding a board that has
         * forgotten everything it was told. */
        if (had && v < link->counters[CBCTL_CTR_UPTIME_MS]) {
            push_event(link, CBCTL_EV_BOARD_RESET, now_ms);
            /* Re-assert the state a reboot destroyed — above all the STOP latch, which is
             * sent once on change and would otherwise never be restored. */
            cbctl_link_reestablish(link);
        }
        link->counters[CBCTL_CTR_UPTIME_MS] = v;
    }

    /* --- edges --- */
    if (had) {
        if (now_flags[CBCTL_FLAG_VEHICLE_PRESENT] && !link->flags[CBCTL_FLAG_VEHICLE_PRESENT]) {
            push_event(link, CBCTL_EV_VEHICLE_PLUGGED, now_ms);
        }
        if (!now_flags[CBCTL_FLAG_VEHICLE_PRESENT] && link->flags[CBCTL_FLAG_VEHICLE_PRESENT]) {
            push_event(link, CBCTL_EV_VEHICLE_UNPLUGGED, now_ms);
        }
        if (now_flags[CBCTL_FLAG_SLAC_MATCHED] && !link->flags[CBCTL_FLAG_SLAC_MATCHED]) {
            push_event(link, CBCTL_EV_SLAC_MATCHED, now_ms);
        }
        if (now_flags[CBCTL_FLAG_VEHICLE_MAC_KNOWN] &&
            !link->flags[CBCTL_FLAG_VEHICLE_MAC_KNOWN]) {
            push_event(link, CBCTL_EV_VEHICLE_MAC_KNOWN, now_ms);
        }
        if (now_flags[CBCTL_FLAG_SESSION_ACTIVE] && !link->flags[CBCTL_FLAG_SESSION_ACTIVE]) {
            push_event(link, CBCTL_EV_SESSION_STARTED, now_ms);
        }
        if (!now_flags[CBCTL_FLAG_SESSION_ACTIVE] && link->flags[CBCTL_FLAG_SESSION_ACTIVE]) {
            push_event(link, CBCTL_EV_SESSION_ENDED, now_ms);
            /* A RELAY LEASE MUST NOT OUTLIVE THE SESSION THAT JUSTIFIED IT.
             *
             * Observed on hardware: the session dropped seconds after the gun was closed, and
             * the lease kept being renewed — 84 renewals against a dead session. The board's
             * safety layer opens a contactor it has no session for, our renewal closes it
             * again, and the result is the audible relay flipping the operator reported.
             * Releasing here makes the caller re-establish the close for the NEXT session,
             * which is the only thing that could justify one. */
            /* THE EV'S TARGET MUST NOT OUTLIVE THE EV'S SESSION.
             *
             * cbplc clears its own target when a session STARTS, not when one ends, so between
             * sessions it keeps reporting the last car's request — and cbctl cached that forever.
             * A controller then reads cbctl_ev_target() as a live demand from a vehicle that has
             * gone, and drives power modules to it. Observed on the bench as ev_tgt_mv=356000
             * alongside substate=Terminated and secc_active=0. */
            link->ev_target_mv = 0u;
            link->ev_target_ma = 0u;
            link->ev_target_valid = false;
            if ((link->relay_state & CBPLC_RELAY_ALL) != 0u) {
                link->relay_state = 0u;
                link->relay_dirty = true;
                link->lease_released_no_session++;
            }
        }
    }

    memcpy(link->flags, now_flags, sizeof link->flags);
    link->have_flags = true;
    link->last_status = cbctl_instant(now_ms);
}
