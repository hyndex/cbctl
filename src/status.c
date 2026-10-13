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

bool cbctl_safety_raw(const cbctl_link_t *link, uint8_t *mask, bool *bus_unreadable)
{
    if (link == NULL || !link->safety_raw_valid) {
        return false;
    }
    if (mask != NULL) {
        *mask = link->safety_raw_mask;
    }
    if (bus_unreadable != NULL) {
        *bus_unreadable = link->safety_bus_unreadable != 0u;
    }
    return true;
}

bool cbctl_relay_drift(const cbctl_link_t *link, uint8_t *readback)
{
    if (link == NULL || !link->relay_drifted) {
        return false;
    }
    if (readback != NULL) {
        *readback = link->relay_drift_readback;
    }
    return true;
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

bool cbctl_evccid(const cbctl_link_t *link, uint8_t out[CBCTL_EVCCID_MAX], size_t *out_len)
{
    /* *out_len is cleared on EVERY refusal path, not just the ones that fail late. A caller
     * that checks the return value is fine either way; one that reads the length first — the
     * natural shape for a variable-length identity — would otherwise act on whatever was in
     * its stack slot, and act on it as a vehicle's identity. */
    if (out_len != NULL) {
        *out_len = 0u;
    }
    if (link == NULL || out == NULL || out_len == NULL ||
        !link->flags[CBCTL_FLAG_EVCCID_KNOWN]) {
        return false;
    }
    /* Defence in depth against a stored length that outran the buffer. The parser already
     * refuses anything above CBCTL_EVCCID_MAX, so this cannot fire today — and it is here
     * precisely because that is the kind of guarantee a later edit removes without noticing
     * that a memcpy two files away depended on it. */
    if (link->evcc_id_len == 0u || link->evcc_id_len > CBCTL_EVCCID_MAX) {
        return false;
    }
    memcpy(out, link->evcc_id, (size_t)link->evcc_id_len);
    *out_len = (size_t)link->evcc_id_len;
    return true;
}

/* ---- the diff ------------------------------------------------------------------------------ */

void cbctl_status_apply(cbctl_link_t *link, const uint8_t *payload, size_t len, uint32_t now_ms);

void cbctl_status_apply(cbctl_link_t *link, const uint8_t *payload, size_t len, uint32_t now_ms)
{
    const char *buf = (const char *)payload;
    bool now_flags[CBCTL_FLAG__COUNT];
    uint32_t v;
    size_t vlen = 0u;
    const char *p;
    unsigned i;

    if (link == NULL || payload == NULL || len == 0u) {
        return;
    }

    /* READ AFTER THE GUARD, NOT BEFORE IT. This initialiser sat in the declaration block above,
     * so it dereferenced `link` three lines before the NULL check that exists to stop exactly
     * that — and at -O2 a compiler is entitled to conclude from the dereference that `link`
     * cannot be NULL and delete the check outright, turning a defensive return into a crash. */
    const bool had = link->have_flags;

    memset(now_flags, 0, sizeof now_flags);

    /* Hand the caller the raw line BEFORE any gate — see cbctl.h. A rejected line is exactly
     * when you most want to look at it. */
    if (link->status_observer != NULL) {
        link->status_observer(buf, len, link->status_observer_ud);
    }

    /* TRUNCATION DETECTION — AND IT MUST COME FIRST.
     *
     * The board's STATUS is one big formatted line with a payload cap, and CBCTL_STATUS_TAIL_KEY
     * is the LAST key in that format string — so its presence proves the whole line arrived.
     * IT MOVES WHENEVER build_status() GAINS A FIELD: a sentinel that is no longer last stops
     * detecting truncation silently, which is the exact failure it exists to catch. Named as a
     * constant, next to cbplc's own TAIL_KEY in tools/probe.py, so the two are findable
     * together. Without
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
     * line instead of going quiet. It legitimately has no tail key. Counting that as truncation
     * would throw away the one message whose entire purpose is to report the overflow. */
    bool overflow_reported = false;
    if (u32_field(buf, len, "status_overflow", &v)) {
        link->counters[CBCTL_CTR_STATUS_OVERFLOWS] = v;
        overflow_reported = true;
    }
    {
        size_t tl = 0u;
        if (field(buf, len, CBCTL_STATUS_TAIL_KEY, &tl) == NULL) {
            /* The overflow MARKER legitimately has no tail key — it IS short, that is its
             * message — so it must not count as a truncation: that inflated
             * CBCTL_CTR_STATUS_TRUNCATED at poll rate for as long as STATUS overflowed,
             * pointing the operator at the transport when the payload size was the story.
             * The presence of status_overflow= alone cannot make that call, because a
             * healthy full line may carry the key too (the loopback's does): only
             * key-present AND tail-missing is the marker. Nothing else on it is applied,
             * by the same completeness rule as everything below. */
            if (!overflow_reported) {
                link->counters[CBCTL_CTR_STATUS_TRUNCATED]++;
            }
            return;
        }
    }

    /* --- live flags --- */

    /* The pilot letter, and ONLY when it is a letter. The board publishes `cp=` as a NAME, not
     * a character: "A".."F", but also "UNKNOWN" when its CP sampler produced no reading or an
     * out-of-range one, and "?" for a state that does not exist. Anything longer than one
     * character is not a pilot state and must not be classified as one — reading p[0] of
     * "UNKNOWN" gets you 'U', which is not 'A', which is how an unreadable pilot became a car. */
    p = field(buf, len, "cp", &vlen);
    if (vlen != 1u) {
        p = NULL;
    }

    /* WHAT COUNTS AS A VEHICLE, AND WHY IT IS NOT "THE PILOT HAS LEFT A".
     *
     * This line read `(p[0] != 'A')`. IEC 61851 has six pilot states: A is nothing plugged,
     * B/C/D are a vehicle, and E and F are FAULTS — the pilot pulled to the diode short, or the
     * EVSE driving its own error state. Under the old rule every one of E, F and "UNKNOWN"
     * counted as a plugged-in car.
     *
     * What that cost, live, 2026-08-01: an e-stop drove CP to F, cbctl reported a vehicle
     * present, and the controller opened an OCPP session for a car that was not there. It did it
     * 192 times on connector 2 — /opt/evse/runtime/state/ocpp_sessions read {"1":0,"2":192} —
     * and the connector latched in Finishing and never returned to Available. The board had been
     * publishing vehicle=0 connected=0 session_reason=vehicle_left the entire time. Nothing in
     * this file read any of it.
     *
     * So read it. `vehicle=` is the board's OWN verdict — cbplc_cp_connected(), which is B/C/D
     * ANDed with whether the decoder trusts its input at all — and it is the same value the
     * board's safety layer gates the gun contactor on (safety.c refuses CBPLC_RELAY_GUN when
     * !pre->vehicle_present, in every ownership mode). Re-deriving that answer from the letter
     * when the board has already stated it is how the two drift apart, and the half of the
     * disagreement that closes a contactor is not the half we control. `connected=` carries the
     * identical value today; one of the two is read, not both, because two sources for one fact
     * is exactly the drift being fixed here.
     *
     * The letter stays as the FALLBACK — restricted to B, C and D — for a board too old to
     * publish `vehicle=`. Neither key present means no vehicle proven, which is the direction
     * that does not energise.
     *
     * BUT "THE BOARD CANNOT SEE THE PILOT" IS NOT "THE BAY IS EMPTY", AND THAT DISTINCTION IS
     * THE WHOLE ELSE-BRANCH BELOW.
     *
     * `vehicle=` is cbplc_cp_connected(), which ANDs in cbplc_cp_trusted() — so a blind pilot
     * publishes `cp=UNKNOWN vehicle=0`, indistinguishable at this parser from a car that drove
     * away. cbplc publishes UNKNOWN on ONE out-of-range sample or one gap in the stream, with
     * no debounce, by design. Believing it here would have been a NEW field failure and a
     * mirror image of the one above: cbplc's session machine does not act on its own raw
     * field, it holds the last believed presence for up to 3000 ms while energy flows
     * (kPresenceHoldEnergisedMs) because a single glitch was ending live charges. cbctl's
     * caller acts on cbctl, immediately and once — cbcontroller-v2 takes !vehicle_present
     * straight to Finishing/EvDisconnected with no debounce of its own — so cbctl has to
     * mirror the board's DECISION, not its sample.
     *
     * Hence: hold the last proven answer while the pilot is unreadable, for
     * CBCTL_PILOT_BLIND_HOLD_MS, then let it fall to absent. Held rather than latched, because
     * a dead CP front end that never recovers must still let the connector go back to
     * Available. `had` keeps the very first STATUS falling to absent rather than to whatever
     * a zeroed struct implies. */
    if (p != NULL) {
        link->pilot_blind_since_ms = 0u;
        if (u32_field(buf, len, "vehicle", &v)) {
            now_flags[CBCTL_FLAG_VEHICLE_PRESENT] = (v != 0u);
        } else {
            now_flags[CBCTL_FLAG_VEHICLE_PRESENT] =
                (p[0] == 'B' || p[0] == 'C' || p[0] == 'D');
        }
    } else {
        if (link->pilot_blind_since_ms == 0u) {
            /* 0 is the "readable" sentinel, so a blind window that starts exactly at tick 0
             * has to borrow tick 1. One millisecond, once, at boot. */
            link->pilot_blind_since_ms = (now_ms == 0u) ? 1u : now_ms;
        }
        now_flags[CBCTL_FLAG_VEHICLE_PRESENT] =
            had && link->flags[CBCTL_FLAG_VEHICLE_PRESENT] &&
            (uint32_t)(now_ms - link->pilot_blind_since_ms) < CBCTL_PILOT_BLIND_HOLD_MS;
    }

    /* A FAULTED PILOT IS NOT AN EMPTY BAY, AND THE CALLER HAS TO BE ABLE TO TELL THEM APART.
     *
     * With presence corrected above, E and F now read as "no vehicle" — right for the energy
     * path and useless to the operator, because a connector nobody can charge on then looks
     * exactly like a free one. That is the other half of the 192-session incident: the connector
     * that never came back to Available was never reported faulted either. A controller needs to
     * FAULT this connector, not idle it, and one flag is the whole difference.
     *
     * E and F only. UNKNOWN is deliberately not a fault: the board publishes it the instant a
     * single sample falls out of range, so faulting on it would take a healthy connector out of
     * service on a glitch. "We cannot see the pilot" is answered by the blind hold above and by
     * the observation-age bound, not by taking the connector out of service.
     *
     * FOR WHOEVER CALLS cbctl_slac_replug(): it does NOT raise this, and that is pinned rather
     * than hoped for. The pulse drives the pilot to E for kReplugDwellMs = 180 ms, and cbplc
     * carries a static_assert that the dwell stays under CBPLC_CP_DISCONNECT_DEBOUNCE_MS =
     * 1200 ms — "a replug that outlasts the disconnect debounce would read as an unplug" — so
     * the driven E never reaches the published state and never reaches this parser. The 3 s
     * boot teardown DOES outlast it: a board that boots with a car already on the gun publishes
     * cp=E, and this flag reads true, until the teardown ends. Honest, and the only
     * self-inflicted case. */
    now_flags[CBCTL_FLAG_PILOT_FAULT] = (p != NULL && (p[0] == 'E' || p[0] == 'F'));

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
        /* A legacy line PROVES the firmware is the classified generation NOW. Without
         * this, safety_raw_valid set by an earlier raw-generation boot stayed sticky
         * across an OTA rollback on a live link (same MAC, reset absorbed, no reopen),
         * and the LED ranking kept consulting a frozen raw mask: a legacy-published
         * e-stop (safety=1) then rendered AVAILABLE instead of EMERGENCY, because the
         * stale raw snapshot said the chain was clear and swallowed the ranking. */
        link->safety_raw_valid = 0u;
    }
    /* The board's OWN charge machine, by name. The LED ranking needs it because the
     * proxies it used instead were measurably wrong on every session: the gun contactor
     * closes for CableCheck tens of seconds before energy flows (so contactor+session
     * claimed CHARGING through PreCharge), and a vehicle-initiated stop never touches the
     * controller's stop latch (so FINISHING was simply missed). An unrecognised name maps
     * to UNKNOWN rather than being guessed at — a future firmware state must degrade to
     * the proxy ranking, not impersonate an old one. */
    {
        static const struct { const char *name; uint8_t value; } kChg[] = {
            {"Idle",        (uint8_t)CBCTL_BOARD_CHG_IDLE},
            {"AwaitAuth",   (uint8_t)CBCTL_BOARD_CHG_AWAIT_AUTH},
            {"Linking",     (uint8_t)CBCTL_BOARD_CHG_LINKING},
            {"Negotiating", (uint8_t)CBCTL_BOARD_CHG_NEGOTIATING},
            {"Energised",   (uint8_t)CBCTL_BOARD_CHG_ENERGISED},
            {"Stopping",    (uint8_t)CBCTL_BOARD_CHG_STOPPING},
            {"Fault",       (uint8_t)CBCTL_BOARD_CHG_FAULT},
        };
        size_t sl = 0u;
        const char *sp = field(buf, len, "session", &sl);
        if (sp != NULL) {
            size_t i;
            link->board_chg = (uint8_t)CBCTL_BOARD_CHG_UNKNOWN;
            for (i = 0u; i < sizeof kChg / sizeof kChg[0]; i++) {
                if (strlen(kChg[i].name) == sl &&
                    memcmp(kChg[i].name, sp, sl) == 0) {
                    link->board_chg = kChg[i].value;
                    break;
                }
            }
        }
    }
    /* Raw-publishing firmware (2026-08-14 onward) drops `safety=` and sends the
     * pins verbatim. The classified flag is DERIVED here so every downstream
     * consumer — the LED ranking above all — keeps working across both
     * generations: an unreadable bus or any asserted bit is an asserted chain.
     * Same hex discipline as relay_closed below: unparseable applies nothing. */
    {
        size_t rl = 0u;
        const char *rp = field(buf, len, "safety_in", &rl);
        unsigned mask = 0u;
        bool ok = (rp != NULL && rl >= 3u && rp[0] == '0' && (rp[1] == 'x' || rp[1] == 'X'));
        for (size_t k = 2u; ok && k < rl; k++) {
            const int nib = hex_nibble(rp[k]);
            if (nib < 0) { ok = false; break; }
            mask = (mask << 4) | (unsigned)nib;
        }
        if (ok) {
            uint32_t bus = 1u; /* absent bus flag on a raw line: assume unreadable */
            (void)u32_field(buf, len, "safety_bus", &bus);
            link->safety_raw_mask = (uint8_t)mask;
            link->safety_bus_unreadable = (bus != 0u) ? 1u : 0u;
            link->safety_raw_valid = 1u;
            now_flags[CBCTL_FLAG_SAFETY_ASSERTED] = (mask != 0u) || (bus != 0u);
        }
    }
    if (u32_field(buf, len, "relay_drifted", &v)) {
        link->relay_drifted = (v != 0u) ? 1u : 0u;
    }
    {
        size_t rl = 0u;
        const char *rp = field(buf, len, "relay_rb", &rl);
        unsigned mask = 0u;
        bool ok = (rp != NULL && rl >= 3u && rp[0] == '0' && (rp[1] == 'x' || rp[1] == 'X'));
        for (size_t k = 2u; ok && k < rl; k++) {
            const int nib = hex_nibble(rp[k]);
            if (nib < 0) { ok = false; break; }
            mask = (mask << 4) | (unsigned)nib;
        }
        if (ok) {
            link->relay_drift_readback = (uint8_t)mask;
        }
    }
    /* relay_closed is a hex mask; a closed GUN bit is the board saying the contactor is in.
     *
     * A VALUE WE CANNOT PARSE IS NOT "EVERY RELAY IS OPEN". The malformed-digit path used to set
     * mask = 0 and then apply it, so one bad character turned "I could not read this" into a
     * published claim that the gun contactor is out — the same class of mistake as the pilot
     * letter above, and the more expensive direction of the two: a controller, an operator or a
     * driver reading "not energised" off a connector that is in fact live. Unparseable now
     * applies nothing and keeps the last known good, which is the discipline the truncation
     * guard at the top of this function already follows for the whole line. */
    {
        size_t rl = 0u;
        const char *rp = field(buf, len, "relay_closed", &rl);
        unsigned mask = 0u;
        bool ok = (rp != NULL && rl >= 3u && rp[0] == '0' && (rp[1] == 'x' || rp[1] == 'X'));
        for (size_t k = 2u; ok && k < rl; k++) {
            const int nib = hex_nibble(rp[k]);
            if (nib < 0) {
                ok = false;
                break;
            }
            mask = (mask << 4) | (unsigned)nib;
        }
        if (ok) {
            now_flags[CBCTL_FLAG_ENERGISED] = ((mask & CBPLC_RELAY_GUN) != 0u);
            link->relay_closed_mask = (uint8_t)mask;
        } else {
            now_flags[CBCTL_FLAG_ENERGISED] = had && link->flags[CBCTL_FLAG_ENERGISED];
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

    /* --- the vehicle EVCCID, length-prefixed hex ---
     *
     * TWO FIELDS, AND THE LENGTH IS THE ONE THAT DECIDES. `evcc_len` says how many bytes the
     * vehicle supplied — 6 for ISO 15118-2, 8 for DIN 70121 — and `evcc` carries exactly that
     * many bytes of hex. Gating on the length rather than on the value is what keeps "no
     * EVCCID this session" distinct from "an EVCCID of all zeros"; the board publishes
     * evcc_len=0 from session start until a SessionSetupReq supplies one.
     *
     * A BOARD TOO OLD TO SEND EITHER KEY leaves the flag clear, because now_flags starts
     * zeroed and only a present field sets it. Unknown, never a phantom identity.
     *
     * `evcc` cannot be found inside `evcc_len` even though it is a prefix of it: field()
     * anchors on a preceding space AND requires '=' immediately after the key, which is the
     * same guard that keeps "ev_mac" out of "ev_mac_ok". Both halves are pinned by tests.
     *
     * A LENGTH WE BELIEVE BUT HEX WE CANNOT PARSE APPLIES NEITHER. Half an identity is worse
     * than none: it would be a DIFFERENT vehicle's id as far as any table is concerned. */
    if (u32_field(buf, len, "evcc_len", &v) && v > 0u && v <= CBCTL_EVCCID_MAX) {
        p = field(buf, len, "evcc", &vlen);
        if (p != NULL && vlen == (size_t)v * 2u) {
            uint8_t id[CBCTL_EVCCID_MAX];
            bool ok = true;
            for (i = 0u; i < (unsigned)v; i++) {
                const int hi = hex_nibble(p[i * 2u]);
                const int lo = hex_nibble(p[(i * 2u) + 1u]);
                if (hi < 0 || lo < 0) {
                    ok = false;
                    break;
                }
                id[i] = (uint8_t)((hi << 4) | lo);
            }
            if (ok) {
                memset(link->evcc_id, 0, sizeof link->evcc_id);
                memcpy(link->evcc_id, id, (size_t)v);
                link->evcc_id_len = (uint8_t)v;
                now_flags[CBCTL_FLAG_EVCCID_KNOWN] = true;
            }
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
        if (now_flags[CBCTL_FLAG_EVCCID_KNOWN] && !link->flags[CBCTL_FLAG_EVCCID_KNOWN]) {
            push_event(link, CBCTL_EV_EVCCID_KNOWN, now_ms);
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
             * which is the only thing that could justify one.
             *
             * THE ENERGY RELAYS ONLY. Relay 3 is the AC feed to modules that several guns
             * share; no session justified it, physical presence did, and it outlives the
             * session by design. This released it with the gun — the same mistake cbplc
             * fixed on its own session-end path (d4b0184) — and a controller that sends
             * relays only on change never asked again. evse-vscs8, 2026-10-13: a remote stop
             * with the car left plugged in dropped the AC feed to both GWBZ modules, and the
             * gun sat on EVSE-MOD-0303 for nine minutes until a USB glitch rebuilt the link. */
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
            if ((link->relay_state & CBPLC_RELAY_ENERGY) != 0u) {
                link->relay_state = (uint8_t)(link->relay_state & ~CBPLC_RELAY_ENERGY);
                link->relay_dirty = true;
                link->lease_released_no_session++;
            }
        }
    }

    memcpy(link->flags, now_flags, sizeof link->flags);
    link->have_flags = true;
    link->last_status = cbctl_instant(now_ms);
}
