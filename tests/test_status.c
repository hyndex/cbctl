/* Observed state and synthesised events, against the real STATUS wire shape. */
#include "cbctl/cbctl.h"
#include "loopback.h"

/* Internal entry point: these two tests feed a hand-built line the loopback cannot produce —
 * a truncated one and the board's short overflow marker. */
void cbctl_status_apply(cbctl_link_t *link, const uint8_t *payload, size_t len, uint32_t now_ms);

#include <stdio.h>
#include <string.h>

static int g_fail;
#define CHECK(c)                                                                          \
    do {                                                                                  \
        if (!(c)) {                                                                       \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                         \
            g_fail++;                                                                     \
        }                                                                                 \
    } while (0)
#define TEST(name) static void test_##name(void)
#define RUN(name)                                                                         \
    do {                                                                                  \
        printf("  %s\n", #name);                                                          \
        test_##name();                                                                    \
    } while (0)

static void step(cbctl_link_t *link, cbctl_loopback_t *lb, uint32_t *now, uint32_t d)
{
    cbctl_tick(link, *now);
    cbctl_loopback_poll(lb, *now);
    cbctl_tick(link, *now);
    cbctl_loopback_poll(lb, *now);
    *now += d;
}

static void bring_up(cbctl_link_t *link, cbctl_loopback_t *lb, uint32_t *now)
{
    cbctl_loopback_init(lb);
    lb->cp = 'A';
    CHECK(cbctl_open(link, &lb->ctrl_tx) == CBCTL_OK);
    for (int i = 0; i < 10 && cbctl_link_state(link) != CBCTL_LINK_READY; i++) {
        step(link, lb, now, 10u);
    }
    for (int i = 0; i < 4; i++) {
        step(link, lb, now, 600u); /* let a first STATUS land */
    }
}

static bool drain_for(cbctl_link_t *link, cbctl_event_kind_t want)
{
    cbctl_event_t ev;
    bool found = false;
    while (cbctl_next_event(link, &ev)) {
        if (ev.kind == want) {
            found = true;
        }
    }
    return found;
}

TEST(status_is_polled_and_parsed)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    CHECK(lb.status_seen >= 1u);
    CHECK(!cbctl_flag(&link, CBCTL_FLAG_VEHICLE_PRESENT)); /* cp=A */
}

/* A plug is an edge on the pilot leaving A, and it must be reported once, not continuously. */
TEST(plugging_in_raises_exactly_one_event)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_event_t ev;
    int plugged = 0;

    bring_up(&link, &lb, &now);
    while (cbctl_next_event(&link, &ev)) { /* discard bring-up edges */
    }

    lb.cp = 'B';
    for (int i = 0; i < 6; i++) {
        step(&link, &lb, &now, 600u);
    }
    while (cbctl_next_event(&link, &ev)) {
        if (ev.kind == CBCTL_EV_VEHICLE_PLUGGED) {
            plugged++;
        }
    }
    CHECK(plugged == 1);
    CHECK(cbctl_flag(&link, CBCTL_FLAG_VEHICLE_PRESENT));
}

/* THE AUTOCHARGE PATH.
 *
 * The MAC becomes known at SLAC match — before any V2G session and long before an EVCCID.
 * A controller recognising a returning vehicle needs it at that moment, so the event and the
 * accessor must both be available then. */
TEST(the_vehicle_mac_arrives_at_slac_match_and_is_readable)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    uint8_t mac[6];
    static const uint8_t expect[6] = { 0xDEu, 0xADu, 0xBEu, 0xEFu, 0x00u, 0x42u };

    bring_up(&link, &lb, &now);
    CHECK(!cbctl_vehicle_mac(&link, mac)); /* not known yet, and says so */

    lb.cp = 'B';
    lb.slac_matched = true;
    memcpy(lb.ev_mac, expect, 6u);
    lb.ev_mac_ok = true;
    for (int i = 0; i < 6; i++) {
        step(&link, &lb, &now, 600u);
    }

    CHECK(drain_for(&link, CBCTL_EV_VEHICLE_MAC_KNOWN));
    CHECK(cbctl_flag(&link, CBCTL_FLAG_SLAC_MATCHED));
    CHECK(cbctl_vehicle_mac(&link, mac));
    CHECK(memcmp(mac, expect, 6u) == 0);
    /* And it is available BEFORE a session exists — the whole point of using the MAC. */
    CHECK(!cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE));
}

/* Live flags and cumulative counters are different questions with different answers. The
 * total number of sessions must never be readable as "a session is open". */
TEST(a_session_total_is_not_a_live_session)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    lb.secc_streams = 7u;     /* seven sessions have happened */
    lb.secc_active = false;   /* none is open right now */
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_counter(&link, CBCTL_CTR_SESSIONS) == 7u);
    CHECK(!cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE));
}

/* A board reset is not silently absorbed: leases and authorisation do not survive it. */
TEST(a_board_reset_is_reported)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    lb.up_ms = 500000u;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    (void)drain_for(&link, CBCTL_EV_NONE);

    lb.up_ms = 300u; /* uptime went BACKWARDS: it rebooted */
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(drain_for(&link, CBCTL_EV_BOARD_RESET));
}

/* The counters that answer today's two open questions on real hardware. */
TEST(the_relay_diagnostic_counters_are_readable)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    lb.relay_safety_opens = 2u;
    lb.relay_lease_opens = 3u;
    lb.gun_open_timeouts = 1u;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_counter(&link, CBCTL_CTR_RELAY_SAFETY_OPENS) == 2u);
    CHECK(cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS) == 3u);
    CHECK(cbctl_counter(&link, CBCTL_CTR_GUN_OPEN_TIMEOUTS) == 1u);
}

/* KEY MATCHING IS ANCHORED, NOT A SUBSTRING SEARCH.
 *
 * "stop" appears inside no key here by accident, but "ev_mac" IS a prefix of "ev_mac_ok" and
 * "relay_lease_opens" shares a tail with "relay_safety_opens". A naive search would read the
 * wrong field and return a plausible number with no error at all. */
TEST(a_key_that_is_a_prefix_of_another_is_not_confused)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    uint8_t mac[6];
    static const uint8_t expect[6] = { 0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u };

    bring_up(&link, &lb, &now);
    memcpy(lb.ev_mac, expect, 6u);
    lb.ev_mac_ok = true;
    lb.relay_safety_opens = 9u;
    lb.relay_lease_opens = 4u;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    /* THE DECOY. The board emits `last_stop=1` BEFORE `stop=0`. "stop" is a suffix of
     * "last_stop", so an unanchored search finds the decoy first and reports a latched stop
     * that does not exist — a plausible number, no error, and a controller that refuses to
     * charge. Anchoring on a preceding space is what prevents it. */
    CHECK(!cbctl_flag(&link, CBCTL_FLAG_STOP_LATCHED));

    /* ev_mac read as the 12-hex value, NOT as ev_mac_ok's "1". */
    CHECK(cbctl_vehicle_mac(&link, mac));
    CHECK(memcmp(mac, expect, 6u) == 0);
    /* And the two relay counters did not swap. */
    CHECK(cbctl_counter(&link, CBCTL_CTR_RELAY_SAFETY_OPENS) == 9u);
    CHECK(cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS) == 4u);
}

/* A TRUNCATED STATUS MUST NOT BECOME BELIEVED STATE.
 *
 * The keys that fall off a truncated line simply read as absent, so every flag derived from
 * them silently reads false: a controller concludes "no vehicle, no session, no match" while a
 * car is plugged in and matched. Worse, applying it would synthesise an UNPLUGGED edge out of
 * a lost tail. So a snapshot without the sentinel is counted and discarded whole. */
TEST(a_truncated_status_is_rejected_rather_than_half_believed)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_event_t ev;

    bring_up(&link, &lb, &now);
    lb.cp = 'B';
    lb.slac_matched = true;
    lb.secc_active = true;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_flag(&link, CBCTL_FLAG_VEHICLE_PRESENT));
    CHECK(cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE));
    while (cbctl_next_event(&link, &ev)) {
    }

    /* Now the board's line starts overflowing. Nothing about the charger changed. */
    lb.truncate_status = true;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }

    CHECK(cbctl_counter(&link, CBCTL_CTR_STATUS_TRUNCATED) > 0u);
    /* The believed state must be the LAST GOOD one, not a false "everything went away". */
    CHECK(cbctl_flag(&link, CBCTL_FLAG_VEHICLE_PRESENT));
    CHECK(cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE));
    /* And no phantom edges were manufactured from the loss. */
    {
        int phantom = 0;
        while (cbctl_next_event(&link, &ev)) {
            if (ev.kind == CBCTL_EV_VEHICLE_UNPLUGGED || ev.kind == CBCTL_EV_SESSION_ENDED) {
                phantom++;
            }
        }
        CHECK(phantom == 0);
    }
    printf("    truncated snapshots counted: %u\n",
           cbctl_counter(&link, CBCTL_CTR_STATUS_TRUNCATED));
}

/* A BOARD THAT STOPS ANSWERING STATUS MUST NOT LOOK CURRENT.
 *
 * Every flag is LAST KNOWN GOOD, which is the right default — flipping them to false on a lost
 * poll would manufacture a phantom "vehicle unplugged". But last-known-good is only safe next
 * to its age. Without a bound, a board that keeps ACKing commands while its STATUS goes silent
 * leaves a controller reading "vehicle present, session active" indefinitely and acting on
 * minutes-old history. The link stays alive; our PICTURE of it goes stale, and that is what
 * DEGRADED reports — a state that existed in the enum and that nothing ever set. */
TEST(a_silent_status_ages_the_snapshot_and_degrades_the_link)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    lb.cp = 'B';
    lb.secc_active = true;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_link_state(&link) == CBCTL_LINK_READY);
    CHECK(cbctl_observation_age_ms(&link, now) < CBCTL_OBSERVATION_STALE_MS);

    /* The board keeps answering everything EXCEPT status. */
    lb.swallow_type = (uint8_t)CBPLC_T_STATUS_REQUEST;
    for (int i = 0; i < 20; i++) {
        step(&link, &lb, &now, 600u);
    }

    printf("    observation age %ums, state=%d, stalls=%u\n",
           cbctl_observation_age_ms(&link, now), (int)cbctl_link_state(&link),
           link.observation_stalls);

    CHECK(cbctl_observation_age_ms(&link, now) >= CBCTL_OBSERVATION_STALE_MS);
    CHECK(cbctl_link_state(&link) == CBCTL_LINK_DEGRADED);
    CHECK(link.observation_stalls > 0u);
    /* The believed state is still the last good one — no phantom edges invented. */
    CHECK(cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE));

    /* And it recovers by itself when STATUS returns. */
    lb.swallow_type = 0u;
    for (int i = 0; i < 10 && cbctl_link_state(&link) != CBCTL_LINK_READY; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_link_state(&link) == CBCTL_LINK_READY);
}

/* Before any snapshot has arrived the age is NOT zero — zero would read as "perfectly
 * current" and is the most dangerous possible answer to "how old is what I believe". */
TEST(never_observed_is_distinct_from_freshly_observed)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;

    cbctl_loopback_init(&lb);
    CHECK(cbctl_open(&link, &lb.ctrl_tx) == CBCTL_OK);
    CHECK(cbctl_observation_age_ms(&link, 5000u) == CBCTL_NEVER_OBSERVED);
}

/* A CLOSE THE BOARD KEEPS UNDOING MUST BE ABANDONED, NOT RE-COMMANDED FOREVER.
 *
 * Measured on hardware: commanding a gun closed with the board's preconditions unmet produced
 * one safety-open per renewal — 23,25,27,30,32,34 against renewals 3,5,7,10,12,14 — with EVERY
 * command ACKed and gate 0. Nothing in the protocol reports it, so the controller happily
 * fights the safety layer forever while the relay actuates on every cycle. That is audible
 * chatter and real contactor wear. */
TEST(a_close_the_board_keeps_undoing_is_abandoned_loudly)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    cbctl_event_t ev;
    bool told = false;

    bring_up(&link, &lb, &now);
    lb.safety_undoes_closes = true;

    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
    for (int i = 0; i < 40; i++) {
        step(&link, &lb, &now, 600u);
    }

    while (cbctl_next_event(&link, &ev)) {
        if (ev.kind == CBCTL_EV_RELAY_OVERRIDDEN) {
            told = true;
        }
    }
    printf("    safety_opens=%u overridden=%u renewals=%u\n", lb.relay_safety_opens,
           link.relay_overridden, link.relay_renewals);

    CHECK(told);                          /* the operator is TOLD, once */
    CHECK(link.relay_overridden > 0u);
    /* And we stopped asking, so the chatter stops rather than running forever. */
    CHECK((link.relay_state & CBPLC_RELAY_GUN) == 0u);
    /* Bounded, not unbounded: a handful of actuations, not one per renewal for all time. */
    CHECK(lb.relay_safety_opens <= 10u);
}

/* THE EV'S OWN SOC — cbplc publishes it, plc_firmware never did.
 *
 * "Unknown" must be -1, not 0. A charge controller told to stop at a target SoC would read a
 * silent vehicle as an empty one and charge it forever; 0 %% is a real, alarming reading and
 * must never double as "no data". */
TEST(ev_soc_is_reported_and_unknown_is_not_zero)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    /* Nothing reported yet. */
    CHECK(cbctl_ev_soc_pct(&link) == -1);
    CHECK(!cbctl_flag(&link, CBCTL_FLAG_EV_SOC_KNOWN));

    lb.cp = 'B';
    lb.secc_active = true;
    lb.ev_soc = 42u;
    lb.ev_ready = true;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    printf("    ev_soc=%d ready=%d paused=%d\n", cbctl_ev_soc_pct(&link),
           cbctl_flag(&link, CBCTL_FLAG_EV_READY) ? 1 : 0,
           cbctl_flag(&link, CBCTL_FLAG_EV_PAUSED) ? 1 : 0);
    CHECK(cbctl_ev_soc_pct(&link) == 42);
    CHECK(cbctl_flag(&link, CBCTL_FLAG_EV_SOC_KNOWN));
    CHECK(cbctl_flag(&link, CBCTL_FLAG_EV_READY));

    /* A 5 %% target is computed from a KNOWN start, never from a guess. */
    {
        const int start = cbctl_ev_soc_pct(&link);
        CHECK(start > 0);
        lb.ev_soc = (uint8_t)(start + 5);
        for (int i = 0; i < 8; i++) {
            step(&link, &lb, &now, 600u);
        }
        CHECK(cbctl_ev_soc_pct(&link) >= start + 5);
    }

    /* An out-of-range report is rejected rather than believed. */
    lb.ev_soc = 200u;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_ev_soc_pct(&link) == -1);
}

/* A RELAY LEASE MUST NOT OUTLIVE ITS SESSION — this is the relay flipping, reproduced.
 *
 * Observed on the charger: the gun was closed on a session that dropped seconds later, and the
 * lease kept renewing against nothing — 84 renewals, 0 lapses, session long gone. The board's
 * safety layer opens a contactor it has no session for; the next renewal closes it again; the
 * operator hears the relay flipping. Neither side is faulty on its own. */
TEST(a_lease_is_released_when_the_session_ends)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;

    bring_up(&link, &lb, &now);
    lb.cp = 'B';
    lb.secc_active = true;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE));
    CHECK(cbctl_relay_close(&link, CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
    for (int i = 0; i < 6; i++) {
        step(&link, &lb, &now, 400u);
    }
    CHECK((link.relay_state & CBPLC_RELAY_GUN) != 0u);

    /* The session drops. Nothing else changes. */
    lb.secc_active = false;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    printf("    released=%u relay_state=0x%02X\n", link.lease_released_no_session,
           link.relay_state);
    CHECK(link.lease_released_no_session > 0u);
    CHECK((link.relay_state & CBPLC_RELAY_ALL) == 0u);   /* we stopped asking */
}

/* The EV's requested voltage must be visible from PreCharge, not only during delivery — a
 * charger that waits for "energised" before producing volts deadlocks: the EV will not close
 * its contactor until the bus is already near its target. */
TEST(the_ev_target_is_visible_before_energised)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    uint32_t mv = 0u, ma = 0u;

    bring_up(&link, &lb, &now);
    CHECK(!cbctl_ev_target(&link, &mv, &ma));

    lb.cp = 'B';
    lb.secc_active = true;
    lb.ev_tgt_mv = 384000u;
    lb.ev_tgt_ma = 19000u;
    lb.relay_closed_mask = 0u;            /* NOT energised yet — precharge */
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(!cbctl_flag(&link, CBCTL_FLAG_ENERGISED));
    CHECK(cbctl_ev_target(&link, &mv, &ma));
    CHECK(mv == 384000u && ma == 19000u);
    printf("    ev target %umV %umA while not energised\n", mv, ma);
}

/* A truncated line must leave NOTHING behind — including the scalars that used to be written
 * straight through to the link before the guard ever ran. Establish a good target, then feed a
 * line that is plausible, contradictory, and missing its b1_dwell tail. */
TEST(a_truncated_line_applies_nothing_at_all)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    uint32_t mv = 0u, ma = 0u;
    uint32_t truncated_before;
    static const char kPartial[] =
        "cp=A secc_active=0 ev_tgt_mv=1 ev_tgt_ma=2 ev_soc=99 relay_closed=0x07 "
        "relay_safety_opens=4242";

    bring_up(&link, &lb, &now);
    lb.cp = 'B';
    lb.secc_active = true;
    lb.ev_tgt_mv = 384000u;
    lb.ev_tgt_ma = 19000u;
    lb.relay_closed_mask = 0x01u;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_ev_target(&link, &mv, &ma));
    CHECK(mv == 384000u && ma == 19000u);
    CHECK(link.relay_closed_mask == 0x01u);
    truncated_before = link.counters[CBCTL_CTR_STATUS_TRUNCATED];

    cbctl_status_apply(&link, (const uint8_t *)kPartial, sizeof kPartial - 1u, now);

    CHECK(link.counters[CBCTL_CTR_STATUS_TRUNCATED] == truncated_before + 1u);
    /* Every one of these was corrupted by the partial line before the guard was moved. */
    CHECK(cbctl_ev_target(&link, &mv, &ma));
    CHECK(mv == 384000u && ma == 19000u);
    CHECK(link.relay_closed_mask == 0x01u);
    CHECK(link.counters[CBCTL_CTR_RELAY_SAFETY_OPENS] != 4242u);
    printf("    truncated line changed nothing (target %umV, relays 0x%02X)\n",
           mv, link.relay_closed_mask);
}

/* The board's overflow marker is short and legitimately has no b1_dwell. It must still be
 * READ — it is the one message whose whole purpose is to say the diagnostic overflowed. */
TEST(the_overflow_marker_is_counted_not_discarded)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    static const char kMarker[] = "status_overflow=7 cp=B session=Negotiating";

    bring_up(&link, &lb, &now);
    CHECK(link.counters[CBCTL_CTR_STATUS_OVERFLOWS] == 0u);

    cbctl_status_apply(&link, (const uint8_t *)kMarker, sizeof kMarker - 1u, now);

    CHECK(link.counters[CBCTL_CTR_STATUS_OVERFLOWS] == 7u);
    printf("    overflow marker read: %u\n", link.counters[CBCTL_CTR_STATUS_OVERFLOWS]);
}

/* A target from a car that has gone is not a demand. cbplc clears its own target at session
 * START, so between sessions it keeps reporting the last vehicle's request; cbctl cached it
 * forever and a controller would drive real power modules to it. */
TEST(the_ev_target_does_not_outlive_its_session)
{
    cbctl_link_t link;
    cbctl_loopback_t lb;
    uint32_t now = 1000u;
    uint32_t mv = 0u, ma = 0u;

    bring_up(&link, &lb, &now);
    lb.cp = 'B';
    lb.secc_active = true;
    lb.ev_tgt_mv = 356000u;
    lb.ev_tgt_ma = 2000u;
    for (int i = 0; i < 8; i++) {
        step(&link, &lb, &now, 600u);
    }
    CHECK(cbctl_ev_target(&link, &mv, &ma));
    CHECK(mv == 356000u);

    /* Session ends. cbplc keeps echoing the SAME stale target on the wire forever — cbctl must
     * stop believing it. The rule is AGE, not a flag: `secc_active` reads 0 while real DC
     * requests are still flowing, so gating on it blocked the module command entirely and the
     * DCDC never started. An unchanging value simply ages out. */
    lb.secc_active = false;
    lb.ev_demand_idle = true;     /* the car stopped asking: dc_ans stops rising ... */
    for (int i = 0; i < 20; i++) {  /* ... while cbplc keeps echoing the same ev_tgt_mv forever */
        step(&link, &lb, &now, 600u);
    }
    CHECK(!cbctl_ev_target(&link, &mv, &ma));
    printf("    stale EV target dropped at session end\n");
}

int main(void)
{
    printf("status\n");
    RUN(status_is_polled_and_parsed);
    RUN(plugging_in_raises_exactly_one_event);
    RUN(the_vehicle_mac_arrives_at_slac_match_and_is_readable);
    RUN(a_session_total_is_not_a_live_session);
    RUN(a_board_reset_is_reported);
    RUN(the_relay_diagnostic_counters_are_readable);
    RUN(a_key_that_is_a_prefix_of_another_is_not_confused);
    RUN(a_truncated_status_is_rejected_rather_than_half_believed);
    RUN(a_silent_status_ages_the_snapshot_and_degrades_the_link);
    RUN(never_observed_is_distinct_from_freshly_observed);
    RUN(a_close_the_board_keeps_undoing_is_abandoned_loudly);
    RUN(ev_soc_is_reported_and_unknown_is_not_zero);
    RUN(a_lease_is_released_when_the_session_ends);
    RUN(the_ev_target_is_visible_before_energised);
    RUN(a_truncated_line_applies_nothing_at_all);
    RUN(the_overflow_marker_is_counted_not_discarded);
    RUN(the_ev_target_does_not_outlive_its_session);
    if (g_fail) {
        printf("FAIL status (%d)\n", g_fail);
        return 1;
    }
    printf("PASS status\n");
    return 0;
}
