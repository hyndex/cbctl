/* F1-F6: the isolation guarantees, each proven by breaking something.
 *
 * A design claim about isolation that has not been tested by wedging a link is an opinion.
 * Every test here therefore damages one link and asserts about the OTHERS.
 */
#include "cbctl/cbctl.h"
#include "loopback.h"

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

#define N 3

typedef struct {
    cbctl_link_t link[N];
    cbctl_loopback_t lb[N];
    cbctl_fleet_t fleet;
    uint32_t now;
} rig_t;

static void charge_like(cbctl_link_t *link)
{
    cbctl_envelope_t env;
    cbctl_measurement_t m;

    memset(&env, 0, sizeof env);
    env.max_voltage = CBCTL_MV(500000u);
    env.max_current = CBCTL_MA(200000u);
    env.max_power = CBCTL_W(100000u);
    env.power_ready = true;
    (void)cbctl_set_envelope(link, &env);

    memset(&m, 0, sizeof m);
    m.present_voltage = CBCTL_MV(400000u);
    m.present_current = CBCTL_MA(100000u);
    m.isolation = CBCTL_ISO_VALID;
    (void)cbctl_set_measurement(link, &m);

    (void)cbctl_authorize(link, CBCTL_AUTH_GRANTED, 8000u);
}

/* One "round" = the fleet ticks, then every board runs. Both halves are bounded. */
static void rig_round(rig_t *r, uint32_t step)
{
    int i;
    cbctl_fleet_tick(&r->fleet, r->now);
    for (i = 0; i < N; i++) {
        cbctl_loopback_poll(&r->lb[i], r->now);
    }
    cbctl_fleet_tick(&r->fleet, r->now);
    for (i = 0; i < N; i++) {
        cbctl_loopback_poll(&r->lb[i], r->now);
    }
    r->now += step;
}

/* rig_up_n charges only the first `n_charging` links; the rest identify and stay genuinely idle
 * — no envelope declared at all, which is what an unplugged gun looks like. rig_up keeps its old
 * meaning (every link charging) so no existing test changes behaviour. */
static void rig_up_n(rig_t *r, int n_charging)
{
    int i;
    memset(r, 0, sizeof *r);
    r->now = 1000u;
    cbctl_fleet_init(&r->fleet);
    for (i = 0; i < N; i++) {
        cbctl_loopback_init(&r->lb[i]);
        /* DISTINCT MACs, because real boards have them. The rig originally left every fake
         * board on the same default identity, and the duplicate-board guard correctly
         * quarantined two of the three — the test setup was the unrealistic part, not the
         * guard. A fleet of clones is precisely what that guard exists to refuse. */
        r->lb[i].board_mac[5] = (uint8_t)(0x50 + i);
        CHECK(cbctl_open(&r->link[i], &r->lb[i].ctrl_tx) == CBCTL_OK);
        CHECK(cbctl_fleet_add(&r->fleet, &r->link[i]) == CBCTL_OK);
    }
    for (i = 0; i < 12; i++) {
        rig_round(r, 10u);
    }
    for (i = 0; i < N; i++) {
        CHECK(cbctl_link_state(&r->link[i]) == CBCTL_LINK_READY);
        if (i < n_charging) {
            charge_like(&r->link[i]);
        }
    }
    for (i = 0; i < 6; i++) {
        rig_round(r, 200u);
    }
}

static void rig_up(rig_t *r)
{
    rig_up_n(r, N);
}

/* F1 + F5 — THE ONE THAT MATTERS.
 *
 * Three boards charging. Board 1 dies mid-charge. Boards 0 and 2 must not notice: their feed
 * must never reach the PLC's 1500 ms staleness bound, because at that bound the PLC withdraws
 * permission to energise and terminates a live session. A driver on gun 0 must not lose their
 * charge because gun 1's board stopped answering. */
TEST(f1_a_dead_board_never_makes_a_sibling_feed_go_stale)
{
    static rig_t r;
    uint32_t worst_sibling_age = 0u;
    int i;

    rig_up(&r);
    r.lb[1].wedged = true; /* board 1 is now a brick */

    for (i = 0; i < 300; i++) {
        rig_round(&r, 200u);
        {
            const uint32_t a0 = cbctl_feed_age_ms(&r.link[0], r.now);
            const uint32_t a2 = cbctl_feed_age_ms(&r.link[2], r.now);
            if (a0 > worst_sibling_age) {
                worst_sibling_age = a0;
            }
            if (a2 > worst_sibling_age) {
                worst_sibling_age = a2;
            }
        }
    }
    printf("    worst sibling feed age with one board dead: %u ms (bound %u)\n",
           worst_sibling_age, (unsigned)CBPLC_LIMITS_STALE_MS);
    CHECK(worst_sibling_age < CBPLC_LIMITS_STALE_MS);

    /* And the survivors are still healthy links, not merely fed. */
    CHECK(cbctl_link_state(&r.link[0]) == CBCTL_LINK_READY);
    CHECK(cbctl_link_state(&r.link[2]) == CBCTL_LINK_READY);
    /* F6: the dead one is contained rather than retried forever. */
    CHECK(cbctl_link_state(&r.link[1]) == CBCTL_LINK_QUARANTINED);
}

/* F2 — a link whose transport refuses every send (a full USB-CDC TX buffer, the classic
 * stall) must not stall the loop for anyone else. */
TEST(f2_a_blocked_transmit_on_one_link_does_not_stall_the_others)
{
    static rig_t r;
    uint32_t worst = 0u;
    int i;

    rig_up(&r);
    r.lb[2].tx_full = true;

    for (i = 0; i < 200; i++) {
        rig_round(&r, 200u);
        {
            const uint32_t a = cbctl_feed_age_ms(&r.link[0], r.now);
            if (a > worst) {
                worst = a;
            }
        }
    }
    CHECK(worst < CBPLC_LIMITS_STALE_MS);
    CHECK(cbctl_link_state(&r.link[0]) == CBCTL_LINK_READY);
}

/* F3 — work per tick is bounded no matter what a board does. Asserted through the counter the
 * link keeps for exactly this: hitting the cap is allowed, but the tick still RETURNS. */
TEST(f3_work_per_tick_stays_bounded_under_a_flood)
{
    static rig_t r;
    int i;

    rig_up(&r);
    for (i = 0; i < 100; i++) {
        rig_round(&r, 50u); /* far faster than the feed period: maximum pressure */
    }
    /* The real assertion is that we got here. A tick with no ceiling would not return. */
    for (i = 0; i < N; i++) {
        CHECK(cbctl_link_state(&r.link[i]) == CBCTL_LINK_READY);
    }
}

/* F4 — fairness. With a fixed 0..n-1 sweep the last link is always serviced last, which is a
 * systematic bias rather than noise. Every link must get comparable service. */
TEST(f4_no_link_is_starved_by_its_position_in_the_array)
{
    static rig_t r;
    unsigned lo, hi;
    int i;

    rig_up(&r);
    for (i = 0; i < 120; i++) {
        rig_round(&r, 400u);
    }
    lo = r.lb[0].limits_seen;
    hi = r.lb[0].limits_seen;
    for (i = 0; i < N; i++) {
        if (r.lb[i].limits_seen < lo) {
            lo = r.lb[i].limits_seen;
        }
        if (r.lb[i].limits_seen > hi) {
            hi = r.lb[i].limits_seen;
        }
    }
    printf("    limits per link: %u..%u\n", lo, hi);
    CHECK(lo > 0u);
    /* Within 25 %: exact equality is not the claim, absence of systematic starvation is. */
    CHECK((hi - lo) * 4u <= hi);
}

/* F5 — one link's fault changes no other link's state. Explicitly: quarantining board 1 must
 * not touch board 0's relay lease, which is the thing physically holding a contactor. */
TEST(f5_one_links_quarantine_does_not_disturb_anothers_relay_lease)
{
    static rig_t r;
    uint32_t renewals_before;
    int i;

    rig_up(&r);
    CHECK(cbctl_relay_close(&r.link[0], CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
    for (i = 0; i < 10; i++) {
        rig_round(&r, 200u);
    }
    renewals_before = r.link[0].relay_renewals;
    CHECK(renewals_before > 0u);

    r.lb[1].wedged = true;
    /* Quarantine needs CBCTL_LOSS_LIMIT consecutive losses, and each loss costs a full
     * request timeout plus its retries (~3.3 s). 20 s was marginal; 40 s is not. Sizing a
     * test to the mechanism rather than to a round number is the difference between a gate
     * and a coin flip. */
    for (i = 0; i < 200; i++) {
        rig_round(&r, 200u);
    }
    CHECK(cbctl_link_state(&r.link[1]) == CBCTL_LINK_QUARANTINED);
    /* Board 0's contactor kept being renewed throughout board 1's death and quarantine. */
    CHECK(r.link[0].relay_renewals > renewals_before + 5u);
}

/* F6 — THE MIRROR OF F5, AND THE ONE THAT WAS MISSING.
 *
 * F5 wedges the IDLE board while the OTHER holds a contactor. Every multi-link test in this file
 * is shaped that way: the fault always lands on the link that is not carrying current. That left
 * the opposite arrangement — the board that is ITSELF holding a closed contactor goes silent —
 * with no coverage at all, and it is the more common field event: a gun mid-charge is precisely
 * the gun whose USB is loaded, whose PLC is busiest, and whose board re-enumerates or WDT-resets.
 *
 * It is also the arrangement that used to take the whole process down. A lease renewal is
 * answered by nothing, the session delivers (NULL, CBPLC_E_TIMEOUT), and the reply path
 * dereferenced that NULL while classifying the relay ACK. In one process driving two links, that
 * is not a gun-0 problem — it is a SIGSEGV, so gun 1's link stops ticking too, its feed goes
 * stale past CBPLC_LIMITS_STALE_MS, and its board drops a contactor that was carrying current.
 * One gun's fault taking the other gun's charge down is the exact property this file exists to
 * deny, so it is asserted here rather than left to the single-link suite. */
TEST(f6_a_fault_on_the_link_holding_the_contactor_spares_the_other)
{
    static rig_t r;
    uint32_t sibling_limits_before, worst_sibling_age = 0u;
    int i;

    rig_up(&r);
    /* Both guns charging at once — the real dual-gun steady state, which nothing else covers. */
    CHECK(cbctl_relay_close(&r.link[0], CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
    CHECK(cbctl_relay_close(&r.link[1], CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
    for (i = 0; i < 10; i++) {
        rig_round(&r, 200u);
    }
    CHECK(r.link[0].relay_acks > 0u);
    CHECK(r.link[1].relay_acks > 0u);
    sibling_limits_before = r.link[1].limits_sent;

    /* The board that is HOLDING THE CLOSED CONTACTOR goes silent. */
    r.lb[0].wedged = true;

    for (i = 0; i < 200; i++) {
        rig_round(&r, 200u);
        if (cbctl_link_state(&r.link[1]) == CBCTL_LINK_READY) {
            const uint32_t age = cbctl_feed_age_ms(&r.link[1], r.now);
            if (age != CBCTL_FEED_NEVER_SENT && age > worst_sibling_age) {
                worst_sibling_age = age;
            }
        }
    }

    printf("    gun0 wedged while holding; gun1 worst feed age=%ums (bound %u) acks=%u\n",
           worst_sibling_age, (unsigned)CBPLC_LIMITS_STALE_MS, r.link[1].relay_acks);

    /* Survived — no NULL dereference on the dead link's loss path. */

    /* The dead gun is contained. */
    CHECK(cbctl_link_state(&r.link[0]) == CBCTL_LINK_QUARANTINED);

    /* THE CLAIM: the surviving gun never noticed. Its feed stayed inside the deadline the BOARD
     * opens a relay at, so its contactor was never at risk. */
    CHECK(worst_sibling_age < CBPLC_LIMITS_STALE_MS);
    CHECK(cbctl_link_state(&r.link[1]) == CBCTL_LINK_READY);
    CHECK(r.link[1].limits_sent > sibling_limits_before);

    /* And its lease kept being CONFIRMED, not merely attempted — the distinction that decides
     * whether a contactor is actually still held. */
    CHECK(r.link[1].relay_acks > 0u);
    CHECK(r.link[1].relay_refusals == 0u);
}

/* F7 — DOES A SECOND (AND THIRD) CHARGING GUN COST THE FIRST ONE ANYTHING?
 *
 * This is the question a two-gun charger is actually bought on, and until now nothing answered
 * it. F1 has three links charging but asserts only that a DEAD board spares the others. F6
 * holds two leases but exists to catch a crash. Neither measures what a HEALTHY second gun costs
 * a healthy first one, which is the number that decides whether the design scales.
 *
 * So measure the same link twice: once as the only gun drawing, once with every gun drawing, and
 * compare. The claim is not "the margins are adequate" — that would pass on a design that
 * degrades badly but started with slack. The claim is that the numbers DO NOT MOVE, because each
 * gun has its own USB device and its own stop-and-wait session, so nothing about link 0's wire
 * is shared with link 1's. Only CPU time in the loop is shared, and that is microseconds.
 *
 * If this ever fails, the fleet has acquired a shared resource it did not have, and the
 * single-loop design needs revisiting. That is worth a test on its own. */
TEST(f7_more_charging_guns_do_not_cost_the_first_gun_margin)
{
    static rig_t r;
    uint32_t solo_feed = 0u, solo_renew_gap = 0u;
    uint32_t all_feed = 0u, all_renew_gap = 0u;
    uint32_t last_ack_at, hold;
    int i, k;

    /* ---- PART 1: link 0 is the ONLY gun charging. ---- */
    rig_up_n(&r, 1);   /* only gun 0 declares anything; 1 and 2 are unplugged guns */
    CHECK(cbctl_relay_close(&r.link[0], CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
    hold = r.link[0].relay_hold_ms;
    last_ack_at = 0u;
    for (i = 0; i < 300; i++) {
        rig_round(&r, 100u);
        {
            const uint32_t age = cbctl_feed_age_ms(&r.link[0], r.now);
            if (age != CBCTL_FEED_NEVER_SENT && age > solo_feed) { solo_feed = age; }
        }
        if (r.link[0].relay_acks > 0u && r.link[0].last_relay.ms != last_ack_at) {
            if (last_ack_at != 0u) {
                const uint32_t g = r.link[0].last_relay.ms - last_ack_at;
                if (g > solo_renew_gap) { solo_renew_gap = g; }
            }
            last_ack_at = r.link[0].last_relay.ms;
        }
    }

    /* ---- PART 2: EVERY gun charging, same link measured. ---- */
    rig_up(&r);
    for (k = 0; k < N; k++) {
        CHECK(cbctl_relay_close(&r.link[k], CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);
        r.lb[k].secc_active = true;   /* a live session, not just a lease */
    }
    last_ack_at = 0u;
    for (i = 0; i < 300; i++) {
        rig_round(&r, 100u);
        {
            const uint32_t age = cbctl_feed_age_ms(&r.link[0], r.now);
            if (age != CBCTL_FEED_NEVER_SENT && age > all_feed) { all_feed = age; }
        }
        if (r.link[0].relay_acks > 0u && r.link[0].last_relay.ms != last_ack_at) {
            if (last_ack_at != 0u) {
                const uint32_t g = r.link[0].last_relay.ms - last_ack_at;
                if (g > all_renew_gap) { all_renew_gap = g; }
            }
            last_ack_at = r.link[0].last_relay.ms;
        }
    }

    printf("    gun0 alone : feed<=%ums  confirmed-renewal gap<=%ums\n", solo_feed, solo_renew_gap);
    printf("    %d charging: feed<=%ums  confirmed-renewal gap<=%ums  (hold %ums, stale %u)\n",
           N, all_feed, all_renew_gap, hold, (unsigned)CBPLC_LIMITS_STALE_MS);

    /* Every gun kept its own lease confirmed, not merely attempted. */
    for (k = 0; k < N; k++) {
        CHECK(cbctl_link_state(&r.link[k]) == CBCTL_LINK_READY);
        CHECK(r.link[k].relay_acks > 10u);
        CHECK(r.link[k].relay_refusals == 0u);
        CHECK(cbctl_counter(&r.link[k], CBCTL_CTR_RELAY_LEASE_OPENS) == 0u);
    }

    /* THE CLAIM: gun count does not move the numbers. Not "still under the limit" — UNCHANGED.
     * One tick of slack (the rig steps 100 ms) is the tolerance; anything systematic exceeds it. */
    CHECK(all_feed <= solo_feed + 100u);
    CHECK(all_renew_gap <= solo_renew_gap + 100u);

    /* And both are far inside the deadlines, so the margin is real and not merely non-negative. */
    CHECK(all_feed * 3u < CBPLC_LIMITS_STALE_MS);
    CHECK(all_renew_gap * 2u < hold);
}

/* A duplicate link would be serviced twice per tick, doubling its own feed and halving
 * everyone else's share of the quantum. */
TEST(a_duplicate_link_is_refused)
{
    static rig_t r;

    rig_up(&r);
    CHECK(cbctl_fleet_add(&r.fleet, &r.link[0]) == CBCTL_E_ARG);
}

/* One nearest deadline for the whole fleet, never an invitation to sleep past the feed. */
TEST(the_fleet_deadline_is_the_nearest_one_and_is_never_too_long)
{
    static rig_t r;
    int i;

    rig_up(&r);
    for (i = 0; i < 30; i++) {
        const uint32_t d = cbctl_fleet_next_deadline_ms(&r.fleet, r.now);
        CHECK(d <= 400u);
        rig_round(&r, 137u);
    }
}

/* A LINK WHOSE TRANSPORT DIES MUST BE QUARANTINED, NOT LEFT DEGRADED FOREVER.
 *
 * Quarantine used to be driven only by request TIMEOUTS, so a link with a dead fd never
 * reached it: nothing is ever in flight, so nothing times out. It sat in DEGRADED being
 * retried at full rate on every tick — a busy-wait against a hole, and an operator signal
 * that says "our picture is stale" when the truth is "this connector is gone".
 *
 * Found by killing one real board's transport mid-run on the bench. */
TEST(a_dead_transport_is_quarantined_and_the_sibling_is_untouched)
{
    static rig_t r;
    uint32_t worst_survivor = 0u;
    int i;

    rig_up(&r);
    CHECK(cbctl_relay_close(&r.link[0], CBPLC_RELAY_GUN, 3000u, NULL) == CBCTL_OK);

    /* Link 1's transport now fails hard on every call, as a closed fd does. */
    r.lb[1].transport_dead = true;

    for (i = 0; i < 200; i++) {
        rig_round(&r, 200u);
        {
            const uint32_t a = cbctl_feed_age_ms(&r.link[0], r.now);
            if (a > worst_survivor) {
                worst_survivor = a;
            }
        }
    }

    printf("    dead-transport link: state=%d transport_failures=%u | survivor worst_feed=%ums\n",
           (int)cbctl_link_state(&r.link[1]), r.link[1].transport_failures, worst_survivor);

    CHECK(r.link[1].transport_failures > 0u);
    CHECK(cbctl_link_state(&r.link[1]) == CBCTL_LINK_QUARANTINED);
    /* The survivor is the point: untouched feed, and its contactor still held. */
    CHECK(worst_survivor < CBPLC_LIMITS_STALE_MS);
    CHECK(cbctl_link_state(&r.link[0]) == CBCTL_LINK_READY);
    CHECK(r.link[0].relay_acks > 5u);
}

/* feed_late counts EXCURSIONS, not ticks. A dead link once reported 6,024,729 of them, which
 * buries the case that actually matters: a healthy link that went late once. */
TEST(feed_late_counts_excursions_not_ticks)
{
    static rig_t r;
    int i;

    rig_up(&r);
    r.lb[1].transport_dead = true;
    for (i = 0; i < 300; i++) {
        rig_round(&r, 200u);
    }
    printf("    dead link feed_late=%u over 300 ticks\n", r.link[1].feed_late);
    /* One excursion, not hundreds. */
    CHECK(r.link[1].feed_late <= 3u);
}

int main(void)
{
    printf("fleet\n");
    RUN(f1_a_dead_board_never_makes_a_sibling_feed_go_stale);
    RUN(f2_a_blocked_transmit_on_one_link_does_not_stall_the_others);
    RUN(f3_work_per_tick_stays_bounded_under_a_flood);
    RUN(f4_no_link_is_starved_by_its_position_in_the_array);
    RUN(f5_one_links_quarantine_does_not_disturb_anothers_relay_lease);
    RUN(f6_a_fault_on_the_link_holding_the_contactor_spares_the_other);
    RUN(f7_more_charging_guns_do_not_cost_the_first_gun_margin);
    RUN(a_duplicate_link_is_refused);
    RUN(the_fleet_deadline_is_the_nearest_one_and_is_never_too_long);
    RUN(a_dead_transport_is_quarantined_and_the_sibling_is_untouched);
    RUN(feed_late_counts_excursions_not_ticks);
    if (g_fail) {
        printf("FAIL fleet (%d)\n", g_fail);
        return 1;
    }
    printf("PASS fleet\n");
    return 0;
}
