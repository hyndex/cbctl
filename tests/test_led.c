/* The driver-facing preset. Every rule here is an ordering decision, so the tests are about
 * what wins when two conditions are true at once. */
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

static void step(cbctl_link_t *l, cbctl_loopback_t *b, uint32_t *n, uint32_t d)
{
    cbctl_tick(l, *n);
    cbctl_loopback_poll(b, *n);
    cbctl_tick(l, *n);
    cbctl_loopback_poll(b, *n);
    *n += d;
}

static void settle(cbctl_link_t *l, cbctl_loopback_t *b, uint32_t *n)
{
    for (int i = 0; i < 8; i++) {
        step(l, b, n, 600u);
    }
}

static void up(cbctl_link_t *l, cbctl_loopback_t *b, uint32_t *n)
{
    cbctl_loopback_init(b);
    b->cp = 'A';
    CHECK(cbctl_open(l, &b->ctrl_tx) == CBCTL_OK);
    for (int i = 0; i < 10 && cbctl_link_state(l) != CBCTL_LINK_READY; i++) {
        step(l, b, n, 10u);
    }
    settle(l, b, n);
}

TEST(the_ordinary_progression)
{
    cbctl_link_t l;
    cbctl_loopback_t b;
    uint32_t n = 1000u;

    /* Before anything is known. */
    memset(&l, 0, sizeof l);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_BOOTING);

    up(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_AVAILABLE);

    b.cp = 'B';
    settle(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_PREPARING);

    b.slac_matched = true;
    b.secc_active = true;
    settle(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_PREPARING); /* session, but no contactor yet */

    b.relay_closed_mask = CBPLC_RELAY_GUN;
    settle(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_CHARGING);

    CHECK(cbctl_begin_stop(&l) == CBCTL_OK);
    settle(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_FINISHING);
}

/* A closed contactor with no session is NOT charging — it is precharge or a fault, and either
 * way the driver has not started getting energy. */
TEST(a_contactor_without_a_session_is_not_charging)
{
    cbctl_link_t l;
    cbctl_loopback_t b;
    uint32_t n = 1000u;

    up(&l, &b, &n);
    b.cp = 'B';
    b.relay_closed_mask = CBPLC_RELAY_GUN;
    b.secc_active = false;
    settle(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) != CBCTL_LED_CHARGING);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_PREPARING);
}

/* THE EXPENSIVE ERROR. A stale picture must never show CHARGING: the driver walks away and
 * returns to an empty battery. Showing FAULTED when the charger is secretly fine costs an
 * unnecessary unplug — that asymmetry is the whole rule. */
TEST(a_stale_observation_never_claims_charging)
{
    cbctl_link_t l;
    cbctl_loopback_t b;
    uint32_t n = 1000u;

    up(&l, &b, &n);
    b.cp = 'B';
    b.secc_active = true;
    b.relay_closed_mask = CBPLC_RELAY_GUN;
    settle(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_CHARGING);

    /* STATUS goes quiet. Nothing else changes. */
    b.swallow_type = (uint8_t)CBPLC_T_STATUS_REQUEST;
    for (int i = 0; i < 20; i++) {
        step(&l, &b, &n, 600u);
    }
    CHECK(cbctl_observation_age_ms(&l, n) >= CBCTL_OBSERVATION_STALE_MS);
    CHECK(cbctl_led_preset(&l, n) != CBCTL_LED_CHARGING);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_FAULTED);
}

/* The safety chain outranks everything, including an active charge. */
TEST(the_safety_chain_outranks_a_live_charge)
{
    cbctl_link_t l;
    cbctl_loopback_t b;
    uint32_t n = 1000u;

    up(&l, &b, &n);
    b.cp = 'B';
    b.secc_active = true;
    b.relay_closed_mask = CBPLC_RELAY_GUN;
    settle(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_CHARGING);

    b.safety_asserted = true;
    settle(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_EMERGENCY);
}

/* A connector we cannot drive is FAULTED, not AVAILABLE — telling a driver to plug into a
 * connector the controller has lost is worse than telling them it is broken. */
TEST(a_quarantined_link_is_faulted_not_available)
{
    cbctl_link_t l;
    cbctl_loopback_t b;
    uint32_t n = 1000u;

    up(&l, &b, &n);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_AVAILABLE);

    b.transport_dead = true;
    for (int i = 0; i < 60 && cbctl_link_state(&l) != CBCTL_LINK_QUARANTINED; i++) {
        step(&l, &b, &n, 400u);
    }
    CHECK(cbctl_link_state(&l) == CBCTL_LINK_QUARANTINED);
    CHECK(cbctl_led_preset(&l, n) == CBCTL_LED_FAULTED);
}

/* The names must match the vocabulary the rest of the codebase already sends, or the value
 * needs a translation table that nobody maintains. */
TEST(the_names_match_the_existing_led_command_vocabulary)
{
    CHECK(strcmp(cbctl_led_name(CBCTL_LED_BOOTING), "booting") == 0);
    CHECK(strcmp(cbctl_led_name(CBCTL_LED_AVAILABLE), "available") == 0);
    CHECK(strcmp(cbctl_led_name(CBCTL_LED_PREPARING), "preparing") == 0);
    CHECK(strcmp(cbctl_led_name(CBCTL_LED_CHARGING), "charging") == 0);
    CHECK(strcmp(cbctl_led_name(CBCTL_LED_FINISHING), "finishing") == 0);
    CHECK(strcmp(cbctl_led_name(CBCTL_LED_FAULTED), "faulted") == 0);
    CHECK(strcmp(cbctl_led_name(CBCTL_LED_EMERGENCY), "emergency") == 0);
    /* An out-of-range value must not produce a reassuring name. */
    CHECK(strcmp(cbctl_led_name((cbctl_led_t)99), "faulted") == 0);
}

int main(void)
{
    printf("led\n");
    RUN(the_ordinary_progression);
    RUN(a_contactor_without_a_session_is_not_charging);
    RUN(a_stale_observation_never_claims_charging);
    RUN(the_safety_chain_outranks_a_live_charge);
    RUN(a_quarantined_link_is_faulted_not_available);
    RUN(the_names_match_the_existing_led_command_vocabulary);
    if (g_fail) {
        printf("FAIL led (%d)\n", g_fail);
        return 1;
    }
    printf("PASS led\n");
    return 0;
}
