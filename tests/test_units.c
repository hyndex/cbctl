/* Units and time. The age tests are regressions for a defect that killed live sessions. */
#include "cbctl/units.h"

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

/* THE DEFECT, as a test. A stamp written a few ms AFTER `now` was captured — which happens
 * on every pass where a handler uses a live clock — must read as fresh. An unsigned
 * subtraction gives 4294967292 ms here, i.e. 49.7 days, and the caller withdraws permission
 * to energise from the freshest possible feed. */
TEST(a_future_stamp_reads_as_age_zero_not_as_49_days)
{
    const cbctl_instant_t now = cbctl_instant(1000u);
    const cbctl_instant_t stamped_later = cbctl_instant(1004u);
    const cbctl_age_t age = cbctl_age(now, stamped_later);
    CHECK(age.ms == 0);
    CHECK(!cbctl_older_than(age, cbctl_dur(1500u)));
}

/* The same arithmetic across the uint32 wrap, which is the other half of the trap: the
 * subtraction is correct MODULO 2^32, so it must still produce a small positive age. */
TEST(age_is_correct_across_the_wrap)
{
    const cbctl_instant_t before = cbctl_instant(0xFFFFFF00u);
    const cbctl_instant_t after = cbctl_instant(0x00000040u); /* 320 ms later, wrapped */
    const cbctl_age_t age = cbctl_age(after, before);
    CHECK(age.ms == 320);
    CHECK(!cbctl_older_than(age, cbctl_dur(1500u)));
}

TEST(a_genuinely_stale_feed_is_still_detected)
{
    const cbctl_age_t age = cbctl_age(cbctl_instant(10000u), cbctl_instant(8000u));
    CHECK(age.ms == 2000);
    CHECK(cbctl_older_than(age, cbctl_dur(1500u)));   /* past CBPLC_LIMITS_STALE_MS */
}

/* cbctl_reached is the UNSIGNED half, for deadlines nobody re-stamps mid-pass. It must
 * survive the wrap too, or a relay lease set just below 2^32 never expires. */
TEST(a_deadline_across_the_wrap_still_fires)
{
    const cbctl_instant_t deadline = cbctl_add(cbctl_instant(0xFFFFFF00u), cbctl_dur(500u));
    CHECK(!cbctl_reached(cbctl_instant(0xFFFFFF00u), deadline));
    CHECK(cbctl_reached(cbctl_instant(0x00000100u), deadline));  /* wrapped past it */
}

TEST(units_carry_their_values)
{
    CHECK(cbctl_mv(CBCTL_MV(500000u)) == 500000u);
    CHECK(cbctl_ma(CBCTL_MA(200000u)) == 200000u);
    CHECK(cbctl_w(CBCTL_W(100000u)) == 100000u);
}

/* The ceilings are cbplc's. If the firmware ever raises one, this fails and someone looks
 * at whether the controller side agrees, rather than discovering it as a BAD_ARGUMENT on a
 * charger. */
TEST(ceilings_track_the_firmware)
{
    CHECK(CBPLC_LIMITS_MAX_VOLTAGE_MV == 1000000u);
    CHECK(CBPLC_LIMITS_MAX_CURRENT_MA == 1000000u);
    CHECK(CBPLC_LIMITS_MAX_POWER_W == 500000u);
    CHECK(CBPLC_LIMITS_STALE_MS == 1500u);
}

int main(void)
{
    printf("units\n");
    RUN(a_future_stamp_reads_as_age_zero_not_as_49_days);
    RUN(age_is_correct_across_the_wrap);
    RUN(a_genuinely_stale_feed_is_still_detected);
    RUN(a_deadline_across_the_wrap_still_fires);
    RUN(units_carry_their_values);
    RUN(ceilings_track_the_firmware);
    if (g_fail) {
        printf("FAIL units (%d)\n", g_fail);
        return 1;
    }
    printf("PASS units\n");
    return 0;
}
