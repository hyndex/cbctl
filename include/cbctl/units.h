/* cbctl — units and time.
 *
 * The purpose of these types is that a wrong value should not be EXPRESSIBLE, not that it
 * should be caught downstream. C will happily pass a uint32_t of milliamps into a parameter
 * documented as millivolts; it will not pass a cbctl_ma_t into a cbctl_mv_t.
 *
 * Every type here is a single-member struct, so it costs nothing: every ABI cbctl targets
 * passes a one-member struct in a register exactly as it would the bare integer.
 */
#ifndef CBCTL_UNITS_H
#define CBCTL_UNITS_H

#include <stdbool.h>
#include <stdint.h>

#include "cbplc/limits.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Assertions are the last line, not the first. A value that trips one has already escaped
 * the type system, so the default is to abort loudly rather than continue with a number the
 * charger cannot mean. Define CBCTL_NO_ASSERT for a release MCU build. */
#ifdef CBCTL_NO_ASSERT
#define CBCTL_ASSERT(cond) ((void)0)
#else
#include <assert.h>
#define CBCTL_ASSERT(cond) assert(cond)
#endif

/* ---- electrical ---------------------------------------------------------------------- */

typedef struct { uint32_t mv; } cbctl_mv_t; /* millivolts */
typedef struct { uint32_t ma; } cbctl_ma_t; /* milliamps  */
typedef struct { uint32_t w;  } cbctl_w_t;  /* watts      */

/* The ceilings are cbplc's, not ours — see third_party/cbplc/include/cbplc/limits.h. A value
 * past one of them is a units error rather than a charger, and the PLC answers BAD_ARGUMENT
 * and LEAVES THE PREVIOUS ENVELOPE STANDING. That last part is why catching it locally
 * matters: the failure mode of the remote check is running for a week on stale numbers. */
static inline uint32_t cbctl_chk_mv_(uint32_t v)
{
    CBCTL_ASSERT(v <= CBPLC_LIMITS_MAX_VOLTAGE_MV);
    return v;
}
static inline uint32_t cbctl_chk_ma_(uint32_t v)
{
    CBCTL_ASSERT(v <= CBPLC_LIMITS_MAX_CURRENT_MA);
    return v;
}
static inline uint32_t cbctl_chk_w_(uint32_t v)
{
    CBCTL_ASSERT(v <= CBPLC_LIMITS_MAX_POWER_W);
    return v;
}

#define CBCTL_MV(x) ((cbctl_mv_t){ cbctl_chk_mv_((uint32_t)(x)) })
#define CBCTL_MA(x) ((cbctl_ma_t){ cbctl_chk_ma_((uint32_t)(x)) })
#define CBCTL_W(x)  ((cbctl_w_t) { cbctl_chk_w_((uint32_t)(x))  })

/* Accessors, so callers never reach into the struct and the field can be renamed. */
static inline uint32_t cbctl_mv(cbctl_mv_t v) { return v.mv; }
static inline uint32_t cbctl_ma(cbctl_ma_t v) { return v.ma; }
static inline uint32_t cbctl_w (cbctl_w_t  v) { return v.w;  }

/* ---- time ---------------------------------------------------------------------------- */

/* Three types, because two of them are not interchangeable.
 *
 * instant + duration type-checks. instant + instant does not, which is the point. */
typedef struct { uint32_t ms; } cbctl_instant_t; /* a monotonic point in time; wraps at 2^32 */
typedef struct { uint32_t ms; } cbctl_dur_t;     /* a length of time                          */
typedef struct { int32_t  ms; } cbctl_age_t;     /* SIGNED — see cbctl_age() below            */

static inline cbctl_instant_t cbctl_instant(uint32_t ms) { return (cbctl_instant_t){ ms }; }
static inline cbctl_dur_t     cbctl_dur(uint32_t ms)     { return (cbctl_dur_t){ ms };     }

static inline cbctl_instant_t cbctl_add(cbctl_instant_t t, cbctl_dur_t d)
{
    return (cbctl_instant_t){ t.ms + d.ms };   /* wrap is intended and handled by cbctl_age */
}

/* HOW OLD IS A STAMP? Signed, and future-tolerant.
 *
 * This is the single most expensive timing defect this project has shipped, so it gets the
 * long comment. `now` is captured once at the top of a pass; something else stamps with a
 * live clock a few milliseconds later; an UNSIGNED (now - stamp) then wraps to ~4.29 billion
 * and reads the freshest possible feed as 49.7 days expired. Every idle test passed. Every
 * real session died.
 *
 * A stamp in the future means "written after we looked", which is fresh, so it reads as
 * age 0 — never as an epoch of staleness. */
static inline cbctl_age_t cbctl_age(cbctl_instant_t now, cbctl_instant_t stamp)
{
    const int32_t d = (int32_t)(now.ms - stamp.ms);
    return (cbctl_age_t){ d < 0 ? 0 : d };
}

static inline bool cbctl_older_than(cbctl_age_t a, cbctl_dur_t limit)
{
    return a.ms >= 0 && (uint32_t)a.ms >= limit.ms;
}

/* UNSIGNED elapsed, for a deadline NOBODY ELSE STAMPS DURING THE PASS — relay expiry, lease
 * elapsed. Using the signed form here would make every input decide it had been stable
 * forever at the wrap. The two helpers exist separately, and are named differently, because
 * choosing the wrong one is silent in both directions. */
static inline bool cbctl_reached(cbctl_instant_t now, cbctl_instant_t deadline)
{
    return (int32_t)(now.ms - deadline.ms) >= 0;
}

#ifdef __cplusplus
}
#endif
#endif /* CBCTL_UNITS_H */
