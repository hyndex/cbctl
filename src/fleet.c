/* N PLCs on one loop.
 *
 * There is almost nothing here, and that is the design. The fleet owns an array and a
 * rotation index; every link is an independent state machine that shares no mutable state
 * with any other. Isolation you can point at in fifty lines is isolation that survives the
 * next refactor.
 */
#include "cbctl/cbctl.h"

#include <string.h>

void cbctl_fleet_init(cbctl_fleet_t *fleet)
{
    if (fleet != NULL) {
        memset(fleet, 0, sizeof *fleet);
    }
}

cbctl_status_t cbctl_fleet_add(cbctl_fleet_t *fleet, cbctl_link_t *link)
{
    unsigned i;

    if (fleet == NULL || link == NULL || fleet->n >= CBCTL_FLEET_MAX) {
        return CBCTL_E_ARG;
    }
    /* The same link twice would be serviced twice per tick and would double its own feed
     * rate while halving everyone else's share of the quantum. Refused rather than tolerated:
     * two readers on one port stealing each other's replies is a failure this project has
     * already paid for on the Python side. */
    for (i = 0u; i < fleet->n; i++) {
        if (fleet->links[i] == link) {
            return CBCTL_E_ARG;
        }
    }
    fleet->links[fleet->n++] = link;
    return CBCTL_OK;
}

/* TWO LINKS MUST NEVER BE THE SAME BOARD.
 *
 * It happens for mundane reasons — a duplicated device path, a udev symlink pointing at the
 * same tty as the raw node, a config copied for gun 2 and not edited. The result is a charger
 * that believes it has two guns, feeds one board twice, and closes "gun 2's" contactor on
 * gun 1. Nothing in the protocol objects, because both links are talking to a healthy board.
 *
 * Identity only arrives asynchronously, so this is checked on the tick rather than at add. */
static void check_for_duplicate_boards(cbctl_fleet_t *fleet)
{
    unsigned i, j;

    for (i = 0u; i < fleet->n; i++) {
        cbctl_identity_t a;
        if (!cbctl_identity(fleet->links[i], &a)) {
            continue;
        }
        for (j = i + 1u; j < fleet->n; j++) {
            cbctl_identity_t b;
            if (!cbctl_identity(fleet->links[j], &b)) {
                continue;
            }
            if (memcmp(a.mac, b.mac, 6u) == 0) {
                /* Quarantine the LATER one: the first link to claim a board keeps it, and the
                 * duplicate is contained loudly rather than silently driving a second gun that
                 * does not exist. */
                if (!fleet->links[j]->quarantine_permanent) {
                    fleet->links[j]->state = CBCTL_LINK_QUARANTINED;
                    fleet->links[j]->quarantine_permanent = true;
                    fleet->duplicate_boards++;
                }
            }
        }
    }
}

cbctl_status_t cbctl_fleet_tick(cbctl_fleet_t *fleet, uint32_t now_ms)
{
    unsigned k;

    if (fleet == NULL) {
        return CBCTL_E_ARG;
    }
    if (fleet->n == 0u) {
        return CBCTL_OK;
    }

    /* ROTATION, not a plain 0..n-1 sweep.
     *
     * A fixed order is unfair in a way that is systematic rather than random: link 0 is
     * serviced first on every single tick, so when the loop runs late it is always the last
     * link that suffers. Over thousands of ticks that is a bias, not noise. Rotating the
     * start costs one modulo and turns it into an even spread. */
    for (k = 0u; k < fleet->n; k++) {
        const unsigned i = (fleet->rr + k) % fleet->n;
        /* THE RESULT IS DELIBERATELY IGNORED. DO NOT "FIX" THIS.
         *
         * Propagating it — `if (rc != CBCTL_OK) return rc;` — looks like good practice and
         * silently destroys the entire multi-PLC guarantee: the failing link is serviced
         * before its siblings on some rotations, so returning early starves them. Measured,
         * by reintroducing exactly that line: the worst sibling feed age goes from 400 ms to
         * 2000 ms, i.e. past CBPLC_LIMITS_STALE_MS, at which point those PLCs correctly
         * withdraw permission to energise and end their charging sessions.
         *
         * A quarantined or failing link is that LINK's state. Callers ask each link how it is
         * doing; the fleet guarantees only that every link got its turn. */
        (void)cbctl_tick(fleet->links[i], now_ms);
    }
    fleet->rr = (fleet->rr + 1u) % fleet->n;
    check_for_duplicate_boards(fleet);
    return CBCTL_OK;
}

uint32_t cbctl_fleet_next_deadline_ms(const cbctl_fleet_t *fleet, uint32_t now_ms)
{
    uint32_t best = 0xFFFFFFFFu;
    unsigned i;

    if (fleet == NULL || fleet->n == 0u) {
        return 0xFFFFFFFFu;
    }
    for (i = 0u; i < fleet->n; i++) {
        const uint32_t d = cbctl_next_deadline_ms(fleet->links[i], now_ms);
        if (d < best) {
            best = d;
        }
    }
    return best;
}

uint32_t cbctl_feed_age_ms(const cbctl_link_t *link, uint32_t now_ms)
{
    if (link == NULL || !link->have_envelope) {
        return 0u; /* nothing declared: there is no feed to be stale */
    }
    if (link->last_limits.ms == 0u) {
        /* DECLARED BUT NEVER SENT IS NOT "FRESH".
         *
         * This returned 0 — indistinguishable from a feed sent microseconds ago — and it hid a
         * real fault on hardware: cbctl was sending no LIMITS at all, the board reported
         * limits_n=0 and refused energy with limits_stale, and this function cheerfully said
         * feed=0ms the entire time. Same mistake as reporting an unobserved snapshot as age 0,
         * which CBCTL_NEVER_OBSERVED already exists to prevent. */
        return CBCTL_FEED_NEVER_SENT;
    }
    return (uint32_t)cbctl_age(cbctl_instant(now_ms), link->last_limits).ms;
}
