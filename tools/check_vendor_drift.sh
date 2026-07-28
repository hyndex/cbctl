#!/bin/sh
# Is the vendored cbplc core still identical to the firmware's?
#
# WHY THIS EXISTS
#   cbctl deliberately vendors cbplc's message/segment/session/stream_frame/limits/auth/safety
#   sources rather than reimplementing them, so the controller and the board cannot disagree
#   about the wire. Vendoring buys that guarantee and introduces exactly one new way to lose
#   it: somebody edits the firmware's copy and this one silently goes stale.
#
#   The failure mode is the worst kind. Nothing breaks at build time. The tests keep passing,
#   because they exercise the vendored copy against itself through the loopback. It only shows
#   up as a charger that will not talk to a board, on hardware, later.
#
#   So the check is mechanical and runs with the tests.
#
# WHEN IT FIRES
#   Re-copy the listed files from ../cbplc and re-run the suite. If a change was intentional
#   and the wire format moved, the schema hash in protocol_generated.h moves with it and both
#   sides must be rebuilt anyway.
#
# If ../cbplc is not present (cbctl used standalone), this SKIPS rather than fails: a consumer
# who vendored cbctl into their own tree has no firmware repo to compare against.
set -eu

here=$(cd "$(dirname "$0")/.." && pwd)
src=${CBPLC_SRC:-"$here/../cbplc"}

if [ ! -d "$src/src" ]; then
    echo "vendor-drift: ../cbplc not present — skipping (standalone checkout)"
    exit 0
fi

status=0

for f in message segment session stream_frame limits auth safety diag; do
    a="$src/src/$f.c"
    b="$here/third_party/cbplc/src/$f.c"
    [ -f "$a" ] || continue
    if ! diff -q "$a" "$b" >/dev/null 2>&1; then
        echo "vendor-drift: src/$f.c differs from $src/src/$f.c"
        status=1
    fi
done

for h in message segment session stream_frame transport protocol_generated limits auth safety diag; do
    a="$src/include/cbplc/$h.h"
    b="$here/third_party/cbplc/include/cbplc/$h.h"
    [ -f "$a" ] || continue
    if ! diff -q "$a" "$b" >/dev/null 2>&1; then
        echo "vendor-drift: include/cbplc/$h.h differs from $src/include/cbplc/$h.h"
        status=1
    fi
done

if [ "$status" -eq 0 ]; then
    echo "vendor-drift: none — the vendored core matches cbplc"
else
    echo "vendor-drift: RE-COPY the files above from $src, then re-run the tests."
fi
exit "$status"
