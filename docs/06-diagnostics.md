# Reading a board's log, and arming the V2G trace

## What this is for

A cbplc board keeps every line of its diagnostic text in a 16 KB RAM ring rather than printing
it, because the control wire carries framed binary and one text line landing mid-stream reads as
a garbage length prefix on the host. `CBPLC_T_DIAG` drains that ring over the protocol, framed
and CRC'd like everything else.

What was missing was a way to say *what* the board should record. Everything it chose to emit
went in unconditionally and the only control was a recompile. That is the wrong shape for the
question this exists to answer: **why did *that* vehicle refuse a charger that works with every
other one.** Answering it needs a per-message decoded HLC trace — roughly 100 bytes a message,
40–60 messages a session — and nobody wants that firehose running on a production charger that
is working.

## Quick start

```
make boardlog
./build/board_log /dev/ttyACM1 60 --v2g          # arm the trace and tail it for a minute
./build/board_log /dev/ttyACM1 60 --level=warn   # quieten a chatty board and watch for trouble
./build/board_log /dev/ttyACM1 60 --level=off    # record nothing at all
```

`board_log` never authorises, never closes a relay and never commands a module. It cannot start
or stop a charge, and it is safe to run against a charger that is delivering.

For a charge run, `charge_soc` takes it inline:

```
CBCTL_DIAG_LOG=/tmp/v2g.log CBCTL_DIAG_V2G=1 ./build/charge_soc /dev/ttyACM1 can0 52 5 /dev/serial0 1
```

It clears the ring at the start so one session's trace cannot be prefixed with the tail of the
previous vehicle's, and drains what accumulated during the charge in the wind-down.

## The one rule: diagnostics are allowed to starve

This is a deliberate inversion of the treatment `STATUS` gets, and the reason is worth stating
because the codebase carries a warning that appears to say the opposite.

`STATUS` was given the right to age past `CBCTL_STATUS_STARVE_MS` and jump the queue, because a
controller that goes blind mid-charge is dangerous — measured under load: limits 60, feedback
60, heartbeat 33, relay 20, **STATUS 1 across 24 seconds**. The note on that fix says not to add
another low-priority message without the same treatment.

`DIAG` is the case where that warning does not apply, and applying it would be actively wrong.
cbplc is stop-and-wait: one outstanding request, and a request occupies the wire for a full
round trip — up to `CBCTL_WORST_BLOCK_MS` (750 ms) against a 1500 ms feed bound and a lease
renewing at hold/6. Spending that on log text during a charge that is *working* buys nothing.

So:

| | STATUS | DIAG |
|---|---|---|
| priority | below the feed and the heartbeat at its normal 1 s cadence, but **ages upward**: past `CBCTL_STATUS_STARVE_MS` (3 s) it is promoted above the feed for one message | strictly last, never ages |
| under load | guaranteed to get through | expected to be starved to zero |
| while energy is live | required | **the drain is suppressed**; level/category control still flows |

Suppression is the load-bearing property and it has a test: `no_drain_competes_with_a_live_charge`
asserts zero drains while a contactor is held or a session is open, and removing the guard makes
it fail (measured: 0 → 3 drains). Nothing is lost by waiting — the board's ring holds the whole
session, and the sessions worth reading are the ones that *failed*, by which point the vehicle
has stopped talking and there is no contention left.

## Cadence

An empty reply doubles the polling interval up to `CBCTL_DIAG_PERIOD_MAX_MS` (8 s); any content
collapses it back to `CBCTL_DIAG_PERIOD_MIN_MS` (250 ms). An idle board therefore costs about
seven round trips a minute, while a board holding a session's worth of trace hands it over at
the floor — measured: 2499 bytes drained in 23 ticks rather than the ~240 the ceiling would
have taken.

## Silence is not the same as loss

`cbctl_diag_state()` reports the board's own `dropped` counter: bytes the ring overwrote because
nobody drained fast enough. **A trace whose drop count is non-zero has holes in it and is not
evidence.** `board_log` says so explicitly at exit, and `charge_soc` prints a warning.

The function returns `false` — never zeros — before the board has ever answered, for the same
reason `cbctl_ev_soc_pct()` returns −1 rather than 0: "we have not been told" and "the board
reports nothing dropped" are different facts.

`dropped` and `pending` are refreshed every `CBCTL_DIAG_STATE_PERIOD_MS`. Without that they
freeze at whatever the last setter's reply carried — observed on hardware reporting
`pending=520` immediately after 520 bytes had been drained.

## Talking to an older board

A firmware built before the subcommands existed answers **any** DIAG request by draining, so a
control request comes back as log text. There is no version negotiation: the control reply
begins with a NUL, which `cbplc_diag_read()` cannot produce because it returns printf output, so
"is this a control reply?" is decidable from the payload alone.

Meeting such a board, cbctl records the verdict once, stops spending round trips on setters, and
**keeps the text** — those bytes are real log content the board handed over, and discarding them
would silently lose the only diagnostics it can give. `cbctl_diag_control_supported()` is how a
caller tells "logging is off" from "I cannot turn it on".

## Nothing persists across a reboot

cbplc deliberately does not save the level or the category mask. A charger silently retaining a
debug firehose through a power cycle is how a diagnostic becomes an incident. cbctl re-asserts
both from `cbctl_link_reestablish()` when it detects a board reset — otherwise a controller that
armed a trace, whose board then reset, would sit reading the silence as evidence.

## Categories

| bit | name | default | notes |
|---|---|---|---|
| 0x01 | `CORE` | on | every pre-existing log site, plus the SDK's own output |
| 0x02 | `V2G` | **off** | the per-message decoded HLC trace — the firehose |
| 0x04 | `SLAC` | on | |
| 0x08 | `CP` | on | |
| 0x10 | `RELAY` | on | |

Filtering happens on the **write** side. A suppressed line never reaches the ring, so it cannot
push out the lines that were asked for — filtering at drain time would have been no filter at
all.

## What a V2G trace line carries

```
v2g ISO2/35 CurrentDemand age=1204 -> rc=0 len=118 tv=389100mV ti=20000mA soc=27 rdy=1 pause=0 energy=1 auth=2 dcans=1454
```

Every field answers a question that has cost a bench session: which message and which stack (a
DIN car and an ISO car fail differently), how long the session has sat in this phase (a vehicle
re-asking the same message for 40 s reads as "stuck" and nothing else says so), what the car
asked for, whether it declared itself ready, and what we answered. `rc` and `len` together
separate "we refused" from "we answered and the car left anyway".

It is produced by a wrapper around `handle_request` rather than by trace points inside it: that
function has more than twenty returns, and the early ones — a stale feed, a refused
authorisation, a torn-down stream — are the interesting ones.

## Verified on hardware

Board `D0:CF:13:71:FF:54`, 2026-07-29:

- control supported, level and categories set and read back (`0x1F` armed → `0x1D` on exit, the
  V2G bit cleared by the tool's own disarm)
- real WARN lines drained end to end, naming the gate that held:
  `W relay: close 0x01 refused rc=14 drift=0 safety=1 veh=0 alive=0 auth=0 hlc=1(stop=0 chg=Idle)`
- 624 bytes drained, `dropped=0`, `pending` correctly back to 0
- 7 drains across 25 s idle — the backoff at its ceiling

**Not yet verified against a vehicle:** the `V2G` category lines themselves need a plugged car to
emit, and the bench vehicle was unplugged when this landed. The arming, readback, drain, sink and
suppression paths are all verified; the trace *content* is not.
