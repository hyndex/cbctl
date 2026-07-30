# cbctl — failure modes, by situation

What can block a charge, in every situation a charger actually meets. Each row says what
happens, what cbctl does about it, and **how it was established** — measured on hardware,
proven by a host test, or reasoned about and not yet observed. Where something is unverified
it says so; a table like this is only useful if its "verified" column can be trusted.

Hardware referenced below: two ESP32-S3 PLC boards (`D0:CF:13:71:FF:54`, `D0:CF:13:73:0A:68`),
firmware build `62a77ad2`, one real vehicle (`88:C9:B3:79:3F:68`), macOS host.

---

## 1. Single gun charging

| # | Situation | What happens | Evidence |
|---|---|---|---|
| 1.1 | Normal charge | feed held at **30–39 ms** against a 1500 ms bound; lease renewed at hold/6, every renewal ACK-confirmed | **measured**, 200 s live |
| 1.2 | Controller re-declares the feed every pass | harmless — the library compares before marking dirty | **measured**; used to make LIMITS permanently due and starve the lease |
| 1.3 | One message unanswered by the board | stop-and-wait blocks everything for the retry sequence — bounded to **750 ms** by cbctl's own timing (3 transmissions x a 250 ms timeout; the retry backoff is configured but never applied), so the derived hold floor (1640 ms) cannot lapse | **host test** + derivation; schema defaults gave 3300 ms and made the lease unrenewable (`acks=1`) |
| 1.4 | Vehicle never opens a session | not a charger fault. The reference harness reports the same on this vehicle | **measured**, both stacks |
| 1.5 | Controller process dies | the board's dead-man opens the relay | **partially verified** — see §6.1 |

## 2. Dual gun

| # | Situation | What happens | Evidence |
|---|---|---|---|
| 2.1 | Both boards healthy | both feeds ≤ **34 ms**; service split evenly | **measured** |
| 2.2 | One board's transport dies mid-run | that link is QUARANTINED in **7 ms**; the survivor's worst feed age is **399 ms** against the 1500 ms bound, stays READY, keeps its lease, 36/36 renewals confirmed | **measured**, USB drop modelled by closing the fd |
| 2.3 | One board wedged (answers nothing) | quarantined after `CBCTL_LOSS_LIMIT` losses, re-probed on capped exponential backoff | **host test** |
| 2.4 | Both boards drop together (hub over-current) | each link quarantines independently; no shared state to corrupt | **host test**; the Pi-4B hub case is the real-world instance |
| 2.5 | Ports swap across a reboot | identity is pinned by MAC; `cbctl_expect_mac()` returns `CBCTL_E_WRONG_BOARD` | **host test** + live identity read |
| 2.6 | Two links resolve to the SAME board | the later one is permanently quarantined, `fleet.duplicate_boards` counts it | **host test** |
| 2.7 | **Both guns charging simultaneously** | expected fine — the links share no mutable state and the tick cost is linear | **NOT VERIFIED — one vehicle available.** See §7 |

## 3. Scaling — how many guns fit

Measured on live boards, steady state, 2000 samples:

| links | p50 | p90 | p99 | max |
|---|---|---|---|---|
| 1 | 20 µs | 38 µs | 93 µs | 3059 µs |
| 2 | 40 µs | 70 µs | 264 µs | 5617 µs |

p50 doubles exactly with the link count, so **cost is linear** and there is no super-linear
term hiding in the fleet loop. At the worst per-link figure ever observed (~2.8 ms), **~142
links fit inside one 400 ms cadence**; at p99, thousands.

**cbctl is not the scaling constraint.** The constraint is the rest of the application: the
OCPP client, the HMI and the control loop share the same wall clock. Treat the numbers above as
a ceiling the rest of your process eats into, not a budget you may spend.

## 4. The relay lease — every way it can end

| # | Cause | Intended? | Handling |
|---|---|---|---|
| 4.1 | Controller stops renewing (crash, hang, power loss) | **yes — this is the point** | the board opens the relay itself |
| 4.2 | Renewal blocked behind an unanswered request | **no** | timing bounded to 750 ms; AUTHORIZE lease floored at 1450 ms, derived from the re-assert period actually used (auth_period() clamps to a 500 ms floor, so the old lease/3 derivation did not hold for short leases and certified an unsafe 1130 ms) |
| 4.3 | Renewal queued but never applied | **no** | the renewal clock is stamped from the **ACK**, not from queueing |
| 4.4 | Renewal refused by a gate | **no** | the stamp is reset so it retries at once, `relay_refusals` counts it |
| 4.5 | Wire quantisation | **no** | hold rounded UP to the 100 ms the wire carries, so the board's lease is never shorter than we believe |
| 4.6 | Board rebooted and forgot the lease | **no** | periodic renewal re-establishes it within one period |
| 4.7 | **Board's safety layer keeps undoing the close** | **no** | see §5 — the worst one |

## 5. Relay chatter — measured, and the reason it was invisible

Command a gun closed with the board's preconditions unmet and you get **one safety-open per
renewal**, forever. Observed on hardware: `safety_opens` climbing 23, 25, 27, 30, 32, 34
against renewals 3, 5, 7, 10, 12, 14.

Every single command was **ACKed with gate 0**. The board is not refusing — it accepts the
close and its safety layer opens it again a moment later. So nothing in the protocol reports a
problem, the controller cheerfully re-commands it on the next renewal, and the relay physically
actuates on every cycle. That is audible chatter and real contactor wear, and it runs until
somebody notices by ear.

cbctl now watches `relay_safety_opens` against a baseline taken when the close intent is
established. Past `CBCTL_RELAY_OVERRIDE_LIMIT` (3) it **abandons the intent**, raises
`CBCTL_EV_RELAY_OVERRIDDEN` once, and stops asking. After the fix, on the same board:
`safety_opens = 40, flat` across renewals 4→18.

The caller re-establishes the close when the precondition it was missing is actually met.

## 5.1 Lease margin — sized for a controller that is merely LATE

The dead-man catches a controller that has **died**. A controller that is only **late** must
never trip it — and on a real charger the same process runs OCPP, an HMI and the control loop,
so being descheduled is the normal condition, not an exception.

| | before | now |
|---|---|---|
| renewals inside one hold | 3 | **6** (`CBCTL_RELAY_RENEW_DIVISOR`) |
| consecutive renewals that may be lost | 2 | **5** |
| lease renewal priority | below the feed | **preempts the feed past hold/2** |
| recommended hold | — | **10 s** (`CBCTL_RELAY_HOLD_RECOMMENDED_MS`) |

**The preemption is the important half.** Normally the feed outranks the lease, because the
feed holds permission to energise. But a starved controller catching up must save the lease
FIRST: a lapsed lease drops a contactor immediately, whereas a feed repeat arriving one tick
late is still well inside its own 1500 ms bound.

Measured against a pathologically starved caller — thirty stalls each swallowing 60 % of the
hold:

```
hold=10000ms  worst tick gap=6000ms  min headroom=3800ms  preempts=30  lapses=0
```

With the old hold/3 and no preemption the same test leaves **2000 ms** of headroom. Neither
lapsed here, but the margin nearly halved, and the failure is not gradual — it is a contactor
opening.

Two counters make this visible in production: `lease_min_headroom_ms` (closest the lease ever
came to lapsing) and `worst_tick_gap_ms` (the longest the CALLER went between ticks). If the
first trends down or the second is large, the loop is being starved and a contactor will
eventually open for no reason but scheduling.

## 5.2 What the driver sees

`cbctl_led_preset()` derives the connector's presentation state from the same observation
everything else reads. The OCPP status, the session state, the charge state and the LED preset
are four parallel state machines; updated in four places they drift, and a driver ends up in
front of a light that disagrees with the charger. One derivation, one order, no drift.

Order, most-dangerous-first: `emergency` (safety chain) → `faulted` (link lost) →
`booting` (not identified) → `faulted` (**stale observation**) → `finishing` (stop raised) →
`charging` (contactor closed AND session open) → `preparing` (plugged) → `available`.

Two rules worth their own line:

- **A stale picture never shows `charging`.** Of the ways to be wrong, the expensive one is a
  driver told they are charging while nothing happens — they walk away and return to an empty
  battery. Showing `faulted` when the charger is secretly fine costs an unnecessary unplug.
  That asymmetry decides the rule, and the threshold is the same one that sets `DEGRADED`, so
  the light and the link state cannot disagree about whether our picture is trustworthy.
- **A closed contactor alone is not `charging`.** Without an open session that is precharge or
  a fault, and the driver is not getting energy yet.

cbctl does not drive pixels: the cbplc wire protocol has **no LED message**. The names match
the vocabulary this codebase already sends (`booting`/`available`/`preparing`/`charging`/
`finishing`/`faulted`/`emergency`) so the value drops into an existing LED command with no
translation table to maintain.

## 6. Known limits — verified as far as they can be, and no further

### 6.1 The dead-man is only partially proven on hardware

The intended path (§4.1) could not be closed on the bench. Holding a lease on the gun with no
vehicle attached means the board's safety layer opens the relay for its own reasons, so there
is never a live lease to lapse — `relay_lease_opens` stayed 0 while `relay_safety_opens`
climbed. What IS proven: the relay ends OPEN after a `SIGKILL` with nothing tidy running on the
way out, and the renewal path is ACK-confirmed continuously.

Closing this properly needs a lease held on a gun whose preconditions are genuinely met, i.e.
a vehicle mid-session. **Not yet done.**

### 6.2 Simultaneous dual-gun charging is unverified

Both boards run, both feeds hold, isolation survives a transport kill. But no second vehicle
was available, so **two live V2G sessions at once has never been observed.** Everything about
it is expected to work and none of it has been watched.

### 6.3 The loopback cannot model everything

The dead-transport quarantine fix changed real behaviour (`DEGRADED` forever → `QUARANTINED`)
while the loopback mutation could not tell the difference — its error path reaches `on_message`
differently than a real closed fd does. **Do not trust the simulator alone for transport-death
behaviour.** Hardware was the authority there, and would be again.

### 6.4 Not implemented

`CBPLC_T_DIAG` (the board's RAM log ring — a controller cannot pull board diagnostics),
`cbctl_evcc_id`, `cbctl_fleet_by_mac`, `cbctl_request_restart`, and the CAN transport (needs
`CBPLC_MIN_TRANSPORT_MTU` 32→8, a firmware-side wire decision).

## 7. What to watch in production

Every one of these is a counter cbctl already keeps. None should move on a healthy charger:

| counter | non-zero means |
|---|---|
| `CBCTL_CTR_RELAY_LEASE_OPENS` | the dead-man lapsed — **the audible relay flake** |
| `CBCTL_CTR_GUN_OPEN_TIMEOUTS` | a contactor broke current instead of waiting for decay |
| `relay_overridden` | a close was abandoned because the board kept undoing it (§5) |
| `feed_late` | the feed went past 1500 ms — **excursions, not ticks** |
| `identity_changed` | the board on a port became a different board |
| `duplicate_boards` | two links are one board |
| `observation_stalls` | STATUS went silent; flags are last-known-good, check their age |
| `CBCTL_CTR_STATUS_TRUNCATED` | the diagnostic line no longer fits its payload cap |
| `transport_failures` | the USB/UART endpoint is failing |
| `reestablishes` | the board rebooted and once-only state was re-asserted |

`cbctl_observation_age_ms()` returns `CBCTL_NEVER_OBSERVED` rather than 0 before the first
snapshot — 0 would read as "perfectly current", the most dangerous possible answer to "how old
is what I believe".

## Deploying more than two PLCs — the constraints are outside the library

fleet_bench (tools/fleet_bench.c) measured N=1/2/4/8, every link charging, on the bench Pi 4B:
tick p50 0/0/1/2 us, worst pass 52/185/204/362 us; link 0's margins bit-identical at every N
(feed 399 ms, renewal gap hold/6, AUTHORIZE gap lease/3); a half-fleet wedge storm quarantined
every wedged board and left every survivor READY. The library's cost is linear microseconds and
the margins are period-bound, not contention-bound. What actually decides an N>2 deployment:

1. **Power the boards off the USB bus. Non-negotiable.** An ESP32-S3 board declares ~500 mA and
   the Pi 4B's whole USB budget is ~1.2 A with GANGED over-current switching: one trip drops
   every port at once. Two boards already trip it in the field (`throttled=0x50000`); eight is
   ~4 A. Use a self-powered hub with per-port over-current, or the enclosure's 5 V rail with
   data-only USB. A bus-powered hub changes nothing.

2. **Open by `/dev/serial/by-id/`, pin with `cbctl_expect_mac()`.** `ttyACM` ordering renumbers
   across reflashes and flaps — measured on the bench: the same board changed device node within
   one session. The by-id path is stable per board; the MAC pin catches the case where even that
   lies (a replaced board on the same cable).

3. **Own the reopen. The library quarantines; only the caller can reconnect.** cbctl contains a
   dead transport correctly — quarantine, capped backoff, siblings untouched — but nothing in
   the library ever closes or reopens an fd, deliberately (the port belongs to the caller). A
   caller that never reopens has turned every USB flap into a permanent single-gun outage whose
   only cure, a process restart, kills the other guns' sessions too. The pattern is in
   tools/live_dual.c: on quarantine, `cbctl_serial_close()`, `cbctl_serial_open()` the same
   by-id path, `cbctl_open()` the same link in place. The fleet slot keeps its pointer, so this
   works at N == CBCTL_FLEET_MAX where an open-new-then-retire shape would be refused; re-assert
   provisioning and intents after, since `cbctl_open()` starts the link from zero.

4. **Budget the caller, not the library.** The binding arithmetic at any N is the caller's pass
   gap G: worst feed age = 400 (period) + 750 (one full retry ladder) + G, against the board's
   1500 ms bound — so G must stay under ~350 ms. cbctl's own share of that at N=8 is the worst
   pass above: 362 us.
