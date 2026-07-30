# Integrating cbctl: the complete walkthrough for a controller developer

This is the document to read first. It walks the whole road — hardware prerequisites, bring-up,
the charging lifecycle, failure handling, shutdown — and it exists because the road has already
been walked on real chargers and every pothole below cost a bench day or a field incident.
Nothing here is speculative; every rule cites the incident that made it a rule, and every code
snippet uses the real API (this guide is audited against `include/cbctl/cbctl.h` — if they ever
disagree, the header wins and this file has a bug).

The other documents go deeper on each area; §12 maps them.

---

## 1. What cbctl is — and the boundary that keeps it simple

cbctl is a **non-blocking C11 library for talking to cbplc PLC boards** over USB serial. One
`cbctl_link_t` per board, one board per gun. It carries: identity, the limits/measurement feed,
authorisation, relay dead-man leases, STATUS observation, diagnostics, provisioning.

It is **not a controller**. It does not own your event loop, does not create threads, does not
sleep, does not open your serial ports' paths for you, does not drive power modules or meters,
and does not decide charging policy. You do. Everything cbctl wants from you is:

1. call `cbctl_tick()` (or `cbctl_fleet_tick()`) often enough — §6 says exactly how often;
2. keep your declarations current — §5;
3. handle the events it queues — §7;
4. reopen dead ports — §9, because the library quarantines but only the caller can reconnect.

The V2G conversation with the vehicle — SLAC, ISO 15118, DIN 70121, the 150 ms-class response
deadlines — lives on the board's own ESP32-S3 and never waits on this library. §10 gives the
full latency model; the short version is that cbctl refreshes the *data* the board answers
from, and the board answers alone.

## 2. Before any code: the hardware checklist

Skipping this section is how "cbctl doesn't work" bugs get filed against electricity.

- **Power the boards properly.** Each board declares 500 mA; a Pi 4B's whole USB budget is
  ~1.2 A with GANGED over-current — one trip drops every port simultaneously. Two boards is
  already marginal on paper; beyond two, a self-powered hub with per-port over-current or the
  enclosure's 5 V rail with data-only USB is mandatory. A bus-powered hub changes nothing.
  Boot inrush can sag the 5 V rail (logged as `Undervoltage detected`, hwmon) with zero USB
  over-current events — those are different failures from different supplies.
- **One owner per port.** If a production controller service is installed, it auto-starts on
  boot and holds the PLC ports. A second opener sees an enumerated, silent device and every
  request times out. Check before diagnosing the board: `lsof /dev/ttyACM*`.
- **Expect enumerated-but-mute boards after a power cycle.** The ESP32-S3 can come up stranded
  in ROM download (the USB auto-reset circuit's DTR state at power-up). It enumerates normally
  and answers nothing. Recovery is a reset pulse:
  `esptool --port <p> --before default_reset --after hard_reset chip_id` — or an EN-pulse if
  your hardware wires one. A controller that treats "IDENTIFYING forever" as "board is dead"
  will condemn healthy hardware.

## 3. Naming the board: by-id and MAC, never port order

`ttyACM` numbering changes across reboots, reflashes, and — measured on the bench — within a
single session. Two rules, both mandatory:

```c
/* 1. Open the stable path, not the tty number. */
cbctl_serial_t ser;
if (!cbctl_serial_open(&ser,
        "/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_D0:CF:13:71:FF:54-if00",
        115200u)) { /* fail loudly */ }

/* 2. After the link identifies, PIN it: refuse to drive the wrong board. */
static const uint8_t kGun1Mac[6] = { 0xD0, 0xCF, 0x13, 0x71, 0xFF, 0x54 };
if (cbctl_expect_mac(&link, kGun1Mac) != CBCTL_OK) { /* wrong board on this cable — stop */ }
```

Store your gun↔MAC map in the controller's config. The MAC pin catches the case the by-id path
cannot: a physically replaced board on the same cable. (The baud argument is nominal — this is
USB-CDC, not a UART; the wire runs at USB speed regardless.)

The fleet adds a third guard for free: two links that resolve to the SAME physical board (a
duplicated config path, a stale symlink) — the later one is permanently quarantined and
`fleet.duplicate_boards` counts it. This guard lives in the fleet loop; drive links without a
fleet and it never runs (§11).

## 4. Bring-up: open, tick, READY

```c
cbctl_link_t link;
if (cbctl_open(&link, &ser.tx) != CBCTL_OK) { /* arg error — not a wire problem */ }

/* IDENTIFYING -> READY happens inside ordinary ticking; nothing blocks. */
uint32_t t = monotonic_ms();
while (cbctl_link_state(&link) != CBCTL_LINK_READY) {
    cbctl_tick(&link, monotonic_ms());
    poll_on(cbctl_serial_fd(&ser), cbctl_next_deadline_ms(&link, monotonic_ms()));
    /* give up after a few seconds -> §2's stranded-board recovery, then retry */
}
```

`cbctl_open()` **zeroes the link**. Anything you installed before it — diag sinks, expectations
— is gone. Open first, configure after. (This matters again, critically, in §9.)

A healthy board identifies in well under a second. `cbctl_identity()` then gives you MAC,
firmware build, wire version, reset reason, uptime.

## 5. Declare, don't command: the five intents

cbctl's API is declarative. You state what should be true; the tick makes the wire agree, keeps
re-stating it on each intent's own cadence, and re-asserts automatically after a board reset.
Setters never block and never touch the wire themselves.

```c
/* 1. THE ENVELOPE — what the charger may offer. This is the feed the board's whole
 *    permission-to-energise hangs on; it repeats every 400 ms once declared. */
cbctl_envelope_t env = {0};
env.max_voltage = CBCTL_MV(500000);   /* wrappers: millivolts / milliamps / watts */
env.max_current = CBCTL_MA(200000);
env.max_power   = CBCTL_W(60000);
env.power_ready = true;               /* false = offer nothing, session survives */
cbctl_set_envelope(&link, &env);

/* 2. THE MEASUREMENT — present V/I the vehicle will be told, plus the isolation verdict.
 *    Restate it every pass; values reach the board within one 400 ms feed period.
 *    THE VALUES COME FROM THE POWER MODULES, NOT THE METER — the meter reads through a CT
 *    downstream of the contactor and is blind during PreCharge, which cost real sessions. */
cbctl_measurement_t m = {0};
m.present_voltage = CBCTL_MV(modules_mv);
m.present_current = CBCTL_MA(modules_ma);
m.isolation = CBCTL_ISO_VALID;        /* UNKNOWN/WARNING refuse energy, by design */
cbctl_set_measurement(&link, &m);

/* 3. AUTHORISATION — a LEASE, not a flag. The board demotes GRANTED to PENDING when the
 *    lease expires and opens the energy relays. A lease below CBCTL_AUTH_LEASE_MIN_SAFE_MS
 *    (1450 ms) is REFUSED with CBCTL_E_ARG, not quietly raised — unlike a relay hold, which is
 *    raised, because permission is the caller's policy and lengthening it would grant more than
 *    was asked. 8000 is a good default. */
cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 8000u);

/* 4. THE CONTACTOR — a dead-man. The board opens it itself if the lease is not renewed;
 *    there is no close-without-hold. Floor is 1900 ms; run production at 10000. */
cbctl_relay_close(&link, CBPLC_RELAY_GUN, 10000u, NULL);
/* ... and when the session ends: */
cbctl_relay_open(&link, CBPLC_RELAY_GUN, NULL);

/* 5. THE STOP LATCH — the emergency path. Preempts everything except a request already
 *    in flight (<= 750 ms), and stays latched until you clear it. */
cbctl_begin_stop(&link);   /* ... later, once safe: */ cbctl_stop_clear(&link);
```

Rules that earned their place:

- **Re-stating an unchanged intent is free** — declarations transmit on change or on their own
  cadence, never per call. Call the setters every pass without guilt.
- **Measurement values deliberately do NOT jump the queue.** Change-triggered feedback was
  tried: sensor jitter made every tick "a change", 618 FEEDBACK frames starved 7 AUTHORIZE
  frames in 23 s, the lease lapsed, and the board refused the gun close 1136 times mid-charge.
  Freshness comes from the 400 ms cadence. The one exception is the **isolation verdict** — a
  discrete safety fact, edge-urgent, sent immediately on change.
- **An ACK is the only proof.** A relay close that left the host is not a closed relay;
  `cbctl_counter(&link, CBCTL_CTR_RELAY_LEASE_OPENS)` and friends tell you what the *board*
  did. A lost renewal is neither an ack nor a refusal — the library accounts it as exactly
  nothing and retries, and so should your logic.

## 6. The loop: your loop, cbctl's deadline

cbctl never owns the loop. The contract is three lines, and the third is a **safety parameter**:

```c
for (;;) {
    uint32_t now = monotonic_ms();
    cbctl_fleet_tick(&fleet, now);                 /* or cbctl_tick per link */

    drain_events(...);                             /* §7 */
    update_measurements_from_modules(...);         /* your power-path work */

    struct pollfd fds[CBCTL_FLEET_MAX];
    for (i = 0; i < n; i++) { fds[i].fd = cbctl_serial_fd(&ser[i]); fds[i].events = POLLIN; }
    poll(fds, n, (int)cbctl_fleet_next_deadline_ms(&fleet, monotonic_ms()));
}
```

**The pass-gap budget:** the board opens relays when its feed goes 1500 ms stale. One lost
frame costs a full 750 ms retry ladder, the feed repeats every 400 ms, so your loop's worst gap
G must satisfy `400 + 750 + G < 1500` → **G < ~350 ms**. cbctl's own share of a pass is
microseconds (362 µs worst ever measured, at 8 links, on a Pi 4B); the budget is consumed by
*your* work — module CAN traffic, meter reads, OCPP, logging. Anything of yours that can block
longer than ~300 ms must come off this thread (the meter was exactly this bug once: a dead
Modbus meter blocked 2.1 s and would have dropped both guns).

There is no `cbctl_pollfds()` — you opened the ports, you own the fds. The deadline function
returns 0 while a request is in flight ("come straight back"), the base cadence for an idle
link, and never asks you to spin.

## 7. Events: drain them every pass

```c
cbctl_event_t ev;
while (cbctl_next_event(&link, &ev)) {
    switch (ev.kind) {
    case CBCTL_EV_VEHICLE_PLUGGED:    /* begin your session logic          */
    case CBCTL_EV_VEHICLE_UNPLUGGED:  /* tear down, open relay, drop auth  */
    case CBCTL_EV_VEHICLE_MAC_KNOWN:  /* Autocharge: cbctl_vehicle_mac()   */
    case CBCTL_EV_SESSION_STARTED:    /* V2G session live                  */
    case CBCTL_EV_SESSION_ENDED:
    case CBCTL_EV_BOARD_RESET:        /* library re-asserts intents itself */
    case CBCTL_EV_LINK_QUARANTINED:   /* §9 — this one is YOURS to act on  */
    case CBCTL_EV_RELAY_OVERRIDDEN:   /* board safety-opened under you     */
    default: break;
    }
}
```

The ring holds 16 events per link, drop-oldest. Drain every pass and it never overflows;
ignore it for seconds during a storm and the oldest edge is gone — which is why state reads
(`cbctl_flag`, `cbctl_counter`) are the durable truth and events are the change notifications.

**Everything you read is last-known-good**, stamped by the ~1 s STATUS poll. `cbctl_ev_target()`
and `cbctl_ev_soc_pct()` return validity, not just values — a target from a session that ended
reads invalid, not zero. And the two age functions carry sentinels:
`cbctl_feed_age_ms()` returns `CBCTL_FEED_NEVER_SENT` (0xFFFFFFFF) and
`cbctl_observation_age_ms()` returns `CBCTL_NEVER_OBSERVED` until there is anything to age.
**Exclude the sentinel at every arithmetic site.** Folding one into a max once produced
"stale (4294967295 ms)" verdicts on healthy links; asserting only `gap < limit` passes loudest
when nothing was ever sent. Check the sentinel, then compare.

## 8. The charging lifecycle, end to end

The complete, hardware-proven sequence (tools/charge_soc.c is the living reference — it has
charged real vehicles through every line of this):

| Phase | You | The board (observed via events/flags) |
|---|---|---|
| Idle | envelope declared (`power_ready=false` is fine), auth PENDING | CP state A |
| Plug | `CBCTL_EV_VEHICLE_PLUGGED` → your auth flow (RFID/OCPP/Autocharge) | CP B, SLAC starts |
| Authorised | `cbctl_authorize(GRANTED, 8000)` + `power_ready=true` | SLAC match → V2G session |
| Session | `CBCTL_EV_SESSION_STARTED`; close the gun: `cbctl_relay_close(GUN, 10000, NULL)` | CableCheck → PreCharge |
| PreCharge | drive modules to the EV's request (`cbctl_ev_target()`), **feed module V/I as the measurement** — the meter reads 0 A here and always will | answers the EV from your feed |
| Delivery | track `cbctl_ev_target()` (updates on the ~1 s STATUS cadence — see §10 Path C), keep declarations fresh | CurrentDemand cycles, sub-10 ms, on-board |
| Taper/stop | EV lowers demand on its own; on `SESSION_ENDED`/unplug: modules off **first**, then `cbctl_relay_open`, auth PENDING | ramps down, opens its side |
| Wind-down | **keep ticking** until the opens are ACKed — there is no blocking flush, the loop IS the flush | |

Do not invert the wind-down order: power off at the modules first, then open the contactor —
opening a contactor that is carrying current is the failure every interlock here exists to
prevent, and the board will refuse patterns that look like it.

## 9. Failure handling: quarantine is the library's, recovery is yours

When a board stops answering (5 consecutive losses ≈ 3.75 s; a dead fd is faster), the link
goes `CBCTL_LINK_QUARANTINED`, `CBCTL_EV_LINK_QUARANTINED` fires, and re-probes back off to
30 s. Siblings never notice — that isolation is measured, on hardware, with a contactor closed
on the surviving gun.

**What the library will never do is reopen the port.** The fd is yours. A controller that does
not implement this pattern has decided that any USB flap permanently downs that gun until a
process restart — which kills the *other* guns' sessions. This is the single most important
snippet in this guide, proven on the bench (kill → quarantine → reopen → READY, same process,
sibling's feed undisturbed at 186 ms):

```c
case CBCTL_EV_LINK_QUARANTINED:
    /* Optionally wait out one backoff; then: */
    cbctl_serial_close(&ser[i]);
    if (cbctl_serial_open(&ser[i], cfg->by_id_path[i], 115200u) &&
        cbctl_open(&link[i], &ser[i].tx) == CBCTL_OK) {
        /* THE LINK IS NOW ZEROED. Everything must be re-declared — this is the
         * difference from a BOARD reset, where the library re-asserts for you: */
        cbctl_expect_mac(&link[i], cfg->mac[i]);
        cbctl_provision_evse_id(&link[i], cfg->evse_id[i]);
        cbctl_provision_attn_trim_db(&link[i], cfg->trim[i]);
        re_declare_envelope_auth_and_diag(i);
        /* The fleet slot kept its pointer: no remove/re-add, works at N == CBCTL_FLEET_MAX. */
    }
    break;
```

The two re-assert cases, side by side, because confusing them ships a charger announcing the
wrong identity:

| Event | Who re-asserts intents? |
|---|---|
| **Board** reset (power blip, restart) — link survives | **The library.** `CBCTL_EV_BOARD_RESET` fires, declarations re-send automatically. |
| **Caller** reopens (`cbctl_open` after a flap) — link zeroed | **You.** Everything: expectation, provisioning, envelope, auth, diag. |

## 10. The latency model: three paths, three very different numbers

Know which path a requirement lives on before reaching for a fix.

| Path | What | Latency | Grows with N? |
|---|---|---|---|
| **A** | Vehicle asks → board answers (CurrentDemandRes, the 150 ms-class V2G deadlines) | **sub-10 ms**, on the board's own MCU, from RAM | **No** — per-gun silicon |
| **B** | Fresh module V/I *into* those answers | one 400 ms feed period + wire | No |
| **C** | EV's *changed demand* out to your power stage | up to **~1000 ms** (the STATUS poll; the board never pushes) | No |

Path A is why 400/1500 are not "V2G latency" — the board answers from what it holds. Path B is
period-bound deliberately (§5's starvation incident). Path C is the largest real latency in the
system and the one worth improving if tight demand-tracking ever matters: the wire reserves
unsolicited events (`request_id 0`) and cbctl already accepts them — a firmware push on target
change would cut it to ~15 ms with zero library changes.

Reaction times, measured or derived (worst case in parentheses):
relay close→ACK **1 ms** (~800 ms mid-retry) · stop latch→wire ~10 ms (750 ms) · limit change
~10 ms (~800 ms) · plug detect ~500 ms (~2 s) · state edge on a *charging* link ≤3 s (starve
promotion) · dead fd→quarantine <10 ms · mute board→quarantine 3.75 s.

## 11. Multiple guns

One loop, one fleet, one link per board. Measured to the design cap on a Pi 4B (`make
fleet-bench`): per-link margins **bit-identical** from 1 to 8 links, worst fleet pass 362 µs,
half-the-fleet storm contained.

**Read that provenance precisely.** The bench has two physical boards, so N=4 and N=8 run
against the in-process loopback — which executes the *real* cbplc session, framing and CRC code,
on the Pi's real CPU. So the host cost and the scheduling behaviour are genuine measurements;
what is modelled rather than measured is USB latency, and that is exactly the term that does not
aggregate (one device per link). Two physical boards are hardware-proven; 4 and 8 are proven on
everything except the wire.
`CBCTL_FLEET_MAX` is 8; the arithmetic holds to ~12; past that, measure first
(`make fleet-bench`).

Do not put links on separate threads to "improve isolation" — the wires are already parallel
(one USB device each), the tick is microseconds, cbctl is deliberately not thread-safe, and
one-link-per-thread silently disables the duplicate-board guard, which exists because a
duplicated device path once meant "gun 2's" close could land on gun 1. If your architecture
truly requires threads, one *fleet* per thread with its own links and transports is the
supported shape — never one fleet shared across threads.

## 12. Do / Don't

| Do | Don't |
|---|---|
| Open by `/dev/serial/by-id/`, pin with `cbctl_expect_mac()` | Trust `ttyACM` order — it changed within one session on our bench |
| Feed module V/I as the measurement | Feed meter V/I — it is blind during PreCharge and the session dies |
| Keep the loop's worst gap under ~350 ms | Block on anything (meter, CAN, disk, network) in the tick thread |
| Implement the §9 reopen on quarantine | Restart the process to recover one gun — it kills the others' sessions |
| Re-declare everything after a caller-side `cbctl_open` | Assume the library re-asserts (that is only for *board* resets) |
| Treat `CBCTL_FEED_NEVER_SENT` / `CBCTL_NEVER_OBSERVED` as "no data" | Do arithmetic on sentinels, or assert only `gap < limit` |
| Run leases long: relay hold 10 s, auth lease 8 s | Run at the floors (1900/1450) in production — they are floors, not targets |
| Modules off, then relay open, then keep ticking | Open a contactor under load, or exit without flushing the opens |
| Let the board's counters be the truth (acks, safety opens) | Believe a command happened because it was sent |
| Restate intents every pass — it's free | Rate-limit your own setters (the library already coalesces) |
| Expect enumerated-but-mute boards after power-up (§2) | Condemn a board for "IDENTIFYING forever" without a reset pulse |
| Stop the installed controller service before bench work | Debug two owners on one tty |

## 13. Deployment checklist

- [ ] Boards powered off-bus (powered hub / enclosure 5 V) — mandatory beyond 2 boards
- [ ] Gun↔MAC↔by-id map in controller config; `cbctl_expect_mac` enforced at bring-up
- [ ] Reopen-on-quarantine implemented and **tested by pulling a cable mid-session**
- [ ] Loop pass gap measured under full production load (OCPP+HMI+logging) — < 350 ms worst
- [ ] Nothing blocking on the tick thread (meter and CAN on their own workers)
- [ ] Wind-down order verified: modules → relay → ticked flush
- [ ] Per-board provisioning declared at link-up (EVSE ID, attn trim) and after every reopen
- [ ] Diag drain armed to a log file; V2G trace OFF by default (arm per-incident)
- [ ] `make test` and `make fleet-bench` pass on the target hardware class

## 14. Where everything else lives

| Doc | Read it for |
|---|---|
| [01-api-reference](01-api-reference.md) | every function, struct, constant |
| [02-design-and-safety-model](02-design-and-safety-model.md) | why the API is declarative; the dead-man philosophy |
| [03-concurrency-and-fleet](03-concurrency-and-fleet.md) | the loop contract in depth; fleet scheduling; threads |
| [04-failure-modes](04-failure-modes.md) | the failure catalogue with measurements; N>2 deployment |
| [05-dc-sequence-and-gaps](05-dc-sequence-and-gaps.md) | the DC session sequence on the wire |
| [06-diagnostics](06-diagnostics.md) | the board log, levels, the V2G trace |
| [07-provisioning](07-provisioning.md) | per-board identity, trim, protocol mask |
| `examples/pi_two_guns.c` | a compilable two-gun skeleton |
| `tools/charge_soc.c` | the full lifecycle against a real vehicle — the living reference |
| `tools/live_dual.c` | isolation, kill and recovery, gated |
| `tools/fleet_bench.c` | the N=1..8 measurement methodology |
