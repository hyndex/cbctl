# cbctl — controller-side API reference

**cbctl** is the library a charger controller uses to operate one or more cbplc PLC boards.
It is plain C11, allocation-free, OS-free and transport-pluggable: the same source runs on a
Raspberry Pi, on a Linux SBC, and on a bare-metal or RTOS microcontroller.

This document is the **contract**. It says what to call, in what order, with what values, what
must precede and follow each call, and what happens when you get it wrong. Read §2 before
writing any code — the ordering rules are not stylistic, they are what stops a charger
energising a connector it should not.

> **Why a library and not just the wire protocol.** Every rule in §2 and §6 exists because
> somebody has already broken it. A controller written directly against the wire gets these
> wrong in predictable ways, and the failures are the expensive kind: a relay closed with no
> dead-man, a feed that went stale mid-charge, a stop that was never cleared, a counter read
> as a live value. cbctl's job is to make each of those either impossible to express or an
> immediate, named error — not a comment in a document you were supposed to have read.

---

> **Correction — no call in this API blocks.** An earlier draft of this document showed
> timeouts on `cbctl_identify()` and described `cbctl_relay_close()` as synchronous. That
> design cannot manage more than one PLC: a synchronous request costs up to 3300 ms of
> retries against a 1500 ms feed deadline, so one unresponsive board would terminate the
> charging sessions on every other board. Every call below records intent and returns
> immediately; `cbctl_tick()` is the only thing that touches the wire. See
> [`03-concurrency-and-fleet.md`](03-concurrency-and-fleet.md) §1 for the arithmetic.

---

## 0. Implementation status — read this before the rest

This document was written as a **design** before the code existed. The library now exists, and
where the two disagree **the header is the truth**. Verified by comparing every `cbctl_*` name
in these docs against `include/cbctl/cbctl.h`.

| Documented below | Shipped as | Why |
|---|---|---|
| `cbctl_identify(&link, &ident, TIMEOUT)` | *(nothing — automatic)* | identification is a link state, not a call; it cannot take a timeout because nothing blocks |
| `cbctl_stop()` | `cbctl_begin_stop()` | it is one decision — raise the latch **and** clear `power_ready` |
| `cbctl_stop_clear()` | `cbctl_stop_clear()` ✅ | implemented |
| `cbctl_refusal_detail()` | `cbctl_op_gate()` | the PLC answers with one gate code |
| `cbctl_pollfds()` | `cbctl_serial_fd()` (per port) | descriptors belong to the port, not the core; `cbctl_next_deadline_ms()` is the mandatory half |
| `cbctl_relay_close(link, mask, hold)` | `cbctl_relay_close(link, mask, hold, &op)` | you observe the outcome through the op, since nothing blocks |

**Documented here and NOT implemented.** Do not write code against these:

- `cbctl_evcc_id()` — the ISO 15118 contract identifier. Only the **vehicle MAC** is
  implemented (`cbctl_vehicle_mac()`), which is what Autocharge needs and which is available
  earlier. EVCCID is a separate identity and stays deliberately unimplemented rather than
  half-implemented, because conflating the two is how a car gets billed to the wrong account.
- `cbctl_fleet_by_mac()` — address a fleet member by board MAC. Not needed yet; index the
  array you own.
- `cbctl_request_restart()` — ask the PLC to restart a session.

A note on how this section came about, since it is the kind of drift that makes a document
worse than useless: eight names in these docs had no counterpart in the header, and one of
them — `cbctl_stop_clear()` — turned out to mark a real defect rather than a naming slip.
`cbctl_begin_stop()` was clearing `power_ready` locally and never sending the latch to the
board, so a controller could raise a stop the PLC never saw, and had no way to clear one it
did. Both halves are now implemented and mutation-tested. Comparing docs against the header
was what surfaced it.

---

## 1. The model in one page

A controller owns the **decisions**. The PLC owns the **hardware and the safety interlocks**.
The wire between them is a request/response protocol with a periodic feed.

```
   your control loop                          cbplc PLC board
   ────────────────                           ───────────────
   cbctl_tick()  ──── HEARTBEAT ───────────►  dead-man refresh
                 ──── AUTHORIZE ───────────►  three-state lease
                 ──── LIMITS ──────────────►  the envelope you can deliver
                 ──── FEEDBACK ────────────►  what you are ACTUALLY measuring
                 ──── RELAY ───────────────►  a LEASE, not a command
                 ──── STOP ────────────────►  latched until cleared
                 ◄─── STATUS ───────────────  ~165 keys of live + cumulative state
                 ◄─── (no push channel) ────  events are synthesised from STATUS; see §4
```

Three things about this that surprise people, all of them load-bearing:

1. **The feed is not optional and not occasional.** LIMITS and FEEDBACK must be repeated. If
   they go stale the PLC withdraws permission to energise — correctly, because it cannot
   supervise a charge it cannot see. See §2.3.
2. **RELAY is a lease.** The PLC opens the relay itself if you stop renewing. That is what
   makes a crashed controller safe rather than merely detectable. See §2.5.
3. **The PLC will refuse you, and a refusal is information.** Every refusal names the gate
   that held. Treat `CBCTL_E_REFUSED` as "the charger disagreed and told you why", not as an
   error to retry blindly. See §6.

---

## 2. The ordering contract

### 2.1 Startup — what must happen before anything works

```c
cbctl_link_t link;
cbctl_status_t rc = cbctl_open(&link, &transport, &cfg);   /* 1. link up      */
cbctl_identify(&link);                                      /* 2. WHICH board  */
rc = cbctl_stop_clear(&link);                               /* 3. clear latch  */
rc = cbctl_authorize(&link, CBCTL_AUTH_PENDING, 0);         /* 4. a decision   */
/* then, every tick, forever: */
rc = cbctl_tick(&link, now_ms);
```

**Step 2 is not optional and not cosmetic.** `cbctl_identify()` returns the board's MAC and
build fingerprint. A run that cannot name the board and firmware that produced it is not
evidence — this project lost days to two boards, one with a corrupted id and one physically
dead, each mistaken for a firmware fault. cbctl therefore refuses every other call until
`identify` has succeeded (`CBCTL_E_NOT_IDENTIFIED`).

**Step 3 exists because STOP is latched.** See §2.6. A PLC that was stopped by a previous
controller — including a previous run of *your* controller — stays stopped. `cbctl_open()`
does not clear it for you, because silently clearing a safety latch on connect is exactly
the wrong default. You must ask.

### 2.2 The tick — the one call you must never skip

```c
cbctl_status_t cbctl_tick(cbctl_link_t *link, uint32_t now_ms);
```

Call it at **at least 5 Hz**; 400 ms is the shape the reference controller uses. `cbctl_tick`:

- sends HEARTBEAT, and re-asserts AUTHORIZE and the current LIMITS/FEEDBACK if they are due
- drains inbound frames and dispatches events
- updates the cached STATUS snapshot
- **advances the library's own watchdogs** and will return `CBCTL_E_FEED_LATE` if you have
  fallen behind your own configured cadence

`now_ms` is a monotonic millisecond clock **you** supply. It must never go backwards. The
library never calls the OS for time — that is what makes it portable — so this is the one
place a caller can hand it a lie, and `cbctl_tick` asserts on a backwards step rather than
computing a wrapped age from it.

> **Why this is a hard rule.** cbplc had a defect where an age computed across a loop pass
> could wrap to 49.7 days and read the freshest possible feed as expired. It killed every
> real session while every idle test passed. cbctl's clock handling is signed and
> future-tolerant for the same reason; see §7.

### 2.3 The feed — LIMITS and FEEDBACK

| | what it says | staleness bound | consequence of exceeding it |
|---|---|---|---|
| `LIMITS` | the envelope you *can* deliver | **1500 ms** | permission to energise withdrawn; DC responses answer `Ongoing` |
| `FEEDBACK` | what you are *actually measuring* | **1500 ms** | same, plus isolation/contactor become unknown |
| grace | — | **10000 ms** | past this the session FAILS with a ResponseCode rather than a TCP reset |

```c
cbctl_status_t cbctl_set_envelope(cbctl_link_t *link, const cbctl_envelope_t *env);
cbctl_status_t cbctl_set_measurement(cbctl_link_t *link, const cbctl_measurement_t *m);
```

You call these when your values **change**. `cbctl_tick` handles the repetition — you do not
write a resend loop, and you cannot forget to. That is the single most common way a
hand-rolled controller kills a charge.

**Re-declaring the same values every pass is also safe**, and is what the worked example does
for clarity. The library compares before marking anything for send, so an unchanged
re-declaration costs nothing on the wire. This is called out because it was NOT true on first
hardware contact: "due now" was expressed by zeroing the last-sent timestamp, so a caller that
re-declared each pass made LIMITS permanently due, starved the relay-lease renewal until the
board opened the contactor on a lapsed dead-man, and made the staleness counter report 6.4
million late ticks on a healthy link. Both styles are now equivalent and tested.

**`power_ready` and stopping are mutually exclusive.** If you have raised a stop, clear
`power_ready`. The library enforces this: `cbctl_set_envelope()` with `power_ready = true`
while a stop is in force returns `CBCTL_E_CONTRADICTORY`. The reason is measured, not
theoretical — an envelope that advertises ready while the controller asks to stop tells the
vehicle two opposite things, and the vehicle resolves it by resetting TCP.

**An absent measurement is not a safe one.** `cbctl_measurement_t.isolation` has a distinct
`CBCTL_ISO_UNKNOWN`, and it is the default. Do not map "I have not measured yet" onto
`VALID`. The PLC relies on the distinction to decide whether CableCheck has a verdict.

### 2.4 Authorisation — three states, not a bool

```c
cbctl_status_t cbctl_authorize(cbctl_link_t *link,
                               cbctl_auth_t decision,   /* DENY | PENDING | GRANTED */
                               uint32_t lease_ms);      /* GRANTED only */
```

| decision | meaning | vehicle sees | lease |
|---|---|---|---|
| `CBCTL_AUTH_DENY` | refused | `FAILED`, session terminates | decays to PENDING after **30 s** |
| `CBCTL_AUTH_PENDING` | still deciding | `Ongoing`, session stays alive | none |
| `CBCTL_AUTH_GRANTED` | may charge | proceeds | **yours to set**, clamped to [100 ms, 600 s] |

The three-way distinction is load-bearing: a driver plugs in and *then* taps their card, so
for a few seconds the honest answer is "still deciding". `PENDING` keeps the session alive
while you find out; `DENY` ends it. Collapsing them into a bool means every slow CSMS lookup
looks like a refusal.

**Set `lease_ms` from how often you actually speak.** A lease shorter than your own tick
interval revokes authorisation between two of your own heartbeats. cbctl refuses a lease
shorter than 3 × your configured tick period with `CBCTL_E_LEASE_TOO_SHORT`, because that
mistake is silent and its symptom (a session that dies for no visible reason) is expensive.

A lapsed GRANTED decays to **PENDING, not DENY**: a controller that went quiet is not a
controller that refused.

### 2.5 Relays — a lease, never a command

```c
cbctl_status_t cbctl_relay_close(cbctl_link_t *link,
                                 cbctl_relay_mask_t which,
                                 uint32_t hold_ms);     /* REQUIRED for GUN/AUX */
cbctl_status_t cbctl_relay_open(cbctl_link_t *link, cbctl_relay_mask_t which);
```

| relay | mask | notes |
|---|---|---|
| `CBCTL_RELAY_GUN` | `0x01` | the DC gun contactor — **the dangerous one** |
| `CBCTL_RELAY_AUX` | `0x02` | auxiliary |
| `CBCTL_RELAY_POWER` | `0x04` | upstream/module power path — not energy at the connector |

Rules the library enforces so you cannot get them wrong:

- **A zero hold on GUN or AUX is refused** (`CBCTL_E_NO_DEADMAN`). A close the lease can never
  open is not a close. `POWER` may be closed without expiry: a site does not want its
  upstream path cycling on a lease.
- **Maximum hold is 25.5 s** on the wire (one byte, 100 ms units). cbctl refuses anything
  larger rather than silently truncating it.
- **Renew well inside the hold.** 2 s renewal against a 5 s hold is the shape the bench uses.
  `cbctl_tick` renews automatically for any lease you have taken; you do not write the loop.
- **Never fire-and-forget.** `cbctl_relay_close` returns a `cbctl_op_t`; you observe the PLC's actual
  answer, because an unacknowledged relay command is indistinguishable from one that was
  never applied.

The PLC will refuse a close it considers unsafe. Refusals are named — see §6 — and the
common one is `CBCTL_E_REFUSED_NO_VEHICLE`: closing the gun onto an empty connector is the
one mistake with no safe outcome, and it is checked with no exception path.

### 2.6 STOP — latched, and that is deliberate

```c
cbctl_status_t cbctl_stop(cbctl_link_t *link);        /* latch  */
cbctl_status_t cbctl_stop_clear(cbctl_link_t *link);  /* clear  */
```

A stop that evaporates because the next tick did not repeat it is not a stop. It stays
latched until you clear it explicitly — across reconnects, across your process restarting,
across everything short of the vehicle being unplugged.

**Both calls read back and verify.** A STOP that silently failed to clear poisons every later
session, and the failure looks like a policy refusal rather than a bug. cbctl retries and
then returns `CBCTL_E_STOP_STUCK` rather than letting you proceed believing otherwise.

### 2.7 The wind-down — how a charge should end

The vehicle needs to be told, through the protocol, not by dropping the link:

1. `cbctl_stop(link)` **and** clear `power_ready` in the same breath — use
   `cbctl_begin_stop()`, which does both atomically and is the only correct way.
2. Keep ticking. The feed must stay alive: `PowerDelivery(Stop)`, `WeldingDetection` and
   `SessionStop` are still controller-fed phases.
3. Report the **real** measured voltage and current, and the contactor as open, once it is.
   `WeldingDetection` is the vehicle measuring your contactor with its own open — a fake 0 V
   here masks a real weld. On a bench with no modules 0 V is both true and safe; **in
   production report the meter verbatim.**
4. Keep ticking until the session is gone. There is **no blocking wait call** — a library
   that blocks cannot also keep your feed alive, and the feed is what the wind-down needs.
   Loop on `cbctl_tick()` and test `cbctl_flag(link, CBCTL_FLAG_SESSION_ACTIVE)`.

Skipping step 1's simultaneity is the classic error: raising the stop while still advertising
`power_ready` means the EVSE-stop frame is never sent, the vehicle never receives
`EVSENotification=StopCharging`, and PD(Stop) → Welding → SessionStop never happen. You get a
TCP reset instead of a clean close.

---

## 3. Identity, and the vehicle MAC

```c
typedef struct {
    uint8_t  mac[6];          /* the BOARD's MAC — its identity, not a stored id */
    uint8_t  wire_version;
    uint8_t  reset_reason;
    uint32_t uptime_ms;
    char     build[9];        /* 8 hex chars + NUL */
} cbctl_identity_t;
```

```c
cbctl_status_t cbctl_vehicle_mac(const cbctl_link_t *link, uint8_t out[6]);
```

**The vehicle's MAC is available at SLAC MATCH** — before any V2G session exists, and
therefore long before EVCCID. That is what makes it usable for Autocharge / Plug-and-Charge
by MAC: you can recognise a returning vehicle at the moment it associates, not after it has
negotiated.

`cbctl_vehicle_mac()` returns `CBCTL_E_NOT_YET` until the match has happened, so an
uninitialised buffer can never be mistaken for a real address.

Two identities, and they are not interchangeable:

| | source | available from | use |
|---|---|---|---|
| `vehicle_mac` | SLAC match (HomePlug) | **SLAC match** | Autocharge by MAC |
| `evcc_id` | `SessionSetupReq` (ISO 15118) | V2G session start | ISO 15118 identity, billing |

`cbctl_evcc_id()` likewise returns `CBCTL_E_NOT_YET` before a session, and reports the
length, because a 6-byte and an 8-byte EVCCID are different values and padding one into the
other is how a vehicle gets billed to the wrong account.

---

## 4. Events — synthesised, because the PLC does not push

**Correction to an earlier draft of this document.** It described a push-event channel with a
callback invoked when the PLC sent an EVENT message. That channel does not exist.
`CBPLC_T_EVENT` is defined in the protocol's type table and **the firmware never emits one** —
verified by grep, not by assumption. A library documenting it would have had callers waiting
for something that never arrives.

What actually exists is better suited to a controller anyway: the PLC publishes a large
STATUS snapshot on request, and a `chg_log` ring of the last state transitions with
timestamps. cbctl derives events by **diffing consecutive snapshots and draining that ring**,
so nothing is missed between polls even though the transport is strictly request/response.

```c
cbctl_event_t ev;
while (cbctl_next_event(&link, &ev)) {
    switch (ev.kind) { ... }
}
```

`cbctl_next_event()` is non-blocking and drains a bounded queue filled during `cbctl_tick()`.
It returns false when empty. There is no callback, no thread, and no interrupt context — the
library never calls you back, you always pull.

| kind | derived from |
|---|---|
| `CBCTL_EV_VEHICLE_PLUGGED` / `UNPLUGGED` | `cp` leaving / entering `A` |
| `CBCTL_EV_SLAC_MATCHED` | `slac_matched` 0→1 |
| `CBCTL_EV_VEHICLE_MAC_KNOWN` | `ev_mac_ok` 0→1 — **the Autocharge trigger** |
| `CBCTL_EV_SESSION_STARTED` / `ENDED` | `secc_streams` increment / `secc_active` 1→0 |
| `CBCTL_EV_CHARGE_STARTED` / `STOPPED` | `chg_log` entering / leaving `Energised` |
| `CBCTL_EV_FAULT` | `chg_log` entering `Fault`, carrying `chg_reason` |
| `CBCTL_EV_BOARD_RESET` | `up_ms` going backwards |

**Poll rate determines event resolution.** A transition shorter than your tick period is still
caught if it appears in `chg_log` — that ring exists precisely because a state machine can flap
faster than any poll rate — but a value that changes and changes back between two polls is
genuinely invisible. Tick at your configured cadence; do not stretch it and expect fidelity.

## 5. Multiple PLCs

Each PLC is one link. There is no addressing on the wire — a board is identified by the MAC
returned from `identify`, which cannot be misconfigured the way a stored id can.

```c
cbctl_fleet_t fleet;
cbctl_fleet_init(&fleet);
cbctl_fleet_add(&fleet, &link_a);
cbctl_fleet_add(&fleet, &link_b);

cbctl_fleet_tick(&fleet, now_ms);                    /* ticks every link      */
cbctl_link_t *l = cbctl_fleet_by_mac(&fleet, mac);   /* address by identity   */
```

Guarantees the fleet layer provides:

- **Fault isolation.** One link erroring never prevents the others being ticked. A dead board
  does not stop a healthy one delivering energy.
- **No shared mutable state.** Each link owns its own buffers, counters and state machine.
- **Duplicate detection.** `cbctl_fleet_add` refuses a MAC already present
  (`CBCTL_E_DUPLICATE`) — two links to one board is a wiring mistake that otherwise presents
  as intermittent nonsense, because both readers steal each other's replies.

---

## 6. Errors — every refusal, and what to do about it

| code | meaning | what you should do |
|---|---|---|
| `CBCTL_E_OK` | — | — |
| `CBCTL_E_NOT_IDENTIFIED` | called before `identify` | call `cbctl_identify` |
| `CBCTL_E_WRONG_STATE` | illegal in the current state | see §2; the library names the state |
| `CBCTL_E_NO_DEADMAN` | zero hold on GUN/AUX | supply a hold |
| `CBCTL_E_LEASE_TOO_SHORT` | lease < 3 × tick period | lengthen the lease or tick faster |
| `CBCTL_E_CONTRADICTORY` | `power_ready` while stopping | clear one of them |
| `CBCTL_E_RANGE` | a value outside its legal range | fix the value; the library says which |
| `CBCTL_E_REFUSED_NO_VEHICLE` | gun close with no vehicle | wait for `VEHICLE_PLUGGED` |
| `CBCTL_E_REFUSED_UNSAFE` | a safety interlock held | read `cbctl_refusal_detail()` |
| `CBCTL_E_STOP_STUCK` | stop did not clear after retries | investigate the board; do not proceed |
| `CBCTL_E_FEED_LATE` | you are behind your own cadence | tick more often |
| `CBCTL_E_TIMEOUT` | no reply within the deadline | retry; if persistent, the link is down |
| `CBCTL_E_LINK` | transport failure | reconnect; see §8 |
| `CBCTL_E_DUPLICATE` | that MAC is already in the fleet | you have two links to one board |
| `CBCTL_E_NOT_YET` | the value does not exist yet | wait for the relevant event |

`cbctl_refusal_detail()` returns the PLC's own breakdown of a refusal — which of
drift / safety / vehicle-present / controller-alive / authorised / no-stop-in-force held. A
bare "not ready" sends you guessing across six gates; this is why the firmware reports all
six and why the library surfaces them individually.

### 6.1 Live values vs cumulative counters

**This distinction has caused four separate bugs in this project.** cbctl makes it a type
error rather than a naming convention:

```c
uint32_t v = cbctl_counter(link, CBCTL_CTR_SESSIONS);       /* cumulative: only rises   */
bool     a = cbctl_flag(link, CBCTL_FLAG_SESSION_ACTIVE);   /* live: true right now     */
```

There is no call that returns a counter as a boolean, and no flag expressed as a count.
Testing a cumulative counter for zero to mean "nothing is happening" can only ever be true
before the first occurrence of a boot — which is exactly the bug that disabled a watchdog for
the entire life of a charger.

---

## 7. Time

The library takes `now_ms` from you and never reads a clock itself. Requirements:

- **Monotonic.** Never steps backwards. `CLOCK_MONOTONIC` on POSIX, a tick counter on bare
  metal. Not wall clock: an NTP correction stepping backwards would make deadlines fire.
- **Wrapping at 2^32 is fine and expected.** Every comparison inside cbctl is a signed
  difference clamped at zero, so a 49.7-day wrap is a non-event and a stamp that is a few
  milliseconds in the *future* — which happens when a reply is processed in the same pass
  that stamps it — reads as age zero rather than as 49 days.

---

## 8. Transport

```c
typedef struct {
    void *ctx;
    int  (*send)(void *ctx, const uint8_t *buf, size_t len);   /* -1 on failure       */
    int  (*recv)(void *ctx, uint8_t *buf, size_t cap);         /* 0 = nothing waiting */
    void (*close)(void *ctx);
} cbctl_transport_t;
```

What cbctl guarantees to the transport: `send` is never called with more than
`CBCTL_MTU` bytes; both are called only from `cbctl_tick`, never
concurrently, never from an interrupt.

What the transport must guarantee: **`send` must not block indefinitely.** A blocking write
against a device that has stopped reading stalls the tick, which stalls the feed, which stops
the charge. On POSIX set a write timeout; return `-1` rather than hanging. This is not
hypothetical — an unbounded write blocked a controller feed for ten seconds and killed a live
charging session.

Provided implementations: `cbctl_posix_serial` (Linux/macOS, USB-CDC or UART) and
`cbctl_transport_loopback` for tests. UART and CAN are the same interface; CAN additionally
needs the segmentation configured for an 8-byte MTU, which the protocol already supports
(`CBPLC_MIN_TRANSPORT_MTU` is 32 today — CAN support requires lowering that floor, see
`docs/03-roadmap.md`).

---

## 9. Worked sequences

### 9.1 A complete successful charge

```c
cbctl_open(&link, &tp, &cfg);
cbctl_identify(&link, &id, 3000);
cbctl_stop_clear(&link);
cbctl_authorize(&link, CBCTL_AUTH_PENDING, 0);

for (;;) {
    cbctl_tick(&link, now_ms());

    cbctl_event_t ev;
    while (cbctl_next_event(&link, &ev))
    switch (ev.kind) {
    case CBCTL_EV_VEHICLE_PLUGGED:
        /* the vehicle is present; nothing to authorise yet */
        break;

    case CBCTL_EV_VEHICLE_MAC_KNOWN:
        cbctl_vehicle_mac(&link, mac);        /* Autocharge lookup happens HERE */
        if (autocharge_known(mac))
            cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 30000);
        break;

    case CBCTL_EV_SESSION_STARTED:
        cbctl_set_envelope(&link, &(cbctl_envelope_t){
            .max_voltage_mv = 500000, .min_voltage_mv = 150000,
            .max_current_ma = 200000, .min_current_ma = 1000,
            .max_power_w    = 100000, .power_ready = true });
        break;

    case CBCTL_EV_CHARGE_STARTED:
        cbctl_relay_close(&link, CBCTL_RELAY_GUN, 5000);   /* renewed by tick */
        break;
    }

    /* every time your measurements change — the library repeats them for you */
    cbctl_set_measurement(&link, &(cbctl_measurement_t){
        .present_voltage_mv = meter_v(), .present_current_ma = meter_a(),
        .isolation = imd_verdict(), .contactor = contactor_state() });

    if (should_stop()) break;
}
```

### 9.2 Ending it cleanly

```c
cbctl_begin_stop(&link);                 /* stop + power_ready=false, atomically */
while (cbctl_flag(&link, CBCTL_FLAG_SESSION_ACTIVE)) {   /* keep ticking: the feed must live */
    cbctl_tick(&link, now_ms());
    cbctl_set_measurement(&link, &(cbctl_measurement_t){
        .present_voltage_mv = meter_v(),   /* VERBATIM — never a fabricated 0 */
        .present_current_ma = meter_a(),
        .isolation = CBCTL_ISO_VALID,
        .contactor = CBCTL_CONTACTOR_OPEN });
}
cbctl_stop_clear(&link);                 /* so the next session is not poisoned */
```

### 9.3 The vehicle refuses, or never returns

A vehicle that has completed a charge re-associates **on its own schedule**. Measured on a
real EV: 169 s to over 300 s, and unmoved by a correct `E → B1 → B2` restart edge. The
field-proven reference firmware shows the same behaviour on the same vehicle.

Do not treat this as a charger fault, and do not build a retry loop that assumes otherwise.
`cbctl_request_restart()` drives the restart edge — which is correct and worth doing — but
its contract is explicitly *"ask the vehicle to re-sound"*, not *"make it"*.

---

## 10. What this library deliberately does not do

- **It does not decide.** Authorisation, tariffs, module control and CSMS interaction are
  yours. cbctl carries decisions; it does not make them.
- **It does not fabricate measurements.** There is no "assume isolation is fine" mode. If you
  have not measured, say `UNKNOWN` and the charger will behave correctly.
- **It does not hide a rebooting board.** Re-establishment is automatic, but the event is
  always delivered.
- **It does not retry a refusal.** A refusal is a decision by the safety layer. Retrying it in
  a loop is how a controller turns a clear signal into a mystery.
</content>
