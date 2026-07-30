# cbctl — design and safety model

This is the **plan**: what cbctl is made of, why each piece exists, and — the point of the
whole exercise — the specific mechanism by which each known controller mistake is made
impossible rather than merely documented.

Read `01-api-reference.md` for *what to call*. This document is *why it is shaped that way*.

---

## 1. The design rule

> **Every rule in the API reference exists because somebody already broke it.**

That is not rhetoric. The list in §4 is drawn from defects found in this codebase and on this
bench, most of them within a single week. A library that only *documents* those rules will
see them broken again, because the failure mode of documentation is that it is not read at
the moment of the mistake.

So the ordering principle is:

1. **Make it impossible** — a type the wrong value cannot inhabit, or a function that does not
   exist for the illegal case.
2. Failing that, **make it fail at the call** — a state check that returns a named error
   before anything reaches the wire.
3. Failing that, **make it fail loudly at runtime** — a counter and an event, never a silent
   degradation.
4. Only then, document it.

A rule that lands at level 4 when it could have landed at level 1 is a design failure, not a
documentation gap.

---

## 2. What the library is made of

```
cbctl/
  include/cbctl/
    cbctl.h            the whole public API — one header, deliberately
    types.h            units, enums, opaque handles
    transport.h        the vtable a port implements
  src/
    link.c             one PLC: state machine, tick, request/response correlation
    feed.c             LIMITS/FEEDBACK repetition and staleness accounting
    lease.c            AUTHORIZE and RELAY leases, renewal, expiry
    state.c            the controller-side session state machine + legality table
    (no codec — cbplc's message/segment/session/stream_frame.c are vendored and reused)
    fleet.c            N links, isolation, addressing by MAC
    status.c           STATUS parsing: live flags vs cumulative counters, typed
  ports/
    posix_serial.c     Linux/macOS USB-CDC and UART
    loopback.c         an in-process fake PLC, for tests
  tests/               host tests, no hardware, no framework
```

**Layering rule:** `src/` is pure C11 with no OS calls, no allocation, no `printf`, no time
source. Everything platform-specific lives in `ports/`. This is the same split that lets
cbplc's core be host-tested with no board attached, and it is why the same source will run on
a Pi and on a bare-metal MCU.

**No dynamic allocation, anywhere.** A link is a caller-provided struct. A fleet is a
caller-provided array. An embedded controller must be able to see its entire memory cost at
compile time, and a charger is not a place to discover an allocator failure.

---

## 3. The type model

The purpose of the types is that **a wrong value should not be expressible**, not that it
should be caught.

### 3.1 Units are types, not comments

```c
typedef struct { uint32_t mv; } cbctl_millivolts_t;
typedef struct { uint32_t ma; } cbctl_milliamps_t;
typedef struct { uint32_t w;  } cbctl_watts_t;
```

Single-field structs, because C will happily let you pass a `uint32_t` of milliamps where
millivolts were wanted and a struct will not. Construct with `CBCTL_MV(500000)`, which is a
compound literal, so there is no runtime cost and no ambiguity at the call site:

```c
cbctl_set_envelope(&link, &(cbctl_envelope_t){
    .max_voltage = CBCTL_MV(500000),
    .max_current = CBCTL_MA(200000),
    .max_power   = CBCTL_W(100000),
});
```

Designated initialisers are mandatory in the reference examples for the same reason: a
five-integer positional call is a transposition waiting to happen, and the compiler cannot
help you.

### 3.2 Enums for everything with a fixed vocabulary

No `int` parameters with a documented range. `cbctl_auth_t`, `cbctl_isolation_t`,
`cbctl_contactor_t`, `cbctl_relay_mask_t`, `cbctl_event_kind_t` are all enums, and the decode
path **rejects an unrecognised enumerant rather than mapping it to a default**. A newer PLC's
value must never read as one of ours.

`cbctl_isolation_t` specifically:

```c
CBCTL_ISO_UNKNOWN = 0,   /* NOT MEASURED YET — the default, and not the same as INVALID */
CBCTL_ISO_INVALID = 1,   /* measured, and bad */
CBCTL_ISO_VALID   = 2,
CBCTL_ISO_WARNING = 3,
CBCTL_ISO_FAULT   = 4,
```

`UNKNOWN` being zero is deliberate: a zeroed struct means "I have not measured", which is the
safe default. A controller that forgets to set isolation gets correct conservative behaviour
rather than an accidental claim of a passed insulation test.

### 3.3 Opaque handles

`cbctl_link_t` and `cbctl_fleet_t` are opaque: declared in the header with a size, defined in
the implementation. Callers cannot reach in and set a field the state machine depends on.

---

## 4. Every known misuse, and the mechanism that prevents it

This table is the core of the design. Each row is a real defect.

| # | The mistake | Prevented by | Level |
|---|---|---|---|
| 1 | Relay close with `hold_ms = 0` — a close with no dead-man | `cbctl_relay_close` returns `CBCTL_E_NO_DEADMAN` for GUN/AUX with zero hold | fail at call |
| 2 | Letting LIMITS/FEEDBACK go stale mid-session | `cbctl_tick` owns repetition; the caller never writes a resend loop | **impossible** |
| 3 | Forgetting STOP is latched, poisoning the next session | `cbctl_open` refuses to proceed until the latch state is read; `stop_clear` verifies by readback | fail at call |
| 4 | Reading a cumulative counter as a live value | `cbctl_counter()` and `cbctl_flag()` are separate calls over disjoint enums | **impossible** |
| 5 | Assuming a RELAY command applied when it was never acked | no way to express "applied" but `cbctl_op_state()`, and `PENDING` is a distinct value from `APPLIED` | **impossible** |
| 6 | Treating an absent measurement as safe | `CBCTL_ISO_UNKNOWN` is the zero value and distinct from `INVALID` | **impossible** |
| 7 | Two processes on one port, stealing each other's replies | `posix_serial` opens exclusively; `fleet_add` refuses a duplicate MAC | fail at call |
| 8 | `power_ready` asserted while stopping | `CBCTL_E_CONTRADICTORY`; `cbctl_begin_stop()` sets both atomically | fail at call |
| 9 | Auth lease shorter than the tick period | `CBCTL_E_LEASE_TOO_SHORT`, checked against the configured cadence | fail at call |
| 10 | Unbounded blocking write stalling the feed | no blocking call exists in the API at all; the transport contract forbids a blocking `send` | **impossible** |
| 11 | Backwards or wall-clock `now_ms` | `cbctl_tick` asserts on a backwards step; all ages are signed and future-tolerant | fail loudly |
| 12 | Calling a command that is illegal in the current state | the legality table in `state.c`; `CBCTL_E_WRONG_STATE` names the state | fail at call |
| 13 | Retrying a safety refusal in a loop | refusals are a distinct error class from transient failures, with a per-gate breakdown | fail at call |
| 14 | Mistaking a rebooted board for a working one | `CBCTL_EV_BOARD_RESET` is always delivered; leases are re-established but never silently | fail loudly |
| 15 | Fabricating 0 V at WeldingDetection, masking a weld | documented at the call site *and* `01-api-reference.md` §2.7; cannot be prevented by types — the value is genuinely the caller's | document |
| 16 | Confusing the vehicle MAC with the EVCCID | separate calls, separate types, both `CBCTL_E_NOT_YET` before they exist | **impossible** |

Row 15 is the only one that lands at level 4, and it is worth naming why: the library cannot
tell a true 0 V from a fabricated one. That is a genuine limit, so it is stated plainly rather
than papered over.

---

## 5. The state model

cbctl maintains its own view of the session and refuses calls that are illegal in it. The
states mirror what the PLC reports, so the two cannot drift:

```
        ┌──────────┐  vehicle plugged   ┌───────────┐  authorised   ┌──────────┐
        │  IDLE    │ ─────────────────► │ AWAIT_AUTH│ ────────────► │ LINKING  │
        └──────────┘                    └───────────┘               └──────────┘
             ▲                                                            │ SLAC matched
             │ unplugged (from any state)                                 ▼
             │                          ┌──────────┐  V2G grants    ┌─────────────┐
             └───────────────────────── │ STOPPING │ ◄───────────── │ NEGOTIATING │
                                        └──────────┘                └─────────────┘
                                             ▲                            │ energy allowed
                                             │  stop / fault / unplug     ▼
                                             └──────────────────────  ENERGISED
```

The legality table (excerpt) — a call outside its allowed set returns `CBCTL_E_WRONG_STATE`
with the current state named:

| call | legal in |
|---|---|
| `cbctl_authorize` | any (a decision is always allowed) |
| `cbctl_set_envelope` / `set_measurement` | any (the feed is always allowed) |
| `cbctl_relay_close(GUN)` | `NEGOTIATING`, `ENERGISED` — and the PLC still enforces its own interlock |
| `cbctl_begin_stop` | `NEGOTIATING`, `ENERGISED`, `STOPPING` |
| `cbctl_request_restart` | `AWAIT_AUTH`, `LINKING`, `NEGOTIATING` — never while energised |

**The library's checks are additional to the PLC's, never a replacement.** cbctl refusing a
call early is a convenience; the PLC refusing it is the safety property. Anyone reading this
should assume the PLC is the authority, because it is.

---

## 6. Configuration

Compile-time, in `cbctl_config.h`, all with defaults that work:

| knob | default | why |
|---|---|---|
| `CBCTL_FLEET_MAX` | 8 | measured to N=8 (fleet_bench: worst pass 362 us on a Pi 4B); the worst-pass arithmetic supports ~12, so treat raises past that with a fresh measurement, not the old "raise freely" |
| `CBCTL_MTU` | 1040 | matches cbplc's transport; lower to 32 for CAN |
| `CBCTL_RX_BUF` | 4352 | one max message plus framing |
| `CBCTL_STATUS_KEYS` | 192 | the parser's key table; cbplc emits ~165 today |
| `CBCTL_NO_ASSERT` | unset | define to compile out assertions for a release MCU build |

Runtime, in `cbctl_config_t` passed to `cbctl_open`:

| field | default | notes |
|---|---|---|
| `tick_period_ms` | 400 | the cadence you promise to meet; lease checks are derived from it |
| `feed_period_ms` | 400 | how often LIMITS/FEEDBACK are repeated |
| `request_timeout_ms` | 1000 | internal to the reconciler — **never a caller-facing wait** |
| `request_attempts` | 3 | a lost reply is normal on a busy link |
| `auto_reestablish` | true | re-assert auth and leases after a board reset |

---

## 7. Testing, with no hardware

The `loopback` port is an in-process fake PLC that speaks the real wire protocol and models
the real refusals — the interlocks, the staleness bounds, the stop latch, the lease expiry.
Every rule in §4 has a test that **exercises the mistake and asserts the library refuses it**.

Two disciplines carried over from cbplc, both of which earned their place:

- **Mutation-verify every safety test.** Reintroduce the defect and confirm the test fails.
  A test that has never failed has not been shown to test anything.
- **A wiring check.** Every `cbctl_*_init` must have a production caller. In cbplc this fired
  three times, each a genuine "initialiser exists, nothing calls it" bug.

---

## 8. Roadmap

| phase | contents |
|---|---|
| 1 | `types.h`, `transport.h`, `codec.c`, loopback port, wire round-trip tests |
| 2 | `link.c` + `state.c`: open/identify/tick, the legality table, request correlation |
| 3 | `feed.c` + `lease.c`: repetition, staleness, auth and relay leases |
| 4 | `status.c`: typed live/cumulative split; events |
| 5 | `posix_serial` port; a worked Pi example end to end |
| 6 | `fleet.c`: N links with isolation |
| 7 | CAN transport — needs `CBPLC_MIN_TRANSPORT_MTU` lowered from 32 to 8, which is a cbplc change and a wire-compatibility decision, not a cbctl one |

Phase 7 is called out because it is the one item that requires a change on the **PLC** side.
Everything else is additive and needs no firmware change: cbctl speaks the protocol that
ships today.
