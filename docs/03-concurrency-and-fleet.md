# cbctl — concurrency, non-blocking I/O, and managing N PLCs

This document answers one question: **how does a controller drive several PLC boards at once
without one of them harming the others?**

It supersedes the parts of `01-api-reference.md` and `02-design-and-safety-model.md` that
described synchronous calls. Those were wrong, and §1 shows the arithmetic that proves it.

---

## 1. Why a blocking API is not merely slow — it destroys charges

The earlier drafts described calls like `cbctl_identify(&link, &ident, timeout)` and stated
that `cbctl_relay_close()` "is synchronous and returns the PLC's answer". That is a design
defect, and the numbers are not close.

Every constant below is read from the shipping firmware, not assumed:

| constant | value | source |
|---|---|---|
| `CBPLC_DEF_REQUEST_TIMEOUT_MS` | 1000 ms | `include/cbplc/protocol_generated.h` |
| `CBPLC_DEF_RETRY_LIMIT` | 3 | same |
| `CBPLC_DEF_RETRY_BACKOFF_MS` | 100 ms | same |
| `CBPLC_LIMITS_STALE_MS` | **1500 ms** | `include/cbplc/limits.h` |

A synchronous request that exhausts its retries occupies the caller for

```
3 × (1000 ms + 100 ms) = 3300 ms
```

while the deadline every *other* PLC is holding the controller to is **1500 ms**. So:

> **One blocking call to one unresponsive PLC exceeds the liveness deadline of every other
> PLC by a factor of 2.2, and the correct, designed response of those PLCs is to withdraw
> permission to energise and terminate their sessions.**

A driver on gun 2 loses their charge because gun 1's board stopped answering. Nothing is
faulty except the controller's API shape. The blast radius of a single wedged board is the
entire charger.

This is not hypothetical arithmetic. It is the same class of defect already recorded in this
project's history: a controller-side intake gate that applied write backpressure stalled the
feeder, drove `meas_age` from 328 ms to 10202 ms, lapsed the authorisation and killed a live
charge. The mechanism — *one slow thing starving the periodic feed* — is identical. It has
happened here before, with a real vehicle attached.

**Therefore the central rule of this library:**

> ### cbctl exposes no blocking call. Not one. There is no timeout parameter anywhere in the public API.
>
> A function that waits cannot also keep N feeds alive, and the feed is the safety property.

### 1.1 The foundation was already correct — the drafts discarded it

The uncomfortable part is that cbplc never asked to be wrapped this way. Its transport
contract is explicit, and it is in the shipping header:

```c
/* Send one frame. ... Must not block. */
cbplc_err_t (*send)(void *ctx, const uint8_t *frame, size_t len);

/* Receive at most one frame. Sets *len to 0 and returns CBPLC_E_TIMEOUT when
 * nothing is waiting — the normal idle case. Never blocks. */
cbplc_err_t (*recv)(void *ctx, uint8_t *frame, size_t cap, size_t *len);
```

And its session layer is already a poll-driven state machine, not a call-and-wait:

```c
cbplc_err_t cbplc_request(cbplc_session_t *s, ...);   /* submit — returns immediately */
cbplc_err_t cbplc_poll   (cbplc_session_t *s, uint32_t now_ms);  /* step   */
bool        cbplc_busy   (const cbplc_session_t *s);  /* observe */
```

Its retry logic is written to resume on the next poll rather than to spin — the source says
so at the retry site: *"rewind by one and retry next poll … BOUNDED, and that bound is the
whole point."*

So the fix is not to invent an async architecture. It is to **stop hiding the one cbplc
already has.** cbctl's job is to preserve `submit / step / observe` all the way to the public
API, adding policy on top rather than a blocking convenience layer that defeats it.

---

## 2. The model: declare intent, reconcile on tick

You never send a message. You **declare the state you want**, and the tick reconciles that
intent against the board, repeatedly, forever.

```c
cbctl_authorize(&link, CBCTL_AUTH_GRANTED, 8000);   /* returns instantly — records intent  */
cbctl_relay_close(&link, CBCTL_RELAY_GUN, 2000);    /* returns instantly — records intent  */
cbctl_set_envelope(&link, &env);                    /* returns instantly — records intent  */

cbctl_tick(&fleet, now_ms);                         /* the ONLY thing that touches the wire */
```

`cbctl_tick()` is bounded: it does a fixed, small amount of work per link per call and
returns. It never waits for a reply. A reply that has not arrived is simply not there yet, and
the next tick looks again.

This single choice eliminates most of the misuse list before the type system is involved. You
cannot forget to repeat the feed, cannot forget to renew a lease, cannot fire-and-forget a
relay, and cannot let AUTHORIZE lapse — because none of those are things you do. They are
things the tick does, from a declaration you made once.

### 2.1 Observing completion without waiting for it

Intent is not confirmation, and the library never pretends otherwise. Every declaration
returns a handle you may — but need not — watch:

```c
cbctl_op_t op = cbctl_relay_close(&link, CBCTL_RELAY_GUN, 2000);

switch (cbctl_op_state(&link, op)) {
case CBCTL_OP_PENDING:   break;                 /* still in flight; keep ticking      */
case CBCTL_OP_APPLIED:   break;                 /* the PLC acked it                   */
case CBCTL_OP_REFUSED:   gate = cbctl_op_gate(&link, op); break;  /* named interlock  */
case CBCTL_OP_LOST:      break;                 /* retries exhausted; link suspect    */
}
```

The old draft claimed misuse #5 ("assuming a RELAY command applied when it was never acked")
was made *impossible* by synchronous return. It is instead made impossible by there being **no
way to express "applied"** other than reading the op state — and `PENDING` is a distinct value
from `APPLIED`, so a caller who ignores the difference is not silently told "yes".

### 2.2 What replaces the blocking startup sequence

`cbctl_identify()` previously took a timeout. It now declares intent, and the link reports its
own readiness as it progresses:

```c
CBCTL_LINK_CLOSED → OPENING → IDENTIFYING → READY → (DEGRADED) → QUARANTINED
```

Your control loop starts ticking immediately and simply does not use a link until
`cbctl_link_state()` says `READY`. Nothing blocks while a board boots, and a board that never
boots costs you nothing but a link that stays in `IDENTIFYING`.

---

## 3. The fleet: N links, one tick, guaranteed isolation

```c
cbctl_link_t  links[4];
cbctl_fleet_t fleet;

cbctl_fleet_init(&fleet, links, 4);
cbctl_fleet_add(&fleet, &links[0], &transport_gun1, &cfg);
cbctl_fleet_add(&fleet, &links[1], &transport_gun2, &cfg);

for (;;) {
    cbctl_tick(&fleet, monotonic_ms());   /* services EVERY link, bounded */
    sleep_until_next(&fleet);
}
```

### 3.1 The isolation guarantees, stated as testable properties

**A correction, from adversarial review of the prior art.** An earlier draft of this section
justified isolation with "no blocking call exists". That reasoning is wrong, and the exact
claim — *concurrency comes from non-blocking state machines advanced by events, so a stalled
transfer does not block its siblings* — was put to three independent verifiers against
libcurl's own documentation and **refuted 0–3**. libcurl retains residual blocking paths, and
OpenSSL's `WANT_READ`/`WANT_WRITE` contract is documented to give *no fixed upper limit* on
iterations before application-level progress. Being non-blocking bounds no deadline by itself.

So the honest statement is:

> Non-blocking I/O is **necessary but not sufficient** for isolation. It removes the
> 3300 ms-vs-1500 ms defect in §1, and nothing more. Isolation is delivered by three
> *additional* mechanisms that have to be designed in deliberately: **per-link deadlines**,
> **bounded work per tick**, and **explicit quarantine**.

The design already carried all three (F3, F6). Only the reasoning was wrong — which is worth
recording, because a rule believed for the wrong reason is the first one dropped during a
refactor.

Each guarantee below has a loopback test that wedges one link and asserts the others are
untouched.

| # | Guarantee | Mechanism |
|---|---|---|
| F1 | A link that never answers cannot delay another link's feed | **per-link deadlines**, independent and each armed from that link's own last exchange — not non-blocking I/O alone (see above) |
| F2 | A link whose transport `send` refuses cannot stall the loop | `send` is contractually non-blocking; a refusal defers that link's frame to the next tick, and only that link's |
| F3 | Work per tick is bounded regardless of N and of inbound volume | each link drains at most `CBCTL_RX_PER_TICK` frames, then yields — a **bounded quantum**, the second of the three mechanisms |
| F4 | Links are serviced fairly; no link is starved by a chattier one | round-robin start index rotates each tick |
| F5 | One link's fault, reset, or quarantine changes no other link's state | links share no mutable state — the fleet owns only the array and the rotation index |
| F6 | A wedged link is detected and contained, not retried forever | consecutive-loss counter → `QUARANTINED` with capped re-probe backoff |

F5 is worth dwelling on. The fleet is deliberately **not** an abstraction that owns the links'
behaviour; it is a loop over independent state machines with a rotation counter. There is no
shared buffer, no shared request-id space, no shared clock stamp. Isolation is structural. It
is not a property maintained by careful coding, which is the kind of property that decays.

### 3.2 Bounded work, and why fairness needs an explicit rotation

A naïve `for (i = 0; i < n; i++) service(link[i])` is unfair under load: link 0 is always
serviced first, and if each link may consume up to `CBCTL_RX_PER_TICK` frames, link 3 is
serviced last on every single tick and is the first to suffer when the loop runs late. Over
thousands of ticks that bias is systematic, not random. So:

```c
for (k = 0; k < n; k++) {
    i = (fleet->rr + k) % n;      /* rotation start moves every tick */
    service(&fleet->links[i], now_ms);
}
fleet->rr = (fleet->rr + 1) % n;
```

Cheap, and it converts a systematic bias into an even one.

### 3.3 Quarantine — containing a wedged board

A board that stops answering must not consume the loop's attention forever, and must not be
retried at full rate indefinitely (a dead USB endpoint that errors instantly turns a retry
loop into a busy-wait). After `CBCTL_LOSS_LIMIT` consecutive lost requests the link enters
`QUARANTINED`: its feed stops, its ops fail fast with `CBCTL_E_QUARANTINED`, and it is
re-probed on an exponential backoff capped at `CBCTL_REPROBE_MAX_MS`.

Quarantine is **loud**: it raises `CBCTL_EV_LINK_QUARANTINED`. A charger that silently
demotes a connector is worse than one that fails, because the operator learns about it from a
customer.

Crucially, quarantining gun 1 does not touch gun 2's session. That is F5, and it is the whole
point of the exercise.

---

## 4. Integrating with the caller's event loop

cbctl never owns the loop, never creates a thread, and never calls `sleep`. A charger
controller already has a main loop — OCPP, HMI, CAN, its own state machines — and a library
that demands ownership of it is a library that gets wrapped in a thread, which reintroduces
every concurrency problem this design removes.

Two integration styles, one API. This is the `libcurl multi` / `sd_event` idiom: **give me
your descriptors and tell me when to call you next.**

```c
/* How long may the caller sleep before cbctl needs servicing?
   0 = service now. CBCTL_FOREVER = nothing pending. */
uint32_t cbctl_next_deadline_ms(const cbctl_fleet_t *fleet, uint32_t now_ms);

/* Descriptors to wait on, for hosts that have them. Returns the count written. */
size_t cbctl_pollfds(const cbctl_fleet_t *fleet, struct pollfd *out, size_t cap);
```

**Hosted (Pi, Linux SBC)** — sleep precisely, wake on data or deadline:

```c
size_t n  = cbctl_pollfds(&fleet, fds, 8);
int    ms = (int)cbctl_next_deadline_ms(&fleet, monotonic_ms());
poll(fds, n, ms);
cbctl_tick(&fleet, monotonic_ms());
```

**Bare metal / RTOS** — no descriptors exist, so ignore them and tick on your existing timer:

```c
if (tick_elapsed(400)) cbctl_tick(&fleet, monotonic_ms());
```

`cbctl_pollfds()` returns 0 on ports with no descriptors, which is not an error — it is the
honest answer, and it degrades to the polled form with no `#ifdef` in the caller. The
descriptor comes from the port (`ports/posix_serial.c`), never from `src/`, which keeps the
core free of OS calls.

**The asymmetry is deliberate and is the reviewed conclusion of the prior art:** of the two
halves of the classic "get fd + get timeout" idiom, only one ports. A descriptor presupposes
an OS that has them; a *next deadline* is meaningful on every target including a bare-metal
timer loop. So in cbctl:

> `cbctl_next_deadline_ms()` is **mandatory** and every port must answer it.
> `cbctl_pollfds()` is **optional** and may honestly return 0.

A note on currency, since this idiom has moved: curl's own documentation now labels the
`curl_multi_fdset` + `curl_multi_timeout` pair **"the older API"** and steers event-driven
callers to `curl_multi_socket_action` with socket/timer callbacks; the poll-shaped
`curl_multi_waitfds` only appeared in 8.8.0. cbctl uses the pull shape with `struct pollfd`
knowingly. The push shape exists to avoid rebuilding an `fd_set` over thousands of sockets
every iteration — a real problem at curl's scale and a non-problem at `CBCTL_FLEET_MAX` = 4.
Choosing the simpler contract here is a scale judgement, not an oversight, and it is recorded
as such so nobody "modernises" it without a reason.

### 4.1 Threads

cbctl is **not** thread-safe, deliberately, and this is a feature rather than an omission. A
`cbctl_fleet_t` is single-threaded: one loop drives all links. That is sufficient because
nothing blocks — the entire reason to reach for threads here would be to hide a blocking call
that does not exist.

If your architecture genuinely requires one thread per PLC, give each thread its **own**
fleet with its own links and its own transports. Sharing a fleet across threads is
unsupported, and the library will not paper over it with a mutex you did not ask to pay for.
A lock inside a control loop with a 1500 ms deadline is a priority-inversion waiting to be
diagnosed at 3am.

That escape hatch is not a grudging concession — it is the shape a safety standard reaches
for. AUTOSAR's remedy for one task interfering with a supervised periodic tick is **task
separation**, not merely writing non-blocking code. cbctl's single-threaded core with
per-thread fleets supports exactly that partitioning if a deployment needs it, without
imposing its cost on the common case where one loop drives four boards comfortably. (Stated
carefully: that is the direction the standard points, not a rule cbctl claims to satisfy —
see §8's warning about overstating safety-standards support.)

---

## 5. Timing budget

With the default 400 ms cadence against a 1500 ms deadline, the controller gets **three
attempts** before a feed goes stale. That margin is deliberate: one lost frame must not be
able to cost a session.

| quantity | value | note |
|---|---|---|
| tick cadence | 400 ms | 3.75× margin on the 1500 ms bound |
| feed repeat | every tick | LIMITS + FEEDBACK |
| worst-case tick cost | bounded by `CBCTL_RX_PER_TICK` × N | no unbounded loop, no wait |
| auth lease | 8000 ms default | ≫ tick, so one missed tick cannot lapse it |
| heartbeat timeout | 32000 ms default | the outer dead-man |
| feed grace | 10000 ms | past this the session fails cleanly, not by TCP reset |

**The tick must be called even when you have nothing to say.** That is what a dead-man is: the
absence of a tick is the signal. A controller that ticks only when its values change has
built a system where a hung controller looks exactly like a steady one.

### 5.1 The clock

`now_ms` is yours to supply and must be **monotonic**. Not `gettimeofday`, not anything NTP
can step. This project has a recorded incident where a clock step caused a watchdog to
compute a bogus 921-second silence and kill a healthy controller.

All ages are computed signed and future-tolerant:

```c
int32_t age = (int32_t)(now_ms - stamp_ms);
if (age < 0) age = 0;      /* stamped after `now` was captured — fresh, not 49 days old */
```

An unsigned subtraction here wraps to ~4.29 billion and reads the *freshest possible* feed as
49.7 days expired. That defect is why cbplc's `limits.c` and `auth.c` carry an explicit signed
`stale()` helper today: it passed every idle test and failed every real session.

---

## 6. What this changes in the existing docs

| doc | claim | status |
|---|---|---|
| `01` §2.1 | `cbctl_identify(&link, &ident, TIMEOUT)` | **wrong** — no timeouts; link state machine instead (§2.2) |
| `01` §2.5 | "`cbctl_relay_close` is synchronous" | **wrong** — returns `cbctl_op_t`; observe via `cbctl_op_state()` |
| `02` §4 row 5 | prevented "by synchronous return" | mechanism replaced: `PENDING ≠ APPLIED` (§2.1) |
| `02` §6 | `request_timeout_ms`, `request_attempts` | internal to the reconciler; not caller-facing waits |
| `02` §2 | `codec.c` | **delete** — vendor cbplc's `message/segment/session/stream_frame.c` |

The last row matters beyond tidiness. Those four files are already pure C11, allocation-free,
OS-free, transport-agnostic and host-tested, and they carry every field fix this project has
paid for — the CRC, the segment joiner, the bounded retry. Reimplementing them would create a
second implementation that can disagree with the firmware's, which is precisely the bug class
a shared protocol core exists to prevent.

---

## 7. Two things that must never be done

### 7.1 Do not move the feed off the tick

Somebody will eventually propose this, because on paper it is elegant: offload the periodic
feed to a cyclic transmitter — SocketCAN's `CAN_BCM` `TX_SETUP`, a DMA ring, an RTOS timer —
so that application jitter can no longer make it late.

**It inverts the safety property and must not be done.** The feed is a *dead-man*. Its meaning
is not "these are the current limits"; its meaning is **"the controller is still alive and
still supervising this connector"**. A kernel or peripheral that keeps emitting it while the
control loop is hung tells the PLC exactly the thing that is not true, and the PLC's whole
withdrawal-of-permission mechanism — the thing that protects the vehicle — is defeated
silently. The correct behaviour when the controller stalls is that the feed **stops**.

The verifiers refuted the framing that this protects the lease, 0–3. Record it as a
prohibition with the reason attached, because the idea is attractive enough to be reinvented.

*(There is a credit-consuming variant with real standards precedent — a supervisor that
refreshes a bounded credit which the transmitter spends down, so a hung supervisor still
results in silence. That is AUTOSAR WdgM's actual shape, §8. It is a legitimate future
option, and it is explicitly **not validated against any implementation** — do not build it
on the strength of this paragraph.)*

### 7.2 Do not let a port's `send` block inside the tick

cbctl's transport `send`/`recv` are callbacks invoked **inside** `cbctl_tick()`. The prior art
carries a precise warning here: nghttp2 is a sans-IO library, but only its buffer API is truly
bytes-in/bytes-out — its *callback* variant runs the application's `write()` inside the
library call, so a blocking application callback blocks the library regardless of how
carefully the library was written.

That is exactly our shape, so the guarantee lives entirely in the port. On POSIX a serial
`write()` **will** block when the TX buffer fills and the peer has stopped reading — which is
precisely what a wedged PLC looks like. `ports/posix_serial.c` therefore must open `O_NONBLOCK`
and return `CBPLC_E_TIMEOUT` on `EAGAIN` rather than waiting. A port that blocks re-creates the
§1 defect underneath an API that promises it cannot happen, which is worse than the original
because the promise stops anyone looking.

This is the single largest residual hazard in the design, and it is a **port** review item on
every new transport.

## 8. Prior art: what it actually supports

Sourced and adversarially verified rather than asserted. Listed with its limits, because the
limits are load-bearing.

| Design element here | Precedent | Strength |
|---|---|---|
| Core does no I/O; app owns the loop | nghttp2 — *"it does not perform any I/O … makes embedding very easy. Usually, the existing applications have its own I/O event loops"* | **high**, verified verbatim |
| One reconcile call re-entered by both readiness and timer expiry | libcurl `curl_multi_socket_action` (timer path re-enters via a sentinel) | **high** |
| One nearest-in-time deadline, not N timers | libcurl `CURLMOPT_TIMERFUNCTION`; sd-event | **high** |
| Deadline mandatory, fd optional | libusb / sd-event / nghttp2 contrast | medium (derived) |
| Feed as a refresh-or-revoke dead-man lease | **AUTOSAR WdgM** — the supervisor does not trigger the watchdog; it hands the driver a decrementing credit spent per trigger, and at zero the driver stops triggering | **high** |
| Per-device deadlines enforced below the app | SocketCAN `CAN_BCM` `RX_SETUP` `ival1` → kernel `RX_TIMEOUT` | **high** (relevant to phase 7) |
| Non-blocking alone ⇒ isolation | — | **refuted 0–3** (§3.1) |

**Do not overstate the safety-standards support.** AUTOSAR WdgM corroborates the *lease shape*
strongly. Its sentence about blocking destroying software-level supervision is an
**informative Note, not a normative "shall"**, and the claim about WdgM's global escalation
semantics was voted 1–2, i.e. not established — so do not cite WdgM as authority for cbctl's
per-device quarantine. Equally, nghttp2/libcurl/libusb/OpenSSL/sd-event are network and host
libraries, not safety-certified components: they all allocate dynamically, and libcurl and
OpenSSL both retain blocking paths. They are strong evidence for **API shape and loop
ownership** and weak-to-irrelevant evidence for anything a certification body would accept.

## 9. What is NOT established — do not infer it from this document

Stated plainly so nobody mistakes silence for coverage:

1. **How EVerest and production OCPP 1.6/2.0.1 stacks isolate per-connector faults.** This is
   the closest domain prior art to cbctl and the research produced **zero** verified claims
   about it. Whether they isolate by process, by thread, or by a bounded quantum in one loop
   is *unknown here*. Nothing in this design should be defended by appeal to them.
2. **MISRA C / IEC 61508 / ISO 26262 positions** on dynamic allocation and unbounded blocking
   in periodic control loops. No surviving claims. cbctl's no-allocation rule stands on its own
   engineering merits (§2 of doc 02), not on a rule number.
3. **Jitter and WCET budgeting.** No verified guidance at all. The 400 ms cadence against
   1500 ms is a 3.75× margin chosen by argument, and has **not** been validated by a
   worst-case execution-time analysis of the tick.
4. **The anti-flap re-admission protocol** for a quarantined link. Capped exponential backoff
   is a reasonable default; no prior art was verified for the correct re-entry path.

Items 1 and 3 are the two worth closing before a fleet deployment.

## 10. Build order

| phase | contents | gate |
|---|---|---|
| 1 | vendor cbplc core; `units.h`, `enums.h`; loopback port | wire round-trip vs real firmware bytes |
| 2 | `link.c` — link state machine, tick, reconciliation | a board that never answers never blocks the tick |
| 3 | `feed.c`, `lease.c` — repetition, staleness, leases | feed held < 1500 ms across a 1 h loopback soak |
| 4 | `status.c`, `event.c` — typed snapshot, synthesised events | live flags vs cumulative counters cannot be confused |
| 5 | `fleet.c` — N links, rotation, quarantine | **F1–F6 each with a wedged-link test** |
| 6 | `posix_serial.c`, `cbctl_pollfds` | worked Pi example, 2 PLCs, one killed mid-charge |
| 7 | CAN transport | needs `CBPLC_MIN_TRANSPORT_MTU` 32 → 8, a firmware-side decision |

Phase 5's gate is the one that matters for this document, and it is stated as a falsifiable
test rather than a review: **with two links live and charging, wedge one — physically unplug
it — and assert the other's feed age never exceeds 1500 ms and its session survives.**

A design claim about isolation that has not been tested by breaking something is an opinion.
