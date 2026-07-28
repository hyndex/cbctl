# The DC charging sequence, and where cbctl/cbplc diverges from the reference

The working stack is `plc_firmware` + its Python harness: it delivered a real 191.2 Wh charge at
388.5 V / 10 A on this bench. This document is the sequence it actually implements, phase by phase,
next to what cbctl + cbplc do — so a divergence is a row in a table rather than a week of bench
time.

Every claim here was read in source or measured on hardware. Where something is unverified it says
so; a table like this is only worth having if its "verified" column can be trusted.

---

## 1. The sequence, phase by phase

| # | Phase | Vehicle waits for | Gun contactor | Power modules |
|---|---|---|---|---|
| 1 | SLAC | match | **open** | off |
| 2 | SDP / SessionSetup | SECC answers | **open** | off |
| 3 | ServiceDiscovery → Authorization | `EVSEProcessing=Finished` | **open** | off |
| 4 | ChargeParameterDiscovery | EVSE envelope + `power_ready` | **open** | off |
| 5 | CableCheck | `EVSEIsolationStatus=Valid`, `Processing=Finished` | **CLOSES here** | off |
| 6 | PreCharge | `EVSEStatusCode=EVSE_Ready` **and** present voltage within ±20 V of its target | closed | **enabled, ramping to target** |
| 7 | PowerDelivery(Start) | EVSE ready + converged | closed | regulating |
| 8 | CurrentDemand | current follows its request | closed | regulating |
| 9 | PowerDelivery(Stop) | — | **opens, but only after current decays** | demand cleared first |
| 10 | WeldingDetection | decaying voltage evidence | open | off |

### The two orderings that matter

**Close onto a dead bus (phase 5).** There is no precharge resistor and no soft-close in this
design, so *precharge is the inrush mitigation*: the contactor closes at ~0 V ΔV and the modules
then walk the bus up to the pack voltage. The vehicle closes **its own** contactor afterwards, once
the voltage matches. Closing at the pack voltage instead would be closing onto a live bus.

**Open only after the current is gone (phase 9).** `plc_firmware` is explicit — opening the DC
contactor before module demand decays "breaks it under load on hardware with NO weld sensor". The
order is: clear demand → poll until current ≤ 2 A (or a bounded blind hold) → open → *then* check
for weld from the decaying voltage. This hardware has no weld sense line and no contactor feedback;
the PLC echoes the commanded relay state, so weld can only be inferred.

### PreCharge has no "finished" flag

`din_PreChargeResType` and its ISO-2 twin carry exactly three fields: `ResponseCode`,
`DC_EVSEStatus`, `EVSEPresentVoltage`. **There is no `EVSEProcessing`.** Completion is signalled by
`EVSEStatusCode == EVSE_Ready` plus the present voltage landing in the EV's acceptance window.

*The voltage is the signal.* Anything that reports a voltage the modules are not actually producing
— an echo of the vehicle's own request, a clamp, a zero — either stalls PreCharge forever or lies
to the car about the state of a high-voltage bus.

---

## 2. cbplc's energy gate is ordered and self-reporting

`cbplc_limits_refusal()` walks one chain and names the first thing that fails:

```
no_limits → no_feedback → limits_stale → feedback_stale → power_not_ready
          → empty_envelope → isolation_* → contactor_open → ""
```

It is published as `limits_refusal=` on the STATUS line. **Read it before theorising.** Bringing
this stack up meant walking that chain one gate at a time — `limits_stale`, then
`power_not_ready`, then `isolation_unknown`, then `contactor_open` — and every step was already
named on the wire before it was diagnosed.

---

## 3. Where cbctl/cbplc diverged, and why

Each of these was found on hardware. Each is fixed; the "was" column is kept because the shape of a
mistake is more reusable than the fix.

| Area | Reference behaviour | What we did | Consequence |
|---|---|---|---|
| Authorisation idle state | never sends a terminal refusal | opened with `AUTH_DENY`, re-asserted at 1 Hz | cbplc answers the car FAILED and latches 30 s; `auth_ok=3` vs `auth_no=104` |
| Gun close trigger | **level**, re-asserted while in the energy phase | one-shot on a `SESSION_STARTED` edge | a missed edge meant the contactor never closed at all |
| Isolation | controller decision (proxy, no IMD) | derived from module telemetry freshness | circular: isolation needs modules, modules need the EV target, the target needs CableCheck |
| `power_ready` | property of the plant | same telemetry gate | EV looped on ChargeParameterDiscovery forever |
| Contactor in FEEDBACK | reported | never set (`memset` only) | `contactor_open` forever, energy refused |
| Bad telemetry | withhold | clamped **up** to the 1000 V ceiling | reported a 1000 V bus to a real vehicle |
| Feed cadence | periodic | dirty on any change; mV jitter ⇒ every tick | 618 FEEDBACK vs 7 AUTHORIZE in 23 s; auth lease starved and lapsed |
| Refused close | back off | retried at once, forever | 2509 refusals; the retries starved the fix |
| Demand liveness | EV's own phase (pushed as an event) | `secc_active` flag → target-changed → `dc_ans` | the flag reads 0 mid-PreCharge; we commanded modules **OFF** all session |

### The one that cost the most

For a whole afternoon the modules "would not start". The wire said otherwise: the only frames we
ever sent were `ALL_SET` with `byte1=0x01` — **"Shut Down (DC, not PFC)"** — and an all-zero
payload. The modules were doing exactly what they were told.

The lesson is not about ENR modules. It is that *absence of output was diagnosed for hours without
once looking at what was being transmitted*. A passive `candump` during a session costs nothing and
would have shown it in seconds.

---

## 4. Observability gap against the reference

`plc_firmware` **pushes** `[SERCTRL] EVT HLC` on every substate change, so a controller sees the
phase trail as it happens. cbplc has no event path at all — `CBPLC_T_EVENT` exists in the enum and
is never emitted — so the trail is poll-only via `substate=` / `furthest=`, and the `dcreq:` log is
rate-limited to answer-shape changes plus a 2 s heartbeat. **A vehicle re-asking the same message
is therefore invisible**, which is precisely the signature of a stalled phase.

Closing this is worth more than any single fix in the table above.

---

## 5. What is verified, and what is not

**Verified on hardware:** the full V2G path (SLAC → SDP → negotiation → `Energised` → PreCharge
answered with `iso=2 contactor=2` → vehicle-initiated `SessionStop`) using cbplc's own bench
harness, which plays the power supply. The gun contactor closes and is confirmed by board readback.
cbmodules drives these ENR-S0 modules correctly standalone: commanded 200 V → **200.0 V**; a
0→1000 V ramp in 50 V steps tracks within ~1 % to **1008.1 V**, no faults.

**Not verified:** a real charge. No session has yet delivered current, so the `dc_ans` liveness rule
in §3's last row is **untested against a vehicle**, and cbplc's stability *while charging* is
unproven — charging has not happened.

**A note on the 200 V floor:** `module_type_min_voltage_v(MaxwellEnr)` is 200 V and is applied
whenever commanded current exceeds the ramp deadband. A vehicle asking for a precharge target below
200 V cannot be served by these modules.
