# Provisioning: what this charger is, and what this board needs trimmed

## The problem

Three facts that belong to a *charger* rather than to *firmware* were compiled in, and one of
them was a real fleet bug:

- **`kEvseId[] = "IN*JPE*E000100010001"`** shipped in every binary, so **every charger we build
  announced the same identity to every vehicle.** Wrong for OCPP, wrong for billing, and
  invisible until two of our chargers stand on one site.
- **The SLAC receive-path correction** was a single 18 dB constant standing in for cable length,
  connector and coupling — properties that vary per unit. ISO 15118-3 expects this measured per
  device; Sevenstax's WhiteBeet manual puts it in one line: the transmission path *"must be
  configured once for each device."*
- **Which HLC protocols to accept** was not selectable, so answering "does this car fail on ISO
  but charge on DIN?" meant a rebuild and a reflash.

## Declared, not stored

cbplc has no NVS, no config file and no board identity, **and that is the design.** A stored
`plc_id` that can corrupt is how a dead board and a misconfigured board became
indistinguishable; MAC-as-identity exists to make that unrepresentable. Giving three fields a
config store would hand that whole class of bug a home again.

So `CBPLC_T_PROVISION` is shaped like `LIMITS` and `AUTHORIZE`:

```
controller declares  ->  board holds it in RAM while powered  ->  reset returns to defaults
                                                              ->  cbctl re-asserts
```

**The controller is the config store.** It has a filesystem and already knows all of this. Store
per-board values keyed by the board's **MAC** — never by port, for the reason
`docs/04-failure-modes.md` gives.

A board that reboots comes back at its compiled defaults, which are exactly the values that
shipped before this existed. Rolling this firmware onto a fleet with un-updated controllers
changes nothing.

## API

```c
cbctl_provision_evse_id(&link, "IN*JPE*E000100010777");   /* 7..37 chars, ISO evseIDType */
cbctl_provision_protocols(&link, CBCTL_PROTO_DIN70121);   /* interop triage */
cbctl_provision_attn_trim_db(&link, -6);                  /* per-board, ±30 dB */
cbctl_provision_feed_deadline_ms(&link, 0);               /* bench only; 0 = board default */
cbctl_request_restart(&link, &op);
bool ok = cbctl_provision_supported(&link);
```

Every setter is an intent — nothing blocks, `cbctl_tick()` puts it on the wire — and marks the
declaration dirty **only on an actual change**. Re-stating an unchanged configuration every pass
is free, which matters because that is exactly what a control loop does; making it re-transmit
is the defect the feed once had.

## Tool

```
make provision
./build/provision /dev/ttyACM1 --evse-id IN*JPE*E000100010777 --protocols din --trim -6
./build/provision /dev/ttyACM1 --show
./build/provision /dev/ttyACM1 --restart
```

It **reads back**. A setter that is not read back is one you believe rather than one you know,
so it waits for the board's ack and then prints what the *board* reports in its own STATUS
(`prov_id`, `prov_proto`, `prov_trim`, `prov_ok/prov_bad`). It exits non-zero if a declaration
was made and the board never confirmed it.

## Rules that earned their place

**A rejected frame changes absolutely nothing.** The codec validates every TLV in one pass
before writing any, so a frame whose third field is malformed cannot leave the first two in
force. A charger running half a declaration announces one identity and behaves as another.
Mutation-tested.

**A declaration is cleared only by the board's own ACK**, never on transmission. Believing it
because it left the host is how a charger ends up announcing an identity nobody applied.
Mutation-tested.

**A board reset re-asserts.** cbplc holds the declaration in RAM, so a reset silently returns
the EVSE ID to the fleet-wide fallback. A controller that did not re-assert would leave a
charger quietly announcing the wrong identity for the rest of its uptime — same class as the
stop latch. Mutation-tested: removing it fails three assertions.

**Unknown TLV tags are skipped, not refused.** A controller newer than the firmware may declare
a field this build has never heard of; refusing the whole frame would mean the fields it *does*
understand never take effect.

**A firmware that predates this NACKs with `CBPLC_E_BAD_TYPE`** — "a feature gap, not a dead
link". cbctl detects that, stops after `CBCTL_PROV_TRIES`, and keeps the intent so a reflash or
reset re-asserts it. `cbctl_provision_supported()` distinguishes *not declared* from *cannot
declare*.

**Restart is gated twice.** cbctl refuses it while *we* hold a relay closed — a self-cancelling
instruction, same class as power_ready-while-stopping. The board refuses it unless the connector
is genuinely idle: CP state A, no session, no SLAC match, every relay **proven open by readback**
rather than believed open. The ack goes out before the reboot, and the gate is **re-checked at
the moment of the reboot** — a vehicle can arrive during the grace window, and the restart is
abandoned rather than queued, because a reboot that fires at a moment nobody chose is worse than
none.

## The attenuation trim is a trim, not a control loop

The fixed 18 dB stays fixed. An adaptive correction that solves toward a target attenuation will
happily make a **distant vehicle look adjacent**, which is precisely the cross-gun mismatch SLAC
exists to prevent. The declared value is *additive*, bounded to ±30 dB by the codec, and nothing
inside the firmware ever writes it. Measure it once per board, store it against the MAC, declare
it at link-up.

## Watch the STATUS tail key

The board publishes `prov_ok prov_bad prov_id prov_proto prov_trim prov_timeout` at the **end**
of its STATUS line, so `prov_timeout` is now the truncation sentinel — `CBCTL_STATUS_TAIL_KEY`
here, `TAIL_KEY` in cbplc's `tools/probe.py`. **It moves whenever `build_status()` gains a
field.** A sentinel that is no longer last does not fail loudly; it silently stops detecting
truncation, which is the exact failure it exists to catch.

## Verified on hardware

Board `D0:CF:13:71:FF:54`, 2026-07-29:

| step | result |
|---|---|
| before | `evse_id=-  protocols=0x03  trim=0  0/0` — compiled defaults |
| declare | `evse_id=IN*JPE*E000100010777  protocols=0x01  trim=-6  1/0`, `provisioning: supported` |
| restart | accepted (connector idle), board rebooted |
| after reboot | `evse_id=-  protocols=0x03  trim=0  0/0` — **nothing persisted**, as designed |
| regression | `live_session` PASS, worst feed age 28 ms against the 1500 ms bound |

`trim` is read back from `cbplc_slac_attn_rx_trim()`, i.e. the value actually in force in the
SLAC FSM rather than the one in the struct — so `-6` proves it reached the front end.

**Not verified against a vehicle:** that a DIN-only mask makes a dual-protocol car fall back
rather than fail. The mask is confirmed in force on the board (`prov_proto=0x01`) and the jpv2g
selection logic is unit-tested, but the fall-back behaviour itself needs a car.
