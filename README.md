# cbctl

A non-blocking C11 library for a charger controller to talk to **cbplc** PLC boards
(ESP32-S3, one board per gun) over USB serial. It carries identity, the limits/measurement
feed, authorisation leases, relay dead-man leases, status observation, board diagnostics and
per-board provisioning — and nothing else. It is a communication library, **not a controller**:
no threads, no sleeps, no owned event loop, no charging policy.

```c
cbctl_serial_t ser;  cbctl_link_t link;
cbctl_serial_open(&ser, "/dev/serial/by-id/usb-Espressif_..._D0:CF:13:71:FF:54-if00", 115200u);
cbctl_open(&link, &ser.tx);
for (;;) {
    cbctl_tick(&link, monotonic_ms());                    /* the only thing touching the wire */
    /* declare intents (envelope / measurement / auth / relay), drain events ... */
    poll_on(cbctl_serial_fd(&ser), cbctl_next_deadline_ms(&link, monotonic_ms()));
}
```

Proven on hardware: real vehicles charged end-to-end on both guns of the bench charger; two
links serviced by one loop with per-link margins **bit-identical from 1 to 8 links** (worst
fleet pass 362 µs on a Pi 4B — two boards physical, 4 and 8 against the in-process loopback
running the real protocol code); a board killed mid-run is quarantined, its sibling never
notices, and an in-place reopen brings it back in the same process.

## Start here

**[docs/00-integration-guide.md](docs/00-integration-guide.md)** — the complete controller
developer walkthrough: hardware prerequisites, bring-up, the five intents, the loop contract,
the charging lifecycle, failure recovery, the latency model, do/don't, deployment checklist.

Then, by need:

| Doc | Covers |
|---|---|
| [01-api-reference](docs/01-api-reference.md) | every function, struct, constant |
| [02-design-and-safety-model](docs/02-design-and-safety-model.md) | the declarative model; dead-man philosophy |
| [03-concurrency-and-fleet](docs/03-concurrency-and-fleet.md) | the loop contract; N links; threads |
| [04-failure-modes](docs/04-failure-modes.md) | failure catalogue, measured; N>2 deployment |
| [05-dc-sequence-and-gaps](docs/05-dc-sequence-and-gaps.md) | the DC session over the wire |
| [06-diagnostics](docs/06-diagnostics.md) | board log, levels, V2G trace |
| [07-provisioning](docs/07-provisioning.md) | per-board identity, SLAC trim, protocol mask |

## Build and test

```sh
make test          # host: 7 suites (link, fleet, status, serial, soak, led, units)
make charge        # bench tool: full charging lifecycle against a real vehicle
make live-dual     # two boards, one loop, kill + in-place recovery, gated
make fleet-bench   # N=1/2/4/8 timing measurement (loopback, real protocol code)
make provision     # per-board identity/trim, with board readback
make boardlog      # drain the board's diagnostic ring; arm the V2G trace
```

The library proper is `src/` + `include/cbctl/` + the vendored `third_party/cbplc` protocol
core. `ports/posix_serial.c` is the POSIX transport; `ports/loopback.c` is an in-process fake
board running the real cbplc session code, which is what the tests and the bench drive.

## The three rules people learn the hard way

1. **Open by `/dev/serial/by-id/`, pin with `cbctl_expect_mac()`.** `ttyACM` order changes —
   across reboots and within a session.
2. **Never block on the tick thread.** The board opens relays at 1500 ms of feed silence; your
   loop's worst gap budget is ~350 ms and a synchronous meter read once cost 2.1 s.
3. **Recovery is yours.** The library quarantines a dead link; only the caller can close and
   reopen the port (the pattern is in the integration guide §9 and `tools/live_dual.c`).
