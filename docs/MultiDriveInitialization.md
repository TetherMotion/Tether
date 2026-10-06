# Multi-Drive Initialization

Bringing up several drives on one bus has a failure mode that does not
exist with a single slave: **the slaves you already configured starve
while you configure the next one.** This document explains why, and the
API pieces that prevent it — primarily `PDOKeepAlive`.

## Why slaves "time out" during init

Each slave's INIT→OP sequence — reset, mailbox setup, PRE_OP SDO writes,
PDO assignment, SAFE_OP and OP transitions — takes on the order of a
second per drive. Meanwhile the slaves that are already in SAFE_OP/OP
are subject to their **Sync Manager process-data watchdog**: as soon as
SM2/SM3 are activated, the slave expects its process-data region to be
written cyclically. If the master stops exchanging while it spends ~1 s
configuring the next slave, the earlier slaves' watchdogs trip:

- SM2/SM3 de-activate (the SM enable bit clears; on strict ESCs the AL
  status code reports a watchdog/sync error).
- Process data stops moving — statuswords freeze, and the drive can no
  longer be enabled. The typical signature is a statusword stuck in
  `SWITCH_ON_DISABLED` (0x40-family patterns) with PDO exchange
  resuming but never recovering the slave.

This is a per-slave hardware watchdog, not a Tether timeout — restarting
the cyclic exchange does not necessarily resurrect the SMs.

Tether mitigates the window *inside* one slave's configuration already:
SM2/SM3 registers are written with activation disabled, the FMMUs are
programmed against the (possibly stale) SM state, and only then are the
SMs enabled — the watchdog starts as late as possible. And
`CiA402Drive`'s SAFE_OP→OP tail enables PDO exchange *before* requesting
OP, because the PDI watchdog expects process data from SAFE_OP onward.
What the master cannot do on its own is feed the *other* slaves while
your code runs the next slave's slow configuration sequence — that part
is on the application, and `PDOKeepAlive` exists for exactly that gap.

## The keep-alive pattern

```cpp
auto& master = /* EtherCAT::Master or ds402.ethercatMaster() */;

// Start BEFORE the first slave's OP transition — the worker exchanges
// each slave's logical window as soon as its PDO config assigns one.
auto keep_alive = master.pdo().startKeepAlive(
    std::chrono::milliseconds{1});

for (uint16_t idx : slave_indices) {
    // 1) reset -> mailbox -> PRE_OP
    // 2) ALL SDO writes here — several drive firmwares stop servicing
    //    the CoE mailbox once the slave leaves PRE_OP
    // 3) PDO assignment + SAFE_OP/OP transition
    //    (entries registered, finalizeMapping, FMMU + window assigned —
    //     the slave joins the keep-alive image from this point on)
    // 4) enable the drive (CiA 402 controlword sequence over PDOs)
    //    and run any post-OP setup (homing, params)
}

// Stop before taking over with your own polled/cyclic exchange — the
// keep-alive worker and a foreground exchange race on PDOManager
// split state.
keep_alive->stop();

// ... run your normal cyclic exchange covering all slaves ...
```

Ordering notes that matter in practice:

- **Start the keep-alive before the loop.** A slave gets a logical
  window at the end of its PDO configuration; the keep-alive skips
  slaves that have none yet and picks each up automatically as its
  window is assigned. There is no registration step.
- **Finish each drive before starting the next.** Run the full
  enable sequence (and ideally post-OP setup like homing) on slave *k*
  before configuring *k+1*. An enabled drive rides out the next slave's
  ~1 s INIT→OP disruption far better than one sitting idle-but-in-OP.
- **Stop it before your own exchange.** The keep-alive is a second pump
  on the same `PDOManager`; interleaving it with `exchangeAll()`,
  `exchangeLRWSlice()`, or a cyclic/queue loop corrupts the split
  send/receive state. `stop()` is idempotent, and the destructor stops
  the thread — keep the `unique_ptr` alive exactly for the init phase.
- **Do all mailbox/SDO work in PRE_OP.** Several drive firmwares stop
  answering CoE requests once the slave enters SAFE_OP (or once PDO
  exchange is enabled). Structure per-slave init so every SDO write —
  operating mode, homing method, limits — happens before the PDO
  assignment/SAFE_OP step, or it will time out.
- **Watchdog tuning (optional).** If your per-slave configuration is
  unusually long, widen the margin with
  `master.configureWatchdogs(SlaveAddress(idx), pdi_timeout_100us,
  pdata_timeout_100us)` (timeouts in 100 µs units;
  `disableWatchdogs()` writes zero). Do this per slave before its SM
  enable. A 1 ms keep-alive period comfortably feeds the common
  ~100 ms process-data watchdogs.

## `PDOKeepAlive` reference

Obtained from `PDOManager::startKeepAlive(period)` — an RAII object that
runs a worker thread exchanging **one LRW datagram per configured slave
window** every `period` (default 1 ms), via
`exchangeConfiguredSlaveWindows()`:

- For each slave with an assigned logical window (see
  `LogicalAddressManager::getSlaveLogicalWindow()`), it emits one LRW
  slice covering exactly that window — `[RxPDO][TxPDO]` bytes of that
  slave. Slaves still being initialized have no window and are skipped.
- Because each datagram covers one window, the WKC stays predictable per
  slave and a not-yet-joined slave never disturbs the others.
- Falls back to a full `exchangeAll()` when no logical address manager
  is wired up (standalone `PDOManager` use without `Master`).
- Successful windows bump the same per-slave `pdo_request_count` /
  `pdo_reply_count` counters the OP-transition liveness check reads.

```cpp
{
    auto keep_alive = master.pdo().startKeepAlive();   // 1 ms default
    // ... slow per-slave configuration happens here ...
    keep_alive->stop();   // or just let it destruct
}
```

Rules:

1. **One pump at a time.** Stop it before `exchangeAll()` /
   `exchangeLRWSlice()` / `exchangeConfiguredSlaveWindows()` are called
   from another thread, and before any cyclic, async, or queue-mode loop
   starts. Never run two keep-alives.
2. **It only feeds configured slaves.** The very first slave gets no
   keep-alive coverage during its *own* configuration — coverage starts
   when `finalizeMapping()` + `buildAddressMap()` assign its window
   (handled inside the OP-transition path). Keep individual slave
   configuration steps snappy for the same reason.
3. **It is not a substitute for the cyclic loop.** Once all slaves are
   up, stop it and run the real exchange — the keep-alive is a
   bring-up/transitional tool, not the production data path.

## If a cyclic loop is already running

When the DC/cyclic loop already owns the wire exchange (e.g. you are
adding or recovering a slave at runtime), do **not** start a keep-alive —
the running loop already feeds every configured window. For mid-loop
mapping changes use `master.suspendCyclicExchange(timeout_us)` /
`resumeCyclicExchange()` to quiesce the exchange while re-registering
entries (`SlaveSupervisor` does this internally during automatic
recovery). Suspension is not reentrant and does not survive a loop
restart.

## Failure signatures quick reference

| Symptom | Likely cause |
|---------|--------------|
| Drive stuck in `SWITCH_ON_DISABLED` after configuring a later slave | SM watchdog tripped — keep-alive missing or started too late |
| First drive dies exactly when the second reaches OP | Same — or a foreground `exchangeAll()` raced the keep-alive worker |
| WKC = 0 on all exchanges | Wrong configured station address on the entries, or `finalizeMapping()` never ran for the slave |
| SDO calls time out after SAFE_OP/OP | Firmware stops servicing the mailbox — move those writes to PRE_OP |
| `exchangeAll()` errors only while a slave is being added | Concurrent pump — suspend the cyclic loop or stop the keep-alive instead of exchanging from two threads |
| OP transition "no PDO traffic" check fails | Entries registered but never exchanged — keep-alive/cyclic loop not running during the transition window |

## See also

- [PDOAddressSpace](PDOAddressSpace.md) — logical image layout, window
  assignment, exchange granularity
- [PDOModes](PDOModes.md) — Direct/Queue/Callback exchange modes
- `include/tether/ethercat/PDOManager.hpp` — `PDOKeepAlive`,
  `startKeepAlive()`, `exchangeConfiguredSlaveWindows()`
- `include/tether/ethercat/Master.hpp` — `suspendCyclicExchange()`,
  `configureWatchdogs()`
- `tests/ethercat/test_process_image.cpp` — keep-alive/window exchange tests
