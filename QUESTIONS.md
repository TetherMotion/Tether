# Design decisions — FastLoop / 4 kHz realtime work

Originally open questions accumulated during the jitter-reduction
implementation (2026-09).  All items were reviewed and decided; this file
now records **what was decided and where it lives**.  Items marked
*deferred* remain deliberate non-goals with the rationale recorded.

## API / semantics

1. **`EntryHandle` fully replaced `PDOEntry::app_buffer`.**  Hard cut, no
   deprecation window.  `PDOEntry` now owns inline
   `storage[kMaxPDOSize]`; `add_rxpdo`/`add_txpdo` (and the broadcast
   variants) take `size` only — no caller buffer.  Access is via
   `entryData()`/`entryDataMut()`/`entryDataAs<T>()`, or epoch-checked
   `entryHandle()`/`resolve()`/`resolveMut()` for references that must
   survive remapping.  `epoch()` bumps on `clear()` and
   `remove_entries_for_slave()`.  All in-tree callers (Slave PDO table,
   DS402/CiA402, Beckhoff terminals, RP20, Axia80, ESC211, examples,
   klipper glue) were migrated.  Implementation: `PDOManager.hpp`,
   `PDOManager.cpp`, `Slave.cpp`.

2. **User-supplied collect phase.**  `CyclicLoopConfig::collect_phase`
   (`TaskPhase`) overrides the phase the split collect task is
   registered on: `Atomic` ignores it; `Split`/`SplitLate` default to
   PostExchange/Diagnostics as before, and an out-of-range value is
   clamped with a warning.  Implemented in
   `Master::setupCyclicDatapath` / `Master.cpp`.

3. **`waitCyclicSlotMask` added to `IPDOTransport`.**  Single-wake
   multi-slot wait: takes the slot bitmask + per-slot tokens + one
   shared absolute deadline, returns the arrived-bitmask so the caller
   can attribute per-slot timeouts.  `Master::waitCyclicSlotMask` is the
   real implementation (one ppoll over eventfd ∪ wire fd, seq_cst
   compares); the `IPDOTransport` default is a sequential per-slot
   fallback for transports without an optimized mask wait.
   `cyclicCollect` uses it for multi-slice frames.

4. **shm header exports an entry table.**  `ShmImageLayout` =
   `[ShmImageHeader][outputs][inputs][ShmEntryDesc table]`.
   `exportEntryTable()` publishes `slave_index`/`pdo_index`/`direction`/
   `offset`/`size` per entry after the handshake; `entry_count` is the
   atomic release-store publication word.  Attachers map the full
   segment (size via `fstat`) and get `entryOffset()`/`entryHandle()`
   for free.  `kShmVersion = 2`.

5. **Optional seqlock on shm regions.**  `ShmImageHeader` carries
   seqlock words for the output and input regions
   (`kShmFlagSeqlock`); `shmWriteBegin()`/`shmWriteEnd()` wrap external
   writer updates, `shmReadBegin()`/`shmReadEnd()` give readers a
   bounded-retry consistent snapshot.  Opt-in via the header flag —
   single-writer/atomic-field users pay nothing.

6. **WKC is derived, not learned.**  `deriveExpectedWkc()` computes the
   expected WKC per slice from the mapped slave/direction sets
   (RxPDO +1, TxPDO +2 per intersecting entry).  Slices whose set cannot
   be derived keep the `0xFFFF` sentinel and *learn* the first
   successful response as before; `strict_wkc` then enforces the
   learned/derived value.

7. **`resetCyclicWkc()` is the explicit invalidation API.**  Derivation
   re-runs on `configure()`/`deinit()`; a hard recovery or re-OP cycle
   can call `resetCyclicWkc()` to drop learned sentinel values without a
   full rebuild.

## Threading / platform

8. **Optional cgroup2 cpuset partition (root, opt-in).**
   `CpuIsolationConfig::create_cpuset` → `CpuIsolation` creates one
   `/sys/fs/cgroup/tether_rt` cpuset covering *all* currently claimed
   CPUs (subsequent claims extend the same partition — no competing
   per-CPU groups), moves the cyclic threads in, and tears the
   partition down on `releaseAll()`.  Failure degrades to
   affinity-only with a logged warning — claims are never denied for
   isolation failure.

9. **Optional IRQ steering (root, opt-in).**
   `CpuIsolationConfig::steer_irqs` → `/proc/irq/*/smp_affinity_list`
   masks are rewritten to exclude claimed CPUs; original masks are
   saved and restored on `releaseAll()`.  Per-IRQ failures are logged
   and skipped — never fatal.

10. **SCHED_DEADLINE auto-measure.**  `SchedClass::Deadline` with
    `dl_runtime_ns == 0` and `dl_auto_measure` (default on) starts on
    SCHED_FIFO, samples per-cycle work for `dl_measure_cycles` (512),
    takes P99, applies `dl_margin_pct` (default 100 → 2× headroom),
    clamps below the deadline, and switches to SCHED_DEADLINE.  Stats
    expose `deadline_active` and the applied `dl_runtime_ns`.

11. **Spin knobs split.**  `rx_spin_ns` (channel `rxPending` busy-poll)
    and `slot_spin_ns` (deposit-seq spin inside `waitCyclicSlotView`)
    are independent `CyclicLoopConfig` fields.

12. **Windows wait = WaitOnAddress (spec, deferred).**  The
    `futexWaitOn`/`futexWakeAllOn` pair in `ProcessImage.cpp` is the
    seam; the Windows port maps them to `WaitOnAddress`/
    `WakeByAddressAll` (available since Windows 8, works on any
    process-local or mapped address — identical semantics to
    `FUTEX_WAIT_PRIVATE`/`FUTEX_WAKE_PRIVATE`).  Spec only — no Windows
    channel exists yet; the `#else` spin+yield fallback remains the
    portable baseline.

13. **Publication is seq_cst.**  The deposit path's waiter check and
    the slot `seq` store use `seq_cst`, closing the "waiter registers
    just after the check" window: a deposit either sees the waiter or
    the waiter's re-check sees the bumped seq.  Bounded either way —
    the stronger ordering just removes the spin-window reliance.

## Wire / transport

14. **`kPiggybackIdx = 0xFE` is reserved** for future
    mailbox-in-cyclic piggyback datagrams (`Types.hpp`) — the same
    reserved value as `kFireAndForgetIdx`.  The BPF accepts first-idx
    `0xF8–0xFD`; a piggybacked `0xFE` datagram rides *inside* a cyclic
    frame and is forwarded to the regular parser by
    `dispatchChannelFrame`.  Invariant kept: LRW stays the first
    datagram so the first-idx demux stays exact.

15. **VLAN-aware BPF + dispatch.**  Socket A now binds `ETH_P_ALL` and
    the extended program demuxes on `frame[17]` (untagged) or
    `frame[21]` (0x8100-tagged).  `dispatchChannelFrame` handles the
    +4 tag offset for both cyclic and routed paths, and
    `parseEtherCATFrame` skips tags in mixed frames.  VLAN cyclic
    traffic gets the fast path instead of falling to VLANRouter.

16. **Sizing rule documented (§10.2).**  At 100 Mbit/s one max-size
    frame ≈ 120 µs on the wire; at 4 kHz (250 µs) a process image is
    effectively capped at ~2 frames.  This is a physics limit, not an
    implementation limit — `configureProcessImage` already fails
    loudly past `kNumCyclicSlots` (6) slices.

17. **TPACKET_V3 RX prototype implemented** behind
    `CyclicChannelConfig::rx_tpacket_v3` (`createCyclicRingChannelForMemory`
    seam + test fixture).  V3's block model batches the retire better
    under burst load; the prototype proved the hold/release cookie
    (`block<<16|pkt`) works and surfaced one real bug (cursor must
    advance *after* the sweep).  V2 remains the default — V3 buys
    nothing at fixed 1600 B slots under normal load.

18. **`PACKET_IGNORE_OUTGOING` is probed at setup.**  The cyclic socket
    applies it when available and reports via channel stats; absence is
    logged (async socket then sees its own cyclic sends — the demux
    still routes correctly, socket B pays the wakeup).  Mirror-filter
    correctness is unaffected.

## Verification gaps

19. **Privileged CI job added.**  `.github/workflows/ci.yml` gained a
    `privileged-tests` job: prefers rootless `unshare -rn` (real
    CAP_NET_RAW/CAP_NET_ADMIN in a fresh netns — veth, AF_PACKET, cBPF,
    TPACKET all work), falls back to `sudo` when the runner disallows
    user namespaces.  `tether_ethercat_rt_privileged_tests` self-skips
    whatever the environment still lacks, so the job is green on any
    runner while exercising the real kernel path wherever possible.

20. **shm header handshake implemented.**  `ShmImageHeader` carries
    `ready` + `checksum` over the fixed fields; the owner publishes
    `ready=1` (release) only after all fields are final, and clears it
    first in `releaseShm`.  `attachShared` validates magic + version +
    ready + checksum before trusting sizes, so a half-written header
    can never attach with wrong geometry.  The packed
    `send_waiters = {epoch:8 | count:24}` word makes crashed-waiter
    reclaim race-free (CAS on the packed word).

21. **`PDOToString` fixed.**  Signed fields are sign-extended for
    1/2/4/8-byte sizes and printed as `value (signed)` —
    `0xFFFF (-1)` restored; hexdump fallback unchanged.

22. **No-fd fallback is configurable.**
    `CyclicLoopConfig::slot_wait_fallback = SlotWaitFallback::Yield
    (default) | Spin` — the portable wait path `sched_yield`s between
    seq re-checks by default; `Spin` restores the pure busy loop for
    the rare case where yield latency exceeds the cycle budget on
    non-Linux/MCU builds.

## Async send-on-change loop (FastLoop)

23. **DC got its own RT thread under async.**
    `AsyncLoopConfig::enable_dc_synchronization` + `dc_interval_us`:
    when set, a dedicated deadline-paced thread emits DC sync frames
    at its own period, sharing the cyclic datapath's CPU-claim and
    mlock wiring (`AsyncCyclicLoop` `dc_sync` callback).  Drives with
    SM watchdogs are covered independently of producer cadence;
    `max_idle_ns` keep-alives remain the watchdog fallback for non-DC
    slaves.

24. **Trigger→wire latency stat.**  `AsyncCyclicLoop` stamps every
    `triggerSend` (release, before the seq bump) and records
    `max_send_latency_ns`/`avg_send_latency_ns` in the loop stats —
    the trigger-to-TX-commit delay is now observable via
    `getAsyncLoopStats()`.

25. **Packed waiter epoch.**  `send_waiters` = `{epoch:8 | count:24}`;
    registration/reclaim is a CAS on the packed word, so a crashed
    waiter's count is reclaimed when the epoch rolls without
    corrupting the live count.  No wasted-wake leak.

26. **Per-producer trigger stats.**  `claimProducerSlot()` hands out
    one of `kMaxShmProducers` slots; `triggerSend(producer_id)`
    increments `producer_triggers[id]` in the shm header —
    per-source attribution is available through
    `producerTriggerCount(id)` for diagnostics while coalescing stays
    global-by-design.

27. **One-shot misuse warning.**  `triggerSend()` while
    `SendConsumer::Cyclic` owns the wire logs once
    ("async trigger while cyclic loop owns the wire") — the seq bump
    is still a harmless no-op, but the warning catches the API misuse.

## Remaining deferred items

* **Windows channel** (Q12): WaitOnAddress spec recorded; no
  implementation until the Npcap-based channel lands.
* **Image > 2 frames at 4 kHz** (Q16): physics, documented as a sizing
  rule — not an implementation task.
