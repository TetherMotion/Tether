# Open design questions — FastLoop / 4 kHz realtime work

Accumulated during the jitter-reduction implementation (2026-09).  Each item
is a decision that was made pragmatically and deserves a deliberate revisit
before the API hardens, or a known limit that needs platform validation.

## API / semantics

1. **Should `EntryHandle` fully replace `PDOEntry::app_buffer`?**
   `entryHandle()` + `outputPtrRaw`/`inputPtrRaw` now provide epoch-checked
   access, but `app_buffer` raw pointers remain the buffered-path mechanism
   and are still handed out at registration.  A full eviction means
   remapping every caller (DS402, klipper glue, examples) and deciding what
   the buffered path gathers/scatters *from*.  Deprecation window vs. hard
   cut?

2. **`ExchangePlacement::SplitLate` collects at Diagnostics.**  Semantically
   odd (Diagnostics is meant for housekeeping) but it's the latest phase.
   Alternatives: a dedicated `Collect` phase before MotionControl, or a
   user-supplied phase index.  Is piggybacking Diagnostics acceptable, or
   should the phase list gain an explicit slot?

3. **Multi-slice collect is sequential `waitCyclicSlotView` per slot.**  With
   all slices in flight at once, a `waitCyclicSlotMask` (single wake for
   "all of mask set") would shave per-slot loop overhead.  Worth adding to
   `IPDOTransport`, or is sequential-with-shared-deadline enough?

4. **`ProcessImage::attachShared` exposes raw offsets only** — the client
   process has no PDO mapping, so `entryHandle()` is unavailable.  Should
   the shm header carry an exported entry table (name/index/offset/size)
   so external motion sources can discover fields instead of hardcoding
   offsets?  Versioned struct exchange, or leave it to the integrator?

5. **shm image forces `Direct` semantics.**  The external writer gets zero
   copies but also zero snapshot protection — a wide field written mid-send
   can tear on the wire.  Should shm mode get an optional seqlock region
   (writer wraps updates in begin/commit like DoubleBuffered), or is the
   documented single-writer/atomic-field contract enough?

6. **`strict_wkc` learns the first successful WKC as truth.**  If the first
   exchange runs while a slave is already degraded, the wrong value becomes
   the reference.  Learn at `buildAddressMap`/configure time from the known
   slave set instead (deterministic LRW WKC = 3×slaves for full R+W)?

7. **`expected_wkc` resets only on `deinit()`.**  Should a hard recovery /
   re-OP cycle invalidate it, or is restart-via-deinit always the path?

## Threading / platform

8. **`CpuIsolation` claims are advisory bookkeeping, not real isolation.**
   Without `isolcpus=` or a cpuset partition, a claimed CPU still runs other
   tasks.  Should we optionally *create* a cgroup2 cpuset partition when
   running as root (real isolation without a reboot), or is detecting +
   preferring `isolcpus` the right ceiling?

9. **CPU claims don't move IRQs.**  Even on a "claimed" CPU, NIC IRQs land
   wherever irqbalance puts them.  Should `CpuIsolation` optionally steer
   `/proc/irq/*/smp_affinity` away from claimed CPUs (needs root, touches
   system state — currently out of scope on purpose)?

10. **`SCHED_DEADLINE` runtime defaults to period/2.**  A CBS budget of 50%
    of the period is safe but arbitrary.  Should the exec measure actual
    per-cycle work and propose runtime = P99_work × margin?  Needs a few
    hundred cycles of measurement first.

11. **`rx_spin_ns` applies to both the channel ring and the slot seq.**
    One knob for two spin sites.  Split into `rx_spin_ns` (channel
    rxPending) vs `slot_spin_ns` (deposit seq), or is one budget simpler
    and sufficient?

12. **`waitInput` on non-Linux is a spin+yield loop.**  Acceptable for the
    ESP32 path (single-core, short windows) but burns CPU on Windows.
    Event or WaitOnAddress equivalent for the Windows channel when it
    lands?

13. **The eventfd write is gated on `cyclic_waiters_ > 0`, checked
    non-atomically against the deposit.**  Race window: a waiter could
    register just after the check and sleep through one publish.  The
    slot-seq re-check inside the wait loop covers it (bounded by spin +
    ppoll timeout) — prove it stays bounded under all interleavings, or
    make the deposit do a seq_cst compare?

## Wire / transport

14. **BPF demux keys on `frame[17]` — the first datagram's idx.**  Exact
    today because cyclic frames only carry `0xF8+` idxes.  If piggybacked
    async datagrams ever share cyclic frames (mailbox-in-cyclic), the
    first-idx rule must hold: keep LRW first, or give piggyback datagrams
    a dedicated reserved idx and forward them manually in `rxPoll`.
    Decide before the mailbox-in-cyclic idea is implemented.

15. **VLAN-tagged cyclic traffic has no fast path.**  The filter rejects
    `0x8100` EtherType → lands on async socket → VLANRouter.  If a
    deployment ever needs VLAN + cyclic acceleration, extend the program
    to check `frame[21]` on `0x8100`.  Realistic need?

16. **Multi-slice wire time is the hard ceiling at 4 kHz.**  At 100 Mbit/s
    one max-size frame ≈ 120 µs on the wire — two max slices already eat
    a 250 µs cycle.  Images > ~2 frames are effectively out at 4 kHz
    regardless of slicing.  Document as a sizing rule, or investigate
    whether real deployments ever exceed one frame?

17. **TPACKET_V2 chosen over V3** for both rings (per-frame slots give
    clean hold/release; V3's block packing buys nothing at these frame
    sizes).  Revisit if profiling ever shows slot-scan cost — V3's block
    retire model batches better under burst load.

18. **`PACKET_IGNORE_OUTGOING` drops own-TX echoes upstream of the BPF.**
    Verified assumption: the async socket never taps cyclic sends.  If a
    future platform lacks that sockopt, the async socket sees its own
    cyclic frames — the demux still routes them correctly, but socket B's
    poll thread pays the wakeup.  Confirm on any non-Linux port.

## Verification gaps

19. **Privileged integration coverage is still thin in CI.**  The veth /
    CAP_NET_RAW tests auto-skip without privilege.  Needs a privileged CI
    job (or rootless netns with AF_PACKET allowed) to run the real
    kernel-path tests on every commit — otherwise regressions in the ring
    backend and BPF attach only surface on real hardware.

20. **No fault-injection for the shm header.**  A crashed writer leaves
    `in_waiters > 0` forever (harmless — wake is just wasted) but a
    half-written header could attach a client with wrong sizes.  Add a
    header checksum / ready-flag handshake?

21. **`PDOToString.FormatsFieldsAndValues` fails pre-existing** (expects
    `"(-1)"` in output, gets none) — unrelated to FastLoop; fix or update
    the expectation.

22. **`waitCyclicSlotView` no-fd fallback is a pure spin** — bounded by
    deadline and cancellation, but on a loaded non-RT system it can burn
    a whole budget spinning.  Should it yield (`sched_yield`) between
    checks on the portable path?
