# Reducing Cyclic Jitter — a practical guide

This is the operator's companion to
[CyclicRealtimeTransport](CyclicRealtimeTransport.md).  That document explains
*how* the cyclic datapath is built; this one tells you **what to actually do**
when `max_jitter_us` or `missed_deadlines` are too high.

The loop budget is absolute: at 1 kHz you have 1000 µs per cycle, at 4 kHz
250 µs.  Jitter comes from exactly three places — **waking up late**
(scheduler/kernel), **working too long** (your code inside the cycle), and
**waiting on the wire too long** (NIC/driver/network).  The steps below are
ordered cheapest-first and map onto those three sources.  Do them in order;
each step only pays off if the earlier ones hold.

Every mitigation in Tether is **best-effort and loud**: a failed capability
probe, mlock, SCHED_FIFO or CPU claim logs a warning and the loop runs on
degraded.  The first debugging instrument is therefore *your own startup
log* — read it before tuning anything else.

---

## Step 0 — Measure before you tune

You cannot tell "wakes up late" from "wire is slow" without numbers.  Two
tools give them to you: the built-in statistics, and standalone benchmarks
that don't involve your application at all.

### Standalone measurement (no slaves needed)

```bash
cmake --build build --target realtime_diagnostics -j8
sudo ./build/bin/realtime_diagnostics -i eth0
```

This sweeps loop frequency × SCHED_FIFO priority × CPU affinity and prints
per-row cycle jitter (mean/max/stddev), `sendto()` latency and CPU% — the
OS/NIC floor your application can never beat.  Add `--slave N` for real
round-trip TX→RX timing against a real slave at position N.

For an application-independent baseline of *pure scheduler* jitter, use
`cyclictest` (rt-tests package):

```bash
# 250 µs period on CPU 3, FIFO 80, ~25 s — approximates a 4 kHz cycle
sudo cyclictest -m -p80 -a 3 -t1 -i 250 -l 100000
```

If cyclictest already shows excursions >> 50 µs, no amount of application
tuning will fix them — skip to Steps 3–4 (kernel, isolation, NIC).

### The loop's own counters

While `startCyclicLoop` runs, snapshot three views:

```cpp
auto es = master.getCyclicLoopStats();   // CyclicExecutive::Stats
auto h  = master.cyclicHealth();         // protocol/wire health
auto* ch = master.cyclicChannel();       // ICyclicChannel* (may be null)
```

| Field | Measures | Bad when |
|---|---|---|
| `es.jitter.max_jitter_us`, `avg_jitter_us` | wake-time deviation from the nominal period | ≫ a few µs on isolated cores, ≫ 50 µs on shared ones |
| `es.missed_deadlines` | cycles that started *after* their deadline | > 0 in steady state |
| `es.max_cycle_work_us` | longest single-cycle work burst | anywhere near `cycle_period_us` |
| `es.max_wake_overshoot_ns`, `avg_wake_overshoot_ns` | how late each tick's wake arrived vs its deadline — the sleep path's own jitter, separate from work and wire | ≫ a few µs with HybridSpin on an isolated core; tens-of-µs+ with Nanosleep under contention — that's expected, not a bug |
| `es.jitter.warning_count` / `critical_count` | threshold crossings (`JitterConfig`: warn = period/2, crit = 2×period) | any persistent growth |
| `h.rtt_us_min/avg/max` | wire round-trip (send → kernel RX stamp) | `rtt_us_max` ≫ `rtt_us_min`, or avg near `rx_budget_ns` |
| `h.wire_loss` | datagrams not back before the cycle deadline | > 0 |
| `h.stale_responses` | replies that arrived *late* (after deadline) | rising — splits "late" from "lost" |
| `h.rx_bank_drops` / `h.kernel_rx_drops` | local RX starvation (socket bank held / kernel tp_drops) | > 0 |
| `h.slices_stuck_stale` | slices whose RTT exceeds a whole period | > 0 |
| `ch->txDeferred()` | TX ring kicks deferred (frame left late) | > 0 at cyclic rates |
| `ch->backendName()` / `zeroCopy()` | which datapath you actually got | `nullptr` channel or `SocketIO` when you expected `PacketRing` |

`CyclicHealth::describe()` formats the whole thing for logs.
`master.cyclicChannel()` also confirms which backend you ended up on —
`Auto` mode falls back ring → socket → software and *logs it*, but checking
`backendName()` is how you know for sure.

### Read the kernel before the loop

```cpp
auto info = Tether::Platform::detectRealtimeKernel();
// info.is_realtime        → PREEMPT_RT
// info.is_low_latency     → full preempt or RT
// info.active_preempt_mode→ "full"/"lazy"/"voluntary"/"none"
```

or hard-fail at startup with
`ensureRealtimeKernelOrExit(RealtimeRequirement::HardRealtime)` if your
deployment demands RT.  Everything the cyclic thread does is bounded by what
the kernel underneath it can preempt — check this first so "the kernel can't
do it" doesn't masquerade as an application bug.

---

## Step 1 — Get the privileges right

Almost every mitigation needs a capability.  A missing one never fails the
loop — it logs a warning and silently *keeps* the jitter you were trying to
remove.

| Capability | Unlocks |
|---|---|
| `CAP_NET_RAW` | AF_PACKET sockets — the whole cyclic channel (ring + socket backends), BPF demux |
| `CAP_SYS_NICE` | `SCHED_FIFO`, CPU affinity pinning |
| `CAP_IPC_LOCK` (or `RLIMIT_MEMLOCK`) | `mlockall`/`mlock` — page-fault elimination |
| `CAP_NET_ADMIN` | disabling EEE (`nicDisableEee`), veth, some ethtool writes |
| root (effectively `CAP_SYS_ADMIN`) | `cpu_isolation.create_cpuset` (exclusive cgroup **v1** cpuset) and `steer_irqs` (/proc/irq rewriting) |

Ways to grant them, in order of preference for production:

```bash
# 1. systemd unit — the clean deployment answer
[Service]
AmbientCapabilities=CAP_NET_RAW CAP_SYS_NICE CAP_IPC_LOCK CAP_NET_ADMIN
CapabilityBoundingSet=CAP_NET_RAW CAP_SYS_NICE CAP_IPC_LOCK CAP_NET_ADMIN
LimitMEMLOCK=infinity
ExecStart=/usr/local/bin/your_app

# 2. File capabilities — no root shell needed
sudo setcap 'cap_net_raw,cap_sys_nice,cap_ipc_lock,cap_net_admin=+ep' ./your_app

# 3. runec — dev convenience (repo's setuid helper, grants
#    NET_RAW+NET_ADMIN+SYS_NICE keeping your uid)
runec ./your_app

# 4. sudo — simplest, widest blast radius
sudo ./your_app
```

`create_cpuset`/`steer_irqs` need real root — under `runec`/`setcap` they
degrade to affinity-only claims with a warning, which is fine (still better
than nothing).

---

## Step 2 — Turn on the application-side knobs

No OS changes required; this is pure `CyclicLoopConfig`.  Start from the
preset, then push further:

```cpp
auto cfg = EtherCAT::CyclicLoopConfig::lowLatency(1000);  // Split + CPU isolation
// lowLatency() deliberately stays conservative — for a tuned deployment:

cfg.wire_mode  = CyclicWireMode::Auto;        // ring → socket → software
cfg.image_mode = ImageMode::DoubleBuffered;   // see trade-offs below
cfg.exchange_placement = ExchangePlacement::Split;

// Wake path — the defaults ARE the shared-CPU profile: Nanosleep +
// sched_class=Auto (Deadline on shared CPUs).  Busy-waiting only pays
// off on a CPU that is genuinely yours:
//   cfg.exec.sleep_mode     = CyclicExecutive::SleepMode::HybridSpin;
//   cfg.exec.spin_window_us = 50;         // ISOLATED CPU ONLY — see below
cfg.exec.dl_auto_measure = true;              // CBS runtime = P99_work × 2
cfg.exec.priority        = 80;                // used by the FIFO fallback

// CPU claiming (runtime isolation without reboot)
cfg.cpu_isolation.enabled      = true;
cfg.cpu_isolation.cyclic_cpu   = -1;          // auto: prefer isolated, skip CPU0
cfg.cpu_isolation.create_cpuset = true;       // root: exclusive cgroup v1 cpuset
cfg.cpu_isolation.steer_irqs    = true;       // root: move IRQs off the claim

// RX latency — busy-poll inside the deadline instead of sleeping.
// ISOLATED CPU ONLY: every spin ns is charged against the
// SCHED_DEADLINE runtime Auto picks on shared CPUs.
//   cfg.rx_spin_ns   = 30'000;   // ring memory-poll
//   cfg.slot_spin_ns = 10'000;   // deposit-seq spin before ppoll

// Memory — defaults already lock image/slots and prefault the stack
cfg.memory_lock.lock_all_process = true;      // optional: whole-process mlockall

master.startCyclicLoop(cfg);
```

On a shared CPU (`isolcpus=` absent, cpuset claim failed), every
busy-wait knob should stay at its default **0/Nanosleep**: a spinning RT
thread can itself be preempted, so it burns CPU for protection that
evaporates exactly when load appears — and under `SCHED_DEADLINE` (what
Auto resolves to there) each spin ns is charged against the thread's own
CBS runtime.  The startup log tells you which side Auto picked; the
exec also warns when Deadline and a nonzero spin budget collide.

The knobs, what they cost, and when to use them:

| Knob | Effect | Cost / caveat |
|---|---|---|
| `wire_mode = Auto` | kernel PACKET_MMAP ring = zero-copy RX, kernel timestamps | needs `CAP_NET_RAW`; verify `backendName()` |
| `image_mode` | Buffered → DoubleBuffered → Triple → Direct/Rotating: fewer copies, stronger writer contract | Rotating = zero copies but *full-refresh every cycle* and needs a channel; Direct can tear unaligned >4 B fields; DoubleBuffered is the safe default for partial writers |
| `exchange_placement = Split` | send in Exchange phase, collect in PostExchange — wire RTT overlaps your in-between phases | inputs are one phase older for in-loop readers; use `Atomic` if your motion callback needs *this* cycle's inputs |
| `SplitLate` | collect in Diagnostics — maximum overlap | only for external motion sources; in-loop readers always see previous-cycle inputs |
| `sleep_mode = HybridSpin` | sleep to `spin_window_us` before the tick, then busy-poll | **isolated CPUs only**: on a contended CPU the spin itself gets preempted (pays the cost, loses the benefit); under SCHED_DEADLINE the tail is folded into the CBS budget — the exec warns |
| `sched_class = Auto` (default) | FIFO when the cyclic CPU is hard-isolated (`isolcpus=`, an isolated cgroup2 partition, or the exclusive cgroup v1 cpuset from `create_cpuset`), Deadline otherwise — FIFO on a shared CPU is exposed to `sched_rt_runtime_us` throttling (~50 ms dequeues) | verify the `sched_class=Auto → …` startup log; force either side with `Fifo`/`Deadline` |
| `sched_class = Deadline` | SCHED_DEADLINE CBS: guaranteed runtime per period on a *vanilla* kernel; a runaway RT thread gets throttled, not the system | needs `CONFIG_SCHED_DEADLINE`; runtime≈period fails admission (`EBUSY`) → falls back to FIFO |
| `dl_auto_measure` | measures P99 work duration for `dl_measure_cycles` (512) on FIFO first, then sets runtime = P99 × (1 + `dl_margin_pct`/100) + the HybridSpin tail if configured | check `es.dl_runtime_ns`/`es.deadline_active` after startup |
| `exec.disable_rt_throttle` | root: writes `-1` to `sched_rt_runtime_us` at thread start — no more ~50 ms RT-group dequeues | **isolated CPUs only** — on a shared CPU a runaway RT task then starves the core with no recovery window; the exec warns when written non-isolated |
| `exec.in_cycle_spin_ns` | forwarded `rx_spin_ns + slot_spin_ns` so the exec can warn when spins meet a Deadline runtime | informational — set by `startCyclicLoop`/`startAsyncLoop`, not a user knob |
| `cpu_isolation.*` | runtime CPU claims; `create_cpuset` builds an exclusive cgroup **v1** cpuset (`/sys/fs/cgroup/cpuset/tether_rt`) that RT threads join via `cgroup.threads`-style per-thread migration — per-thread CPU dedication needs v1 (v2 partitions are process-granular only); `steer_irqs` moves IRQs off | claims without a cpuset/`isolcpus=` are *reservations* — other tasks still preempt; `create_cpuset` needs root + a cpuset-v1 mount (hybrid cgroup layout); pure-v2 systems degrade to affinity-only with a warning |
| `rx_spin_ns` / `slot_spin_ns` | busy-poll the ring/deposit word instead of ppoll — DMA writes are visible with zero syscalls | **isolated CPUs only** — pure CPU burn inside the deadline AND charged to the SCHED_DEADLINE runtime on shared CPUs; keep `rx_spin_ns` ≪ `rx_budget_ns` |
| `memory_lock.*` | page faults are 1–10 µs each and can recur forever under memory pressure | all sections default on except `lock_all_process` (heaviest) |
| `exec.low_timer_slack` | `PR_SET_TIMERSLACK=1ns` — kills the kernel's ~50 µs default slack on sleeps | default on; only matters on the non-RT fallback path |
| `exec.stack_prefault_bytes` | pre-touches the cyclic thread's stack | default 128 KiB, on |
| `rx_budget_ns` | collect deadline for the wire reply; 0 = 80 % of cycle | must exceed real RTT — measure `rtt_us_*`; a VLAN hop or switch can push RTT well past 500 µs |

---

## Step 3 — Fix the operating system

If `realtime_diagnostics`/`cyclictest` show a bad floor, the problem is below
your process.  Ordered by impact:

1. **Kernel preemption.**  `detectRealtimeKernel()` tells you what you have.
   `PREEMPT_RT` > full preempt (`PREEMPT_DYNAMIC` active mode `full`) >
   low-latency desktop > generic.  You do **not** strictly need RT:
   `SCHED_DEADLINE` reserves CPU budget on a vanilla kernel.  But for
   worst-case-bound 4 kHz work on a busy machine, PREEMPT_RT is the answer —
   install the distro RT kernel (`linux-rt`, `linux-image-rt-amd64`, the RT
   Raspberry Pi OS kernel) or a PREEMPT_DYNAMIC kernel booted `preempt=full`.

2. **CPU frequency.**  Frequency ramping is wake jitter:

   ```bash
   # all cores, or at least the cyclic CPU
   for g in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
       echo performance | sudo tee "$g"
   done
   ```

   On Intel, pinning `scaling_min_freq` near `scaling_max_freq` removes ramp
   latency entirely.  On a Pi, `force_turbo=1` in config.txt does the same.

3. **Deep C-states.**  Waking a core out of C6 costs tens of µs.  Options:
   `intel_idle.max_cstate=1` (or `processor.max_cstate=1`) on the kernel
   command line, or keep a `/dev/cpu_dma_latency` fd open at 0 µs from your
   process.  On Pi-class SoCs this is mostly a non-issue.

4. **CPU isolation — use Tether's exclusive cpuset.**  This is the
   recommended path and it's already configured in Step 2:
   `cpu_isolation.enabled` + `create_cpuset` + `steer_irqs`, running as
   root.  The claim then *creates* an exclusive cgroup **v1** cpuset
   (`/sys/fs/cgroup/cpuset/tether_rt`, `cpu_exclusive=1`) covering the
   claimed CPUs and moves the cyclic threads' TIDs into its `tasks` file —
   true per-thread scheduler isolation **without a reboot**, torn down
   automatically on `stopCyclicLoop`/exit.  Per-thread dedication needs
   cgroup v1: v2 partitions are process-granular, so on a pure-v2 system
   (no `/sys/fs/cgroup/cpuset` mount — check before deploying) the claim
   degrades to affinity-only with a warning.  `steer_irqs` keeps
   interrupts off the same cores and restores the original masks on
   release.

   Only if the residual floor still isn't low enough, go deeper with
   boot-time `isolcpus=` — it additionally removes the CPU from the
   scheduler domains and combines with `nohz_full`/`rcu_nocbs`:

   ```
   isolcpus=domain,3 nohz_full=3 rcu_nocbs=3 irqaffinity=0-2
   ```

   `isolcpus=domain` removes the CPU from the scheduler domains; `nohz_full`
   stops the tick; `rcu_nocbs` offloads RCU callbacks; `irqaffinity` seeds
   default IRQ placement away from it.  Tether detects kernel-isolated CPUs
   via `/sys/devices/system/cpu/isolated` and prefers them automatically
   (`prefer_isolated`, default on) — the same Step-2 config picks them up
   with no extra work.

5. **IRQs.**  Disable `irqbalance` (or ban the cyclic CPU in its config), or
   let `cpu_isolation.steer_irqs` rewrite `/proc/irq/*/smp_affinity_list` at
   runtime — it saves originals and restores on release.  To do it by hand:

   ```bash
   for f in /proc/irq/*/smp_affinity_list; do echo 0-2 | sudo tee "$f"; done
   ```

   Keep CPU0 for housekeeping — `avoid_cpu0` (default) exists because CPU0
   takes the default IRQ landing and boot-time bookkeeping.

6. **The rest of the machine.**  Anything that runs can preempt: desktop
   environments, browser tabs, cron jobs, `updatedb`, packagekit.  On a
   dedicated controller keep the housekeeping load near zero.  Memory
   pressure is a jitter source too — prefault + mlock cover your own pages,
   but a swapping system stalls everything else around you.

---

## Step 4 — Fix the NIC and the wire

* **Dedicated NIC.**  Run EtherCAT on its own interface — not the LAN port
  carrying SSH/HTTP updates.  The kernel BPF demux stops async *EtherCAT*
  traffic from waking the cyclic thread, but a busy NIC still generates
  interrupts, softirq load and driver contention on your core.  Prefer PCIe
  NICs; USB NICs add USB scheduling jitter on every frame — usable at 1 kHz,
  a problem at 4 kHz.

* **EEE.**  Energy-Efficient Ethernet puts the PHY to sleep between bursts;
  LPI exit latency lands directly in your RTT as jitter.  `LinuxEthernet`
  probes it at init (`nicLatencyCheck`, default on) and tries to disable it
  (`nicDisableEee`, default on — needs `CAP_NET_ADMIN`).  If the warning
  persists:

  ```bash
  sudo ethtool --set-eee eth0 eee off
  ```

* **RX interrupt coalescing.**  Batched interrupts mean your cyclic response
  waits for the batch timer.  The init probe *warns* but doesn't change it:

  ```bash
  sudo ethtool -C eth0 rx-usecs 0 rx-frames 1
  ```

* **Direct link or dedicated segment.**  A switch between master and slaves
  adds queueing delay to RTT — usually fine, but measure it (`rtt_us_*`) and
  size `rx_budget_ns` above it.  Check the link is actually full-duplex and
  at expected speed (`ethtool eth0`).

* **Frame size vs. cycle budget.**  One max-size frame ≈ 120 µs on the wire
  at 100 Mbit/s.  At 4 kHz (250 µs) an image beyond ~2 slices/frames is
  physically infeasible regardless of software — shrink the image, drop to
  1 kHz, or use 1 Gbit (≈12 µs/frame) before blaming the scheduler.

---

## Step 5 — Keep the cycle work clean

The fastest scheduler in the world can't save a 300 µs task inside a 250 µs
cycle.  Rules for anything that runs in the cyclic thread (phase tasks,
`motion_in_loop` callback, exchange code):

* **No allocation, no I/O, no logging, no mutexes.**  `max_cycle_work_us`
  is your budget tracker — check it after any task addition.
* **Register everything before `start()`.**  `addTask` is not RT-safe.
* **Placement honesty.**  `Split`/`SplitLate` overlap the wire round-trip
  with your phases — free latency hiding — but in-loop readers see older
  inputs.  Controllers that need this cycle's inputs: `Atomic`.
* **Honor the image-mode contract.**  `TripleBuffered`/`Rotating` need a
  full refresh of `[0, outputBytes())` every cycle — a stalled writer sends
  stale bytes *by design*.  `DoubleBuffered` partial writers use
  `beginWrite()`/`commitOutputs()` for multi-field consistency.
* **DC placement.**  `DedicatedThread` (default) puts sync frames on their
  own higher-priority deadline thread — keep `dc_cpu` on a *different*
  claimed CPU than `cyclic_cpu`, and leave `dc_priority` above `priority`.
* **External motion source?**  `motion_in_loop = false` +
  `shm_image_name` moves the motion compute out of the cyclic thread
  entirely — the strongest possible work reduction.

---

## Step 6 — Read the residual signature

Still jittery?  The counters tell you which of the three sources remains:

| Observation | Points at | Do next |
|---|---|---|
| `max_jitter_us`/`max_wake_overshoot_ns` high, `missed_deadlines` ≈ 0 | wake latency | CPU isolation/cpuset, governor, C-states, PREEMPT_RT; HybridSpin only once the CPU is actually isolated |
| `missed_deadlines` rising + `max_cycle_work_us` near period | work overrun | shrink in-cycle work, move to `shm`/external motion, SplitLate |
| `wire_loss` ↑ while `stale_responses` ≈ 0 | frames not returning | link/switch/cabling, slave OP state, FMMU map |
| `wire_loss` ≈ `stale_responses` | replies late, not lost | `rtt_us_max` vs `rx_budget_ns`; NIC jitter, IRQ placement, EEE |
| `rtt_us_max` ≫ `rtt_us_min` | wire-side jitter | EEE, coalescing, shared NIC, switch queueing |
| `rx_bank_drops` > 0 | local bank exhaustion | a consumer is leaking `rxHold`s — audit view lifetimes |
| `kernel_rx_drops` (tp_drops) > 0 | ring not drained fast enough | collect happening too late — earlier `collect_phase`, CPU starvation |
| `txDeferred` > 0 | TX ring couldn't kick in time | a deadline event — CPU starvation or ring congestion |
| `slices_stuck_stale` > 0 | RTT > one full period | `rx_budget_ns`, cycle rate vs wire speed, image too big |
| `deadline_active == false` after requesting Deadline | CBS admission failed | `dl_runtime_ns` ≈ period on a busy CPU — isolate or use Fifo |
| `backendName()` = `SocketIO`/`nullptr` | channel fallback | missing `CAP_NET_RAW`, UDP encap, non-Linux — check startup log |

---

## Deployment checklists

**Minimum (unprivileged, dev box):**
`setcap 'cap_net_raw,cap_sys_nice=+ep'` on the binary → `lowLatency()` +
`image_mode = DoubleBuffered` → read startup warnings → check
`backendName()` and `jitter.max_jitter_us`.

**Recommended (root, no reboot):**
everything above + `cpu_isolation.create_cpuset` + `steer_irqs` +
`sched_class = Auto` (the default picks Deadline on non-isolated CPUs,
FIFO on isolated ones — check the `sched_class=Auto → …` log line) +
verify EEE off and coalescing off (the init warnings tell you) +
`performance` governor.  Only when the log confirms an isolated CPU:
add `HybridSpin`, `rx_spin_ns`, `slot_spin_ns`, and optionally
`exec.disable_rt_throttle` — they're dedicated-CPU tools, not
shared-CPU tools.

**Maximum (production 4 kHz):**
PREEMPT_RT kernel + governor pinned + dedicated PCIe NIC on its own
segment + all of Recommended + `lock_all_process` + alarm on
`missed_deadlines`, `wire_loss`, `txDeferred`, `kernel_rx_drops` in your
supervisor.  If the exclusive cpuset still leaves a residual floor, add
`isolcpus=domain,N nohz_full=N rcu_nocbs=N irqaffinity=…` at the kernel
command line — `prefer_isolated` finds it automatically.

**Then iterate:** change one thing at a time, re-run under load
(`stress-ng --cpu $(nproc) --io 2` in a separate shell), and keep what
measurably helps.  The reference point on a suitable deployment:
~10–30 µs worst-case wake jitter at a 250 µs period — vs. occasional
100 µs+ excursions on an untuned system.
