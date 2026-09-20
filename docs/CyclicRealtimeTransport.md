# Cyclic Realtime Transport

This document describes the zero-copy cyclic datapath: the concepts behind it,
how it is implemented, where its limits are, and why the design choices were
made.  It covers `ICyclicChannel`, the Linux socket/PACKET_MMAP backends, the
kernel-BPF traffic demultiplexer, the `ProcessImage` multi-buffering API, and
the `Master`/`LogicalAddressManager` integration.

Target audience: developers integrating an external motion source (Klipper-style
planners, CNC controllers) or porting the cyclic datapath to a new platform.

---

## 1. Motivation

The framework targets deterministic cyclic operation up to **4 kHz** on
Raspberry-Pi-class embedded Linux systems — a 250 µs cycle budget.  The
original cyclic path was functionally correct but paid for it in jitter:

| Stage (per cycle, LRW path)        | TX copies | RX copies | per-entry loops | sync primitives        |
|------------------------------------|-----------|-----------|-----------------|------------------------|
| Legacy path                        | 3         | 3         | 2 (gather+scatter) | router + condvar + mutex |
| This design — socket backend       | 0–1       | 1         | 0–1             | seqlock + eventfd      |
| This design — ring + Rotating      | **0**     | **0**     | **0**           | seqlock only           |

The payload bandwidth is trivial (a typical image is 200–500 B/cycle).  The
real jitter sources were structural:

* **Per-entry gather/scatter loops** — branchy, data-dependent work inside
  the deadline.
* **`RxDatagram` materialization** — a 1.5 kB copy per response, plus a
  `TransactionRouter` lookup and a condition-variable wakeup.
* **Wake-up races** — the asynchronous poll thread and the cyclic thread
  shared one socket; whichever blocked in `poll()` first got the response,
  so the cyclic response frequently took a detour through the poll thread
  (deposit → eventfd → wake).
* **API friction** — an external motion source had to bounce setpoints
  through per-entry `app_buffer` pointers rather than writing a process
  image directly.

The redesign removes copies where they are cheap to remove, removes the
router/condvar from the hot path entirely, and makes the kernel — not the
thread scheduler — responsible for keeping cyclic and async traffic apart.

### Design goals, in order

1. **Determinism before bandwidth.**  Every mechanism is judged on worst-case
   latency, not throughput.
2. **Correctness must never depend on acceleration.**  Every fast path has a
   software fallback with identical semantics; a failed BPF attach or an
   unavailable ring degrades performance, never correctness.
3. **A selectable safety/speed hierarchy.**  The application picks how much
   contract it is willing to sign (safe multi-buffered → unsafe zero-copy).
4. **External motion sources are first-class.**  Writing setpoints must be
   possible from a non-Tether thread with no per-entry bookkeeping.
5. **Platform-specific datapaths.**  Linux gets rings+ BPF; microcontrollers
   and Windows get their own channel implementations behind the same
   interface.

---

## 2. The wire contract

Everything builds on one invariant about the EtherCAT datagram index byte.

### 2.1 Reserved index range

```
idx 0x00..0xF7   asynchronous traffic  (allocIdx() pool, router-routed)
idx 0xF8..0xFD   cyclic slots          (kCyclicSlotBaseIdx, kNumCyclicSlots=6)
idx 0xFE         fire-and-forget       (kFireAndForgetIdx)
idx 0xFF         unused
```

`kCyclicSlotBaseIdx`/`kNumCyclicSlots` are defined in `Types.hpp`; the same
values are re-exported as `IPDOTransport::kCyclicSlotBase` /
`kNumCyclicSlots`.  `allocIdx()` never returns an index ≥ 0xF8, so **no async
datagram can ever carry a cyclic index** — the separation is enforced by the
allocator, not by convention.

### 2.2 Why index-based demultiplexing is exact

EtherCAT slaves never originate frames.  Every frame on the wire was created
by this master, and a response frame returns with the *same datagrams in the
same order with the same indexes*.  Therefore the index of the *first*
datagram in a received frame identifies the class of the whole frame —
provided a frame never mixes classes, which the allocator guarantees.

The first datagram's index sits at a fixed byte offset in an untagged frame:

```
byte  0..13   Ethernet header (dst 6 | src 6 | EtherType 0x88A4)
byte 14..15   EtherCAT header (len:11 | rsvd:1 | type:4)
byte 16       datagram command
byte 17       datagram index          ← the demux byte
byte 18..25   datagram address/len/irq fields
byte 26       first datagram payload  (kCyclicFramePayloadOff)
```

There is no L2 fragmentation in EtherCAT: a frame is always whole on the
wire, and the "more" flag only chains datagrams *inside* one frame.  A
process image larger than one datagram (~1468 B payload) is exchanged as
multiple LRW datagrams (`exchangeLRWSlice`); in the cyclic path each slice
gets its own slot index, so every one of those frames still has a cyclic
first index.

### 2.3 Fixed response slots

Each reserved index maps to a fixed `CyclicRxSlot` in `Master`:

```cpp
struct CyclicRxSlot {
    std::atomic<uint64_t> seq{0};   // bumped after fields are written
    uint8_t  cmd;  uint16_t adp, ado, datalen, wkc;
    const uint8_t* payload;         // view into channel memory, or data[]
    int64_t cookie{-1};             // channel cookie, -1 = inline copy
    uint8_t  data[1486];            // inline buffer for the copy path
};
```

Publication is single-writer: write fields → `seq.fetch_add(1, release)`.
The reader (cyclic thread) snapshots the token *before* sending and waits
for `seq != token` — no router lookup, no allocation, no condvar on the
wait itself.

---

## 3. Architecture

```
                 ┌──────────────────────────────────────────────┐
                 │ CyclicExecutive (deadline + phases, shared)  │
                 └───────────────┬──────────────────────────────┘
                                 │  exchange_fn → PDOManager
                                 │             → LogicalAddressManager
                                 │             → IPDOTransport fast path
                 ┌───────────────▼──────────────────────────────┐
                 │  Master cyclic slot bank + ICyclicChannel    │
                 │  waitCyclicSlotView / dispatchChannelFrame   │
                 └───────┬───────────────────────┬──────────────┘
                         │                       │
              ┌──────────▼─────────┐   ┌─────────▼───────────┐
              │ Socket A (cyclic)  │   │ Socket B (async)    │
              │ AF_PACKET, 0x88A4  │   │ existing iface fd   │
              │ BPF: idx∈[F8,FD]   │   │ BPF: idx∉[F8,FD]    │
              │ ├ ring backend     │   │ poll thread, router │
              │ └ socket backend   │   │ (unchanged)         │
              └────────────────────┘   └─────────────────────┘
```

The `CyclicExecutive` (deadline scheduling, phase ordering, jitter stats) is
unchanged and platform-shared.  The wire datapath is pluggable behind
`ICyclicChannel`; Linux has two backends, other platforms get their own.

### Component map

| Component | File | Role |
|---|---|---|
| `ICyclicChannel`, `CyclicFrameView`, `CyclicTxParts`, `CyclicSlotView` | `include/tether/ethercat/CyclicChannel.hpp` | Datapath abstraction |
| `LinuxSocketChannel`, `LinuxRingChannel`, BPF programs, factory | `src/ethercat/raw/CyclicChannel_linux.cpp` | Linux backends |
| `ProcessImage`, `ImageMode` | `include/tether/ethercat/ProcessImage.hpp`, `src/ethercat/ProcessImage.cpp` | Multi-buffered image API |
| Slot publish/dispatch/wait, channel TX | `src/ethercat/raw/Master_transport.cpp` | Master fast path |
| Channel + image wiring in `startCyclicLoop` | `src/ethercat/raw/Master.cpp` | Lifecycle |
| `computeImageOffsets`, `exchangeAllLRWCyclic` | `src/ethercat/raw/LogicalAddressManager.cpp` | Image-mode exchange |
| `PDOEntry::image_exclude`, `CyclicLoopConfig::{wire_mode,image_mode}` | `PDOManager.hpp`, `Master.hpp` | Configuration surface |

---

## 4. `ICyclicChannel` — the datapath abstraction

```cpp
class ICyclicChannel {
    // TX
    virtual uint8_t* txAcquire() = 0;                     // compose in place
    virtual size_t   txCapacity() const = 0;
    virtual bool     txCommitFrame(uint32_t len) = 0;
    virtual bool     txSendParts(const CyclicTxParts&);   // scatter-gather
    // RX
    virtual int  rxPoll(CyclicFrameView*, int, uint32_t timeout_ns) = 0;
    virtual void rxHold(uint32_t cookie) = 0;
    virtual void rxRelease(uint32_t cookie) = 0;
    // introspection
    virtual int  fd() const = 0;
    virtual bool zeroCopy() const = 0;
    virtual const char* backendName() const = 0;
    virtual uint64_t droppedRx() const { return 0; }
};
```

### 4.1 RX view model

`rxPoll()` fills `CyclicFrameView` records — `{frame, frame_len, stamp_ns,
cookie}` — pointing into **channel-owned memory** (a ring slot or a bank
buffer).  A view is valid until the next `rxPoll()` *unless held*:
`rxHold(cookie)`/`rxRelease(cookie)` refcount the underlying buffer.  This is
what lets the slot table and the process image publish *pointers* instead of
copies.

Contract details:

* `rxPoll` returns `>0` views filled, `0` on timeout, `<0` on error
  (`-errno`).  Callers must keep the result signed — assigning it to an
  unsigned type turns `-EAGAIN` into a giant loop bound.
* Hold/release is a *consumer* contract.  Releasing an unheld cookie is a
  contract violation; both backends clamp the refcount at 0 so a
  double-release cannot pin a slot forever.
* `droppedRx()` counts *deferrals* — `rxPoll` calls that could not emit a
  received frame because every bank slot was held.  The datagram stays
  queued in the kernel; nothing is lost.  A rising counter means a consumer
  is leaking holds.

### 4.2 TX: two styles

* **`txAcquire()`/`txCommitFrame(len)`** — compose a complete Ethernet frame
  in channel memory.  Required by the *Rotating* image mode (the app writes
  the payload region of the acquired buffer directly).  On the ring backend
  this is a kernel-mapped TX slot → zero copies.
* **`txSendParts(header, payload, wkc)`** — scatter-gather from separate
  buffers.  Socket backend: one `sendmsg()` with a 3-part `iovec` → zero
  payload copies.  Ring backend: one contiguous copy into a TX slot.
  Default implementation stages through `txAcquire()` (one copy) so new
  backends get correct behaviour for free.

### 4.3 Backend selection

`CyclicWireMode`:

| Mode | Behaviour |
|---|---|
| `Auto` (default) | Try PACKET_MMAP rings; on any failure log a warning and fall back to socket mode. |
| `SocketIO` | Plain AF_PACKET `recvfrom`/`sendmsg` + owned frame bank. |
| `PacketRing` | Rings only — `createCyclicChannel` returns `nullptr` if setup fails. |

If no channel can be created at all (no `CAP_NET_RAW`, bad ifindex,
non-Linux), `Master` runs the **software deposit path**: the existing socket
+ parser deposits into the slot's inline `data[]` buffer.  Same semantics,
one extra copy.

---

## 5. Linux backends

### 5.1 `LinuxSocketChannel`

* RX: a bank of 4 × 1600 B buffers.  `rxPoll` drains the nonblocking socket
  into free bank slots; each emitted slot gets a refcount.  Unheld slots are
  recycled at the top of the next `rxPoll`.
* The timeout path is a single `ppoll()` on the fd — the cyclic thread
  sleeps in the kernel until the frame arrives.
* Timestamps are `CLOCK_MONOTONIC` (`PACKET_TIMESTAMP` is enabled on the
  socket; software stamps are used for simplicity — see limitations).
* TX `sendmsg` builds `sockaddr_ll` from the frame's own destination-MAC
  bytes, so callers never configure MACs on the channel.
* Frames shorter than 60 B are padded on send (minimum Ethernet frame
  without FCS).

### 5.2 `LinuxRingChannel` — PACKET_MMAP TPACKET_V2

Two `mmap`'d rings on the same socket: `PACKET_RX_RING` (default 128 ×
4 KiB blocks) and `PACKET_TX_RING` (16 blocks).  Frames are laid out one per
slot (`frame_size = TPACKET_ALIGN(TPACKET2_HDRLEN + 1600)`).

**RX walk:** `rxPoll` scans slot headers for `TP_STATUS_USER`.  A surfaced
frame keeps `TP_STATUS_USER` and is marked *consumed*; the slot returns to
the kernel (`TP_STATUS_KERNEL`) only when consumed *and* hold-count is zero —
either at the start of the next walk or eagerly inside `rxRelease`.  The
view's `frame` pointer is `hdr + tp_mac`, `stamp_ns` comes from the ring
metadata (`tp_sec`/`tp_nsec` — kernel timestamps, hardware when the NIC
provides them).

**TX:** `txAcquire` hands back a `TP_STATUS_AVAILABLE` slot (skipping
in-flight ones); `txCommitFrame` stamps `tp_len`/`tp_mac`/`tp_net`, flips
`TP_STATUS_SEND_REQUEST`, and kicks the ring with one `sendto(fd, NULL, 0,
MSG_DONTWAIT)`.  One kick flushes *every* queued slot — a multi-datagram
cyclic frame or back-to-back slices cost one syscall total.  `EAGAIN` on the
kick is benign (deferred to the next commit).

If the TX ring fails to initialize the channel keeps running: TX falls back
to the socket path (`sendto`/`sendmsg`) while RX stays zero-copy.

**Why TPACKET_V2 and not V3:** V3 packs variable-size frames into blocks,
which is a memory-density optimisation for mixed-size capture traffic.  It
complicates per-frame hold/release for no benefit at our frame sizes — V2's
"one frame per slot, status flag flip" model maps exactly onto the cookie
refcount contract.

### 5.3 Two sockets, kernel-enforced separation

Socket A (the channel) and socket B (the existing async socket) are two
independent AF_PACKET taps on the same interface.  Each carries a classic
BPF filter (`SO_ATTACH_FILTER` + `SO_LOCK_FILTER`) that runs in softirq
**before** the kernel copies the frame into that socket's queue or ring:

```c
/* socket A program — mirror program on socket B */
ldh  [12]              ; EtherType
jeq  0x88A4, +0, ->rej ; non-ECAT → reject on A (accept on B)
ldb  [17]              ; first datagram idx
jge  0xF8, +0, ->rej   ; below cyclic range → reject on A
jgt  0xFD, ->rej, +0   ; above cyclic range → reject on A
ret  ACCEPT            ; A: accept / B: reject
ret  REJECT            ; A: reject / B: accept
```

Consequences:

* **The poll thread literally cannot wake for cyclic traffic** — rejected
  frames are never copied, queued, or scheduled on socket B.  The wake-up
  race that used to route cyclic responses through the non-RT thread is
  eliminated, not just made unlikely.
* **The cyclic ring stays clean** — an async burst (SDO storm, FoE transfer)
  cannot occupy ring slots ahead of the cyclic response.
* **Cross-traffic is impossible by construction**, not by scheduling
  convention.
* **Failure is soft.**  `SO_ATTACH_FILTER` failing logs a warning and the
  sockets run unfiltered; a cyclic datagram arriving on socket B is still
  deposited correctly because `parseEtherCATFrame` keys on the index byte,
  not on which socket delivered the frame.  The filter is a *latency
  boundary*, never a correctness boundary.
* `PACKET_IGNORE_OUTGOING` is set on both sockets so socket B does not tap
  socket A's transmitted frames.

Edge semantics (verified by tests):

* **Short frames** (< 18 B): the index load is out-of-bounds; kernel cBPF
  semantics reject the packet on *both* filters.  Malformed traffic is
  dropped entirely — desirable.
* **Non-EtherCAT EtherTypes**: socket B accepts them (the EtherType branch
  bypasses the index load), socket A rejects.
* **VLAN-tagged frames** (EtherType 0x8100): fail the EtherType check →
  async path.  Consistent with the existing "no fast path over VLAN" rule;
  the filter could be extended to check byte 21 on 0x8100 if that ever
  changes.
* **Ordering between sockets is not synchronized** — an async TX can
  overtake a cyclic TX sent µs earlier on the other socket.  Benign for
  EtherCAT (protocols are per-slave ordered by arrival), but documented.

---

## 6. `ProcessImage` — the multi-buffered image API

The LRW datagram payload *is* a fixed-layout process image: RxPDO region
`[0, rx_bytes)` (master→slave setpoints), TxPDO region `[rx_bytes,
rx_bytes+tx_bytes)` (slave→master inputs).  `ProcessImage` exposes it
directly instead of bouncing through per-entry `app_buffer` pointers.

### 6.1 Mode hierarchy (safe → unsafe)

| Mode | Writer contract | Copies/cycle | Tearing | Pointer stability |
|---|---|---|---|---|
| `Buffered` | legacy `app_buffer` gather/scatter | 2 + per-entry loops | n/a | n/a |
| `DoubleBuffered` | **partial writes OK** — call `commitOutputs()` after a burst | 1 hidden memcpy (W→S under seqlock) | none for any field size when burst markers used; aligned ≤4 B fields never tear regardless | `outputWrite()` stable |
| `Direct` | writer in-loop, or only aligned ≤4 B fields | 0 | possible on >4 B fields from a foreign thread | stable |
| `TripleBuffered` | **full refresh per commit** | 0 | impossible (buffer rotation) | `outputWrite()` rotates |
| `Rotating` | **full refresh every cycle** + re-query pointer per cycle | **0 — writes land in the TX frame** | impossible (frame ownership) | changes every cycle |

#### DoubleBuffered — carry-forward semantics

`dbl_[0]` is a *persistent* write buffer owned by the application writer;
`dbl_[1]` is the send snapshot owned by the wire.  Untouched fields keep
their last values forever — partial writes are inherently safe.

`acquireSendImage()` (cyclic thread) copies W→S only when `commitOutputs()`
set the *dirty* bit; the copy runs under a seqlock word
(`dblSeq_`: bits 31:2 sequence, bit 1 dirty, bit 0 writer-active):

* Without a commit, the previous snapshot is resent verbatim — a stalled
  writer never produces a blank or rotated frame.
* A commit landing *during* the copy sets dirty again → next cycle refreshes.
* `beginWrite()`/`commitOutputs()` bracket a multi-field burst so the
  seqlock detects mid-burst snapshots (retries up to 8, then sends the
  best-effort copy — aligned ≤4 B fields are still coherent values).
* The W→S copy is one contiguous `memcpy` hidden inside the RX wait window
  — tens of nanoseconds for a typical image — versus the per-entry loops it
  replaces.

This is the mode for external motion sources that update a subset of axes
per cycle (e.g. only the axes that moved).

#### TripleBuffered — lock-free publication

Three buffers rotate through write/ready/send roles via a packed atomic
(`bits [1:0]=write, [3:2]=ready, [5:4]=send, bit6=fresh`).  `commitOutputs()`
swaps write↔ready and sets fresh; `acquireSendImage()` swaps ready↔send and
returns the *old ready* buffer.  Tearing is impossible at any field size
because the wire never reads the buffer being written.

Contract: the writer must **fully refresh** the output region per commit —
each buffer carries its own history, there is no carry-forward across
commits.  Repeated acquires without a commit resend the last sent buffer
(the fresh bit gates rotation).

#### Rotating — write into the next frame

`outputWrite()` points *inside the channel's next TX frame* at
`base + kCyclicFramePayloadOff`.  The exchange composes the 26-byte header
in place, appends WKC, commits — zero copies end to end, including on the
ring backend where the buffer is kernel-mapped.

Contract: **full refresh every cycle and re-query the pointer per cycle.**
After commit the pointer goes null until the executive attaches the next
frame (`attachTxFrame`); writes to nullptr are dropped, and unwritten bytes
would be sent verbatim — there is intentionally no memset on this path.

#### Direct — single staging image

One owned image; `outputWrite()` is the send buffer.  Safe when the only
writer is in-loop (MotionControl phase — same thread as the exchange) or
when all written fields are naturally aligned ≤4-byte values
(single-copy-atomic on x86 and ARMv7+).  An asynchronous writer tearing a
wider field is the documented hazard.

### 6.2 Input side — always view-based

`publishInputView()` points `inputRead()` at the received payload inside
channel ring/bank memory (held via cookie); `publishInputCopy()` stages into
an owned bank allocated at `configure()` time for the no-channel path (no
RT allocation).  `inputSeq()` bumps per publish — readers detect new data or
staleness.  The previous hold is released before the new one is taken;
`configure({})`/destruction release the final hold.

Input is the *whole LRW payload* — outputs echo at `[0, rx_bytes)`, inputs
at `[rx_bytes, size)` — matching what the wire returns.

### 6.3 Entry addressing and forced-buffered entries

`entryOffset(i)` is entry *i*'s byte offset in the image, computed by
`LogicalAddressManager::computeImageOffsets()` using the same layout walk as
`describeEntries()` (per-slave running offsets from the FMMU base).  Entries
marked `-1` stay on the `app_buffer` path even in image modes — the gather/
scatter loops in `exchangeAllLRWCyclic` skip only entries with `entryOffset
>= 0`.

Marked `-1`:

* **Bit-packed entries** — any entry whose `[offset, offset+size)` overlaps
  a neighbour's.  Two writers doing read-modify-write on a shared byte is a
  real race; detection is a pairwise range check over the mapping.
* **`PDOEntry::image_exclude`** — set for FSoE-managed PDOs: safe frames are
  staged and CRC'd by the FSoE layer and must not be written in place.
* Disabled entries, zero-size entries, entries on inactive slaves.

Typed accessors `outputPtr<T>(entry_idx)` / `inputPtr<T>(entry_idx)` combine
offset lookup + bounds check; `epoch()` bumps on every `configure()` so
cached offsets/pointers can be validated across re-mapping.

---

## 7. Master integration

### 7.1 Exchange flow (`LogicalAddressManager::exchangeAllLRWCyclic`)

1. Pick the TX payload:
   * `Rotating` → the payload region of the acquired TX frame
     (`image->rotatingFrameBase()`, acquiring + attaching on first use).
   * `Direct`/`Double`/`TripleBuffered` → `image->acquireSendImage()`.
   * Buffered/fallback → the persistent `cyclic_payload_` staging buffer
     (memset + full gather, legacy semantics).
2. Gather RxPDO `app_buffer` bytes for **forced-buffered entries only**
   (`entryOffset(i) < 0`).  No memset on image paths — unwritten regions
   carry stale bytes forward, which is exactly the Rotating/DoubleBuffered
   contract.
3. Send on reserved slot 0:
   * Rotating → `composeCyclicHeader` into the frame + `sendCyclicFrame`;
     then attach next cycle's frame so `outputWrite()` stays valid.
   * Otherwise → `sendCyclicDatagram` → channel `txSendParts` (socket:
     `sendmsg` iovec, zero payload copies) or legacy `sendWithEncapsulation`.
4. `waitCyclicSlotView(slot, token, timeout)` — see below.
5. Publish the response payload as the input image (`publishInputView` when
   a channel holds it, else `publishInputCopy`).
6. Scatter TxPDO bytes into `app_buffer` for forced-buffered entries only.

WKC == 0 and timeout are still counted and reported as failures — unchanged
semantics.

### 7.2 `dispatchChannelFrame` — frame classification

Every frame surfaced by the channel's `rxPoll` gets a *classify pass* that
walks the datagram chain (honouring the `more` flag and bounds):

* **All datagrams cyclic** → publish pass: each datagram's payload view is
  published to its slot (`rxHold` the frame cookie per published datagram;
  the slot releases the *previous* cookie it held).  Multi-datagram cyclic
  frames — e.g. `[LRW 0xF8][ARMW DC-feedback 0xF9]` — fill all their slots
  from one ring read.
* **Anything else** (async or mixed — only possible when the BPF is absent
  or failed) → the whole frame goes to `handleRxFrame`, the normal parser,
  which deposits cyclic datagrams by copy and routes async ones.  Nothing
  is ever lost to misclassification.

### 7.3 `waitCyclicSlotView` — the wait hierarchy

The wait tries, in order:

1. **Fast path** — `seq != token` already → seqlock-read the slot (bounded
   8-try re-check; a second publish mid-read is detected and retried).
2. **Channel drain** — when a channel exists, the cyclic thread calls
   `rxPoll` itself with the remaining deadline and dispatches every frame.
   This is the zero-hop path: the RT thread receives its own response.
3. **Software path** — `ppoll()` on `{deposit eventfd, iface socket}`:
   the eventfd is written by `depositCyclicSlot`/`publishCyclicSlotView` so
   a poll thread that won the recv race still wakes the waiter immediately;
   the socket fd is drained inline when readable (direct-recv fallback).
4. **Bounded spin** — no fds available (non-Linux, no eventfd) → spin on
   `seq` until the deadline.  Bounded so a dead link cannot wedge the
   cyclic thread.

Cancellation (`cancel_requested_`) is checked in every loop.

`waitCyclicSlot` (the old `RxDatagram`-copy API) is now a thin wrapper that
copies the view out — legacy callers keep working.

### 7.4 Lifecycle (`startCyclicLoop` / `stopCyclicLoop`)

`startCyclicLoop`:

* Creates the channel only when the transport exposes a raw fd:
  `iface_.receive && iface_.native_handle && !isUdpEncapsulationEnabled()`.
  VLAN and UDP-encapsulated transports keep the software path.
* Derives `ifindex` via `getsockname()` on the iface fd; passes it as
  `async_fd` so the mirror BPF lands on socket B.
* `ImageMode::Rotating` without a channel degrades to `Direct` (logged).
* `configureProcessImage` pulls `computeImageOffsets` + total rx/tx bytes
  from the LAM; failure → Buffered (logged).
* The exchange closure calls `exchangeAllLRWCyclic(200 µs, &process_image_)`.

`stopCyclicLoop`: stops the executive, `process_image_.configure({})`
(releases the held input cookie *before* the channel dies), releases every
slot's held cookie, then destroys the channel.  Order matters — the
channel must outlive every cookie it issued.

### 7.5 Test seam

`friend struct MasterCyclicTestAccess` exposes `cyclic_channel_` injection
and `dispatchChannelFrame` to tests — the whole publish/release lifecycle
is testable with a scripted `ICyclicChannel`, no raw sockets needed.

---

## 8. Correctness & fallback hierarchy

Every acceleration layer degrades independently:

```
ring backend      ──setup fail──▶  socket backend
channel exists?   ──no CAP_NET_RAW▶ software deposit path (inline copies)
BPF attached?     ──fail─────────▶ unfiltered sockets + parser deposit
Rotating mode     ──no channel───▶ Direct mode
image mode        ──configure fail▶ Buffered (app_buffer) exchange
cyclic fast path  ──no transport──▶ exchangeAllLRW (router path)
```

Invariants that hold on *every* level:

* A cyclic-index datagram always ends up in its slot — via view publish,
  inline deposit, or the parser — regardless of which socket/fd delivered it.
* A missed response is a timeout error counted in stats, never a hang:
  every wait is deadline-bounded.
* `allocIdx()` can never allocate a reserved index; the demux invariant is
  structural.
* Held buffers are refcounted; the channel outlives its cookies by
  construction of `stopCyclicLoop`.

---

## 9. Limitations & hazards

### 9.1 Platform / privilege

* **`CAP_NET_RAW` is required** for any channel (AF_PACKET).  Without it the
  master silently runs the software path — same semantics, more copies.
  `Auto` mode's ring→socket fallback needs the capability too; only the
  socket-fd-less deposit path needs nothing.
* **Linux only.**  Non-Linux builds return `nullptr` from all factories and
  attach helpers; the software deposit path is the portable baseline.
  ESP32 (EMAC descriptor channel) and Windows (Npcap) channels are intended
  follow-ups behind the same interface — the `ICyclicChannel` contract was
  shaped for them (e.g. `rxRelease` returning driver-owned buffers to a DMA
  pool).
* **VLAN-tagged networks** get no fast path: tagged frames route to the
  async socket and through `VLANRouter`.  Both the fast path and the
  channel stay non-VLAN.
* **UDP encapsulation** and transports without `receive`/`native_handle`
  (pure polling transports) never get a channel.
* **Two sockets** means two AF_PACKET fds and two binds; on systems where
  socket count is restricted this can matter.

### 9.2 Mechanism limits

* **TPACKET_V2 linear scan.**  `walkRing` scans all RX slots per call —
  O(`rx_frames`) ≈ 256 header checks worst case.  At our rates (a handful
  of frames per cycle) this is microseconds of predictable work, but it is
  not a cursor-based ring walk.
* **Frame size ceiling.**  Bank/ring slots are sized for 1600 B frames;
  jumbo frames are not supported.  `txSendParts`/`txCommitFrame` reject
  over-capacity compositions.
* **Bank exhaustion** on the socket backend defers delivery (frames stay
  kernel-queued) and bumps `droppedRx()` — monitor it if you hold views.
* **`stamp_ns` precision.**  Ring backend uses kernel (optionally hardware)
  timestamps; socket backend uses `CLOCK_MONOTONIC` at `recvfrom` time.
  For wire-time jitter measurement prefer the ring backend.
* **Software timestamps** are taken in the RX path; the latency includes
  scheduling delay.  `SIOCSHWTSTAMP`-grade hardware timestamps are a
  possible extension (V2 metadata carries them when enabled).

### 9.3 Application contract hazards

* **Rotating mode sends stale bytes** if the writer doesn't fully refresh —
  by design; there is no memset or versioning on that path.  A stalled
  writer produces repeated-but-valid frames only because slaves see a valid
  datagram; the *contents* are whatever was last written.
* **Direct mode can tear** >4-byte or unaligned fields when written from a
  foreign thread during `sendmsg`.  In-loop writers and aligned ≤4 B fields
  are safe by hardware single-copy atomicity.
* **DoubleBuffered without burst markers** can send a mid-burst snapshot —
  individual ≤4 B fields stay coherent, but a cross-field invariant
  (e.g. position+velocity pair) can be split.  Use `beginWrite()`/
  `commitOutputs()` for multi-field updates.
* **Pointer lifetime.**  `inputRead()`/`outputWrite()` pointers are
  invalidated by the next publish/commit/cycle attach and by
  `configure()` — check `epoch()` if you cache them, and never cache
  `Rotating`'s pointer across cycles.
* **Image offsets are byte-granular.**  Bit-level PDO fields inside an
  image-mapped entry can still be updated by the app, but two entries that
  share a byte are forced-buffered precisely because byte-level RMW races
  can't be made safe.
* **Ordering between sockets** is not globally serialized (§5.3).

---

## 10. Rationale — decisions and alternatives considered

**Why a channel abstraction instead of #ifdef'd fast paths?**
The platform datapaths are genuinely different (AF_PACKET rings vs. ESP32
EMAC descriptors vs. Windows Npcap buffering), while the executive,
slot semantics, and image modes are shared.  An interface with per-platform
implementations keeps `Master` platform-clean and makes the whole datapath
mockable — every Master-level test injects a scripted channel.

**Why two sockets + BPF instead of one socket with an owner rule?**
With one socket, whoever is blocked in `poll()` when the response lands
gets the frame — and the non-RT poll thread, which waits permanently, often
wins.  The "direct drain" then degenerates to deposit→eventfd→wake, the hop
we set out to remove.  Two readers on one PACKET_MMAP ring are genuinely
racy.  Two sockets + cBPF make crosstalk *impossible* in kernel for ~80
lines of filter code, keep the poll thread's code unchanged, and fail soft.

**Why TPACKET_V2 over V3?**  Per-frame slots map 1:1 onto the cookie
hold/release contract; V3's block packing optimises memory for mixed-size
capture traffic — irrelevant at 1600 B fixed slots — and complicates
per-frame holds.

**Why the persistent-write-buffer DoubleBuffered instead of rotating
ping-pong?**  A rotating pair breaks partial writers: the rotated-in buffer
carries two-cycle-old values for untouched fields.  Options were
full-refresh contract (that's just TripleBuffered) or copy-forward after
swap (a hidden memcpy per cycle *and* pointer instability).  The persistent
W + snapshot S design gives partial-write safety, a stable write pointer,
and one bounded memcpy under a seqlock — the strongest contract per cost.

**Why is the BPF filter not a correctness boundary?**
Because a missed demux is unobservable except in latency: the parser keys
on the index byte.  Keeping correctness off the filter means `SO_ATTACH_FILTER`
failure, a kernel without cBPF, or a future mixed-index frame are all
performance events, never protocol bugs.

**Why keep `app_buffer` at all?**  Per-entry granularity.  FSoE-managed PDOs
must stay staged for CRC computation, bit-packed entries can't be written
in place, and Buffered mode remains the semantic reference implementation —
the image modes are opt-in per `CyclicLoopConfig::image_mode`.

**Why eventfd + ppoll instead of the condvar?**
`ppoll` on real fds gives kernel-scheduled wakeup with nanosecond deadlines
and no userspace lock; the eventfd only exists for the residual case where
the poll thread consumed the frame (BPF absent/failed).  On the ring path
the cyclic thread polls the ring itself — no wakeup object at all.

**Cost accounting honesty:** the bandwidth removed is ~µs-scale per cycle;
the wins are the *structural* costs — per-entry loops, the 1.5 kB
`RxDatagram` materialization, router lookup, condvar wakeup latency, and the
poll-thread detour — which are exactly the jitter sources on a 250 µs
budget.  Zero-copy is the enabler; deterministic ownership is the point.

---

## 11. Testing & verification

Tests live in `tests/ethercat/test_cyclic_channel.cpp` (35 tests) and
`tests/ethercat/test_process_image.cpp` (21 tests), in the
`tether_ethercat_master_tests` / `tether_ethercat_pdo_tests` targets.

### What is proven where

| Layer | Method | Coverage |
|---|---|---|
| BPF program correctness | userspace cBPF interpreter over the exact exported instruction bytes (`cyclicChannelBpfProgram`) | accept/reject at boundary indexes 0xF7/0xF8/0xFD/0xFE, non-ECAT EtherTypes, short frames |
| BPF **kernel execution** | same bytes attached via `SO_ATTACH_FILTER` to an `AF_UNIX` datagram socketpair — the real kernel verifier+interpreter, no privileges needed | accept/reject behaviour in-kernel |
| Socket channel | `createCyclicSocketChannelForFd` over `AF_UNIX` sockets | rxPoll delivery/timeout/nonblock, bank exhaustion + `droppedRx`, hold/release lifecycle, invalid cookies, TX failure paths, `txSendParts` staging + oversize + null-payload |
| Master fast path | `MasterCyclicTestAccess` + scripted channel | software deposit copy path, view publish + cookie release, mixed-frame → parser routing, short-frame ignore, `txSendParts` composition, timeout + cancellation, header layout |
| ProcessImage / LAM | `configure()` + fake `IPDOTransport` | all 5 modes, carry-forward, commit gating, rotating attach/detach, publish view/copy + seq, entry offsets + exclusions, epoch, WKC/timeout failures, forced-buffered gather/scatter |
| **Live kernel demux + rings** | `LivePacketTest` on loopback: two real AF_PACKET sockets with opposite filters, real ring RX/TX, factory `Auto` | **auto-skipped without `CAP_NET_RAW`** — runs in privileged CI |
| Factory validation | `CyclicChannelFactory.InvalidIfindexFails` — fails before socket creation | unconditional |

### The privilege seam

The container/CI matrix matters: `AF_UNIX` sockets accept `SO_ATTACH_FILTER`
without privileges, which is what makes the kernel-BPF tests runnable
everywhere.  `createCyclicSocketChannelForFd` /
`createCyclicRingChannelForFd` exist so backends are exercisable over
caller-provided fds.  The live AF_PACKET tests (`KernelDemuxOnLoopback`,
`RingChannelRxAndTx`, `FactoryAutoOnLoopback`, `AsyncFilterAttachViaConfig`)
gate on a runtime `CAP_NET_RAW` probe and `GTEST_SKIP()` — they are real
integration coverage where privileges exist, and honest skips where they
don't.

A skipped live test is **not** claimed as live AF_PACKET coverage — the
kernel-verifier path (AF_UNIX attach) plus the userspace VM over the exact
instruction bytes is the proof that runs unconditionally.

---

## 12. API quick reference

```cpp
// --- Configure ---------------------------------------------------------
Master::CyclicLoopConfig cfg;
cfg.cycle_period_us = 250;                       // 4 kHz
cfg.wire_mode  = CyclicWireMode::Auto;           // ring → socket → software
cfg.image_mode = ImageMode::DoubleBuffered;      // or Rotating / Triple / Direct
cfg.motion_in_loop = false;                      // external motion source
master.startCyclicLoop(cfg);

// --- Writer thread (any thread) ----------------------------------------
auto& img = master.processImage();
if (auto* w = img.outputWrite()) {
    // mode-dependent contract:
    //   Double/Partial: poke changed fields, then commitOutputs()
    //   Triple/Rotating: fully refresh [0, outputBytes()) every cycle
    img.beginWrite();                            // DoubleBuffered, optional
    *img.outputPtr<int32_t>(axis_pos_entry) = target;
    img.commitOutputs();
}

// --- Reader -------------------------------------------------------------
const uint64_t s = img.inputSeq();               // bump = new frame
if (const uint8_t* in = img.inputRead())
    int32_t pos = *img.inputPtr<int32_t>(status_pos_entry);

// --- Entry offsets (stable within an epoch) -----------------------------
const int32_t off = img.entryOffset(entry_idx);  // -1 = app_buffer path
const uint32_t ep = img.epoch();                 // changes on reconfigure
```

`PDOManager::configureProcessImage()` is called internally by
`startCyclicLoop`; `PDOEntry::image_exclude` is the per-entry opt-out (set
it for FSoE-managed PDOs).  `Master::cyclicChannel()` exposes the active
channel for diagnostics (`backendName()`, `zeroCopy()`, `droppedRx()`).

---

## 13. Source map

| File | Contents |
|---|---|
| `include/tether/ethercat/CyclicChannel.hpp` | `ICyclicChannel`, views, modes, config, factories, BPF access |
| `src/ethercat/raw/CyclicChannel_linux.cpp` | BPF programs, `LinuxSocketChannel`, `LinuxRingChannel`, factory, non-Linux stubs |
| `include/tether/ethercat/ProcessImage.hpp` / `src/ethercat/ProcessImage.cpp` | image modes, buffers, publish/hold, entry offsets |
| `src/ethercat/raw/Master_transport.cpp` | `depositCyclicSlot`, `publishCyclicSlotView`, `dispatchChannelFrame`, `waitCyclicSlotView`, `sendCyclicDatagram`/`sendCyclicFrame`/`composeCyclicHeader` |
| `src/ethercat/raw/Master.cpp` | `startCyclicLoop`/`stopCyclicLoop` wiring, `MasterPDOTransport` forwards |
| `src/ethercat/raw/LogicalAddressManager.cpp` | `computeImageOffsets`, `exchangeAllLRWCyclic` |
| `include/tether/ethercat/PDOManager.hpp` | `PDOEntry::image_exclude`, `IPDOTransport` cyclic API |
| `include/tether/ethercat/Master.hpp` | `CyclicLoopConfig::{wire_mode,image_mode}`, slot bank, test seam |
| `tests/ethercat/test_cyclic_channel.cpp` | BPF VM + kernel + socket + Master + live tests |
| `tests/ethercat/test_process_image.cpp` | image modes + LAM exchange tests |
