# PDO Address Space and Multi-Slave Process Image

This document explains how Tether maps PDO data for one or more slaves: the
three address domains involved, how the logical process image is laid out,
how exchanges cover every configured slave in a single cycle, and how to
bring multiple drives online incrementally.

## The three address domains

PDO traffic touches three distinct address domains. Keeping them separate
makes the API much easier to reason about:

| Domain | Scope | Set by | Used by |
|--------|-------|--------|---------|
| **Slave selection** | the wire — which slave a datagram reaches | `PDOAddressMode` per mapping entry (`Position`, `ConfiguredAddress`, `Broadcast`, `Logical`) | APRD/APWR, FPRD/FPWR, BRD/BWR datagrams |
| **Physical (ESC memory)** | inside one slave — the Sync Manager buffer holding its process data | SM2/SM3 physical start addresses from the slave's SII (e.g. `0x1800` / `0x1C00`); per-entry offsets assigned by `PDOManager::finalizeMapping()` | register-level reads/writes, FMMU physical side |
| **Logical (process image)** | the whole bus — one flat 32-bit address space shared by all slaves | `LogicalAddressManager` (base `0x10000` by default) | LRW datagrams, FMMU logical side |

### Slave-selection addressing

Each `PDOEntry` carries a `PDOAddressMode` that decides which datagram
command a *direct, per-entry* exchange uses:

- `Position` — auto-increment (APRD/APWR). The ADP is `0 − slave_index`
  (`Master::adpForSlaveIndex()`); the frame is consumed by the Nth slave
  on the wire. This is the default and works regardless of the slave's
  configured station address.
- `ConfiguredAddress` — fixed addressing (FPRD/FPWR) against the slave's
  station address (ESC register `0x0010`). Tell the mapping the value
  first with `PDOMapping::set_slave_configured_address()` — it is copied
  into each entry at `add_*pdo()` time, so call it **before** registering
  the slave's entries.
- `Broadcast` — BRD/BWR against `physical_offset` directly (every slave
  that sees the offset participates).
- `Logical` — the entry lives in the LRW process image; it is skipped by
  the per-entry datagram path and exchanged by the
  `LogicalAddressManager` instead.

### Per-slave physical layout

Inside each slave, process data lives in Sync Manager buffers — typically
SM2 for outputs (RxPDO, master→slave) and SM3 for inputs (TxPDO,
slave→master), with physical start addresses taken from the SII
(`0x1800`/`0x1C00` on many drives).

`PDOManager::finalizeMapping(slave_index)` walks the mapping and assigns
each of that slave's entries a `physical_offset`: RxPDO entries are packed
consecutively at the SM2 base, TxPDO entries consecutively at the SM3
base, **in registration order**. It also updates the slave's recorded SM
lengths:

```
slave ESC memory
  SM2 @0x1800: [RxPDO entry a][RxPDO entry b][...]   ← outputs
  SM3 @0x1C00: [TxPDO entry a][TxPDO entry b][...]   ← inputs
```

Several entries per direction on the same slave simply concatenate — the
slave sees one contiguous SM image and your entries are byte windows into
it.

### The logical process image

For the cyclic exchange, Tether does not address slaves individually.
`LogicalAddressManager` allocates each slave a **contiguous logical
window** = `[RxPDO bytes][TxPDO bytes]`, starting at the base logical
address (`0x00010000` by default, `setBaseLogicalAddress()` to change).
Windows are assigned **append-only in configuration order** and are never
relocated — a slave's FMMU is programmed once against its window, so a
later `buildAddressMap()` must not move it; re-configured or late-joining
slaves get a fresh window at the end.

Worked example — two identical drives, each with a 35-byte RxPDO and a
47-byte TxPDO:

```
logical address   region                          mapped onto
0x00010000 ┬ slave0 RxPDO  (35 B / 0x23)    →  slave0 SM2 @0x1800  [output FMMU]
0x00010023 ┴ slave0 TxPDO  (47 B / 0x2F)    →  slave0 SM3 @0x1C00  [input FMMU]
0x00010052 ┬ slave1 RxPDO  (35 B / 0x23)    →  slave1 SM2 @0x1800  [output FMMU]
0x00010075 ┴ slave1 TxPDO  (47 B / 0x2F)    →  slave1 SM3 @0x1C00  [input FMMU]
           total image = 0xA4 = 164 bytes  →  one LRW datagram
```

During each slave's OP-transition configuration
(`Master::configureProcessDataSyncManagersFromSii`), the master calls
`finalizeMapping()`, rebuilds the address map, then programs the slave's
FMMUs: one **output** FMMU maps the slave's RxPDO logical range onto its
SM2 physical base; one **input** FMMU maps the TxPDO logical range onto
its SM3 base.

A single LRW datagram covering `[base, base + image_size)` then reaches
every slave at once: each ESC's FMMUs write "their" output slice into SM2
and read "their" input slice out of SM3. WKC semantics follow the
EtherCAT rules — a slave that writes output bytes in the datagram's range
adds +1, a slave that reads input bytes adds +2 — so a healthy full-image
exchange on N such drives returns `WKC = 3 × N`. The cyclic path derives
per-slice expectations automatically (`deriveExpectedWkc()`); slices that
cannot be derived fall back to learning the WKC of the first successful
response.

Queries into this layout:

- `LogicalAddressManager::getRxPDOLogicalAddr/Length(slave)` /
  `getTxPDOLogicalAddr/Length(slave)` — a slave's window halves.
- `getSlaveLogicalWindow(slave, offset, length)` — the slave's whole
  `[offset, offset+length)` slice relative to the base.
- `describeEntries(mapping)` / `PDOManager::describeLogicalEntries()` —
  every enabled entry's `{offset, length, direction}` in image space.
- `totalRxPDOBytes()` / `totalTxPDOBytes()` / `totalLogicalSize()`.

## Where the API pieces live

| Task | API |
|------|-----|
| Register an output image for a slave | `pdo().mapping().add_rxpdo(slave, size, pdo_index, mode)` → entry index |
| Register an input image for a slave | `pdo().mapping().add_txpdo(slave, size, pdo_index, mode)` → entry index |
| Set a station alias before registering | `pdo().mapping().set_slave_configured_address(slave, addr)` |
| Read/write an entry's bytes | `mapping().entryDataMut(idx)` / `entryData(idx)` / `entryDataAs<T>(idx)` (manager-owned storage) |
| Cache an entry pointer | `mapping().entryHandle(idx)` + `resolve()` — epoch-checked; invalidated by `clear()`/`remove_entries_for_slave()` |
| Assign physical offsets + SM lengths | `pdo().finalizeMapping(slave)` — per slave, after its entries exist |
| Drop a slave's entries | `mapping().remove_entries_for_slave(slave)` — compacts; other slaves' entry indices are preserved |
| Drive the exchange | `pdo().exchangeAll()`, or the DC/cyclic loop via `cyclicSend()`/`cyclicCollect()` |
| Inspect the image layout | `pdo().describeLogicalEntries()`, `logicalAddressManager()` |

`add_*pdo()` **appends without deduplication**. Registering a second
output buffer for a slave does not replace the first — both get distinct
physical offsets inside SM2 and the slave's SM length grows to cover both.
To change a slave's image, remove its entries first
(`remove_entries_for_slave()`), re-register, and re-finalize.

## One exchange covers every slave

The exchange granularity is the **whole process image**, not the slave.
`PDOManager::exchangeAll()` (and `cyclicSend()`/`cyclicCollect()` on the
realtime path, or the DC-owned loop) emits one LRW datagram covering
`[base, base + totalLogicalSize)` — or several contiguous LRW slices when
the image exceeds `maxLogicalSliceLength()` — touching every configured
slave's FMMU region. There is intentionally no "exchange only slave 3".

The consequence for multi-drive code: **stage each slave's outputs into
its own entry storage, run a single exchange, then collect each slave's
inputs.** One wire cycle updates the entire fleet.

```cpp
auto& pdo = master.pdo();
auto& map = pdo.mapping();

// Register one output + one input buffer per drive (sizes need not match
// across slaves — each window is sized independently).
int rx0 = map.add_rxpdo(0, 35, 0x1600);
int tx0 = map.add_txpdo(0, 47, 0x1A00);
int rx1 = map.add_rxpdo(1, 35, 0x1600);
int tx1 = map.add_txpdo(1, 47, 0x1A00);

pdo.finalizeMapping(0);   // pack slave 0's entries at its SM2/SM3 base
pdo.finalizeMapping(1);   // same for slave 1

// Cache storage pointers once (manager-owned; marks the entries
// storage_bound so image-mode exchanges keep them bridged).
uint8_t*       out0 = map.entryDataMut(rx0);
uint8_t*       out1 = map.entryDataMut(rx1);
const uint8_t* in0  = map.entryData(tx0);
const uint8_t* in1  = map.entryData(tx1);

while (running) {
    writeDemandA(out0);            // stage drive 0's outputs
    writeDemandB(out1);            // stage drive 1's outputs

    pdo.exchangeAll();             // ONE wire exchange covers both slaves

    readFeedbackA(in0);            // collect drive 0's inputs
    readFeedbackB(in1);            // collect drive 1's inputs
    wait_next_cycle();
}
```

A device-facing wrapper class can therefore hold just its own two buffers
plus its entry indices — the exchange itself stays a shared,
manager-level operation.

### Serialization rule

Exactly one agent may pump the exchange at a time. Do not call
`exchangeAll()`/`exchangeLRWSlice()` from a foreground thread while a DC
realtime loop, `CyclicExecutive`, queue-mode loop, or `PDOKeepAlive`
worker is also exchanging — they share `PDOManager` split state and the
same wire indices. Stop the background pump (`PDOKeepAlive::stop()`,
`Master::suspendCyclicExchange()`) before running polled exchanges, and
vice versa.

## Partial and per-slave exchanges

When a full-image exchange is wrong (image larger than one frame, a
fast subset needing a higher rate, or slaves still being configured),
the image can be exchanged in pieces:

| API | Behaviour |
|-----|-----------|
| `pdo().exchangeLRWSlice(offset, len)` | One atomic LRW over `[offset, offset+len)` — only intersected entries are gathered/scattered |
| `pdo().maxLogicalSliceLength()` | Largest slice fitting one datagram (`frame payload − 12 B overhead`) |
| `pdo().describeLogicalEntries()` | Entry placements, for composing slices on real boundaries |
| `pdo().exchangeConfiguredSlaveWindows()` | One LRW per slave that already has a window — the keep-alive exchange |
| `LogicalAddressManager::exchangeLRWForSlaves(mapping, mask)` | LRW limited to a slave bitmask |
| `pdo().definePDOSlice(spec)` | Declarative slice (entries or ranges) with its own wire index and `every_n` decimation, exchanged alongside the cyclic image |
| `pdo().setImageExchangeDecimation(n)` | Run the whole-image exchange only every Nth cycle |

A process image bigger than `maxLogicalSliceLength()` is handled
automatically on the cyclic path: `cyclicSend()` splits it into
`maxSliceLength()`-sized LRW slices across reserved slots (bounded by
`IPDOTransport::kNumCyclicSlots`).

## Bringing slaves up incrementally

Slaves are normally configured one at a time (mailbox → PRE_OP → SDOs →
PDO assignment → SAFE_OP → OP), which can take ~1 s per drive. The
mapping machinery is built for that:

- **Sticky windows**: when slave *k* finishes `finalizeMapping()`, the
  next `buildAddressMap()` appends its window after the existing ones.
  Already-configured slaves keep the windows their FMMUs were programmed
  with — the image grows at the end, never in the middle.
- **Keep-alive**: `pdo().startKeepAlive(period)` starts a background
  `PDOKeepAlive` thread that calls `exchangeConfiguredSlaveWindows()` —
  one LRW datagram per slave that already has a window. This feeds the
  earlier slaves' SM process-data watchdogs while a later slave runs its
  slow INIT→OP sequence; the new slave joins the image automatically once
  its entries are registered and it is skipped until then. Stop the
  keep-alive before taking over with foreground exchanges.
- **Re-registration**: `remove_entries_for_slave()` compacts the entry
  array but only removes that slave's entries — other slaves keep their
  indices. It bumps the mapping epoch, so outstanding `entryHandle()`s
  fail `resolve()` instead of pointing at recycled storage.

See [MultiDriveInitialization](MultiDriveInitialization.md) for the full
bring-up pattern and the failure modes the keep-alive prevents.

## Multiple logical address spaces

For disjoint slave groups (e.g. different update rates or separate
management), `Master::createPdoGroup(slave_indices, base_logical_addr)`
creates an extra `PDOManager` + `LogicalAddressManager` pair with its own
address space. Each group's base must not overlap another's (the default
space starts at `0x10000`). `pdoForSlave(slave)` and
`logicalAddressManagerForSlave(slave)` then route to the owning group;
the default `pdo()`/`logicalAddressManager()` covers everything not
assigned to a group.

## Limits

| Limit | Default | Tunable |
|-------|---------|---------|
| Slaves with PDOs | 64 (`PDO::kMaxPDOSlaves`) | `ECAT_PDO_MAX_SLAVES` (EtherCAT cap 247) |
| Mapping entries total | 32 (`PDO::kMaxPDOEntries`) | `ECAT_PDO_MAX_ENTRIES` |
| Bytes per entry storage | 1024 (`PDO::kMaxPDOSize`) | `ECAT_PDO_MAX_BUFFER_SIZE` |
| Entries per slave | 16 | `ECAT_PDO_MAX_ENTRIES_PER_SLAVE` |
| PDOs per sync manager (multi-PDO) | 16 | `PDO::kMaxPDOsPerSM` |
| LRW slice length | frame payload − 12 B | transport `maxEtherCATPayloadPerFrame()` |
| Logical base address | `0x00010000` | `LogicalAddressManager::setBaseLogicalAddress()` |

## See also

- `include/tether/ethercat/PDOManager.hpp` — mapping entries, exchange API
- `include/tether/ethercat/LogicalAddressManager.hpp` — logical layout, LRW, slices
- `include/tether/fmmu/FMMUManager.hpp` — per-slave FMMU programming
- `include/tether/ethercat/ProcessImage.hpp` — image-buffered exchange modes
- [PDOModes](PDOModes.md) — Direct/Queue/Callback interaction modes
- [MultiDriveInitialization](MultiDriveInitialization.md) — multi-slave bring-up and `PDOKeepAlive`
- [CyclicRealtimeTransport](CyclicRealtimeTransport.md) — the cyclic fast path and wire timing
- `tests/ethercat/test_logical_address_manager.cpp` — layout and slice tests
