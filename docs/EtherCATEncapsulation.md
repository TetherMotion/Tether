# EtherCAT Frame Encapsulation (`--encapsulation`)

All Linux host examples accept a `--encapsulation` option that controls
how EtherCAT traffic is wrapped on the wire — raw EtherType `0x88A4`
frames, 802.1Q VLAN-tagged frames, EtherCAT-over-UDP (ETG.1000.3), or
combinations.  One flag drives the whole chain: kernel socket filter,
master UDP configuration, VLAN routing/tagging, and RX dispatch.

```
list_slaves -i eth0                                   # raw EtherCAT (default)
list_slaves -i eth0 --encapsulation vlan:1999         # VLAN 1999 only
list_slaves -i eth0 --encapsulation vlan:100-200,vlantx:150
list_slaves -i eth0 --encapsulation vlan:any,vlantx:off
list_slaves -i eth0 --encapsulation udp               # EtherCAT-over-UDP :34980
list_slaves -i eth0 --encapsulation udp:5000          # custom UDP port
list_slaves -i eth0 --encapsulation vlan:1999,udp     # VLAN + UDP combined
```

## On-the-wire formats

```
raw:      [dst MAC][src MAC][0x88A4][EtherCAT hdr][datagrams]
vlan:     [dst MAC][src MAC][0x8100][TCI][0x88A4][EtherCAT hdr][datagrams]
udp:      [dst MAC][src MAC][0x0800][IPv4][UDP dport=34980][EtherCAT hdr][datagrams]
vlan+udp: [dst MAC][src MAC][0x8100][TCI][0x0800][IPv4][UDP][EtherCAT hdr][datagrams]
```

The 802.1Q TCI word is `PCP(3) | DEI(1) | VID(12)` — VID 1–4095 are
usable; RX matching masks off PCP/DEI.

## Grammar

Comma-separated tokens, order-independent:

| Token                | Meaning |
|----------------------|---------|
| *(empty)*, `raw`, `none` | Plain EtherCAT (default). Cannot be combined with other tokens. |
| `vlan:<vid>`         | RX accepts only tagged frames with this VID; TX frames are tagged with the same VID. |
| `vlan:<lo>-<hi>`     | RX accepts tagged frames with VID in the inclusive range; TX tag defaults to `lo`. |
| `vlan:any`           | RX accepts *any* tagged EtherCAT frame (routed via the router's undefined target); TX is untagged unless `vlantx:` is given. |
| `vlantx:<vid>`       | Override the TX VLAN tag independently of the RX range. |
| `vlantx:off`         | TX frames are sent untagged even when RX is VLAN-filtered. |
| `udp`                | EtherCAT-over-UDP on the standard port 34980 (`0x88A4`). |
| `udp:<port>`         | EtherCAT-over-UDP on a custom destination port (1–65535). |

Duplicates (`vlan:` twice, `udp` twice, `vlantx:` twice) and unknown
tokens are rejected with a diagnostic on stderr.

## RX/TX semantics

RX and TX are configured independently:

- `vlan:1999` → RX accepts VID 1999, TX inserts tag 1999.
- `vlan:100-200` → RX accepts VID 100–200, TX inserts tag 100 (the
  range lower bound).
- `vlan:100-200,vlantx:150` → RX accepts 100–200, TX inserts 150 —
  useful when the master's replies must arrive on a different VLAN than
  the accepted receive window.
- `vlan:1999,vlantx:off` → RX accepts VID 1999, TX sent **untagged**
  (the switch adds the tag on egress, or the segment is one-way).
- `vlantx:200` alone → RX accepts untagged *and* any tagged EtherCAT,
  TX inserts tag 200 (legacy `--tx-vlan` behaviour).

TX tagging is applied by `VLANRouter`: each master runs on a per-master
`NetworkInterface` whose `send` prepends the 802.1Q header before
delegating to the raw backend.  UDP encapsulation is applied inside the
master's own transport (`EtherCATTransport` prepends IPv4+UDP headers
and the parser strips them on RX).  With `vlan:…,udp` the order on the
wire is always **outer VLAN, inner UDP** — matching what the kernel
filter and `Master::parseEtherCATFrame()` accept.

## What the filter rejects

In VLAN mode the attached cBPF program accepts a frame **iff** it is
802.1Q-tagged, the VID matches, and the inner EtherType is `0x88A4` (or
the inner UDP payload targets the EtherCAT port when `udp` is also
set).  **Untagged frames — including untagged EtherCAT — are dropped
in the kernel.**  QinQ/802.1ad frames, other VIDs, and tagged non-EtherCAT
payloads are likewise dropped.  See
[EtherCATBPFFiltering](EtherCATBPFFiltering.md) for the full filter
semantics, the `CBPFProgramFactory` API, and `CBPFSpec` for custom
policies (e.g. TPID `0x88A8`).

In UDP mode the filter additionally requires IPv4 protocol 17, IHL ≥ 5
(IP options handled), fragment offset 0, and the configured destination
port.  Truncated frames are rejected implicitly — out-of-bounds loads
fault in the kernel interpreter.

## Example code

Every example that calls `addInterfaceArg()` gets `--encapsulation` for
free.  The idiomatic host-side flow is:

```cpp
Tether::Examples::EncapsulationConfig encapsulation;
if (!Tether::Examples::parseEncapsulationArg(
        program.get<std::string>("--encapsulation"), encapsulation, TAG))
    return 1;
Tether::Examples::logEncapsulationConfig(encapsulation, TAG);

Tether::Examples::HostEtherNetSession session;
if (!Tether::Examples::initHostEthernet(session, iface, TAG))
    return 2;

EtherCAT::Master master;
if (!Tether::Examples::setupEncapsulation(session, master, encapsulation, TAG))
    return 5;                       // cBPF + UDP + VLAN router + RX callback

Tether::Examples::startHostPollThread(session, TAG);
if (!Tether::Examples::startHostMaster(session, master, TAG))
    return 5;                       // starts on session.masterInterface
```

`setupEncapsulation()` is the single call that applies the whole config:

1. attaches the kernel cBPF ingress filter to the socket,
2. enables EtherCAT-over-UDP on the master and disables the HAL's
   software EtherType filter (which would otherwise drop IPv4 frames),
3. creates and wires the `VLANRouter` when VLAN is active, or installs
   a direct `master.handleRxFrame` callback otherwise,

and stores the interface the master must `start()` on
(`session.masterInterface` — the router's per-master view under VLAN,
otherwise the session's own interface).

For code that manages its own `IEthernet`/`NetworkInterface` (no
session), the lower-level overload does the same work and returns the
interface to start on:

```cpp
std::unique_ptr<EtherCAT::VLANRouter> router;   // must outlive the session
EtherCAT::NetworkInterface* master_iface =
    Tether::Examples::setupEncapsulation(
        *eth, *ni, master, router, encapsulation, TAG);
if (!master_iface) return 5;
master.start(*master_iface, src_mac);
```

## Requirements and caveats

- **UDP requires a compile-time flag**: `vlan:` works in every build;
  `udp` needs `-DTETHER_ENABLE_UDP_ENCAPSULATION=ON` at configure time.
  On a build without it, `setupEncapsulation()` fails with a clear
  error rather than silently sending raw frames.
- **Filter attach is best-effort**: `SO_ATTACH_FILTER` needs only the
  `CAP_NET_RAW` the socket already holds; if the kernel rejects the
  program the code logs a warning and continues — userspace parsing
  still drops non-EtherCAT traffic, only the wake-up savings are lost.
- **NIC VLAN offload**: some NICs strip the VLAN tag in hardware.  The
  `VLANRouter` still tags TX correctly, but RX frames may arrive
  untagged depending on the driver — check `ethtool -k <iface>` for
  `rx-vlan-offload` if tagged traffic mysteriously disappears.
- **`vlan:any` and the undefined target**: frames with *any* VID are
  delivered to the master (after tag-stripping).  This is the catch-all
  mode — e.g. for `ethercat_dump_sii` on a trunk port.
- **Slave-side encapsulation**: `slave_emulator` accepts the flag and
  attaches the filter, but does **not** decapsulate — it warns about
  this at startup.  VLAN/UDP encapsulation is a host-side (master)
  feature.
- **`interpret_pcapng --vlan`** is an unrelated offline-analysis option
  and is not affected by this flag.

## API reference

```cpp
// examples/common/ExampleHelpers.hpp
struct EncapsulationConfig {
    std::optional<EtherCAT::VLANRouter::VLANRange> rxRange;  // VLAN RX range
    std::optional<uint16_t> txVlan;    // TX tag (nullopt = untagged)
    bool     rxAny   = false;          // vlan:any catch-all
    bool     udp     = false;          // EtherCAT-over-UDP
    uint16_t udpPort = EtherCAT::kEtherCATUdpPort;  // 34980
    bool vlanActive() const;           // VLAN routing/tagging in use
    bool enabled()    const;           // any encapsulation in use
};

bool parseEncapsulationArg(const std::string& spec,
                           EncapsulationConfig& out, const char* tag);
void logEncapsulationConfig(const EncapsulationConfig& config,
                            const char* tag);
std::vector<EtherCAT::CBPFInsn>
    buildEncapsulationBpfProgram(const EncapsulationConfig& config);
void attachEncapsulationBpfFilter(EtherCAT::HAL::IEthernet& eth,
                                  const EncapsulationConfig& config,
                                  const char* tag);
EtherCAT::NetworkInterface* setupEncapsulation(
    EtherCAT::HAL::IEthernet& eth, EtherCAT::NetworkInterface& backend,
    EtherCAT::Master& master,
    std::unique_ptr<EtherCAT::VLANRouter>& routerStorage,
    const EncapsulationConfig& encapsulation, const char* tag);

// examples/common/EtherCATHostSetup.hpp — session wrappers
bool setupEncapsulation(HostEtherNetSession&, EtherCAT::Master&,
                        const EncapsulationConfig&, const char* tag);
bool startHostMaster(HostEtherNetSession&, EtherCAT::Master&,
                     const char* tag);
```
