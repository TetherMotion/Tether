// SPDX-License-Identifier: MIT
/**
 * @file beckhoff_slave_chain.cpp
 * @brief Emulated Beckhoff I/O fleet on a virtual EtherCAT interface.
 *
 * Serves the 10-node fleet from BeckhoffFleet.hpp (two EK1100 couplers,
 * each followed by EL1004/EL2004/EL3104/EL4104) as a chain of
 * EtherCAT::slave::CiA401Slave instances.  Complete Ethernet frames
 * (dst MAC + ethertype 0x88A4 + EtherCAT payload) are exchanged one
 * frame at a time; each frame is run through every slave in chain
 * order and the response frame is sent back.
 *
 * Two carriers are supported:
 *  - UDP (default): one datagram carries one Ethernet frame.  No
 *    privileges needed; the peer learns the reply address from the
 *    sender of each datagram.
 *  - --iface NAME: raw L2 frames via AF_PACKET on a kernel interface,
 *    typically one end of a veth pair.  Needs CAP_NET_RAW; the peer
 *    is whatever is attached to the other end.
 *
 * The fleet's I/O is also exposed over Tether IO (tether_io TCP/SLIP):
 *   params  fleet.<key>.di / .ai<i>   (writable slave inputs)
 *   params  fleet.<key>.vendor / .product / .serial (read-only identity)
 *   signals fleet.<key>.do / .ao<i>   (read-only slave outputs)
 *   signals fleet.ticks               (1 kHz liveness counter)
 *
 * Usage:
 *   beckhoff_slave_chain [--port 47820] [--bind 127.0.0.1]
 *                        [--iface NAME]
 *                        [--io-port 4001] [--io-bind 127.0.0.1]
 *                        [--hz 1000] [--time SEC]
 *
 * A master connects either by sending EtherCAT frames to UDP port
 * --port (replies go to the datagram sender) or by attaching to the
 * other end of the --iface interface.  See ethercat_fleet_probe for
 * the matching master side.
 */
#include "common/BeckhoffFleet.hpp"
#include "common/UdpEthercatLink.hpp"
#include "common/PacketEthercatLink.hpp"

#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/Server.hpp"
#include "tether/io/TcpTransport.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <argparse/argparse.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace tether::examples;
using namespace tether::io;

namespace {

std::atomic<bool> g_stop{false};
void onSigint(int) { g_stop = true; }

uint64_t nowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

void slaveLog(const char* level, const char* msg, ...) {
    va_list ap;
    va_start(ap, msg);
    std::fprintf(stderr, "[fleet:%s] ", level);
    std::vfprintf(stderr, msg, ap);
    std::fprintf(stderr, "\n");
    va_end(ap);
}

std::atomic<uint64_t> g_ticks{0};

// ---------------------------------------------------------------------------
// Tether IO surface
// ---------------------------------------------------------------------------

/// One schema scalar node per IO entry; keys encode node+kind+channel.
/// Must outlive the SchemaCatalog (the catalog references the graph).
struct IoSchemas {
    SchemaGraph graph;
    std::vector<SchemaManifestEntry> manifest;

    SchemaRef addScalar(uint8_t nodeIdx, uint8_t kind, uint8_t channel,
                        ValueType type, const std::string& name) {
        SchemaNode n;
        n.key = {};
        n.key[0] = 0xB2;                    // beckhoff fleet namespace
        n.key[1] = nodeIdx;
        n.key[2] = kind;
        n.key[3] = channel;
        n.kind = SchemaKind::Scalar;
        n.scalarType = type;
        n.name = name;
        SchemaRef ref{n.key, computeSchemaDigest(n)};
        manifest.push_back({ref, 1});
        graph.nodes.push_back(std::move(n));
        return ref;
    }
};

/// Pending registry entries waiting for their post-install schema slot.
struct PendingIo {
    bool isParam = true;
    ParamEntry param;
    SignalEntry signal;
    SchemaRef ref;
};

/// Phase 1: build every schema node + registry entry (schemaSlot filled
/// in phase 3 once the catalog assigns real slots).
bool collectFleetIo(std::vector<PendingIo>& out, IoSchemas& s,
                    BeckhoffFleet& fleet, uint64_t& nextId) {
    const auto& infos = fleet.infos();

    auto addParam = [&](PendingIo& e, const std::string& name,
                        const std::string& desc, ValueType t,
                        ReadFn r, WriteFn w) {
        e.isParam = true;
        e.param.id = nextId++;
        e.param.name = name;
        e.param.group = "fleet";
        e.param.description = desc;
        e.param.valueType = t;
        e.param.readFn = std::move(r);
        e.param.writeFn = std::move(w);
        out.push_back(std::move(e));
    };
    auto addSignal = [&](PendingIo& e, const std::string& name,
                         const std::string& desc, ValueType t, ReadFn r) {
        e.isParam = false;
        e.signal.id = nextId++;
        e.signal.name = name;
        e.signal.group = "fleet";
        e.signal.description = desc;
        e.signal.valueType = t;
        e.signal.readFn = std::move(r);
        out.push_back(std::move(e));
    };

    // Identity params for every node (couplers included).
    for (size_t i = 0; i < infos.size(); ++i) {
        const auto& inf = infos[i];
        const uint8_t nodeIdx = static_cast<uint8_t>(i + 1);
        for (uint8_t k = 0; k < 3; ++k) {
            static const char* kSuffix[] = {".vendor", ".product", ".serial"};
            static const char* kDesc[] = {"Vendor ID (SII)",
                                          "Product code (SII)",
                                          "Serial number"};
            const uint32_t value = k == 0 ? inf.vendorId
                                : k == 1 ? inf.productCode
                                         : static_cast<uint32_t>(i + 1);
            PendingIo e;
            e.ref = s.addScalar(nodeIdx, 1, k, ValueType::U32,
                                inf.key + kSuffix[k]);
            addParam(e, inf.key + kSuffix[k], kDesc[k], ValueType::U32,
                     [value](void* dst) {
                         std::memcpy(dst, &value, sizeof(value));
                     }, nullptr);
        }
    }

    // I/O params/signals for the I/O modules.
    for (size_t i = 0; i < infos.size(); ++i) {
        const auto& inf = infos[i];
        if (inf.isCoupler) continue;
        const uint8_t nodeIdx = static_cast<uint8_t>(i + 1);
        auto& sl = fleet.slave(i);

        // Slave inputs (TxPDO): writable digital-input bytes + analog
        // channels.
        for (uint8_t g = 0; g < inf.diSize; ++g) {
            const std::string nm = inf.key + ".di" + std::to_string(g);
            PendingIo e;
            e.ref = s.addScalar(nodeIdx, 4, g, ValueType::U8, nm);
            addParam(e, nm, "Digital input byte " + std::to_string(g),
                     ValueType::U8,
                     [&sl, g](void* dst) {
                         const uint8_t v = sl.getDigitalInput8(g);
                         std::memcpy(dst, &v, 1);
                     },
                     [&sl, g](const void* src) {
                         sl.setDigitalInput8(g,
                             *static_cast<const uint8_t*>(src));
                     });
        }
        for (uint8_t c = 0; c < inf.aiSize / 2; ++c) {
            const std::string nm = inf.key + ".ai" + std::to_string(c);
            PendingIo e;
            e.ref = s.addScalar(nodeIdx, 5, c, ValueType::I16, nm);
            addParam(e, nm, "Analog input channel " + std::to_string(c),
                     ValueType::I16,
                     [&sl, c](void* dst) {
                         const int16_t v = sl.getAnalogInput(c);
                         std::memcpy(dst, &v, 2);
                     },
                     [&sl, c](const void* src) {
                         sl.setAnalogInput(c,
                             *static_cast<const int16_t*>(src));
                     });
        }

        // Slave outputs (RxPDO): read-only digital-output bytes + analog
        // channels, updated by the master's cyclic writes.
        for (uint8_t g = 0; g < inf.doSize; ++g) {
            const std::string nm = inf.key + ".do" + std::to_string(g);
            PendingIo e;
            e.ref = s.addScalar(nodeIdx, 6, g, ValueType::U8, nm);
            addSignal(e, nm, "Digital output byte " + std::to_string(g),
                      ValueType::U8,
                      [&sl, g](void* dst) {
                          const uint8_t v = sl.getDigitalOutput8(g);
                          std::memcpy(dst, &v, 1);
                      });
        }
        for (uint8_t c = 0; c < inf.aoSize / 2; ++c) {
            const std::string nm = inf.key + ".ao" + std::to_string(c);
            PendingIo e;
            e.ref = s.addScalar(nodeIdx, 7, c, ValueType::I16, nm);
            addSignal(e, nm, "Analog output channel " + std::to_string(c),
                      ValueType::I16,
                      [&sl, c](void* dst) {
                          const int16_t v = sl.getAnalogOutput(c);
                          std::memcpy(dst, &v, 2);
                      });
        }
    }

    // Liveness counter.
    {
        PendingIo e;
        e.ref = s.addScalar(0xFF, 0, 0, ValueType::U64, "ticks");
        addSignal(e, "ticks", "Slave tick counter (increments at --hz)",
                  ValueType::U64,
                  [](void* dst) {
                      const uint64_t v = g_ticks.load();
                      std::memcpy(dst, &v, 8);
                  });
    }
    return true;
}

/// Phase 2+3: install the schema catalog, resolve real slots, register.
bool installFleetIo(Registry& reg, SchemaCatalog& catalog,
                    IoSchemas& s, std::vector<PendingIo>& pending) {
    if (!catalog.install(s.graph, s.manifest)) return false;
    for (auto& e : pending) {
        const auto slot = catalog.slotFor(e.ref);
        if (!slot) return false;
        if (e.isParam) {
            e.param.schema = e.ref;
            e.param.schemaSlot = *slot;
            if (!reg.addParam(e.param)) return false;
        } else {
            e.signal.schema = e.ref;
            e.signal.schemaSlot = *slot;
            if (!reg.addSignal(e.signal)) return false;
        }
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser prog("beckhoff_slave_chain");
    prog.add_argument("--port")
        .help("UDP port carrying EtherCAT frames")
        .default_value(std::string("47820"));
    prog.add_argument("--bind")
        .help("UDP bind address")
        .default_value(std::string("127.0.0.1"));
    prog.add_argument("--iface")
        .help("raw-frame mode: kernel interface carrying EtherCAT "
              "frames (e.g. a veth end; overrides --port/--bind, "
              "needs CAP_NET_RAW)")
        .default_value(std::string(""));
    prog.add_argument("--io-port")
        .help("Tether IO TCP port (0 disables the IO server)")
        .default_value(std::string("4001"));
    prog.add_argument("--io-bind")
        .help("Tether IO bind address")
        .default_value(std::string("127.0.0.1"));
    prog.add_argument("--hz")
        .help("Slave tick rate in Hz")
        .default_value(std::string("1000"));
    prog.add_argument("--time")
        .help("Run for N seconds (0 = until SIGINT/SIGTERM)")
        .default_value(std::string("0"));
    prog.add_argument("--fleet")
        .help("fleet description file (EK1100/ELxxxx per line, chain "
              "order; empty = default two-coupler fleet)")
        .default_value(std::string(""));
    try { prog.parse_args(argc, argv); }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }

    const uint16_t udpPort = static_cast<uint16_t>(
        std::stoul(prog.get<std::string>("--port")));
    const std::string bind = prog.get<std::string>("--bind");
    const std::string iface = prog.get<std::string>("--iface");
    const uint16_t ioPort = static_cast<uint16_t>(
        std::stoul(prog.get<std::string>("--io-port")));
    const std::string ioBind = prog.get<std::string>("--io-bind");
    const uint32_t hz = std::stoul(prog.get<std::string>("--hz"));
    const double runSecs = std::stod(prog.get<std::string>("--time"));
    const std::string fleetPath = prog.get<std::string>("--fleet");
    if (hz == 0) {
        std::fprintf(stderr, "--hz must be > 0\n");
        return 2;
    }

    signal(SIGINT, onSigint);
    signal(SIGTERM, onSigint);

    std::vector<FleetSlaveInfo> infos;
    if (fleetPath.empty()) {
        infos = fleetInfos();
    } else {
        std::string ferr;
        infos = tether::examples::detail::fleetInfosFromFile(
            fleetPath, &ferr);
        if (infos.empty()) {
            std::fprintf(stderr, "beckhoff_slave_chain: %s\n",
                         ferr.c_str());
            return 1;
        }
    }
    BeckhoffFleet fleet(std::move(infos));
    fleet.start();

    UdpEthercatLink link;
    PacketEthercatLink packetLink;
    const bool useIface = !iface.empty();
    // The peer fields are unused on the server side; UDP replies go to
    // the sender learned from recvfrom(), packet replies take their
    // destination MAC from the frame header itself.
    if (useIface) {
        if (!packetLink.open(iface)) {
            std::fprintf(stderr,
                "beckhoff_slave_chain: AF_PACKET open on %s failed "
                "(needs CAP_NET_RAW and an existing interface)\n",
                iface.c_str());
            return 1;
        }
    } else if (!link.open("127.0.0.1", 0, udpPort, bind)) {
        std::fprintf(stderr, "beckhoff_slave_chain: UDP bind %s:%u failed\n",
                     bind.c_str(), udpPort);
        return 1;
    }

    // --- Tether IO server ---------------------------------------------------
    // IoSchemas must outlive the catalog (it references the graph).
    IoSchemas ioSchemas;
    std::vector<PendingIo> pendingIo;
    uint64_t nextIoId = 1;
    SchemaCatalog catalog;
    Registry registry;
    if (ioPort != 0) {
        if (!collectFleetIo(pendingIo, ioSchemas, fleet, nextIoId) ||
            !installFleetIo(registry, catalog, ioSchemas, pendingIo)) {
            std::fprintf(stderr, "beckhoff_slave_chain: IO registration failed\n");
            return 1;
        }
    }
    ServerConfig ioCfg;
    ioCfg.framing = Framing::Slip;
    ioCfg.timestampFn = nowUs;
    ioCfg.logFn = slaveLog;
    ioCfg.schemaCatalog = ioPort ? &catalog : nullptr;
    ioCfg.maxClients = 8;
    std::unique_ptr<Server> ioServer;
    if (ioPort != 0) {
        ioServer = std::make_unique<Server>(
            registry, std::make_unique<TcpTransportServer>(ioPort, ioBind),
            ioCfg);
        if (!ioServer->start()) {
            std::fprintf(stderr,
                "beckhoff_slave_chain: IO listen %s:%u failed\n",
                ioBind.c_str(), ioPort);
            return 1;
        }
    }

    const auto tickNs = 1'000'000'000ULL / hz;
    const auto tickPeriod =
        std::chrono::nanoseconds(static_cast<int64_t>(tickNs));
    const auto deadline = runSecs > 0
        ? std::chrono::steady_clock::now() +
              std::chrono::duration<double>(runSecs)
        : std::chrono::steady_clock::time_point::max();

    if (useIface) {
        std::printf("beckhoff_slave_chain: %zu slaves, EtherCAT/raw on %s",
                    fleet.slaveCount(), iface.c_str());
    } else {
        std::printf("beckhoff_slave_chain: %zu slaves, EtherCAT/UDP on %s:%u",
                    fleet.slaveCount(), bind.c_str(), udpPort);
    }
    if (ioServer) std::printf(", tether_io on %s:%u", ioBind.c_str(), ioPort);
    std::printf(" (%u Hz)\n", hz);
    std::fflush(stdout);

    uint8_t frame[2048];
    auto nextTick = std::chrono::steady_clock::now();
    pollfd pfd{useIface ? packetLink.fd() : link.fd(), POLLIN, 0};

    while (!g_stop && std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();
        int timeoutMs = 1;
        if (nextTick > now) {
            timeoutMs = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    nextTick - now).count());
            if (timeoutMs < 0) timeoutMs = 0;
            if (timeoutMs > 1) timeoutMs = 1;   // tick at --hz granularity
        }

        const int pr = ::poll(&pfd, 1, timeoutMs);
        if (pr > 0 && (pfd.revents & POLLIN)) {
            // Drain every pending frame; each is one Ethernet frame.
            for (;;) {
                size_t n = 0;
                if (useIface) {
                    if (!packetLink.recvFrom(frame, sizeof(frame), &n)) break;
                    auto resp = fleet.processFrame(frame, n);
                    if (!resp.empty()) {
                        packetLink.sendFrame(resp.data(), resp.size());
                    }
                } else {
                    sockaddr_in from{};
                    if (!link.recvFrom(frame, sizeof(frame), &n, from)) break;
                    auto resp = fleet.processFrame(frame, n);
                    if (!resp.empty()) {
                        link.sendTo(resp.data(), resp.size(), from);
                    }
                }
            }
        }

        if (std::chrono::steady_clock::now() >= nextTick) {
            fleet.tick(tickNs);
            ++g_ticks;
            nextTick += tickPeriod;
            // Avoid burst catch-up after a stall.
            if (nextTick < std::chrono::steady_clock::now()) {
                nextTick = std::chrono::steady_clock::now() + tickPeriod;
            }
        }
    }

    if (ioServer) ioServer->stop();
    fleet.stop();
    return 0;
}
