// SPDX-License-Identifier: MIT
/**
 * @file ethercat_fleet_probe.cpp
 * @brief EtherCAT master for the emulated Beckhoff fleet.
 *
 * Connects to beckhoff_slave_chain over a virtual link as an
 * EtherCAT master: discovery, AL transitions INIT->PREOP->SAFEOP->OP,
 * SM2/SM3 + fixed PDO registration, then cyclic process-data exchange.
 * The link is UDP by default (--peer/--port) or raw L2 frames on a
 * kernel interface with --iface (e.g. a veth end; needs CAP_NET_RAW).
 *
 * Modes:
 *   probe (default)  bring the bus up, verify the 10-node Beckhoff
 *                    topology and SII identities, run --cycles cyclic
 *                    exchanges, dump the PDO images, exit 0/1.
 *   --simdir DIR     pump mode: bridge PDO entries to files so an
 *                    external controller (e.g. the TetherPLC bridge)
 *                    can drive/observe the fleet:
 *                      <key>.tx  TxPDO image (slave inputs, written here)
 *                      <key>.rx  RxPDO image (slave outputs, read here)
 *                    Files are raw PDO payloads of exactly the entry's
 *                    data_size bytes, little-endian fields as laid out
 *                    by CiA401Slave (digital bytes then int16 analogs).
 *
 * Usage:
 *   ethercat_fleet_probe [--peer 127.0.0.1] [--port 47820]
 *                        [--iface NAME]
 *                        [--cycles 100] [--simdir DIR] [--hz 200]
 *                        [--time SEC]
 */
#include "common/BeckhoffFleet.hpp"
#include "common/EthercatFleetMaster.hpp"

#include <argparse/argparse.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

using namespace tether::examples;

namespace {

std::atomic<bool> g_stop{false};
void onSigint(int) { g_stop = true; }

std::string hexDump(const uint8_t* p, size_t n, size_t max = 32) {
    std::string out;
    const size_t m = n < max ? n : max;
    char buf[4];
    for (size_t i = 0; i < m; ++i) {
        std::snprintf(buf, sizeof(buf), "%02x", p[i]);
        out += buf;
        if (i + 1 < m) out += ' ';
    }
    if (n > max) out += " ..";
    return out;
}

/// Write @p data to @p path via temp+rename so readers never see a
/// partial file.
bool writeFileAtomic(const std::filesystem::path& path,
                     const uint8_t* data, size_t len) {
    const auto tmp = path.string() + ".tmp";
    FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(data, 1, len, f) == len;
    std::fclose(f);
    if (!ok) { std::remove(tmp.c_str()); return false; }
    return std::rename(tmp.c_str(), path.c_str()) == 0;
}

bool readFile(const std::filesystem::path& path,
              uint8_t* dst, size_t len) {
    FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    const size_t n = std::fread(dst, 1, len, f);
    std::fclose(f);
    return n == len;
}

const FleetSlaveInfo* infoForIndex(
    const std::vector<FleetSlaveInfo>& infos, uint16_t index) {
    for (const auto& i : infos)
        if (i.chainIndex == index) return &i;
    return nullptr;
}

/// Locate the enabled master-side PDO entry for @p slave / direction.
int findPdoEntry(FleetMaster& fm, uint16_t slave, bool txpdo) {
    return fm.findEntry(slave, txpdo);
}

} // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser prog("ethercat_fleet_probe");
    prog.add_argument("--peer")
        .help("slave UDP host")
        .default_value(std::string("127.0.0.1"));
    prog.add_argument("--port")
        .help("slave UDP port")
        .default_value(std::string("47820"));
    prog.add_argument("--iface")
        .help("raw-frame mode: kernel interface carrying EtherCAT "
              "frames (e.g. a veth end; overrides --peer/--port, "
              "needs CAP_NET_RAW)")
        .default_value(std::string(""));
    prog.add_argument("--cycles")
        .help("cyclic exchanges in probe mode")
        .default_value(std::string("100"));
    prog.add_argument("--simdir")
        .help("pump mode: directory for <key>.tx/.rx PDO files "
              "(empty = probe mode)")
        .default_value(std::string(""));
    prog.add_argument("--hz")
        .help("pump cycle rate in Hz")
        .default_value(std::string("200"));
    prog.add_argument("--time")
        .help("pump duration in seconds (0 = until signal)")
        .default_value(std::string("0"));
    try { prog.parse_args(argc, argv); }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 2;
    }

    signal(SIGINT, onSigint);
    signal(SIGTERM, onSigint);

    const std::string peer = prog.get<std::string>("--peer");
    const uint16_t port = static_cast<uint16_t>(
        std::stoul(prog.get<std::string>("--port")));
    const std::string iface = prog.get<std::string>("--iface");
    const uint32_t cycles = std::stoul(prog.get<std::string>("--cycles"));
    const std::string simdir = prog.get<std::string>("--simdir");
    const uint32_t hz = std::stoul(prog.get<std::string>("--hz"));
    const double runSecs = std::stod(prog.get<std::string>("--time"));

    const auto infos = fleetInfos();

    FleetMaster fm;
    const bool linkOk = iface.empty() ? fm.open(peer, port)
                                      : fm.openIface(iface);
    if (!linkOk) {
        std::fprintf(stderr, "ethercat_fleet_probe: link open failed\n");
        return 1;
    }

    std::vector<EtherCAT::DiscoveredSlave> discovered;
    const int n = fm.bringUp(infos, &discovered);
    if (n < 0) {
        std::fprintf(stderr, "ethercat_fleet_probe: bring-up failed\n");
        return 1;
    }
    if (static_cast<size_t>(n) != infos.size()) {
        std::fprintf(stderr,
            "ethercat_fleet_probe: expected %zu slaves, got %d\n",
            infos.size(), n);
        return 1;
    }

    // --- identity verification ---------------------------------------------
    int identityFails = 0;
    for (size_t i = 0; i < discovered.size() && i < infos.size(); ++i) {
        const auto& d = discovered[i];
        const auto& inf = infos[i];
        auto eq = [](const std::optional<uint32_t>& got, uint32_t want,
                     const char* what, size_t idx) {
            if (!got || *got != want) {
                std::fprintf(stderr,
                    "ethercat_fleet_probe: slave %zu %s mismatch "
                    "(got 0x%08x want 0x%08x)\n", idx, what,
                    got.value_or(0xFFFFFFFFu), want);
                return false;
            }
            return true;
        };
        if (!eq(d.vendor_id, inf.vendorId, "vendor", i) ||
            !eq(d.product_code, inf.productCode, "product", i) ||
            !eq(d.revision_number, beckhoff_ids::kRevision, "revision", i)) {
            ++identityFails;
        }
        std::printf("slave %2zu: %-22s vendor=0x%08x product=0x%08x rev=0x%08x\n",
                    i, inf.name.c_str(),
                    d.vendor_id.value_or(0), d.product_code.value_or(0),
                    d.revision_number.value_or(0));
    }
    if (identityFails) {
        std::fprintf(stderr, "ethercat_fleet_probe: %d identity check(s) failed\n",
                     identityFails);
        return 1;
    }

    // --- PDO entry map -------------------------------------------------------
    struct PdoLink {
        uint16_t slave;
        bool     txpdo;
        std::string key;
        uint16_t size;
        int      entry;
    };
    std::vector<PdoLink> links;
    for (const auto& inf : infos) {
        if (inf.isCoupler) continue;
        for (bool tx : {true, false}) {
            const uint16_t size = tx ? inf.txSize : inf.rxSize;
            if (!size) continue;
            const int e = findPdoEntry(fm, static_cast<uint16_t>(inf.chainIndex), tx);
            if (e < 0) {
                std::fprintf(stderr,
                    "ethercat_fleet_probe: no %s PDO entry for %s\n",
                    tx ? "Tx" : "Rx", inf.key.c_str());
                return 1;
            }
            links.push_back({static_cast<uint16_t>(inf.chainIndex), tx,
                             inf.key, size, e});
        }
    }
    std::printf("ethercat_fleet_probe: %zu PDO entries across %zu slaves\n",
                links.size(), infos.size());

    // --- probe mode ----------------------------------------------------------
    if (simdir.empty()) {
        uint32_t ok = 0;
        for (uint32_t c = 0; c < cycles && !g_stop; ++c) {
            if (fm.exchange()) ++ok;
        }
        std::printf("ethercat_fleet_probe: %u/%u exchanges ok\n", ok, cycles);
        if (ok != cycles) return 1;

        // Dump the TxPDO images (slave->master inputs; zeros until the
        // slave-side IO server is stimulated).
        for (const auto& l : links) {
            if (!l.txpdo) continue;
            const auto* e = fm.pdo().mapping().get_entry(l.entry);
            if (!e) continue;
            std::printf("  %-16s TxPDO[%u] %s\n", l.key.c_str(),
                        e->data_size, hexDump(e->storage, e->data_size).c_str());
        }
        std::printf("ethercat_fleet_probe: PASS\n");
        return 0;
    }

    // --- pump mode -----------------------------------------------------------
    std::filesystem::create_directories(simdir);
    const auto period = std::chrono::nanoseconds(1'000'000'000ULL / hz);
    const auto deadline = runSecs > 0
        ? std::chrono::steady_clock::now() +
              std::chrono::duration<double>(runSecs)
        : std::chrono::steady_clock::time_point::max();

    uint64_t cyclesDone = 0;
    auto next = std::chrono::steady_clock::now();
    while (!g_stop && std::chrono::steady_clock::now() < deadline) {
        // RxPDO (master->slave): file -> entry storage.
        for (const auto& l : links) {
            if (l.txpdo) continue;
            auto* e = fm.pdo().mapping().get_entry_mut(l.entry);
            if (!e) continue;
            readFile(std::filesystem::path(simdir) /
                         (l.key + ".rx"),
                     e->storage, e->data_size);
        }
        if (!fm.exchange()) {
            std::fprintf(stderr, "ethercat_fleet_probe: exchange failed\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        // TxPDO (slave->master): entry storage -> file.
        for (const auto& l : links) {
            if (!l.txpdo) continue;
            const auto* e = fm.pdo().mapping().get_entry(l.entry);
            if (!e) continue;
            writeFileAtomic(std::filesystem::path(simdir) /
                                (l.key + ".tx"),
                            e->storage, e->data_size);
        }
        ++cyclesDone;
        next += period;
        const auto now = std::chrono::steady_clock::now();
        if (next > now) std::this_thread::sleep_until(next);
        else next = now + period;
    }
    std::printf("ethercat_fleet_probe: pump ran %llu cycles\n",
                static_cast<unsigned long long>(cyclesDone));
    return 0;
}
