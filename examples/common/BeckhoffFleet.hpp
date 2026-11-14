// SPDX-License-Identifier: MIT
/**
 * @file BeckhoffFleet.hpp
 * @brief Emulated Beckhoff I/O fleet behind two EK1100 couplers.
 *
 * Builds a 10-node EtherCAT chain that mirrors a real Beckhoff segment:
 *
 *   EK1100 #1 -> EL1004 -> EL2004 -> EL3104 -> EL4104
 *   EK1100 #2 -> EL1004 -> EL2004 -> EL3104 -> EL4104
 *
 * Every node is a slave::CiA401Slave carrying the real Beckhoff vendor /
 * product identity (vendor 0x00000002).  The EK1100 couplers have no
 * process data of their own, matching the real part.  Each I/O module
 * gets the PDO layout CiA401Slave produces for its channel counts:
 *
 *   TxPDO (slave -> master): [digital inputs][analog inputs int16 LE]
 *   RxPDO (master -> slave): [digital outputs][analog outputs int16 LE]
 *
 * PDO buffers are kept coherent with the profile I/O arrays by the
 * per-tick updateTxPDO()/processRxPDO() calls in tick(), so the fleet
 * works with every master exchange mode (position, configured address,
 * or logical LRW via the PDO exchange callback).
 */
#pragma once

#include "slave/profiles/CiA401Slave.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace tether::examples {

/// Real Beckhoff identities used by the fleet.
namespace beckhoff_ids {
inline constexpr uint32_t kVendor = 0x00000002;
inline constexpr uint32_t kEK1100Product = 0x044C2C52;
inline constexpr uint32_t kEL1004Product = 0x03EC3052;
inline constexpr uint32_t kEL1014Product = 0x03F63052;
inline constexpr uint32_t kEL2004Product = 0x07D43052;
inline constexpr uint32_t kEL2008Product = 0x07D83052;
inline constexpr uint32_t kEL3104Product = 0x0C203052;
inline constexpr uint32_t kEL3114Product = 0x0C2A3052;
inline constexpr uint32_t kEL4104Product = 0x10083052;
inline constexpr uint32_t kEL4114Product = 0x10123052;
inline constexpr uint32_t kRevision      = 0x00010000;
} // namespace beckhoff_ids

/// Static description of one node on the emulated bus.
struct FleetSlaveInfo {
    size_t      chainIndex = 0;   ///< position on the EtherCAT chain
    bool        isCoupler = false;
    uint8_t     coupler = 0;      ///< which coupler this node sits on (1 or 2)
    std::string key;              ///< short id, e.g. "ek1100.c1", "el1004.c2"
    std::string name;             ///< e.g. "EK1100 #1", "EL1004"
    uint32_t    vendorId = 0;
    uint32_t    productCode = 0;

    // PDO byte layout (mirrors CiA401Slave's pdoLayout_).
    // tx = TxPDO slave->master, rx = RxPDO master->slave.
    uint16_t txSize = 0, rxSize = 0;
    uint16_t diOff = 0, diSize = 0;   ///< digital inputs inside TxPDO
    uint16_t aiOff = 0, aiSize = 0;   ///< analog inputs (int16) inside TxPDO
    uint16_t doOff = 0, doSize = 0;   ///< digital outputs inside RxPDO
    uint16_t aoOff = 0, aoSize = 0;   ///< analog outputs (int16) inside RxPDO
};

namespace detail {

inline EtherCAT::slave::CiA401SlaveConfig moduleConfig(
    const FleetSlaveInfo& inf, uint8_t di8, uint8_t do8,
    uint8_t ai, uint8_t ao, const char* devName, uint32_t serial)
{
    EtherCAT::slave::CiA401SlaveConfig c;
    c.identity.vendorId      = inf.vendorId;
    c.identity.productCode   = inf.productCode;
    c.identity.revisionNumber = beckhoff_ids::kRevision;
    c.identity.serialNumber  = serial;
    c.identity.deviceName    = devName;
    c.digitalInputs8   = di8;
    c.digitalOutputs8  = do8;
    c.analogInputs     = ai;
    c.analogOutputs    = ao;
    c.supportsDC       = true;
    c.defaultCycleTime = 1000000;  // 1 ms
    return c;
}

} // namespace detail

/// The fixed fleet topology as plain data (no slave objects).  The
/// BeckhoffFleet ctor and the master-side probe/pump share this so
/// PDO file keys and byte layouts stay in lockstep.
inline std::vector<FleetSlaveInfo> fleetInfos() {
    namespace b = beckhoff_ids;
    struct ModuleSpec {
        const char* key;      // short id suffix, e.g. "el1004.c1"
        const char* name;     // Beckhoff part name
        uint32_t    product;
        uint8_t     di8, do8, ai, ao;
    };
    static const ModuleSpec kModules[] = {
        {"el1004", "EL1004", b::kEL1004Product, 1, 0, 0, 0},
        {"el2004", "EL2004", b::kEL2004Product, 0, 1, 0, 0},
        {"el3104", "EL3104", b::kEL3104Product, 0, 0, 4, 0},
        {"el4104", "EL4104", b::kEL4104Product, 0, 0, 0, 4},
    };

    std::vector<FleetSlaveInfo> v;
    for (uint8_t c = 1; c <= 2; ++c) {
        FleetSlaveInfo ci;
        ci.chainIndex  = v.size();
        ci.isCoupler   = true;
        ci.coupler     = c;
        ci.key         = "ek1100.c" + std::to_string(c);
        ci.name        = "EK1100 #" + std::to_string(c);
        ci.vendorId    = b::kVendor;
        ci.productCode = b::kEK1100Product;
        v.push_back(std::move(ci));

        for (const auto& m : kModules) {
            FleetSlaveInfo mi;
            mi.chainIndex  = v.size();
            mi.isCoupler   = false;
            mi.coupler     = c;
            mi.key         = std::string(m.key) + ".c" + std::to_string(c);
            mi.name        = std::string(m.name) + " (EK1100 #" +
                             std::to_string(c) + ")";
            mi.vendorId    = b::kVendor;
            mi.productCode = m.product;
            mi.diSize      = m.di8;
            mi.doSize      = m.do8;
            mi.aiSize      = static_cast<uint16_t>(m.ai * 2);
            mi.aoSize      = static_cast<uint16_t>(m.ao * 2);
            mi.txSize      = static_cast<uint16_t>(mi.diSize + mi.aiSize);
            mi.rxSize      = static_cast<uint16_t>(mi.doSize + mi.aoSize);
            mi.diOff       = 0;
            mi.aiOff       = mi.diSize;
            mi.doOff       = 0;
            mi.aoOff       = mi.doSize;
            v.push_back(std::move(mi));
        }
    }
    return v;
}

namespace detail {

/// Beckhoff module type → PDO channel layout.  di/do are whole-byte
/// counts inside the PDO image; ai/ao are int16 channel counts —
/// mirrors what CiA401Slave produces for the same counts.
struct FleetModuleType {
    const char* name;      ///< part number, e.g. "EL1014"
    uint32_t    product;   ///< Beckhoff product code
    uint8_t     di8, do8;  ///< digital in/out bytes
    uint8_t     ai, ao;    ///< analog in/out channels (int16 each)
};

/// The module catalog shared by fleetInfosFromText and the WebUI's
/// terminal presets — keep channel counts/PDO sizes in lockstep.
inline const FleetModuleType* fleetModuleCatalog(size_t& n) {
    namespace b = beckhoff_ids;
    static const FleetModuleType kTypes[] = {
        {"EL1004", b::kEL1004Product, 1, 0, 0, 0},
        {"EL1014", b::kEL1014Product, 1, 0, 0, 0},
        {"EL2004", b::kEL2004Product, 0, 1, 0, 0},
        {"EL2008", b::kEL2008Product, 0, 1, 0, 0},   // 8 DO → 1 byte
        {"EL3104", b::kEL3104Product, 0, 0, 4, 0},
        {"EL3114", b::kEL3114Product, 0, 0, 4, 0},
        {"EL4104", b::kEL4104Product, 0, 0, 0, 4},
        {"EL4114", b::kEL4114Product, 0, 0, 0, 4},
    };
    n = sizeof(kTypes) / sizeof(kTypes[0]);
    return kTypes;
}

inline const FleetModuleType* fleetModuleType(const std::string& name) {
    size_t n = 0;
    const FleetModuleType* t = fleetModuleCatalog(n);
    for (size_t i = 0; i < n; ++i)
        if (name == t[i].name) return &t[i];
    return nullptr;
}

inline std::string lowerStr(std::string s) {
    for (auto& c : s)
        c = (char)std::tolower((unsigned char)c);
    return s;
}

/**
 * @brief Fleet topology from a line-based description — one node per
 *        line in EtherCAT chain order; `EK1100` opens a new coupler
 *        segment, module lines attach to the current coupler.
 *
 *   EK1100
 *   EL1014
 *   EL1014
 *   EL2004
 *   EL3104
 *   EK1100
 *   EL1014
 *
 * PDO endpoint keys: the first module of a type on a coupler is
 * `<type>.c<N>` (e.g. `el1014.c1`); repeats get `.2`, `.3`, … so "a
 * bunch of EL1014" stays addressable.  `err` receives a message on
 * failure (unknown type, module before any coupler).
 */
inline std::vector<FleetSlaveInfo>
fleetInfosFromText(const std::string& text, std::string* err = nullptr) {
    auto fail = [&](const std::string& m) {
        if (err) *err = m;
        return std::vector<FleetSlaveInfo>{};
    };
    std::vector<FleetSlaveInfo> v;
    uint8_t coupler = 0;
    std::map<std::string, int> seen;   // "type.coupler" → occurrence
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        size_t h = line.find('#');
        if (h != std::string::npos) line.resize(h);
        std::istringstream ls(line);
        std::string word;
        if (!(ls >> word)) continue;
        word = lowerStr(word);
        if (word == "ek1100") {
            ++coupler;
            FleetSlaveInfo ci;
            ci.chainIndex  = v.size();
            ci.isCoupler   = true;
            ci.coupler     = coupler;
            ci.key         = "ek1100.c" + std::to_string(coupler);
            ci.name        = "EK1100 #" + std::to_string(coupler);
            ci.vendorId    = beckhoff_ids::kVendor;
            ci.productCode = beckhoff_ids::kEK1100Product;
            v.push_back(std::move(ci));
            continue;
        }
        if (coupler == 0)
            return fail("fleet: module '" + word +
                        "' before any EK1100 coupler");
        std::string upper = word;
        for (auto& c : upper) c = (char)std::toupper((unsigned char)c);
        const FleetModuleType* t = fleetModuleType(upper);
        if (!t)
            return fail("fleet: unknown module type '" + word + "'");
        std::string base = std::string(t->name) + ".c" +
                           std::to_string(coupler);
        std::string baseLower = lowerStr(base);
        int occ = ++seen[baseLower];
        FleetSlaveInfo mi;
        mi.chainIndex  = v.size();
        mi.isCoupler   = false;
        mi.coupler     = coupler;
        mi.key         = occ == 1 ? baseLower : baseLower + "." +
                                  std::to_string(occ);
        mi.name        = std::string(t->name) + " (EK1100 #" +
                         std::to_string(coupler) + ")";
        mi.vendorId    = beckhoff_ids::kVendor;
        mi.productCode = t->product;
        mi.diSize      = t->di8;
        mi.doSize      = t->do8;
        mi.aiSize      = static_cast<uint16_t>(t->ai * 2);
        mi.aoSize      = static_cast<uint16_t>(t->ao * 2);
        mi.txSize      = static_cast<uint16_t>(mi.diSize + mi.aiSize);
        mi.rxSize      = static_cast<uint16_t>(mi.doSize + mi.aoSize);
        mi.diOff       = 0;
        mi.aiOff       = mi.diSize;
        mi.doOff       = 0;
        mi.aoOff       = mi.doSize;
        v.push_back(std::move(mi));
    }
    if (v.empty()) return fail("fleet: empty description");
    return v;
}

inline std::vector<FleetSlaveInfo>
fleetInfosFromFile(const std::string& path, std::string* err = nullptr) {
    std::ifstream f(path);
    if (!f) {
        if (err) *err = "fleet: cannot open " + path;
        return {};
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return fleetInfosFromText(ss.str(), err);
}

} // namespace detail

/**
 * @brief The emulated bus: two EK1100 couplers with four I/O modules
 *        behind each, chained in EtherCAT order.
 *
 * Usage:
 * @code
 *   BeckhoffFleet fleet;
 *   fleet.start();
 *   // per received frame:
 *   auto resp = fleet.processFrame(frame, len);
 *   // periodically (e.g. 1 kHz):
 *   fleet.tick(1'000'000);
 * @endcode
 */
class BeckhoffFleet {
public:
    /// Default two-coupler fleet (fleetInfos()).
    BeckhoffFleet() : BeckhoffFleet(fleetInfos()) {}

    /// Custom topology — e.g. detail::fleetInfosFromText/File output
    /// with a bunch of EL1014s behind one coupler.
    explicit BeckhoffFleet(std::vector<FleetSlaveInfo> infos)
        : infos_(std::move(infos)) {
        buildTopology();
        for (auto& s : slaves_) s->start();
        wireMailboxAndPdo();
    }

    ~BeckhoffFleet() { stop(); }

    BeckhoffFleet(const BeckhoffFleet&) = delete;
    BeckhoffFleet& operator=(const BeckhoffFleet&) = delete;

    /// Number of EtherCAT nodes (always 10 for the default topology).
    size_t slaveCount() const { return slaves_.size(); }

    /// Node at chain position @p i (0 = first slave after the master).
    EtherCAT::slave::CiA401Slave& slave(size_t i) { return *slaves_.at(i); }
    const EtherCAT::slave::CiA401Slave& slave(size_t i) const { return *slaves_.at(i); }

    const FleetSlaveInfo& info(size_t i) const { return infos_.at(i); }
    const std::vector<FleetSlaveInfo>& infos() const { return infos_; }

    /// Run a received Ethernet frame through every slave in chain order.
    /// Returns the response frame (empty for malformed input).
    std::vector<uint8_t> processFrame(const uint8_t* frame, size_t len) {
        std::vector<uint8_t> f(frame, frame + len);
        for (auto& s : slaves_) {
            f = s->getCore().processFrame(f.data(), f.size());
            if (f.empty()) return {};
        }
        return f;
    }

    /// Advance every slave: core simulate (mailbox/DC/watchdog), profile
    /// simulate, and PDO buffer sync.
    void tick(uint64_t deltaNs) {
        for (size_t i = 0; i < slaves_.size(); ++i) {
            slaves_[i]->simulate(deltaNs);
            slaves_[i]->getCore().simulate(deltaNs);
            slaves_[i]->updateTxPDO();
            slaves_[i]->processRxPDO();
        }
    }

    void start() {
        for (auto& s : slaves_) s->getCore().start();
    }

    void stop() {
        for (auto& s : slaves_) s->getCore().stop();
    }

private:
    void buildTopology() {
        for (const auto& inf : infos_) {
            const uint8_t di8 = static_cast<uint8_t>(inf.diSize);
            const uint8_t do8 = static_cast<uint8_t>(inf.doSize);
            const uint8_t ai  = static_cast<uint8_t>(inf.aiSize / 2);
            const uint8_t ao  = static_cast<uint8_t>(inf.aoSize / 2);
            const std::string dev = inf.name.substr(0, inf.name.find(" ("));
            auto cfg = detail::moduleConfig(inf, di8, do8, ai, ao,
                                             dev.c_str(),
                                             static_cast<uint32_t>(slaves_.size() + 1));
            slaves_.push_back(
                std::make_unique<EtherCAT::slave::CiA401Slave>(cfg));
        }
    }

    void wireMailboxAndPdo() {
        for (auto& s : slaves_) {
            auto& core = s->getCore();
            // CoE mailbox: share the profile's object dictionary with the
            // core through a non-owning alias (the profile owns it).
            auto& od = s->getObjectDictionary();
            std::shared_ptr<EtherCAT::slave::IObjectDictionary> odsp(
                &od, [](EtherCAT::slave::IObjectDictionary*) {});
            core.setObjectDictionary(odsp);
            core.addMailboxHandler(EtherCAT::slave::createCoEHandler(odsp));
            // Immediate PDO refresh on logical (LRD/LWR/LRW) exchanges;
            // tick() covers position/configured-address exchanges.
            auto* raw = s.get();
            core.setPDOExchangeCallback([raw]() {
                raw->updateTxPDO();
                raw->processRxPDO();
            });
        }
    }

    std::vector<std::unique_ptr<EtherCAT::slave::CiA401Slave>> slaves_;
    std::vector<FleetSlaveInfo> infos_;
};

} // namespace tether::examples
