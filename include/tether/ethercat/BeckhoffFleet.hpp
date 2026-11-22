// SPDX-License-Identifier: MIT
/**
 * @file BeckhoffFleet.hpp
 * @brief Beckhoff EK/EL terminal fleet description — library data.
 *
 * Bus-domain catalog and topology language shared by masters
 * (tplc-ec-pump, ethercat_fleet_probe) and the emulated fleet
 * (examples/common/BeckhoffFleet.hpp):
 *
 *   - beckhoff product identities (vendor 0x00000002)
 *   - FleetSlaveInfo: one node's chain position, PDO byte layout
 *     (mirrors CiA401Slave's pdoLayout_), identity and endpoint key
 *   - fleetInfos(): the reference two-coupler fleet as plain data
 *   - detail::fleetInfosFromText/File: line-based fleet description —
 *     one node per line in EtherCAT chain order; `EK1100` opens a
 *     coupler segment, module lines attach to the current coupler.
 *     PDO endpoint keys auto-suffix (`el1014.c1`, `el1014.c1.2`, …)
 *   - detail::fleetTextFromScan/fleetModuleForProduct/
 *     fleetIsCouplerProduct: inverse mapping — discovered
 *     (vendor, product) identities back onto fleet text, for bus
 *     autodiscovery (`tplc-ec-pump --scan` / `--fleet auto`)
 *   - detail::fleetTextFromInfos: canonical re-emit of resolved
 *     FleetSlaveInfos (stable artifact for .fleet files)
 *
 * Header-only; no EtherCAT slave/master dependencies — pure data and
 * text parsing so any consumer (master, pump, IDE tooling) can use it.
 */
#pragma once

#include <cctype>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace tether::ethercat::beckhoff {

/// Real Beckhoff identities used by the fleet.
namespace ids {
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
} // namespace ids

/// Static description of one node on the bus/fleet.
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

/// The reference fleet topology as plain data (no slave objects):
/// two EK1100 couplers with EL1004/EL2004/EL3104/EL4104 behind each.
/// Masters and the emulated fleet share this so PDO file keys and
/// byte layouts stay in lockstep.
inline std::vector<FleetSlaveInfo> fleetInfos() {
    namespace b = ids;
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
    namespace b = ids;
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
            ci.vendorId    = ids::kVendor;
            ci.productCode = ids::kEK1100Product;
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
        mi.vendorId    = ids::kVendor;
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

/* ---- bus-scan helpers -------------------------------------------------
   Inverse of fleetInfosFromText: map discovered (vendor, product)
   identities back onto fleet-description lines so a master can build
   its topology from the bus itself (`tplc-ec-pump --scan` /
   `--fleet auto`) instead of a hand-written fleet file. */

/// One discovered slave's identity, reduced to what the catalog
/// resolves. `name` is the optional SII device name (display only).
struct FleetScanSlave {
    uint32_t    vendor  = 0;
    uint32_t    product = 0;
    std::string name;
};

struct FleetScanResult {
    std::string                 text;     ///< EK1100/ELxxxx lines
    std::vector<FleetScanSlave> unknown;  ///< unresolved identities
    size_t                      couplers = 0;
    size_t                      modules  = 0;
};

/// Catalog module whose Beckhoff product code matches @p product,
/// or nullptr (EK1100 and non-catalog parts never match).
inline const FleetModuleType* fleetModuleForProduct(uint32_t product) {
    size_t n = 0;
    const FleetModuleType* t = fleetModuleCatalog(n);
    for (size_t i = 0; i < n; ++i)
        if (t[i].product == product) return &t[i];
    return nullptr;
}

/// True when @p product is the EK1100 coupler identity.
inline bool fleetIsCouplerProduct(uint32_t product) {
    return product == ids::kEK1100Product;
}

/**
 * @brief Fleet-description text for a discovered slave chain.
 *
 * EK1100 slaves emit `EK1100` lines (opening coupler segments);
 * catalog modules emit their part-number line; any other identity is
 * appended to `unknown` and contributes no line — callers must treat
 * a non-empty `unknown` as "cannot auto-configure this bus".
 */
inline FleetScanResult fleetTextFromScan(
    const std::vector<FleetScanSlave>& slaves) {
    FleetScanResult r;
    for (const auto& s : slaves) {
        if (fleetIsCouplerProduct(s.product)) {
            r.text += "EK1100\n";
            ++r.couplers;
            continue;
        }
        if (const FleetModuleType* t = fleetModuleForProduct(s.product)) {
            r.text += t->name;
            r.text += '\n';
            ++r.modules;
            continue;
        }
        r.unknown.push_back(s);
    }
    return r;
}

/**
 * @brief Fleet-description text reconstructed from resolved
 *        FleetSlaveInfos (post-parse canonical form — safe to write
 *        back out as a fleet file or topology artifact).
 */
inline std::string fleetTextFromInfos(
    const std::vector<FleetSlaveInfo>& infos) {
    std::string out;
    for (const auto& i : infos) {
        if (i.isCoupler) {
            out += "EK1100\n";
            continue;
        }
        out += i.name.substr(0, i.name.find(" ("));
        out += '\n';
    }
    return out;
}

} // namespace detail

} // namespace tether::ethercat::beckhoff
