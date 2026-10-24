/**
 * @file NamedSdoExposer.hpp
 * @brief Exposes object-dictionary SDO entries under human-readable names
 *        slugified from the configured register names.
 *
 * Complements index-addressed SDO catalogs such as AS715NExposer's
 * `drive<N>.sdo.<INDEX>_<SUB>` parameters: every object-dictionary entry is
 * additionally registered as `<prefix>.sdo.<slug>` where `<slug>` is derived
 * from the entry's `.name` — lowercased, non-alphanumeric runs folded to '_'
 * ("1st speed loop gain" -> "1st_speed_loop_gain",
 * "Identity: vendor ID" -> "identity_vendor_id").
 *
 * The exposer is drive-agnostic: it works over generic
 * EtherCAT::Drives::Registers::RegisterList collections, so any device whose
 * object dictionary is described as ObjectDictionaryEntry lists can use it.
 *
 * Characteristics:
 *  - Slug collisions (same `.name` in different groups, e.g. F30 vs F31)
 *    resolve deterministically by appending `_<INDEX>_<SUB>` hex; literal
 *    duplicates (same index AND subindex in the lists twice) are skipped.
 *  - Entries without a name fall back to `obj_<INDEX>_<SUB>`.
 *  - Metadata matches the index-named exposer (index/subindex/od_type/
 *    default/min/max/unit/modification/effective/options_enum) plus
 *    `sdo_name` pointing at the canonical `<prefix>.sdo.<INDEX>_<SUB>` name.
 *  - Local ids are tagged 0x02 (vs 0x01 for the index-named scheme), so a
 *    NamedSdoExposer can share its idBase with an AS715NExposer for the same
 *    drive without id collisions.
 *  - All entries carry EntryFlags::NoStream — poll via GetParam.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/io/exposers/detail/OdSdoParam.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/drives/AS715N/Registers/Common.hpp"

#include <cstdint>
#include <format>
#include <string>
#include <unordered_set>
#include <utility>

namespace tether { namespace io { namespace exposers {

/**
 * @class NamedSdoExposer
 * @brief Registers SDO-backed parameters under slugified register names.
 *
 * Example (AS715N axis, exposing the same catalog AS715NExposer uses but
 * with human-readable aliases):
 *
 * @code
 *   NamedSdoExposer named(master.sdoManager(slaveIdx),
 *                         AS715NExposer<>::sdoCatalog(),
 *                         "drive" + std::to_string(slaveIdx), "as715n");
 *   named.expose(registry, idBase);   // same idBase as AS715NExposer is fine
 *   // "1st speed loop gain" is now readable as
 *   //   get drive0.sdo.1st_speed_loop_gain
 * @endcode
 */
class NamedSdoExposer : public IParameterExposer {
public:
    /// Local-id tag byte for named entries (index-named entries use 0x01).
    static constexpr uint8_t kLocalIdTag = 0x02;

    /**
     * @param coe        Queue-serialized CoE manager of the slave; SDO access
     *                   runs on the IO session thread.  Must outlive the
     *                   registered entries.
     * @param groups     Register groups to expose (e.g.
     *                   AS715NExposer<>::sdoCatalog()).
     * @param prefix     Name prefix for entries, e.g. "drive0" produces
     *                   "drive0.sdo.<slug>".
     * @param moduleName Module name for moduleName() and the leading group
     *                   segment, e.g. "as715n" produces the group
     *                   "as715n.drive0.sdo" — matching AS715NExposer.
     */
    NamedSdoExposer(::EtherCAT::CoE::CoEManager& coe,
                    ::EtherCAT::Drives::Registers::RegisterListOfLists groups,
                    std::string prefix,
                    std::string moduleName = "sdo")
        : coe_(coe)
        , groups_(std::move(groups))
        , prefix_(std::move(prefix))
        , module_(std::move(moduleName)) {}

    /// Append another register group after construction.  The list must
    /// outlive the exposer (pass the address of a static kRegisterList).
    void addRegisterList(
        const ::EtherCAT::Drives::Registers::RegisterList* list) {
        if (list) groups_.push_back(list);
    }

    const char* moduleName() const override { return module_.c_str(); }

    /// Parameter id of the named alias for a given object — shared idBase
    /// safe: tag 0x02 never collides with the 0x01-tagged index scheme.
    static constexpr uint64_t namedId(uint64_t idBase, uint16_t index,
                                      uint8_t sub) {
        return makeId(idBase, (static_cast<uint32_t>(kLocalIdTag) << 24) |
                              (static_cast<uint32_t>(index) << 8) | sub);
    }

    void expose(Registry& registry, uint64_t idBase) override {
        const std::string group = module_ + "." + prefix_ + ".sdo";
        std::unordered_set<std::string> used;
        for (const auto* list : groups_) {
            if (!list) continue;
            for (const auto* entry : *list) {
                if (!entry) continue;
                exposeEntry(registry, idBase, group, *entry, used);
            }
        }
    }

private:
    void exposeEntry(Registry& registry, uint64_t idBase,
                     const std::string& group,
                     const ::EtherCAT::ObjectDictionary::ObjectDictionaryEntry& e,
                     std::unordered_set<std::string>& used) {
        std::string slug = detail::slugifyName(e.name ? e.name : "");
        if (slug.empty())
            slug = std::format("obj_{:04X}_{:02X}", e.index, e.subindex);

        // First collision gets the unambiguous index/subindex suffix; a
        // second means the exact same object appeared twice — skip it.
        std::string leaf = slug;
        if (!used.insert(leaf).second) {
            leaf = std::format("{}_{:04X}_{:02X}", slug, e.index, e.subindex);
            if (!used.insert(leaf).second) return;
        }

        ParamEntry p = detail::makeSdoParam(
            coe_, e, namedId(idBase, e.index, e.subindex),
            prefix_ + ".sdo." + leaf, group);
        // Cross-reference the canonical index-addressed name so clients can
        // show/resolve both spellings of the same object.
        p.metadata["sdo_name"] =
            std::format("{}.sdo.{:04X}_{:02X}", prefix_, e.index, e.subindex);
        registry.addParam(std::move(p));
    }

    ::EtherCAT::CoE::CoEManager& coe_;
    ::EtherCAT::Drives::Registers::RegisterListOfLists groups_;
    std::string prefix_;
    std::string module_;
};

}}} // namespace tether::io::exposers
