/**
 * @file AS715NExposer.hpp
 * @brief Exposes a Metexon AS715N drive's full PDO image and complete
 *        machine-readable SDO catalog to the IO protocol.
 *
 * What this exposes (per drive, named "drive<N>.*"):
 *
 *  - `pdo.rx_*` / `pdo.tx_*` — every field of RxPDO 0x1704 and TxPDO 0x1B04
 *    as fixed-size signals, read through an application-provided IPdoAccess
 *    (which applies whatever locking guards the PDO buffers).
 *  - `pdo.rx_image` / `pdo.tx_image` — the raw packed PDO images as Binary
 *    signals for clients that want the whole frame.
 *  - `sdo.<index>_<sub>` — every entry from the AS715N register headers'
 *    kRegisterList collections as parameters.  Readable via GetParam;
 *    writable unless the metadata marks the entry ReadOnly.  SDO access goes
 *    through the queue-serialized CoEManager, so reads/writes run on the IO
 *    session thread and are safe to use from SetParam.  All SDO entries carry
 *    EntryFlags::NoStream: an SDO mailbox round-trip per stream row would
 *    saturate the mailbox — poll them with GetParam instead.
 *  - `ringSource()` — an SpscRingStreamSource covering all pdo.* signal IDs.
 *    Hand it to ServerConfig::ringSources and call produceRow() once per
 *    realtime cycle; any client stream configured on (a subset of) the PDO
 *    signals then receives ring-buffered, producer-timestamped rows instead
 *    of session-thread-poll samples.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/ParameterExposer.hpp"
#include "tether/io/RingStreamSource.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"

#include "tether/drives/AS715N/Registers/C00-Parameters.hpp"
#include "tether/drives/AS715N/Registers/C01-BasicGainParameters.hpp"
#include "tether/drives/AS715N/Registers/C02-AdvancedGainParameters.hpp"
#include "tether/drives/AS715N/Registers/C03-InstructionParameters.hpp"
#include "tether/drives/AS715N/Registers/C04-IOParameters.hpp"
#include "tether/drives/AS715N/Registers/C05-StopMode.hpp"
#include "tether/drives/AS715N/Registers/C06-ProtectionParameters.hpp"
#include "tether/drives/AS715N/Registers/C07-AutoTuningParameters.hpp"
#include "tether/drives/AS715N/Registers/C0A-CommunicationParameters.hpp"
#include "tether/drives/AS715N/Registers/C10-RotationHomingParameters.hpp"
#include "tether/drives/AS715N/Registers/C13-EtherCATParameters.hpp"
#include "tether/drives/AS715N/Registers/F30-ControlInProgress.hpp"
#include "tether/drives/AS715N/Registers/F31-ControlInProgress.hpp"
#include "tether/drives/AS715N/Registers/R20-MotorParameters.hpp"
#include "tether/drives/AS715N/Registers/R22-Motor-Gain-Parameters.hpp"
#include "tether/drives/AS715N/Registers/U40-RunningMonitoringParameters.hpp"
#include "tether/drives/AS715N/Registers/U41-StatusMonitoringParameters.hpp"
#include "tether/drives/AS715N/Registers/U42-VersionParameters.hpp"

#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace tether { namespace io { namespace exposers {

namespace AS715N_pdo = ::EtherCAT::Drives::AS715N_pdo;
namespace RegAS715N  = ::EtherCAT::Drives::Registers::AS715N;
namespace CoENS      = ::EtherCAT::CoE;
namespace OD         = ::EtherCAT::ObjectDictionary;

/**
 * @brief Producer-side PDO image access, implemented by the application.
 *
 * The implementation must serialize against the realtime path that owns the
 * PDO buffers (e.g. the bridge's state mutex) and must tolerate the drive
 * being absent (EtherCAT recovery, shutdown): return false in that case and
 * the exposer reports zeros.
 */
class IAS715NPdoAccess {
public:
    virtual ~IAS715NPdoAccess() = default;
    virtual bool readRxPDO1704(AS715N_pdo::AS715N_RxPDO_1704& out) = 0;
    virtual bool readTxPDO1B04(AS715N_pdo::AS715N_TxPDO_1B04& out) = 0;
};

/**
 * @brief Packed stream row: producer timestamp + RxPDO 0x1704 + TxPDO 0x1B04.
 *
 * Field order matches kPdoFields below.  Producers fill rows via
 * AS715NExposer::produceRow() — typically once per cyclic/PDO update.
 */
struct AS715NPdoRow {
    uint64_t ts_us;
    // RxPDO 0x1704 (master -> slave)
    uint16_t rx_controlword;
    int32_t  rx_target_position;
    int32_t  rx_target_velocity;
    int16_t  rx_target_torque;
    int8_t   rx_modes_of_operation;
    uint16_t rx_touch_probe_function;
    uint32_t rx_max_profile_velocity;
    uint16_t rx_positive_torque_limit;
    uint16_t rx_negative_torque_limit;
    // TxPDO 0x1B04 (slave -> master)
    uint16_t tx_error_code;
    uint16_t tx_statusword;
    int32_t  tx_position_actual;
    int16_t  tx_torque_actual;
    int8_t   tx_modes_of_operation_display;
    int32_t  tx_position_deviation;
    uint16_t tx_touch_probe_status;
    int32_t  tx_touch_probe_pos1;
    int32_t  tx_touch_probe_pos2;
    int32_t  tx_speed_feedback;
} __attribute__((packed));

namespace detail {

/// One field of the PDO stream schema / signal set.
struct AS715NPdoField {
    const char* name;      ///< Signal name suffix ("rx_controlword")
    const char* desc;      ///< Object index + CiA 402 name
    ValueType   type;
    uint8_t     size;
    bool        rx;        ///< true: RxPDO image, false: TxPDO image
    uint16_t    offset;    ///< offsetof() in the respective PDO struct
};

using RxPDO1704 = AS715N_pdo::AS715N_RxPDO_1704;
using TxPDO1B04 = AS715N_pdo::AS715N_TxPDO_1B04;

inline constexpr AS715NPdoField kPdoFields[] = {
    {"rx_controlword",                "RxPDO 0x6040 Controlword",                 ValueType::U16, sizeof(uint16_t), true,  offsetof(RxPDO1704, controlword)},
    {"rx_target_position",            "RxPDO 0x607A Target Position",             ValueType::I32, sizeof(int32_t),  true,  offsetof(RxPDO1704, target_position)},
    {"rx_target_velocity",            "RxPDO 0x60FF Target Velocity",             ValueType::I32, sizeof(int32_t),  true,  offsetof(RxPDO1704, target_velocity)},
    {"rx_target_torque",              "RxPDO 0x6071 Target Torque",               ValueType::I16, sizeof(int16_t),  true,  offsetof(RxPDO1704, target_torque)},
    {"rx_modes_of_operation",         "RxPDO 0x6060 Modes of Operation",          ValueType::I8,  sizeof(int8_t),   true,  offsetof(RxPDO1704, modes_of_operation)},
    {"rx_touch_probe_function",       "RxPDO 0x60B8 Touch Probe Function",        ValueType::U16, sizeof(uint16_t), true,  offsetof(RxPDO1704, touch_probe_function)},
    {"rx_max_profile_velocity",       "RxPDO 0x607F Max Profile Velocity",        ValueType::U32, sizeof(uint32_t), true,  offsetof(RxPDO1704, max_profile_velocity)},
    {"rx_positive_torque_limit",      "RxPDO 0x60E0 Positive Torque Limit",       ValueType::U16, sizeof(uint16_t), true,  offsetof(RxPDO1704, positive_torque_limit)},
    {"rx_negative_torque_limit",      "RxPDO 0x60E1 Negative Torque Limit",       ValueType::U16, sizeof(uint16_t), true,  offsetof(RxPDO1704, negative_torque_limit)},
    {"tx_error_code",                 "TxPDO 0x603F Error Code",                  ValueType::U16, sizeof(uint16_t), false, offsetof(TxPDO1B04, error_code)},
    {"tx_statusword",                 "TxPDO 0x6041 Statusword",                  ValueType::U16, sizeof(uint16_t), false, offsetof(TxPDO1B04, statusword)},
    {"tx_position_actual",            "TxPDO 0x6064 Position Actual Value",       ValueType::I32, sizeof(int32_t),  false, offsetof(TxPDO1B04, position_actual)},
    {"tx_torque_actual",              "TxPDO 0x6077 Torque Actual Value",         ValueType::I16, sizeof(int16_t),  false, offsetof(TxPDO1B04, torque_actual)},
    {"tx_modes_of_operation_display", "TxPDO 0x6061 Modes of Operation Display",  ValueType::I8,  sizeof(int8_t),   false, offsetof(TxPDO1B04, modes_of_operation_display)},
    {"tx_position_deviation",         "TxPDO 0x60F4 Following Error",             ValueType::I32, sizeof(int32_t),  false, offsetof(TxPDO1B04, position_deviation)},
    {"tx_touch_probe_status",         "TxPDO 0x60B9 Touch Probe Status",          ValueType::U16, sizeof(uint16_t), false, offsetof(TxPDO1B04, touch_probe_status)},
    {"tx_touch_probe_pos1",           "TxPDO 0x60BA Touch Probe Pos1",            ValueType::I32, sizeof(int32_t),  false, offsetof(TxPDO1B04, touch_probe_pos1)},
    {"tx_touch_probe_pos2",           "TxPDO 0x60BC Touch Probe Pos2",            ValueType::I32, sizeof(int32_t),  false, offsetof(TxPDO1B04, touch_probe_pos2)},
    {"tx_speed_feedback",             "TxPDO 0x606C Velocity Actual Value",       ValueType::I32, sizeof(int32_t),  false, offsetof(TxPDO1B04, speed_feedback)},
};

inline constexpr size_t kPdoFieldCount =
    sizeof(kPdoFields) / sizeof(kPdoFields[0]);

/// Map an object-dictionary data type to an IO value type.
/// Returns false for types with no meaningful scalar representation.
inline bool odToValueType(OD::ObjectDictionaryDataType t, ValueType& out) {
    using T = OD::ObjectDictionaryDataType;
    switch (t) {
        case T::Boolean:       out = ValueType::Bool;   return true;
        case T::Integer8:      out = ValueType::I8;     return true;
        case T::Integer16:     out = ValueType::I16;    return true;
        case T::Integer32:     out = ValueType::I32;    return true;
        case T::Integer64:     out = ValueType::I64;    return true;
        case T::Unsigned8:     out = ValueType::U8;     return true;
        case T::Unsigned16:    out = ValueType::U16;    return true;
        case T::Unsigned32:    out = ValueType::U32;    return true;
        case T::Unsigned64:    out = ValueType::U64;    return true;
        case T::Real32:        out = ValueType::F32;    return true;
        case T::Real64:        out = ValueType::F64;    return true;
        case T::VisibleString:
        case T::UnicodeString: out = ValueType::String; return true;
        default:               out = ValueType::Binary; return true;
    }
}

/// Fixed wire size for scalar OD types; 0 for variable-length / odd types.
inline uint8_t odFixedSize(OD::ObjectDictionaryDataType t) {
    using T = OD::ObjectDictionaryDataType;
    switch (t) {
        case T::Boolean:
        case T::Integer8:
        case T::Unsigned8:    return 1;
        case T::Integer16:
        case T::Unsigned16:   return 2;
        case T::Integer24:
        case T::Unsigned24:   return 3;
        case T::Integer32:
        case T::Unsigned32:
        case T::Real32:       return 4;
        case T::Integer40:
        case T::Unsigned40:   return 5;
        case T::Integer48:
        case T::Unsigned48:   return 6;
        case T::Integer56:
        case T::Unsigned56:   return 7;
        case T::Integer64:
        case T::Unsigned64:
        case T::Real64:
        case T::TimeOfDay:
        case T::TimeDifference: return 8;
        default:              return 0;
    }
}

inline const char* modificationModeName(OD::ModificationMode m) {
    switch (m) {
        case OD::ModificationMode::ReadOnly:        return "readonly";
        case OD::ModificationMode::AtStop:          return "at_stop";
        case OD::ModificationMode::DuringOperation: return "during_operation";
        default:                                    return "unknown";
    }
}

inline const char* effectiveTimeName(OD::EffectiveTime e) {
    switch (e) {
        case OD::EffectiveTime::Immediately:    return "immediately";
        case OD::EffectiveTime::UponRepowerOn:  return "upon_repower_on";
        default:                                return "unknown";
    }
}

/// SDO transaction timeout for IO-triggered reads/writes (session thread).
inline constexpr uint32_t kSdoTimeoutMs = 500;

} // namespace detail

/**
 * @class AS715NExposer
 * @brief Exposes AS715N PDO signals, PDO raw images, and the full SDO
 *        register catalog; owns the ring stream source for PDO telemetry.
 *
 * @tparam RingCapacity  PDO row ring depth.  16384 covers ~16 s at 1 kHz;
 *                       1 gives best-effort latest-sample streaming.
 */
template <size_t RingCapacity = 16384>
class AS715NExposer : public IParameterExposer {
public:
    /**
     * @param pdo        PDO image access (locks PDO buffers; must outlive
     *                   this exposer and tolerate drive absence).
     * @param coe        Queue-serialized CoE manager for this slave.
     * @param driveIndex Index for naming, e.g. "drive0".
     */
    AS715NExposer(IAS715NPdoAccess& pdo, CoENS::CoEManager& coe,
                  uint16_t driveIndex = 0)
        : pdo_(pdo), coe_(coe), driveIndex_(driveIndex) {}

    const char* moduleName() const override { return "as715n"; }

    /// Ring stream source covering all pdo.* signals — register into
    /// ServerConfig::ringSources after calling expose().
    IRingStreamSource& ringSource() { return ringSource_; }

    /**
     * Producer side — call once per PDO update cycle from the realtime
     * thread.  Cheap when no session is streaming (single atomic load);
     * never blocks; drops (counted) when the ring is full.
     */
    template <typename F>
    bool produceRow(F&& fill) { return ringSource_.produce(std::forward<F>(fill)); }

    uint64_t ringDropped() const { return ringSource_.dropped(); }

    void expose(Registry& registry, uint64_t idBase) override {
        prefix_ = "drive" + std::to_string(driveIndex_);
        const std::string group = "as715n." + prefix_;

        exposePdoSignals(registry, idBase, group);
        exposeSdoCatalog(registry, idBase, group);
    }

private:
    // ---- PDO signals + ring schema ----

    void exposePdoSignals(Registry& registry, uint64_t idBase,
                          const std::string& group) {
        std::vector<uint64_t> ids;
        std::vector<uint16_t> sizes;
        ids.reserve(detail::kPdoFieldCount);
        sizes.reserve(detail::kPdoFieldCount);

        for (size_t i = 0; i < detail::kPdoFieldCount; ++i) {
            const auto& f = detail::kPdoFields[i];
            const uint64_t id = makeId(idBase, static_cast<uint32_t>(i) + 1);
            ids.push_back(id);
            sizes.push_back(f.size);
            registry.addSignal({
                id, prefix_ + ".pdo." + f.name, f.desc, group + ".pdo",
                f.type,
                [this, &f](void* d) { readPdoField(f, d); },
                nullptr, 0, {}
            });
        }

        // Full raw PDO images (variable-length Binary signals).
        registry.addSignal({
            makeId(idBase, 0x0100), prefix_ + ".pdo.rx_image",
            "RxPDO 0x1704 raw image (23 bytes)", group + ".pdo",
            ValueType::Binary, nullptr,
            [this](void* d, size_t maxLen) -> size_t {
                AS715N_pdo::AS715N_RxPDO_1704 img{};
                if (maxLen < sizeof(img) || !pdo_.readRxPDO1704(img)) return 0;
                std::memcpy(d, &img, sizeof(img));
                return sizeof(img);
            },
            sizeof(AS715N_pdo::AS715N_RxPDO_1704), {}
        });
        registry.addSignal({
            makeId(idBase, 0x0101), prefix_ + ".pdo.tx_image",
            "TxPDO 0x1B04 raw image (24 bytes)", group + ".pdo",
            ValueType::Binary, nullptr,
            [this](void* d, size_t maxLen) -> size_t {
                AS715N_pdo::AS715N_TxPDO_1B04 img{};
                if (maxLen < sizeof(img) || !pdo_.readTxPDO1B04(img)) return 0;
                std::memcpy(d, &img, sizeof(img));
                return sizeof(img);
            },
            sizeof(AS715N_pdo::AS715N_TxPDO_1B04), {}
        });

        ringSource_.setSchema(std::move(ids), std::move(sizes));
    }

    void readPdoField(const detail::AS715NPdoField& f, void* d) {
        if (f.rx) {
            AS715N_pdo::AS715N_RxPDO_1704 img{};
            if (pdo_.readRxPDO1704(img))
                std::memcpy(d, reinterpret_cast<const uint8_t*>(&img) + f.offset,
                            f.size);
            else
                std::memset(d, 0, f.size);
        } else {
            AS715N_pdo::AS715N_TxPDO_1B04 img{};
            if (pdo_.readTxPDO1B04(img))
                std::memcpy(d, reinterpret_cast<const uint8_t*>(&img) + f.offset,
                            f.size);
            else
                std::memset(d, 0, f.size);
        }
    }

    // ---- SDO catalog ----

    void exposeSdoCatalog(Registry& registry, uint64_t idBase,
                          const std::string& group) {
        // All register groups' kRegisterList collections.
        const ::EtherCAT::Drives::Registers::RegisterListOfLists groups = {
            &RegAS715N::C00::kRegisterList,
            &RegAS715N::C01::kRegisterList,
            &RegAS715N::C02::kRegisterList,
            &RegAS715N::C03::kRegisterList,
            &RegAS715N::C04::kRegisterList,
            &RegAS715N::C05::kRegisterList,
            &RegAS715N::C06::kRegisterList,
            &RegAS715N::C07::kRegisterList,
            &RegAS715N::C0A::kRegisterList,
            &RegAS715N::C10::kRegisterList,
            &RegAS715N::C13::kRegisterList,
            &RegAS715N::F30::kRegisterList,
            &RegAS715N::F31::kRegisterList,
            &RegAS715N::R20::kRegisterList,
            &RegAS715N::R22::kRegisterList,
            &RegAS715N::U40::kRegisterList,
            &RegAS715N::U41::kRegisterList,
            &RegAS715N::U42::kRegisterList,
        };

        for (const auto* list : groups) {
            for (const auto* entry : *list) {
                if (entry) exposeSdoEntry(registry, idBase, group, *entry);
            }
        }
    }

    void exposeSdoEntry(Registry& registry, uint64_t idBase,
                        const std::string& group,
                        const OD::ObjectDictionaryEntry& e) {
        // Local id encodes 0x01 | index | subindex — decodable by clients.
        const uint32_t local =
            0x01000000u | (static_cast<uint32_t>(e.index) << 8) | e.subindex;
        const uint64_t id = makeId(idBase, local);

        char nameBuf[64];
        std::snprintf(nameBuf, sizeof(nameBuf), "%s.sdo.%04X_%02X",
                      prefix_.c_str(), e.index, e.subindex);

        std::string desc = e.name ? e.name : "";
        if (e.comment && *e.comment) {
            desc += " — ";
            desc += e.comment;
        }

        ParamEntry p;
        p.id          = id;
        p.name        = nameBuf;
        p.description = desc;
        p.group       = group + ".sdo";
        p.extraFlags  = EntryFlags::NoStream;  // SDO reads must not be polled
                                               // at stream rate
        p.metadata    = {
            {"index",        hexU16(e.index)},
            {"subindex",     hexU8(e.subindex)},
            {"od_type",      hexU16(static_cast<uint16_t>(e.data_type))},
            {"default",      std::to_string(e.default_value)},
            {"min",          std::to_string(e.min_value)},
            {"max",          std::to_string(e.max_value)},
            {"unit",         std::to_string(static_cast<uint32_t>(e.unit))},
            {"modification", detail::modificationModeName(e.modification_mode)},
            {"effective",    detail::effectiveTimeName(e.effective_time)},
        };
        if (e.options_enum.has_value() && e.options_enum.name())
            p.metadata["options_enum"] = e.options_enum.name();

        const uint16_t index = e.index;
        const uint8_t  sub   = e.subindex;
        const bool writable  =
            e.modification_mode != OD::ModificationMode::ReadOnly;
        const uint8_t fixedSize = detail::odFixedSize(e.data_type);

        if (fixedSize > 0) {
            // Fixed-size scalar (including odd-width ints as raw bytes).
            ValueType vt;
            detail::odToValueType(e.data_type, vt);
            p.valueType = vt;
            p.readFn = [this, index, sub, fixedSize](void* d) {
                uint8_t buf[8] = {};
                coe_.readSync(index, sub, buf, fixedSize,
                              detail::kSdoTimeoutMs, nullptr);
                std::memcpy(d, buf, fixedSize);
            };
            if (writable) {
                p.writeFn = [this, index, sub, fixedSize](const void* src) {
                    coe_.writeSync(index, sub, src, fixedSize);
                };
            }
        } else {
            // Variable-length (strings, octet strings, domains).
            p.valueType     = ValueType::Binary;
            p.maxValueSize  = 256;
            p.varReadFn = [this, index, sub](void* d, size_t maxLen) -> size_t {
                size_t actual = 0;
                coe_.readSync(index, sub, d, maxLen,
                              detail::kSdoTimeoutMs, &actual);
                return actual;
            };
            if (writable) {
                p.varWriteFn = [this, index, sub](const void* src, size_t len) {
                    coe_.writeSync(index, sub, src, len);
                };
            }
        }

        registry.addParam(std::move(p));
    }

    static std::string hexU16(uint16_t v) {
        char b[8];
        std::snprintf(b, sizeof(b), "0x%04X", v);
        return b;
    }
    static std::string hexU8(uint8_t v) {
        char b[8];
        std::snprintf(b, sizeof(b), "0x%02X", v);
        return b;
    }

    IAS715NPdoAccess& pdo_;
    CoENS::CoEManager& coe_;
    uint16_t    driveIndex_;
    std::string prefix_;
    SpscRingStreamSource<AS715NPdoRow, RingCapacity> ringSource_;
};

} // namespace exposers
} // namespace io
} // namespace tether
