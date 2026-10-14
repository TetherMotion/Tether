#include "tether/drives/NexcobotESC211/RSAPInformationProcessor.hpp"

#include <format>

#include "logging/Logger.hpp"
#include "tether/drives/NexcobotESC211/Registers/SafetyStatus.hpp"
#include "tether/ethercat/SdoCommandChannel.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

namespace {
constexpr const char* TAG = "RSAPInfoProcessor";

namespace Reg = ::EtherCAT::Drives::Registers::NexcobotESC211::SafetyStatus;

const char* bitName(uint16_t index, uint32_t bit) {
    const auto labels = Reg::bitLabelsFor(index);
    for (const auto& label : labels) {
        if (label.mask == (1u << bit)) {
            return label.name;
        }
    }
    return nullptr;
}

// Enum decoders for the scalar state objects — value names per the
// "ESC CoE Object" manual (0x4000 tab).
const char* rsapStateName(uint8_t v) {
    static const char* const names[] = {
        "IDLE", "MONITORING", "CONFIGURED", "STOPPING", "ERROR"
    };
    return v < 5 ? names[v] : "Unknown";
}

// 0x4001 is a bitmask of MONITORING sub-states (Normal / Collaborative /
// Recovery / Demand / Initial Reset / Calibration); decode it via the
// bit labels instead of a plain enum.
std::string monitoringSubStateName(uint32_t v) {
    if (v == 0) {
        return {};
    }
    std::string out;
    for (uint32_t bit = 0; bit < 8; ++bit) {
        if (!(v & (1u << bit))) {
            continue;
        }
        if (!out.empty()) {
            out += '|';
        }
        out += bitName(Reg::MonitoringSubStateIndex, bit)
                 ? bitName(Reg::MonitoringSubStateIndex, bit)
                 : "?";
    }
    return out;
}

const char* operationModeName(uint8_t v) {
    static const char* const names[] = {
        "T1", "T2", "Auto", "Ext"
    };
    return v < 4 ? names[v] : "Unknown";
}

const char* stopStateName(uint8_t v) {
    static const char* const names[] = {
        "NotTriggered", "SOS", "STO"
    };
    return v < 3 ? names[v] : "Unknown";
}

const char* stopCategoryName(uint8_t v) {
    static const char* const names[] = {
        "Cat0", "Cat1", "Cat2", "ReduceSpeed", "NotTriggered"
    };
    return v < 5 ? names[v] : "Unknown";
}

// Value decoration for scalar objects (empty string when no decode known).
std::string scalarValueName(uint16_t index, uint32_t v) {
    switch (index) {
        case Reg::RSAPStateIndex:         return rsapStateName(static_cast<uint8_t>(v));
        case Reg::MonitoringSubStateIndex:return monitoringSubStateName(v);
        case Reg::OperationModeIndex:     return operationModeName(static_cast<uint8_t>(v));
        case Reg::StopStateIndex:         return stopStateName(static_cast<uint8_t>(v));
        case Reg::StopCategoryIndex:      return stopCategoryName(static_cast<uint8_t>(v));
    }
    return {};
}
} // namespace

const std::array<RSAPInformationProcessor::ObjectInfo,
                 RSAPInformationProcessor::kObjectCount>
RSAPInformationProcessor::kObjects = {{
    // Scalar state objects
    {0x4000, 0x00, "RSAP State",                        1, true},
    {0x4001, 0x00, "Monitoring Sub-State",              1, true},
    {0x4002, 0x00, "Error Code",                        2, true},
    {0x4003, 0x00, "Operation Mode",                    1, true},
    {0x4004, 0x00, "Stop State",                        1, true},
    {0x4005, 0x00, "Stop Category",                     1, true},
    // Bitfield status objects
    {0x4006, 0x00, "Fault and Violation Status",        1, false},
    {0x4007, 0x00, "Drive Status",                      1, false},
    // 0x4008 record: per-drive valid status
    {0x4008, 0x01, "Drive Position Valid Status",       1, false},
    {0x4008, 0x02, "Drive Velocity Valid Status",       1, false},
    {0x4008, 0x03, "Drive Error Status",                1, false},
    {0x4008, 0x04, "Drive STO Status",                  1, false},
    {0x4008, 0x05, "Drive SOS Status",                  1, false},
    // 0x4009 record: per-drive user defined bit status
    {0x4009, 0x01, "Drive 1 User Defined Bit Status",   1, false},
    {0x4009, 0x02, "Drive 2 User Defined Bit Status",   1, false},
    {0x4009, 0x03, "Drive 3 User Defined Bit Status",   1, false},
    {0x4009, 0x04, "Drive 4 User Defined Bit Status",   1, false},
    {0x4009, 0x05, "Drive 5 User Defined Bit Status",   1, false},
    {0x4009, 0x06, "Drive 6 User Defined Bit Status",   1, false},
    {0x4009, 0x07, "Drive 7 User Defined Bit Status",   1, false},
    // Input discrepancy + input states
    {0x400A, 0x00, "Input Discrepancy Status",          2, false},
    {0x400B, 0x00, "Emergency Stop Input State",        1, false},
    {0x400C, 0x00, "Normal Stop Input State",           1, false},
    {0x400D, 0x00, "Protective Stop Input State",       1, false},
    {0x400E, 0x00, "Enabling Device Input State",       1, false},
    {0x400F, 0x00, "Operation Mode Input State",        1, false},
    {0x4010, 0x00, "Reset Input State",                 1, false},
    {0x4011, 0x00, "Collaborative Input State",         1, false},
    {0x4012, 0x00, "HGC Input State",                   1, false},
    {0x4013, 0x00, "Monitored Position Input State",    1, false},
    {0x4014, 0x00, "SSM Input State",                   1, false},
    // Output / function / limit status objects
    {0x4015, 0x00, "Output Discrepancy Status",         2, false},
    {0x4016, 0x00, "Safety Output State",               2, false},
    {0x4017, 0x00, "Safety Control Function State",     2, false},
    {0x4018, 0x00, "Safety Limit Function State",       2, false},
    {0x4019, 0x00, "Axis Position Limit Status",        1, false},
    {0x401A, 0x00, "TCP Position Limit Status",         2, false},
    {0x401B, 0x00, "Endpoint Position Limit Status",    1, false},
    {0x401C, 0x00, "Axis Speed Limit Status",           1, false},
    {0x401D, 0x00, "TCP Speed Limit Status",            2, false},
    {0x401E, 0x00, "Endpoint Speed Limit Status",       1, false},
    {0x401F, 0x00, "Axis Torque Limit Status",          1, false},
    {0x4020, 0x00, "TCP Force Limit Status",            2, false},
    {0x4021, 0x00, "Endpoint 2 Force Limit State",      1, false},
    {0x4022, 0x00, "TCP 0 Orientation Limit State",     1, false},
    {0x4023, 0x00, "TCP 0 Robot Power Limit State",     1, false},
}};

// Object table for the generic poll engine — built from kObjects.
const std::array<EtherCAT::SdoObjectEntry,
                 RSAPInformationProcessor::kObjectCount>
kSdoEntries = [] {
    std::array<EtherCAT::SdoObjectEntry,
               RSAPInformationProcessor::kObjectCount> t{};
    for (size_t i = 0; i < t.size(); ++i) {
        t[i] = {RSAPInformationProcessor::kObjects[i].index,
                RSAPInformationProcessor::kObjects[i].subindex,
                RSAPInformationProcessor::kObjects[i].bytes};
    }
    return t;
}();

RSAPInformationProcessor::RSAPInformationProcessor(EtherCAT::Slave& slave)
    : slave_(slave) {}

size_t RSAPInformationProcessor::poll() {
    // Carry previous values forward so objects not attempted in a paused
    // or partially-failed poll keep their last known state.
    Snapshot next;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        next = snap_;
    }
    next.generation++;

    next.read_ok = EtherCAT::pollSdoObjects(
        slave_, kSdoEntries, next.value, next.ok,
        [this] { return paused_.load(std::memory_order_relaxed); },
        [](size_t i, const EtherCAT::SdoObjectEntry& e,
           EtherCAT::SlaveError err) {
            TETHER_LOGD(TAG, "0x{:04X}:{} ({}): SDO read failed ({})",
                        e.index, e.subindex,
                        RSAPInformationProcessor::kObjects[i].name,
                        static_cast<int>(err));
        });

    std::lock_guard<std::mutex> lock(mtx_);
    snap_ = next;
    return next.read_ok;
}

RSAPInformationProcessor::Snapshot RSAPInformationProcessor::snapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return snap_;
}

bool RSAPInformationProcessor::anyTriggered(const Snapshot& snap) {
    for (size_t i = 0; i < kObjectCount; ++i) {
        if (!snap.ok[i]) {
            continue;
        }
        const auto& o = kObjects[i];
        if (!o.scalar && snap.value[i] != 0) {
            return true;
        }
        if (o.index == Reg::ErrorCodeIndex && snap.value[i] != 0) {
            return true;
        }
    }
    return false;
}

std::vector<std::string>
RSAPInformationProcessor::decodeAll(const Snapshot& snap) {
    std::vector<std::string> lines;
    for (size_t i = 0; i < kObjectCount; ++i) {
        if (!snap.ok[i]) {
            continue;
        }
        const auto& o = kObjects[i];
        const uint32_t v = snap.value[i];
        if (o.scalar) {
            std::string detail = std::format("0x{:04X}:{} {} = {} (0x{:04X})",
                                             o.index, o.subindex, o.name,
                                             static_cast<int16_t>(v), v);
            if (const std::string name = scalarValueName(o.index, v);
                !name.empty()) {
                detail += std::format(" [{}]", name);
            }
            lines.push_back(std::move(detail));
        } else {
            std::string detail = std::format("0x{:04X}:{} {} = 0x{:02X}",
                                             o.index, o.subindex, o.name, v);
            if (const auto labels = Reg::bitLabelsFor(o.index); !labels.empty()) {
                std::string active;
                for (const auto& label : labels) {
                    if (label.isActive(v)) {
                        if (!active.empty()) active += ' ';
                        active += label.name;
                    }
                }
                if (!active.empty()) {
                    detail += std::format(" ({})", active);
                }
            }
            lines.push_back(detail);
        }
    }
    return lines;
}

std::vector<std::string>
RSAPInformationProcessor::decodeTriggered(const Snapshot& snap) {
    std::vector<std::string> lines;
    for (size_t i = 0; i < kObjectCount; ++i) {
        if (!snap.ok[i]) {
            continue;
        }
        const auto& o = kObjects[i];
        const uint32_t v = snap.value[i];

        if (o.scalar) {
            // Only the error code counts as a triggered state.
            if (o.index == Reg::ErrorCodeIndex && v != 0) {
                lines.push_back(std::format(
                    "0x{:04X}:{} {}: error {} (0x{:04X})",
                    o.index, o.subindex, o.name,
                    static_cast<int16_t>(v), v));
            }
            continue;
        }

        if (v == 0) {
            continue;
        }

        std::string active;
        const uint32_t bit_count = o.bytes * 8;
        for (uint32_t bit = 0; bit < bit_count; ++bit) {
            if (v & (1u << bit)) {
                if (!active.empty()) {
                    active += ", ";
                }
                if (const char* name = bitName(o.index, bit)) {
                    active += std::format("{}({})", name, bit);
                } else {
                    active += std::format("bit {}", bit);
                }
            }
        }
        if (!active.empty()) {
            lines.push_back(std::format(
                "0x{:04X}:{} {} [{}]", o.index, o.subindex, o.name, active));
        }
    }
    return lines;
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
