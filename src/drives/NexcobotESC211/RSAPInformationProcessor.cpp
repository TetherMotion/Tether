#include "tether/drives/NexcobotESC211/RSAPInformationProcessor.hpp"

#include <format>
#include <optional>

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

} // namespace

// Error-code descriptions per the vendor manual's "Error Code" table.
// Range entries (DI/DO diagnosis) are handled by range checks.
const char* RSAPInformationProcessor::errorCodeName(int32_t code) {
    if (code >= -310 && code <= -300) return "Safety DI diagnosis error";
    if (code >= -330 && code <= -320) return "Safety DO diagnosis error";
    switch (code) {
        case 0:         return "success";
        // RSAP
        case -1:        return "operation not accepted in current state";
        case -2:        return "unexpected exception";
        case -3:        return "system not ready";
        case -4:        return "invalid parameter number";
        case -5:        return "invalid parameter value";
        case -6:        return "access to invalid area";
        case -7:
        case -8:        return "input pointer is null";
        case -9:        return "structure size incompatible";
        case -10:       return "incompatible current state";
        case -11:       return "RSAP CONFIG DIO enable invalid";
        case -12:       return "RSAP function argument out of range";
        case -13:       return "RSAP cannot execute command in current state";
        case -14:       return "command out of range";
        case -15:       return "operation mode out of range";
        case -16:       return "stop-type state out of range";
        case -17:       return "stop category out of range";
        // CONFIG
        case -101:      return "RSAP config info CRC invalid";
        case -102:      return "RSAP config info header invalid";
        case -103:      return "RSAP config info version invalid";
        case -104:      return "RSAP config parameter index invalid";
        case -105:      return "RSAP config parameter count invalid";
        case -106:      return "RSAP config parameter length invalid";
        case -107:      return "RSAP config parameter subindex invalid";
        case -108:      return "RSAP config parameter datatype invalid";
        case -109:      return "RSAP config parameter count invalid (2)";
        case -110:      return "RSAP config command invalid";
        case -111:      return "RSAP CONFIG access to invalid area";
        case -112:      return "RSAP CONFIG DIO enable invalid (2)";
        case -201:      return "drive switched to STO unexpectedly";
        case -202:      return "drive did not switch to STO within time (CONFIG_STO_Timeout)";
        // SYSMGR
        case -10000001: return "fatal system error";
        case -10000002: return "null pointer (SYSMGR)";
        case -10000009: return "FlashMgr state unexpected";
        case -10000010: return "FlashMgr FNI CRC error";
        case -10000011: return "FlashMgr config CRC error";
        case -10000100: return "system state denies command";
        case -10000101: return "unknown control command";
        case -10000102: return "control command timeout";
        case -10000201: return "RSAP safety parameter verification failed";
        case -10000301: return "dual-MPU sync data mismatch";
        case -10000401: return "RSAP command timeout";
        // FSoE
        case -20001001: return "FSoE null pointer";
        case -20001002: return "FSoE TPG info array full";
        case -20001003: return "FSoE RX PDU queue full";
        case -20001004: return "FSoE RX PDU queue empty";
        case -20001005: return "FSoE TX PDU queue full";
        case -20001006: return "FSoE TX PDU queue empty";
        case -20001007: return "FSoE address unknown";
        case -20002001: return "FSoE null pointer (2)";
        case -20002004: return "FSoE CRC check failed";
        case -20002005: return "FSoE connection count mismatch";
        case -20002006: return "FSoE connection does not exist";
        case -20002007: return "invalid communication parameter length";
        case -20002008: return "invalid application parameter length";
        case -20002009: return "FSoE address is zero";
        case -20002010: return "duplicate FSoE address";
        case -20002011: return "FSoE connection ID is zero";
        case -20002012: return "duplicate FSoE connection ID";
        case -20002013: return "FSoE watchdog value is zero";
        case -20006002: return "invalid FSoE input parameter";
        case -20006003: return "invalid FSoE data length";
        case -20006004: return "FSoE null pointer (3)";
        case -20007001: return "fatal FSoE error";
        case -20007002: return "invalid FSoE pointer";
        case -20007003: return "invalid FSoE operation";
        case -20007004: return "FSoE initialization failed";
        case -20007005: return "invalid or missing FSoE connection";
        case -20007006: return "failed to retrieve FSoE state";
        case -20007007: return "invalid FSoE index";
        case -20007008: return "FSoE SAPL not initialized";
        case -20007009: return "FSoE not initialized";
        case -20007010: return "critical FSoE communication error";
        case -20007011: return "FSoE communication error";
        case -20007012: return "FSoE operation denied in current state";
    }
    return nullptr;
}

std::string RSAPInformationProcessor::scalarName(uint16_t index, uint32_t v) {
    switch (index) {
        case Reg::RSAPStateIndex:         return rsapStateName(static_cast<uint8_t>(v));
        case Reg::MonitoringSubStateIndex:return monitoringSubStateName(v);
        case Reg::ErrorCodeIndex:
            if (const char* n = errorCodeName(static_cast<int16_t>(v))) {
                return n;
            }
            return {};
        case Reg::OperationModeIndex:     return operationModeName(static_cast<uint8_t>(v));
        case Reg::StopStateIndex:         return stopStateName(static_cast<uint8_t>(v));
        case Reg::StopCategoryIndex:      return stopCategoryName(static_cast<uint8_t>(v));
    }
    return {};
}

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
    : SdoObjectPoller(slave, kSdoEntries,
                      +[](size_t i) { return kObjects[i].name; },
                      TAG) {}

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
            if (const std::string name = scalarName(o.index, v);
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

// ---------------------------------------------------------------------------
// Color-coded multi-panel display model
// ---------------------------------------------------------------------------

namespace {

using DisplayTone  = RSAPInformationProcessor::DisplayTone;
using DisplayCell  = RSAPInformationProcessor::DisplayCell;
using DisplayRow   = RSAPInformationProcessor::DisplayRow;
using DisplayPanel = RSAPInformationProcessor::DisplayPanel;

std::optional<uint32_t> snapValue(
    const RSAPInformationProcessor::Snapshot& snap,
    uint16_t index, uint8_t subindex) {
    for (size_t i = 0; i < RSAPInformationProcessor::kObjectCount; ++i) {
        const auto& o = RSAPInformationProcessor::kObjects[i];
        if (o.index == index && o.subindex == subindex) {
            return snap.ok[i] ? std::optional<uint32_t>(snap.value[i])
                              : std::nullopt;
        }
    }
    return std::nullopt;
}

using Tether::UI::labeledRow;

// Thin RSAP adapters over the generic bitfield chip builders
// (tether/ui/ChipPanels.hpp): they resolve the object value from the
// snapshot and delegate the tone semantics.
void pushExpectedSet(DisplayRow& row,
                     const RSAPInformationProcessor::Snapshot& snap,
                     uint16_t index) {
    Tether::UI::appendBitfieldChips(row, snapValue(snap, index, 0),
                                    Reg::bitLabelsFor(index),
                                    Tether::UI::ChipSemantics::ExpectedSet);
}

void pushExpectedClear(DisplayRow& row,
                       const RSAPInformationProcessor::Snapshot& snap,
                       uint16_t index) {
    Tether::UI::appendBitfieldChips(row, snapValue(snap, index, 0),
                                    Reg::bitLabelsFor(index),
                                    Tether::UI::ChipSemantics::ExpectedClear);
}

void pushModeChips(DisplayRow& row,
                   const RSAPInformationProcessor::Snapshot& snap,
                   uint16_t index) {
    Tether::UI::appendBitfieldChips(row, snapValue(snap, index, 0),
                                    Reg::bitLabelsFor(index),
                                    Tether::UI::ChipSemantics::Mode);
}

void pushScalar(DisplayRow& row, uint16_t index,
                const RSAPInformationProcessor::Snapshot& snap,
                DisplayTone tone) {
    std::optional<std::string> text;
    if (const auto v = snapValue(snap, index, 0)) {
        std::string name = RSAPInformationProcessor::scalarName(index, *v);
        text = name.empty()
                   ? std::format("{}", static_cast<int16_t>(*v))
                   : std::move(name);
    }
    Tether::UI::appendScalarChip(row, std::move(text), tone);
}

} // namespace

std::vector<RSAPInformationProcessor::DisplayPanel>
RSAPInformationProcessor::buildDisplayPanels(const Snapshot& snap,
                                             size_t expected_drives) {
    std::vector<DisplayPanel> panels;

    if (expected_drives == 0) {
        // Derive from the highest drive bit seen in the 0x4008 records.
        size_t n = 1;
        for (uint8_t sub = 1; sub <= 5; ++sub) {
            if (const auto v = snapValue(snap, Reg::DriveValidStatusIndex,
                                         sub)) {
                while (n < 8 && (*v >> n)) {
                    ++n;
                }
            }
        }
        expected_drives = n;
    }
    if (expected_drives > 7) {
        expected_drives = 7;
    }

    // ---- Panel 1: RSAP scalar state + fault/discrepancy ----
    {
        DisplayPanel p;
        p.title = std::format("RSAP state ({}/{} objects)",
                              snap.read_ok, kObjectCount);

        {
            auto v = snapValue(snap, Reg::RSAPStateIndex, 0);
            auto r = labeledRow("State");
            if (v) {
                DisplayTone t = DisplayTone::Warn;
                if (*v == 1)      t = DisplayTone::Ok;    // MONITORING
                else if (*v == 4) t = DisplayTone::Fault; // ERROR
                r.cells.push_back(
                    {scalarName(Reg::RSAPStateIndex, *v), t});
            } else {
                r.cells.push_back({"n/a", DisplayTone::Muted});
            }
            p.rows.push_back(std::move(r));
        }
        {
            auto r = labeledRow("Sub");
            pushModeChips(r, snap, Reg::MonitoringSubStateIndex);
            p.rows.push_back(std::move(r));
        }
        {
            auto r = labeledRow("OpMode");
            pushScalar(r, Reg::OperationModeIndex, snap, DisplayTone::Info);
            p.rows.push_back(std::move(r));
        }
        {
            // A stop state engaged is the safe state -> green.
            const auto v = snapValue(snap, Reg::StopStateIndex, 0);
            auto r = labeledRow("Stop");
            DisplayTone t = DisplayTone::Muted;
            if (v && *v != 0)  t = DisplayTone::Ok;    // stop engaged
            if (v && *v > 2)   t = DisplayTone::Warn;  // unknown value
            pushScalar(r, Reg::StopStateIndex, snap, t);
            p.rows.push_back(std::move(r));
        }
        {
            const auto v = snapValue(snap, Reg::StopCategoryIndex, 0);
            auto r = labeledRow("Cat");
            DisplayTone t = DisplayTone::Muted;
            if (v && *v < 4) t = DisplayTone::Ok;      // Cat0/1/2/RedSpd
            if (v && *v > 4) t = DisplayTone::Warn;
            pushScalar(r, Reg::StopCategoryIndex, snap, t);
            p.rows.push_back(std::move(r));
        }
        {
            const auto v = snapValue(snap, Reg::ErrorCodeIndex, 0);
            auto r = labeledRow("Err");
            if (!v) {
                r.cells.push_back({"n/a", DisplayTone::Muted});
            } else if (*v == 0) {
                r.cells.push_back({"0 success", DisplayTone::Ok});
            } else {
                const int32_t code = static_cast<int16_t>(*v);
                r.cells.push_back({std::format("{}", code),
                                   DisplayTone::Fault});
                if (const char* n = errorCodeName(code)) {
                    r.cells.push_back({n, DisplayTone::Fault});
                }
            }
            p.rows.push_back(std::move(r));
        }
        {
            auto r = labeledRow("Faults");
            pushExpectedClear(r, snap, Reg::FaultAndViolationStatusIndex);
            p.rows.push_back(std::move(r));
        }
        {
            auto r = labeledRow("InDisc");
            pushExpectedClear(r, snap, Reg::InputDiscrepancyStatusIndex);
            p.rows.push_back(std::move(r));
        }
        panels.push_back(std::move(p));
    }

    // ---- Panel 2: per-drive matrix + summary + user bits ----
    {
        DisplayPanel p;
        p.title = "Drives";

        {
            DisplayRow r;
            r.cells.push_back({"", DisplayTone::Muted});
            for (size_t d = 0; d < expected_drives; ++d) {
                r.cells.push_back({std::format("D{}", d + 1),
                                   DisplayTone::Info});
            }
            p.rows.push_back(std::move(r));
        }
        static const struct {
            const char* name;
            uint8_t     sub;
            bool        good_when_set;
        } matrix[] = {
            {"posV", 1, true},  {"velV", 2, true}, {"err", 3, false},
            {"stoV", 4, true},  {"sosV", 5, true},
        };
        for (const auto& m : matrix) {
            auto r = labeledRow(m.name);
            const auto v =
                snapValue(snap, Reg::DriveValidStatusIndex, m.sub);
            if (!v) {
                r.cells.push_back({"n/a", DisplayTone::Muted});
            } else {
                for (size_t d = 0; d < expected_drives; ++d) {
                    const bool set = *v & (1u << d);
                    const DisplayTone t =
                        m.good_when_set
                            ? (set ? DisplayTone::Ok : DisplayTone::Missing)
                            : (set ? DisplayTone::Fault : DisplayTone::Ok);
                    r.cells.push_back({"█", t});
                }
            }
            p.rows.push_back(std::move(r));
        }
        {
            auto r = labeledRow("sum");
            pushModeChips(r, snap, Reg::DriveStatusIndex);
            p.rows.push_back(std::move(r));
        }
        for (size_t d = 0; d < expected_drives; ++d) {
            DisplayRow r;
            r.cells.push_back({std::format("usr D{}", d + 1),
                               DisplayTone::Muted});
            const auto v =
                snapValue(snap, Reg::DriveUserBitStatusIndex,
                          static_cast<uint8_t>(d + 1));
            if (!v) {
                r.cells.push_back({"n/a", DisplayTone::Muted});
            } else if (*v == 0) {
                r.cells.push_back({"none", DisplayTone::Muted});
            } else {
                for (const auto& l :
                     Reg::bitLabelsFor(Reg::DriveUserBitStatusIndex)) {
                    if (l.isActive(*v)) {
                        r.cells.push_back({l.name, DisplayTone::Info});
                    }
                }
                // any set bit without a label
                for (uint32_t b = 0; b < 8; ++b) {
                    if ((*v & (1u << b)) &&
                        !bitName(Reg::DriveUserBitStatusIndex, b)) {
                        r.cells.push_back({std::format("bit{}", b),
                                           DisplayTone::Info});
                    }
                }
            }
            p.rows.push_back(std::move(r));
        }
        panels.push_back(std::move(p));
    }

    // ---- Panel 3: safety input states (expected-set) ----
    {
        DisplayPanel p;
        p.title = "Safety inputs";
        static const struct { const char* name; uint16_t index; } in[] = {
            {"EStop",  Reg::EmergencyStopInputStateIndex},
            {"NStop",  Reg::NormalStopInputStateIndex},
            {"PStop",  Reg::ProtectiveStopInputStateIndex},
            {"EnaDev", Reg::EnablingDeviceInputStateIndex},
            {"OpMode", Reg::OperationModeInputStateIndex},
            {"Reset",  Reg::ResetInputStateIndex},
            {"Collab", Reg::CollaborativeInputStateIndex},
            {"HGC",    Reg::HGCInputStateIndex},
            {"MonPos", Reg::MonitoredPositionInputStateIndex},
            {"SSM",    Reg::SSMInputStateIndex},
        };
        for (const auto& i : in) {
            auto r = labeledRow(i.name);
            pushExpectedSet(r, snap, i.index);
            p.rows.push_back(std::move(r));
        }
        panels.push_back(std::move(p));
    }

    // ---- Panel 4: outputs & safety functions ----
    {
        DisplayPanel p;
        p.title = "Outputs & functions";
        {
            auto r = labeledRow("OutDisc");
            pushExpectedClear(r, snap, Reg::OutputDiscrepancyStatusIndex);
            p.rows.push_back(std::move(r));
        }
        {
            auto r = labeledRow("SafeOut");
            pushExpectedSet(r, snap, Reg::SafetyOutputStateIndex);
            p.rows.push_back(std::move(r));
        }
        {
            auto r = labeledRow("CtrlFunc");
            pushExpectedSet(r, snap, Reg::SafetyControlFunctionStateIndex);
            p.rows.push_back(std::move(r));
        }
        {
            auto r = labeledRow("LimFunc");
            pushExpectedClear(r, snap, Reg::SafetyLimitFunctionStateIndex);
            p.rows.push_back(std::move(r));
        }
        panels.push_back(std::move(p));
    }

    // ---- Panel 5: limit statuses (expected-clear) ----
    {
        DisplayPanel p;
        p.title = "Limits";
        static const struct { const char* name; uint16_t index; } lim[] = {
            {"AxisPos",  0x4019}, {"TCPPos",   0x401A},
            {"EndPos",   0x401B}, {"AxisSpd",  0x401C},
            {"TCPSpd",   0x401D}, {"EndSpd",   0x401E},
            {"AxisTrq",  0x401F}, {"TCPForce", 0x4020},
            {"End2Frc",  0x4021}, {"Orient",   0x4022},
            {"TCP0Pwr",  0x4023},
        };
        for (const auto& l : lim) {
            auto r = labeledRow(l.name);
            pushExpectedClear(r, snap, l.index);
            p.rows.push_back(std::move(r));
        }
        panels.push_back(std::move(p));
    }

    return panels;
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
