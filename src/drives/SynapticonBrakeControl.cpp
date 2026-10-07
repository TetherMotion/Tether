/**
 * @file SynapticonBrakeControl.cpp
 * @brief Synapticon SOMANET brake control convenience helpers (0x2004)
 *
 * Implements the BrakeControl static methods declared in
 * Synapticon/BrakeControl.hpp.  All access is via synchronous CoE/SDO
 * transactions through the per-slave CoEManager.
 *
 * Object 0x2004 (Brake options) documentation:
 *   https://doc.synapticon.com/node/sw5.1/objects_html/2xxx/2004.html
 */
#include "tether/drives/Synapticon/BrakeControl.hpp"

#include "tether/ethercat/CoEManager.hpp"
#include "tether/platform/Platform.hpp"

#include <format>
#include <string>

namespace EtherCAT {
namespace Drives {
namespace Synapticon {

using namespace Registers::Synapticon::Obj2004;

namespace {
const char* TAG = "synapticon.brake";

const char* brakeStatusName(BrakeStatusValue v) {
    switch (v) {
        case BrakeStatusValue::NotConfigured: return "Not configured";
        case BrakeStatusValue::Engaged:       return "Engaged";
        case BrakeStatusValue::Disengaged:    return "Disengaged";
    }
    return "Unknown";
}

uint32_t resolveTimeout(uint32_t timeout_ms) {
    return (timeout_ms != 0) ? timeout_ms : kBrakeSdoTimeoutMs;
}
} // namespace

// ============================================================================
// disengageBrake — write 2 to 0x2004:7
// ============================================================================
bool BrakeControl::disengageBrake(EtherCAT::CoE::CoEManager& sdo,
                                  uint32_t timeout_ms,
                                  bool verify) {
    const uint32_t to = resolveTimeout(timeout_ms);
    TETHER_LOGI(TAG, "{}: Disengaging brake (0x2004:7 <- 2)...",
                sdo.logPrefix().c_str());

    const auto wr = sdo.writeEntry(BrakeStatus,
                                   static_cast<uint64_t>(BrakeStatusValue::Disengaged),
                                   {.timeout_ms = to});
    if (!wr.has_value()) {
        TETHER_LOGE(TAG, "{}: Failed to write 0x2004:7=2 (disengage)",
                    sdo.logPrefix().c_str());
        return false;
    }

    // Allow the solenoid time to actuate before verifying / proceeding.
    Tether::Platform::Clock::instance().delayMilliseconds(kBrakeActuationDelayMs);

    if (!verify) {
        TETHER_LOGI(TAG, "{}: Brake disengage command sent (not verified)",
                    sdo.logPrefix().c_str());
        return true;
    }

    const auto status = readBrakeStatus(sdo, to);
    if (!status.has_value()) {
        TETHER_LOGW(TAG, "{}: Brake disengage written but status readback failed",
                    sdo.logPrefix().c_str());
        return true;  // write succeeded; only verification failed
    }

    if (*status == BrakeStatusValue::Disengaged) {
        TETHER_LOGI(TAG, "{}: Brake DISENGAGED (0x2004:7 = {})",
                    sdo.logPrefix().c_str(), brakeStatusName(*status));
        return true;
    }

    TETHER_LOGW(TAG, "{}: Brake not yet disengaged (0x2004:7 = {})",
                sdo.logPrefix().c_str(), brakeStatusName(*status));
    return false;
}

// ============================================================================
// engageBrake — write 1 to 0x2004:7
// ============================================================================
bool BrakeControl::engageBrake(EtherCAT::CoE::CoEManager& sdo,
                               uint32_t timeout_ms,
                               bool verify) {
    const uint32_t to = resolveTimeout(timeout_ms);
    TETHER_LOGI(TAG, "{}: Engaging brake (0x2004:7 <- 1)...",
                sdo.logPrefix().c_str());

    const auto wr = sdo.writeEntry(BrakeStatus,
                                   static_cast<uint64_t>(BrakeStatusValue::Engaged),
                                   {.timeout_ms = to});
    if (!wr.has_value()) {
        TETHER_LOGE(TAG, "{}: Failed to write 0x2004:7=1 (engage)",
                    sdo.logPrefix().c_str());
        return false;
    }

    Tether::Platform::Clock::instance().delayMilliseconds(kBrakeActuationDelayMs);

    if (!verify) {
        TETHER_LOGI(TAG, "{}: Brake engage command sent (not verified)",
                    sdo.logPrefix().c_str());
        return true;
    }

    const auto status = readBrakeStatus(sdo, to);
    if (!status.has_value()) {
        TETHER_LOGW(TAG, "{}: Brake engage written but status readback failed",
                    sdo.logPrefix().c_str());
        return true;
    }

    if (*status == BrakeStatusValue::Engaged) {
        TETHER_LOGI(TAG, "{}: Brake ENGAGED (0x2004:7 = {})",
                    sdo.logPrefix().c_str(), brakeStatusName(*status));
        return true;
    }

    TETHER_LOGW(TAG, "{}: Brake not yet engaged (0x2004:7 = {})",
                sdo.logPrefix().c_str(), brakeStatusName(*status));
    return false;
}

// ============================================================================
// readBrakeStatus — read 0x2004:7
// ============================================================================
std::optional<BrakeStatusValue> BrakeControl::readBrakeStatus(
    EtherCAT::CoE::CoEManager& sdo,
    uint32_t timeout_ms) {
    const uint32_t to = resolveTimeout(timeout_ms);
    const auto res = sdo.readEntry(BrakeStatus, {.timeout_ms = to});
    if (!res.has_value()) {
        TETHER_LOGW(TAG, "{}: Failed to read 0x2004:7 (brake status)",
                    sdo.logPrefix().c_str());
        return std::nullopt;
    }
    const uint8_t raw = static_cast<uint8_t>(res.value());
    if (raw > static_cast<uint8_t>(BrakeStatusValue::Disengaged)) {
        TETHER_LOGW(TAG, "{}: Unexpected brake status value 0x{:02X}",
                    sdo.logPrefix().c_str(), raw);
        return std::nullopt;
    }
    return static_cast<BrakeStatusValue>(raw);
}

// ============================================================================
// isDisengaged / isEngaged
// ============================================================================
bool BrakeControl::isDisengaged(EtherCAT::CoE::CoEManager& sdo,
                                uint32_t timeout_ms) {
    const auto status = readBrakeStatus(sdo, timeout_ms);
    return status.has_value() && *status == BrakeStatusValue::Disengaged;
}

bool BrakeControl::isEngaged(EtherCAT::CoE::CoEManager& sdo,
                             uint32_t timeout_ms) {
    const auto status = readBrakeStatus(sdo, timeout_ms);
    return status.has_value() && *status == BrakeStatusValue::Engaged;
}

// ============================================================================
// setReleaseStrategy — write 0x2004:4
// ============================================================================
bool BrakeControl::setReleaseStrategy(
    EtherCAT::CoE::CoEManager& sdo,
    ReleaseStrategyOptions strategy,
    uint32_t timeout_ms) {
    const uint32_t to = resolveTimeout(timeout_ms);
    TETHER_LOGI(TAG, "{}: Setting brake release strategy (0x2004:4 <- {})...",
                sdo.logPrefix().c_str(), static_cast<unsigned>(strategy));

    const auto wr = sdo.writeEntry(ReleaseStrategy,
                                   static_cast<uint64_t>(strategy),
                                   {.timeout_ms = to});
    if (!wr.has_value()) {
        TETHER_LOGE(TAG, "{}: Failed to write 0x2004:4 (release strategy)",
                    sdo.logPrefix().c_str());
        return false;
    }
    return true;
}

// ============================================================================
// readConfig — best-effort snapshot of all 0x2004 entries
// ============================================================================
BrakeControl::Config BrakeControl::readConfig(EtherCAT::CoE::CoEManager& sdo,
                                              uint32_t timeout_ms) {
    const uint32_t to = resolveTimeout(timeout_ms);
    Config cfg;
    const auto rd = [&](const ObjectDictionary::ObjectDictionaryEntry& e)
            -> std::optional<uint64_t> {
        const auto r = sdo.readEntry(e, {.timeout_ms = to});
        if (!r.has_value()) return std::nullopt;
        return *r;
    };
    if (const auto v = rd(ReleaseStrategy))
        cfg.release_strategy = static_cast<uint8_t>(*v);
    if (const auto v = rd(PullVoltage))
        cfg.pull_voltage = static_cast<uint32_t>(*v);
    if (const auto v = rd(HoldVoltage))
        cfg.hold_voltage = static_cast<uint32_t>(*v);
    if (const auto v = rd(PullTime))
        cfg.pull_time = static_cast<uint16_t>(*v);
    if (const auto v = rd(BrakeStatus)) {
        const uint8_t raw = static_cast<uint8_t>(*v);
        if (raw <= static_cast<uint8_t>(BrakeStatusValue::Disengaged))
            cfg.status = static_cast<BrakeStatusValue>(raw);
    }
    if (const auto v = rd(OutputVoltage))
        cfg.output_voltage = static_cast<uint16_t>(*v);
    return cfg;
}

void BrakeControl::dumpConfig(EtherCAT::CoE::CoEManager& sdo,
                              const char* tag, uint32_t timeout_ms) {
    const Config cfg = readConfig(sdo, timeout_ms);
    const std::string prefix = sdo.logPrefix();
    const auto fmt = [](const auto& opt) -> std::string {
        if (opt.has_value()) return std::format("{}", *opt);
        return "<read failed>";
    };
    TETHER_LOGI(tag, "{}: brake cfg 0x2004:1 Pull voltage      = {}",
                prefix.c_str(), fmt(cfg.pull_voltage));
    TETHER_LOGI(tag, "{}: brake cfg 0x2004:2 Hold voltage      = {}",
                prefix.c_str(), fmt(cfg.hold_voltage));
    TETHER_LOGI(tag, "{}: brake cfg 0x2004:3 Pull time         = {}",
                prefix.c_str(), fmt(cfg.pull_time));
    TETHER_LOGI(tag, "{}: brake cfg 0x2004:4 Release strategy  = {}",
                prefix.c_str(), fmt(cfg.release_strategy));
    TETHER_LOGI(tag, "{}: brake cfg 0x2004:7 Brake status      = {}",
                prefix.c_str(),
                cfg.status ? brakeStatusName(*cfg.status) : "<read failed>");
    TETHER_LOGI(tag, "{}: brake cfg 0x2004:A Output voltage    = {}",
                prefix.c_str(), fmt(cfg.output_voltage));
}

bool BrakeControl::verifyDisengaged(
    EtherCAT::CoE::CoEManager& sdo,
    std::optional<bool> statusword_disengaged,
    const char* tag, uint32_t timeout_ms) {
    const auto status = readBrakeStatus(sdo, timeout_ms);
    const char* sw_str = statusword_disengaged.has_value()
        ? (*statusword_disengaged ? "DISENGAGED" : "engaged")
        : "n/a";
    TETHER_LOGI(tag, "{}: brake check: statusword bit15={} | 0x2004:7={}",
                sdo.logPrefix().c_str(), sw_str,
                status ? brakeStatusName(*status) : "read-failed");
    return status.has_value() && *status == BrakeStatusValue::Disengaged;
}

} // namespace Synapticon
} // namespace Drives
} // namespace EtherCAT
