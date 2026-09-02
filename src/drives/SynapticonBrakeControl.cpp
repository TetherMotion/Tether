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

    const auto wr = sdo.writeU8(ObjectIndex, kSubBrakeStatus,
                                static_cast<uint8_t>(BrakeStatusValue::Disengaged),
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

    const auto wr = sdo.writeU8(ObjectIndex, kSubBrakeStatus,
                                static_cast<uint8_t>(BrakeStatusValue::Engaged),
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
    const auto res = sdo.readU8(ObjectIndex, kSubBrakeStatus, {.timeout_ms = to});
    if (!res.has_value()) {
        TETHER_LOGW(TAG, "{}: Failed to read 0x2004:7 (brake status)",
                    sdo.logPrefix().c_str());
        return std::nullopt;
    }
    const uint8_t raw = res.value();
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

    const auto wr = sdo.writeU8(ObjectIndex, kSubReleaseStrategy,
                                static_cast<uint8_t>(strategy),
                                {.timeout_ms = to});
    if (!wr.has_value()) {
        TETHER_LOGE(TAG, "{}: Failed to write 0x2004:4 (release strategy)",
                    sdo.logPrefix().c_str());
        return false;
    }
    return true;
}

} // namespace Synapticon
} // namespace Drives
} // namespace EtherCAT
