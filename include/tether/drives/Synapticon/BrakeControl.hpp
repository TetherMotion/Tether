/**
 * @file BrakeControl.hpp
 * @brief Synapticon SOMANET brake control convenience helpers (0x2004)
 *
 * Wraps the Synapticon manufacturer object 0x2004 (Brake options) into
 * easy-to-use SDO helpers.  The brake is spring-activated — it engages when
 * powered off for safety and must be disengaged by applying current to a
 * solenoid.
 *
 * Documentation:
 *   https://doc.synapticon.com/node/sw5.1/objects_html/2xxx/2004.html
 *
 * 0x2004:7 (Brake status) is the primary control point in automatic mode
 * (Pin brake / Clutch brake release strategy):
 *   0 = Not configured (no action)
 *   1 = Engaged        (brake prevents motion)
 *   2 = Disengaged     (brake released)
 *
 * Typical usage — disengage the brake before starting FSoE communication:
 * @code
 *   auto& sdo = master.ethercatMaster().sdoManager(slave_idx);
 *   if (!EtherCAT::Drives::Synapticon::BrakeControl::disengageBrake(sdo)) {
 *       // handle failure
 *   }
 * @endcode
 *
 * The CoEManager is forward-declared to keep this header lightweight; the
 * implementation lives in SynapticonBrakeControl.cpp and links against
 * tether_ethercat_master (which provides CoEManager).
 */
#pragma once

#include <cstdint>
#include <optional>

#include "tether/drives/Synapticon/Registers/DriveConfig2000.hpp"

namespace EtherCAT { namespace CoE { class CoEManager; } }

namespace EtherCAT {
namespace Drives {
namespace Synapticon {

/// Brake status / control values for 0x2004:7.
using BrakeStatusValue = Registers::Synapticon::Obj2004::BrakeStatusOptions;

/// SDO transaction timeout used by the brake helpers when none is given.
static constexpr uint32_t kBrakeSdoTimeoutMs = 3000;

/// Delay applied after commanding a brake state change so the solenoid has
/// time to actuate before the caller proceeds.
static constexpr uint32_t kBrakeActuationDelayMs = 200;

/**
 * @brief Convenience helpers for the Synapticon 0x2004 Brake options object.
 *
 * All methods perform synchronous CoE/SDO access through the supplied
 * CoEManager and return true on success.  They are safe to call while the
 * slave is in PRE_OP or higher (mailbox communication must be available).
 */
class BrakeControl {
public:
    /// Disengage (release) the brake by writing 2 to 0x2004:7.
    /// Optionally verifies the status afterwards and waits for actuation.
    /// @param sdo       CoE manager bound to the target slave.
    /// @param timeout_ms SDO transaction timeout (0 = use default).
    /// @param verify    When true, re-reads 0x2004:7 and confirms Disengaged.
    /// @return true if the write (and optional verification) succeeded.
    static bool disengageBrake(EtherCAT::CoE::CoEManager& sdo,
                               uint32_t timeout_ms = 0,
                               bool verify = true);

    /// Engage the brake by writing 1 to 0x2004:7.
    /// @param sdo       CoE manager bound to the target slave.
    /// @param timeout_ms SDO transaction timeout (0 = use default).
    /// @param verify    When true, re-reads 0x2004:7 and confirms Engaged.
    /// @return true if the write (and optional verification) succeeded.
    static bool engageBrake(EtherCAT::CoE::CoEManager& sdo,
                            uint32_t timeout_ms = 0,
                            bool verify = true);

    /// Read the current brake status from 0x2004:7.
    /// @return the BrakeStatusValue on success, or std::nullopt on SDO failure.
    static std::optional<BrakeStatusValue> readBrakeStatus(
        EtherCAT::CoE::CoEManager& sdo,
        uint32_t timeout_ms = 0);

    /// @return true when the brake reports Disengaged (0x2004:7 == 2).
    static bool isDisengaged(EtherCAT::CoE::CoEManager& sdo,
                             uint32_t timeout_ms = 0);

    /// @return true when the brake reports Engaged (0x2004:7 == 1).
    static bool isEngaged(EtherCAT::CoE::CoEManager& sdo,
                          uint32_t timeout_ms = 0);

    /// Configure the automatic release strategy (0x2004:4).
    /// Use this to switch from manual output voltage (0) to Pin brake (2) or
    /// Clutch-style brake (1) control before disengaging.
    /// @return true on successful SDO write.
    static bool setReleaseStrategy(
        EtherCAT::CoE::CoEManager& sdo,
        Registers::Synapticon::Obj2004::ReleaseStrategyOptions strategy,
        uint32_t timeout_ms = 0);
};

} // namespace Synapticon
} // namespace Drives
} // namespace EtherCAT
