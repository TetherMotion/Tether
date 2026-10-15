#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "tether/ethercat/SdoObjectPoller.hpp"
#include "tether/ethercat/Slave.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/// Polled table size — every data sub-entry of 0x4100-0x4115.
/// Free-standing because the class needs it in its base-specifier.
inline constexpr size_t kRSAPMonitoringPollObjects = 104;

/**
 * @brief Snapshot of one RSAP-monitoring poll cycle.
 *
 * Adds (index, subindex) lookup helpers to the generic storage; the
 * search table is `RSAPMonitoringProcessor::kObjects`.
 */
struct RSAPMonitoringSnapshot
    : SdoPollSnapshot<kRSAPMonitoringPollObjects> {
    /**
     * @brief Raw value of one object by (index, subindex).
     * @return the value, or nullopt when the object is not in the
     *         table or its last read failed.
     */
    std::optional<uint32_t> get(uint16_t index, uint8_t subindex = 0) const;

    /// Signed interpretation of get() for DINT objects.
    std::optional<int32_t> getSigned(uint16_t index,
                                     uint8_t subindex = 0) const;
};

/**
 * @brief Reads the ESC211 RSAP monitoring objects 0x4100-0x4115 via SDO.
 *
 * Where `RSAPInformationProcessor` covers the *status* range 0x4000-0x4023
 * (states, triggered bits, error codes), this processor covers the
 * *calculated values and monitoring limits* range — the quantities the
 * safety application continuously computes and compares against the
 * configured limits:
 *
 *   0x4100  TCP 0 position calculated            (record, 6x DINT)
 *   0x4101  TCP monitoring speed limit           (UDINT scalar)
 *   0x4102  TCP 0..8 speed calculated            (record, 9x UDINT)
 *   0x4103  TCP monitoring force limit           (UDINT scalar)
 *   0x4104  TCP 0 + safe TCP 1..8 force calc.    (record, 9x UDINT)
 *   0x4105  TCP 0 force vector calculated        (record, 3x DINT)
 *   0x4106  TCP monitoring power limit           (UDINT scalar)
 *   0x4107  TCP 0 power calculated               (UDINT)
 *   0x4108  Endpoints monitoring speed limit     (UDINT scalar)
 *   0x4109  Endpoint 1..7 speed calculated       (record, 7x UDINT)
 *   0x410A  Endpoints monitoring force limit     (UDINT scalar)
 *   0x410B  Endpoint 2 force calculated          (UDINT)
 *   0x410C  Axis 1..7 position calculated        (record, 7x DINT)
 *   0x410D  Axis 1..7 monitoring speed limit     (record, 7x UDINT)
 *   0x410E  Axis 0..6 speed calculated           (record, 7x UDINT —
 *           0-based axis names in the manual, subs align positionally)
 *   0x4110  Axis 1..7 acceleration calculated    (record, 7x UDINT)
 *   0x4111  Axis 1..7 monitoring torque limit    (record, 7x UDINT)
 *   0x4112  Axis 1..7 torque calculated          (record, 7x UDINT)
 *   0x4113  Joint 1..7 inverse dynamics torque   (record, 7x DINT)
 *   0x4114  Joint 1..7 friction torque           (record, 7x DINT)
 *   0x4115  Axis 1..7 lever arm length to TCP 0  (record, 7x UDINT)
 *
 * All data entries are 4 bytes (UDINT or DINT); record sub-index 0 is
 * the entry count and is not polled.  Values are raw device units — the
 * ESI does not publish scaling factors.
 *
 * Typical use (same poll/publish split as RSAPInformationProcessor):
 * @code
 *   RSAPMonitoringProcessor mon(slave);
 *   mon.poll();                       // in a ~1 Hz background thread
 *   auto snap = mon.snapshot();
 *   if (auto lim = snap.get(0x410D, 1))  // Axis 1 speed limit
 *       ...
 * @endcode
 */
class RSAPMonitoringProcessor
    : public SdoObjectPoller<kRSAPMonitoringPollObjects,
                             RSAPMonitoringSnapshot> {
public:
    using Snapshot = RSAPMonitoringSnapshot;

    /// Object dictionary metadata for one polled (sub)object.
    struct ObjectInfo {
        uint16_t    index;     ///< CoE index (0x4100..0x4115)
        uint8_t     subindex;  ///< CoE subindex (0 for scalars, 1..n records)
        const char* name;      ///< ESI object name
        bool        is_signed; ///< DINT when true, UDINT otherwise
    };

    /// All polled objects (every data sub-entry of 0x4100-0x4115).
    static constexpr size_t kObjectCount = kRSAPMonitoringPollObjects;
    static const std::array<ObjectInfo, kObjectCount> kObjects;

    explicit RSAPMonitoringProcessor(EtherCAT::Slave& slave);

    // poll()/snapshot()/pause()/resume()/isPaused() are inherited from
    // SdoObjectPoller — failed reads mark the slot !ok and polling
    // continues; while paused the poll aborts between objects, keeping
    // previous values.  pause() keeps SDO reads from overlapping 0xF100
    // safety state machine command sequences.
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
