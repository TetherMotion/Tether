#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "tether/ethercat/SdoObjectPoller.hpp"
#include "tether/ethercat/Slave.hpp"
#include "tether/ui/ChipPanels.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

/// Polled table size — every 0x4000-0x4023 (sub)object.
/// Free-standing because the class needs it in its base-specifier.
inline constexpr size_t kRSAPInformationPollObjects = 46;

/**
 * @brief Reads the ESC211 RSAP information objects 0x4000-0x4023 via SDO.
 *
 * The 0x4000 range is the live safety-application status interface (also
 * exposed cyclically as TxPDO 0x1A02 "TxPDO-Map-RSAP-Info", ESI v0.9:
 * 46 entries / 55 bytes).  Object layout per ESI v0.9:
 *
 *   0x4000  RSAP state                        (USINT, scalar)
 *   0x4001  Sub state in MONITORING state     (USINT, scalar)
 *   0x4002  Error code                        (INT,   scalar)
 *   0x4003  Operation mode                    (USINT, scalar)
 *   0x4004  Stop state                        (USINT, scalar)
 *   0x4005  Stop category                     (USINT, scalar)
 *   0x4006  Fault and violation status        (USINT, bitfield)
 *   0x4007  Drive status                      (USINT, bitfield)
 *   0x4008:1-5  Drive valid status            (USINT, bitfield each)
 *   0x4009:1-7  Drive N user defined bit      (USINT, bitfield each)
 *   0x400A  Input discrepancy status          (UINT,  bitfield)
 *   0x400B-0x4014  Input states (emergency stop, normal stop, protective
 *             stop, enabling device, operation mode, reset, collaborative,
 *             HGC, monitored position, SSM)   (USINT, bitfield each)
 *   0x4015  Output discrepancy status         (UINT,  bitfield)
 *   0x4016  Safety output state               (UINT,  bitfield)
 *   0x4017  Safety control function state     (UINT,  bitfield)
 *   0x4018  Safety limit function state       (UINT,  bitfield)
 *   0x4019-0x4023  Limit statuses (axis/TCP/endpoint position, speed,
 *             torque/force, TCP0 orientation, robot power)  (bitfield)
 *
 * For the bitfield objects each bit represents one configured element
 * (input channel, output channel, limiting function, axis, ...); a set bit
 * means that element's state is currently triggered/active.  The scalar
 * objects carry plain values; a non-zero 0x4002 error code counts as
 * triggered.
 *
 * Typical use (mirrors SystemErrorManager's poll/publish split):
 * @code
 *   RSAPInformationProcessor rsap(slave);
 *   // in a ~3 Hz background thread:
 *   rsap.poll(slave);
 *   auto snap = rsap.snapshot();
 *   for (const auto& line : RSAPInformationProcessor::decodeTriggered(snap))
 *       log(line);
 * @endcode
 */
class RSAPInformationProcessor
    : public SdoObjectPoller<kRSAPInformationPollObjects> {
public:
    /// All RSAP information objects, in index order.
    static constexpr size_t kObjectCount = kRSAPInformationPollObjects;

    /// Copyable snapshot of one poll cycle — safe to share across threads.
    using Snapshot = SdoPollSnapshot<kObjectCount>;

    /// Object dictionary metadata for one 0x40xx (sub)object.
    struct ObjectInfo {
        uint16_t    index;    ///< CoE index (0x4000..0x4023)
        uint8_t     subindex; ///< CoE subindex (0, or 1..n for records)
        const char* name;     ///< ESI object name
        uint8_t     bytes;    ///< 1 (USINT) or 2 (UINT/INT)
        bool        scalar;   ///< true = plain value, false = bitfield
    };

    static const std::array<ObjectInfo, kObjectCount> kObjects;

    explicit RSAPInformationProcessor(EtherCAT::Slave& slave);

    // poll()/snapshot()/pause()/resume()/isPaused() are inherited from
    // SdoObjectPoller — best-effort reads (failed objects marked !ok,
    // polling continues; safe to call concurrently with snapshot()/
    // decodeTriggered()), pause() aborts between objects so no SDO
    // read overlaps safety state machine command sequences (0xF100).

    /// True if any bitfield object has a set bit or the error code != 0.
    static bool anyTriggered(const Snapshot& snap);

    /**
     * @brief Decode a snapshot into human-readable lines.
     *
     * One line per object: bitfields show the set bit indices
     * ("… State: 0x05 [bits 0,2]"), scalars show the plain value.
     * Only objects that were read successfully appear.
     */
    static std::vector<std::string> decodeAll(const Snapshot& snap);

    /**
     * @brief Decode only the triggered states — the objects whose value
     *        indicates an active/triggered element.
     *
     * Bitfields: one summary line per non-zero object, listing each set bit
     * as "Name(bit)" (or "bit N" when no label is known), e.g.
     * "0x400B:0 Emergency stop input state [BasicEmergencyStopState(0)]".
     * Scalars: only 0x4002 Error code counts as triggered when non-zero.
     * Empty when nothing is triggered.
     */
    static std::vector<std::string> decodeTriggered(const Snapshot& snap);

    /// Value name for a scalar object per the vendor manual — e.g.
    /// RSAP state 1 -> "MONITORING", op mode 0 -> "T1", error code
    /// -202 -> "Drive does not switch to STO status within the
    /// specified time".  Empty string when no decode exists.
    static std::string scalarName(uint16_t index, uint32_t value);

    /// Error-code description from the vendor manual's Error Code
    /// table (covers the RSAP/CONFIG/SAFETY/SYSMGR/FSoE ranges).
    /// nullptr for unknown codes.
    static const char* errorCodeName(int32_t code);

    // ------------------------------------------------------------------
    // Color-coded multi-panel display model
    //
    // buildDisplayPanels() turns a snapshot into a set of panels
    // ("RSAP state", "Drives", "Safety inputs", "Outputs & functions",
    // "Limits") made of rows of cells.  Each cell carries a DisplayTone;
    // the UI layer maps tones to its own colors:
    //
    //   Ok      — enabled / present / safe state            (green)
    //   Missing — expected but absent/inactive              (red)
    //   Fault   — active fault / violation / error          (red+bold)
    //   Warn    — transitional / needs attention            (yellow)
    //   Info    — informational value                       (cyan/blue)
    //   Muted   — inactive but acceptable / unknown         (dim)
    //
    // The tone assignment encodes the object semantics: bits that are
    // expected to be set for a healthy configuration (drive valid
    // flags, wired safety inputs, driven safety outputs, active control
    // functions) render red when clear — that is what highlights
    // "what is missing".  Fault/limit/discrepancy objects are the
    // inverse: they show a green "none" when clear and red chips for
    // every set bit.
    // ------------------------------------------------------------------

    // The cell/row/panel types are the generic Tether::UI chip-panel
    // model (tether/ui/ChipPanels.hpp) — aliased here so existing
    // RSAPInformationProcessor::DisplayPanel code keeps compiling.
    using DisplayTone  = Tether::UI::DisplayTone;
    using DisplayCell  = Tether::UI::DisplayCell;
    using DisplayRow   = Tether::UI::DisplayRow;
    using DisplayPanel = Tether::UI::DisplayPanel;

    /**
     * @brief Build the color-coded multi-panel display model of a snapshot.
     *
     * @param expected_drives number of configured FSoE drives; bits for
     *        these drives are expected set in the 0x4008 records and are
     *        highlighted Missing when clear.  0 = derive the count from
     *        the highest bit seen in the snapshot (min. 1).
     * @return panels in display order; the UI renders them side by side
     *         or stacked.
     */
    static std::vector<DisplayPanel>
    buildDisplayPanels(const Snapshot& snap, size_t expected_drives = 0);
};

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
