/**
 * @file Synapticon.hpp
 * @brief Synapticon SOMANET CiA402 drive — aggregate header
 *
 * Vendor/Product from ESI (SOMANET_CiA_402_v5.1.9.xml):
 *   Vendor ID:    0x000022D2  (Synapticon GmbH)
 *   Product Code: 0x00000201  (SOMANET Node)
 *   Product Code: 0x00000301  (SOMANET Circulo)
 *   Product Code: 0x00000302  (SOMANET Circulo 7 SafeMotion)
 *
 * This header provides:
 *   - Identity constants (vendor ID, product codes)
 *   - PDO layout definitions (packed structs + descriptors)
 *   - Complete manufacturer object dictionary (0x2000-0x2705)
 *
 * The object dictionary entries are extracted from the ESI file with
 * access rights supplemented from the Synapticon online documentation:
 *   https://doc.synapticon.com/circulo_safe_motion/sw5.1/objects_html/2xxx/
 */

#pragma once

#include <array>
#include <cstdint>

#include "tether/drives/Synapticon/SynapticonPDO.hpp"
#include "tether/drives/Synapticon/SafetyDiagnostics.hpp"
#include "tether/drives/Synapticon/BrakeControl.hpp"
#include "tether/drives/Synapticon/Registers/Common.hpp"
#include "tether/drives/Synapticon/Registers/DriveConfig2000.hpp"
#include "tether/drives/Synapticon/Registers/Controllers2010.hpp"
#include "tether/drives/Synapticon/Registers/Monitoring2030.hpp"
#include "tether/drives/Synapticon/Registers/Encoders2110.hpp"
#include "tether/drives/Synapticon/Registers/GPIO2210.hpp"
#include "tether/drives/Synapticon/Registers/Safety2600.hpp"
#include "tether/drives/Synapticon/Registers/TuningUser2700.hpp"

namespace EtherCAT {
namespace Drives {

// ============================================================================
// Identity constants (from ESI Vendor section)
// ============================================================================

namespace Synapticon {

static constexpr uint32_t kVendorId    = 0x000022D2;  // Synapticon GmbH
static constexpr uint32_t kProductCodeNode    = 0x00000201;  // SOMANET Node
static constexpr uint32_t kProductCodeCirculo = 0x00000301;  // SOMANET Circulo
static constexpr uint32_t kProductCodeCirculo7SafeMotion = 0x00000302;  // SOMANET Circulo 7 SafeMotion

// All known SOMANET product codes (used for identity verification).
inline constexpr std::array<uint32_t, 3> kKnownProductCodes = {
    kProductCodeNode,
    kProductCodeCirculo,
    kProductCodeCirculo7SafeMotion,
};

// @brief Check whether a product code is a known SOMANET device.
inline constexpr bool isKnownProductCode(uint32_t product_code) {
    for (const auto pc : kKnownProductCodes) {
        if (pc == product_code) return true;
    }
    return false;
}

// Mailbox configuration (from ESI Sm elements)
// The SOMANET ESI advertises 1024-byte mailbox buffers, but the firmware
// only accepts 512 bytes.  Configuring 1024 causes AL_STATUS_CODE 0x0016
// ("Invalid mailbox configuration (PRE_OP)").  Use 512 — the value the
// drive actually accepts.
static constexpr uint16_t kMailboxWriteAddr = 0x1000;
static constexpr uint16_t kMailboxWriteSize = 512;
static constexpr uint16_t kMailboxReadAddr  = 0x1400;
static constexpr uint16_t kMailboxReadSize  = 512;
static constexpr uint16_t kMailboxProtocols = 0x000C;  // CoE | FoE
static constexpr uint32_t kSdoTimeoutMs     = 6000;    // ESI ResponseTimeout

// Process data sync managers (from ESI Sm elements)
static constexpr uint16_t kOutputsSmAddr = 0x1800;  // SM2 (M->S)
static constexpr uint16_t kInputsSmAddr  = 0x1C00;  // SM3 (S->M)

// SM control bytes (from ESI Sm ControlByte attributes)
static constexpr uint8_t kOutputsSmControlByte = 0x64;  // SM2: Buffered|Write|Watchdog|RepeatReq
static constexpr uint8_t kInputsSmControlByte  = 0x20;  // SM3: Buffered|Read|Watchdog

// Total PDO sizes (from ESI Sm DefaultSize attributes)
static constexpr uint16_t kOutputsSmSize = 35;  // SM2: 19+8+8 bytes
static constexpr uint16_t kInputsSmSize  = 47;  // SM3: 13+12+4+18 bytes

} // namespace Synapticon

// ============================================================================
// Aggregate register list (all manufacturer-specific 0x2xxx objects)
// ============================================================================

namespace SynapticonRegisters {

using RegisterListOfLists = ::EtherCAT::Drives::Registers::Synapticon::RegisterListOfLists;

inline const RegisterListOfLists kAllRegisterLists = {
    &Registers::Synapticon::Obj2000::kRegisterList,
    &Registers::Synapticon::Obj2001::kRegisterList,
    &Registers::Synapticon::Obj2002::kRegisterList,
    &Registers::Synapticon::Obj2003::kRegisterList,
    &Registers::Synapticon::Obj2004::kRegisterList,
    &Registers::Synapticon::Obj2005::kRegisterList,
    &Registers::Synapticon::Obj2006::kRegisterList,
    &Registers::Synapticon::Obj2008::kRegisterList,
    &Registers::Synapticon::Obj200A::kRegisterList,
    &Registers::Synapticon::Obj200B::kRegisterList,
    &Registers::Synapticon::Obj2010::kRegisterList,
    &Registers::Synapticon::Obj2011::kRegisterList,
    &Registers::Synapticon::Obj2012::kRegisterList,
    &Registers::Synapticon::Obj2017::kRegisterList,
    &Registers::Synapticon::Obj2021::kRegisterList,
    &Registers::Synapticon::Obj2022::kRegisterList,
    &Registers::Synapticon::Obj2023::kRegisterList,
    &Registers::Synapticon::Obj2027::kRegisterList,
    &Registers::Synapticon::Obj2030::kRegisterList,
    &Registers::Synapticon::Obj2031::kRegisterList,
    &Registers::Synapticon::Obj2038::kRegisterList,
    &Registers::Synapticon::Obj203F::kRegisterList,
    &Registers::Synapticon::Obj2040::kRegisterList,
    &Registers::Synapticon::Obj20E1::kRegisterList,
    &Registers::Synapticon::Obj20F0::kRegisterList,
    &Registers::Synapticon::Obj20F2::kRegisterList,
    &Registers::Synapticon::Obj20F3::kRegisterList,
    &Registers::Synapticon::Obj2110::kRegisterList,
    &Registers::Synapticon::Obj2111::kRegisterList,
    &Registers::Synapticon::Obj2112::kRegisterList,
    &Registers::Synapticon::Obj2113::kRegisterList,
    &Registers::Synapticon::Obj2210::kRegisterList,
    &Registers::Synapticon::Obj2211::kRegisterList,
    &Registers::Synapticon::Obj2212::kRegisterList,
    &Registers::Synapticon::Obj2213::kRegisterList,
    &Registers::Synapticon::Obj2214::kRegisterList,
    &Registers::Synapticon::Obj2215::kRegisterList,
    &Registers::Synapticon::Obj2401::kRegisterList,
    &Registers::Synapticon::Obj2402::kRegisterList,
    &Registers::Synapticon::Obj2403::kRegisterList,
    &Registers::Synapticon::Obj2404::kRegisterList,
    &Registers::Synapticon::Obj2600::kRegisterList,
    &Registers::Synapticon::Obj2601::kRegisterList,
    &Registers::Synapticon::Obj2602::kRegisterList,
    &Registers::Synapticon::Obj2603::kRegisterList,
    &Registers::Synapticon::Obj2604::kRegisterList,
    &Registers::Synapticon::Obj2605::kRegisterList,
    &Registers::Synapticon::Obj2610::kRegisterList,
    &Registers::Synapticon::Obj2611::kRegisterList,
    &Registers::Synapticon::Obj2620::kRegisterList,
    &Registers::Synapticon::Obj2621::kRegisterList,
    &Registers::Synapticon::Obj2625::kRegisterList,
    &Registers::Synapticon::Obj2630::kRegisterList,
    &Registers::Synapticon::Obj2631::kRegisterList,
    &Registers::Synapticon::Obj2635::kRegisterList,
    &Registers::Synapticon::Obj2641::kRegisterList,
    &Registers::Synapticon::Obj2650::kRegisterList,
    &Registers::Synapticon::Obj2668::kRegisterList,
    &Registers::Synapticon::Obj2670::kRegisterList,
    &Registers::Synapticon::Obj2690::kRegisterList,
    &Registers::Synapticon::Obj26A0::kRegisterList,
    &Registers::Synapticon::Obj26F0::kRegisterList,
    &Registers::Synapticon::Obj2701::kRegisterList,
    &Registers::Synapticon::Obj2702::kRegisterList,
    &Registers::Synapticon::Obj2703::kRegisterList,
    &Registers::Synapticon::Obj2704::kRegisterList,
    &Registers::Synapticon::Obj2705::kRegisterList,
};

} // namespace SynapticonRegisters

// ============================================================================
// Device struct (matching AS715N pattern)
// ============================================================================

struct SOMANET {
    static constexpr uint32_t kVendorId = Synapticon::kVendorId;
    static constexpr uint32_t kProductCode = Synapticon::kProductCodeNode;

    // Error report object
    static constexpr uint16_t kErrorReportIndex = 0x203F;

    // Safety diagnostic objects
    static constexpr uint16_t kSafetyModuleInputDiagnosticsIndex = 0x2611;
    static constexpr uint16_t kManufacturingParametersIndex = 0x2610;
    static constexpr uint16_t kGeneralSafetyIndex = 0x2620;

    // Access to all registers
    static inline const auto& kAllRegisterLists = SynapticonRegisters::kAllRegisterLists;
};

// Backwards-compatible type alias
using SOMANETDevice = SOMANET;

} // namespace Drives
} // namespace EtherCAT
