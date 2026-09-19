/**
 * @file as715n_reset_errors.cpp
 * @brief AS715N — dedicated fault/error reset utility
 *
 * Uses the AS715N-specific error reset mechanism:
 *   - Fault reset via 0x2031:01 (F31.00), 0 -> 1 -> 0 sequence, performed
 *     after clearing the S-ON (Switch-On) bit in the controlword as required
 *     by the A6-EC manual.
 *   - Optional software reset via 0x2031:02 (F31.02) with --software-reset.
 *   - DC sync errors are handled by the specialised handleNoSyncError()
 *     recovery sequence.
 *
 * The slave is only brought to PRE_OP — no PDO mapping, no OP, no motion.
 *
 * Usage:
 *   ./as715n_reset_errors -i eth0                 # reset active fault
 *   ./as715n_reset_errors -i eth0 --force         # reset even if no fault is reported
 *   ./as715n_reset_errors -i eth0 --software-reset # full drive software reset
 */

#include <cstdint>
#include <string>

#include "DS402ExampleSupport.hpp"
#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715N/Registers/F31-ControlInProgress.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/platform/EspCompat.hpp"

namespace {

constexpr const char* TAG = "as715n_reset";
constexpr uint16_t kSlaveIndex = 0;

using EtherCAT::Drives::AS715NFaultHandler;
using EtherCAT::Drives::AS715NError;

int resetErrors(EtherCAT::DS402Master& master, bool software_reset, bool force)
{
    auto& sdo = master.ethercatMaster().sdoManager(kSlaveIndex);

    if (software_reset) {
        constexpr auto& SwReg =
            EtherCAT::Drives::Registers::AS715N::F31::SoftwareReset;
        TETHER_LOGI(TAG, "Performing software reset via 0x{:04X}:0x{:02X} ({})...",
                    SwReg.index, SwReg.subindex, SwReg.name);
        if (!sdo.writeU16(SwReg.index, SwReg.subindex, 1,
                          {.timeout_ms = 3000}).has_value()) {
            TETHER_LOGE(TAG, "Failed to write software reset register");
            return 3;
        }
        TETHER_LOGI(TAG, "Software reset command sent — drive is rebooting");
        return 0;
    }

    uint16_t mfr_error = 0, cia402_error = 0;
    const bool has_fault =
        AS715NFaultHandler::checkFault(sdo, kSlaveIndex, &mfr_error, &cia402_error);

    if (!has_fault) {
        if (!force) {
            TETHER_LOGI(TAG, "Slave {} reports no fault — nothing to reset", kSlaveIndex);
            return 0;
        }
        TETHER_LOGI(TAG, "Slave {} reports no fault — forcing reset anyway",
                    kSlaveIndex);
    }

    // The A6-EC manual requires S-ON (controlword bit 0) to be off before
    // requesting the fault reset via F31.00.
    auto cw_result = sdo.readU16(0x6040, 0x00, {.timeout_ms = 3000});
    if (!cw_result.has_value()) {
        TETHER_LOGE(TAG, "Failed to read Controlword (0x6040) — cannot clear S-ON");
        return 3;
    }
    const uint16_t cw = cw_result.value();
    if (cw & 0x0001) {
        TETHER_LOGI(TAG, "Clearing S-ON bit in Controlword: 0x{:04X} -> 0x{:04X}",
                    cw, cw & ~static_cast<uint16_t>(0x0001));
        if (!sdo.writeU16(0x6040, 0x00,
                          static_cast<uint16_t>(cw & ~0x0001u),
                          {.timeout_ms = 3000}).has_value()) {
            TETHER_LOGE(TAG, "Failed to clear S-ON bit");
            return 3;
        }
        Tether::Platform::Clock::instance().delayMilliseconds(50);
    }

    // DC sync errors get the specialised recovery sequence; everything else
    // uses the plain 0x2031:01 fault reset.
    const AS715NError err = AS715NError::parse(mfr_error);
    bool reset_ok;
    if (has_fault && err.isDCSyncError()) {
        TETHER_LOGI(TAG, "DC sync error {} — using handleNoSyncError()", err.name);
        reset_ok = AS715NFaultHandler::handleNoSyncError(sdo, kSlaveIndex, 3);
    } else if (has_fault && !err.is_recoverable && !force) {
        TETHER_LOGE(TAG, "Error {} is marked non-recoverable — refusing reset "
                         "(use --force to override)", err.name);
        return 4;
    } else {
        if (has_fault && !err.is_recoverable) {
            TETHER_LOGW(TAG, "Error {} is marked non-recoverable — forcing reset",
                        err.name);
        }
        reset_ok = AS715NFaultHandler::resetFault(sdo, kSlaveIndex);
    }

    if (!reset_ok) {
        TETHER_LOGE(TAG, "Fault reset failed — error persists");
        return 5;
    }

    TETHER_LOGI(TAG, "Slave {} fault reset OK", kSlaveIndex);
    return 0;
}

} // namespace

int main(int argc, char** argv)
{
    argparse::ArgumentParser program("as715n_reset_errors", "1.0",
                                     argparse::default_arguments::help);
    Tether::Examples::addInterfaceArg(program);
    program.add_argument("--software-reset")
        .default_value(false)
        .implicit_value(true)
        .help("Perform a full software reset via 0x2031:02 (F31.02) instead of "
              "a fault reset");
    program.add_argument("--force")
        .default_value(false)
        .implicit_value(true)
        .help("Attempt the fault reset even when no fault is reported or the "
              "error is marked non-recoverable");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << '\n' << program;
        return 1;
    }

    const std::string iface = Tether::Examples::resolveInterface(
        program.get<std::string>("--interface"), TAG);
    if (iface.empty()) {
        return 1;
    }
    const bool software_reset = program.get<bool>("--software-reset");
    const bool force = program.get<bool>("--force");

    Tether::Platform::ensureRealtimeKernelOrExit();

    EtherCAT::DS402Master master;
    Tether::Examples::HostMasterSession session;
    if (!Tether::Examples::startHostMasterSession(iface, master, session, TAG)) {
        return 2;
    }

    if (master.ethercatMaster().discovery().discover(EtherCAT::DiscoveryOptions()).empty()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    if (!master.waitForDriveCount(kSlaveIndex + 1, 2000)) {
        TETHER_LOGE(TAG, "Timed out waiting for drive at index {}", kSlaveIndex);
        Tether::Examples::stopHostMasterSession(master, session);
        return 2;
    }

    // SDO access only needs the mailbox configured and the slave in PRE_OP —
    // no PDO mapping and no OP transition.
    if (!master.ethercatMaster().autoConfigureMailbox(kSlaveIndex,
            Tether::Platform::LogLevel::Info)) {
        TETHER_LOGE(TAG, "Mailbox configuration failed for slave {}", kSlaveIndex);
        Tether::Examples::stopHostMasterSession(master, session);
        return 3;
    }
    if (!master.ethercatMaster().transitionSlaveToPreOperational(kSlaveIndex)) {
        TETHER_LOGE(TAG, "Failed to bring slave {} to PRE_OP", kSlaveIndex);
        Tether::Examples::stopHostMasterSession(master, session);
        return 3;
    }

    const int rc = resetErrors(master, software_reset, force);

    Tether::Examples::stopHostMasterSession(master, session);
    return rc;
}
