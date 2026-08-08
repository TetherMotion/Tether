/**
 * @file as715n_check_error_code.cpp
 * @brief AS715N — Read & optionally reset manufacturer / CiA402 error codes
 *
 * Usage (host build):
 *   ./as715n_check_error_code -i eth0                    # just print codes
 *   ./as715n_check_error_code -r -i eth0                  # attempt reset (or use --interface)
 *
 * This example only performs error-code inspection via EtherCAT (SDO/PDO
 * helpers) and prints all available details, then exits. It purposely does
 * NOT enable the drive or perform motion.
 */

#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

#include "tether/ethercat/Diagnostics.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715N/Registers/F31-ControlInProgress.hpp"
#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/ESIFile.hpp"
#include "tether/ethercat/ESIParser.hpp"
#include "tether/platform/EspCompat.hpp"

#include "common/ExampleHelpers.hpp"
#include "common/EtherCATHostSetup.hpp"

static const char* TAG = "AS715N_CheckError";

using namespace EtherCAT::Drives;

static void printAS715NErrorDetails(EtherCAT::CoE::CoEManager& sdo, uint16_t slave_idx, uint16_t mfr_error, uint16_t cia402_error) {
    if (mfr_error == 0 && cia402_error == 0) {
        TETHER_LOGI(TAG, "Slave %u: no manufacturer or CiA402 error reported", slave_idx);
        return;
    }

    auto mfr_ext = AS715NFaultHandler::readManufacturerFaultExtended(sdo, slave_idx);
    TETHER_LOGI(TAG, "Slave %u: Manufacturer fault 0x203F (external=0x%04X internal=0x%04X)",
               slave_idx, mfr_ext.external_code, mfr_ext.internal_code);
    if (mfr_error != 0) {
        AS715NError err = AS715NError::parse(mfr_error);
        char name[32] = {0};
        AS715NError::format(name, sizeof(name), err.class_code, err.sub_code);

        TETHER_LOGI(TAG, "  - Raw:        0x%04X\n  - Name:       %s\n  - Class/Sub:  0x%X / 0x%X\n  - Desc:       %s\n  - Recoverable:%s\n  - DC SyncErr: %s",
                   err.raw_code, name, err.class_code, err.sub_code,
                   err.description ? err.description : "(none)",
                   err.is_recoverable ? " YES" : " NO",
                   err.isDCSyncError() ? " YES" : " NO");
    }

    TETHER_LOGI(TAG, "Slave %u: CiA402 error (0x%04X / %u)", slave_idx, cia402_error, cia402_error);
    if (cia402_error != 0) {
        TETHER_LOGI(TAG, "  - Raw CiA402 error: 0x%04X (%u)\n  - Note: CiA402 manufacturer-specific faults are reported with high-byte 0x87 (see device documentation)", cia402_error, cia402_error);
    }
}

static int inspectAndMaybeReset(EtherCAT::Master& master, bool do_reset, bool do_sw_reset) {
    uint16_t slave_count = master.getDiscoveredSlaveCount();
    if (slave_count == 0) {
        TETHER_LOGE(TAG, "No slaves discovered");
        return 2;
    }

    // For simplicity target slave 0 (common usage in examples). If multiple
    // slaves exist, we will still only inspect slave index 0.
    uint16_t slave_idx = 0;
    auto& sdo = master.sdoManager(slave_idx);

    // Request PREOP state for the slave (required for SDO access)

    // --- Pre-PRE_OP: auto-configure mailbox from SII with verbose diagnostics ---
    EtherCAT::Diagnostics::PreOperationalMailboxDiagnosticsOptions mailbox_diagnostics_options;
    mailbox_diagnostics_options.auto_configure_log_level = Tether::Platform::LogLevel::Debug;
    EtherCAT::Diagnostics::logPreOperationalMailboxDiagnostics(master, slave_idx, TAG, mailbox_diagnostics_options);

    TETHER_LOGI(TAG, "Requesting PREOP state for slave %u...", slave_idx);
    if (!master.transitionSlaveToPreOperational(slave_idx)) {
        TETHER_LOGE(TAG, "Failed to bring slave %u to PREOP state", slave_idx);
        return 7;
    }
    TETHER_LOGI(TAG, "Slave %u is now in PREOP state", slave_idx);

    // Print full parsed SII (human-readable) for diagnostics
    (void)EtherCAT::Diagnostics::logParsedSlaveSII(master, slave_idx, TAG);

    // Read fault codes using the AS715N helper
    uint16_t mfr_error = 0, cia402_error = 0;
    bool has_fault = AS715NFaultHandler::checkFault(sdo, slave_idx, &mfr_error, &cia402_error);

    if (!has_fault) {
        TETHER_LOGI(TAG, "Slave %u reports no fault (0x203F=0, 0x603F=0)", slave_idx);
        return 0;
    }

    // If we couldn't read StatusWord, AS715NFaultHandler will still try to populate 0x203F/0x603F.
    // Provide additional EtherCAT-side diagnostics here (requested: slave state / global error info).
    if (mfr_error == 0 && cia402_error == 0) {
        EtherCAT::Diagnostics::logSlaveApplicationLayerDiagnostics(master, slave_idx, TAG);
    }

    printAS715NErrorDetails(sdo, slave_idx, mfr_error, cia402_error);

    if (!do_reset && !do_sw_reset) return 0;

    // Software reset: write 1 to F31.02 (0x2031:02) and return immediately
    if (do_sw_reset) {
        constexpr auto& SwReg = ::EtherCAT::Drives::Registers::AS715N::F31::SoftwareReset;
        TETHER_LOGI(TAG, "Performing software reset via %04X:%02X (%s)...",
                   SwReg.index, SwReg.subindex, SwReg.name);
        if (!sdo.writeU16(SwReg.index, SwReg.subindex, 1, {.timeout_ms = 3000}).has_value()) {
            TETHER_LOGE(TAG, "Slave %u: Failed to write %04X:%02X = 1 (software reset)",
                       slave_idx, SwReg.index, SwReg.subindex);
            return 3;
        }
        TETHER_LOGI(TAG, "Software reset command sent to slave %u", slave_idx);
        return 0;
    }

    // When performing a control-word based reset (-r), ensure Switch-On bit is cleared
    if (do_reset) {
        auto cw_result = sdo.readU16(0x6040, 0x00, {.timeout_ms = 3000});
        if (!cw_result.has_value()) {
            TETHER_LOGE(TAG, "Slave %u: failed to read Controlword (0x6040) — cannot proceed with -r reset", slave_idx);
            return 3;
        }
        uint16_t cw = cw_result.value();
        uint16_t new_cw = static_cast<uint16_t>(cw & ~static_cast<uint16_t>(0x0001)); // clear Switch-On bit (bit 0)
        if (new_cw != cw) {
            TETHER_LOGI(TAG, "Slave %u: clearing Switch-On bit in Controlword (0x6040): 0x%04X -> 0x%04X", slave_idx, cw, new_cw);
            if (!sdo.writeU16(0x6040, 0x00, new_cw, {.timeout_ms = 3000}).has_value()) {
                TETHER_LOGE(TAG, "Slave %u: failed to write Controlword (0x6040) to clear Switch-On bit", slave_idx);
                return 3;
            }
            Tether::Platform::Clock::instance().delayMilliseconds(50);
        } else {
            TETHER_LOGD(TAG, "Slave %u: Switch-On bit already cleared (Controlword=0x%04X)", slave_idx, cw);
        }
        (void)cw; // used in log above
    }

    // Decide which reset method to use based on error semantics
    AS715NError err = AS715NError::parse(mfr_error);

    TETHER_LOGI(TAG, "Attempting reset for slave %u...", slave_idx);

    bool reset_ok = false;
    if (err.isDCSyncError()) {
        // Use specialized handler for DC sync errors — usually successful
        TETHER_LOGI(TAG, "Detected DC-sync error (%s) — using handleNoSyncError()",
                    err.name);
        reset_ok = AS715NFaultHandler::handleNoSyncError(sdo, slave_idx, 3);
    } else if (!err.is_recoverable) {
        TETHER_LOGW(TAG, "Error is marked NOT recoverable by device — will not attempt control-word reset");
        reset_ok = false;
    } else {
        // Generic reset: use the published `FaultReset` register entry (0x2031:01)
        constexpr auto& Reg = ::EtherCAT::Drives::Registers::AS715N::F31::FaultReset;
        TETHER_LOGI(TAG, "Using register %04X:%02X (%s) to request fault-reset...",
                   Reg.index, Reg.subindex, Reg.name);

        // Follow the 0 -> 1 -> 0 sequence the device expects.
        if (!sdo.writeU16(Reg.index, Reg.subindex, 0, {.timeout_ms = 3000}).has_value()) {
            TETHER_LOGE(TAG, "Slave %u: Failed to write %04X:%02X = 0", slave_idx, Reg.index, Reg.subindex);
            reset_ok = false;
        } else {
            Tether::Platform::Clock::instance().delayMilliseconds(50);
            if (!sdo.writeU16(Reg.index, Reg.subindex, 1, {.timeout_ms = 3000}).has_value()) {
                TETHER_LOGE(TAG, "Slave %u: Failed to write %04X:%02X = 1", slave_idx, Reg.index, Reg.subindex);
                reset_ok = false;
            } else {
                Tether::Platform::Clock::instance().delayMilliseconds(200);
                (void)sdo.writeU16(Reg.index, Reg.subindex, 0, {.timeout_ms = 3000}).has_value();
                Tether::Platform::Clock::instance().delayMilliseconds(50);

                // Verify by re-reading manufacturer/CiA402 error fields
                uint16_t new_mfr = AS715NFaultHandler::readManufacturerFault(sdo, slave_idx);
                uint16_t new_cia = AS715NFaultHandler::readCiA402Error(sdo, slave_idx);
                if (new_mfr == 0 && new_cia == 0) {
                    TETHER_LOGI(TAG, "Fault cleared for slave %u", slave_idx);
                    return 0;
                }

                TETHER_LOGW(TAG, "Fault still present after register-reset (0x203F=0x%03X, 0x603F=0x%04X)", new_mfr, new_cia);
                printAS715NErrorDetails(sdo, slave_idx, new_mfr, new_cia);
                return 4;
            }
        }
    }

    if (!reset_ok) {
        TETHER_LOGE(TAG, "Reset attempt returned FAILURE or NO-OP (fault may be non-resettable)");
        return 3;
    }

    return 0;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("as715n_check_error_code");
    Tether::Examples::addInterfaceArg(program);
    Tether::Examples::addMailboxSizeArg(program);
    Tether::Examples::addMailboxAddressArg(program);
    Tether::Examples::addEsiXmlArg(program);

    program.add_argument("-r","--reset").default_value(false).implicit_value(true)
        .help("Attempt to reset the reported error (if recoverable)");
    program.add_argument("--software-reset").default_value(false).implicit_value(true)
        .help("Perform a software reset via F31.02 (write 1 to 0x2031:02); does not require a fault to be present");

    try { program.parse_args(argc, argv); }
    catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        return 1;
    }

    std::string iface = program.get<std::string>("--interface");
    bool do_reset = program.get<bool>("--reset");
    bool do_sw_reset = program.get<bool>("--software-reset");
    std::string esi_xml = program.get<std::string>("--esi-xml");

    std::optional<EtherCAT::ESIFile> esi;
    if (!esi_xml.empty()) {
        esi.emplace(esi_xml);
        if (esi->empty()) {
            TETHER_LOGE(TAG, "Failed to parse ESI XML '%s': %s",
                        esi_xml.c_str(), esi->errorMessage().c_str());
            return 1;
        }
        TETHER_LOGI(TAG, "Loaded ESI XML '%s' (%zu device(s)) for cross-reference",
                    esi_xml.c_str(), esi->devices().size());
    }

    TETHER_LOGI(TAG, "AS715N error-code inspector (host)\nNetwork interface: %s", iface.c_str());

    Tether::Examples::HostEtherNetSession session;
    if (!Tether::Examples::initHostEthernet(session, iface, TAG)) {
        return 2;
    }

    EtherCAT::Master::Config mcfg;
    mcfg.enable_mailbox_fallback = true;
    EtherCAT::Master master(mcfg);

    session.eth->setRxCallback([&master](const uint8_t* frame, size_t len,
                                           const EtherCAT::HAL::RxFrameInfo& info, void*){
        (void)info; master.handleRxFrame(frame, len);
    }, nullptr);

    Tether::Examples::startHostPollThread(session, TAG);

    master.start(*session.ni, session.srcMac);

    if (!master.discoverSlaves()) {
        TETHER_LOGW(TAG, "No slaves discovered");
    }

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    if (slaves == 0) {
        Tether::Examples::shutdownHostEthernet(session);
        return 5;
    }

    int rc = inspectAndMaybeReset(master, do_reset, do_sw_reset);

    // Print ESI device info for cross-reference if --esi-xml was provided
    if (esi && !esi->empty()) {
        TETHER_LOGI(TAG, "\n=== ESI XML Cross-Reference (%s) ===", esi_xml.c_str());
        for (const auto& dev : esi->devices()) {
            TETHER_LOGI(TAG, "%s",
                        EtherCAT::ESI::formatDeviceHumanReadable(dev, true).c_str());
        }
    }

    Tether::Examples::shutdownHostEthernet(session);

    return rc;
}
