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

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>

#include "tether/ethercat/EtherCATDiagnostics.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"
#include "tether/drives/AS715N.hpp"
#include "tether/drives/AS715N/Registers/F31-ControlInProgress.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/platform/EspCompat.hpp"

#ifdef UNIT_TEST_HOST
#include <argparse/argparse.hpp>
#include "tether/hal/IEthernet.hpp"
#include <thread>
#include <atomic>
#include <dirent.h>
#include <iostream>
#endif

static const char* TAG = "AS715N_CheckError";

using namespace EtherCAT::Drives;

// Forward-declare a small subset of Raw transport helpers used by the host example
namespace EtherCAT {
namespace Raw {
    void set_network_interface(const ::EtherCAT::NetworkInterface* iface);
    const ::EtherCAT::NetworkInterface* network_interface();
    void set_src_mac(const uint8_t src_mac[6]);
    const uint8_t* get_src_mac();
}
}

static void printAS715NErrorDetails(EtherCAT::SDO::SDOManager& sdo, uint16_t slave_idx, uint16_t mfr_error, uint16_t cia402_error) {
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

static int inspectAndMaybeReset(EtherCAT::EtherCATMaster& master, bool do_reset, bool do_sw_reset) {
    auto& sdo = master.sdoManager();
    uint16_t slave_count = master.getDiscoveredSlaveCount();
    if (slave_count == 0) {
        TETHER_LOGE(TAG, "No slaves discovered");
        return 2;
    }

    // For simplicity target slave 0 (common usage in examples). If multiple
    // slaves exist, we will still only inspect slave index 0.
    uint16_t slave_idx = 0;

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
        if (!sdo.writeU16(slave_idx, SwReg.index, SwReg.subindex, 1, 3000)) {
            TETHER_LOGE(TAG, "Slave %u: Failed to write %04X:%02X = 1 (software reset)",
                       slave_idx, SwReg.index, SwReg.subindex);
            return 3;
        }
        TETHER_LOGI(TAG, "Software reset command sent to slave %u", slave_idx);
        return 0;
    }

    // When performing a control-word based reset (-r), ensure Switch-On bit is cleared
    if (do_reset) {
        uint16_t cw = 0;
        if (!sdo.readU16(slave_idx, 0x6040, 0x00, cw, 3000)) {
            TETHER_LOGE(TAG, "Slave %u: failed to read Controlword (0x6040) — cannot proceed with -r reset", slave_idx);
            return 3;
        }
        uint16_t new_cw = static_cast<uint16_t>(cw & ~static_cast<uint16_t>(0x0001)); // clear Switch-On bit (bit 0)
        if (new_cw != cw) {
            TETHER_LOGI(TAG, "Slave %u: clearing Switch-On bit in Controlword (0x6040): 0x%04X -> 0x%04X", slave_idx, cw, new_cw);
            if (!sdo.writeU16(slave_idx, 0x6040, 0x00, new_cw, 3000)) {
                TETHER_LOGE(TAG, "Slave %u: failed to write Controlword (0x6040) to clear Switch-On bit", slave_idx);
                return 3;
            }
            Tether::Platform::Clock::instance().delayMilliseconds(50);
        } else {
            TETHER_LOGD(TAG, "Slave %u: Switch-On bit already cleared (Controlword=0x%04X)", slave_idx, cw);
        }
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
        if (!sdo.writeU16(slave_idx, Reg.index, Reg.subindex, 0, 3000)) {
            TETHER_LOGE(TAG, "Slave %u: Failed to write %04X:%02X = 0", slave_idx, Reg.index, Reg.subindex);
            reset_ok = false;
        } else {
            Tether::Platform::Clock::instance().delayMilliseconds(50);
            if (!sdo.writeU16(slave_idx, Reg.index, Reg.subindex, 1, 3000)) {
                TETHER_LOGE(TAG, "Slave %u: Failed to write %04X:%02X = 1", slave_idx, Reg.index, Reg.subindex);
                reset_ok = false;
            } else {
                Tether::Platform::Clock::instance().delayMilliseconds(200);
                (void)sdo.writeU16(slave_idx, Reg.index, Reg.subindex, 0, 3000);
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

#ifndef UNIT_TEST_HOST
extern "C" void as715n_check_error_code_main(const EtherCAT::NetworkInterface* iface,
                                               const uint8_t src_mac[6]) {
    TETHER_LOGI(TAG, "AS715N error-code inspector (embedded)");

    EtherCAT::EtherCATMaster::Config cfg;
    EtherCAT::EtherCATMaster master(cfg);

    if (!iface || !src_mac) { TETHER_LOGE(TAG, "No network interface registered"); return; }
    master.start(*iface, src_mac);

    vTaskDelay(pdMS_TO_TICKS(500));

    // Do not attempt reset by default in embedded entry
    int rc = inspectAndMaybeReset(master, false, false);
    (void)rc;
}

#else // UNIT_TEST_HOST

int main(int argc, char** argv) {
    argparse::ArgumentParser program("as715n_check_error_code");

    program.add_argument("-i", "--interface").default_value(std::string("eth0"))
        .help("Network interface name for host builds (e.g. eth0)");

    program.add_argument("-r","--reset").default_value(false).implicit_value(true)
        .help("Attempt to reset the reported error (if recoverable)");

    program.add_argument("-s","--software-reset").default_value(false).implicit_value(true)
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

    TETHER_LOGI(TAG, "AS715N error-code inspector (host)\nNetwork interface: %s", iface.c_str());

    auto eth = EtherCAT::HAL::createDefaultEthernet();
    if (!eth) { TETHER_LOGE(TAG, "No Ethernet HAL available"); return 1; }

    EtherCAT::HAL::EthernetConfig cfg;
    cfg.interfaceName = iface.c_str();
    cfg.promiscuous = true;
    cfg.ethertypeFilter = static_cast<uint16_t>(EtherCAT::kEtherTypeEtherCAT);

    {
        auto err = eth->init(cfg);
        if (err != EtherCAT::HAL::Error::OK) {
            if (err == EtherCAT::HAL::Error::InterfaceNotFound) {
                TETHER_LOGE(TAG, "Network interface '%s' not found — verify interface name (run: `ip link`)", iface.c_str());
            } else if (err == EtherCAT::HAL::Error::PermissionDenied) {
                TETHER_LOGE(TAG, "Permission denied while opening interface '%s' — raw sockets require root or CAP_NET_RAW (try: sudo or `sudo setcap cap_net_raw+ep %s`)", iface.c_str(), argv[0]);
            } else {
                TETHER_LOGE(TAG, "Failed to init Ethernet interface '%s' (%s)", iface.c_str(), EtherCAT::HAL::errorToString(err));
            }
            return 2;
        }

        // Fail fast when the physical link is down — avoids repeated send failures
        {
            EtherCAT::HAL::LinkStatus ls = eth->getLinkStatus();
            if (!ls.up) {
                TETHER_LOGE(TAG, "Network interface '%s' link is DOWN — check cable/driver", iface.c_str());
                return 6; // distinct, non-retry exit code for link-down
            }
        }
    }

    EtherCAT::HAL::MacAddress mac;
    if (eth->getMacAddress(mac) != EtherCAT::HAL::Error::OK) {
        TETHER_LOGE(TAG, "Failed to read MAC address");
        return 3;
    }

    uint8_t src_mac[6]; std::memcpy(src_mac, mac.bytes, 6);

    auto ni_ptr = std::make_unique<EtherCAT::NetworkInterface>();
    ni_ptr->send = [eth = eth.get()](const uint8_t* data, size_t len) -> bool {
        return eth->transmit(data, len) == EtherCAT::HAL::Error::OK;
    };

    EtherCAT::Raw::set_network_interface(ni_ptr.get());
    EtherCAT::Raw::set_src_mac(src_mac);

    EtherCAT::EtherCATMaster::Config mcfg;
    mcfg.enable_mailbox_fallback = true;  // Enable fallback to default mailbox on configuration errors
    EtherCAT::EtherCATMaster master(mcfg);

    // Route RX frames directly to master — avoids the fragile findByNetworkInterface lookup
    eth->setRxCallback([&master](const uint8_t* frame, size_t len, const EtherCAT::HAL::RxFrameInfo& info, void*){
        (void)info; master.handleRxFrame(frame, len);
    }, nullptr);

    std::atomic<bool> poll_running{true};
    std::thread poll_thread([&](){
        // Best-effort: set polling thread to realtime on Linux
        if (!Tether::Platform::setCurrentThreadRealtime(-1)) {
            TETHER_LOGW(TAG, "poll_thread: could not set realtime scheduling (continuing)");
        }
        while (poll_running.load()) eth->poll(1);
    });

    master.start(*EtherCAT::Raw::network_interface(), src_mac);

    // Allow discovery
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    uint16_t slaves = master.getDiscoveredSlaveCount();
    TETHER_LOGI(TAG, "Discovered %u slave(s)", slaves);
    if (slaves == 0) {
        poll_running.store(false);
        poll_thread.join();
        eth->shutdown();
        return 5;
    }

    int rc = inspectAndMaybeReset(master, do_reset, do_sw_reset);

    // Cleanup
    poll_running.store(false);
    poll_thread.join();
    eth->shutdown();

    return rc;
}

#endif // UNIT_TEST_HOST
