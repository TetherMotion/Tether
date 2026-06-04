#include "tether/ethercat/EtherCATDiagnostics.hpp"

#include "logging/Logger.hpp"
#include "tether/ethercat/EtherCATFaultDetection.hpp"
#include "tether/ethercat/EtherCATSDO.hpp"
#include "tether/sii/SIIReader.hpp"

namespace {

constexpr uint16_t kAlStatusRegister = 0x0130;
constexpr uint16_t kAlStatusCodeRegister = 0x0134;
constexpr uint16_t kSm0BaseRegister = 0x0800;
constexpr uint16_t kSm1BaseRegister = 0x0808;

void logSyncManagerRegister(EtherCAT::EtherCATMaster& master,
                            uint16_t slave_index,
                            uint16_t base_register,
                            const char* sync_manager_name,
                            const char* tag)
{
    uint8_t buffer[8] = {0};
    if (master.readRegister(slave_index, base_register, buffer, sizeof(buffer), 200)) {
        const uint16_t start = static_cast<uint16_t>(buffer[0] | (buffer[1] << 8));
        const uint16_t length = static_cast<uint16_t>(buffer[2] | (buffer[3] << 8));
        TETHER_LOGI(tag, "[PREINIT] ESC %s: start=0x%04X len=%u ctrl=0x%02X act=0x%02X",
                    sync_manager_name,
                    start,
                    static_cast<unsigned>(length),
                    buffer[4],
                    buffer[6]);
        return;
    }

    TETHER_LOGD(tag, "[PREINIT] ESC %s read failed (may be empty in INIT)", sync_manager_name);
}

} // namespace

namespace EtherCAT::Diagnostics {

void logPreOperationalMailboxDiagnostics(
    EtherCATMaster& master,
    uint16_t slave_index,
    const char* tag,
    const PreOperationalMailboxDiagnosticsOptions& options)
{
    if (options.attempt_auto_configure) {
        TETHER_LOGI(tag, "[PREINIT] Attempting mailbox auto-configuration from SII for slave %u (before PRE_OP)", slave_index);
        if (master.autoConfigureMailbox(slave_index, options.auto_configure_log_level)) {
            TETHER_LOGI(tag, "[PREINIT] autoConfigureMailbox: SUCCESS for slave %u", slave_index);
        } else {
            TETHER_LOGW(tag, "[PREINIT] autoConfigureMailbox: FAILED or partial for slave %u - will continue and allow fallback at PRE_OP", slave_index);
        }
    }

    if (options.log_sdo_mailbox_config) {
        uint16_t write_address = 0;
        uint16_t write_length = 0;
        uint16_t read_address = 0;
        uint16_t read_length = 0;
        if (master.sdoManager().getSlaveMailbox(slave_index,
                                                &write_address,
                                                &write_length,
                                                &read_address,
                                                &read_length)) {
            TETHER_LOGI(tag, "[PREINIT] SDO mailbox (post-auto-config): Send(MbxOut/SM0, S->M)=0x%04X/%u   Receive(MbxIn/SM1, M->S)=0x%04X/%u",
                        read_address,
                        static_cast<unsigned>(read_length),
                        write_address,
                        static_cast<unsigned>(write_length));
            if (write_address == 0 || read_address == 0) {
                TETHER_LOGW(tag, "[PREINIT] SDO mailbox contains zero addresses (possible fallback or incomplete config)");
            }
        } else {
            TETHER_LOGW(tag, "[PREINIT] SDO subsystem has NO mailbox config after autoConfigureMailbox() (SDO may be unavailable)");
        }
    }

    if (options.log_sii_mailbox) {
        EtherCAT::SII::SIIMailboxConfig mailbox{};
        if (EtherCAT::SII::readSIIMailbox(master, slave_index, mailbox)) {
            TETHER_LOGI(tag, "[PREINIT] SII Mailbox (parsed): RX(off=0x%04X size=%u)  TX(off=0x%04X size=%u)  proto=0x%04X",
                        mailbox.std_rx_offset,
                        static_cast<unsigned>(mailbox.std_rx_size),
                        mailbox.std_tx_offset,
                        static_cast<unsigned>(mailbox.std_tx_size),
                        mailbox.protocols);
        } else {
            TETHER_LOGD(tag, "[PREINIT] Unable to read SII mailbox category (this is OK; will use HW/defaults)");
        }
    }

    if (options.log_sync_manager_registers) {
        logSyncManagerRegister(master, slave_index, kSm0BaseRegister, "SM0", tag);
        logSyncManagerRegister(master, slave_index, kSm1BaseRegister, "SM1", tag);
    }
}

bool logParsedSlaveSII(
    EtherCATMaster& master,
    uint16_t slave_index,
    const char* tag)
{
    EtherCAT::SII::SIIData sii{};
    if (!EtherCAT::SII::readSII(master, slave_index, sii)) {
        TETHER_LOGW(tag, "Failed to read/parse SII for slave %u", slave_index);
        return false;
    }

    TETHER_LOGI(tag, "--- Parsed SII for slave %u (checksum: %s) ---",
                slave_index,
                sii.checksum_ok ? "OK" : "INVALID");
    EtherCAT::SII::logSIIData(sii, tag);
    return true;
}

void logSlaveApplicationLayerDiagnostics(
    EtherCATMaster& master,
    uint16_t slave_index,
    const char* tag)
{
    uint8_t state = 0;
    if (master.readSlaveApplicationLayerState(slave_index, state)) {
        TETHER_LOGW(tag, "Slave %u current EC state: 0x%02X (%s)",
                    slave_index,
                    state,
                    EtherCAT::EtherCATMaster::getECStateName(state));
    } else {
        TETHER_LOGW(tag, "Slave %u: failed to read EC state (possible WKC=0 / transport issue)", slave_index);
    }

    uint16_t al_status = 0;
    uint16_t al_code = 0;
    const bool have_al_status = master.readRegister(slave_index, kAlStatusRegister, al_status, 200);
    const bool have_al_code = master.readRegister(slave_index, kAlStatusCodeRegister, al_code, 200);

    if (have_al_status) {
        TETHER_LOGW(tag, "Slave %u AL_STATUS: 0x%04X (%s)%s",
                    slave_index,
                    al_status,
                    al_status_get_state_name(al_status),
                    al_status_has_error(al_status) ? ", ERROR" : "");
    } else {
        TETHER_LOGW(tag, "Slave %u AL_STATUS read FAILED (APRD) - likely WKC=0 or no response", slave_index);
    }

    if (have_al_code) {
        TETHER_LOGW(tag, "Slave %u AL_STATUS_CODE: 0x%04X (%s)",
                    slave_index,
                    al_code,
                    getALStatusCodeName(al_code));
    } else {
        TETHER_LOGW(tag, "Slave %u AL_STATUS_CODE read FAILED (APRD)", slave_index);
    }
}

} // namespace EtherCAT::Diagnostics