#include "raw/internal.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/platform/Platform.hpp"
#include "tether/sii/SIIReader.hpp"
#include "tether/ethercat/PDOManager.hpp"

namespace EtherCAT {
namespace Raw {

static const char *TAG = "ethercat_sii";

/**
 * @brief Configure mailbox from SII EEPROM data
 * 
 * Reads the SII EEPROM mailbox configuration and extracts mailbox addresses,
 * sizes, and supported protocols. Falls back to conservative defaults if:
 * - SII reading fails
 * - No mailbox is configured in SII
 * - Invalid/inconsistent mailbox data
 * 
 * Per standard EtherCAT convention:
 * - SM0 = Receive mailbox (MbxIn) = Master→Slave = std_rx in SII
 * - SM1 = Send mailbox (MbxOut) = Slave→Master = std_tx in SII
 * 
 * @param master Master instance for network I/O
 * @param adp Auto-increment address (slave position)
 * @param out_wr_addr Receive mailbox address (MbxIn, Master→Slave, SM0, std_rx)
 * @param out_wr_len Receive mailbox size in bytes
 * @param out_rd_addr Send mailbox address (MbxOut, Slave→Master, SM1, std_tx)
 * @param out_rd_len Send mailbox size in bytes
 * @param out_mbx_proto Supported mailbox protocol flags
 * @return true if mailbox configuration was read (or defaults applied), false on critical error
 */
bool configure_mailbox_from_sii(
    Master& master,
    uint16_t slave_index,
    uint16_t *out_wr_addr,
    uint16_t *out_wr_len,
    uint16_t *out_rd_addr,
    uint16_t *out_rd_len,
    uint16_t *out_mbx_proto)
{
    // Default conservative mailbox (standard EtherCAT convention):
    // SM0: Receive (MbxIn, Master→Slave)
    // SM1: Send    (MbxOut, Slave→Master)
    constexpr uint16_t kDefaultReceiveAddr = 0x1000;  // Receive/MbxIn (M→S)
    constexpr uint16_t kDefaultReceiveLen = 256;
    constexpr uint16_t kDefaultSendAddr = 0x1400;     // Send/MbxOut (S→M)
    constexpr uint16_t kDefaultSendLen = 256;
    constexpr uint16_t kDefaultProto = static_cast<uint16_t>(
        EtherCAT::SII::MBX_PROTO_COE | EtherCAT::SII::MBX_PROTO_EOE | EtherCAT::SII::MBX_PROTO_AOE
    );

    // Try to read SII mailbox configuration
    EtherCAT::SII::SIIMailboxConfig mailbox;
    EtherCAT::SII::SIIReader sii_reader(master);
    EtherCAT::SII::SIIParser sii_parser(sii_reader);
    EtherCAT::SII::SIIData sii_data;
    bool sii_ok = sii_parser.parseIdentity(slave_index, sii_data);
    if (sii_ok) {
        mailbox = sii_data.mailbox;
    }
    if (!sii_ok) {
        const char* err = sii_parser.lastError();
        if (err && err[0]) {
            TETHER_LOGW(TAG, "Slave %u: SII mailbox read failed: %s — using defaults", slave_index, err);
        } else {
            TETHER_LOGW(TAG, "Slave %u: SII mailbox read failed — using defaults", slave_index);
        }
    }
    
    // Determine if we have valid mailbox data
    bool valid_mailbox = sii_ok && mailbox.hasMailbox();
    
    // Validate mailbox configuration if present
    if (valid_mailbox) {
        // Check for obviously invalid configurations
        if (mailbox.std_rx_size == 0 || mailbox.std_tx_size == 0) {
            TETHER_LOGW(TAG, "SII mailbox has zero size (Receive=%u Send=%u), using defaults",
                     mailbox.std_rx_size, mailbox.std_tx_size);
            valid_mailbox = false;
        }
        // Check for suspiciously small mailboxes (< 32 bytes is unusual)
        else if (mailbox.std_rx_size < 32 || mailbox.std_tx_size < 32) {
            TETHER_LOGW(TAG, "SII mailbox unusually small (Receive=%u Send=%u), but accepting",
                     mailbox.std_rx_size, mailbox.std_tx_size);
        }
        // Check for overlapping mailboxes
        else if (mailbox.std_rx_offset == mailbox.std_tx_offset) {
            TETHER_LOGW(TAG, "SII mailbox Receive and Send have same address (0x%04X), using defaults",
                     mailbox.std_rx_offset);
            valid_mailbox = false;
        }
    }
    
    // Use SII data or defaults
    // Variable names use EtherCAT spec terminology (slave perspective):
    // - mbx_receive = Receive mailbox (MbxIn) = Master→Slave = SM0 = std_rx
    // - mbx_send = Send mailbox (MbxOut) = Slave→Master = SM1 = std_tx
    uint16_t mbx_receive_addr, mbx_receive_len, mbx_send_addr, mbx_send_len, proto;
    
    if (valid_mailbox) {
        // Use SII mailbox configuration
        mbx_receive_addr = mailbox.std_rx_offset;  // Receive (MbxIn, M→S, SM0)
        mbx_receive_len = mailbox.std_rx_size;
        mbx_send_addr = mailbox.std_tx_offset;  // Send (MbxOut, S→M, SM1)
        mbx_send_len = mailbox.std_tx_size;
        proto = mailbox.protocols;
        
        TETHER_LOGI(TAG, "Mailbox from SII: Receive(SM0/M→S)=0x%04X/%u Send(SM1/S→M)=0x%04X/%u proto=0x%04X",
                 mbx_receive_addr, (unsigned)mbx_receive_len, mbx_send_addr, (unsigned)mbx_send_len, proto);
    } else {
        // Use defaults
        mbx_receive_addr = kDefaultReceiveAddr;
        mbx_receive_len = kDefaultReceiveLen;
        mbx_send_addr = kDefaultSendAddr;
        mbx_send_len = kDefaultSendLen;
        proto = kDefaultProto;
        
        if (!sii_ok) {
            TETHER_LOGW(TAG, "Failed to read SII, using defaults: Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u proto=0x%04X",
                     mbx_receive_addr, (unsigned)mbx_receive_len, mbx_send_addr, (unsigned)mbx_send_len, proto);
        } else {
            TETHER_LOGW(TAG, "No mailbox in SII, using defaults: Receive(SM0)=0x%04X/%u Send(SM1)=0x%04X/%u proto=0x%04X",
                     mbx_receive_addr, (unsigned)mbx_receive_len, mbx_send_addr, (unsigned)mbx_send_len, proto);
        }
    }

    // Validate address ordering: SM0 (Receive) should typically be at lower address than SM1 (Send)
    if (mbx_receive_addr >= mbx_send_addr) {
        TETHER_LOGW(TAG, "Mailbox addresses non-standard: Receive(SM0)=0x%04X >= Send(SM1)=0x%04X; typical is SM0 < SM1",
                    mbx_receive_addr, mbx_send_addr);
    }

    // Write output parameters
    // Output param names use master perspective (wr/rd);
    // internally we use proper EtherCAT "Receive/Send" terminology (slave perspective)
    if (out_wr_addr) *out_wr_addr = mbx_receive_addr;  // Master writes = Slave receives = Receive/MbxIn/SM0
    if (out_wr_len)  *out_wr_len = mbx_receive_len;
    if (out_rd_addr) *out_rd_addr = mbx_send_addr;     // Master reads = Slave sends = Send/MbxOut/SM1
    if (out_rd_len)  *out_rd_len = mbx_send_len;
    if (out_mbx_proto) *out_mbx_proto = proto;

    // -------------------------------------------------------------------------
    // Cross-check against actual Sync Manager (SM0/SM1) register configuration.
    // Some devices (or some EEPROM contents) have swapped RX/TX mailbox offsets
    // in SII. The authoritative config is what the ESC exposes in SM registers.
    // If we can read SM0/SM1 and they clearly indicate mailbox mode with
    // SLAVE->MASTER and MASTER->SLAVE directions, prefer those addresses.
    //
    // This block is intentionally verbose to allow one-shot field debugging.
    // -------------------------------------------------------------------------
    struct SmRegs {
        bool ok{false};
        uint16_t start{0};
        uint16_t len{0};
        uint8_t control{0};
        uint8_t status{0};
        uint8_t activate{0};
        uint8_t pdi_ctrl{0};
    };

    auto read_sm = [&](uint16_t sm_base, unsigned sm_index) -> SmRegs {
        SmRegs r;
        uint8_t buf[8] = {0};
        if (!master.readRegister(EtherCAT::SlaveAddress(slave_index), sm_base, buf, sizeof(buf), 200)) {
            TETHER_LOGD(TAG, "[MBOXTRACE] SM%u reg read failed (slave=%u base=0x%04X)", sm_index, (unsigned)slave_index, (unsigned)sm_base);
            return r;
        }

        r.ok = true;
        r.start = static_cast<uint16_t>(buf[0] | (static_cast<uint16_t>(buf[1]) << 8));
        r.len = static_cast<uint16_t>(buf[2] | (static_cast<uint16_t>(buf[3]) << 8));
        r.control = buf[4];
        r.status = buf[5];
        r.activate = buf[6];
        r.pdi_ctrl = buf[7];
        return r;
    };

    auto fmt_dir = [&](uint8_t ctrl) -> const char* {
        using namespace EtherCAT::PDO;
        return ((ctrl & SM_CTRL_DIR_WRITE) != 0) ? "MASTER->SLAVE" : "SLAVE->MASTER";
    };
    auto is_mailbox = [&](uint8_t ctrl) -> bool {
        using namespace EtherCAT::PDO;
        return ((ctrl & SM_CTRL_MODE_MASK) == SM_CTRL_MODE_MAILBOX);
    };

    const SmRegs sm0 = read_sm(EC_REG_SM0, 0);
    const SmRegs sm1 = read_sm(EC_REG_SM1, 1);

    if (sm0.ok && sm1.ok) {
        TETHER_LOGD(TAG, "[MBOXTRACE] SII-derived mailbox: Receive(SM0/M→S)=0x%04X/%u Send(SM1/S→M)=0x%04X/%u proto=0x%04X valid_sii=%s\n[MBOXTRACE] SM0(Receive/MbxIn): start=0x%04X len=%u ctrl=0x%02X dir=%s act=0x%02X stat=0x%02X pdi=0x%02X\n[MBOXTRACE] SM1(Send/MbxOut): start=0x%04X len=%u ctrl=0x%02X dir=%s act=0x%02X stat=0x%02X pdi=0x%02X",
                    mbx_receive_addr, (unsigned)mbx_receive_len, mbx_send_addr, (unsigned)mbx_send_len, proto, valid_mailbox ? "yes" : "no",
                    sm0.start, (unsigned)sm0.len, sm0.control, fmt_dir(sm0.control), sm0.activate, sm0.status, sm0.pdi_ctrl,
                    sm1.start, (unsigned)sm1.len, sm1.control, fmt_dir(sm1.control), sm1.activate, sm1.status, sm1.pdi_ctrl);

        const bool sm0_mbx = is_mailbox(sm0.control) && (sm0.activate & 0x01);
        const bool sm1_mbx = is_mailbox(sm1.control) && (sm1.activate & 0x01);

        // Validate SM controls match expected mailbox configuration
        auto ctrl_mode_name = [&](uint8_t ctrl) -> const char* {
            using namespace EtherCAT::PDO;
            switch (ctrl & SM_CTRL_MODE_MASK) {
                case SM_CTRL_MODE_BUFFERED: return "Buffered";
                case SM_CTRL_MODE_MAILBOX:  return "Mailbox";
                case SM_CTRL_MODE_3PDO:     return "Buffered3PDO";
                default:                    return "Reserved";
            }
        };
        if (sm0.ok && !sm0_mbx) {
            TETHER_LOGW(TAG, "SM0 control byte 0x%02X does not indicate an active mailbox (mode=%s, dir=%s)",
                        sm0.control, ctrl_mode_name(sm0.control), fmt_dir(sm0.control));
        }
        if (sm1.ok && !sm1_mbx) {
            TETHER_LOGW(TAG, "SM1 control byte 0x%02X does not indicate an active mailbox (mode=%s, dir=%s)",
                        sm1.control, ctrl_mode_name(sm1.control), fmt_dir(sm1.control));
        }

        // Cross-check SM registers intelligently, accounting for swap correction applied above.
        // Only attempt cross-check when both SM0/SM1 look like enabled mailbox SMs.
        if (sm0_mbx && sm1_mbx && sm0.start != 0 && sm1.start != 0 && sm0.len != 0 && sm1.len != 0) {
            // Determine what the hardware SM registers indicate based on direction bits.
            // Per EtherCAT ESC specification:
            // - SM_CTRL_DIR_WRITE (bit2=1): ECAT writes, PDI reads → M→S → Receive mailbox (MbxIn, SM0)
            // - SM_CTRL_DIR_READ  (bit2=0): ECAT reads, PDI writes → S→M → Send mailbox (MbxOut, SM1)
            const uint16_t hw_receive_addr = (((sm0.control & EtherCAT::PDO::SM_CTRL_DIR_WRITE) != 0) ? sm0.start : sm1.start);
            const uint16_t hw_receive_len  = (((sm0.control & EtherCAT::PDO::SM_CTRL_DIR_WRITE) != 0) ? sm0.len   : sm1.len);
            const uint16_t hw_send_addr = (((sm0.control & EtherCAT::PDO::SM_CTRL_DIR_WRITE) == 0) ? sm0.start : sm1.start);
            const uint16_t hw_send_len  = (((sm0.control & EtherCAT::PDO::SM_CTRL_DIR_WRITE) == 0) ? sm0.len   : sm1.len);

            // Compare hardware SM registers to SII values but DO NOT modify output addresses.
            if (hw_receive_addr == mbx_receive_addr && hw_receive_len == mbx_receive_len &&
                hw_send_addr == mbx_send_addr && hw_send_len == mbx_send_len) {
                TETHER_LOGD(TAG, "[MBOXTRACE] Hardware SM registers match SII mailbox config ✓");
            } else {
                TETHER_LOGW(TAG, "[MBOXTRACE] Hardware SM differs from SII (hw: Receive=0x%04X/%u Send=0x%04X/%u vs sii: Receive=0x%04X/%u Send=0x%04X/%u)",
                           hw_receive_addr, (unsigned)hw_receive_len, hw_send_addr, (unsigned)hw_send_len,
                           mbx_receive_addr, (unsigned)mbx_receive_len, mbx_send_addr, (unsigned)mbx_send_len);
            }
        } else {
            TETHER_LOGD(TAG, "[MBOXTRACE] SM0/SM1 not both active mailbox SMs (sm0_mbx=%s sm1_mbx=%s); trusting SII",
                        sm0_mbx ? "yes" : "no", sm1_mbx ? "yes" : "no");
        }
    }

    return true;
}

bool sii_read_string(Master& master, uint16_t slave_index, uint16_t string_number, char *out, size_t out_cap) {
    (void)master;
    (void)slave_index;
    (void)string_number;
    if (out) *out = '\0';
    return false;
}

} // namespace Raw

namespace raw {
bool sii_read_string(Master& master, uint16_t slave_index, uint16_t string_number, char *out, size_t out_cap) {
    // Forward to the capitalized Raw implementation
    return Raw::sii_read_string(master, slave_index, string_number, out, out_cap);
}

} // namespace raw
} // namespace EtherCAT
