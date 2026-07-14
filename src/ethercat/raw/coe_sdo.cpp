#include "raw/internal.hpp"
#include "tether/platform/EspCompat.hpp"
#include "tether/ethercat/Platform.hpp"
#include "tether/ethercat/DebugFlags.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/ethercat/Master.hpp"
#include <thread>
#include <chrono>
#include <cinttypes>

namespace EtherCAT {
namespace Raw {

static const char *TAG = "ethercat";

// ============================================================================
// SDO abort code decoding
// ============================================================================

static const char* sdo_abort_code_str(uint32_t code) {
    switch (code) {
        case 0x05030000: return "Toggle bit not alternated";
        case 0x05040000: return "SDO protocol timeout";
        case 0x05040001: return "Invalid command";
        case 0x05040002: return "Invalid block size";
        case 0x05040003: return "Invalid sequence number";
        case 0x05040004: return "CRC error";
        case 0x05040005: return "Out of memory";
        case 0x06010000: return "Unsupported access";
        case 0x06010001: return "Write to read-only object";
        case 0x06010002: return "Read from write-only object";
        case 0x06020000: return "Object does not exist";
        case 0x06040041: return "Object cannot be mapped to PDO";
        case 0x06040042: return "PDO length exceeded";
        case 0x06040043: return "Parameter incompatibility";
        case 0x06040047: return "General internal incompatibility";
        case 0x06060000: return "Hardware error";
        case 0x06070010: return "Data type mismatch, length mismatch";
        case 0x06070012: return "Data type mismatch, length too high";
        case 0x06070013: return "Data type mismatch, length too low";
        case 0x06090011: return "Subindex does not exist";
        case 0x06090030: return "Invalid value for parameter";
        case 0x06090031: return "Value too high";
        case 0x06090032: return "Value too low";
        case 0x06090036: return "Maximum less than minimum";
        case 0x060A0023: return "Resource not available";
        case 0x08000000: return "General error";
        case 0x08000020: return "Data transfer aborted";
        case 0x08000021: return "Local control error";
        case 0x08000022: return "Wrong device state";
        case 0x08000023: return "Object dictionary not present";
        case 0x08000024: return "No data available";
        default:        return "Unknown abort code";
    }
}

static bool is_pdo_mapping_index(uint16_t idx) {
    return (idx >= 0x1600 && idx <= 0x167F) ||
           (idx >= 0x1A00 && idx <= 0x1A7F);
}

// When a SubindexNotFound abort occurs on a PDO mapping object, attempt to
// read subindex 0 (entry count) to tell the user the max supported entries.
static void log_pdo_mapping_subindex_diagnostic(
    Master& master, uint16_t adp, uint8_t* inout_mbx_cnt,
    uint16_t mbx_write_addr, uint16_t mbx_write_len,
    uint16_t mbx_read_addr, uint16_t mbx_read_len,
    uint16_t index, uint8_t sub,
    bool diag_enabled, unsigned int poll_interval_ms,
    unsigned int transaction_timeout_ms)
{
    if (sub == 0) return;

    uint8_t count_buf[4] = {0};
    size_t count_len = 0;
    bool ok = coe_sdo_upload(master, adp, inout_mbx_cnt,
                             mbx_write_addr, mbx_write_len,
                             mbx_read_addr, mbx_read_len,
                             index, 0x00,
                             count_buf, sizeof(count_buf), &count_len,
                             diag_enabled, poll_interval_ms, transaction_timeout_ms);
    if (ok && count_len >= 1) {
        TETHER_LOGE(TAG, "  -> PDO mapping object 0x%04X supports max %u entries (subindices 1-%u), but subindex %u was accessed",
                    index, static_cast<unsigned>(count_buf[0]),
                    static_cast<unsigned>(count_buf[0]),
                    static_cast<unsigned>(sub));
    } else {
        TETHER_LOGE(TAG, "  -> PDO mapping object 0x%04X: subindex %u does not exist (failed to read max entry count from subindex 0)",
                    index, static_cast<unsigned>(sub));
    }
}

// Compile-time option to enable low-level SDO mailbox diagnostics.
// Define TETHER_DIAG_SDO_IO at build time to enable verbose dumps of mailbox
// writes/reads and related SM/AL status useful for debugging SDO failures.
#ifdef TETHER_DIAG_SDO_IO
#include <cstdio>
static void diag_hexdump(const uint8_t *data, size_t len, size_t max_print = 64) {
    char buf[256];
    size_t to_print = (len < max_print) ? len : max_print;
    buf[0] = '\0';
    for (size_t i = 0; i < to_print; ++i) {
        char tmp[8];
        snprintf(tmp, sizeof(tmp), "%02X", data[i]);
        if (i) strncat(buf, " ", sizeof(buf) - strlen(buf) - 1);
        strncat(buf, tmp, sizeof(buf) - strlen(buf) - 1);
    }
    if (len > to_print) strncat(buf, " ...", sizeof(buf) - strlen(buf) - 1);
    TETHER_LOGI(TAG, "MBX data (%zu bytes): %s", len, buf);
}
#endif

// Check SM1 status for stale mailbox data from a previous unfinished request.
// If present, read and discard it to clear the mailbox state.
// Note: we must NOT write to SM1 (the slave's transmit mailbox) — the ESC
// rejects PWR to a read-direction sync manager with wkc=0.  Reading the
// stale data is sufficient to clear the mailbox full flag.
//
// This function loops to drain ALL pending stale responses, not just one.
// The slave may have multiple queued responses from previously timed-out
// requests.  Each read from SM1 clears the mailbox full flag, allowing the
// slave to write the next queued response.  We keep draining until SM1 is
// no longer full or we hit the max drain limit.
static void mbx_drain_all_stale(Master& master, uint16_t adp,
                                uint16_t mbx_read_addr, uint16_t mbx_read_len,
                                unsigned int max_drain = 16)
{
    for (unsigned int i = 0; i < max_drain; ++i) {
        uint8_t sm1_status = 0;
        if (!master.readRegister(Master::slaveAddressFromADP(adp), sm_status_address(1), sm1_status, 100)) {
            break;
        }
        if ((sm1_status & EC_SM_STATUS_MBXFULL) == 0) {
            break;
        }
        if (i == 0) {
            TETHER_LOGW(TAG, "Stale mailbox data detected (SM1 full, adp=0x%04X). Draining before new SDO request.",
                        adp);
        }
        uint8_t drain_buf[256] = {0};
        uint16_t drain_len = mbx_read_len;
        if (drain_len > sizeof(drain_buf)) {
            drain_len = static_cast<uint16_t>(sizeof(drain_buf));
        }
        (void)master.readRegister(Master::slaveAddressFromADP(adp), mbx_read_addr, drain_buf, drain_len, 200);
        TETHER_LOGW(TAG, "Drained stale mailbox data #%u (adp=0x%04X)", i + 1, adp);
    }
}

static void sm_control_str(uint8_t ctrl_byte, char* buf, size_t buflen)
{
    const auto ctrl = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(ctrl_byte);
    size_t pos = 0;

    const char* mode = "UNKNOWN";
    if (ctrl.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Buffered)) mode = "BUFFERED";
    else if (ctrl.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Mailbox))  mode = "MAILBOX";
    else if (ctrl.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::ThreePDO)) mode = "THREE_PDO";

    pos += snprintf(buf + pos, buflen - pos, "%s %s", mode, ctrl.direction ? "M->S" : "S->M");
    if (ctrl.ecat_irq)   pos += snprintf(buf + pos, buflen - pos, " IRQ_ECAT");
    if (ctrl.pdi_irq)    pos += snprintf(buf + pos, buflen - pos, " IRQ_PDI");
    if (ctrl.watchdog)   pos += snprintf(buf + pos, buflen - pos, " WATCHDOG");
    if (ctrl.repeat_req) pos += snprintf(buf + pos, buflen - pos, " REPEAT_REQ");
}

static void sm_status_str(uint8_t stat_byte, char* buf, size_t buflen)
{
    const auto stat = std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(stat_byte);
    if (stat_byte == 0) {
        snprintf(buf, buflen, "-");
        return;
    }
    size_t pos = 0;
    if (stat.write_event)    pos += snprintf(buf + pos, buflen - pos, "WRITE_EVENT ");
    if (stat.read_event)     pos += snprintf(buf + pos, buflen - pos, "READ_EVENT ");
    if (stat.mailbox_full)   pos += snprintf(buf + pos, buflen - pos, "MBOX_FULL ");
    if (stat.read_buf_full)  pos += snprintf(buf + pos, buflen - pos, "READ_BUF_FULL ");
    if (stat.write_buf_full) pos += snprintf(buf + pos, buflen - pos, "WRITE_BUF_FULL ");
    if (pos > 0 && buf[pos - 1] == ' ') buf[pos - 1] = '\0';
}

static void sm_activate_str(uint8_t act_byte, char* buf, size_t buflen)
{
    const auto act = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(act_byte);
    snprintf(buf, buflen, "%s", act.enable ? "ENABLED" : "disabled");
}

static void mbx_diag_dump_slave_state(Master& master, uint16_t adp, uint16_t mbx_wr_addr, uint16_t mbx_rd_addr)
{
    // Keep this lightweight and only call on errors.
    uint16_t al_status = 0;
    uint16_t al_code = 0;
    uint8_t sm0[8] = {0};
    uint8_t sm1[8] = {0};
    uint8_t sm0_stat_act[2] = {0};
    uint8_t sm1_stat_act[2] = {0};

    (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0130, al_status, 200);
    (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0134, al_code, 200);
    (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0800, sm0, sizeof(sm0), 200);
    (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0808, sm1, sizeof(sm1), 200);
    (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0805, sm0_stat_act, sizeof(sm0_stat_act), 200);
    (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x080D, sm1_stat_act, sizeof(sm1_stat_act), 200);

    const uint16_t al_s = le16_to_host(al_status);
    const uint16_t al_c = le16_to_host(al_code);
    const uint16_t sm0_start = static_cast<uint16_t>(sm0[0] | (static_cast<uint16_t>(sm0[1]) << 8));
    const uint16_t sm0_len   = static_cast<uint16_t>(sm0[2] | (static_cast<uint16_t>(sm0[3]) << 8));
    const uint8_t  sm0_ctrl  = sm0[4];
    const uint8_t  sm0_stat  = sm0[5];
    const uint8_t  sm0_act   = sm0[6];

    const uint16_t sm1_start = static_cast<uint16_t>(sm1[0] | (static_cast<uint16_t>(sm1[1]) << 8));
    const uint16_t sm1_len   = static_cast<uint16_t>(sm1[2] | (static_cast<uint16_t>(sm1[3]) << 8));
    const uint8_t  sm1_ctrl  = sm1[4];
    const uint8_t  sm1_stat  = sm1[5];
    const uint8_t  sm1_act   = sm1[6];

    char sm0_ctrl_desc[64];  sm_control_str(sm0_ctrl, sm0_ctrl_desc, sizeof(sm0_ctrl_desc));
    char sm1_ctrl_desc[64];  sm_control_str(sm1_ctrl, sm1_ctrl_desc, sizeof(sm1_ctrl_desc));
    char sm0_stat_desc[64];  sm_status_str(sm0_stat, sm0_stat_desc, sizeof(sm0_stat_desc));
    char sm1_stat_desc[64];  sm_status_str(sm1_stat, sm1_stat_desc, sizeof(sm1_stat_desc));
    char sm0_act_desc[16];   sm_activate_str(sm0_act, sm0_act_desc, sizeof(sm0_act_desc));
    char sm1_act_desc[16];   sm_activate_str(sm1_act, sm1_act_desc, sizeof(sm1_act_desc));

    char sm0_sa_stat_desc[64]; sm_status_str(sm0_stat_act[0], sm0_sa_stat_desc, sizeof(sm0_sa_stat_desc));
    char sm1_sa_stat_desc[64]; sm_status_str(sm1_stat_act[0], sm1_sa_stat_desc, sizeof(sm1_sa_stat_desc));
    char sm0_sa_act_desc[16];  sm_activate_str(sm0_stat_act[1], sm0_sa_act_desc, sizeof(sm0_sa_act_desc));
    char sm1_sa_act_desc[16];  sm_activate_str(sm1_stat_act[1], sm1_sa_act_desc, sizeof(sm1_sa_act_desc));

    TETHER_LOGE(TAG, "[SDO_DIAG] AL_STATUS=0x%04X state=%s%s | AL status code: %s (0x%04X)\n[SDO_DIAG] MBX cfg: wr=0x%04X rd=0x%04X | SM0(start=0x%04X len=%u ctrl=0x%02X [%s] stat=0x%02X act=0x%02X) SM1(start=0x%04X len=%u ctrl=0x%02X [%s] stat=0x%02X act=0x%02X)\n[SDO_DIAG] SM0 status=0x%02X [%s] act=0x%02X [%s] | SM1 status=0x%02X [%s] act=0x%02X [%s]",
               al_s,
               EtherCAT::al_status_get_state_name(al_s),
               EtherCAT::al_status_has_error(al_s) ? " ERROR" : "",
               EtherCAT::getALStatusCodeName(static_cast<EtherCAT::ALStatusCode>(al_c)),
               al_c,
               mbx_wr_addr, mbx_rd_addr,
               sm0_start, (unsigned)sm0_len, sm0_ctrl, sm0_ctrl_desc, sm0_stat, sm0_act,
               sm1_start, (unsigned)sm1_len, sm1_ctrl, sm1_ctrl_desc, sm1_stat, sm1_act,
               sm0_stat_act[0], sm0_sa_stat_desc, sm0_stat_act[1], sm0_sa_act_desc,
               sm1_stat_act[0], sm1_sa_stat_desc, sm1_stat_act[1], sm1_sa_act_desc);
}

static uint16_t slaveIndexFromADP(uint16_t adp) {
    return Master::slaveAddressFromADP(adp).slavePosition();
}

static void logCoeMbxPacket(const char* dir, uint16_t adp, uint16_t index, uint8_t sub,
                            const uint8_t* data, size_t len, bool enabled)
{
    if (!enabled) return;

    TETHER_LOGI(TAG, "[CoE-%s] adp=0x%04X index=0x%04X:%u len=%zu",
                dir, adp, index, sub, len);

    if (len == 0) return;

    bool all_zero = true;
    for (size_t i = 0; i < len; ++i) {
        if (data[i] != 0) { all_zero = false; break; }
    }
    if (all_zero) {
        TETHER_LOGI(TAG, "[CoE-%s] Data (%zu bytes): All zeroes", dir, len);
        return;
    }

    constexpr size_t kMaxHexDump = 64;
    const size_t dump_len = (len < kMaxHexDump) ? len : kMaxHexDump;
    char hexbuf[256];
    size_t pos = 0;
    for (size_t i = 0; i < dump_len && pos + 3 < sizeof(hexbuf); ++i) {
        pos += snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "%02X ", data[i]);
    }
    if (len > kMaxHexDump) {
        pos += snprintf(hexbuf + pos, sizeof(hexbuf) - pos, "...");
    }
    TETHER_LOGI(TAG, "[CoE-%s] Data (%zu/%zu bytes): %s", dir, dump_len, len, hexbuf);
}

static bool mbx_wait_sm0_not_full(Master& master, uint16_t adp,
                                   unsigned int timeout_ms,
                                   unsigned int poll_interval_ms = 5)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (master.isCancelRequested()) {
            return false;
        }
        uint8_t sm0_status = 0;
        if (master.readRegister(Master::slaveAddressFromADP(adp), sm_status_address(0), sm0_status, 100)) {
            if ((sm0_status & EC_SM_STATUS_MBXFULL) == 0) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
    }
    TETHER_LOGE(TAG, "SM0 mailbox stayed full (adp=0x%04X) after %ums timeout — slave PDI not draining mailbox",
                adp, timeout_ms);
    return false;
}

static bool mbx_apwr_with_wkc_probe(
    Master& master,
    uint16_t adp,
    uint16_t primary_addr,
    uint16_t alt_addr,
    const uint8_t* payload,
    uint16_t payload_len,
    unsigned int timeout_ms,
    bool* out_used_alt)
{
    if (out_used_alt) *out_used_alt = false;

    // Wait for SM0 (master→slave mailbox) to be not full before writing.
    // If the slave's PDI hasn't read the previous message yet, the ESC will
    // reject the write (wkc=0) and silently drop the data.
    if (!mbx_wait_sm0_not_full(master, adp, timeout_ms)) {
        TETHER_LOGE(TAG, "mailbox write aborted: SM0 still full (adp=0x%04X addr=0x%04X)", adp, primary_addr);
        return false;
    }

    // Use the master's descriptive write API, which still pre-registers a waiter,
    // the response arrives before we start waiting.
    if (master.writeRegister(Master::slaveAddressFromADP(adp), primary_addr, payload, payload_len, timeout_ms)) {
        return true;
    }

    if (master.lastWkc() == 0) {
        TETHER_LOGE(TAG, "mailbox transaction failed: Working counter is 0 (adp=0x%04X addr=0x%04X)", adp, primary_addr);
        return false;
    }

    // Probe alternate address if primary was not acknowledged after retries.
    TETHER_LOGW(TAG, "SDO mailbox APWR not acknowledged for adp=0x%04X addr=0x%04X (len=%u) after retries. Probing alt addr=0x%04X...",
                adp, primary_addr, (unsigned)payload_len, alt_addr);

    if (master.writeRegister(Master::slaveAddressFromADP(adp), alt_addr, payload, payload_len, timeout_ms)) {
        if (out_used_alt) *out_used_alt = true;
        TETHER_LOGW(TAG, "SDO mailbox APWR acknowledged on alt addr=0x%04X. Treating mailbox wr/rd as swapped for this SDO op.",
                    alt_addr);
        return true;
    }

    // Both failed -> emit a focused state dump.
    TETHER_LOGE(TAG, "SDO mailbox APWR not acknowledged on both addr=0x%04X and alt=0x%04X (adp=0x%04X)",
                primary_addr, alt_addr, adp);
    mbx_diag_dump_slave_state(master, adp, primary_addr, alt_addr);
    return false;
}

/**
 * @brief Poll SyncManager 1 status register until the mailbox is full.
 *
 * After the master writes an SDO request into the slave's RX mailbox (SM0),
 * the slave's PDI must parse the request, fetch data, and write the response
 * into the TX mailbox (SM1).  We know SM1 is ready when bit 3 of the
 * SM1 Status register (ESC address 0x080D) is set.
 *
 * @param master       Master instance
 * @param adp          Auto-increment address of the target slave
 * @param timeout_ms   Maximum time to wait for the mailbox to become full
 * @return true if the mailbox became full within the timeout
 */
static bool mbx_poll_sm1_full(Master& master, uint16_t adp,
                                unsigned int timeout_ms,
                                unsigned int poll_interval_ms = 5)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (master.isCancelRequested()) {
            return false;
        }
        uint8_t sm1_status = 0;
        if (master.readRegister(Master::slaveAddressFromADP(adp), sm_status_address(1), sm1_status, 100)) {
            if ((sm1_status & EC_SM_STATUS_MBXFULL) != 0) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
    }
    return false;
}

bool coe_sdo_upload(
    Master& master,
    uint16_t adp,
    uint8_t *inout_mbx_cnt,
    uint16_t mbx_write_addr,
    uint16_t mbx_write_len,
    uint16_t mbx_read_addr,
    uint16_t mbx_read_len,
    uint16_t index,
    uint8_t sub,
    uint8_t *out,
    size_t out_cap,
    size_t *out_len,
    bool diag_enabled,
    unsigned int poll_interval_ms,
    unsigned int transaction_timeout_ms)
{
    if (out_len) {
        *out_len = 0;
    }

    uint8_t mbxbuf[256] = {0};
    if (mbx_write_len > sizeof(mbxbuf) || mbx_read_len > sizeof(mbxbuf)) {
        TETHER_LOGE(TAG, "Mailbox size too large (wr=%u rd=%u)", mbx_write_len, mbx_read_len);
        return false;
    }

    uint16_t coe_number = static_cast<uint16_t>(master.allocIdx());

    uint8_t mbx_cnt = 0;
    if (inout_mbx_cnt != nullptr) {
        mbx_cnt = *inout_mbx_cnt;
    }

    const uint8_t expected_mbx_cnt = mbx_cnt;

    MbxHeader mbx{};
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoInitUploadReq)));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);

    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDOREQ));

    SdoInitUploadReq sdo{};
    sdo.cmd = EC_SDO_UP_REQ;
    sdo.index_le = host_to_le16(index);
    sdo.sub = sub;
    std::memset(sdo.reserved, 0, sizeof(sdo.reserved));

    const size_t msg_len = sizeof(mbx) + sizeof(coe) + sizeof(sdo);
    if (msg_len > mbx_write_len) {
        TETHER_LOGE(TAG, "Mailbox write len too small (%u < %u)", mbx_write_len, static_cast<unsigned>(msg_len));
        return false;
    }

    std::memcpy(mbxbuf, &mbx, sizeof(mbx));
    std::memcpy(mbxbuf + sizeof(mbx), &coe, sizeof(coe));
    std::memcpy(mbxbuf + sizeof(mbx) + sizeof(coe), &sdo, sizeof(sdo));

    logCoeMbxPacket("TX", adp, index, sub, mbxbuf, msg_len,
                    master.debugFlags().coeTxPackets && master.debugFlags().coeTxPacketsFilt.allows(slaveIndexFromADP(adp)));

    mbx_cnt = static_cast<uint8_t>((mbx_cnt >= 7) ? 1 : (mbx_cnt + 1));
    if (inout_mbx_cnt != nullptr) {
        *inout_mbx_cnt = mbx_cnt;
    }

    // Write mailbox request into slave RX mailbox.
    {
        // First, check SM0 status to see if mailbox is ready
        uint8_t sm0_status = 0;
        (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0805, sm0_status, 100);
        
        // Read AL_STATUS to verify we're in PRE_OP
        uint16_t al_status = 0;
        (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0130, al_status, 100);
        
        // Reduce logging spam - only log once every 1000 calls
        static uint32_t s_mbx_write_count = 0;
        s_mbx_write_count++;
        if ((s_mbx_write_count % 1000) == 1) {
            TETHER_LOGI(TAG, "SDO upload (read) request to adp=0x%04X: index=0x%04X:%u read_size=%zu [mailbox #%lu -> 0x%04X, len=%u, SM0=0x%02X, AL=0x%04X]",
                     adp, index, sub, out_cap, (unsigned long)s_mbx_write_count, mbx_write_addr, mbx_write_len, sm0_status, al_status);
        }

#ifdef TETHER_DIAG_SDO_IO
        if (diag_enabled) {
            // Diagnostic: show what we're writing and via which mailbox addresses
            TETHER_LOGI(TAG, "SDO Upload INIT: adp=0x%04x index=0x%04x sub=%u coe_num=%u mbx_wr=0x%04x/%u mbx_rd=0x%04x/%u SM0=0x%02x AL=0x%04x",
                     adp, index, sub, coe_number, mbx_write_addr, mbx_write_len, mbx_read_addr, mbx_read_len, sm0_status, al_status);
            diag_hexdump(mbxbuf, msg_len, 64);
        }
#endif

        (void)mbx_drain_all_stale(master, adp, mbx_read_addr, mbx_read_len);

        // IMPORTANT: actually write the upload request into the slave RX mailbox.
        // This used to be missing, causing all SDO reads to fail silently.
        {
            bool used_alt = false;
            uint16_t local_wr_addr = mbx_write_addr;
            uint16_t local_rd_addr = mbx_read_addr;

            if (!mbx_apwr_with_wkc_probe(master, adp,
                                         local_wr_addr,
                                         local_rd_addr,
                                         mbxbuf,
                                         static_cast<uint16_t>(mbx_write_len),
                                         500,
                                         &used_alt)) {
                return false;
            }

            if (used_alt) {
                // We wrote to the alternate address; swap wr/rd for response polling.
                std::swap(mbx_write_addr, mbx_read_addr);
                std::swap(mbx_write_len, mbx_read_len);
            }
        }

        // Wait for the slave to finish processing and populate SM1 (mailbox full).
        if (!mbx_poll_sm1_full(master, adp, transaction_timeout_ms, poll_interval_ms)) {
            TETHER_LOGE(TAG, "SDO upload: SM1 mailbox never became full (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u timeout=%ums)",
                        adp, mbx_write_addr, mbx_read_addr, index, sub, transaction_timeout_ms);
            mbx_diag_dump_slave_state(master, adp, mbx_write_addr, mbx_read_addr);
            return false;
        }

        // Poll mailbox read area for response.
        bool logged_any_mbx = false;
        bool logged_mbx_mismatch = false;
        bool got_init_response = false;
        // When a stale response is detected, we re-send the same request (same
        // mailbox counter) and re-wait for SM1 to become full.  This clears the
        // stale data and gives the slave a fresh chance to respond correctly.
        // Limit retries to avoid infinite loops on a permanently desynced slave.
        static constexpr int MAX_STALE_RETRIES = 8;
        int stale_retry_count = 0;
        // Variables captured from initial upload response (kept after loop for segmented transfers)
        const uint8_t* r_sdo_bytes = nullptr;
        uint8_t sdo_cmd = 0;
        uint16_t r_len = 0;

        // Helper: re-send the same SDO upload request and re-wait for SM1 full.
        // Returns true if the re-send and SM1 wait both succeeded, false on failure.
        // The mbxbuf already contains the correct request with the original counter.
        auto resend_and_wait = [&]() -> bool {
            if (!mbx_apwr_with_wkc_probe(master, adp,
                                         mbx_write_addr,
                                         mbx_read_addr,
                                         mbxbuf,
                                         static_cast<uint16_t>(mbx_write_len),
                                         500,
                                         nullptr)) {
                TETHER_LOGE(TAG, "SDO upload: re-send failed after stale response (adp=0x%04X index=0x%04X:%u)",
                            adp, index, sub);
                return false;
            }
            if (!mbx_poll_sm1_full(master, adp, transaction_timeout_ms, poll_interval_ms)) {
                TETHER_LOGE(TAG, "SDO upload: SM1 never full after re-send (adp=0x%04X index=0x%04X:%u timeout=%ums)",
                            adp, index, sub, transaction_timeout_ms);
                mbx_diag_dump_slave_state(master, adp, mbx_write_addr, mbx_read_addr);
                return false;
            }
            return true;
        };
        for (int attempt = 0; attempt < 50; attempt++) {
            if (master.isCancelRequested()) {
                TETHER_LOGW(TAG, "SDO upload cancelled");
                return false;
            }
            if (!master.readRegister(Master::slaveAddressFromADP(adp), mbx_read_addr, mbxbuf, static_cast<uint16_t>(mbx_read_len), 200)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            const auto *r_mbx = reinterpret_cast<const MbxHeader *>(mbxbuf);
            r_len = le16_to_host(r_mbx->length_le);
            const uint8_t r_type = static_cast<uint8_t>(r_mbx->mbxtype & 0x0Fu);
            const uint8_t r_cnt = static_cast<uint8_t>((r_mbx->mbxtype >> 4) & 0x0Fu);
            if (!logged_any_mbx) {
                logged_any_mbx = true;
#ifdef TETHER_DIAG_SDO_IO
                if (diag_enabled) {
                    const uint8_t *p = mbxbuf;
                    TETHER_LOGI(TAG, "MBX poll: len=%u type=0x%02x cnt=%u rawType=0x%02x bytes=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                             r_len, r_type, r_cnt, r_mbx->mbxtype, p[0], p[1], p[2], p[3], p[4], p[5], p[6],
                             p[7], p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
                }
#endif
            }
            if (r_len == 0 || r_type != EC_MBXT_COE) {
                if (!logged_mbx_mismatch) {
                    logged_mbx_mismatch = true;
                    if (r_type == 0x00 && r_len >= 4) {
                        const uint16_t err = le16_to_host(*reinterpret_cast<const uint16_t *>(mbxbuf + sizeof(MbxHeader) + 0));
                        const uint16_t detail = le16_to_host(*reinterpret_cast<const uint16_t *>(mbxbuf + sizeof(MbxHeader) + 2));
                        char tmp_err[64];
                        snprintf(tmp_err, sizeof(tmp_err), "MBX error: cnt=%u err=0x%04x detail=0x%04x", r_cnt, err, detail);
                        TETHER_LOGW(TAG, "%s", tmp_err);
                    }
                    {
                        char tmp[96];
                        snprintf(tmp, sizeof(tmp), "MBX mismatch: len=%u type=0x%02x cnt=%u prio=0x%02x rawType=0x%02x", r_len, r_type,
                                 r_cnt, r_mbx->priority, r_mbx->mbxtype);
                        TETHER_LOGI(TAG, "%s", tmp);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            if (sizeof(MbxHeader) + r_len > mbx_read_len) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // Use memcpy instead of reinterpret_cast for safety
            CoeHeader r_coe;
            std::memcpy(&r_coe, mbxbuf + sizeof(MbxHeader), sizeof(r_coe));
            const uint16_t r_coe_raw = le16_to_host(r_coe.raw_le);
            const uint16_t r_number = r_coe_raw & 0x01FFu;
            const uint8_t r_service = (r_coe_raw >> 12) & 0x0Fu;
            if (r_service != EC_COES_SDORES) {
                if (!logged_mbx_mismatch) {
                    logged_mbx_mismatch = true;
                    const char* svc_name = "Unknown";
                    switch (r_service) {
                        case 0x00: svc_name = "Emergency (0x0)"; break;
                        case 0x01: svc_name = "EMail (0x1)"; break;
                        case 0x02: svc_name = "SDO Request (0x2)"; break;
                        case 0x03: svc_name = "SDO Response (0x3)"; break;
                        case 0x04: svc_name = "TxPDO (0x4)"; break;
                        case 0x05: svc_name = "RxPDO (0x5)"; break;
                        case 0x06: svc_name = "TxPDO RR (0x6)"; break;
                        case 0x07: svc_name = "RxPDO RR (0x7)"; break;
                        default:   break;
                    }
                    char tmp[160];
                    snprintf(tmp, sizeof(tmp),
                             "Expected SDO Response (CoE service 0x3) but received %s "
                             "[ CoE header=0x%04X service=0x%X number=%u ]",
                             svc_name, r_coe_raw, r_service, r_number);
                    TETHER_LOGI(TAG, "%s", tmp);
#ifdef TETHER_DIAG_SDO_IO
                    if (diag_enabled) {
                        TETHER_LOGI(TAG, "CoE mismatch raw mbx (len=%u)", r_len);
                        diag_hexdump(mbxbuf, r_len, 256);
                    }
#endif
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            // Validate mailbox counter — a stale response from a previous
            // (possibly timed-out) request will have a different counter.
            if (r_cnt != expected_mbx_cnt) {
                TETHER_LOGW(TAG, "Stale mailbox response: cnt=%u expected=%u (adp=0x%04X index=0x%04X:%u) — clearing and re-sending",
                            r_cnt, expected_mbx_cnt, adp, index, sub);
                if (++stale_retry_count <= MAX_STALE_RETRIES) {
                    if (!resend_and_wait()) {
                        return false;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                continue;
            }
            r_sdo_bytes = mbxbuf + sizeof(MbxHeader) + sizeof(CoeHeader);
            r_len = le16_to_host(r_mbx->length_le);
            got_init_response = true;
            if (r_len < (sizeof(CoeHeader) + sizeof(SdoInitUploadRes))) {
                return false;
            }
            sdo_cmd = r_sdo_bytes[0];
            if (sdo_cmd == EC_SDO_ABORT) {
                if (r_len >= sizeof(CoeHeader) + sizeof(SdoAbort)) {
                    const auto *ab = reinterpret_cast<const SdoAbort *>(r_sdo_bytes);
                    const uint32_t abort_code = le32_to_host(ab->abortCode_le);
                    TETHER_LOGE(TAG, "SDO abort 0x%04x:%u code=0x%08" PRIx32 " (%s)",
                             le16_to_host(ab->index_le), ab->sub,
                             abort_code, sdo_abort_code_str(abort_code));
                }
#ifdef TETHER_DIAG_SDO_IO
                if (diag_enabled) {
                    TETHER_LOGI(TAG, "SDO abort raw response (len=%u)", r_len);
                    diag_hexdump(mbxbuf, r_len, 256);
                }
#endif
                return false;
            }
            // Validate SDO index/subindex — a stale response for a different
            // object must not be accepted as the response to this request.
            if (r_len >= sizeof(CoeHeader) + sizeof(SdoInitUploadRes)) {
                const auto *res = reinterpret_cast<const SdoInitUploadRes *>(r_sdo_bytes);
                const uint16_t r_index = le16_to_host(res->index_le);
                const uint8_t r_sub = res->sub;
                if (r_index != index || r_sub != sub) {
                    TETHER_LOGW(TAG, "Stale SDO response: idx=0x%04X:%u expected=0x%04X:%u (adp=0x%04X) — clearing and re-sending",
                                r_index, r_sub, index, sub, adp);
                    if (++stale_retry_count <= MAX_STALE_RETRIES) {
                        if (!resend_and_wait()) {
                            return false;
                        }
                    } else {
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    continue;
                }
            }
            // Matching non-abort response found — stop polling.
            // CRITICAL: We must break here so that subsequent APRD calls
            // do not overwrite mbxbuf (and therefore r_sdo_bytes) with
            // new/empty mailbox data, corrupting the response payload.
            break;
        }

        if (!got_init_response) {
            TETHER_LOGE(TAG, "SDO upload timeout: no mailbox response (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u)",
                        adp, mbx_write_addr, mbx_read_addr, index, sub);
            mbx_diag_dump_slave_state(master, adp, mbx_write_addr, mbx_read_addr);
            return false;
        }

        const uint8_t ccs = (sdo_cmd >> 5) & 0x07u;
        const bool expedited = (sdo_cmd & 0x02u) != 0;
        const bool size_indicated = (sdo_cmd & 0x01u) != 0;

        if (ccs != 2) {
            return false;
        }

        if (expedited) {
            const auto *res = reinterpret_cast<const SdoInitUploadRes *>(r_sdo_bytes);
            const uint8_t n = (sdo_cmd >> 2) & 0x03u;
            const size_t data_bytes = 4u - n;
            const uint32_t v = le32_to_host(res->data_or_size_le);
#ifdef TETHER_DIAG_SDO_IO
            if (diag_enabled) {
                TETHER_LOGI(TAG, "SDO Upload MATCH: idx=0x%04x:%u cmd=0x%02x n=%u data_bytes=%zu v=0x%08x raw=%02x %02x %02x %02x",
                         le16_to_host(res->index_le), res->sub, sdo_cmd, n, data_bytes, v,
                         r_sdo_bytes[4], r_sdo_bytes[5], r_sdo_bytes[6], r_sdo_bytes[7]);
            }
#endif
            if (out && out_cap > 0) {
                const size_t copy_n = (data_bytes < out_cap) ? data_bytes : out_cap;
                std::memcpy(out, &v, copy_n);
                if (out_len) {
                    *out_len = copy_n;
                }
            }
            logCoeMbxPacket("RX", adp, index, sub, mbxbuf, mbx_read_len,
                            master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
            return true;
        }

        uint32_t total_size = 0;
        if (size_indicated && r_len >= sizeof(CoeHeader) + sizeof(SdoInitUploadRes)) {
            const auto *res = reinterpret_cast<const SdoInitUploadRes *>(r_sdo_bytes);
            total_size = le32_to_host(res->data_or_size_le);
        }

        bool toggle = false;
        size_t produced = 0;
        for (int seg = 0; seg < 200; seg++) {
            MbxHeader seg_mbx{};
            seg_mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + 8));
            seg_mbx.address_le = host_to_le16(0);
            seg_mbx.priority = 0;
            seg_mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);

            CoeHeader seg_coe{};
            seg_coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDOREQ));

            const uint8_t seg_req_cmd = static_cast<uint8_t>(EC_SDO_SEG_UP_REQ | (toggle ? 0x10u : 0x00u));

            std::memset(mbxbuf, 0, sizeof(mbxbuf));
            std::memcpy(mbxbuf, &seg_mbx, sizeof(seg_mbx));
            std::memcpy(mbxbuf + sizeof(seg_mbx), &seg_coe, sizeof(seg_coe));
            mbxbuf[sizeof(seg_mbx) + sizeof(seg_coe) + 0] = seg_req_cmd;

            mbx_cnt = static_cast<uint8_t>((mbx_cnt >= 7) ? 1 : (mbx_cnt + 1));
            if (inout_mbx_cnt != nullptr) {
                *inout_mbx_cnt = mbx_cnt;
            }

            // Send segment request; write full SM buffer to trigger mailbox event.
            {
                bool used_alt = false;
                if (!mbx_apwr_with_wkc_probe(master, adp,
                                             mbx_write_addr,
                                             mbx_read_addr,
                                             mbxbuf,
                                             static_cast<uint16_t>(mbx_write_len),
                                             500,
                                             &used_alt)) {
                    return false;
                }
                if (used_alt) {
                    std::swap(mbx_write_addr, mbx_read_addr);
                    std::swap(mbx_write_len, mbx_read_len);
                }
            }

            // Wait for the slave to populate the response mailbox.
            if (!mbx_poll_sm1_full(master, adp, transaction_timeout_ms, poll_interval_ms)) {
                TETHER_LOGE(TAG, "SDO upload segment: SM1 mailbox never became full (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u timeout=%ums)",
                            adp, mbx_write_addr, mbx_read_addr, index, sub, transaction_timeout_ms);
                mbx_diag_dump_slave_state(master, adp, mbx_write_addr, mbx_read_addr);
                return false;
            }

            // Read segment response.
            bool got = false;
            for (int attempt2 = 0; attempt2 < 50; attempt2++) {
                if (master.isCancelRequested()) {
                    TETHER_LOGW(TAG, "SDO upload segment cancelled");
                    return false;
                }
                if (!master.readRegister(Master::slaveAddressFromADP(adp), mbx_read_addr, mbxbuf, static_cast<uint16_t>(mbx_read_len), 200)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                // Use memcpy instead of reinterpret_cast for safety
                MbxHeader r2_mbx;
                std::memcpy(&r2_mbx, mbxbuf, sizeof(r2_mbx));
                const uint16_t r2_len = le16_to_host(r2_mbx.length_le);
                const uint8_t r2_type = static_cast<uint8_t>(r2_mbx.mbxtype & 0x0Fu);
                if (r2_len < sizeof(CoeHeader) + 8 || r2_type != EC_MBXT_COE) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                const auto *r2_coe = reinterpret_cast<const CoeHeader *>(mbxbuf + sizeof(MbxHeader));
                const uint16_t r2_coe_raw = le16_to_host(r2_coe->raw_le);
                const uint8_t r2_service = (r2_coe_raw >> 12) & 0x0Fu;
                if (r2_service != EC_COES_SDORES) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                const uint8_t *seg_res = mbxbuf + sizeof(MbxHeader) + sizeof(CoeHeader);
                const uint8_t seg_cmd = seg_res[0];
                const uint8_t seg_ccs = (seg_cmd >> 5) & 0x07u;
                if (seg_ccs != 0) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                const bool seg_toggle = (seg_cmd & 0x10u) != 0;
                if (seg_toggle != toggle) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    continue;
                }
                const uint8_t n = (seg_cmd >> 1) & 0x07u;
                const bool last = (seg_cmd & 0x01u) != 0;
                const size_t seg_data_bytes = 7u - n;
                if (out && out_cap > produced) {
                    const size_t copy_n = (seg_data_bytes < (out_cap - produced)) ? seg_data_bytes : (out_cap - produced);
                    std::memcpy(out + produced, seg_res + 1, copy_n);
                }
                produced += seg_data_bytes;
                if (last) {
                    if (out_len) {
                        *out_len = produced;
                    }
                    if (total_size != 0 && produced > total_size && out_len) {
                        *out_len = total_size;
                    }
                    logCoeMbxPacket("RX", adp, index, sub, mbxbuf, mbx_read_len,
                            master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
                    return true;
                }
                toggle = !toggle;
                got = true;
                break;
            }
            if (!got) {
                TETHER_LOGE(TAG, "SDO upload segment timeout (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%u)",
                            adp, mbx_write_addr, mbx_read_addr, index, sub);
                mbx_diag_dump_slave_state(master, adp, mbx_write_addr, mbx_read_addr);
                return false;
            }
        }
        return false;
    }

    return false;
}

bool coe_sdo_download(
    Master& master,
    uint16_t adp,
    uint8_t *inout_mbx_cnt,
    uint16_t mbx_write_addr,
    uint16_t mbx_write_len,
    uint16_t mbx_read_addr,
    uint16_t mbx_read_len,
    uint16_t index,
    uint8_t sub,
    const uint8_t *data,
    size_t data_len,
    bool diag_enabled,
    unsigned int poll_interval_ms,
    unsigned int transaction_timeout_ms)
{
    if (data == nullptr || data_len == 0 || data_len > 4) {
        TETHER_LOGE(TAG, "Invalid SDO download parameters (len=%u)", static_cast<unsigned>(data_len));
        return false;
    }

    uint8_t mbxbuf[256] = {0};
    if (mbx_write_len > sizeof(mbxbuf) || mbx_read_len > sizeof(mbxbuf)) {
        TETHER_LOGE(TAG, "Mailbox size too large (wr=%u rd=%u)", mbx_write_len, mbx_read_len);
        return false;
    }

    uint8_t mbx_cnt = 0;
    if (inout_mbx_cnt != nullptr) {
        mbx_cnt = *inout_mbx_cnt;
    }

    const uint8_t expected_mbx_cnt = mbx_cnt;

    // Build mailbox header
    MbxHeader mbx{};
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoInitDownloadReq)));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);

    // Build CoE header
    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDOREQ));

    // Build SDO download request
    // Expedited transfer: cmd = 0x20 | (expedited << 1) | (size_specified << 0)
    // For expedited with size: cmd = 0x20 | 0x02 | 0x01 = 0x23
    // n = (4 - data_len) & 0x03
    // cmd |= (n << 2)
    const uint8_t n = static_cast<uint8_t>((4u - data_len) & 0x03u);
    SdoInitDownloadReq sdo{};
    sdo.cmd = static_cast<uint8_t>(EC_SDO_DOWN_REQ | 0x02u | 0x01u | (n << 2));
    sdo.index_le = host_to_le16(index);
    sdo.sub = sub;
    
    // Copy data (little-endian, fill remaining bytes with zeros)
    uint32_t data_u32 = 0;
    std::memcpy(&data_u32, data, data_len);
    sdo.data_le = host_to_le32(data_u32);

    const size_t msg_len = sizeof(mbx) + sizeof(coe) + sizeof(sdo);
    if (msg_len > mbx_write_len) {
        TETHER_LOGE(TAG, "Mailbox write len too small (%u < %u)", mbx_write_len, static_cast<unsigned>(msg_len));
        return false;
    }

    std::memcpy(mbxbuf, &mbx, sizeof(mbx));
    std::memcpy(mbxbuf + sizeof(mbx), &coe, sizeof(coe));
    std::memcpy(mbxbuf + sizeof(mbx) + sizeof(coe), &sdo, sizeof(sdo));

    logCoeMbxPacket("TX", adp, index, sub, mbxbuf, msg_len,
                    master.debugFlags().coeTxPackets && master.debugFlags().coeTxPacketsFilt.allows(slaveIndexFromADP(adp)));

    mbx_cnt = static_cast<uint8_t>((mbx_cnt >= 7) ? 1 : (mbx_cnt + 1));
    if (inout_mbx_cnt != nullptr) {
        *inout_mbx_cnt = mbx_cnt;
    }

    // Write mailbox request into slave RX mailbox
    {
        // First, check SM0 status to see if mailbox is ready
        uint8_t sm0_status = 0;
        (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0805, sm0_status, 100);

        // Read AL_STATUS to verify we're in PRE_OP
        uint16_t al_status = 0;
        (void)master.readRegister(Master::slaveAddressFromADP(adp), 0x0130, al_status, 100);

        // Reduce logging spam - only log once every 1000 calls
        static uint32_t s_mbx_write_count = 0;
        s_mbx_write_count++;
        if ((s_mbx_write_count % 1000) == 1) {
            TETHER_LOGI(TAG, "SDO download (write) request to adp=0x%04X: index=0x%04X:%u [mailbox #%lu -> 0x%04X, len=%u, SM0=0x%02X, AL=0x%04X]",
                     adp, index, sub, (unsigned long)s_mbx_write_count, mbx_write_addr, mbx_write_len, sm0_status, al_status);
        }

#ifdef TETHER_DIAG_SDO_IO
        if (diag_enabled) {
            // Diagnostic: show what we're writing for download
            TETHER_LOGI(TAG, "SDO Download INIT: adp=0x%04x index=0x%04x sub=%u mbx_wr=0x%04x/%u mbx_rd=0x%04x/%u",
                     adp, index, sub, mbx_write_addr, mbx_write_len, mbx_read_addr, mbx_read_len);
            diag_hexdump(mbxbuf, msg_len, 64);
        }
#endif
        (void)mbx_drain_all_stale(master, adp, mbx_read_addr, mbx_read_len);

        // Write full SM buffer to trigger ESC mailbox event (must reach last byte)
        bool used_alt = false;
        if (!mbx_apwr_with_wkc_probe(master, adp,
                                     mbx_write_addr,
                                     mbx_read_addr,
                                     mbxbuf,
                                     static_cast<uint16_t>(mbx_write_len),
                                     500,
                                     &used_alt)) {
            TETHER_LOGE(TAG, "SDO download: Mailbox write failed (adp=0x%04X wr=0x%04X rd=0x%04X)",
                        adp, mbx_write_addr, mbx_read_addr);
            return false;
        }
        if (used_alt) {
            std::swap(mbx_write_addr, mbx_read_addr);
            std::swap(mbx_write_len, mbx_read_len);
        }
    }

    // Wait for the slave to populate the response mailbox.
    if (!mbx_poll_sm1_full(master, adp, transaction_timeout_ms, poll_interval_ms)) {
        TETHER_LOGE(TAG, "SDO download: SM1 mailbox never became full (adp=0x%04X wr=0x%04X rd=0x%04X index=0x%04X:%02x timeout=%ums)",
                    adp, mbx_write_addr, mbx_read_addr, index, sub, transaction_timeout_ms);
        mbx_diag_dump_slave_state(master, adp, mbx_write_addr, mbx_read_addr);
        return false;
    }

    // When a stale response is detected, re-send the same request and re-wait.
    static constexpr int MAX_STALE_RETRIES = 8;
    int stale_retry_count = 0;

    // Helper: re-send the same SDO download request and re-wait for SM1 full.
    auto resend_and_wait = [&]() -> bool {
        if (!mbx_apwr_with_wkc_probe(master, adp,
                                     mbx_write_addr,
                                     mbx_read_addr,
                                     mbxbuf,
                                     static_cast<uint16_t>(mbx_write_len),
                                     500,
                                     nullptr)) {
            TETHER_LOGE(TAG, "SDO download: re-send failed after stale response (adp=0x%04X index=0x%04X:%u)",
                        adp, index, sub);
            return false;
        }
        if (!mbx_poll_sm1_full(master, adp, transaction_timeout_ms, poll_interval_ms)) {
            TETHER_LOGE(TAG, "SDO download: SM1 never full after re-send (adp=0x%04X index=0x%04X:%u timeout=%ums)",
                        adp, index, sub, transaction_timeout_ms);
            mbx_diag_dump_slave_state(master, adp, mbx_write_addr, mbx_read_addr);
            return false;
        }
        return true;
    };

    // Poll mailbox read area for response
    for (int attempt = 0; attempt < 50; attempt++) {
        if (master.isCancelRequested()) {
            TETHER_LOGW(TAG, "SDO download cancelled");
            return false;
        }
        if (!master.readRegister(Master::slaveAddressFromADP(adp), mbx_read_addr, mbxbuf, static_cast<uint16_t>(mbx_read_len), 500)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        MbxHeader resp_mbx{};
        std::memcpy(&resp_mbx, mbxbuf, sizeof(resp_mbx));
        const uint16_t resp_len = le16_to_host(resp_mbx.length_le);
        const uint8_t resp_cnt = static_cast<uint8_t>((resp_mbx.mbxtype >> 4) & 0x0Fu);

        // Validate mailbox counter — reject stale responses from previous requests.
        if (resp_cnt != expected_mbx_cnt) {
            TETHER_LOGW(TAG, "Stale mailbox response (download): cnt=%u expected=%u (adp=0x%04X index=0x%04X:%u) — clearing and re-sending",
                        resp_cnt, expected_mbx_cnt, adp, index, sub);
            if (++stale_retry_count <= MAX_STALE_RETRIES) {
                if (!resend_and_wait()) {
                    return false;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            continue;
        }

        if (resp_len < sizeof(CoeHeader)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        CoeHeader resp_coe{};
        std::memcpy(&resp_coe, mbxbuf + sizeof(MbxHeader), sizeof(resp_coe));

        const size_t sdo_offset = sizeof(MbxHeader) + sizeof(CoeHeader);
        const uint8_t sdo_cmd = mbxbuf[sdo_offset];

        // Check for abort
        if ((sdo_cmd & 0xE0u) == EC_SDO_ABORT) {
            if (mbx_read_len >= sdo_offset + sizeof(SdoAbort)) {
                SdoAbort abort{};
                std::memcpy(&abort, mbxbuf + sdo_offset, sizeof(abort));
                const uint32_t abort_code = le32_to_host(abort.abortCode_le);
                TETHER_LOGE(TAG, "SDO download abort: index=0x%04x:%02x code=0x%08" PRIx32 " (%s)",
                         index, sub, abort_code, sdo_abort_code_str(abort_code));
                if (abort_code == 0x06090011 && is_pdo_mapping_index(index)) {
                    log_pdo_mapping_subindex_diagnostic(
                        master, adp, inout_mbx_cnt,
                        mbx_write_addr, mbx_write_len,
                        mbx_read_addr, mbx_read_len,
                        index, sub,
                        diag_enabled, poll_interval_ms, transaction_timeout_ms);
                }
            } else {
                TETHER_LOGE(TAG, "SDO download abort (malformed response)");
            }
#ifdef TETHER_DIAG_SDO_IO
            if (diag_enabled) {
                TETHER_LOGI(TAG, "SDO download abort raw response (mbx_read_len=%u)", (unsigned)mbx_read_len);
                diag_hexdump(mbxbuf + sdo_offset, mbx_read_len - sdo_offset, 256);
            }
#endif
            return false;
        }

        // Check for download response (0x60)
        if ((sdo_cmd & 0xE0u) == 0x60u) {
            if (mbx_read_len >= sdo_offset + sizeof(SdoInitDownloadRes)) {
                SdoInitDownloadRes res{};
                std::memcpy(&res, mbxbuf + sdo_offset, sizeof(res));
                const uint16_t res_index = le16_to_host(res.index_le);

                if (res_index == index && res.sub == sub) {
                    logCoeMbxPacket("RX", adp, index, sub, mbxbuf, mbx_read_len,
                            master.debugFlags().coeRxPackets && master.debugFlags().coeRxPacketsFilt.allows(slaveIndexFromADP(adp)));
                    return true;
                }
                // Counter matches but index/sub doesn't — stale response.
                TETHER_LOGW(TAG, "Stale SDO download response: idx=0x%04X:%u expected=0x%04X:%u (adp=0x%04X) — clearing and re-sending",
                            res_index, res.sub, index, sub, adp);
                if (++stale_retry_count <= MAX_STALE_RETRIES) {
                    if (!resend_and_wait()) {
                        return false;
                    }
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                continue;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    TETHER_LOGE(TAG, "SDO download timeout: index=0x%04x:%02x (adp=0x%04X wr=0x%04X rd=0x%04X)",
                index, sub, adp, mbx_write_addr, mbx_read_addr);
    mbx_diag_dump_slave_state(master, adp, mbx_write_addr, mbx_read_addr);
    return false;
}

} // namespace Raw
} // namespace EtherCAT
