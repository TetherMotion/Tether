#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/FaultDetection.hpp"
#include "tether/platform/Platform.hpp"
#include "raw/internal.hpp"
#include <bit>
#include <cstdio>
#include <cstring>

namespace EtherCAT {
namespace Raw {

static const char* TAG = "ethercat";

static uint16_t slaveIndexFromADP(uint16_t adp) {
    return Master::slaveAddressFromADP(adp).slavePosition();
}

void SDODiagnostics::smControlStr(uint8_t ctrlByte, char* buf, size_t bufLen) const {
    const auto ctrl = std::bit_cast<EtherCAT::SyncManager::SMControlReg>(ctrlByte);
    size_t pos = 0;

    const char* mode = "UNKNOWN";
    if (ctrl.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Buffered)) mode = "BUFFERED";
    else if (ctrl.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::Mailbox))  mode = "MAILBOX";
    else if (ctrl.mode == static_cast<uint8_t>(EtherCAT::SyncManager::SMMode::ThreePDO)) mode = "THREE_PDO";

    pos += snprintf(buf + pos, bufLen - pos, "%s %s", mode, ctrl.direction ? "M->S" : "S->M");
    if (ctrl.ecat_irq)   pos += snprintf(buf + pos, bufLen - pos, " IRQ_ECAT");
    if (ctrl.pdi_irq)    pos += snprintf(buf + pos, bufLen - pos, " IRQ_PDI");
    if (ctrl.watchdog)   pos += snprintf(buf + pos, bufLen - pos, " WATCHDOG");
    if (ctrl.repeat_req) pos += snprintf(buf + pos, bufLen - pos, " REPEAT_REQ");
}

void SDODiagnostics::smStatusStr(uint8_t statByte, char* buf, size_t bufLen) const {
    const auto stat = std::bit_cast<EtherCAT::SyncManager::SMStatusReg>(statByte);
    if (statByte == 0) {
        snprintf(buf, bufLen, "-");
        return;
    }
    size_t pos = 0;
    if (stat.write_event)    pos += snprintf(buf + pos, bufLen - pos, "WRITE_EVENT ");
    if (stat.read_event)     pos += snprintf(buf + pos, bufLen - pos, "READ_EVENT ");
    if (stat.mailbox_full)   pos += snprintf(buf + pos, bufLen - pos, "MBOX_FULL ");
    if (stat.read_buf_full)  pos += snprintf(buf + pos, bufLen - pos, "READ_BUF_FULL ");
    if (stat.write_buf_full) pos += snprintf(buf + pos, bufLen - pos, "WRITE_BUF_FULL ");
    if (pos > 0 && buf[pos - 1] == ' ') buf[pos - 1] = '\0';
}

void SDODiagnostics::smActivateStr(uint8_t actByte, char* buf, size_t bufLen) const {
    const auto act = std::bit_cast<EtherCAT::SyncManager::SMActivateReg>(actByte);
    snprintf(buf, bufLen, "%s", act.enable ? "ENABLED" : "disabled");
}

void SDODiagnostics::dumpSlaveState(Master& master, uint16_t adp,
                                    uint16_t mbxWrAddr, uint16_t mbxRdAddr) {
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

    char sm0_ctrl_desc[64];  smControlStr(sm0_ctrl, sm0_ctrl_desc, sizeof(sm0_ctrl_desc));
    char sm1_ctrl_desc[64];  smControlStr(sm1_ctrl, sm1_ctrl_desc, sizeof(sm1_ctrl_desc));
    char sm0_stat_desc[64];  smStatusStr(sm0_stat, sm0_stat_desc, sizeof(sm0_stat_desc));
    char sm1_stat_desc[64];  smStatusStr(sm1_stat, sm1_stat_desc, sizeof(sm1_stat_desc));
    char sm0_act_desc[16];   smActivateStr(sm0_act, sm0_act_desc, sizeof(sm0_act_desc));
    char sm1_act_desc[16];   smActivateStr(sm1_act, sm1_act_desc, sizeof(sm1_act_desc));

    char sm0_sa_stat_desc[64]; smStatusStr(sm0_stat_act[0], sm0_sa_stat_desc, sizeof(sm0_sa_stat_desc));
    char sm1_sa_stat_desc[64]; smStatusStr(sm1_stat_act[0], sm1_sa_stat_desc, sizeof(sm1_sa_stat_desc));
    char sm0_sa_act_desc[16];  smActivateStr(sm0_stat_act[1], sm0_sa_act_desc, sizeof(sm0_sa_act_desc));
    char sm1_sa_act_desc[16];  smActivateStr(sm1_stat_act[1], sm1_sa_act_desc, sizeof(sm1_sa_act_desc));

    TETHER_LOGE(TAG, "[SDO_DIAG] Slave {}: AL_STATUS=0x{:04X} state={}{} | AL status code: {} (0x{:04X})\n[SDO_DIAG] MBX cfg: wr=0x{:04X} rd=0x{:04X} | SM0(start=0x{:04X} len={} ctrl=0x{:02X} [{}] stat=0x{:02X} act=0x{:02X}) SM1(start=0x{:04X} len={} ctrl=0x{:02X} [{}] stat=0x{:02X} act=0x{:02X})\n[SDO_DIAG] SM0 status=0x{:02X} [{}] act=0x{:02X} [{}] | SM1 status=0x{:02X} [{}] act=0x{:02X} [{}]",
               slaveIndexFromADP(adp),
               al_s,
               EtherCAT::al_status_get_state_name(al_s),
               EtherCAT::al_status_has_error(al_s) ? " ERROR" : "",
               EtherCAT::getALStatusCodeName(static_cast<EtherCAT::ALStatusCode>(al_c)),
               al_c,
               mbxWrAddr, mbxRdAddr,
               sm0_start, (unsigned)sm0_len, sm0_ctrl, sm0_ctrl_desc, sm0_stat, sm0_act,
               sm1_start, (unsigned)sm1_len, sm1_ctrl, sm1_ctrl_desc, sm1_stat, sm1_act,
               sm0_stat_act[0], sm0_sa_stat_desc, sm0_stat_act[1], sm0_sa_act_desc,
               sm1_stat_act[0], sm1_sa_stat_desc, sm1_stat_act[1], sm1_sa_act_desc);
}

void SDODiagnostics::logCoeMbxPacket(const char* dir, uint16_t adp,
                                      uint16_t index, uint8_t sub,
                                      const uint8_t* data, size_t len, bool enabled) {
    if (!enabled) return;

    TETHER_LOGI(TAG, "[CoE-{}] Slave {}: index=0x{:04X}:{} len={}",
                dir, slaveIndexFromADP(adp), index, sub, len);

    if (len == 0) return;

    bool all_zero = true;
    for (size_t i = 0; i < len; ++i) {
        if (data[i] != 0) { all_zero = false; break; }
    }
    if (all_zero) {
        TETHER_LOGI(TAG, "[CoE-{}] Data ({} bytes): All zeroes", dir, len);
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
    TETHER_LOGI(TAG, "[CoE-{}] Data ({}/{} bytes): {}", dir, dump_len, len, hexbuf);
}

bool SDODiagnostics::isPdoMappingIndex(uint16_t idx) const {
    return (idx >= 0x1600 && idx <= 0x167F) ||
           (idx >= 0x1A00 && idx <= 0x1A7F);
}

void SDODiagnostics::logPdoMappingSubindexDiagnostic(Master& master, uint16_t adp,
                                                      uint8_t* inoutMbxCnt,
                                                      uint16_t mbxWriteAddr, uint16_t mbxWriteLen,
                                                      uint16_t mbxReadAddr, uint16_t mbxReadLen,
                                                      uint16_t index, uint8_t sub,
                                                      bool diagEnabled,
                                                      unsigned int pollIntervalMs,
                                                      unsigned int transactionTimeoutMs,
                                                      UploadFn uploadFn) {
    if (sub == 0) return;

    uint8_t count_buf[4] = {0};
    size_t count_len = 0;
    bool ok = uploadFn(master, adp, inoutMbxCnt,
                       mbxWriteAddr, mbxWriteLen,
                       mbxReadAddr, mbxReadLen,
                       index, 0x00,
                       count_buf, sizeof(count_buf), &count_len,
                       diagEnabled, pollIntervalMs, transactionTimeoutMs);
    if (ok && count_len >= 1) {
        TETHER_LOGE(TAG, "  -> PDO mapping object 0x{:04X} supports max {} entries (subindices 1-{}), but subindex {} was accessed",
                    index, static_cast<unsigned>(count_buf[0]),
                    static_cast<unsigned>(count_buf[0]),
                    static_cast<unsigned>(sub));
    } else {
        TETHER_LOGE(TAG, "  -> PDO mapping object 0x{:04X}: subindex {} does not exist (failed to read max entry count from subindex 0)",
                    index, static_cast<unsigned>(sub));
    }
}

#ifdef TETHER_DIAG_SDO_IO
void SDODiagnostics::diagHexdump(const uint8_t* data, size_t len, size_t maxPrint) const {
    char buf[256];
    size_t to_print = (len < maxPrint) ? len : maxPrint;
    buf[0] = '\0';
    for (size_t i = 0; i < to_print; ++i) {
        char tmp[8];
        snprintf(tmp, sizeof(tmp), "%02X", data[i]);
        if (i) strncat(buf, " ", sizeof(buf) - strlen(buf) - 1);
        strncat(buf, tmp, sizeof(buf) - strlen(buf) - 1);
    }
    if (len > to_print) strncat(buf, " ...", sizeof(buf) - strlen(buf) - 1);
    TETHER_LOGI(TAG, "MBX data ({} bytes): {}", len, buf);
}
#endif

} // namespace Raw
} // namespace EtherCAT
