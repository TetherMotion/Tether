/**
 * @file interpret_pcapng.cpp
 * @brief Read and interpret a pcapng capture of EtherCAT traffic
 *
 * Usage:
 *   ./interpret_pcapng capture.pcapng
 *   ./interpret_pcapng capture.pcapng --select ethercat-transactions --select coe-transactions
 *   ./interpret_pcapng capture.pcapng --select raw --verbose --max-data 256
 *   ./interpret_pcapng capture.pcapng --select statistics
 *   ./interpret_pcapng capture.pcapng --select pdo --pdo-addr 0x00000000 --slave 0
 *   ./interpret_pcapng capture.pcapng --select mailbox
 *   ./interpret_pcapng capture.pcapng --select coe-transactions --coe-index 0x1018 --coe-sub 1
 *   ./interpret_pcapng capture.pcapng --select ethercat-transactions --errors-only
 */

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <argparse/argparse.hpp>

#include "tether/ethercat/Types.hpp"
#include "tether/packetloggers/pcap/PCAPNGReader.hpp"

namespace {

namespace PCP = Tether::PacketLoggers::PCAP;

// ============================================================================
// UTF-8 box-drawing formatter
// ============================================================================

class Utf8Formatter {
public:
    // Box-drawing characters
    static constexpr const char* topLeft     = "┏";
    static constexpr const char* topRight    = "┓";
    static constexpr const char* bottomLeft  = "┗";
    static constexpr const char* bottomRight = "┛";
    static constexpr const char* horizontal  = "━";
    static constexpr const char* vertical    = "┃";
    static constexpr const char* arrow       = "→";
    static constexpr const char* bullet      = "▸";
    static constexpr const char* check       = "✓";
    static constexpr const char* cross       = "✗";
    static constexpr const char* diamond     = "◇";

    // Generate a titled box header
    static std::string titledBox(const std::string& title) {
        std::string s = topLeft;
        for (size_t i = 0; i < title.size() + 2; ++i) s += horizontal;
        s += topRight;
        s += "\n";
        s += vertical;
        s += " ";
        s += title;
        s += " ";
        s += vertical;
        s += "\n";
        s += bottomLeft;
        for (size_t i = 0; i < title.size() + 2; ++i) s += horizontal;
        s += bottomRight;
        s += "\n";
        return s;
    }

    // Generate a horizontal divider line
    static std::string divider(int width = 60) {
        std::string s;
        for (int i = 0; i < width; ++i) s += horizontal;
        s += "\n";
        return s;
    }
};

// ============================================================================
// Selections
// ============================================================================

enum class Selection {
    Raw,
    EthercatTransactions,
    Mailbox,
    CoeTransactions,
    Pdo,
    Statistics,
};

std::optional<Selection> parseSelection(const std::string& name) {
    const std::string lower = [&name]{
        std::string s = name;
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }();
    if (lower == "raw") return Selection::Raw;
    if (lower == "ethercat-transactions") return Selection::EthercatTransactions;
    if (lower == "mailbox") return Selection::Mailbox;
    if (lower == "coe-transactions") return Selection::CoeTransactions;
    if (lower == "pdo") return Selection::Pdo;
    if (lower == "statistics") return Selection::Statistics;
    return std::nullopt;
}

// ============================================================================
// Filters
// ============================================================================

struct Filters {
    std::optional<uint16_t> slave;
    std::optional<uint16_t> addr;       // ado filter
    std::optional<uint16_t> coeIndex;
    std::optional<uint8_t>  coeSub;
    std::optional<uint32_t> pdoAddr;
    std::optional<uint16_t> wkc;
    bool errorsOnly = false;
    bool onlyEtherCAT = false;
    std::optional<uint16_t> vlan;
    std::optional<EtherCAT::Command> command;
    uint64_t maxPackets = 0;
    size_t maxData = 64;
    bool verbose = false;
};

// ============================================================================
// Helpers
// ============================================================================

std::string upperCase(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return s;
}

std::optional<EtherCAT::Command> parseCommand(const std::string& name) {
    const std::string u = upperCase(name);
    struct Entry { const char* name; EtherCAT::Command cmd; };
    static const Entry entries[] = {
        {"NOP", EtherCAT::Command::NOP},
        {"APRD", EtherCAT::Command::APRD},
        {"APWR", EtherCAT::Command::APWR},
        {"APRW", EtherCAT::Command::APRW},
        {"FPRD", EtherCAT::Command::FPRD},
        {"FPWR", EtherCAT::Command::FPWR},
        {"FPRW", EtherCAT::Command::FPRW},
        {"BRD", EtherCAT::Command::BRD},
        {"BWR", EtherCAT::Command::BWR},
        {"BRW", EtherCAT::Command::BRW},
        {"LRD", EtherCAT::Command::LRD},
        {"LWR", EtherCAT::Command::LWR},
        {"LRW", EtherCAT::Command::LRW},
        {"ARMW", EtherCAT::Command::ARMW},
        {"FRMW", EtherCAT::Command::FRMW},
    };
    for (const auto& e : entries) {
        if (u == e.name) return e.cmd;
    }
    return std::nullopt;
}

std::string hex16(uint16_t v) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "0x%04X", v);
    return buf;
}

std::string hex32(uint32_t v) {
    char buf[12];
    std::snprintf(buf, sizeof(buf), "0x%08X", v);
    return buf;
}

std::string hexBytes(const uint8_t* data, size_t len, size_t max = 0) {
    std::string s;
    size_t n = (max > 0 && len > max) ? max : len;
    char buf[4];
    for (size_t i = 0; i < n; ++i) {
        if (i > 0) s += ' ';
        std::snprintf(buf, sizeof(buf), "%02X", data[i]);
        s += buf;
    }
    if (max > 0 && len > max) {
        s += " ... (" + std::to_string(len - max) + " more)";
    }
    return s;
}

bool isAutoPositionCommand(EtherCAT::Command cmd) {
    return cmd == EtherCAT::Command::APRD || cmd == EtherCAT::Command::APWR ||
           cmd == EtherCAT::Command::APRW;
}

bool isLogicalCommand(EtherCAT::Command cmd) {
    return cmd == EtherCAT::Command::LRD || cmd == EtherCAT::Command::LWR ||
           cmd == EtherCAT::Command::LRW;
}

bool isPhysicalCommand(EtherCAT::Command cmd) {
    return cmd == EtherCAT::Command::APRD || cmd == EtherCAT::Command::APWR ||
           cmd == EtherCAT::Command::APRW ||
           cmd == EtherCAT::Command::FPRD || cmd == EtherCAT::Command::FPWR ||
           cmd == EtherCAT::Command::FPRW;
}

const char* accessPrefix(EtherCAT::Command cmd) {
    if (isLogicalCommand(cmd)) return "L";
    if (isPhysicalCommand(cmd)) return "P";
    if (cmd == EtherCAT::Command::BRD || cmd == EtherCAT::Command::BWR) return "B";
    return "";
}

// Use EtherCAT::isReadCommand and EtherCAT::isWriteCommand directly

std::string formatMs(uint64_t ns) {
    double ms = static_cast<double>(ns) / 1e6;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << ms;
    return oss.str();
}

uint64_t g_baseTs = 0;

void setBaseTimestamp(uint64_t ts) { g_baseTs = ts; }

std::string tsRelMs(uint64_t ns) {
    return formatMs(ns - g_baseTs);
}

// ============================================================================
// Minimal CoE/mailbox wire structures for parsing
// ============================================================================

struct MbxHeaderLite {
    uint16_t length_le;
    uint16_t address_le;
    uint8_t  priority;
    uint8_t  mbxtype;  // low nibble = type, high nibble = counter
};

struct CoeHeaderLite {
    uint16_t raw_le;  // number[9] | reserved[3] | service[4]
};

static constexpr uint8_t EC_MBXT_COE    = 0x03;
static constexpr uint8_t EC_COES_SDOREQ = 0x02;
static constexpr uint8_t EC_COES_SDORES = 0x03;
static constexpr uint8_t EC_SDO_UP_REQ   = 0x40;
static constexpr uint8_t EC_SDO_DOWN_REQ = 0x20;
static constexpr uint8_t EC_SDO_ABORT    = 0x80;

struct CoeTransactionInfo {
    uint8_t mbxCounter;
    uint16_t coeNumber;
    uint8_t service;   // SDOREQ or SDORES
    uint8_t sdoCmd;    // command byte
    uint16_t odIndex;
    uint8_t  odSub;
    bool isUpload;
    bool isDownload;
    bool isAbort;
    bool isResponse;
    std::vector<uint8_t> data;
    uint32_t abortCode;
    uint64_t timestampNs;
    uint16_t slaveAdp;
    bool isError;
};

std::optional<CoeTransactionInfo> tryParseCoe(const PCP::EtherCATDatagramInfo& dg, uint64_t ts) {
    if (dg.data.size() < sizeof(MbxHeaderLite) + sizeof(CoeHeaderLite) + 1)
        return std::nullopt;

    const uint8_t* p = dg.data.data();
    MbxHeaderLite mbx;
    std::memcpy(&mbx, p, sizeof(mbx));
    const uint8_t mbxType = mbx.mbxtype & 0x0F;
    if (mbxType != EC_MBXT_COE) return std::nullopt;

    CoeHeaderLite coe;
    std::memcpy(&coe, p + sizeof(MbxHeaderLite), sizeof(coe));
    const uint16_t coeRaw = coe.raw_le;  // already LE in memory
    const uint16_t coeNumber = coeRaw & 0x01FF;
    const uint8_t service = (coeRaw >> 12) & 0x0F;

    const uint8_t* sdoBytes = p + sizeof(MbxHeaderLite) + sizeof(CoeHeaderLite);
    const size_t sdoLen = dg.data.size() - (sizeof(MbxHeaderLite) + sizeof(CoeHeaderLite));
    if (sdoLen < 1) return std::nullopt;

    CoeTransactionInfo info;
    info.mbxCounter = (mbx.mbxtype >> 4) & 0x0F;
    info.coeNumber = coeNumber;
    info.service = service;
    info.sdoCmd = sdoBytes[0];
    info.isUpload = (info.sdoCmd & 0xE0) == EC_SDO_UP_REQ ||
                    (info.sdoCmd & 0xE0) == 0x60;  // segmented upload
    info.isDownload = (info.sdoCmd & 0xE0) == EC_SDO_DOWN_REQ;
    info.isAbort = (info.sdoCmd & 0xE0) == EC_SDO_ABORT;
    info.isResponse = (service == EC_COES_SDORES);
    info.timestampNs = ts;
    info.slaveAdp = dg.adp;
    info.isError = info.isAbort;
    info.abortCode = 0;

    if (sdoLen >= 8) {
        std::memcpy(&info.odIndex, sdoBytes + 1, 2);  // LE
        info.odSub = sdoBytes[3];
        if (info.isAbort && sdoLen >= 8) {
            std::memcpy(&info.abortCode, sdoBytes + 4, 4);
        }
        // For expedited upload response, data is in bytes 4-7
        if (info.isResponse && (info.sdoCmd & 0x02) != 0) {
            const uint8_t n = (info.sdoCmd >> 2) & 0x03;
            size_t dataBytes = 4 - n;
            info.data.assign(sdoBytes + 4, sdoBytes + 4 + dataBytes);
        } else if (info.isDownload && (info.sdoCmd & 0x02) != 0) {
            // Expedited download: data in bytes 4-7
            const uint8_t n = (info.sdoCmd >> 2) & 0x03;
            size_t dataBytes = 4 - n;
            info.data.assign(sdoBytes + 4, sdoBytes + 4 + dataBytes);
        } else if (sdoLen > 8) {
            info.data.assign(sdoBytes + 8, sdoBytes + sdoLen);
        }
    }

    return info;
}

// ============================================================================
// EtherCAT Transaction (request/response correlation by idx)
// ============================================================================

struct Transaction {
    uint8_t idx;
    EtherCAT::Command cmd;
    uint16_t adp;
    uint16_t ado;
    uint16_t dataLength;
    bool hasRequest;
    bool hasResponse;
    std::vector<uint8_t> reqData;
    std::vector<uint8_t> respData;
    uint16_t reqWkc;
    uint16_t respWkc;
    uint64_t reqTs;
    uint64_t respTs;
    uint16_t frameIndex;

    uint32_t logicalAddress() const {
        return (static_cast<uint32_t>(ado) << 16) | adp;
    }

    bool isError() const {
        if (EtherCAT::isWriteCommand(cmd) && hasResponse && respWkc == 0) return true;
        if (hasResponse && respWkc == 0 && EtherCAT::isReadCommand(cmd)) return true;
        return false;
    }
};

// ============================================================================
// Mailbox configuration event
// ============================================================================

// SM register layout: 8 bytes per Sync Manager
//   offset 0-1: StartAddr (uint16_t LE)
//   offset 2-3: Length     (uint16_t LE)
//   offset 4:   Control    (uint8_t)
//   offset 5:   Status     (uint8_t)
//   offset 6:   Activate   (uint8_t)
//   offset 7:   PDI Control (uint8_t)

struct SmRegInfo {
    uint8_t smIndex;
    uint8_t byteOffset;  // 0-7 within the 8-byte SM block
    const char* name;
    bool isTwoByte;
};

std::optional<SmRegInfo> smRegInfo(uint16_t addr) {
    if (addr < 0x0800 || addr > 0x081F) return std::nullopt;
    uint16_t rel = addr - 0x0800;
    uint8_t sm = rel / 8;
    uint8_t off = rel % 8;
    if (sm > 3) return std::nullopt;
    switch (off) {
        case 0: return SmRegInfo{sm, 0, "StartAddr", true};
        case 1: return SmRegInfo{sm, 1, "StartAddr+1", false};  // high byte — rarely accessed alone
        case 2: return SmRegInfo{sm, 2, "Length", true};
        case 3: return SmRegInfo{sm, 3, "Length+1", false};
        case 4: return SmRegInfo{sm, 4, "Control", false};
        case 5: return SmRegInfo{sm, 5, "Status", false};
        case 6: return SmRegInfo{sm, 6, "Activate", false};
        case 7: return SmRegInfo{sm, 7, "PDIControl", false};
        default: return std::nullopt;
    }
}

bool isSmRegister(uint16_t ado) {
    return ado >= 0x0800 && ado <= 0x081F;
}

bool isAlRegister(uint16_t ado) {
    return ado >= 0x0120 && ado <= 0x013F;
}

bool isMailboxArea(uint16_t ado) {
    return ado >= 0x1000 && ado <= 0x1FFF;
}

std::string smRegisterName(uint16_t addr) {
    auto info = smRegInfo(addr);
    if (!info) return hex16(addr);
    return "SM" + std::to_string(info->smIndex) + "." + info->name + " " + hex16(addr);
}

std::string alRegisterName(uint16_t addr) {
    switch (addr) {
        case 0x0120: return "AL_CONTROL " + hex16(addr);
        case 0x0130: return "AL_STATUS " + hex16(addr);
        case 0x0134: return "AL_STATUS_CODE " + hex16(addr);
        default: return "AL? " + hex16(addr);
    }
}

// --- Register value interpretation ---

std::string interpretSmControl(uint8_t ctrl) {
    std::string s;
    uint8_t mode = ctrl & 0x03;
    switch (mode) {
        case 0x00: s += "Buffered"; break;
        case 0x02: s += "Mailbox"; break;
        case 0x03: s += "3PDO"; break;
        default: s += "Mode?"; break;
    }
    if (ctrl & 0x04) s += ", Write";
    else             s += ", Read";
    if (ctrl & 0x10) s += ", IRQ_ECAT";
    if (ctrl & 0x20) s += ", IRQ_PDI";
    if (ctrl & 0x40) s += ", Watchdog";
    return s;
}

std::string interpretSmStatus(uint8_t status) {
    std::string s;
    if (status & 0x01) s += "IntWrite ";
    if (status & 0x02) s += "IntRead ";
    if (status & 0x08) s += "MailboxFull ";
    uint8_t bufState = (status >> 4) & 0x03;
    s += "buf=" + std::to_string(bufState);
    return s;
}

std::string interpretSmActivate(uint8_t act) {
    if (act & 0x01) return "enabled";
    return "disabled";
}

std::string interpretSlaveState(uint8_t state) {
    switch (state & 0x0F) {
        case 0x01: return "INIT";
        case 0x02: return "PRE-OP";
        case 0x03: return "BOOT";
        case 0x04: return "SAFE-OP";
        case 0x08: return "OP";
        default: return "UNKNOWN(0x" + hexBytes(&state, 1) + ")";
    }
}

std::string interpretRegisterValue(uint16_t addr, const uint8_t* data, size_t len) {
    if (len == 0) return "(empty)";

    // SM registers
    auto smi = smRegInfo(addr);
    if (smi) {
        std::string raw;
        if (smi->isTwoByte && len >= 2) {
            uint16_t val;
            std::memcpy(&val, data, 2);
            raw = hex16(val);
        } else {
            raw = hexBytes(data, len);
        }

        std::string desc;
        if (smi->byteOffset == 0 && len >= 2) {
            uint16_t val;
            std::memcpy(&val, data, 2);
            desc = "addr=" + hex16(val);
        } else if (smi->byteOffset == 2 && len >= 2) {
            uint16_t val;
            std::memcpy(&val, data, 2);
            desc = std::to_string(val) + " bytes";
        } else if (smi->byteOffset == 4) {
            desc = interpretSmControl(data[0]);
        } else if (smi->byteOffset == 5) {
            desc = interpretSmStatus(data[0]);
        } else if (smi->byteOffset == 6) {
            desc = interpretSmActivate(data[0]);
        } else {
            desc = raw;
        }
        return raw + " (" + desc + ")";
    }

    // AL registers
    switch (addr) {
        case 0x0120: {  // AL_CONTROL
            if (len >= 1) {
                uint8_t state = data[0] & 0x0F;
                uint16_t raw16 = 0;
                if (len >= 2) std::memcpy(&raw16, data, 2);
                return hex16(raw16) + " (" + interpretSlaveState(state) + ")";
            }
            break;
        }
        case 0x0130: {  // AL_STATUS
            if (len >= 1) {
                uint8_t state = data[0] & 0x0F;
                bool error = (data[0] & 0x10) != 0;
                uint16_t raw16 = 0;
                if (len >= 2) std::memcpy(&raw16, data, 2);
                std::string desc = interpretSlaveState(state);
                if (error) desc += ", ERROR";
                return hex16(raw16) + " (" + desc + ")";
            }
            break;
        }
        case 0x0134: {  // AL_STATUS_CODE
            if (len >= 2) {
                uint16_t code;
                std::memcpy(&code, data, 2);
                return hex16(code) + " (" + EtherCAT::alcode::alStatusCodeToString(code) + ")";
            }
            break;
        }
    }

    // Generic fallback
    if (len == 1) return hexBytes(data, len);
    if (len == 2) { uint16_t v; std::memcpy(&v, data, 2); return hex16(v); }
    if (len == 4) { uint32_t v; std::memcpy(&v, data, 4); return hex32(v); }
    return hexBytes(data, len, 64);
}

// ============================================================================
// Filter matching
// ============================================================================

bool datagramPassesFilters(const PCP::EtherCATDatagramInfo& dg, const Filters& f) {
    if (f.command && dg.cmd != *f.command) return false;
    if (f.addr && dg.ado != *f.addr) return false;
    if (f.wkc && dg.wkc != *f.wkc) return false;
    if (f.errorsOnly) {
        if (EtherCAT::isWriteCommand(dg.cmd) && dg.wkc == 0) return true;
        if (EtherCAT::isReadCommand(dg.cmd) && dg.wkc == 0) return true;
        return false;
    }
    if (f.slave) {
        if (isAutoPositionCommand(dg.cmd)) {
            if (dg.adp != *f.slave) return false;
        } else if (isLogicalCommand(dg.cmd)) {
            // Can't filter by slave for logical commands without address map
        } else {
            if (dg.adp != *f.slave) return false;
        }
    }
    if (f.pdoAddr && isLogicalCommand(dg.cmd)) {
        if (dg.logicalAddress() != *f.pdoAddr) return false;
    }
    return true;
}

bool framePassesFilters(const PCP::InterpretedFrame& frame, const Filters& f) {
    if (f.onlyEtherCAT && !frame.isEtherCAT) return false;
    if (f.vlan && (!frame.vlanId.has_value() || *frame.vlanId != *f.vlan)) return false;
    return true;
}

// ============================================================================
// Selection: Raw (existing output, enhanced with UTF-8)
// ============================================================================

void displayRaw(const std::vector<PCP::InterpretedFrame>& frames, const Filters& f) {
    std::cout << Utf8Formatter::titledBox("Raw Frame Dump");
    uint64_t shown = 0;
    uint64_t packetIndex = 0;

    for (const auto& frame : frames) {
        ++packetIndex;
        if (f.maxPackets > 0 && shown >= f.maxPackets) break;
        if (!framePassesFilters(frame, f)) continue;

        bool hasMatchingDg = false;
        for (const auto& dg : frame.datagrams) {
            if (datagramPassesFilters(dg, f)) { hasMatchingDg = true; break; }
        }
        if (!f.command && !f.addr && !f.wkc && !f.errorsOnly && !f.pdoAddr) {
            hasMatchingDg = true;
        }
        if (!hasMatchingDg) continue;

        std::cout << "\n" << Utf8Formatter::bullet << " Packet " << packetIndex << "\n";
        std::cout << PCP::formatInterpretedFrame(frame, f.verbose, f.maxData);
        ++shown;
    }
    std::cout << "\n" << Utf8Formatter::diamond << " Total shown: " << shown << " / " << packetIndex << "\n";
}

// ============================================================================
// Selection: EtherCAT Transactions
// ============================================================================

void displayEthercatTransactions(const std::vector<PCP::InterpretedFrame>& frames, const Filters& f) {
    std::cout << Utf8Formatter::titledBox("EtherCAT Transactions");

    // Map idx -> pending request datagram
    std::map<uint8_t, Transaction> pending;
    std::vector<Transaction> completed;
    uint64_t frameIdx = 0;

    for (const auto& frame : frames) {
        ++frameIdx;
        if (!frame.isEtherCAT) continue;

        for (const auto& dg : frame.datagrams) {
            if (!datagramPassesFilters(dg, f)) continue;

            const bool isTx = (frame.direction == PCP::PacketDirection::Outbound);
            const bool isRx = (frame.direction == PCP::PacketDirection::Inbound);

            if (isTx && dg.wkc == 0) {
                // Request
                Transaction& txn = pending[dg.idx];
                txn.idx = dg.idx;
                txn.cmd = dg.cmd;
                txn.adp = dg.adp;
                txn.ado = dg.ado;
                txn.dataLength = dg.dataLength;
                txn.hasRequest = true;
                txn.reqData = dg.data;
                txn.reqWkc = dg.wkc;
                txn.reqTs = frame.timestampNs;
                txn.frameIndex = frameIdx;
            } else {
                // Response (RX or TX with WKC > 0)
                auto it = pending.find(dg.idx);
                if (it != pending.end()) {
                    Transaction& txn = it->second;
                    txn.hasResponse = true;
                    txn.respData = dg.data;
                    txn.respWkc = dg.wkc;
                    txn.respTs = frame.timestampNs;
                    if (f.errorsOnly && !txn.isError()) {
                        pending.erase(it);
                        continue;
                    }
                    completed.push_back(txn);
                    pending.erase(it);
                } else {
                    // No matching request — create standalone
                    Transaction txn;
                    txn.idx = dg.idx;
                    txn.cmd = dg.cmd;
                    txn.adp = dg.adp;
                    txn.ado = dg.ado;
                    txn.dataLength = dg.dataLength;
                    txn.hasRequest = false;
                    txn.hasResponse = true;
                    txn.respData = dg.data;
                    txn.respWkc = dg.wkc;
                    txn.respTs = frame.timestampNs;
                    txn.reqWkc = 0;
                    txn.frameIndex = frameIdx;
                    if (!f.errorsOnly || txn.isError()) {
                        completed.push_back(txn);
                    }
                }
            }
        }
    }

    // Display completed transactions
    uint64_t shown = 0;
    for (const auto& txn : completed) {
        if (f.maxPackets > 0 && shown >= f.maxPackets) break;

        std::cout << Utf8Formatter::bullet << " " << tsRelMs(txn.reqTs) << "ms"
                  << "  idx=" << static_cast<int>(txn.idx)
                  << "  " << EtherCAT::commandToString(txn.cmd);

        if (isAutoPositionCommand(txn.cmd)) {
            std::cout << "  slave=" << txn.adp << "  addr=" << hex16(txn.ado);
        } else if (isLogicalCommand(txn.cmd)) {
            std::cout << "  logAddr=" << hex32(txn.logicalAddress());
        } else {
            std::cout << "  adp=" << hex16(txn.adp) << "  ado=" << hex16(txn.ado);
        }

        std::cout << "  len=" << txn.dataLength;

        if (txn.hasResponse) {
            if (txn.isError()) {
                std::cout << "  " << Utf8Formatter::cross << " wkc=" << txn.respWkc;
            } else {
                std::cout << "  " << Utf8Formatter::check << " wkc=" << txn.respWkc;
            }
            if (!txn.respData.empty() && f.verbose) {
                std::cout << "  data=" << hexBytes(txn.respData.data(), txn.respData.size(), f.maxData);
            } else if (!txn.respData.empty() && txn.respData.size() <= 4) {
                std::cout << "  data=" << hexBytes(txn.respData.data(), txn.respData.size());
            }
        } else if (txn.hasRequest) {
            std::cout << "  (no response)";
        }

        std::cout << "\n";
        ++shown;
    }

    // Show unmatched pending requests
    for (const auto& [idx, txn] : pending) {
        if (f.maxPackets > 0 && shown >= f.maxPackets) break;
        if (f.errorsOnly && !txn.isError()) continue;
        std::cout << Utf8Formatter::bullet << " " << tsRelMs(txn.reqTs) << "ms"
                  << "  idx=" << static_cast<int>(idx)
                  << "  " << EtherCAT::commandToString(txn.cmd)
                  << "  (pending, no response)\n";
        ++shown;
    }

    std::cout << "\n" << Utf8Formatter::diamond << " Transactions shown: " << shown
              << " | Pending: " << pending.size() << "\n";
}

// ============================================================================
// Selection: Mailbox Configuration (transaction-correlated)
// ============================================================================

struct MbxConfigTxn {
    uint8_t idx;
    uint16_t adp;
    uint16_t addr;
    bool isWrite;
    EtherCAT::Command cmd;
    std::vector<uint8_t> reqData;
    std::vector<uint8_t> respData;
    uint16_t wkc;
    bool hasResponse;
    uint64_t reqTs;
    uint64_t respTs;
    std::string description;
};

void displayMailbox(const std::vector<PCP::InterpretedFrame>& frames, const Filters& f) {
    std::cout << Utf8Formatter::titledBox("Mailbox");

    // Correlate request/response by idx — first occurrence is request,
    // second occurrence is response. This works regardless of frame
    // direction metadata, which may be Unknown in some captures.
    std::map<uint8_t, MbxConfigTxn> pending;
    std::vector<MbxConfigTxn> completed;

    for (const auto& frame : frames) {
        if (!frame.isEtherCAT) continue;

        for (const auto& dg : frame.datagrams) {
            if (!datagramPassesFilters(dg, f)) continue;

            if (!isAutoPositionCommand(dg.cmd) && dg.cmd != EtherCAT::Command::FPRD &&
                dg.cmd != EtherCAT::Command::FPWR)
                continue;
            if (!isSmRegister(dg.ado) && !isAlRegister(dg.ado) && !isMailboxArea(dg.ado))
                continue;

            auto it = pending.find(dg.idx);
            if (it == pending.end()) {
                // First occurrence → request
                MbxConfigTxn& txn = pending[dg.idx];
                txn.idx = dg.idx;
                txn.adp = dg.adp;
                txn.addr = dg.ado;
                txn.isWrite = EtherCAT::isWriteCommand(dg.cmd);
                txn.cmd = dg.cmd;
                txn.reqData = dg.data;
                txn.wkc = 0;
                txn.hasResponse = false;
                txn.reqTs = frame.timestampNs;
                txn.respTs = 0;
                if (isSmRegister(dg.ado))
                    txn.description = smRegisterName(dg.ado);
                else if (isAlRegister(dg.ado))
                    txn.description = alRegisterName(dg.ado);
                else
                    txn.description = "MbxData " + hex16(dg.ado);
            } else {
                // Second occurrence → response
                MbxConfigTxn& txn = it->second;
                txn.hasResponse = true;
                txn.respData = dg.data;
                txn.wkc = dg.wkc;
                txn.respTs = frame.timestampNs;
                if (f.errorsOnly && dg.wkc != 0) {
                    pending.erase(it);
                    continue;
                }
                completed.push_back(txn);
                pending.erase(it);
            }
        }
    }

    // Display completed transactions
    uint64_t shown = 0;
    for (const auto& txn : completed) {
        if (f.maxPackets > 0 && shown >= f.maxPackets) break;

        std::cout << Utf8Formatter::bullet << " " << tsRelMs(txn.reqTs) << "ms"
                  << "  slave=" << txn.adp
                  << "  " << accessPrefix(txn.cmd) << (txn.isWrite ? "WR" : "RD")
                  << "  " << txn.description;

        // Show value with interpretation
        const std::vector<uint8_t>& valData = txn.isWrite ? txn.reqData : txn.respData;
        if (!valData.empty()) {
            std::cout << "  " << Utf8Formatter::arrow << " "
                      << interpretRegisterValue(txn.addr, valData.data(), valData.size());
        }

        // WKC and status
        if (txn.hasResponse) {
            std::cout << "  wkc=" << txn.wkc;
            if (txn.wkc == 0) std::cout << "  " << Utf8Formatter::cross;
            else std::cout << "  " << Utf8Formatter::check;
        } else {
            std::cout << "  (no response)  " << Utf8Formatter::cross;
        }

        std::cout << "\n";
        ++shown;
    }

    // Show pending (no response)
    for (const auto& [idx, txn] : pending) {
        if (f.maxPackets > 0 && shown >= f.maxPackets) break;
        if (f.errorsOnly) {
            std::cout << Utf8Formatter::bullet << " " << tsRelMs(txn.reqTs) << "ms"
                      << "  slave=" << txn.adp
                      << "  " << accessPrefix(txn.cmd) << (txn.isWrite ? "WR" : "RD")
                      << "  " << txn.description;
            if (!txn.reqData.empty())
                std::cout << "  " << Utf8Formatter::arrow << " "
                          << interpretRegisterValue(txn.addr, txn.reqData.data(), txn.reqData.size());
            std::cout << "  (no response)  " << Utf8Formatter::cross << "\n";
            ++shown;
        }
    }

    std::cout << "\n" << Utf8Formatter::diamond << " Config transactions: " << shown
              << " | Pending: " << pending.size() << "\n";
}

// ============================================================================
// Selection: CoE Transactions
// ============================================================================

void displayCoeTransactions(const std::vector<PCP::InterpretedFrame>& frames, const Filters& f) {
    std::cout << Utf8Formatter::titledBox("CoE Transactions");

    // Track pending CoE requests by (coeNumber, mbxCounter)
    std::map<std::pair<uint16_t, uint8_t>, CoeTransactionInfo> pending;
    uint64_t shown = 0;

    for (const auto& frame : frames) {
        if (!frame.isEtherCAT) continue;

        for (const auto& dg : frame.datagrams) {
            if (!datagramPassesFilters(dg, f)) continue;

            // Check if this is a mailbox write to a slave
            if (!isAutoPositionCommand(dg.cmd) && dg.cmd != EtherCAT::Command::FPWR) continue;

            auto coeOpt = tryParseCoe(dg, frame.timestampNs);
            if (!coeOpt) continue;

            auto& coe = *coeOpt;

            // Apply CoE filters
            if (f.coeIndex && coe.odIndex != *f.coeIndex) continue;
            if (f.coeSub && coe.odSub != *f.coeSub) continue;
            if (f.errorsOnly && !coe.isError) continue;

            auto key = std::make_pair(coe.coeNumber, coe.mbxCounter);

            if (!coe.isResponse) {
                // Request
                pending[key] = coe;
            } else {
                // Response — try to match with request
                auto it = pending.find(key);
                if (it != pending.end()) {
                    auto& req = it->second;

                    std::cout << Utf8Formatter::bullet << " " << tsRelMs(req.timestampNs) << "ms"
                              << "  slave=" << dg.adp;

                    if (req.isUpload) {
                        std::cout << "  SDO Upload  index=" << hex16(req.odIndex)
                                  << ":" << static_cast<int>(req.odSub);
                    } else if (req.isDownload) {
                        std::cout << "  SDO Download  index=" << hex16(req.odIndex)
                                  << ":" << static_cast<int>(req.odSub);
                        if (!req.data.empty()) {
                            std::cout << "  data=" << hexBytes(req.data.data(), req.data.size(), f.maxData);
                        }
                    } else {
                        std::cout << "  SDO  index=" << hex16(req.odIndex)
                                  << ":" << static_cast<int>(req.odSub);
                    }

                    std::cout << "  " << Utf8Formatter::arrow;

                    if (coe.isAbort) {
                        std::cout << "  " << Utf8Formatter::cross << " ABORT  code=" << hex32(coe.abortCode);
                    } else if (!coe.data.empty()) {
                        std::cout << "  " << Utf8Formatter::check << " data=" << hexBytes(coe.data.data(), coe.data.size(), f.maxData);
                    } else {
                        std::cout << "  " << Utf8Formatter::check << " OK";
                    }

                    std::cout << "\n";
                    ++shown;
                    pending.erase(it);
                } else {
                    // Standalone response
                    std::cout << Utf8Formatter::bullet << " " << tsRelMs(coe.timestampNs) << "ms"
                              << "  slave=" << dg.adp
                              << "  SDO Response  index=" << hex16(coe.odIndex)
                              << ":" << static_cast<int>(coe.odSub);
                    if (coe.isAbort) {
                        std::cout << "  " << Utf8Formatter::cross << " ABORT  code=" << hex32(coe.abortCode);
                    } else if (!coe.data.empty()) {
                        std::cout << "  " << Utf8Formatter::check << " data=" << hexBytes(coe.data.data(), coe.data.size(), f.maxData);
                    } else {
                        std::cout << "  " << Utf8Formatter::check;
                    }
                    std::cout << "  (no matching request)\n";
                    ++shown;
                }
            }
        }
    }

    // Show pending requests without responses
    for (const auto& [key, req] : pending) {
        if (f.maxPackets > 0 && shown >= f.maxPackets) break;
        if (f.errorsOnly) {
            std::cout << Utf8Formatter::bullet << " " << tsRelMs(req.timestampNs) << "ms"
                      << "  slave=" << req.slaveAdp
                      << "  SDO  index=" << hex16(req.odIndex)
                      << ":" << static_cast<int>(req.odSub)
                      << "  (timeout, no response)  " << Utf8Formatter::cross << "\n";
            ++shown;
        } else {
            std::cout << Utf8Formatter::bullet << " " << tsRelMs(req.timestampNs) << "ms"
                      << "  slave=" << req.slaveAdp
                      << "  SDO  index=" << hex16(req.odIndex)
                      << ":" << static_cast<int>(req.odSub)
                      << "  (pending)\n";
            ++shown;
        }
    }

    std::cout << "\n" << Utf8Formatter::diamond << " CoE transactions shown: " << shown
              << " | Pending: " << pending.size() << "\n";
}

// ============================================================================
// Selection: PDO
// ============================================================================

void displayPdo(const std::vector<PCP::InterpretedFrame>& frames, const Filters& f) {
    std::cout << Utf8Formatter::titledBox("PDO (Process Data)");

    uint64_t shown = 0;
    uint64_t frameIdx = 0;

    for (const auto& frame : frames) {
        ++frameIdx;
        if (!frame.isEtherCAT) continue;
        if (!framePassesFilters(frame, f)) continue;

        bool hasPdo = false;
        for (const auto& dg : frame.datagrams) {
            if (!isLogicalCommand(dg.cmd)) continue;
            if (f.pdoAddr && dg.logicalAddress() != *f.pdoAddr) continue;
            if (f.wkc && dg.wkc != *f.wkc) continue;
            if (f.errorsOnly && dg.wkc != 0) continue;
            hasPdo = true;
        }
        if (!hasPdo) continue;
        if (f.maxPackets > 0 && shown >= f.maxPackets) break;

        std::cout << Utf8Formatter::bullet << " " << tsRelMs(frame.timestampNs) << "ms"
                  << "  Frame " << frameIdx;
        if (frame.direction == PCP::PacketDirection::Outbound) std::cout << "  [TX]";
        else if (frame.direction == PCP::PacketDirection::Inbound) std::cout << "  [RX]";
        std::cout << "\n";

        for (const auto& dg : frame.datagrams) {
            if (!isLogicalCommand(dg.cmd)) continue;
            if (f.pdoAddr && dg.logicalAddress() != *f.pdoAddr) continue;
            if (f.wkc && dg.wkc != *f.wkc) continue;
            if (f.errorsOnly && dg.wkc != 0) continue;

            std::cout << "    " << EtherCAT::commandToString(dg.cmd)
                      << "  logAddr=" << hex32(dg.logicalAddress())
                      << "  len=" << dg.dataLength
                      << "  wkc=" << dg.wkc;

            if (dg.wkc == 0) std::cout << "  " << Utf8Formatter::cross;
            else std::cout << "  " << Utf8Formatter::check;

            if (f.verbose && !dg.data.empty()) {
                std::cout << "\n      data: " << hexBytes(dg.data.data(), dg.data.size(), f.maxData);
            } else if (!dg.data.empty() && dg.data.size() <= 8) {
                std::cout << "  data=" << hexBytes(dg.data.data(), dg.data.size());
            }
            std::cout << "\n";
        }
        ++shown;
    }

    std::cout << "\n" << Utf8Formatter::diamond << " PDO frames shown: " << shown << "\n";
}

// ============================================================================
// Selection: Statistics
// ============================================================================

void displayStatistics(const std::vector<PCP::InterpretedFrame>& frames, const Filters& f) {
    std::cout << Utf8Formatter::titledBox("Statistics");

    uint64_t totalFrames = 0;
    uint64_t ethercatFrames = 0;
    uint64_t totalDatagrams = 0;
    std::map<EtherCAT::Command, uint64_t> cmdCounts;
    std::map<uint16_t, uint64_t> wkcDistribution;  // wkc -> count
    uint64_t errorCount = 0;
    uint64_t coeCount = 0;
    uint64_t coeAbortCount = 0;
    uint64_t pdoFrames = 0;
    uint64_t mailboxFrames = 0;

    uint64_t firstTs = 0, lastTs = 0;
    bool haveTs = false;

    std::set<uint16_t> seenSlaves;
    std::set<uint32_t> seenLogicalAddrs;

    for (const auto& frame : frames) {
        ++totalFrames;
        if (!haveTs) { firstTs = frame.timestampNs; haveTs = true; }
        lastTs = frame.timestampNs;

        if (!frame.isEtherCAT) continue;
        ++ethercatFrames;

        bool hasPdo = false;
        bool hasMailbox = false;
        bool hasCoe = false;

        for (const auto& dg : frame.datagrams) {
            ++totalDatagrams;
            cmdCounts[dg.cmd]++;
            wkcDistribution[dg.wkc]++;

            if (EtherCAT::isWriteCommand(dg.cmd) && dg.wkc == 0) ++errorCount;
            if (EtherCAT::isReadCommand(dg.cmd) && dg.wkc == 0 && dg.cmd != EtherCAT::Command::BRD) ++errorCount;

            if (isAutoPositionCommand(dg.cmd)) {
                seenSlaves.insert(dg.adp);
            }
            if (isLogicalCommand(dg.cmd)) {
                hasPdo = true;
                seenLogicalAddrs.insert(dg.logicalAddress());
            }
            if (isAutoPositionCommand(dg.cmd) && isMailboxArea(dg.ado)) {
                hasMailbox = true;
                auto coeOpt = tryParseCoe(dg, frame.timestampNs);
                if (coeOpt) {
                    hasCoe = true;
                    ++coeCount;
                    if (coeOpt->isAbort) ++coeAbortCount;
                }
            }
        }

        if (hasPdo) ++pdoFrames;
        if (hasMailbox) ++mailboxFrames;
    }

    std::cout << Utf8Formatter::bullet << " Total frames:       " << totalFrames << "\n";
    std::cout << Utf8Formatter::bullet << " EtherCAT frames:    " << ethercatFrames << "\n";
    std::cout << Utf8Formatter::bullet << " Non-EtherCAT:       " << (totalFrames - ethercatFrames) << "\n";
    std::cout << Utf8Formatter::bullet << " Total datagrams:    " << totalDatagrams << "\n";
    std::cout << "\n";

    std::cout << Utf8Formatter::bullet << " Unique slaves:      " << seenSlaves.size() << "\n";
    for (uint16_t s : seenSlaves) {
        std::cout << "    slave " << s << "\n";
    }
    std::cout << "\n";

    std::cout << Utf8Formatter::bullet << " Logical addresses:  " << seenLogicalAddrs.size() << "\n";
    for (uint32_t a : seenLogicalAddrs) {
        std::cout << "    " << hex32(a) << "\n";
    }
    std::cout << "\n";

    std::cout << Utf8Formatter::bullet << " Command distribution:\n";
    for (const auto& [cmd, count] : cmdCounts) {
        std::cout << "    " << EtherCAT::commandToString(cmd)
                  << ": " << count << "\n";
    }
    std::cout << "\n";

    std::cout << Utf8Formatter::bullet << " WKC distribution:\n";
    for (const auto& [wkc, count] : wkcDistribution) {
        std::cout << "    wkc=" << wkc << ": " << count << " datagrams";
        if (wkc == 0) std::cout << "  " << Utf8Formatter::cross;
        std::cout << "\n";
    }
    std::cout << "\n";

    std::cout << Utf8Formatter::bullet << " Errors (wkc=0):     " << errorCount << "\n";
    std::cout << Utf8Formatter::bullet << " CoE transactions:   " << coeCount << "\n";
    std::cout << Utf8Formatter::bullet << " CoE aborts:         " << coeAbortCount << "  " << Utf8Formatter::cross << "\n";
    std::cout << Utf8Formatter::bullet << " PDO frames:         " << pdoFrames << "\n";
    std::cout << Utf8Formatter::bullet << " Mailbox frames:     " << mailboxFrames << "\n";

    if (haveTs && totalFrames > 1) {
        double durationSec = static_cast<double>(lastTs - firstTs) / 1e9;
        std::cout << "\n";
        std::cout << Utf8Formatter::bullet << " Capture duration:   " << std::fixed
                  << std::setprecision(6) << durationSec << " s\n";
        if (durationSec > 0) {
            std::cout << Utf8Formatter::bullet << " Frame rate:         " << std::fixed
                      << std::setprecision(1)
                      << (static_cast<double>(totalFrames) / durationSec) << " fps\n";
            if (pdoFrames > 0) {
                std::cout << Utf8Formatter::bullet << " PDO cycle time:     " << std::fixed
                          << std::setprecision(3)
                          << (durationSec / pdoFrames * 1000.0) << " ms\n";
            }
        }
    }
    std::cout << "\n";
}

} // anonymous namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser program("interpret_pcapng");
    program.add_argument("input")
        .help("Path to the pcapng file to interpret");

    // Selections
    program.add_argument("--select")
        .default_value(std::string{"raw"})
        .help("Selection(s): raw, ethercat-transactions, mailbox, "
              "coe-transactions, pdo, statistics (comma-separated or repeat)");

    // Display filters
    program.add_argument("--slave")
        .default_value(uint64_t{0})
        .scan<'u', uint64_t>()
        .help("Filter by slave position (0 = all)");
    program.add_argument("--addr")
        .default_value(std::string{})
        .help("Filter by EtherCAT address (ado) in hex, e.g. 0x0800");
    program.add_argument("--coe-index")
        .default_value(std::string{})
        .help("Filter CoE transactions by OD index in hex, e.g. 0x1018");
    program.add_argument("--coe-sub")
        .default_value(uint64_t{0})
        .scan<'u', uint64_t>()
        .help("Filter CoE transactions by OD subindex (0 = all)");
    program.add_argument("--pdo-addr")
        .default_value(std::string{})
        .help("Filter PDO datagrams by logical address in hex, e.g. 0x00000000");
    program.add_argument("--wkc")
        .default_value(uint64_t{0})
        .scan<'u', uint64_t>()
        .help("Filter by working counter (0 = all)");
    program.add_argument("--errors-only")
        .default_value(false)
        .implicit_value(true)
        .help("Only show errors (wkc=0, SDO abort, etc.)");

    // Existing options
    program.add_argument("-v", "--verbose")
        .default_value(false)
        .implicit_value(true)
        .help("Print full payload hex dumps");
    program.add_argument("--json")
        .default_value(false)
        .implicit_value(true)
        .help("Output compact JSON instead of human-readable text");
    program.add_argument("--max-packets")
        .default_value(uint64_t{0})
        .scan<'u', uint64_t>()
        .help("Stop after this many packets (0 = unlimited)");
    program.add_argument("--max-data")
        .default_value(size_t{64})
        .scan<'u', size_t>()
        .help("Maximum payload bytes to dump per datagram (0 = no limit)");
    program.add_argument("--vlan")
        .default_value(uint64_t{0})
        .scan<'u', uint64_t>()
        .help("Only show frames with the given VLAN ID (0 = all)");
    program.add_argument("--command")
        .default_value(std::string{})
        .help("Only show datagrams with the given command (e.g. LRW, APRD)");
    program.add_argument("--only-ethercat")
        .default_value(false)
        .implicit_value(true)
        .help("Skip non-EtherCAT frames");

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << "\n" << program;
        return 1;
    }

    const std::string input = program.get<std::string>("input");
    const bool verbose = program.get<bool>("--verbose");
    const bool json = program.get<bool>("--json");
    const uint64_t maxPackets = program.get<uint64_t>("--max-packets");
    const size_t maxData = program.get<size_t>("--max-data");
    const uint64_t vlanFilter = program.get<uint64_t>("--vlan");
    const std::string cmdName = program.get<std::string>("--command");
    const bool onlyEtherCAT = program.get<bool>("--only-ethercat");
    const uint64_t slaveFilter = program.get<uint64_t>("--slave");
    const std::string addrStr = program.get<std::string>("--addr");
    const std::string coeIndexStr = program.get<std::string>("--coe-index");
    const uint64_t coeSub = program.get<uint64_t>("--coe-sub");
    const std::string pdoAddrStr = program.get<std::string>("--pdo-addr");
    const uint64_t wkcFilter = program.get<uint64_t>("--wkc");
    const bool errorsOnly = program.get<bool>("--errors-only");

    // Parse selections (comma-separated or multiple --select)
    std::set<Selection> selections;
    std::string selectStr = program.get<std::string>("--select");
    // Also check for multiple --select args
    if (program.is_used("--select")) {
        // argparse may store multiple values; try to get them all
        // Actually argparse stores only the last for non-multi args.
        // We'll parse comma-separated.
    }
    {
        std::stringstream ss(selectStr);
        std::string token;
        while (std::getline(ss, token, ',')) {
            // Trim whitespace
            token.erase(token.begin(), std::find_if(token.begin(), token.end(),
                [](int ch) { return !std::isspace(ch); }));
            token.erase(std::find_if(token.rbegin(), token.rend(),
                [](int ch) { return !std::isspace(ch); }).base(), token.end());
            auto sel = parseSelection(token);
            if (sel) {
                selections.insert(*sel);
            } else {
                std::cerr << "Unknown selection: " << token << "\n";
                std::cerr << "Available: raw, ethercat-transactions, mailbox, "
                          << "coe-transactions, pdo, statistics\n";
                return 1;
            }
        }
    }
    if (selections.empty()) {
        selections.insert(Selection::Raw);
    }

    // Parse filters
    Filters filters;
    filters.verbose = verbose;
    filters.maxPackets = maxPackets;
    filters.maxData = maxData;
    filters.errorsOnly = errorsOnly;
    filters.onlyEtherCAT = onlyEtherCAT;

    if (vlanFilter > 0) filters.vlan = static_cast<uint16_t>(vlanFilter);
    if (slaveFilter > 0) filters.slave = static_cast<uint16_t>(slaveFilter);
    if (wkcFilter > 0) filters.wkc = static_cast<uint16_t>(wkcFilter);
    if (coeSub > 0) filters.coeSub = static_cast<uint8_t>(coeSub);

    if (!addrStr.empty()) {
        filters.addr = static_cast<uint16_t>(std::stoul(addrStr, nullptr, 0));
    }
    if (!coeIndexStr.empty()) {
        filters.coeIndex = static_cast<uint16_t>(std::stoul(coeIndexStr, nullptr, 0));
    }
    if (!pdoAddrStr.empty()) {
        filters.pdoAddr = static_cast<uint32_t>(std::stoul(pdoAddrStr, nullptr, 0));
    }

    if (!cmdName.empty()) {
        filters.command = parseCommand(cmdName);
        if (!filters.command) {
            std::cerr << "Unknown EtherCAT command: " << cmdName << "\n";
            return 1;
        }
    }

    // Open file
    PCP::PCAPNGReader reader;
    if (!reader.open(input)) {
        std::cerr << "Failed to open pcapng file: " << input << "\n";
        return 2;
    }

    // Collect all frames first — this also populates section/interfaces metadata.
    auto allFrames = reader.readAll();
    if (allFrames.empty()) {
        std::cerr << "No frames found in pcapng file\n";
        return 3;
    }

    // Print PCAPNG summary (unless JSON)
    const auto& section = reader.sectionInfo();
    if (!json) {
        std::cout << Utf8Formatter::titledBox("PCAPNG Summary");
        std::cout << Utf8Formatter::bullet << " Byte order swapped: " << (section.byteOrderSwapped ? "yes" : "no") << "\n";
        std::cout << Utf8Formatter::bullet << " Section length: "
                  << (section.sectionLength >= 0 ? std::to_string(section.sectionLength) : "unknown")
                  << "\n";
        if (!section.hardware.empty()) std::cout << Utf8Formatter::bullet << " Hardware: " << section.hardware << "\n";
        if (!section.os.empty()) std::cout << Utf8Formatter::bullet << " OS: " << section.os << "\n";
        if (!section.application.empty()) std::cout << Utf8Formatter::bullet << " Application: " << section.application << "\n";
        if (!section.comment.empty()) std::cout << Utf8Formatter::bullet << " Comment: " << section.comment << "\n";

        const auto& ifaces = reader.interfaces();
        std::cout << Utf8Formatter::bullet << " Interfaces: " << ifaces.size() << "\n";
        for (const auto& iface : ifaces) {
            std::cout << "    iface " << iface.id
                      << ": linkType=" << iface.linkType
                      << " snapLen=" << iface.snapLen;
            if (!iface.name.empty()) std::cout << " name=\"" << iface.name << "\"";
            if (!iface.description.empty()) std::cout << " desc=\"" << iface.description << "\"";
            std::cout << "\n";
        }

        // Count EtherCAT frames (including UDP-encapsulated) for a quick overview.
        uint64_t ecatCount = 0;
        uint64_t ecatUdpCount = 0;
        for (const auto& fr : allFrames) {
            if (fr.isEtherCAT) {
                ++ecatCount;
                if (fr.isEtherCATOverUDP) ++ecatUdpCount;
            }
        }
        std::cout << Utf8Formatter::bullet << " Frames: " << allFrames.size()
                  << "  EtherCAT: " << ecatCount
                  << "  (over-UDP: " << ecatUdpCount << ")"
                  << "\n\n";
    }

    // Set base timestamp for relative time output
    setBaseTimestamp(allFrames.front().timestampNs);

    // JSON mode: output raw JSON for each frame
    if (json) {
        for (const auto& frame : allFrames) {
            if (!framePassesFilters(frame, filters)) continue;
            std::cout << PCP::frameToJson(frame);
        }
        return 0;
    }

    // Display each selected view
    for (Selection sel : selections) {
        switch (sel) {
            case Selection::Raw:
                displayRaw(allFrames, filters);
                break;
            case Selection::EthercatTransactions:
                displayEthercatTransactions(allFrames, filters);
                break;
            case Selection::Mailbox:
                displayMailbox(allFrames, filters);
                break;
            case Selection::CoeTransactions:
                displayCoeTransactions(allFrames, filters);
                break;
            case Selection::Pdo:
                displayPdo(allFrames, filters);
                break;
            case Selection::Statistics:
                displayStatistics(allFrames, filters);
                break;
        }
        std::cout << "\n";
    }

    return 0;
}
