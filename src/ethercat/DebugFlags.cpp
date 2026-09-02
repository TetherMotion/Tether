/**
 * @file DebugFlags.cpp
 * @brief Backing storage for EtherCAT debug flag system.
 */

#include "tether/ethercat/DebugFlags.hpp"
#include "logging/Logger.hpp"

#include <sstream>
#include <cctype>

namespace EtherCAT {

// ============================================================================
// Helper: parse "0,2,5" or "1-3" into a vector of indices
// ============================================================================

static std::vector<uint16_t> parseIndexList(const std::string& str) {
    std::vector<uint16_t> out;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, ',')) {
        // Trim whitespace
        size_t start = 0, end = token.size();
        while (start < end && std::isspace(static_cast<unsigned char>(token[start]))) ++start;
        while (end > start && std::isspace(static_cast<unsigned char>(token[end - 1]))) --end;
        if (start >= end) continue;
        std::string t = token.substr(start, end - start);

        size_t dash = t.find('-');
        if (dash == std::string::npos) {
            try {
                out.push_back(static_cast<uint16_t>(std::stoi(t)));
            } catch (...) {}
        } else {
            try {
                int a = std::stoi(t.substr(0, dash));
                int b = std::stoi(t.substr(dash + 1));
                if (a <= b) {
                    for (int i = a; i <= b; ++i) {
                        out.push_back(static_cast<uint16_t>(i));
                    }
                }
            } catch (...) {}
        }
    }
    return out;
}

// ============================================================================
// EtherCATMasterDebugFlags
// ============================================================================

bool EtherCATMasterDebugFlags::isEnabled(const std::string& name, uint16_t slave_index) const {
    if (name == "rx-pdo")            return rxPDO && rxPDOFilt.allows(slave_index);
    if (name == "tx-pdo")            return txPDO && txPDOFilt.allows(slave_index);
    if (name == "al-state")          return stateMachine && stateMachineFilt.allows(slave_index);
    if (name == "tx-ethercat-packets") return txPackets && txPacketsFilt.allows(slave_index);
    if (name == "rx-ethercat-packets") return rxPackets && rxPacketsFilt.allows(slave_index);
    if (name == "fmmu")              return fmmu && fmmuFilt.allows(slave_index);
    if (name == "sii-eeprom")        return siiEeprom && siiEepromFilt.allows(slave_index);
    if (name == "eeprom")            return eeprom && eepromFilt.allows(slave_index);
    if (name == "coe-reads")         return coeReads && coeReadsFilt.allows(slave_index);
    if (name == "coe-writes")        return coeWrites && coeWritesFilt.allows(slave_index);
    if (name == "coe-rx-packets")    return coeRxPackets && coeRxPacketsFilt.allows(slave_index);
    if (name == "coe-tx-packets")    return coeTxPackets && coeTxPacketsFilt.allows(slave_index);
    if (name == "verify-preop")      return verifyPreOp && verifyPreOpFilt.allows(slave_index);
    if (name == "verify-safeop")     return verifySafeOp && verifySafeOpFilt.allows(slave_index);
    if (name == "pdo-sm")            return pdoSm && pdoSmFilt.allows(slave_index);
    if (name == "dc")                return dc && dcFilt.allows(slave_index);
    if (name == "pdo-configuration") return pdoConfiguration && pdoConfigurationFilt.allows(slave_index);
#ifdef TETHER_ENABLE_FSOE
    if (name == "fsoe")              return fsoe && fsoeFilt.allows(slave_index);
    if (name == "fsoe-frame")        return fsoeFrame && fsoeFrameFilt.allows(slave_index);
    if (name == "fsoe-raw")          return fsoeRaw && fsoeRawFilt.allows(slave_index);
    if (name == "fsoe-wire")         return fsoeWire && fsoeWireFilt.allows(slave_index);
    if (name == "fsoe-sequence")     return fsoeSequence && fsoeSequenceFilt.allows(slave_index);
    if (name == "fsoe-crc")          return fsoeCrc && fsoeCrcFilt.allows(slave_index);
#endif
    return false;
}

void EtherCATMasterDebugFlags::setFlag(const std::string& name, bool enabled) {
    if (name == "rx-pdo")            rxPDO = enabled;
    else if (name == "tx-pdo")       txPDO = enabled;
    else if (name == "al-state")     stateMachine = enabled;
    else if (name == "tx-ethercat-packets") txPackets = enabled;
    else if (name == "rx-ethercat-packets") rxPackets = enabled;
    else if (name == "fmmu")         fmmu = enabled;
    else if (name == "sii-eeprom")   siiEeprom = enabled;
    else if (name == "eeprom")       eeprom = enabled;
    else if (name == "coe-reads")    coeReads = enabled;
    else if (name == "coe-writes")   coeWrites = enabled;
    else if (name == "coe-rx-packets") coeRxPackets = enabled;
    else if (name == "coe-tx-packets") coeTxPackets = enabled;
    else if (name == "verify-preop")   verifyPreOp = enabled;
    else if (name == "verify-safeop")  verifySafeOp = enabled;
    else if (name == "pdo-sm")         pdoSm = enabled;
    else if (name == "dc")             dc = enabled;
    else if (name == "pdo-configuration") pdoConfiguration = enabled;
#ifdef TETHER_ENABLE_FSOE
    else if (name == "fsoe")           fsoe = enabled;
    else if (name == "fsoe-frame")     fsoeFrame = enabled;
    else if (name == "fsoe-raw")       fsoeRaw = enabled;
    else if (name == "fsoe-wire")      fsoeWire = enabled;
    else if (name == "fsoe-sequence")  fsoeSequence = enabled;
    else if (name == "fsoe-crc")       fsoeCrc = enabled;
#endif
}

void EtherCATMasterDebugFlags::setFilter(const std::string& name, const SlaveFilter& filter) {
    if (name == "rx-pdo")            rxPDOFilt = filter;
    else if (name == "tx-pdo")       txPDOFilt = filter;
    else if (name == "al-state")     stateMachineFilt = filter;
    else if (name == "tx-ethercat-packets") txPacketsFilt = filter;
    else if (name == "rx-ethercat-packets") rxPacketsFilt = filter;
    else if (name == "fmmu")         fmmuFilt = filter;
    else if (name == "sii-eeprom")   siiEepromFilt = filter;
    else if (name == "eeprom")       eepromFilt = filter;
    else if (name == "coe-reads")    coeReadsFilt = filter;
    else if (name == "coe-writes")   coeWritesFilt = filter;
    else if (name == "coe-rx-packets") coeRxPacketsFilt = filter;
    else if (name == "coe-tx-packets") coeTxPacketsFilt = filter;
    else if (name == "verify-preop")   verifyPreOpFilt = filter;
    else if (name == "verify-safeop")  verifySafeOpFilt = filter;
    else if (name == "pdo-sm")         pdoSmFilt = filter;
    else if (name == "dc")             dcFilt = filter;
    else if (name == "pdo-configuration") pdoConfigurationFilt = filter;
#ifdef TETHER_ENABLE_FSOE
    else if (name == "fsoe")           fsoeFilt = filter;
    else if (name == "fsoe-frame")     fsoeFrameFilt = filter;
    else if (name == "fsoe-raw")       fsoeRawFilt = filter;
    else if (name == "fsoe-wire")      fsoeWireFilt = filter;
    else if (name == "fsoe-sequence")  fsoeSequenceFilt = filter;
    else if (name == "fsoe-crc")       fsoeCrcFilt = filter;
#endif
}

void EtherCATMasterDebugFlags::applyFromString(const std::string& spec,
                                                uint16_t slave_count,
                                                const char* tag) {
    if (spec.empty()) return;

    std::stringstream ss(spec);
    std::string entry;
    while (std::getline(ss, entry, ',')) {
        // Trim
        size_t start = 0, end = entry.size();
        while (start < end && std::isspace(static_cast<unsigned char>(entry[start]))) ++start;
        while (end > start && std::isspace(static_cast<unsigned char>(entry[end - 1]))) --end;
        if (start >= end) continue;
        std::string e = entry.substr(start, end - start);

        // Find first ':' that separates flagname from filters
        size_t colon = e.find(':');
        std::string flagname = (colon == std::string::npos) ? e : e.substr(0, colon);
        std::string filter_str = (colon == std::string::npos) ? std::string() : e.substr(colon + 1);

        setFlag(flagname, true);

        if (!filter_str.empty()) {
            // Parse parenthesised filter groups: (slaves:0,2,5),(otherfilter:1)
            size_t pos = 0;
            while (pos < filter_str.size()) {
                size_t open = filter_str.find('(', pos);
                if (open == std::string::npos) break;
                size_t close = filter_str.find(')', open);
                if (close == std::string::npos) break;
                std::string group = filter_str.substr(open + 1, close - open - 1);
                pos = close + 1;

                size_t gcolon = group.find(':');
                if (gcolon == std::string::npos) continue;
                std::string ftype = group.substr(0, gcolon);
                std::string fvals = group.substr(gcolon + 1);

                if (ftype == "slaves") {
                    auto indices = parseIndexList(fvals);
                    SlaveFilter f;
                    f.setMask(indices, slave_count);
                    setFilter(flagname, f);
                }
                // Future filter types can be added here
            }
        }

        if (tag) {
            TETHER_LOGI(tag, "Debug flag '%s' enabled%s", flagname.c_str(),
                        filter_str.empty() ? " (all slaves)" : " with filter");
        }
    }
}

void EtherCATMasterDebugFlags::resizeFilters(uint16_t slave_count) {
    rxPDOFilt.resize(slave_count);
    txPDOFilt.resize(slave_count);
    stateMachineFilt.resize(slave_count);
    txPacketsFilt.resize(slave_count);
    rxPacketsFilt.resize(slave_count);
    fmmuFilt.resize(slave_count);
    siiEepromFilt.resize(slave_count);
    eepromFilt.resize(slave_count);
    coeReadsFilt.resize(slave_count);
    coeWritesFilt.resize(slave_count);
    coeRxPacketsFilt.resize(slave_count);
    coeTxPacketsFilt.resize(slave_count);
    verifyPreOpFilt.resize(slave_count);
    verifySafeOpFilt.resize(slave_count);
    pdoSmFilt.resize(slave_count);
    dcFilt.resize(slave_count);
    pdoConfigurationFilt.resize(slave_count);
#ifdef TETHER_ENABLE_FSOE
    fsoeFilt.resize(slave_count);
    fsoeFrameFilt.resize(slave_count);
    fsoeRawFilt.resize(slave_count);
    fsoeWireFilt.resize(slave_count);
    fsoeSequenceFilt.resize(slave_count);
    fsoeCrcFilt.resize(slave_count);
#endif
}

namespace debug {

const std::vector<DebugFlagInfo>& allDebugFlags() {
    static const std::vector<DebugFlagInfo> kFlags = {
        {"sii-derivation",
         "Dump SII mailbox derivation after slave discovery"},
        {"mailbox-configuration",
         "Dump mailbox hardware configuration after slave discovery"},
        {"al-state",
         "Log detailed EtherCAT state-machine transitions"},
        {"tx-ethercat-packets",
         "Log all transmitted EtherCAT packets"},
        {"rx-ethercat-packets",
         "Log all received EtherCAT packets"},
        {"rx-pdo",
         "Log received PDO (master->slave) data"},
        {"tx-pdo",
         "Log transmitted PDO (slave->master) data"},
        {"dc",
         "Log distributed-clock (DC) debug information"},
        {"fmmu",
         "Log FMMU register writes during configuration"},
        {"sii-eeprom",
         "Log SII / EEPROM access details"},
        {"eeprom",
         "Log low-level EEPROM register reads/writes (EEPCTL/EEPSTAT/EEPDAT)"},
        {"coe-reads",
         "Log CoE (SDO) read operations"},
        {"coe-writes",
         "Log CoE (SDO) write operations"},
        {"coe-rx-packets",
         "Log CoE received packets"},
        {"coe-tx-packets",
         "Log CoE transmitted packets"},
        {"verify-preop",
         "Dump detailed SM register verification before PRE_OP transition"},
        {"verify-safeop",
         "Dump detailed SM register verification before SAFE_OP transition"},
        {"pdo-sm",
         "Print detailed configuration of non-mailbox sync managers (SM2/SM3)"},
        {"pdo-configuration",
         "Detailed PDO assignment logging: SM register writes, 0x1C12/0x1C13 "
         "PDO index lists (including which are fixed/skipped), FMMU mapping, "
         "and readback verification"},
#ifdef TETHER_ENABLE_FSOE
        {"fsoe",
         "High-level FSoE protocol trace (state machine decisions)"},
        {"fsoe-frame",
         "Decoded FSoE PDO struct fields (on change)"},
        {"fsoe-raw",
         "FSoE protocol trace + raw frame hex dumps (on change)"},
        {"fsoe-wire",
         "Every-cycle PDO wire dumps (firehose)"},
        {"fsoe-sequence",
         "Per-cycle frame accept/reject + state change summary"},
        {"fsoe-crc",
         "CRC parameters used for TX build and RX check"},
#endif
    };
    return kFlags;
}

} // namespace debug
} // namespace EtherCAT
