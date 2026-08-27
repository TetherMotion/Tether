/**
 * @file DebugFlags.hpp
 * @brief Per-master debug flags with per-slave filtering.
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tether/ethercat/DebugGate.hpp"

namespace EtherCAT {

/**
 * @brief Slave index filter for a debug flag.
 *
 * When pass_all is true (default), the flag applies to all slaves and
 * the mask is ignored.  Otherwise the flag is enabled only for slave
 * indices that are present in the mask.
 */
struct SlaveFilter {
    bool pass_all = true;             ///< If true, applies to all slaves.
    std::vector<bool> slave_mask;     ///< Per-slave enable bits.
    std::vector<uint16_t> filter_indices; ///< Raw indices parsed from CLI (for late resize).

    /**
     * @brief Query whether a slave passes this filter.
     * @param slave_index  Slave index to check.
     * @return true if the slave passes (or if pass_all is set).
     */
    bool allows(uint16_t slave_index) const {
        if (pass_all) return true;
        if (slave_index >= slave_mask.size()) return false;
        return slave_mask[slave_index];
    }

    /**
     * @brief Set the mask from a list of indices.
     * @param indices  Slave indices to enable.
     * @param count    Total number of slaves (sizes the mask).
     */
    void setMask(const std::vector<uint16_t>& indices, uint16_t count) {
        pass_all = false;
        filter_indices = indices;
        slave_mask.assign(count, false);
        for (uint16_t idx : indices) {
            if (idx < count) slave_mask[idx] = true;
        }
    }

    /**
     * @brief Resize the mask to a new slave count, preserving indices.
     * @param count  New total number of slaves.
     */
    void resize(uint16_t count) {
        if (pass_all) return;
        slave_mask.assign(count, false);
        for (uint16_t idx : filter_indices) {
            if (idx < count) slave_mask[idx] = true;
        }
    }
};

/**
 * @brief Pre-computed per-slave debug flags.
 *
 * This lightweight struct is distributed by the Master to each Slave
 * and CoEManager instance so they can check debug state without
 * querying the master on every log statement.
 */
struct EtherCATSlaveDebugFlags {
    bool rxPDO = false;
    bool txPDO = false;
    bool stateMachine = false;
    bool txPackets = false;
    bool rxPackets = false;
    bool fmmu = false;
    bool siiEeprom = false;
    bool eeprom = false;
    bool coeReads = false;
    bool coeWrites = false;
    bool coeRxPackets = false;
    bool coeTxPackets = false;
    bool verifyPreOp = false;
    bool verifySafeOp = false;
    bool pdoSm = false;
    bool dc = false;
};

/**
 * @brief Per-master debug flags with slave filtering.
 *
 * Each flag has an "enabled" bool and a SlaveFilter.  The master
 * pre-computes an EtherCATSlaveDebugFlags for each slave and pushes
 * it to the Slave and CoEManager objects.
 */
class EtherCATMasterDebugFlags {
public:
    bool rxPDO = false;
    bool txPDO = false;
    bool stateMachine = false;
    bool txPackets = false;
    bool rxPackets = false;
    bool fmmu = false;
    bool siiEeprom = false;
    bool eeprom = false;
    bool coeReads = false;
    bool coeWrites = false;
    bool coeRxPackets = false;
    bool coeTxPackets = false;
    bool verifyPreOp = false;
    bool verifySafeOp = false;
    bool pdoSm = false;
    bool dc = false;

    SlaveFilter rxPDOFilt;
    SlaveFilter txPDOFilt;
    SlaveFilter stateMachineFilt;
    SlaveFilter txPacketsFilt;
    SlaveFilter rxPacketsFilt;
    SlaveFilter fmmuFilt;
    SlaveFilter siiEepromFilt;
    SlaveFilter eepromFilt;
    SlaveFilter coeReadsFilt;
    SlaveFilter coeWritesFilt;
    SlaveFilter coeRxPacketsFilt;
    SlaveFilter coeTxPacketsFilt;
    SlaveFilter verifyPreOpFilt;
    SlaveFilter verifySafeOpFilt;
    SlaveFilter pdoSmFilt;
    SlaveFilter dcFilt;

    // Gate for conditional debugging (nullptr = always active, current behavior)
    DebugGate* gate_ = nullptr;

    void setGate(DebugGate* gate) { gate_ = gate; }

    /**
     * @brief Check if any debug flag is enabled at all (ignoring gate).
     */
    bool isAnyFlagEnabled() const {
        return rxPDO || txPDO || stateMachine || txPackets || rxPackets ||
               fmmu || siiEeprom || eeprom || coeReads || coeWrites || coeRxPackets ||
               coeTxPackets || verifyPreOp || verifySafeOp || pdoSm || dc;
    }

    /**
     * @brief Compute the pre-computed flags for a single slave.
     */
    EtherCATSlaveDebugFlags computeForSlave(uint16_t slave_index) const {
        // If a gate is set and it's closed for this slave, return all-false
        if (gate_ && !gate_->isActiveForSlave(slave_index)) {
            return EtherCATSlaveDebugFlags{};
        }

        EtherCATSlaveDebugFlags s;
        s.rxPDO        = rxPDO && rxPDOFilt.allows(slave_index);
        s.txPDO        = txPDO && txPDOFilt.allows(slave_index);
        s.stateMachine = stateMachine && stateMachineFilt.allows(slave_index);
        s.txPackets    = txPackets && txPacketsFilt.allows(slave_index);
        s.rxPackets    = rxPackets && rxPacketsFilt.allows(slave_index);
        s.fmmu         = fmmu && fmmuFilt.allows(slave_index);
        s.siiEeprom    = siiEeprom && siiEepromFilt.allows(slave_index);
        s.eeprom       = eeprom && eepromFilt.allows(slave_index);
        s.coeReads     = coeReads && coeReadsFilt.allows(slave_index);
        s.coeWrites    = coeWrites && coeWritesFilt.allows(slave_index);
        s.coeRxPackets = coeRxPackets && coeRxPacketsFilt.allows(slave_index);
        s.coeTxPackets = coeTxPackets && coeTxPacketsFilt.allows(slave_index);
        s.verifyPreOp   = verifyPreOp && verifyPreOpFilt.allows(slave_index);
        s.verifySafeOp  = verifySafeOp && verifySafeOpFilt.allows(slave_index);
        s.pdoSm         = pdoSm && pdoSmFilt.allows(slave_index);
        s.dc            = dc && dcFilt.allows(slave_index);
        return s;
    }

    /**
     * @brief Convenience: is a named flag enabled for a given slave?
     */
    bool isEnabled(const std::string& name, uint16_t slave_index) const;

    /**
     * @brief Enable/disable a flag by name (no filter).
     */
    void setFlag(const std::string& name, bool enabled);

    /**
     * @brief Set the filter for a named flag.
     */
    void setFilter(const std::string& name, const SlaveFilter& filter);

    /**
     * @brief Set all flags and filters from a string specification.
     *
     * Format: comma-separated entries, each entry is `flagname` or
     * `flagname:(filtertype:values),...`.
     *
     * Examples:
     *   --debug rx-pdo,stateMachine:(slaves:0,2,5),coe-reads:(slaves:1-3)
     */
    void applyFromString(const std::string& spec, uint16_t slave_count, const char* tag = nullptr);

    /**
     * @brief Set filters for all flags from the current slave count.
     */
    void resizeFilters(uint16_t slave_count);
};

namespace debug {

/**
 * @brief Metadata for a single debug flag.
 */
struct DebugFlagInfo {
    std::string name;           ///< Flag name used on the command line.
    std::string description;    ///< Human-readable description.
};

/**
 * @brief Registry of all available debug flags.
 */
const std::vector<DebugFlagInfo>& allDebugFlags();

} // namespace debug
} // namespace EtherCAT
