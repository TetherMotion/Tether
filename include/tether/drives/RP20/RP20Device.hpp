/**
 * @file RP20Device.hpp
 * @brief Runtime driver class for Kinco RP20 modular EtherCAT I/O couplers
 *
 * Provides slot scanning, CoE init command dispatch, per-module configuration
 * (TC/AI/RD signal form, filter, CJC), PDO buffer registration, PDO-to-SM
 * assignment, and sync manager length update.
 *
 * The static descriptor tables, PDO layouts, register definitions, and init
 * command arrays live in RP20Module.hpp / RP20PDO.hpp / RP20Registers.hpp /
 * RP20ModuleConfig.hpp.  This class wraps the runtime orchestration that
 * every consumer of the RP20 coupler would otherwise need to duplicate.
 */

#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "tether/drives/RP20/RP20Module.hpp"
#include "tether/drives/RP20/RP20ModuleConfig.hpp"
#include "tether/drives/RP20/RP20PDO.hpp"
#include "tether/drives/RP20/RP20Registers.hpp"

namespace EtherCAT { class Master; }

namespace EtherCAT {
namespace Drives {
namespace RP20Module {

// Reg alias is provided by RP20Module.hpp (included above)

// ---------------------------------------------------------------------------
// Runtime module instance — one per discovered slot
// ---------------------------------------------------------------------------

struct ModuleInstance {
    uint16_t slave_index = 0;
    uint8_t  slot = 0;
    const ModuleDescriptor* descriptor = nullptr;
    std::vector<uint8_t> tx_buffer;
    std::vector<uint8_t> rx_buffer;
    int tx_pdo_entry = -1;
    int rx_pdo_entry = -1;
};

// ---------------------------------------------------------------------------
// Per-module configuration options (applied via SDO before SAFE-OP)
// ---------------------------------------------------------------------------

struct ModuleConfig {
    std::optional<::EtherCAT::Drives::Registers::RP20::TCSignalForm> tc_signal_form;
    std::optional<::EtherCAT::Drives::Registers::RP20::FilteringMode> tc_filter;
    std::optional<::EtherCAT::Drives::Registers::RP20::ColdJunctionCompensation> tc_cjc;
    std::optional<::EtherCAT::Drives::Registers::RP20::AISignalForm> ai_signal_form;
    std::optional<::EtherCAT::Drives::Registers::RP20::FilteringMode> ai_filter;
    std::optional<::EtherCAT::Drives::Registers::RP20::RTDSignalForm> rd_signal_form;
    std::optional<::EtherCAT::Drives::Registers::RP20::FilteringMode> rd_filter;
};

// ---------------------------------------------------------------------------
// RP20Device — runtime driver for RP20 modular couplers
// ---------------------------------------------------------------------------

class RP20Device {
public:
    explicit RP20Device(EtherCAT::Master& master);
    ~RP20Device() = default;

    RP20Device(const RP20Device&) = delete;
    RP20Device& operator=(const RP20Device&) = delete;

    // -- Module discovery ----------------------------------------------------

    /// Scan all discovered slaves, slots 0–15, for RP20 modules.
    /// Populates the internal module list with appropriately-sized PDO buffers.
    /// @return true if at least one module was found.
    bool scanModules();

    /// Number of discovered modules.
    size_t moduleCount() const { return modules_.size(); }

    /// Access discovered modules.
    const std::vector<ModuleInstance>& modules() const { return modules_; }
    std::vector<ModuleInstance>& modules() { return modules_; }

    // -- CoE init commands ---------------------------------------------------

    /// Send CoE init commands for a specific module (by index into modules()).
    /// @return true if all commands succeeded.
    bool sendInitCommands(size_t module_index);

    /// Send CoE init commands for all discovered modules.
    /// @return true if all modules' commands succeeded.
    bool sendInitCommandsAll();

    // -- Module configuration ------------------------------------------------

    /// Apply signal form / filter / CJC configuration to a specific module.
    /// @return true if all writes succeeded (or no config to apply).
    bool configureModule(size_t module_index, const ModuleConfig& config);

    /// Apply the same configuration to all discovered modules.
    /// @return true if all modules configured successfully.
    bool configureAllModules(const ModuleConfig& config);

    // -- PDO registration ----------------------------------------------------

    /// Register TxPDO/RxPDO buffers with the master's PDO mapping.
    /// Must be called after scanModules() and before assignPDOs().
    /// @return true if all registrations succeeded.
    bool registerPDOs();

    // -- PDO assignment to sync managers -------------------------------------

    /// Assign RxPDOs to SM2 (0x1C12) and TxPDOs to SM3 (0x1C13) for a slave.
    /// @param slave_index  Slave bus index
    /// @return true if assignment succeeded.
    bool assignPDOs(uint16_t slave_index);

    /// Assign PDOs for all slaves that have RP20 modules.
    /// @return true if all slaves succeeded.
    bool assignPDOsAll();

    // -- Sync manager length update ------------------------------------------

    /// Update SM2/SM3 lengths in slave configs to match total PDO sizes.
    /// @param slave_index  Slave bus index
    void updateSyncManagerLengths(uint16_t slave_index);

    /// Update SM lengths for all slaves that have RP20 modules.
    void updateSyncManagerLengthsAll();

private:
    EtherCAT::Master& master_;
    std::vector<ModuleInstance> modules_;
};

} // namespace RP20Module
} // namespace Drives
} // namespace EtherCAT
