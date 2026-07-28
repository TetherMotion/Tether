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
#include <string>
#include <string_view>
#include <vector>

#include "tether/drives/RP20/RP20Module.hpp"
#include "tether/drives/RP20/RP20ModuleConfig.hpp"
#include "tether/drives/RP20/RP20PDO.hpp"
#include "tether/drives/RP20/RP20Registers.hpp"

namespace EtherCAT { class Master; class Slave; enum class SlaveError : uint8_t; }

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

    /// Human-readable summary, e.g. "Slave 0 Slot 2: TC_4 (Thermocouple, ident=0x12)"
    std::string toString() const;
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

    /// Format a human-readable label for the given module type, e.g. "[Type_K, None filter, Internal CJC]"
    std::string formatLabel(ModuleType type) const;
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

    // -- Full state-transition orchestration (P1) ----------------------------

    /// Configure PDOs, assign to SMs, finalize mapping, and transition all
    /// RP20-bearing slaves to SAFE-OP.  Calls sendInitCommandsAll(),
    /// configureAllModules(), registerPDOs(), then per-slave: assignPDOs(),
    /// configureProcessDataSyncManagersFromSii(), updateSyncManagerLengths(),
    /// finalizeMapping(), assumePDOAlreadyConfigured(), transitionToSafeOp().
    /// @param config  Module configuration to apply (use empty config for defaults)
    /// @return true if all slaves reached SAFE-OP successfully.
    bool bringToSafeOp(const ModuleConfig& config = {});

    /// Transition all RP20-bearing slaves from SAFE-OP to OP.
    /// @return true if all slaves reached OP successfully.
    bool bringToOp();

    // -- Typed I/O channel accessors (P2) ------------------------------------

    /// Read digital inputs from a DI module (DI_16, Multi_DIO_8).
    /// @param module_index  Index into modules()
    /// @param field_idx     PDO field index (channel group)
    /// @return Raw 8-bit value, or 0 if invalid.
    uint8_t readDigitalInputs(size_t module_index, size_t field_idx) const;

    /// Read thermocouple temperature from a TC module.
    /// @return Temperature in °C (raw value / 10.0), or 0 if invalid.
    float readTemperature(size_t module_index, size_t field_idx) const;

    /// Read analog input from an AI/RD/Mixed_AIO module.
    /// @return Raw 16-bit signed value, or 0 if invalid.
    int16_t readAnalogInput(size_t module_index, size_t field_idx) const;

    /// Write digital outputs to a DO/DR/Multi_DIO module.
    /// @param module_index  Index into modules()
    /// @param field_idx     PDO field index (channel group)
    /// @param pattern       8-bit output pattern
    void writeDigitalOutputs(size_t module_index, size_t field_idx, uint8_t pattern);

    /// Write analog output to an AO/Mixed_AIO module.
    /// @param module_index  Index into modules()
    /// @param field_idx     PDO field index (channel)
    /// @param value         16-bit signed value
    void writeAnalogOutput(size_t module_index, size_t field_idx, int16_t value);

    /// Write a single output bit to a digital output module.
    /// @param module_index  Index into modules()
    /// @param field_idx     PDO field index (channel group)
    /// @param bit           Bit position within the field
    /// @param value         true = ON, false = OFF
    void writeOutputBit(size_t module_index, size_t field_idx, uint8_t bit, bool value);

    // -- String-to-enum parsing with vendor aliases (P4) ---------------------

    static std::optional<::EtherCAT::Drives::Registers::RP20::TCSignalForm>
    parseTCSignalForm(std::string_view s);

    static std::optional<::EtherCAT::Drives::Registers::RP20::AISignalForm>
    parseAISignalForm(std::string_view s);

    static std::optional<::EtherCAT::Drives::Registers::RP20::RTDSignalForm>
    parseRTDSignalForm(std::string_view s);

    static std::optional<::EtherCAT::Drives::Registers::RP20::FilteringMode>
    parseFilteringMode(std::string_view s);

    static std::optional<::EtherCAT::Drives::Registers::RP20::ColdJunctionCompensation>
    parseColdJunctionCompensation(std::string_view s);

private:
    EtherCAT::Master& master_;
    std::vector<ModuleInstance> modules_;
};

} // namespace RP20Module
} // namespace Drives
} // namespace EtherCAT
