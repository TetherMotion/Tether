/**
 * @file FMMUConfiguration.hpp
 * @brief Fieldbus Memory Management Unit (FMMU) Configuration for EtherCAT
 * 
 * @details
 * The FMMU translates logical addresses (used in master PDO frames) to physical
 * addresses (in slave memory where Sync Managers operate). This module provides:
 * 
 * - FMMU register definitions per EtherCAT spec (ETG.1000)
 * - Configuration structures for each FMMU channel (0-7)
 * - Functions to configure FMMUs based on SII FMMU type hints and SM config
 * - Diagnostic and verification utilities
 * 
 * ## FMMU Architecture
 * 
 * ```
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │                        MASTER (Logical Addressing)                        │
 * │  LRW Frame:  [Logical Address 0x00000000] [Data 23B] [Data 25B] ...      │
 * └───────────────────────────┬──────────────────────────────────────────────┘
 *                             │  Logical Address Translation
 *                             ▼
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │                        FMMU Translation Layer                             │
 * │  FMMU0: Log 0x00000000 (23B) ──► Phys 0x1800 (SM2 - RxPDO)              │
 * │  FMMU1: Log 0x00000017 (25B) ──► Phys 0x1C00 (SM3 - TxPDO)              │
 * └──────────────────────────────────────────────────────────────────────────┘
 *                             │  Physical Address
 *                             ▼
 * ┌──────────────────────────────────────────────────────────────────────────┐
 * │                        SLAVE (Physical Memory)                            │
 * │  SM2 @ 0x1800: RxPDO buffer (outputs from master, 23 bytes)              │
 * │  SM3 @ 0x1C00: TxPDO buffer (inputs to master, 25 bytes)                 │
 * └──────────────────────────────────────────────────────────────────────────┘
 * ```
 * 
 * ## FMMU Registers (per FMMU, 16 bytes each)
 * 
 * | Offset | Size | Name             | Description                           |
 * |--------|------|------------------|---------------------------------------|
 * | 0x00   | 4    | LogicalStart     | Starting logical address              |
 * | 0x04   | 2    | Length           | Number of bytes mapped                |
 * | 0x06   | 1    | LogicalStartBit  | Bit offset in first byte (usually 0)  |
 * | 0x07   | 1    | LogicalEndBit    | Bit offset in last byte (usually 7)   |
 * | 0x08   | 2    | PhysicalStart    | Physical address in slave memory      |
 * | 0x0A   | 1    | PhysicalStartBit | Bit offset in physical memory         |
 * | 0x0B   | 1    | Type             | FMMU type (read/write/read-write)     |
 * | 0x0C   | 1    | Activate         | Enable/disable FMMU                   |
 * | 0x0D   | 3    | Reserved         | Must be zero                          |
 * 
 * ## FMMU Types (from SII CAT_FMMU category)
 * 
 * | Value | Name       | Description                                       |
 * |-------|------------|---------------------------------------------------|
 * | 0x00  | Unused     | FMMU not used                                     |
 * | 0x01  | Output     | Master → Slave (RxPDO, typically SM2)            |
 * | 0x02  | Input      | Slave → Master (TxPDO, typically SM3)            |
 * | 0x03  | MboxSync   | Mailbox sync manager status (rare)                |
 * 
 * @see ETG.1000 EtherCAT Specification Part 5 (Application Layer)
 * @see ETG.2010 EtherCAT Slave Information Interface (SII)
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>

#include "tether/platform/EspCompat.hpp"

// Forward declarations
namespace EtherCAT {
class EtherCATMaster;
namespace SII {
struct SIIData;
struct SIIFMMU;
}
namespace PDO {
struct SlaveConfig;
}
}

namespace EtherCAT {
namespace fmmu {

// ============================================================================
// FMMU Constants
// ============================================================================

/// Maximum number of FMMUs per slave (ESC hardware limit)
constexpr size_t kMaxFMMUs = 8;

/// FMMU register base address (each FMMU is 16 bytes)
constexpr uint16_t kFMMURegBase = 0x0600;

/// Size of each FMMU register block
constexpr size_t kFMMURegSize = 16;

// ============================================================================
// FMMU Type Enumeration
// ============================================================================

/**
 * @brief FMMU type classification from SII EEPROM (ETG.2010)
 */
enum class FMMUType : uint8_t {
    Unused      = 0x00,  ///< FMMU not used
    Output      = 0x01,  ///< Master outputs to slave (RxPDO, typically uses SM2)
    Input       = 0x02,  ///< Slave inputs to master (TxPDO, typically uses SM3)
    MboxSync    = 0x03,  ///< Mailbox sync manager status (rarely used)
};

/**
 * @brief Get human-readable name for FMMU type
 */
const char* getFMMUTypeName(FMMUType type);

// ============================================================================
// FMMU Direction (Register Type Field)
// ============================================================================

/**
 * @brief FMMU direction/operation type (register 0x0B)
 * 
 * This is the value written to the FMMU Type register, which controls
 * how the FMMU processes data in LRW frames.
 */
namespace FMMURegType {
    /// FMMU responds to read portion of LRW (slave→master)
    constexpr uint8_t Read          = 0x01;
    /// FMMU responds to write portion of LRW (master→slave)  
    constexpr uint8_t Write         = 0x02;
    /// FMMU responds to both read and write (bidirectional)
    constexpr uint8_t ReadWrite     = 0x03;
}

// ============================================================================
// FMMU Activate Register
// ============================================================================

namespace FMMUActivate {
    /// Enable FMMU operation
    constexpr uint8_t Enable        = 0x01;
    /// Disable FMMU operation
    constexpr uint8_t Disable       = 0x00;
}

// ============================================================================
// FMMU Configuration Structure
// ============================================================================

/**
 * @brief Configuration for a single FMMU channel
 * 
 * This structure holds all parameters needed to configure one FMMU.
 * Use FMMUConfigBuilder for convenient construction.
 */
struct FMMUConfig {
    /// Logical address in master frame (32-bit)
    uint32_t logical_start_addr{0};
    
    /// Number of bytes to map
    uint16_t length{0};
    
    /// Starting bit offset in logical space (usually 0)
    uint8_t logical_start_bit{0};
    
    /// Ending bit offset in logical space (usually 7)
    uint8_t logical_end_bit{7};
    
    /// Physical address in slave memory (maps to SM address)
    uint16_t physical_start_addr{0};
    
    /// Physical starting bit (usually 0)
    uint8_t physical_start_bit{0};
    
    /// FMMU type register value (see FMMURegType)
    uint8_t type{0};
    
    /// Activate register value (0x01 to enable)
    uint8_t activate{FMMUActivate::Disable};
    
    /// Which SM this FMMU is associated with (for reference)
    uint8_t associated_sm{0xFF};
    
    /// SII FMMU type hint (from EEPROM)
    FMMUType sii_type{FMMUType::Unused};
    
    /**
     * @brief Check if this FMMU is configured and enabled
     */
    bool isEnabled() const {
        return (activate & FMMUActivate::Enable) != 0 && length > 0;
    }
    
    /**
     * @brief Check if this FMMU handles outputs (master → slave)
     */
    bool isOutput() const {
        return (type & FMMURegType::Write) != 0;
    }
    
    /**
     * @brief Check if this FMMU handles inputs (slave → master)
     */
    bool isInput() const {
        return (type & FMMURegType::Read) != 0;
    }
    
    /**
     * @brief Create an output FMMU configuration (master → slave)
     * 
     * @param logical_addr Starting logical address
     * @param phys_addr Physical address (typically SM2 address like 0x1800)
     * @param len Number of bytes
     * @param sm_index Associated Sync Manager index
     */
    static FMMUConfig output(uint32_t logical_addr, uint16_t phys_addr, 
                             uint16_t len, uint8_t sm_index = 2) {
        FMMUConfig cfg;
        cfg.logical_start_addr = logical_addr;
        cfg.physical_start_addr = phys_addr;
        cfg.length = len;
        cfg.logical_start_bit = 0;
        cfg.logical_end_bit = 7;
        cfg.physical_start_bit = 0;
        cfg.type = FMMURegType::Write;
        cfg.activate = FMMUActivate::Enable;
        cfg.associated_sm = sm_index;
        cfg.sii_type = FMMUType::Output;
        return cfg;
    }
    
    /**
     * @brief Create an input FMMU configuration (slave → master)
     * 
     * @param logical_addr Starting logical address
     * @param phys_addr Physical address (typically SM3 address like 0x1C00)
     * @param len Number of bytes
     * @param sm_index Associated Sync Manager index
     */
    static FMMUConfig input(uint32_t logical_addr, uint16_t phys_addr,
                            uint16_t len, uint8_t sm_index = 3) {
        FMMUConfig cfg;
        cfg.logical_start_addr = logical_addr;
        cfg.physical_start_addr = phys_addr;
        cfg.length = len;
        cfg.logical_start_bit = 0;
        cfg.logical_end_bit = 7;
        cfg.physical_start_bit = 0;
        cfg.type = FMMURegType::Read;
        cfg.activate = FMMUActivate::Enable;
        cfg.associated_sm = sm_index;
        cfg.sii_type = FMMUType::Input;
        return cfg;
    }
};

// ============================================================================
// FMMU Wire Format (16 bytes per FMMU)
// ============================================================================

/**
 * @brief FMMU register block wire format (16 bytes)
 * 
 * This is the exact layout written to FMMU registers at 0x0600+N*16.
 */
struct __attribute__((packed)) FMMURegBlock {
    uint32_t logical_start_le;       ///< 0x00: Logical start address
    uint16_t length_le;              ///< 0x04: Length in bytes
    uint8_t logical_start_bit;       ///< 0x06: Logical start bit
    uint8_t logical_end_bit;         ///< 0x07: Logical end bit
    uint16_t physical_start_le;      ///< 0x08: Physical start address
    uint8_t physical_start_bit;      ///< 0x0A: Physical start bit
    uint8_t type;                    ///< 0x0B: FMMU type (read/write)
    uint8_t activate;                ///< 0x0C: Activate register
    uint8_t reserved[3];             ///< 0x0D-0x0F: Reserved (must be 0)
};
static_assert(sizeof(FMMURegBlock) == 16, "FMMURegBlock must be 16 bytes");

// ============================================================================
// Slave FMMU Configuration Set
// ============================================================================

/**
 * @brief Complete FMMU configuration for one slave
 */
struct SlaveFMMUConfig {
    /// Slave index
    uint16_t slave_index{0};
    
    /// Configuration for each FMMU channel
    std::array<FMMUConfig, kMaxFMMUs> fmmus{};
    
    /// Number of FMMUs actually configured
    size_t fmmu_count{0};
    
    /// Next available logical address (for auto-allocation)
    uint32_t next_logical_addr{0};
    
    /// Flag indicating if configuration has been written to hardware
    bool configured{false};
    
    /**
     * @brief Add an output FMMU using next available logical address
     * 
     * @param phys_addr Physical address (SM2 address)
     * @param length Bytes to map
     * @param sm_index Associated SM
     * @return true if added successfully
     */
    bool addOutput(uint16_t phys_addr, uint16_t length, uint8_t sm_index = 2) {
        if (fmmu_count >= kMaxFMMUs || length == 0) return false;
        
        fmmus[fmmu_count] = FMMUConfig::output(next_logical_addr, phys_addr, length, sm_index);
        next_logical_addr += length;
        fmmu_count++;
        return true;
    }
    
    /**
     * @brief Add an input FMMU using next available logical address
     */
    bool addInput(uint16_t phys_addr, uint16_t length, uint8_t sm_index = 3) {
        if (fmmu_count >= kMaxFMMUs || length == 0) return false;
        
        fmmus[fmmu_count] = FMMUConfig::input(next_logical_addr, phys_addr, length, sm_index);
        next_logical_addr += length;
        fmmu_count++;
        return true;
    }
    
    /**
     * @brief Get FMMU by SII type
     */
    const FMMUConfig* findByType(FMMUType type) const {
        for (size_t i = 0; i < fmmu_count; i++) {
            if (fmmus[i].sii_type == type) {
                return &fmmus[i];
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Get output FMMU (for RxPDO)
     */
    const FMMUConfig* getOutputFMMU() const {
        return findByType(FMMUType::Output);
    }
    
    /**
     * @brief Get input FMMU (for TxPDO)
     */
    const FMMUConfig* getInputFMMU() const {
        return findByType(FMMUType::Input);
    }
    
    /**
     * @brief Clear all FMMU configuration
     */
    void clear() {
        for (auto& f : fmmus) {
            f = FMMUConfig{};
        }
        fmmu_count = 0;
        next_logical_addr = 0;
        configured = false;
    }
};

// ============================================================================
// Transport Interface for FMMU register I/O
// ============================================================================

/**
 * @brief Abstract transport for reading/writing slave FMMU registers.
 *
 * Concrete implementations delegate to the EtherCAT raw layer (production)
 * or to mock objects (unit tests).
 */
class IFMMUTransport {
public:
    virtual ~IFMMUTransport() = default;

    /// Write data to a slave register via auto-increment addressing.
    virtual bool apwr(uint16_t adp, uint16_t ado,
                      const void* data, uint16_t len,
                      unsigned int timeout_ms) = 0;

    /// Read data from a slave register via auto-increment addressing.
    virtual bool aprd(uint16_t adp, uint16_t ado,
                      void* out, uint16_t len,
                      unsigned int timeout_ms) = 0;

    /// Convert a slave index to the auto-increment ADP value.
    virtual uint16_t adpForSlaveIndex(uint16_t slave_index) = 0;
};

/// Maximum slaves supported for FMMU configuration
constexpr size_t kMaxFMMUSlaves = 8;

// ============================================================================
// FMMUManager — instance-based FMMU configuration (no globals)
// ============================================================================

/**
 * @brief Manages FMMU configuration for up to kMaxFMMUSlaves slaves.
 *
 * All state is instance-owned.  Multiple FMMUManagers can coexist with
 * independent state (e.g. for unit testing).
 *
 * @code
 * MockFMMUTransport transport;
 * FMMUManager mgr(transport);
 * mgr.init();
 * mgr.configureManual(0, 0x1800, 23, 0x1C00, 25, 0);
 * mgr.writeToSlave(0);
 * @endcode
 */
class FMMUManager {
public:
    explicit FMMUManager(IFMMUTransport& transport);

    /// Initialise / reset all FMMU state.
    void init();

    /// @return true after init() has been called.
    bool isInitialized() const { return initialized_; }

    /// Direct access to the slave config array.
    SlaveFMMUConfig* getSlaveConfigs() { return configs_; }

    /// Get FMMU config for a specific slave, or nullptr if out of range.
    SlaveFMMUConfig* getConfig(uint16_t slave_index);

    /// Configure FMMUs from SII data.
    bool configureFromSii(uint16_t slave_index,
                          const SII::SIIData* sii,
                          const PDO::SlaveConfig* sm_config,
                          uint32_t base_logical_addr = 0);

    /// Manually configure FMMUs for a slave.
    bool configureManual(uint16_t slave_index,
                         uint16_t output_phys, uint16_t output_len,
                         uint16_t input_phys, uint16_t input_len,
                         uint32_t base_logical_addr = 0);

    /// Write FMMU registers to slave hardware.
    bool writeToSlave(uint16_t slave_index);

    /// Read FMMU registers from slave hardware.
    size_t readFromSlave(uint16_t slave_index,
                         FMMUConfig* out_configs, size_t max_fmmus);

    /// Verify FMMU registers match expected values.
    bool verify(uint16_t slave_index);

    /// Disable all FMMUs on a slave.
    bool disableAll(uint16_t slave_index);

    /// Get output logical address for a slave.
    uint32_t getOutputLogicalAddr(uint16_t slave_index) const;

    /// Get input logical address for a slave.
    uint32_t getInputLogicalAddr(uint16_t slave_index) const;

    /// Get total logical address space used across all slaves.
    uint32_t getTotalLogicalSize() const { return global_logical_addr_; }

    /// Log FMMU configuration for a slave.
    void logConfig(uint16_t slave_index, const char* tag) const;

    /// Log FMMU hardware state read from a slave.
    void logHardware(uint16_t slave_index, const char* tag);

private:
    IFMMUTransport& transport_;
    SlaveFMMUConfig configs_[kMaxFMMUSlaves];
    uint32_t global_logical_addr_ = 0;
    bool initialized_ = false;
};

// ============================================================================
// Global FMMU Configuration Storage (backward-compat free functions)
// ============================================================================

/**
 * @brief Get pointer to global FMMU configurations array
 */
SlaveFMMUConfig* fmmu_get_slave_configs();

/**
 * @brief Get FMMU configuration for a specific slave
 */
SlaveFMMUConfig* fmmu_get_config(uint16_t slave_index);

// ============================================================================
// FMMU Configuration Functions
// ============================================================================

/**
 * @brief Initialize FMMU module
 * 
 * Clears all FMMU configurations and resets logical address allocator.
 */
void fmmu_init();

/**
 * @brief Configure FMMUs for a slave based on SII data and SM config
 * 
 * This function reads the FMMU type hints from the slave's SII EEPROM
 * (CAT_FMMU category) and combines them with the Sync Manager configuration
 * to create a complete FMMU mapping.
 * 
 * The mapping follows standard conventions:
 * - SII FMMU type 1 (Output) → FMMU for SM2 (RxPDO)
 * - SII FMMU type 2 (Input) → FMMU for SM3 (TxPDO)
 * 
 * @param slave_index Slave index
 * @param sii SII data containing FMMU hints (can be nullptr for defaults)
 * @param sm_config Sync Manager configuration with physical addresses and sizes
 * @param base_logical_addr Starting logical address for this slave
 * @return true if configuration was successful
 */
bool fmmu_configure_from_sii(uint16_t slave_index, 
                              const SII::SIIData* sii,
                              const PDO::SlaveConfig* sm_config,
                              uint32_t base_logical_addr = 0);

/**
 * @brief Manually configure FMMUs for a slave
 * 
 * Use this when SII data is not available or you need custom configuration.
 * 
 * @param slave_index Slave index
 * @param output_phys Physical address for outputs (SM2, typically 0x1800)
 * @param output_len Output data length in bytes
 * @param input_phys Physical address for inputs (SM3, typically 0x1C00)
 * @param input_len Input data length in bytes
 * @param base_logical_addr Starting logical address
 */
bool fmmu_configure_manual(uint16_t slave_index,
                           uint16_t output_phys, uint16_t output_len,
                           uint16_t input_phys, uint16_t input_len,
                           uint32_t base_logical_addr = 0);

/**
 * @brief Write FMMU configuration to slave hardware registers
 * 
 * This function writes the FMMUConfig structures to the actual FMMU
 * registers (0x0600-0x067F) on the slave.
 * 
 * @param src_mac Source MAC address
 * @param slave_index Slave index
 * @return true if all FMMU registers were written successfully
 */
bool fmmu_write_to_slave(const uint8_t src_mac[6], uint16_t slave_index);

/**
 * @brief Read FMMU configuration from slave hardware registers
 * 
 * Reads the current FMMU register values from the slave for verification.
 * 
 * @param src_mac Source MAC address
 * @param slave_index Slave index
 * @param out_configs Output array for FMMU configurations
 * @param max_fmmus Maximum FMMUs to read
 * @return Number of FMMUs read, or 0 on failure
 */
size_t fmmu_read_from_slave(const uint8_t src_mac[6], uint16_t slave_index,
                            FMMUConfig* out_configs, size_t max_fmmus);

/**
 * @brief Verify FMMU configuration matches expected values
 * 
 * Reads back FMMU registers and compares with configured values.
 * 
 * @param src_mac Source MAC address  
 * @param slave_index Slave index
 * @return true if all configured FMMUs match hardware
 */
bool fmmu_verify(const uint8_t src_mac[6], uint16_t slave_index);

/**
 * @brief Disable all FMMUs on a slave
 * 
 * Writes 0 to all FMMU activate registers.
 */
bool fmmu_disable_all(const uint8_t src_mac[6], uint16_t slave_index);

// ============================================================================
// Diagnostic Functions
// ============================================================================

/**
 * @brief Log FMMU configuration details
 * 
 * Outputs detailed FMMU configuration to ESP_LOG.
 */
void fmmu_log_config(uint16_t slave_index, const char* tag);

/**
 * @brief Log FMMU hardware state read from slave
 */
void fmmu_log_hardware(const uint8_t src_mac[6], uint16_t slave_index, const char* tag);

/**
 * @brief Get logical address for a slave's outputs
 */
uint32_t fmmu_get_output_logical_addr(uint16_t slave_index);

/**
 * @brief Get logical address for a slave's inputs
 */
uint32_t fmmu_get_input_logical_addr(uint16_t slave_index);

/**
 * @brief Get total logical address space used
 */
uint32_t fmmu_get_total_logical_size();

/**
 * @brief Set the EtherCATMaster pointer for the FMMU transport.
 *
 * Must be called before any fmmu_write_to_slave / fmmu_read_from_slave /
 * fmmu_verify / fmmu_disable_all calls when using the master-backed transport.
 */
void fmmu_set_master(EtherCAT::EtherCATMaster* master);

} // namespace fmmu
} // namespace EtherCAT
