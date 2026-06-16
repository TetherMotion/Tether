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
class Master;
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

    /// Write data to a slave register (ADP is implicit — slave knows its own).
    virtual bool apwr(uint16_t ado, const void* data,
                      uint16_t len, unsigned int timeout_ms) = 0;

    /// Read data from a slave register (ADP is implicit).
    virtual bool aprd(uint16_t ado, void* out,
                      uint16_t len, unsigned int timeout_ms) = 0;
};

// ============================================================================
// FMMUManager — instance-based FMMU configuration (no globals)
// ============================================================================

/**
 * @brief Per-slave FMMU configuration manager.
 *
 * Each Slave owns one FMMUManager instance.  All state is local to
 * that slave; there is no global array.
 *
 * @code
 * MockFMMUTransport transport;
 * FMMUManager mgr(transport);
 * mgr.configureManual(0x1800, 23, 0x1C00, 25, 0);
 * mgr.writeToSlave();
 * @endcode
 */
class FMMUManager {
public:
    explicit FMMUManager(IFMMUTransport& transport);

    /// Direct access to this slave's config.
    SlaveFMMUConfig& config() { return config_; }
    const SlaveFMMUConfig& config() const { return config_; }

    /// Configure FMMUs from SII data.
    bool configureFromSii(const SII::SIIData* sii,
                          const PDO::SlaveConfig* sm_config,
                          uint32_t base_logical_addr = 0);

    /// Manually configure FMMUs.
    bool configureManual(uint16_t output_phys, uint16_t output_len,
                         uint16_t input_phys, uint16_t input_len,
                         uint32_t base_logical_addr = 0);

    /// Write FMMU registers to slave hardware.
    bool writeToSlave();

    /// Read FMMU registers from slave hardware.
    size_t readFromSlave(FMMUConfig* out_configs, size_t max_fmmus);

    /// Verify FMMU registers match expected values.
    bool verify();

    /// Read FMMU registers from slave hardware and verify they match expected config.
    /// Returns true if the slave has no enabled FMMUs, or if all enabled FMMUs
    /// match the expected configuration.  Always reads from the slave — never
    /// relies on a cached "configured" flag.
    bool verifyFromSlave();

    /// Disable all FMMUs on this slave.
    bool disableAll();

    /// Get output logical address.
    uint32_t getOutputLogicalAddr() const;

    /// Get input logical address.
    uint32_t getInputLogicalAddr() const;

    /// Get total logical address space used by this slave.
    uint32_t getTotalLogicalSize() const { return config_.next_logical_addr; }

    /// Log FMMU configuration.
    void logConfig(const char* tag) const;

    /// Log FMMU hardware state read from slave.
    void logHardware(const char* tag);

private:
    IFMMUTransport& transport_;
    SlaveFMMUConfig config_;
};

} // namespace fmmu
} // namespace EtherCAT
