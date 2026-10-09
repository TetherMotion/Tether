/**
 * @file SlavePdoTypes.hpp
 * @brief Slave PDO configuration value types.
 *
 * Split out of Slave.hpp.  Slave re-exports them under their old nested
 * names (Slave::SIIPDOConfig, …).
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "tether/ethercat/Types.hpp"
#include "tether/ethercat/PDOMappingConfig.hpp"
#include "tether/ethercat/CustomPDOMapping.hpp"
#include "tether/ethercat/ObjectDictionary.hpp"

namespace EtherCAT {

struct SIIPDOConfig {
    uint16_t rxpdo_index = 0;   ///< RxPDO object index (e.g. 0x1600)
    uint16_t txpdo_index = 0;   ///< TxPDO object index (e.g. 0x1A00)
    uint16_t rxpdo_size = 0;    ///< Total RxPDO size in bytes
    uint16_t txpdo_size = 0;    ///< Total TxPDO size in bytes
    bool has_rxpdo = false;     ///< True if an RxPDO was found in SII
    bool has_txpdo = false;     ///< True if a TxPDO was found in SII
};

struct MultiPDOAssignment {
    struct SMConfig {
        uint8_t  sm_index;          ///< SM index (0-7)
        uint16_t phys_start_addr;   ///< Physical start address in ESC memory
        uint8_t  control_byte;      ///< Control register byte
        std::vector<PDO::PDOMappingRegion> pdo_mappings;
    };
    std::vector<SMConfig> sm_configs;
};

struct CustomPDOInfo {
    uint16_t pdo_index = 0;
    PDO::PDODirection direction = PDO::PDODirection::TxPDO;
    uint16_t total_size = 0;
    std::vector<CustomPDOFieldLayout> fields;
    int mapping_entry_index = -1;
    /// OD entries synthesized from the slave's own PDO mapping by
    /// registerExistingPDO().  Owns the objects that fields[].entry
    /// points at so they stay valid for the lifetime of this info.
    std::vector<ObjectDictionary::ObjectDictionaryEntry> owned_entries;
};

} // namespace EtherCAT
