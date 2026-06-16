/**
 * @file PcapLogger.hpp
 * @brief Shared PcapNG logging interface for EtherCAT Master and Slave
 * 
 * Provides a common interface for packet capture logging that can be
 * shared between master and slave implementations to avoid code duplication.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <atomic>
#include <fstream>
#include <mutex>

namespace EtherCAT {

// ============================================================================
// Forward Declarations
// ============================================================================

class IPcapLogger;

// ============================================================================
// Packet Direction
// ============================================================================

enum class PacketDirection : uint8_t {
    Unknown = 0,
    Inbound = 1,     // Received packet
    Outbound = 2,    // Transmitted packet
    Loopback = 3     // Internal loopback
};

// ============================================================================
// Packet Metadata
// ============================================================================

struct PacketMetadata {
    uint64_t timestampNs{0};           // Nanosecond timestamp
    uint32_t capturedLength{0};        // Captured length
    uint32_t originalLength{0};        // Original length
    PacketDirection direction{PacketDirection::Unknown};
    uint8_t interfaceId{0};            // Interface index
    
    // Optional fields
    bool hasHash{false};
    uint32_t packetHash{0};
    
    bool hasFlags{false};
    uint32_t packetFlags{0};
    
    bool hasDropCount{false};
    uint64_t dropCount{0};
    
    // EtherCAT-specific
    uint16_t slaveAddress{0};          // For slave-specific logging
    bool isProcessData{false};         // PDO vs mailbox
    uint8_t workingCounter{0};
};

// ============================================================================
// Interface Information
// ============================================================================

struct InterfaceInfo {
    std::string name;                  // Interface name
    std::string description;           // Interface description
    uint32_t linkType{1};              // LINKTYPE_ETHERNET = 1
    uint32_t snapLength{65535};        // Max capture length
    uint64_t timestampResolution{9};   // 10^-9 = nanoseconds
    
    // Optional hardware info
    std::array<uint8_t, 6> macAddress{};
    uint32_t speed{0};                 // Speed in bits/sec
    
    // Optional filter
    std::string filter;
    
    // EtherCAT info
    bool isEtherCAT{true};
    std::string role;                  // "Master" or "Slave"
};

// ============================================================================
// Section Header Info
// ============================================================================

struct SectionHeaderInfo {
    std::string hardware;              // Hardware description
    std::string os;                    // OS description
    std::string application;           // Application name
    std::string comment;               // Optional comment
};

// ============================================================================
// PcapNG Block Types
// ============================================================================

namespace PcapNg {
    constexpr uint32_t BLOCK_TYPE_SHB = 0x0A0D0D0A;  // Section Header Block
    constexpr uint32_t BLOCK_TYPE_IDB = 0x00000001;  // Interface Description Block
    constexpr uint32_t BLOCK_TYPE_EPB = 0x00000006;  // Enhanced Packet Block
    constexpr uint32_t BLOCK_TYPE_SPB = 0x00000003;  // Simple Packet Block
    constexpr uint32_t BLOCK_TYPE_NRB = 0x00000004;  // Name Resolution Block
    constexpr uint32_t BLOCK_TYPE_ISB = 0x00000005;  // Interface Statistics Block
    constexpr uint32_t BLOCK_TYPE_DSB = 0x0000000A;  // Decryption Secrets Block
    constexpr uint32_t BLOCK_TYPE_CB1 = 0x00000BAD;  // Custom Block (copy allowed)
    constexpr uint32_t BLOCK_TYPE_CB2 = 0x40000BAD;  // Custom Block (no copy)
    
    constexpr uint32_t BYTE_ORDER_MAGIC = 0x1A2B3C4D;
    constexpr uint16_t VERSION_MAJOR = 1;
    constexpr uint16_t VERSION_MINOR = 0;
    
    // Option codes
    constexpr uint16_t OPT_ENDOFOPT = 0;
    constexpr uint16_t OPT_COMMENT = 1;
    
    // SHB options
    constexpr uint16_t SHB_HARDWARE = 2;
    constexpr uint16_t SHB_OS = 3;
    constexpr uint16_t SHB_USERAPPL = 4;
    
    // IDB options
    constexpr uint16_t IDB_NAME = 2;
    constexpr uint16_t IDB_DESCRIPTION = 3;
    constexpr uint16_t IDB_IPV4ADDR = 4;
    constexpr uint16_t IDB_IPV6ADDR = 5;
    constexpr uint16_t IDB_MACADDR = 6;
    constexpr uint16_t IDB_EUIADDR = 7;
    constexpr uint16_t IDB_SPEED = 8;
    constexpr uint16_t IDB_TSRESOL = 9;
    constexpr uint16_t IDB_TZONE = 10;
    constexpr uint16_t IDB_FILTER = 11;
    constexpr uint16_t IDB_OS = 12;
    constexpr uint16_t IDB_FCSLEN = 13;
    constexpr uint16_t IDB_TSOFFSET = 14;
    
    // EPB options
    constexpr uint16_t EPB_FLAGS = 2;
    constexpr uint16_t EPB_HASH = 3;
    constexpr uint16_t EPB_DROPCOUNT = 4;
    constexpr uint16_t EPB_PACKETID = 5;
    constexpr uint16_t EPB_QUEUE = 6;
    constexpr uint16_t EPB_VERDICT = 7;
    
    // Custom options (for EtherCAT-specific data)
    constexpr uint16_t EPB_ETHERCAT_SLAVE = 0x8001;
    constexpr uint16_t EPB_ETHERCAT_WC = 0x8002;
    constexpr uint16_t EPB_ETHERCAT_PDO = 0x8003;
}

// ============================================================================
// IPcapLogger Interface
// ============================================================================

/**
 * Interface for PcapNG packet logging
 * 
 * Can be implemented by both master and slave to share logging code.
 */
class IPcapLogger {
public:
    virtual ~IPcapLogger() = default;
    
    // File management
    virtual bool open(const std::string& filename) = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual bool flush() = 0;
    
    // Section/Interface setup
    virtual void setSectionHeader(const SectionHeaderInfo& info) = 0;
    virtual uint8_t addInterface(const InterfaceInfo& info) = 0;
    
    // Packet logging
    virtual bool logPacket(const uint8_t* data, size_t length, 
                          const PacketMetadata& metadata) = 0;
    
    // Convenience overload
    virtual bool logPacket(const uint8_t* data, size_t length,
                          PacketDirection direction = PacketDirection::Unknown,
                          uint8_t interfaceId = 0) = 0;
    
    // Statistics
    virtual uint64_t getPacketCount() const = 0;
    virtual uint64_t getByteCount() const = 0;
    virtual uint64_t getDropCount() const = 0;
    
    // Control
    virtual void setEnabled(bool enabled) = 0;
    virtual bool isEnabled() const = 0;
    
    // Buffer management
    virtual void setBufferSize(size_t bytes) = 0;
    virtual size_t getBufferUsage() const = 0;
};

// ============================================================================
// PcapNgLogger Implementation
// ============================================================================

class PcapNgLogger : public IPcapLogger {
public:
    PcapNgLogger();
    ~PcapNgLogger() override;
    
    // File management
    bool open(const std::string& filename) override;
    void close() override;
    bool isOpen() const override;
    bool flush() override;
    
    // Section/Interface setup
    void setSectionHeader(const SectionHeaderInfo& info) override;
    uint8_t addInterface(const InterfaceInfo& info) override;
    
    // Packet logging
    bool logPacket(const uint8_t* data, size_t length,
                  const PacketMetadata& metadata) override;
    bool logPacket(const uint8_t* data, size_t length,
                  PacketDirection direction = PacketDirection::Unknown,
                  uint8_t interfaceId = 0) override;
    
    // Statistics
    uint64_t getPacketCount() const override { return packetCount_.load(); }
    uint64_t getByteCount() const override { return byteCount_.load(); }
    uint64_t getDropCount() const override { return dropCount_.load(); }
    
    // Control
    void setEnabled(bool enabled) override { enabled_ = enabled; }
    bool isEnabled() const override { return enabled_; }
    
    // Buffer management
    void setBufferSize(size_t bytes) override;
    size_t getBufferUsage() const override;
    
    // Additional features
    void setCompressionEnabled(bool enabled) { compression_ = enabled; }
    void setMaxFileSize(uint64_t bytes) { maxFileSize_ = bytes; }
    void setRotationEnabled(bool enabled, uint32_t maxFiles = 10) {
        rotation_ = enabled;
        maxRotationFiles_ = maxFiles;
    }
    
private:
    void writeSectionHeaderBlock();
    void writeInterfaceDescriptionBlock(const InterfaceInfo& info);
    void writeEnhancedPacketBlock(const uint8_t* data, size_t length,
                                  const PacketMetadata& metadata);
    
    void writeBlock(uint32_t blockType, const std::vector<uint8_t>& content);
    void writeOption(std::vector<uint8_t>& options, uint16_t code, 
                    const void* data, uint16_t length);
    void writeOptionString(std::vector<uint8_t>& options, uint16_t code,
                          const std::string& str);
    void writeEndOfOptions(std::vector<uint8_t>& options);
    
    static uint32_t padTo32(uint32_t length);
    uint64_t getCurrentTimestampNs() const;
    
    void rotateFile();
    std::string getRotatedFilename(uint32_t index) const;
    
private:
    std::ofstream file_;
    std::string filename_;
    mutable std::mutex mutex_;
    
    SectionHeaderInfo sectionInfo_;
    std::vector<InterfaceInfo> interfaces_;
    
    std::atomic<bool> enabled_{true};
    std::atomic<uint64_t> packetCount_{0};
    std::atomic<uint64_t> byteCount_{0};
    std::atomic<uint64_t> dropCount_{0};
    
    // Buffer for batched writes
    std::vector<uint8_t> writeBuffer_;
    size_t bufferSize_{1024 * 1024};  // 1MB default
    
    // Compression
    bool compression_{false};
    
    // File rotation
    bool rotation_{false};
    uint32_t maxRotationFiles_{10};
    uint64_t maxFileSize_{100 * 1024 * 1024};  // 100MB default
    uint32_t currentRotationIndex_{0};
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * Create a standard PcapNG logger
 */
std::unique_ptr<IPcapLogger> createPcapLogger();

/**
 * Create a PcapNG logger with specified configuration
 */
std::unique_ptr<IPcapLogger> createPcapLogger(const std::string& filename,
                                              const SectionHeaderInfo& sectionInfo);

/**
 * Create a null logger (discards all packets)
 */
std::unique_ptr<IPcapLogger> createNullPcapLogger();

/**
 * Create a memory-based logger (stores in memory, can be retrieved)
 */
std::unique_ptr<IPcapLogger> createMemoryPcapLogger(size_t maxBytes = 10 * 1024 * 1024);

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Get current nanosecond timestamp
 */
uint64_t getPcapTimestampNs();

/**
 * Convert EtherCAT frame to metadata
 */
PacketMetadata extractEtherCATMetadata(const uint8_t* frame, size_t length);

}  // namespace EtherCAT
