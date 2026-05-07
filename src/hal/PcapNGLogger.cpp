/**
 * @file PcapNGLogger.cpp
 * @brief PcapNG frame logger implementation
 *
 * Implements IPcapLogger interface for logging Ethernet frames to PcapNG format.
 */

#include "hal/IPcapLogger.hpp"
#include "hal/HALTypes.hpp"
#include "hal/IEthernet.hpp"
#include "hal/IClock.hpp"

#include <cstring>
#include <cstdio>
#include <algorithm>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// PcapNG Constants
// ============================================================================

// PcapNG block types
static constexpr uint32_t PCAPNG_SHB = 0x0A0D0D0A;  // Section Header Block
static constexpr uint32_t PCAPNG_IDB = 0x00000001;  // Interface Description Block
static constexpr uint32_t PCAPNG_EPB = 0x00000006;  // Enhanced Packet Block

// Link type for Ethernet
static constexpr uint16_t LINKTYPE_ETHERNET = 1;

// ============================================================================
// PcapNGLoggerImpl Implementation
// ============================================================================

class PcapNGLoggerImpl : public IPcapLogger {
public:
    PcapNGLoggerImpl()
        : m_file(nullptr)
        , m_frameCount(0)
        , m_fileSize(0)
        , m_logTx(true)
        , m_logRx(true)
    {}

    ~PcapNGLoggerImpl() override {
        close();
    }

    Error init(const PcapLoggerConfig& config) override {
        if (m_file) close();
        
        m_config = config;
        m_logTx = config.logTx;
        m_logRx = config.logRx;
        
        const char* mode = config.appendMode ? "ab" : "wb";
        m_file = std::fopen(config.filename.c_str(), mode);
        if (!m_file) {
            return Error::PermissionDenied;
        }
        
        m_frameCount = 0;
        m_fileSize = 0;
        
        // Write Section Header Block
        if (!writeSHB()) {
            close();
            return Error::InternalError;
        }
        
        // Write Interface Description Block
        if (!writeIDB()) {
            close();
            return Error::InternalError;
        }
        
        return Error::OK;
    }

    void close() override {
        if (m_file) {
            std::fclose(m_file);
            m_file = nullptr;
        }
    }

    bool isOpen() const override {
        return m_file != nullptr;
    }

    Error logFrame(const uint8_t* frame, size_t length,
                   FrameDirection direction, Timestamp timestamp) override {
        if (!m_file) return Error::NotInitialized;
        
        // Filter by direction
        if (direction == FrameDirection::Tx && !m_logTx) return Error::OK;
        if (direction == FrameDirection::Rx && !m_logRx) return Error::OK;
        
        // Check file size limit
        if (m_config.maxFileSize > 0 && m_fileSize >= m_config.maxFileSize) {
            if (m_config.rotateFiles) {
                // TODO: Implement file rotation
            }
            return Error::NoMemory;
        }
        
        // Write Enhanced Packet Block
        if (!writeEPB(frame, length, direction, timestamp)) {
            return Error::InternalError;
        }
        
        if (direction == FrameDirection::Tx) {
            m_stats.txFrames++;
        } else {
            m_stats.rxFrames++;
        }
        m_stats.totalBytes += length;
        m_frameCount++;
        
        return Error::OK;
    }

    Error logFrameWithInfo(const uint8_t* frame, size_t length,
                           FrameDirection direction,
                           const RxFrameInfo& info) override {
        return logFrame(frame, length, direction, info.timestamp);
    }

    void flush() override {
        if (m_file) std::fflush(m_file);
    }

    uint64_t getFrameCount() const override {
        return m_frameCount;
    }

    size_t getFileSize() const override {
        return m_fileSize;
    }

    Stats getStats() const override {
        return m_stats;
    }

private:
    PcapLoggerConfig m_config;
    std::FILE* m_file;
    uint64_t m_frameCount;
    size_t m_fileSize;
    bool m_logTx;
    bool m_logRx;
    Stats m_stats;

    // Write bytes in little-endian
    void writeLE16(uint16_t val) {
        uint8_t bytes[2] = { static_cast<uint8_t>(val), static_cast<uint8_t>(val >> 8) };
        std::fwrite(bytes, 1, 2, m_file);
    }

    void writeLE32(uint32_t val) {
        uint8_t bytes[4] = {
            static_cast<uint8_t>(val),
            static_cast<uint8_t>(val >> 8),
            static_cast<uint8_t>(val >> 16),
            static_cast<uint8_t>(val >> 24)
        };
        std::fwrite(bytes, 1, 4, m_file);
    }

    void writeLE64(uint64_t val) {
        writeLE32(static_cast<uint32_t>(val));
        writeLE32(static_cast<uint32_t>(val >> 32));
    }

    // Pad to 4-byte boundary
    void writePadding(size_t dataLen) {
        size_t padLen = (4 - (dataLen & 3)) & 3;
        uint8_t zeros[4] = {0, 0, 0, 0};
        if (padLen > 0) {
            std::fwrite(zeros, 1, padLen, m_file);
        }
    }

    bool writeSHB() {
        // Section Header Block
        uint32_t blockLen = 28;  // Minimum SHB size without options
        
        writeLE32(PCAPNG_SHB);           // Block Type
        writeLE32(blockLen);              // Block Total Length
        writeLE32(0x1A2B3C4D);           // Byte-Order Magic
        writeLE16(1);                     // Major Version
        writeLE16(0);                     // Minor Version
        writeLE64(0xFFFFFFFFFFFFFFFF);   // Section Length (unspecified)
        writeLE32(blockLen);              // Block Total Length (repeated)
        
        return !std::ferror(m_file);
    }

    bool writeIDB() {
        // Interface Description Block
        uint32_t blockLen = 20;  // Minimum IDB size without options
        
        writeLE32(PCAPNG_IDB);            // Block Type
        writeLE32(blockLen);               // Block Total Length
        writeLE16(LINKTYPE_ETHERNET);      // LinkType
        writeLE16(0);                      // Reserved
        writeLE32(65535);                  // SnapLen
        writeLE32(blockLen);               // Block Total Length (repeated)
        
        return !std::ferror(m_file);
    }

    bool writeEPB(const uint8_t* frame, size_t length,
                  FrameDirection direction, Timestamp timestamp) {
        (void)direction;  // Could be used for custom options
        
        // Enhanced Packet Block
        size_t paddedLen = (length + 3) & ~3;
        
        uint32_t blockLen = 32 + paddedLen;
        
        // Timestamp in microseconds
        uint64_t ts_us = timestamp;
        uint32_t ts_high = static_cast<uint32_t>(ts_us >> 32);
        uint32_t ts_low = static_cast<uint32_t>(ts_us);
        
        writeLE32(PCAPNG_EPB);             // Block Type
        writeLE32(blockLen);                // Block Total Length
        writeLE32(0);                       // Interface ID
        writeLE32(ts_high);                 // Timestamp (High)
        writeLE32(ts_low);                  // Timestamp (Low)
        writeLE32(static_cast<uint32_t>(length));  // Captured Packet Length
        writeLE32(static_cast<uint32_t>(length));  // Original Packet Length
        
        // Packet data
        std::fwrite(frame, 1, length, m_file);
        writePadding(length);
        
        writeLE32(blockLen);                // Block Total Length (repeated)
        
        m_fileSize += blockLen;
        return !std::ferror(m_file);
    }
};

// ============================================================================
// Factory Function
// ============================================================================

std::unique_ptr<IPcapLogger> createPcapLogger() {
    return std::make_unique<PcapNGLoggerImpl>();
}

} // namespace hal
} // namespace EtherCAT
