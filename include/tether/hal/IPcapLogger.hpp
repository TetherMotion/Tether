/**
 * @file IPcapLogger.hpp
 * @brief PcapNG frame logging interface
 *
 * Provides an interface for logging Ethernet frames to PcapNG format files.
 * Implemented without external libraries as per requirements.
 */

#pragma once

#include "hal/HALTypes.hpp"
#include "hal/IEthernet.hpp"
#include <memory>
#include <string>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// PcapNG Constants
// ============================================================================

/// PcapNG block types
enum class PcapBlockType : uint32_t {
    SectionHeader = 0x0A0D0D0A,
    InterfaceDescription = 0x00000001,
    EnhancedPacket = 0x00000006,
    SimplePacket = 0x00000003,
    NameResolution = 0x00000004,
    InterfaceStatistics = 0x00000005,
    CustomBlock = 0x00000BAD,
};

/// Link type for Ethernet
constexpr uint16_t kLinkTypeEthernet = 1;

// ============================================================================
// PcapNG Logger Interface
// ============================================================================

/**
 * @brief Configuration for PcapNG logging
 */
struct PcapLoggerConfig {
    std::string filename;             ///< Output file path
    std::string interfaceName;        ///< Interface description
    std::string interfaceDescription; ///< Additional description
    bool logTx = true;                ///< Log transmitted frames
    bool logRx = true;                ///< Log received frames
    bool appendMode = false;          ///< Append to existing file
    size_t maxFileSize = 0;           ///< Maximum file size (0 = unlimited)
    bool rotateFiles = false;         ///< Rotate files when max size reached
    int maxRotatedFiles = 5;          ///< Maximum number of rotated files
};

/**
 * @brief Direction for logged frames
 */
enum class FrameDirection {
    Rx,     ///< Received frame
    Tx,     ///< Transmitted frame
};

/**
 * @brief Abstract PcapNG logger interface
 */
class IPcapLogger {
public:
    virtual ~IPcapLogger() = default;
    
    /**
     * @brief Initialize the logger
     * @param config Configuration
     * @return Error code
     */
    virtual Error init(const PcapLoggerConfig& config) = 0;
    
    /**
     * @brief Close the logger
     */
    virtual void close() = 0;
    
    /**
     * @brief Check if logger is open
     */
    virtual bool isOpen() const = 0;
    
    /**
     * @brief Log a frame
     * @param frame Frame data
     * @param length Frame length
     * @param direction RX or TX
     * @param timestamp Timestamp in microseconds (0 = use current time)
     * @return Error code
     */
    virtual Error logFrame(const uint8_t* frame, size_t length,
                           FrameDirection direction,
                           Timestamp timestamp = 0) = 0;
    
    /**
     * @brief Log a frame with metadata
     * @param frame Frame data
     * @param length Frame length
     * @param direction RX or TX
     * @param info Frame metadata
     * @return Error code
     */
    virtual Error logFrameWithInfo(const uint8_t* frame, size_t length,
                                    FrameDirection direction,
                                    const RxFrameInfo& info) = 0;
    
    /**
     * @brief Flush pending writes
     */
    virtual void flush() = 0;
    
    /**
     * @brief Get number of logged frames
     */
    virtual uint64_t getFrameCount() const = 0;
    
    /**
     * @brief Get current file size
     */
    virtual size_t getFileSize() const = 0;
    
    /**
     * @brief Get statistics
     */
    struct Stats {
        uint64_t rxFrames = 0;
        uint64_t txFrames = 0;
        uint64_t totalBytes = 0;
        uint64_t droppedFrames = 0;
    };
    virtual Stats getStats() const = 0;
};

// ============================================================================
// Logging Ethernet Wrapper
// ============================================================================

/**
 * @brief Ethernet wrapper that logs all frames to PcapNG
 */
class LoggingEthernetWrapper : public IEthernet {
public:
    LoggingEthernetWrapper(std::unique_ptr<IEthernet> inner,
                           std::unique_ptr<IPcapLogger> logger);
    ~LoggingEthernetWrapper() override;
    
    // IEthernet implementation
    Error init(const EthernetConfig& config) override;
    void shutdown() override;
    bool isInitialized() const override;
    Error getMacAddress(MacAddress& mac) const override;
    Error setMacAddress(const MacAddress& mac) override;
    Error transmit(const uint8_t* frame, size_t length) override;
    Error transmitVlan(const uint8_t* frame, size_t length,
                       uint16_t vlanId, uint8_t priority) override;
    Error transmitGather(const BufferDesc* iov, size_t count) override;
    void setRxCallback(RxCallback callback, void* userData) override;
    int poll(Milliseconds timeoutMs) override;
    void setEthertypeFilter(uint16_t ethertype) override;
    Error setPromiscuous(bool enable) override;
    Error addMulticastAddress(const MacAddress& mac) override;
    Error removeMulticastAddress(const MacAddress& mac) override;
    Error setAllMulticast(bool enable) override;
    LinkStatus getLinkStatus() const override;
    void setLinkCallback(LinkCallback callback, void* userData) override;
    Error waitForLinkUp(Milliseconds timeoutMs) override;
    EthernetStats getStats() const override;
    void resetStats() override;
    void* nativeHandle() override;
    const char* getInterfaceName() const override;
    
    // Logging control
    void enableTxLogging(bool enable);
    void enableRxLogging(bool enable);
    IPcapLogger& getLogger() { return *m_logger; }
    
private:
    std::unique_ptr<IEthernet> m_inner;
    std::unique_ptr<IPcapLogger> m_logger;
    RxCallback m_userCallback;
    void* m_userData;
    bool m_logTx;
    bool m_logRx;
    
    void handleRx(const uint8_t* frame, size_t length,
                  const RxFrameInfo& info, void* userData);
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create PcapNG logger
 */
std::unique_ptr<IPcapLogger> createPcapLogger();

/**
 * @brief Create logging Ethernet wrapper
 */
std::unique_ptr<IEthernet> createLoggingEthernet(
    std::unique_ptr<IEthernet> inner,
    const PcapLoggerConfig& config);

} // namespace HAL
} // namespace EtherCAT
