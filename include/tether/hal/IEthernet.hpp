/**
 * @file IEthernet.hpp
 * @brief Ethernet Hardware Abstraction Layer Interface
 *
 * This header defines the comprehensive interface for raw Ethernet access
 * across different platforms: ESP32, Linux, STM32.
 */

#pragma once

#include "hal/HALTypes.hpp"
#include "packetloggers/PacketLogger.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace EtherCAT {
namespace HAL {

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Ethernet interface configuration
 */
struct EthernetConfig {
    const char* interfaceName = nullptr;  ///< Interface name ("eth0", "enp0s3", etc.)
    MacAddress macAddress;                 ///< MAC address (zero = use default)
    bool promiscuous = false;              ///< Enable promiscuous mode
    bool enableVlan = false;               ///< Enable 802.1q VLAN processing
    uint16_t vlanId = 0;                   ///< Default VLAN ID (1-4094)
    uint8_t vlanPriority = 0;              ///< Default VLAN priority (0-7)
    size_t rxBufferSize = 32;              ///< Number of RX buffers
    size_t txBufferSize = 16;              ///< Number of TX buffers
    uint16_t ethertypeFilter = 0;          ///< Filter by EtherType (0 = all)
};

/**
 * @brief Ethernet link status
 */
struct LinkStatus {
    bool up = false;                ///< Link is up
    uint32_t speedMbps = 0;         ///< Link speed in Mbps
    bool fullDuplex = false;        ///< Full duplex mode
    bool autoneg = false;           ///< Autonegotiation enabled
};

/**
 * @brief Ethernet statistics
 */
struct EthernetStats {
    uint64_t txFrames = 0;      ///< Frames transmitted
    uint64_t txBytes = 0;       ///< Bytes transmitted
    uint64_t txErrors = 0;      ///< Transmit errors
    uint64_t txDropped = 0;     ///< Frames dropped (buffer full)
    
    uint64_t rxFrames = 0;      ///< Frames received
    uint64_t rxBytes = 0;       ///< Bytes received
    uint64_t rxErrors = 0;      ///< Receive errors
    uint64_t rxDropped = 0;     ///< Frames dropped (buffer full)
    uint64_t rxFiltered = 0;    ///< Frames filtered by EtherType
    
    uint64_t collisions = 0;    ///< Collision count (half-duplex)
};

/**
 * @brief Received frame metadata
 */
struct RxFrameInfo {
    Timestamp timestamp = 0;           ///< Receive timestamp (microseconds)
    uint16_t vlanId = 0;               ///< VLAN ID (0 = no VLAN or stripped)
    uint8_t vlanPriority = 0;          ///< VLAN priority
    bool vlanTagPresent = false;       ///< VLAN tag was present
    bool checksumValid = false;        ///< Hardware checksum validated
    uint32_t hwTimestampNs = 0;        ///< Hardware timestamp (if available)
};

// ============================================================================
// Callback Types
// ============================================================================

/**
 * @brief Callback for received frames
 * 
 * @param frame Pointer to frame data (including Ethernet header).
 *               **Lifetime contract:** the buffer pointed to by `frame` is
 *               only valid for the duration of the callback. The HAL frees
 *               or reuses it immediately after the callback returns. If the
 *               receiver needs the data beyond the callback, it must copy
 *               it into its own storage before returning.
 * @param length Frame length in bytes
 * @param info Frame metadata
 * @param userData User-provided context
 */
using RxCallback = std::function<void(const uint8_t* frame, size_t length, 
                                       const RxFrameInfo& info, void* userData)>;

/**
 * @brief Callback for link state changes
 */
using LinkCallback = std::function<void(const LinkStatus& status, void* userData)>;

// ============================================================================
// Ethernet HAL Interface
// ============================================================================

/**
 * @brief Abstract Ethernet hardware abstraction interface
 */
class IEthernet {
public:
    virtual ~IEthernet() = default;
    
    // --- Lifecycle ---
    
    /**
     * @brief Initialize the Ethernet interface
     * @param config Configuration parameters
     * @return Error code
     */
    virtual Error init(const EthernetConfig& config = {}) = 0;
    
    /**
     * @brief Shutdown the interface
     */
    virtual void shutdown() = 0;
    
    /**
     * @brief Check if initialized
     */
    virtual bool isInitialized() const = 0;
    
    // --- MAC Address ---
    
    /**
     * @brief Get the interface MAC address
     * @param[out] mac MAC address buffer
     * @return Error code
     */
    virtual Error getMacAddress(MacAddress& mac) const = 0;
    
    /**
     * @brief Set a custom MAC address
     * @param mac New MAC address
     * @return Error code
     */
    virtual Error setMacAddress(const MacAddress& mac) = 0;
    
    // --- Transmit ---
    
    /**
     * @brief Transmit a raw Ethernet frame
     * @param frame Complete frame including Ethernet header
     * @param length Frame length in bytes
     * @return Error code
     * 
     * The frame must include the Ethernet header but NOT the FCS.
     * The hardware will add the FCS automatically.
     */
    virtual Error transmit(const uint8_t* frame, size_t length) = 0;
    
    /**
     * @brief Transmit with VLAN tag
     * @param frame Frame data (without VLAN tag)
     * @param length Frame length
     * @param vlanId VLAN ID
     * @param priority VLAN priority
     * @return Error code
     */
    virtual Error transmitVlan(const uint8_t* frame, size_t length,
                               uint16_t vlanId, uint8_t priority = 0) = 0;
    
    /**
     * @brief Transmit multiple frames (scatter-gather)
     * @param iov Array of buffer descriptors
     * @param count Number of descriptors
     * @return Error code
     * 
     * Note: Not all platforms support this efficiently.
     */
    struct BufferDesc {
        const uint8_t* data;
        size_t length;
    };
    virtual Error transmitGather(const BufferDesc* iov, size_t count) = 0;
    
    // --- Receive ---
    
    /**
     * @brief Set callback for received frames
     * @param callback Function to call on frame reception. The `frame` buffer
     *                  passed to the callback is only valid for the duration
     *                  of the callback invocation — see RxCallback docs.
     * @param userData User context passed to callback
     */
    virtual void setRxCallback(RxCallback callback, void* userData = nullptr) = 0;
    
    /**
     * @brief Poll for received frames (for polling-mode implementations)
     * @param timeoutMs Maximum time to wait (0 = non-blocking)
     * @return Number of frames processed
     */
    virtual int poll(Milliseconds timeoutMs = 0) = 0;
    
    // --- Filtering ---
    
    /**
     * @brief Set EtherType filter
     * @param ethertype Filter value (0 = disable filter)
     */
    virtual void setEthertypeFilter(uint16_t ethertype) = 0;
    
    /**
     * @brief Enable/disable promiscuous mode
     * @param enable Enable promiscuous mode
     * @return Error code
     */
    virtual Error setPromiscuous(bool enable) = 0;
    
    /**
     * @brief Add multicast MAC address to filter
     * @param mac Multicast MAC address
     * @return Error code
     */
    virtual Error addMulticastAddress(const MacAddress& mac) = 0;
    
    /**
     * @brief Remove multicast MAC address from filter
     * @param mac Multicast MAC address
     * @return Error code
     */
    virtual Error removeMulticastAddress(const MacAddress& mac) = 0;
    
    /**
     * @brief Enable/disable all-multicast mode
     * @param enable Enable all-multicast reception
     * @return Error code
     */
    virtual Error setAllMulticast(bool enable) = 0;
    
    // --- Link Status ---
    
    /**
     * @brief Get current link status
     */
    virtual LinkStatus getLinkStatus() const = 0;
    
    /**
     * @brief Set callback for link state changes
     */
    virtual void setLinkCallback(LinkCallback callback, void* userData = nullptr) = 0;
    
    /**
     * @brief Wait for link up
     * @param timeoutMs Timeout in milliseconds
     * @return Error code (Timeout if link doesn't come up)
     */
    virtual Error waitForLinkUp(Milliseconds timeoutMs) = 0;
    
    // --- Statistics ---
    
    /**
     * @brief Get interface statistics
     */
    virtual EthernetStats getStats() const = 0;
    
    /**
     * @brief Reset statistics counters
     */
    virtual void resetStats() = 0;
    
    // --- Platform-specific ---
    
    /**
     * @brief Get native driver handle
     * @return Platform-specific handle (void* on ESP32, socket fd on Linux, etc.)
     */
    virtual void* nativeHandle() = 0;
    
    /**
     * @brief Get interface name
     */
    virtual const char* getInterfaceName() const = 0;
};

// ============================================================================
// VLAN Wrapper
// ============================================================================

/**
 * @brief VLAN tagging wrapper around IEthernet
 * 
 * This class wraps an IEthernet implementation and handles 802.1q
 * VLAN tagging/untagging transparently.
 */
class VlanEthernetWrapper : public IEthernet {
public:
    /**
     * @brief Create VLAN wrapper
     * @param inner Inner Ethernet implementation
     * @param vlanId Default VLAN ID
     * @param priority Default priority
     * @param stripTags Strip incoming VLAN tags
     */
    VlanEthernetWrapper(std::unique_ptr<IEthernet> inner, 
                        uint16_t vlanId, uint8_t priority = 0,
                        bool stripTags = true);
    
    ~VlanEthernetWrapper() override;
    
    // IEthernet implementation - delegates to inner with VLAN handling
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
    
    // VLAN-specific methods
    void setDefaultVlan(uint16_t vlanId, uint8_t priority = 0);
    void setTagStripping(bool strip);
    
private:
    std::unique_ptr<IEthernet> m_inner;
    uint16_t m_vlanId;
    uint8_t m_vlanPriority;
    bool m_stripTags;
    RxCallback m_userCallback;
    void* m_userData;
    
    void handleRx(const uint8_t* frame, size_t length, 
                  const RxFrameInfo& info, void* userData);
};

// ============================================================================
// Traffic Splitter
// ============================================================================

/**
 * @brief Traffic filter rule
 */
struct TrafficRule {
    uint16_t ethertype = 0;       ///< Match EtherType (0 = don't match)
    MacAddress dstMac;            ///< Match destination MAC
    bool matchDstMac = false;     ///< Enable MAC matching
    uint16_t vlanId = 0;          ///< Match VLAN ID (0 = don't match)
    int priority = 0;             ///< Rule priority (higher = first)
    RxCallback callback;          ///< Callback for matched frames
    void* userData = nullptr;     ///< User data for callback
};

/**
 * @brief Traffic splitter for routing frames to different handlers
 */
class TrafficSplitter : public IEthernet {
public:
    TrafficSplitter(std::unique_ptr<IEthernet> inner);
    ~TrafficSplitter() override;
    
    /**
     * @brief Add a traffic rule
     * @param rule Rule to add
     * @return Rule ID for later removal
     */
    int addRule(const TrafficRule& rule);
    
    /**
     * @brief Remove a traffic rule
     * @param ruleId Rule ID returned by addRule
     */
    void removeRule(int ruleId);
    
    /**
     * @brief Set default callback for unmatched frames
     */
    void setDefaultCallback(RxCallback callback, void* userData = nullptr);
    
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
    
private:
    std::unique_ptr<IEthernet> m_inner;
    std::vector<std::pair<int, TrafficRule>> m_rules;
    RxCallback m_defaultCallback;
    void* m_defaultUserData;
    int m_nextRuleId;
    
    void handleRx(const uint8_t* frame, size_t length,
                  const RxFrameInfo& info, void* userData);
};

// ============================================================================
// Logging Ethernet Wrapper
// ============================================================================

/**
 * @brief Ethernet wrapper that logs all frames through a PacketLogger
 */
class LoggingEthernetWrapper : public IEthernet {
public:
    LoggingEthernetWrapper(std::unique_ptr<IEthernet> inner,
                           std::shared_ptr<Tether::PacketLoggers::PacketLogger> logger);
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
    Tether::PacketLoggers::PacketLogger& getLogger() { return *m_logger; }

private:
    std::unique_ptr<IEthernet> m_inner;
    std::shared_ptr<Tether::PacketLoggers::PacketLogger> m_logger;
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
 * @brief Create platform-appropriate Ethernet implementation
 */
std::unique_ptr<IEthernet> createDefaultEthernet();

/**
 * @brief Create ESP32 Ethernet implementation
 */
#ifdef HAL_PLATFORM_ESP32
std::unique_ptr<IEthernet> createESP32Ethernet();
#endif

/**
 * @brief Create Linux raw socket Ethernet implementation
 */
#ifdef HAL_PLATFORM_LINUX
std::unique_ptr<IEthernet> createLinuxRawSocketEthernet();
#endif

/**
 * @brief Create STM32 Ethernet implementation
 */
#ifdef HAL_PLATFORM_STM32
std::unique_ptr<IEthernet> createSTM32Ethernet();
#endif

/**
 * @brief Create mock Ethernet implementation (for testing)
 */
std::unique_ptr<IEthernet> createMockEthernet();

/**
 * @brief Create VLAN-wrapped Ethernet
 */
std::unique_ptr<IEthernet> createVlanEthernet(std::unique_ptr<IEthernet> inner,
                                               uint16_t vlanId, uint8_t priority = 0);

/**
 * @brief Create traffic splitter
 */
std::unique_ptr<TrafficSplitter> createTrafficSplitter(std::unique_ptr<IEthernet> inner);

/**
 * @brief Create logging Ethernet wrapper
 * @param inner Inner Ethernet implementation
 * @param logger Packet logger (must be already initialized)
 */
std::unique_ptr<IEthernet> createLoggingEthernet(
    std::unique_ptr<IEthernet> inner,
    std::shared_ptr<Tether::PacketLoggers::PacketLogger> logger);

} // namespace HAL
} // namespace EtherCAT
