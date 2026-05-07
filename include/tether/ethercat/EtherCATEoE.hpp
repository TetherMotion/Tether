/**
 * @file EtherCATEoE.hpp
 * @brief Ethernet over EtherCAT (EoE) protocol implementation
 * 
 * @details
 * EoE (Ethernet over EtherCAT) tunnels standard Ethernet frames through the
 * EtherCAT mailbox protocol. This creates a virtual Ethernet network between
 * the master and slaves that support EoE.
 * 
 * ## Protocol Overview
 * 
 * EoE fragments standard Ethernet frames into mailbox-sized chunks:
 * 
 * ```
 * Standard Ethernet Frame (up to 1518 bytes)
 * ┌──────────┬──────────────────────────────────────────┐
 * │ Eth Hdr  │ Payload (IP, ARP, etc.)                  │
 * └──────────┴──────────────────────────────────────────┘
 *      ↓ Fragment into mailbox-sized pieces
 * ┌──────────┬──────────┬───────────────┐
 * │ Mbx Hdr  │ EoE Hdr  │ Fragment 1    │ → Mailbox TX
 * └──────────┴──────────┴───────────────┘
 * ┌──────────┬──────────┬───────────────┐
 * │ Mbx Hdr  │ EoE Hdr  │ Fragment 2    │ → Mailbox TX
 * └──────────┴──────────┴───────────────┘
 *                    ...
 * ┌──────────┬──────────┬──────┐
 * │ Mbx Hdr  │ EoE Hdr  │ Last │ → Mailbox TX (last fragment)
 * └──────────┴──────────┴──────┘
 * ```
 * 
 * ## EoE Frame Types
 * 
 * | Type | Name | Description |
 * |------|------|-------------|
 * | 0 | FRAGMENT_REQ | Ethernet frame fragment (request) |
 * | 1 | TIMESTAMP_RES | Timestamp response |
 * | 2 | INIT_REQ | Initialize request |
 * | 3 | INIT_RES | Initialize response |
 * | 4 | SET_IP_REQ | Set IP parameters request |
 * | 5 | SET_IP_RES | Set IP parameters response |
 * | 6 | SET_FILTER_REQ | Set address filter request |
 * | 7 | SET_FILTER_RES | Set address filter response |
 * | 8 | GET_IP_REQ | Get IP parameters request |
 * | 9 | GET_IP_RES | Get IP parameters response |
 * 
 * ## Use Cases
 * 
 * - **Remote Configuration**: Web-based slave configuration via HTTP
 * - **Diagnostics**: TCP/IP-based diagnostic protocols
 * - **IT Integration**: Connect industrial devices to enterprise networks
 * - **Protocol Tunneling**: TCP/IP applications over EtherCAT
 * 
 * ## Architecture
 * 
 * ```
 *  Application Layer (HTTP, SSH, etc.)
 *         │
 *   ┌─────▼─────┐
 *   │   TCP/IP  │  (lwIP on ESP32)
 *   │   Stack   │
 *   └─────┬─────┘
 *         │
 *   ┌─────▼─────┐
 *   │ EoE TAP   │  Virtual network interface
 *   │ Interface │
 *   └─────┬─────┘
 *         │
 *   ┌─────▼─────┐
 *   │    EoE    │  Fragment/reassemble
 *   │  Protocol │
 *   └─────┬─────┘
 *         │
 *   ┌─────▼─────┐
 *   │ EtherCAT  │  Mailbox transport
 *   │  Mailbox  │
 *   └───────────┘
 * ```
 * 
 * ## Usage Examples
 * 
 * ### Initialize EoE for a slave
 * @code
 * #include "EtherCATEoE.hpp"
 * 
 * using namespace EtherCAT::EoE;
 * 
 * // Configure EoE for slave 0
 * EoESlaveConfig cfg;
 * cfg.slave_index = 0;
 * cfg.mac_address[0] = 0x02; // Locally administered
 * cfg.mac_address[1] = 0x00;
 * cfg.mac_address[2] = 0x00;
 * cfg.mac_address[3] = 0x00;
 * cfg.mac_address[4] = 0x00;
 * cfg.mac_address[5] = 0x01;
 * 
 * if (eoe_configure_slave(cfg)) {
 *     printf("EoE configured for slave %d\n", cfg.slave_index);
 * }
 * @endcode
 * 
 * ### Assign IP address to slave
 * @code
 * EoEIPConfig ip_cfg;
 * ip_cfg.slave_index = 0;
 * ip_cfg.ip_address = IP4_ADDR(192, 168, 1, 100);
 * ip_cfg.subnet_mask = IP4_ADDR(255, 255, 255, 0);
 * ip_cfg.gateway = IP4_ADDR(192, 168, 1, 1);
 * ip_cfg.dns_server = IP4_ADDR(8, 8, 8, 8);
 * 
 * if (eoe_set_ip(ip_cfg)) {
 *     printf("IP assigned to slave\n");
 * }
 * @endcode
 * 
 * ### Send raw Ethernet frame
 * @code
 * uint8_t frame[1518];
 * size_t frame_len = build_ethernet_frame(frame, ...);
 * 
 * EoEResult result = eoe_send_frame(0, frame, frame_len);
 * if (!result.success) {
 *     printf("Send failed: %s\n", eoe_error_string(result.error_code));
 * }
 * @endcode
 * 
 * ### Register frame receive callback
 * @code
 * void on_frame_received(uint16_t slave_index, const uint8_t* frame, size_t len) {
 *     printf("Received %zu bytes from slave %d\n", len, slave_index);
 *     // Pass to IP stack or process directly
 * }
 * 
 * eoe_register_frame_callback(on_frame_received);
 * @endcode
 * 
 * ### Create virtual network interface (ESP32)
 * @code
 * // Create TAP-like interface for lwIP
 * esp_netif_t* eoe_netif = eoe_create_netif(0, "eoe0");
 * if (eoe_netif) {
 *     // Can now use standard socket API over EoE
 * }
 * @endcode
 * 
 * ## Performance Considerations
 * 
 * - EoE adds significant overhead (fragmentation + mailbox)
 * - Typical throughput: 100 Kbit/s - 1 Mbit/s depending on cycle time
 * - Not suitable for high-bandwidth applications
 * - Latency is much higher than native EtherCAT PDO
 * 
 * ## Thread Safety
 * 
 * EoE runs in a dedicated background task. Frame transmission is thread-safe.
 * Receive callbacks are invoked from the EoE task context.
 * 
 * @note Configure EoE in EtherCATConfig.hpp with ECAT_FEATURE_EOE_ENABLED
 */

#pragma once

#include "EtherCATConfig.hpp"

#if ECAT_FEATURE_EOE_ENABLED

#include <cstdint>
#include <cstddef>
#include <functional>
#include <atomic>

namespace EtherCAT {
namespace EoE {

// ============================================================================
// Constants
// ============================================================================

/** @brief EoE mailbox type identifier */
constexpr uint8_t kEoEMailboxType = 0x02;

/** @brief Maximum Ethernet frame size */
constexpr size_t kMaxFrameSize = ECAT_EOE_MAX_FRAME_SIZE;

/** @brief EoE fragment size */
constexpr size_t kFragmentSize = ECAT_EOE_FRAGMENT_SIZE;

/** @brief Number of frame buffers */
constexpr size_t kFrameBufferCount = ECAT_EOE_FRAME_BUFFER_COUNT;

/** @brief Ethernet header size */
constexpr size_t kEthernetHeaderSize = 14;

/** @brief Minimum Ethernet frame size (excluding FCS) */
constexpr size_t kMinFrameSize = 60;

// ============================================================================
// EoE Frame Types
// ============================================================================

/**
 * @brief EoE frame type codes
 */
enum class EoEFrameType : uint8_t {
    FRAGMENT_REQ    = 0,   ///< Ethernet frame fragment
    TIMESTAMP_RES   = 1,   ///< Timestamp response
    INIT_REQ        = 2,   ///< Initialize request
    INIT_RES        = 3,   ///< Initialize response
    SET_IP_REQ      = 4,   ///< Set IP parameters request
    SET_IP_RES      = 5,   ///< Set IP parameters response
    SET_FILTER_REQ  = 6,   ///< Set address filter request
    SET_FILTER_RES  = 7,   ///< Set address filter response
    GET_IP_REQ      = 8,   ///< Get IP parameters request
    GET_IP_RES      = 9,   ///< Get IP parameters response
};

/**
 * @brief Get frame type string
 */
const char* eoe_frame_type_string(EoEFrameType type);

// ============================================================================
// Error Codes
// ============================================================================

/**
 * @brief EoE error codes
 */
enum class EoEError : uint32_t {
    SUCCESS             = 0x0000,  ///< No error
    TIMEOUT             = 0x0001,  ///< Communication timeout
    MAILBOX_ERROR       = 0x0002,  ///< Mailbox communication failed
    FRAGMENT_ERROR      = 0x0003,  ///< Fragment reassembly failed
    FRAME_TOO_LARGE     = 0x0004,  ///< Frame exceeds maximum size
    BUFFER_FULL         = 0x0005,  ///< No buffer available
    INVALID_STATE       = 0x0006,  ///< Invalid protocol state
    NOT_INITIALIZED     = 0x0007,  ///< EoE not initialized
    SLAVE_NOT_SUPPORTED = 0x0008,  ///< Slave doesn't support EoE
    IP_ERROR            = 0x0009,  ///< IP configuration error
};

/**
 * @brief Get error string
 */
const char* eoe_error_string(EoEError error);

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief EoE slave configuration
 */
struct EoESlaveConfig {
    uint16_t slave_index;       ///< Slave index (0-based)
    uint8_t mac_address[6];     ///< Virtual MAC address for slave
    bool enabled;               ///< Enable EoE for this slave
    
    EoESlaveConfig()
        : slave_index(0)
        , enabled(true)
    {
        // Default to locally administered MAC
        mac_address[0] = 0x02;
        mac_address[1] = 0x00;
        mac_address[2] = 0x00;
        mac_address[3] = 0x00;
        mac_address[4] = 0x00;
        mac_address[5] = 0x00;
    }
};

/**
 * @brief EoE IP configuration
 */
struct EoEIPConfig {
    uint16_t slave_index;   ///< Slave index
    uint32_t ip_address;    ///< IP address (network byte order)
    uint32_t subnet_mask;   ///< Subnet mask (network byte order)
    uint32_t gateway;       ///< Default gateway (network byte order)
    uint32_t dns_server;    ///< DNS server (network byte order)
    char hostname[32];      ///< Hostname (optional)
    
    EoEIPConfig()
        : slave_index(0)
        , ip_address(0)
        , subnet_mask(0)
        , gateway(0)
        , dns_server(0)
    {
        hostname[0] = '\0';
    }
};

/**
 * @brief Helper to create IPv4 address
 */
static inline uint32_t IP4_ADDR(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
    return (static_cast<uint32_t>(a) << 0) |
           (static_cast<uint32_t>(b) << 8) |
           (static_cast<uint32_t>(c) << 16) |
           (static_cast<uint32_t>(d) << 24);
}

// ============================================================================
// Result Structures
// ============================================================================

/**
 * @brief EoE operation result
 */
struct EoEResult {
    bool success;           ///< true if operation succeeded
    EoEError error_code;    ///< Error code if failed
    
    operator bool() const { return success; }
};

// ============================================================================
// Callbacks
// ============================================================================

/**
 * @brief Frame received callback
 * 
 * Called when a complete Ethernet frame is received from a slave.
 * 
 * @param slave_index Source slave
 * @param frame Frame data (including Ethernet header)
 * @param frame_len Frame length
 */
using EoEFrameCallback = std::function<void(
    uint16_t slave_index,
    const uint8_t* frame,
    size_t frame_len
)>;

/**
 * @brief Link state change callback
 * 
 * @param slave_index Slave that changed state
 * @param link_up true if link is up
 */
using EoELinkCallback = std::function<void(
    uint16_t slave_index,
    bool link_up
)>;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize EoE subsystem
 * @return true on success
 */
bool eoe_init();

/**
 * @brief Deinitialize EoE subsystem
 */
void eoe_deinit();

/**
 * @brief Check if EoE is initialized
 */
bool eoe_is_initialized();

// ============================================================================
// Slave Configuration
// ============================================================================

/**
 * @brief Configure EoE for a slave
 * 
 * @param config Slave configuration
 * @return true on success
 */
bool eoe_configure_slave(const EoESlaveConfig& config);

/**
 * @brief Set slave IP parameters
 * 
 * @param config IP configuration
 * @return true on success
 */
bool eoe_set_ip(const EoEIPConfig& config);

/**
 * @brief Get slave IP parameters
 * 
 * @param slave_index Target slave
 * @param[out] config Received IP configuration
 * @return true on success
 */
bool eoe_get_ip(uint16_t slave_index, EoEIPConfig* config);

/**
 * @brief Check if slave supports EoE
 * 
 * @param slave_index Slave to check
 * @return true if EoE is supported
 */
bool eoe_slave_supports(uint16_t slave_index);

// ============================================================================
// Frame Transmission
// ============================================================================

/**
 * @brief Send Ethernet frame to slave
 * 
 * Frame is fragmented and sent via mailbox.
 * 
 * @param slave_index Target slave
 * @param frame Frame data (including Ethernet header)
 * @param frame_len Frame length (14-1518 bytes)
 * @return Result
 */
EoEResult eoe_send_frame(uint16_t slave_index, const uint8_t* frame, size_t frame_len);

/**
 * @brief Broadcast Ethernet frame to all EoE slaves
 * 
 * @param frame Frame data
 * @param frame_len Frame length
 * @return Number of slaves frame was sent to
 */
size_t eoe_broadcast_frame(const uint8_t* frame, size_t frame_len);

// ============================================================================
// Frame Reception
// ============================================================================

/**
 * @brief Register frame receive callback
 * 
 * Only one callback can be registered at a time.
 * 
 * @param callback Callback function
 */
void eoe_register_frame_callback(EoEFrameCallback callback);

/**
 * @brief Unregister frame receive callback
 */
void eoe_unregister_frame_callback();

/**
 * @brief Register link state callback
 */
void eoe_register_link_callback(EoELinkCallback callback);

/**
 * @brief Unregister link state callback
 */
void eoe_unregister_link_callback();

// ============================================================================
// Network Interface (Platform-Specific)
// ============================================================================

/**
 * @brief Create virtual network interface for EoE
 * 
 * Platform-specific implementation creates a TAP-like interface
 * that integrates with the system's network stack (lwIP on ESP32).
 * 
 * @param slave_index Slave to bridge
 * @param if_name Interface name (e.g., "eoe0")
 * @return Platform-specific interface handle, nullptr on failure
 */
void* eoe_create_netif(uint16_t slave_index, const char* if_name);

/**
 * @brief Destroy virtual network interface
 * 
 * @param netif Interface handle from eoe_create_netif
 */
void eoe_destroy_netif(void* netif);

/**
 * @brief Get network interface for slave
 * 
 * @param slave_index Slave index
 * @return Interface handle, nullptr if not created
 */
void* eoe_get_netif(uint16_t slave_index);

// ============================================================================
// Statistics
// ============================================================================

/**
 * @brief EoE statistics
 */
struct EoEStats {
    uint32_t frames_sent;           ///< Complete frames sent
    uint32_t frames_received;       ///< Complete frames received
    uint32_t fragments_sent;        ///< Total fragments sent
    uint32_t fragments_received;    ///< Total fragments received
    uint32_t fragment_errors;       ///< Fragment reassembly errors
    uint32_t timeouts;              ///< Timeout count
    uint64_t bytes_sent;            ///< Total bytes sent
    uint64_t bytes_received;        ///< Total bytes received
    uint32_t dropped_frames;        ///< Frames dropped (buffer full)
};

/**
 * @brief Get EoE statistics
 */
EoEStats eoe_get_stats();

/**
 * @brief Reset EoE statistics
 */
void eoe_reset_stats();

/**
 * @brief Get EoE statistics for specific slave
 */
EoEStats eoe_get_slave_stats(uint16_t slave_index);

// ============================================================================
// Realtime Loop Integration
// ============================================================================

/**
 * @brief Process pending EoE operations
 * 
 * Called from the realtime loop to process pending transmissions
 * and check for received fragments. This is typically called automatically
 * when EoE is enabled.
 * 
 * @return Number of operations processed
 */
size_t eoe_process();

/**
 * @brief Enable/disable EoE processing in realtime loop
 * 
 * @param enabled true to enable
 */
void eoe_set_enabled(bool enabled);

/**
 * @brief Check if EoE is enabled in realtime loop
 */
bool eoe_is_enabled();

// ============================================================================
// Wire Format Structures
// ============================================================================

/**
 * @brief EoE header structure
 * 
 * 4-byte header at the start of each EoE mailbox message.
 */
struct __attribute__((packed)) EoEHeader {
    // First 16 bits
    uint16_t fragment_number : 6;   ///< Fragment number (0-63)
    uint16_t offset_buffer : 6;     ///< Offset or buffer
    uint16_t frame_number : 4;      ///< Frame number for reassembly
    
    // Second 16 bits
    uint16_t complete : 1;          ///< Last fragment flag
    uint16_t port : 4;              ///< Port number
    uint16_t time_appended : 1;     ///< Timestamp appended
    uint16_t time_request : 1;      ///< Timestamp requested
    uint16_t reserved : 5;          ///< Reserved
    uint16_t frame_type : 4;        ///< Frame type (EoEFrameType)
};
static_assert(sizeof(EoEHeader) == 4, "EoEHeader must be 4 bytes");

/**
 * @brief EoE Set IP request structure
 */
struct __attribute__((packed)) EoESetIPRequest {
    uint32_t flags;             ///< Which parameters to set
    uint32_t ip_address;        ///< IP address
    uint32_t subnet_mask;       ///< Subnet mask
    uint32_t gateway;           ///< Default gateway
    uint32_t dns_server;        ///< DNS server
    char dns_name[32];          ///< DNS name
};

/**
 * @brief EoE Set IP flags
 */
enum class EoEIPFlags : uint32_t {
    MAC_INCLUDED    = 0x01,
    IP_INCLUDED     = 0x02,
    SUBNET_INCLUDED = 0x04,
    GATEWAY_INCLUDED = 0x08,
    DNS_INCLUDED    = 0x10,
    DNS_NAME_INCLUDED = 0x20,
};

} // namespace EoE

// ============================================================================
// EoEManager — instance-based wrapper
// ============================================================================

class EtherCATMaster; // forward

class EoEManager {
public:
    explicit EoEManager(EtherCATMaster& master) : master_(master) {}
    ~EoEManager() { EoE::eoe_deinit(); }

    bool init()    { return EoE::eoe_init(); }
    void deinit()  { EoE::eoe_deinit(); }
    bool isInitialized() const { return EoE::eoe_is_initialized(); }
    bool isEnabled() const { return EoE::eoe_is_enabled(); }
    void setEnabled(bool en) { EoE::eoe_set_enabled(en); }

    EtherCATMaster& master() { return master_; }
private:
    EtherCATMaster& master_;
};

} // namespace EtherCAT

#endif // ECAT_FEATURE_EOE_ENABLED
