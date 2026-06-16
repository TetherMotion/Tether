/**
 * @file EtherCATVoE.hpp
 * @brief Vendor-specific over EtherCAT (VoE) protocol implementation
 * 
 * @details
 * VoE (Vendor over EtherCAT) provides a vendor-specific mailbox channel for
 * proprietary features not covered by standard EtherCAT protocols. This allows
 * vendors to implement custom functionality while maintaining interoperability.
 * 
 * ## Protocol Overview
 * 
 * VoE messages use mailbox type 0x0F (vendor specific). The payload format
 * is entirely vendor-defined, but typically includes:
 * 
 * ```
 * ┌──────────────┬────────────────┬────────────────────────┐
 * │ Mailbox Hdr  │ Vendor Header  │ Vendor Payload         │
 * │ (6 bytes)    │ (vendor def)   │ (vendor defined)       │
 * └──────────────┴────────────────┴────────────────────────┘
 * ```
 * 
 * ## Common VoE Use Cases
 * 
 * - **Proprietary Diagnostics**: Vendor-specific diagnostic protocols
 * - **Custom Configuration**: Parameters not in standard object dictionary
 * - **Firmware Features**: Access to proprietary firmware functionality
 * - **Debug Interfaces**: Development and debugging tools
 * - **Legacy Protocol Tunneling**: Wrapping legacy protocols over EtherCAT
 * 
 * ## Usage Examples
 * 
 * ### Send vendor-specific command
 * @code
 * #include "EtherCATVoE.hpp"
 * 
 * using namespace EtherCAT::VoE;
 * 
 * // Define vendor-specific command structure
 * struct MyVendorCmd {
 *     uint8_t cmd_code;
 *     uint16_t param1;
 *     uint32_t param2;
 * } __attribute__((packed));
 * 
 * MyVendorCmd cmd = { .cmd_code = 0x01, .param1 = 100, .param2 = 0x12345678 };
 * 
 * VoERequest req = {
 *     .slave_index = 0,
 *     .vendor_id = 0x00001234,  // Your vendor ID
 *     .data = &cmd,
 *     .data_len = sizeof(cmd),
 *     .timeout_ms = 1000
 * };
 * 
 * uint8_t response[64];
 * size_t resp_len;
 * VoEResult result = voe_transact(&req, response, sizeof(response), &resp_len);
 * 
 * if (result.success) {
 *     printf("VoE response: %zu bytes\n", resp_len);
 * }
 * @endcode
 * 
 * ### Register vendor-specific handler (slave simulation)
 * @code
 * // Handler for incoming VoE messages
 * bool my_voe_handler(uint32_t vendor_id, const uint8_t* data, size_t len,
 *                     uint8_t* response, size_t* resp_len) {
 *     if (vendor_id != MY_VENDOR_ID) return false;
 *     
 *     // Process command, fill response
 *     response[0] = 0x00; // Success
 *     *resp_len = 1;
 *     return true;
 * }
 * 
 * voe_register_handler(MY_VENDOR_ID, my_voe_handler);
 * @endcode
 * 
 * ### Async VoE with callback
 * @code
 * void on_voe_complete(const VoEResponse& resp) {
 *     if (resp.success) {
 *         process_vendor_response(resp.data, resp.data_len);
 *     }
 * }
 * 
 * VoERequest req = {};
 * req.slave_index = 0;
 * req.vendor_id = 0x00001234;
 * req.callback = on_voe_complete;
 * 
 * voe_queue_request(req);
 * @endcode
 * 
 * ## Vendor ID
 * 
 * EtherCAT vendor IDs are assigned by ETG. Using an unregistered vendor ID
 * is acceptable for internal development but not for products.
 * 
 * ## Thread Safety
 * 
 * VoE requests are processed by the SDO background task. The queue is
 * thread-safe for submitting requests from any context.
 * 
 * @note Configure VoE in EtherCATConfig.hpp with ECAT_FEATURE_VOE_ENABLED
 */

#pragma once

#include "EtherCATConfig.hpp"

#if ECAT_FEATURE_VOE_ENABLED

#include <cstdint>
#include <cstddef>
#include <functional>
#include <atomic>

namespace EtherCAT {
namespace VoE {

// ============================================================================
// Constants
// ============================================================================

/** @brief VoE mailbox type identifier */
constexpr uint8_t kVoEMailboxType = 0x0F;

/** @brief Maximum VoE data size */
constexpr size_t kMaxDataSize = ECAT_VOE_MAX_DATA_SIZE;

/** @brief VoE queue depth */
constexpr size_t kQueueDepth = ECAT_VOE_QUEUE_DEPTH;

// ============================================================================
// Error Codes
// ============================================================================

/**
 * @brief VoE error codes
 * 
 * Since VoE is vendor-specific, most errors are protocol-level rather than
 * vendor-defined.
 */
enum class VoEError : uint32_t {
    SUCCESS           = 0x0000,  ///< No error
    TIMEOUT           = 0x0001,  ///< Communication timeout
    MAILBOX_ERROR     = 0x0002,  ///< Mailbox communication failed
    INVALID_VENDOR_ID = 0x0003,  ///< Unknown/unsupported vendor ID
    BUFFER_TOO_SMALL  = 0x0004,  ///< Response buffer too small
    INVALID_STATE     = 0x0005,  ///< Invalid protocol state
    NOT_INITIALIZED   = 0x0006,  ///< VoE not initialized
    QUEUE_FULL        = 0x0007,  ///< Request queue full
    VENDOR_ERROR      = 0x1000,  ///< Vendor-specific error (check vendor data)
};

/**
 * @brief Get error string
 */
const char* voe_error_string(VoEError error);

// ============================================================================
// Request/Response Structures
// ============================================================================

/**
 * @brief VoE response structure (for async)
 */
struct VoEResponse {
    bool success;               ///< true if transaction succeeded
    VoEError error_code;        ///< Error code if failed
    uint16_t slave_index;       ///< Responding slave
    uint32_t vendor_id;         ///< Vendor ID from response
    uint8_t data[kMaxDataSize]; ///< Response data
    size_t data_len;            ///< Response data length
};

/**
 * @brief VoE completion callback
 */
using VoECallback = std::function<void(const VoEResponse&)>;

/**
 * @brief VoE request structure
 */
struct VoERequest {
    uint16_t slave_index;       ///< Target slave (0-based)
    uint32_t vendor_id;         ///< Vendor ID
    const void* data;           ///< Request data
    size_t data_len;            ///< Request data length
    uint32_t timeout_ms;        ///< Timeout (default: ECAT_VOE_TIMEOUT_MS)
    VoECallback callback;       ///< Optional completion callback
    void* user_data;            ///< User-provided context
    
    VoERequest()
        : slave_index(0)
        , vendor_id(0)
        , data(nullptr)
        , data_len(0)
        , timeout_ms(ECAT_VOE_TIMEOUT_MS)
        , callback(nullptr)
        , user_data(nullptr)
    {}
};

/**
 * @brief VoE transaction result (for sync operations)
 */
struct VoEResult {
    bool success;           ///< true if transaction succeeded
    VoEError error_code;    ///< Error code if failed
    size_t response_len;    ///< Response data length
    uint32_t duration_ms;   ///< Transaction duration
    
    operator bool() const { return success; }
};

// ============================================================================
// Vendor Handler (for implementing VoE responders)
// ============================================================================

/**
 * @brief Vendor message handler callback
 * 
 * @param vendor_id Vendor ID from request
 * @param request Request data
 * @param request_len Request data length
 * @param response Response buffer
 * @param response_cap Response buffer capacity
 * @param[out] response_len Actual response length
 * @return true if handled, false if not (try next handler)
 */
using VoEHandler = bool(*)(
    uint32_t vendor_id,
    const uint8_t* request,
    size_t request_len,
    uint8_t* response,
    size_t response_cap,
    size_t* response_len
);

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize VoE subsystem
 * @return true on success
 */
bool voe_init();

/**
 * @brief Deinitialize VoE subsystem
 */
void voe_deinit();

/**
 * @brief Check if VoE is initialized
 */
bool voe_is_initialized();

// ============================================================================
// Synchronous API
// ============================================================================

/**
 * @brief Perform synchronous VoE transaction
 * 
 * Sends request and waits for response.
 * 
 * @param request VoE request
 * @param response_buf Buffer for response data
 * @param response_cap Response buffer capacity
 * @param[out] response_len Actual response length
 * @return Transaction result
 */
VoEResult voe_transact(const VoERequest* request,
                       void* response_buf, size_t response_cap,
                       size_t* response_len);

/**
 * @brief Send VoE data without waiting for response
 * 
 * Fire-and-forget for commands that don't return data.
 * 
 * @param request VoE request
 * @return true if sent successfully
 */
bool voe_send(const VoERequest* request);

// ============================================================================
// Asynchronous API
// ============================================================================

/**
 * @brief Queue async VoE request
 * 
 * Request is processed in background; callback invoked on completion.
 * 
 * @param request VoE request (must have callback set)
 * @return true if queued successfully
 */
bool voe_queue_request(const VoERequest& request);

/**
 * @brief Get number of pending VoE requests
 */
size_t voe_pending_count();

// ============================================================================
// Handler Registration
// ============================================================================

/**
 * @brief Register vendor-specific handler
 * 
 * Used when implementing VoE responders (less common for masters).
 * 
 * @param vendor_id Vendor ID to handle (0 = all)
 * @param handler Handler callback
 * @return true if registered
 */
bool voe_register_handler(uint32_t vendor_id, VoEHandler handler);

/**
 * @brief Unregister vendor handler
 */
void voe_unregister_handler(uint32_t vendor_id);

// ============================================================================
// Statistics
// ============================================================================

/**
 * @brief VoE statistics
 */
struct VoEStats {
    uint32_t requests_sent;     ///< Total requests sent
    uint32_t responses_received;///< Responses received
    uint32_t timeouts;          ///< Timeout count
    uint32_t errors;            ///< Error count
    uint64_t bytes_sent;        ///< Total bytes sent
    uint64_t bytes_received;    ///< Total bytes received
};

/**
 * @brief Get VoE statistics
 */
VoEStats voe_get_stats();

/**
 * @brief Reset VoE statistics
 */
void voe_reset_stats();

// ============================================================================
// Wire Format Structures
// ============================================================================

/**
 * @brief VoE header structure
 * 
 * The header format is implementation-specific. This is a common layout.
 * Some vendors use different headers.
 */
struct __attribute__((packed)) VoEHeader {
    uint32_t vendor_id_le;  ///< Vendor ID (little-endian)
    uint16_t vendor_type;   ///< Vendor-specific message type
    // Followed by vendor-specific payload
};
static_assert(sizeof(VoEHeader) == 6, "VoEHeader size check");

} // namespace VoE

// ============================================================================
// VoEManager — instance-based wrapper
// ============================================================================

class Master; // forward

class VoEManager {
public:
    explicit VoEManager(Master& master) : master_(master) {}
    ~VoEManager() { VoE::voe_deinit(); }

    bool init()   { return VoE::voe_init(); }
    void deinit() { VoE::voe_deinit(); }

    VoE::VoEResult transact(const VoE::VoERequest* req, void* resp, size_t cap, size_t* len) {
        return VoE::voe_transact(req, resp, cap, len);
    }
    bool send(const VoE::VoERequest* req) { return VoE::voe_send(req); }
    bool queueRequest(const VoE::VoERequest& req) { return VoE::voe_queue_request(req); }
    bool registerHandler(uint32_t vid, VoE::VoEHandler h) { return VoE::voe_register_handler(vid, h); }
    void unregisterHandler(uint32_t vid) { VoE::voe_unregister_handler(vid); }

    Master& master() { return master_; }
private:
    Master& master_;
};

} // namespace EtherCAT

#endif // ECAT_FEATURE_VOE_ENABLED
