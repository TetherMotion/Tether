/**
 * @file raw_voe.cpp
 * @brief Vendor over EtherCAT (VoE) protocol implementation
 */

#include "VoE.hpp"
#include "TetherConfig.hpp"

#if ECAT_FEATURE_VOE_ENABLED

#include "ethercat/raw/internal.hpp"
#include "tether/platform/Platform.hpp"
#include <cstring>

namespace EtherCAT {
namespace VoE {

bool voe_init() { return true; }
void voe_deinit() {}

bool voe_is_initialized() { return true; }

VoEResult voe_transact(const VoERequest* request, void* response_buf, size_t response_cap, size_t* response_len) {
    (void)request; (void)response_buf; (void)response_cap; (void)response_len;
    VoEResult r = {};
    r.success = false;
    r.error_code = VoEError::NOT_INITIALIZED;
    return r;
}

bool voe_send(const VoERequest* request) {
    (void)request;
    return false;
}

bool voe_queue_request(const VoERequest& request) {
    (void)request;
    return false;
}

size_t voe_pending_count() { return 0; }

bool voe_register_handler(uint32_t vendor_id, VoEHandler handler) {
    (void)vendor_id; (void)handler; return false;
}

void voe_unregister_handler(uint32_t vendor_id) { (void)vendor_id; }

VoEStats voe_get_stats() { VoEStats s = {}; return s; }

void voe_reset_stats() {}

} // namespace VoE
} // namespace EtherCAT
#endif
