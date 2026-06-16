/**
 * @file raw_internal_bridge.cpp
 * @brief Minimal bridge — all global state has been removed.
 *
 * The former BridgeContext singleton is gone.  All transport primitives
 * are now member functions on Master; call them directly.
 *
 * Only utility / adapter wrappers remain here for firmware-only code
 * that has not yet been migrated (EtherCATDCConsistency.cpp).
 */

#include "tether/ethercat/EtherCATMaster.hpp"
#include "raw/internal.hpp"
#include "tether/platform/Platform.hpp"

#include <cstring>

namespace EtherCAT {
namespace Raw {

// ============================================================================
// adp_for_slave_index — pure helper, no global state
// ============================================================================

uint16_t adp_for_slave_index(uint16_t slave_index) {
    return Master::adpForSlaveIndex(slave_index);
}

// ============================================================================
// Log deduplication — stateless helpers (thread-local would be ideal
// but a static buffer is acceptable for logging).
// ============================================================================

static char s_dedup_key[192] = {};
static int  s_dedup_count = 0;

void log_dedup_key(int level, const char* key, const char* msg) {
    if (!key) key = msg;
    if (std::strncmp(key, s_dedup_key, sizeof(s_dedup_key)) == 0) {
        s_dedup_count++;
        if (s_dedup_count <= 2) {
            switch (level) {
                case 0: TETHER_LOGI("ec", "%s", msg); break;
                case 1: TETHER_LOGW("ec", "%s", msg); break;
                case 2: TETHER_LOGE("ec", "%s", msg); break;
                default: TETHER_LOGD("ec", "%s", msg); break;
            }
        }
        return;
    }
    if (s_dedup_count > 2)
        TETHER_LOGI("ec", "(previous msg repeated %d times)", s_dedup_count - 2);
    std::strncpy(s_dedup_key, key, sizeof(s_dedup_key) - 1);
    s_dedup_key[sizeof(s_dedup_key) - 1] = '\0';
    s_dedup_count = 1;
    switch (level) {
        case 0: TETHER_LOGI("ec", "%s", msg); break;
        case 1: TETHER_LOGW("ec", "%s", msg); break;
        case 2: TETHER_LOGE("ec", "%s", msg); break;
        default: TETHER_LOGD("ec", "%s", msg); break;
    }
}

void log_dedup(int level, const char* msg) { log_dedup_key(level, msg, msg); }

} // namespace Raw
} // namespace EtherCAT
