/**
 * @file raw_eoe.cpp
 * @brief Ethernet over EtherCAT (EoE) protocol implementation
 */

#include "EoE.hpp"
#include "TetherConfig.hpp"

#if ECAT_FEATURE_EOE_ENABLED

#include "ethercat/raw/internal.hpp"
#include "tether/platform/Platform.hpp"
#include <cstring>

namespace EtherCAT {
namespace EoE {

bool eoe_init() { return true; }
void eoe_deinit() {}
EoEStats eoe_get_stats() { return EoEStats{}; }

} // namespace EoE
} // namespace EtherCAT
#endif
