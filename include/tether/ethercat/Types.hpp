/**
 * @file Types.hpp
 * @brief Common EtherCAT type definitions shared across modules
 * 
 * This header provides shared type definitions that are used by multiple
 * EtherCAT modules including the packet router, retry logic, and slave emulator.
 * 
 * The types are designed to be portable between ESP32 target and host testing.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <array>
#include <functional>

#ifdef ESP_PLATFORM
#include "esp_eth_driver.h"
#endif

// Individual type groups (split out of this header).

#include "tether/ethercat/EtherCATProtocolConstants.hpp"
#include "tether/ethercat/EtherCATCommonTypesErrors.hpp"
#include "tether/ethercat/NetworkAbstraction.hpp"
#include "tether/ethercat/EtherCATCommandTypes.hpp"
#include "tether/ethercat/EtherCATWireFormatStructures.hpp"
#include "tether/ethercat/EtherCATRegisterAddresses.hpp"
#include "tether/ethercat/EtherCATALStatusCodes.hpp"
#include "tether/ethercat/EtherCATSlaveState.hpp"
