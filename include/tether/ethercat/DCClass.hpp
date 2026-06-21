/**
 * @file DCClass.hpp
 * @brief Class-based EtherCAT Distributed Clock (DC) synchronization (umbrella header)
 *
 * This header has been split into focused, modular headers:
 * - DCTypes.hpp    — SlaveTimeInfo, DCLoopStats, DCConfig, DCState, DCRegisters, DCSyncActBits
 * - EtherCATDC.hpp — EtherCATDC class and NoDistributedClockConfigured sentinel
 *
 * This umbrella header preserves backward compatibility.
 */

#pragma once

#include "tether/ethercat/DCTypes.hpp"
#include "tether/ethercat/EtherCATDC.hpp"
