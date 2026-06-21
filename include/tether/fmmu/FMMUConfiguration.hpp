/**
 * @file FMMUConfiguration.hpp
 * @brief FMMU Configuration (umbrella header)
 *
 * This header has been split into focused, modular headers:
 * - FMMUTypes.hpp   — FMMUType enum, FMMUConfig struct (bitstruct-backed), FMMURegBlock
 * - FMMUManager.hpp — SlaveFMMUConfig, IFMMUTransport, FMMUManager class
 *
 * This umbrella header preserves backward compatibility.
 */

#pragma once

#include "tether/fmmu/FMMUTypes.hpp"
#include "tether/fmmu/FMMUManager.hpp"
