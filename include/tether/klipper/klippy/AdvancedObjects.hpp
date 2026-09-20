#pragma once

// Generated feature flags (TETHER_ENABLE_*). Including it here keeps
// feature-gated declarations identical between the library and consumers.
#if __has_include("tether/TetherConfig.hpp")
#include "tether/TetherConfig.hpp"
#endif

/// @file AdvancedObjects.hpp
/// @brief Umbrella header for all advanced Klippy feature components.
///
/// This file includes all topic-based component headers. Individual
/// components can also be included directly for faster compilation.

#include "tether/klipper/klippy/VirtualSdcard.hpp"
#include "tether/klipper/klippy/BedMeshPrinterObject.hpp"
#if TETHER_ENABLE_PRESSURE_ADVANCE
#include "tether/klipper/klippy/PressureAdvance.hpp"
#endif
#include "tether/klipper/klippy/InputShaper.hpp"
#include "tether/klipper/klippy/FirmwareRetraction.hpp"
#include "tether/kinematics/DeltaPrinter.hpp"
#include "tether/klipper/klippy/TmcDriverConfig.hpp"
#include "tether/klipper/klippy/FilamentLoader.hpp"
#include "tether/klipper/klippy/MultiMcuManager.hpp"
#include "tether/klipper/klippy/SkewCorrection.hpp"
#include "tether/klipper/klippy/CaseLight.hpp"
