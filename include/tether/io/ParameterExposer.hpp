/**
 * @file ParameterExposer.hpp
 * @brief Base interface for Tether modules to expose parameters and signals.
 *
 * Each Tether module (EtherCAT, PID control, motion planner, etc.) can
 * implement an exposer that registers the module's parameters and signals
 * into a Registry.  This decouples the core module code from the IO protocol.
 *
 * The exposer pattern:
 *  1. The core module has no dependency on tether_io_protocol.
 *  2. An optional exposer (in tether_io_protocol or a bridge library) takes
 *     a reference to the module and a Registry, and registers entries.
 *  3. Users wire things up at application level.
 *
 * @copyright Copyright (C) 2025-2026 Tether Authors
 */
#pragma once

#include "tether/io/Registry.hpp"
#include <cstdint>
#include <string>

namespace tether { namespace io {

/**
 * @class IParameterExposer
 * @brief Interface for modules that expose parameters and signals to the IO protocol.
 *
 * Implementations should call registry.addParam() and registry.addSignal()
 * in their expose() method to register all available entries.
 */
class IParameterExposer {
public:
    virtual ~IParameterExposer() = default;

    /// Register all parameters and signals into the registry.
    /// @param registry  The registry to add entries to.
    /// @param idBase    Starting ID for this module's entries (for namespace isolation).
    virtual void expose(Registry& registry, uint64_t idBase) = 0;

    /// Return the module name (e.g. "ethercat", "pid", "motion_planner").
    virtual const char* moduleName() const = 0;
};

/// Helper: generate a parameter ID from a module base + local offset.
inline constexpr uint64_t makeId(uint64_t moduleBase, uint32_t localId) {
    return (moduleBase << 32) | localId;
}

// ---------------------------------------------------------------------------
// Well-known module ID bases (upper 32 bits of the entry ID)
// ---------------------------------------------------------------------------
namespace ModuleId {
    inline constexpr uint64_t EtherCATMaster   = 0x0001;
    inline constexpr uint64_t CiA402Drive      = 0x0002;
    inline constexpr uint64_t PIDController    = 0x0003;
    inline constexpr uint64_t MotionPlanner    = 0x0004;
    inline constexpr uint64_t GCodeInterpreter = 0x0005;
    inline constexpr uint64_t Simulation       = 0x0006;
    inline constexpr uint64_t MotionControl    = 0x0007;
    inline constexpr uint64_t HAL              = 0x0008;
    inline constexpr uint64_t Export           = 0x0009;
    inline constexpr uint64_t Destabilizer     = 0x000A;
    inline constexpr uint64_t CiA301           = 0x000B;
    inline constexpr uint64_t CiA401           = 0x000C;
    inline constexpr uint64_t CiA406           = 0x000D;
    inline constexpr uint64_t FSoE             = 0x000E;
    inline constexpr uint64_t Kinematics       = 0x000F;
    inline constexpr uint64_t Identification   = 0x0010;
    inline constexpr uint64_t SlaveEmulation   = 0x0011;
    inline constexpr uint64_t User             = 0x1000;  ///< User-defined modules start here
} // namespace ModuleId

}} // namespace tether::io
