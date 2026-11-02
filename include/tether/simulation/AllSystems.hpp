#pragma once
/**Master include for all simulation system headers */
#include "systems/MechanicalSystems.hpp"
#include "systems/RotationalSystems.hpp"
#include "systems/AerospaceSystems.hpp"
#include "systems/ThermalSystems.hpp"
#include "systems/FluidSystems.hpp"
#include "systems/ElectricalSystems.hpp"
#include "systems/OpticalSystems.hpp"
#include "systems/ChemicalSystems.hpp"
#include "systems/BiologicalSystems.hpp"
#include "systems/ChaoticSystems.hpp"
#include "systems/DelaySystems.hpp"

#include <memory>
#include <vector>

namespace Simulation {

/**
 * @brief Factory for creating systems by ID.
 * @param systemId  ID from listSystems().
 * @return new system instance (null for unknown IDs).
 */
std::unique_ptr<DynamicalSystem> createSystem(int systemId);

/**
 * @brief Get list of all available systems.
 * @return (id, name) pairs for every registered system.
 */
std::vector<std::pair<int, const char*>> listSystems();

/**
 * @brief Get the total number of systems.
 * @return registered system count.
 */
int systemCount();

} // namespace Simulation
