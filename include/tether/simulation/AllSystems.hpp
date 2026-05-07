#pragma once
/// Master include for all simulation system headers
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

/// Factory for creating systems by ID
std::unique_ptr<DynamicalSystem> createSystem(int systemId);

/// Get list of all available systems
std::vector<std::pair<int, const char*>> listSystems();

/// Get the total number of systems
int systemCount();

} // namespace Simulation
