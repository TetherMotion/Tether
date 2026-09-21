/**
 * @file MasterTransports.hpp
 * @brief Factories for the Master transport-adapter classes.
 *
 * Internal header — not part of the public API.  The adapter classes
 * (MasterPDOTransport, MasterFaultTransport, Master::MasterSDOTransport)
 * live in MasterTransports.cpp; Master only sees the abstract interfaces.
 */

#pragma once

#include <memory>

namespace EtherCAT {

class Master;
class IPDOTransport;
class IFaultTransport;
namespace SDO { class ISDOTransport; }

/// Adapt Master to the PDO-manager transport interface.
std::unique_ptr<IPDOTransport> makeMasterPDOTransport(Master& master);

/// Adapt Master to the fault-detector transport interface.
std::unique_ptr<IFaultTransport> makeMasterFaultTransport(Master& master);

/// Adapt Master to the CoE SDO transport interface.
std::unique_ptr<SDO::ISDOTransport> makeMasterSDOTransport(Master& master);

} // namespace EtherCAT
