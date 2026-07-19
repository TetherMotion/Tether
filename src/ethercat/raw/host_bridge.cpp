// host_bridge.cpp
//
// Historically held process-global "registered NetworkInterface / src MAC"
// state used by host-side examples to locate the active EtherCAT master.
// That global state has been removed to comply with the "no global state"
// requirement: examples now pass their NetworkInterface / src MAC directly
// to Master::start(), and incoming frames are delivered via
// Master::handleRxFrame() (or the VLAN router) without going through a
// process-global registry.
//
// This file is intentionally left empty so the existing GLOB-based build
// rules in cmake/components/ethercat_master.cmake keep working unchanged.

namespace EtherCAT {
namespace Raw {
} // namespace Raw
} // namespace EtherCAT
