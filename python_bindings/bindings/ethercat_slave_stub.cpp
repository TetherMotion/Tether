/**
 * @file ethercat_slave_stub.cpp
 * @brief Stub Python bindings for EtherCAT Slave
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_ethercat_slave, m) {
    m.doc() = "EtherCAT slave emulation bindings (full implementation coming soon)";
    m.def("placeholder", []() { return "EtherCAT slave bindings - full implementation pending"; });
}
