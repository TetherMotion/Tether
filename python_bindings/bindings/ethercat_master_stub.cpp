/**
 * @file ethercat_master_stub.cpp
 * @brief Stub Python bindings for EtherCAT Master
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_ethercat_master, m) {
    m.doc() = "EtherCAT master bindings (full implementation coming soon)";
    m.def("placeholder", []() { return "EtherCAT master bindings - full implementation pending"; });
}
