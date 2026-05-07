/**
 * @file cia402_stub.cpp
 * @brief Stub Python bindings for CiA 402 drive profile
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_cia402, m) {
    m.doc() = "CiA 402 drive profile bindings (full implementation coming soon)";
    m.def("placeholder", []() { return "CiA 402 bindings - full implementation pending"; });
}
