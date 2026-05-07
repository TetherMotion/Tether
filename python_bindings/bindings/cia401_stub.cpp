/**
 * @file cia401_stub.cpp
 * @brief Stub Python bindings for CiA 401 I/O profile
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_cia401, m) {
    m.doc() = "CiA 401 I/O profile bindings (full implementation coming soon)";
    m.def("placeholder", []() { return "CiA 401 bindings - full implementation pending"; });
}
