/**
 * @file fsoe_stub.cpp
 * @brief Stub Python bindings for FSoE
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_fsoe, m) {
    m.doc() = "FSoE (Fail-Safe over EtherCAT) bindings (full implementation coming soon)";
    m.def("placeholder", []() { return "FSoE bindings - full implementation pending"; });
}
