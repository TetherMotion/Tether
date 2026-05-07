/**
 * @file ethercat_common_stub.cpp
 * @brief Stub Python bindings for EtherCAT Common types
 */

#include <pybind11/pybind11.h>

namespace py = pybind11;

// Placeholder enum for stub
enum class SlaveState {
    INIT = 1,
    PREOP = 2,
    SAFEOP = 4,
    OP = 8
};

PYBIND11_MODULE(_ethercat_common, m) {
    m.doc() = "EtherCAT common types bindings (full implementation coming soon)";
    
    // Placeholder SlaveState enum
    py::enum_<SlaveState>(m, "SlaveState")
        .value("INIT", SlaveState::INIT)
        .value("PREOP", SlaveState::PREOP)
        .value("SAFEOP", SlaveState::SAFEOP)
        .value("OP", SlaveState::OP);
    
    m.def("placeholder", []() { return "EtherCAT common bindings - full implementation pending"; });
}
