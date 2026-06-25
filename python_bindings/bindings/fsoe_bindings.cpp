/**
 * @file fsoe_bindings.cpp
 * @brief Python bindings for FSoE (Fail-Safe over EtherCAT)
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/fsoe/FSoESlave.hpp"

namespace py = pybind11;
using namespace EtherCAT::FSoE;

PYBIND11_MODULE(_fsoe, m) {
    m.doc() = "Python bindings for FSoE (Fail-Safe over EtherCAT)";

    // ==== FSoE State ====
    py::enum_<FSoEState>(m, "FSoEState")
        .value("RESET", FSoEState::Reset)
        .value("SESSION", FSoEState::Session)
        .value("CONNECTION", FSoEState::Connection)
        .value("PARAMETER", FSoEState::Parameter)
        .value("DATA", FSoEState::Data)
        .value("FAILSAFE", FSoEState::FailSafe);

    // ==== FSoE Config ====
    py::class_<FSoEConfig>(m, "FSoEConfig")
        .def(py::init<>())
        .def_readwrite("connection_id", &FSoEConfig::connectionId)
        .def_readwrite("watchdog_time_ms", &FSoEConfig::watchdogTimeMs)
        .def_readwrite("safe_inputs_size", &FSoEConfig::safeInputsSize)
        .def_readwrite("safe_outputs_size", &FSoEConfig::safeOutputsSize);

    // ==== Safety Data ====
    py::class_<SafetyData>(m, "SafetyData")
        .def(py::init<>())
        .def_readwrite("data", &SafetyData::data)
        .def_readwrite("valid", &SafetyData::valid);

    // ==== FSoE Connection ====
    py::class_<FSoEConnection>(m, "FSoEConnection")
        .def(py::init<>())
        .def("configure", &FSoEConnection::configure,
             py::arg("config"),
             "Configure FSoE connection")
        .def("open", &FSoEConnection::open,
             "Open FSoE connection")
        .def("close", &FSoEConnection::close,
             "Close FSoE connection")
        .def("get_state", &FSoEConnection::getState,
             "Get current FSoE state")
        .def("is_safe", &FSoEConnection::isSafe,
             "Check if connection is in safe state")
        .def("read_safe_inputs", &FSoEConnection::readSafeInputs,
             "Read safe input data")
        .def("write_safe_outputs", &FSoEConnection::writeSafeOutputs,
             py::arg("data"),
             "Write safe output data")
        .def("process", &FSoEConnection::process,
             "Process FSoE protocol (call in cyclic loop)");

    // ==== FSoE Slave ====
    py::class_<FSoESlave>(m, "FSoESlave")
        .def(py::init<>())
        .def("configure", &FSoESlave::configure,
             py::arg("config"),
             "Configure FSoE slave")
        .def("start", &FSoESlave::start,
             "Start FSoE slave")
        .def("stop", &FSoESlave::stop,
             "Stop FSoE slave")
        .def("get_state", &FSoESlave::getState,
             "Get current FSoE state")
        .def("set_safe_inputs", &FSoESlave::setSafeInputs,
             py::arg("data"),
             "Set safe input data (from application)")
        .def("get_safe_outputs", &FSoESlave::getSafeOutputs,
             "Get safe output data (from master)")
        .def("trigger_failsafe", &FSoESlave::triggerFailSafe,
             "Trigger fail-safe condition")
        .def("process", &FSoESlave::process,
             "Process FSoE protocol (call in cyclic loop)");
}
