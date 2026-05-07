/**
 * @file cia402_bindings.cpp
 * @brief Python bindings for CiA 402 drive profile
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "tether/cia402/CiA402Drive.hpp"
#include "tether/cia402/CiA402StateMachine.hpp"
#include "tether/cia402/HomingHandler.hpp"
#include "tether/cia402/MotionProfile.hpp"
#include "tether/cia301/CiA402Defs.hpp"

namespace py = pybind11;
using namespace EtherCAT;

PYBIND11_MODULE(_cia402, m) {
    m.doc() = "Python bindings for CiA 402 drive profile";

    // ==== Operation Mode Enum ====
    py::enum_<CiA402::OperationMode>(m, "OperationMode")
        .value("NO_MODE", CiA402::OperationMode::NO_MODE)
        .value("PP", CiA402::OperationMode::PP)
        .value("VL", CiA402::OperationMode::VL)
        .value("PV", CiA402::OperationMode::PV)
        .value("TQ", CiA402::OperationMode::TQ)
        .value("HM", CiA402::OperationMode::HM)
        .value("IP", CiA402::OperationMode::IP)
        .value("CSP", CiA402::OperationMode::CSP)
        .value("CSV", CiA402::OperationMode::CSV)
        .value("CST", CiA402::OperationMode::CST);

    // ==== Drive State Enum ====
    py::enum_<CiA402::DriveState>(m, "DriveState")
        .value("NOT_READY_TO_SWITCH_ON", CiA402::DriveState::NOT_READY_TO_SWITCH_ON)
        .value("SWITCH_ON_DISABLED", CiA402::DriveState::SWITCH_ON_DISABLED)
        .value("READY_TO_SWITCH_ON", CiA402::DriveState::READY_TO_SWITCH_ON)
        .value("SWITCHED_ON", CiA402::DriveState::SWITCHED_ON)
        .value("OPERATION_ENABLED", CiA402::DriveState::OPERATION_ENABLED)
        .value("QUICK_STOP_ACTIVE", CiA402::DriveState::QUICK_STOP_ACTIVE)
        .value("FAULT_REACTION_ACTIVE", CiA402::DriveState::FAULT_REACTION_ACTIVE)
        .value("FAULT", CiA402::DriveState::FAULT);

    // ==== Homing Method Enum ====
    py::enum_<CiA402::HomingMethod>(m, "HomingMethod")
        .value("CURRENT_POSITION", CiA402::HomingMethod::CURRENT_POSITION)
        .value("NEGATIVE_LIMIT_SWITCH", CiA402::HomingMethod::NEGATIVE_LIMIT_SWITCH)
        .value("POSITIVE_LIMIT_SWITCH", CiA402::HomingMethod::POSITIVE_LIMIT_SWITCH)
        .value("HOME_SWITCH_NEGATIVE", CiA402::HomingMethod::HOME_SWITCH_NEGATIVE)
        .value("HOME_SWITCH_POSITIVE", CiA402::HomingMethod::HOME_SWITCH_POSITIVE)
        .value("INDEX_PULSE", CiA402::HomingMethod::INDEX_PULSE);

    // ==== PDO Mapping Preset ====
    py::enum_<PDOMappingPreset>(m, "PDOMappingPreset")
        .value("CSP_BASIC", PDOMappingPreset::CSP_Basic)
        .value("CSP_FULL", PDOMappingPreset::CSP_Full)
        .value("CSV_BASIC", PDOMappingPreset::CSV_Basic)
        .value("CSV_FULL", PDOMappingPreset::CSV_Full)
        .value("CST_BASIC", PDOMappingPreset::CST_Basic)
        .value("CST_FULL", PDOMappingPreset::CST_Full)
        .value("PP_MODE", PDOMappingPreset::PP_Mode);

    // ==== Control Word bits ====
    py::class_<CiA402::ControlWord>(m, "ControlWord")
        .def(py::init<>())
        .def_readwrite("value", &CiA402::ControlWord::raw)
        .def_property("switch_on",
            [](const CiA402::ControlWord& cw) { return cw.bits.switchOn; },
            [](CiA402::ControlWord& cw, bool v) { cw.bits.switchOn = v; })
        .def_property("enable_voltage",
            [](const CiA402::ControlWord& cw) { return cw.bits.enableVoltage; },
            [](CiA402::ControlWord& cw, bool v) { cw.bits.enableVoltage = v; })
        .def_property("quick_stop",
            [](const CiA402::ControlWord& cw) { return cw.bits.quickStop; },
            [](CiA402::ControlWord& cw, bool v) { cw.bits.quickStop = v; })
        .def_property("enable_operation",
            [](const CiA402::ControlWord& cw) { return cw.bits.enableOperation; },
            [](CiA402::ControlWord& cw, bool v) { cw.bits.enableOperation = v; });

    // ==== Status Word bits ====
    py::class_<CiA402::StatusWord>(m, "StatusWord")
        .def(py::init<>())
        .def_readonly("value", &CiA402::StatusWord::raw)
        .def_property_readonly("ready_to_switch_on",
            [](const CiA402::StatusWord& sw) { return sw.bits.readyToSwitchOn; })
        .def_property_readonly("switched_on",
            [](const CiA402::StatusWord& sw) { return sw.bits.switchedOn; })
        .def_property_readonly("operation_enabled",
            [](const CiA402::StatusWord& sw) { return sw.bits.operationEnabled; })
        .def_property_readonly("fault",
            [](const CiA402::StatusWord& sw) { return sw.bits.fault; })
        .def_property_readonly("target_reached",
            [](const CiA402::StatusWord& sw) { return sw.bits.targetReached; });

    // ==== Drive Config ====
    py::class_<CiA402DriveConfig>(m, "DriveConfig")
        .def(py::init<>())
        .def_readwrite("slave_index", &CiA402DriveConfig::slaveIndex)
        .def_readwrite("operation_mode", &CiA402DriveConfig::operationMode)
        .def_readwrite("pdo_preset", &CiA402DriveConfig::pdoPreset);

    // ==== CiA 402 Drive ====
    py::class_<CiA402Drive>(m, "CiA402Drive")
        .def(py::init<uint16_t>(), py::arg("slave_index"))
        .def("configure", &CiA402Drive::configure,
             py::arg("config"),
             "Configure drive parameters")
        .def("apply_pdo_mapping", &CiA402Drive::applyPDOMapping,
             py::arg("preset"),
             "Apply PDO mapping preset")
        .def("enable", &CiA402Drive::enable,
             "Enable drive (transition to OPERATION_ENABLED)")
        .def("disable", &CiA402Drive::disable,
             "Disable drive")
        .def("quick_stop", &CiA402Drive::quickStop,
             "Execute quick stop")
        .def("fault_reset", &CiA402Drive::faultReset,
             "Reset fault condition")
        .def("set_mode", &CiA402Drive::setMode,
             py::arg("mode"),
             "Set operation mode")
        .def("get_mode", &CiA402Drive::getMode,
             "Get current operation mode")
        .def("get_state", &CiA402Drive::getState,
             "Get current drive state")
        .def("get_status_word", &CiA402Drive::getStatusWord,
             "Get current status word")
        .def("set_target_position", &CiA402Drive::setTargetPosition,
             py::arg("position"),
             "Set target position (CSP mode)")
        .def("set_target_velocity", &CiA402Drive::setTargetVelocity,
             py::arg("velocity"),
             "Set target velocity (CSV mode)")
        .def("set_target_torque", &CiA402Drive::setTargetTorque,
             py::arg("torque"),
             "Set target torque (CST mode)")
        .def("get_actual_position", &CiA402Drive::getActualPosition,
             "Get actual position")
        .def("get_actual_velocity", &CiA402Drive::getActualVelocity,
             "Get actual velocity")
        .def("get_actual_torque", &CiA402Drive::getActualTorque,
             "Get actual torque")
        .def("start_homing", &CiA402Drive::startHoming,
             py::arg("method"),
             "Start homing procedure")
        .def("is_homing_complete", &CiA402Drive::isHomingComplete,
             "Check if homing is complete")
        .def("home_to_current_position", &CiA402Drive::homeToCurrentPosition,
             "Set current position as home")
        .def("update", &CiA402Drive::update,
             "Update drive state (call in cyclic loop)");

    // ==== Homing Handler ====
    py::class_<CiA402HomingHandler>(m, "HomingHandler")
        .def(py::init<CiA402Drive&>())
        .def("start", &CiA402HomingHandler::start,
             py::arg("method"),
             "Start homing with specified method")
        .def("update", &CiA402HomingHandler::update,
             "Update homing state")
        .def("is_complete", &CiA402HomingHandler::isComplete,
             "Check if homing is complete")
        .def("is_error", &CiA402HomingHandler::isError,
             "Check if homing error occurred")
        .def("abort", &CiA402HomingHandler::abort,
             "Abort homing procedure");
}
