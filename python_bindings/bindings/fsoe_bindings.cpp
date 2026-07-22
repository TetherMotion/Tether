/**
 * @file fsoe_bindings.cpp
 * @brief Python bindings for FSoE (Fail-Safe over EtherCAT)
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tether/fsoe/FSoEMasterConnection.hpp"
#include "tether/fsoe/FSoESlave.hpp"

namespace py = pybind11;
using namespace FSoE;

PYBIND11_MODULE(_fsoe, m) {
    m.doc() = "Python bindings for FSoE (Fail-Safe over EtherCAT)";

    // ==== FSoE Connection State ====
    py::enum_<uint8_t>(m, "ConnectionState")
        .value("RESET", ConnectionState::Reset)
        .value("SESSION", ConnectionState::Session)
        .value("CONNECTION", ConnectionState::Connection)
        .value("PARAMETER", ConnectionState::Parameter)
        .value("DATA", ConnectionState::Data)
        .value("FAILSAFE", ConnectionState::FailSafe)
        .value("ERROR", ConnectionState::Error);

    // ==== Master Connection Config ====
    py::class_<MasterConnectionConfig>(m, "MasterConnectionConfig")
        .def(py::init<>())
        .def_readwrite("slave_addr", &MasterConnectionConfig::slave_addr)
        .def_readwrite("slave_safety_addr", &MasterConnectionConfig::slave_safety_addr)
        .def_readwrite("connection_id", &MasterConnectionConfig::connection_id)
        .def_readwrite("master_addr", &MasterConnectionConfig::master_addr)
        .def_readwrite("watchdog_timeout_ms", &MasterConnectionConfig::watchdog_timeout_ms)
        .def_readwrite("conn_timeout_ms", &MasterConnectionConfig::conn_timeout_ms)
        .def_readwrite("session_timeout_ms", &MasterConnectionConfig::session_timeout_ms)
        .def_readwrite("recovery_delay_ms", &MasterConnectionConfig::recovery_delay_ms)
        .def_readwrite("safety_level", &MasterConnectionConfig::safety_level)
        .def_readwrite("input_size", &MasterConnectionConfig::input_size)
        .def_readwrite("output_size", &MasterConnectionConfig::output_size)
        .def_readwrite("auto_recovery_enabled", &MasterConnectionConfig::auto_recovery_enabled)
        .def_readwrite("auto_fail_safe_on_error", &MasterConnectionConfig::auto_fail_safe_on_error);

    // ==== FSoEMasterConnection ====
    py::class_<FSoEMasterConnection>(m, "FSoEMasterConnection")
        .def(py::init<const MasterConnectionConfig&>(), py::arg("config"))
        .def("initialize", &FSoEMasterConnection::initialize,
             "Initialize FSoE master connection")
        .def("is_initialized", &FSoEMasterConnection::isInitialized,
             "Check if initialized")
        .def("start_connection", &FSoEMasterConnection::startConnection,
             "Start FSoE connection")
        .def("reset_connection", &FSoEMasterConnection::resetConnection,
             "Reset FSoE connection")
        .def("trigger_fail_safe", &FSoEMasterConnection::triggerFailSafe,
             py::arg("error_code") = ErrorCode::ApplicationError,
             "Trigger fail-safe condition")
        .def("clear_error", &FSoEMasterConnection::clearError,
             "Clear error state")
        .def("process_rx_frame", [](FSoEMasterConnection& self, py::bytes data) {
             std::string s = data;
             return self.processRxFrame(reinterpret_cast<const uint8_t*>(s.data()), s.size());
         }, py::arg("data"),
             "Process received FSoE frame from slave")
        .def("prepare_tx_frame", [](FSoEMasterConnection& self, size_t max_len) {
             std::vector<uint8_t> buf(max_len, 0);
             size_t len = self.prepareTxFrame(buf.data(), max_len);
             buf.resize(len);
             return buf;
         }, py::arg("max_len") = 64,
             "Prepare FSoE frame to send to slave")
        .def("update", &FSoEMasterConnection::update,
             py::arg("current_time_ms"),
             "Update state machine (call periodically)")
        .def("set_safe_outputs", [](FSoEMasterConnection& self, py::bytes data) {
             std::string s = data;
             return self.setSafeOutputs(reinterpret_cast<const uint8_t*>(s.data()), s.size());
         }, py::arg("data"),
             "Write safe output data")
        .def("get_safe_inputs", [](FSoEMasterConnection& self) {
             std::vector<uint8_t> buf(self.getConfig().input_size, 0);
             size_t len = self.getSafeInputs(buf.data(), buf.size());
             buf.resize(len);
             return py::bytes(reinterpret_cast<const char*>(buf.data()), buf.size());
         },
             "Read safe input data")
        .def("get_state", &FSoEMasterConnection::getState,
             "Get current FSoE state")
        .def("is_operational", &FSoEMasterConnection::isOperational,
             "Check if connection is operational")
        .def("is_fail_safe", &FSoEMasterConnection::isFailSafe,
             "Check if connection is in fail-safe state")
        .def("get_error_code", &FSoEMasterConnection::getErrorCode,
             "Get last error code")
        .def("get_stats", &FSoEMasterConnection::getStats,
             py::return_value_policy::reference_internal,
             "Get connection statistics")
        .def("reset_stats", &FSoEMasterConnection::resetStats,
             "Reset statistics")
        .def("get_diagnostics", &FSoEMasterConnection::getDiagnostics,
             "Get diagnostic string");

    // ==== FSoE Slave Config ====
    py::class_<FSoESlaveConfig>(m, "FSoESlaveConfig")
        .def(py::init<>())
        .def_readwrite("slave_address", &FSoESlaveConfig::slaveAddress)
        .def_readwrite("connection_id", &FSoESlaveConfig::connectionId)
        .def_readwrite("safety_address", &FSoESlaveConfig::safetyAddress)
        .def_readwrite("safety_level", &FSoESlaveConfig::safetyLevel)
        .def_readwrite("watchdog_timeout_ms", &FSoESlaveConfig::watchdogTimeoutMs)
        .def_readwrite("connection_timeout_ms", &FSoESlaveConfig::connectionTimeoutMs)
        .def_readwrite("safe_input_size", &FSoESlaveConfig::safeInputSize)
        .def_readwrite("safe_output_size", &FSoESlaveConfig::safeOutputSize)
        .def_readwrite("auto_recovery_enabled", &FSoESlaveConfig::autoRecoveryEnabled);

    // ==== FSoE Slave ====
    py::class_<FSoESlave>(m, "FSoESlave")
        .def(py::init<const FSoESlaveConfig&>(), py::arg("config"))
        .def("initialize", &FSoESlave::initialize,
             "Initialize FSoE slave")
        .def("is_initialized", &FSoESlave::isInitialized,
             "Check if initialized")
        .def("get_state", &FSoESlave::getState,
             "Get current FSoE state")
        .def("is_operational", &FSoESlave::isOperational,
             "Check if slave is operational")
        .def("is_fail_safe", &FSoESlave::isFailSafe,
             "Check if slave is in fail-safe state")
        .def("trigger_fail_safe", &FSoESlave::triggerFailSafe,
             py::arg("error_code") = ErrorCode::ApplicationError,
             "Trigger fail-safe condition")
        .def("attempt_recovery", &FSoESlave::attemptRecovery,
             "Attempt recovery from fail-safe")
        .def("reset", &FSoESlave::reset,
             "Reset state machine")
        .def("process_rx_frame", [](FSoESlave& self, py::bytes data) {
             std::string s = data;
             return self.processRxFrame(reinterpret_cast<const uint8_t*>(s.data()), s.size());
         }, py::arg("data"),
             "Process received FSoE frame from master")
        .def("prepare_tx_frame", [](FSoESlave& self, size_t max_len) {
             std::vector<uint8_t> buf(max_len, 0);
             size_t len = self.prepareTxFrame(buf.data(), max_len);
             buf.resize(len);
             return buf;
         }, py::arg("max_len") = 64,
             "Prepare FSoE frame to send to master")
        .def("update", &FSoESlave::update,
             py::arg("current_time_ms"),
             "Update state machine (call periodically)")
        .def("set_safe_inputs", [](FSoESlave& self, py::bytes data) {
             std::string s = data;
             return self.setSafeInputs(reinterpret_cast<const uint8_t*>(s.data()), s.size());
         }, py::arg("data"),
             "Set safe input data (from application)")
        .def("get_safe_outputs", [](FSoESlave& self) {
             std::vector<uint8_t> buf(self.getConfig().safeOutputSize, 0);
             size_t len = self.getSafeOutputs(buf.data(), buf.size());
             buf.resize(len);
             return py::bytes(reinterpret_cast<const char*>(buf.data()), buf.size());
         },
             "Get safe output data (from master)");
}
