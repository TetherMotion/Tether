/**
 * @file ethercat_master_bindings.cpp
 * @brief Python bindings for EtherCAT master
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "tether/ethercat/Raw.hpp"
#include "tether/ethercat/SDOManager.hpp"
#include "tether/ethercat/PDOManager.hpp"
#include "tether/ethercat/DC.hpp"
#include "tether/ethercat/Types.hpp"

namespace py = pybind11;
using namespace EtherCAT;

PYBIND11_MODULE(_ethercat_master, m) {
    m.doc() = "Python bindings for EtherCAT master";

    // ==== Master Configuration ====
    py::class_<MasterConfig>(m, "MasterConfig")
        .def(py::init<>())
        .def_readwrite("interface_name", &MasterConfig::interfaceName)
        .def_readwrite("cycle_time_us", &MasterConfig::cycleTimeUs)
        .def_readwrite("dc_enabled", &MasterConfig::dcEnabled);

    // ==== DC Configuration ====
    py::class_<DCConfig>(m, "DCConfig")
        .def(py::init<>())
        .def_readwrite("sync0_cycle", &DCConfig::sync0Cycle)
        .def_readwrite("sync0_shift", &DCConfig::sync0Shift)
        .def_readwrite("sync1_cycle", &DCConfig::sync1Cycle)
        .def_readwrite("sync1_shift", &DCConfig::sync1Shift);

    // ==== Slave Info from scan ====
    py::class_<SlaveInfo>(m, "SlaveInfo")
        .def(py::init<>())
        .def_readonly("position", &SlaveInfo::position)
        .def_readonly("vendor_id", &SlaveInfo::vendorId)
        .def_readonly("product_code", &SlaveInfo::productCode)
        .def_readonly("revision", &SlaveInfo::revision)
        .def_readonly("name", &SlaveInfo::name);

    // ==== Raw EtherCAT Master ====
    py::class_<RawMaster>(m, "Master")
        .def(py::init<>())
        .def("init", &RawMaster::init,
             py::arg("interface_name"),
             "Initialize master on network interface")
        .def("shutdown", &RawMaster::shutdown,
             "Shutdown master")
        .def("scan_network", &RawMaster::scanNetwork,
             "Scan network and return list of slaves")
        .def("get_slave_count", &RawMaster::getSlaveCount,
             "Get number of discovered slaves")
        .def("get_slave_info", &RawMaster::getSlaveInfo,
             py::arg("slave_index"),
             "Get information about a specific slave")
        .def("set_state", &RawMaster::setState,
             py::arg("slave_index"), py::arg("state"),
             "Request state transition for a slave")
        .def("get_state", &RawMaster::getState,
             py::arg("slave_index"),
             "Get current state of a slave")
        .def("process_data", &RawMaster::processData,
             "Exchange process data with all slaves")
        .def("read_sdo", &RawMaster::readSDO,
             py::arg("slave_index"), py::arg("index"), py::arg("subindex"),
             "Read SDO value")
        .def("write_sdo", &RawMaster::writeSDO,
             py::arg("slave_index"), py::arg("index"), py::arg("subindex"), py::arg("data"),
             "Write SDO value")
        .def("configure_dc", &RawMaster::configureDC,
             py::arg("config"),
             "Configure distributed clocks")
        .def("get_dc_time", &RawMaster::getDCTime,
             "Get current DC time");
}
