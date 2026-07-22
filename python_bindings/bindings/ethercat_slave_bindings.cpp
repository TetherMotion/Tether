/**
 * @file ethercat_slave_bindings.cpp
 * @brief Python bindings for EtherCAT slave emulation
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "tether/slave/core/SlaveCore.hpp"
#include "tether/slave/core/ObjectDictionary.hpp"

namespace py = pybind11;
using namespace EtherCAT::slave;

PYBIND11_MODULE(_ethercat_slave, m) {
    m.doc() = "Python bindings for EtherCAT slave emulation";

    // ==== Slave State ====
    py::enum_<SlaveState>(m, "SlaveState")
        .value("INIT", SlaveState::INIT)
        .value("PREOP", SlaveState::PREOP)
        .value("SAFEOP", SlaveState::SAFEOP)
        .value("OP", SlaveState::OP);

    // ==== Slave Configuration ====
    py::class_<SlaveConfig>(m, "SlaveConfig")
        .def(py::init<>())
        .def_readwrite("vendor_id", &SlaveConfig::vendorId)
        .def_readwrite("product_code", &SlaveConfig::productCode)
        .def_readwrite("revision", &SlaveConfig::revision)
        .def_readwrite("serial_number", &SlaveConfig::serialNumber)
        .def_readwrite("name", &SlaveConfig::name);

    // ==== Object Dictionary Entry ====
    py::class_<ObjectEntry>(m, "ObjectEntry")
        .def(py::init<>())
        .def_readwrite("index", &ObjectEntry::index)
        .def_readwrite("subindex", &ObjectEntry::subindex)
        .def_readwrite("data_type", &ObjectEntry::dataType)
        .def_readwrite("access", &ObjectEntry::access)
        .def_readwrite("name", &ObjectEntry::name);

    // ==== Object Dictionary ====
    py::class_<ObjectDictionary>(m, "ObjectDictionary")
        .def(py::init<>())
        .def("add_entry", &ObjectDictionary::addEntry,
             py::arg("entry"),
             "Add an entry to the dictionary")
        .def("get_entry", &ObjectDictionary::getEntry,
             py::arg("index"), py::arg("subindex"),
             "Get an entry from the dictionary")
        .def("read", &ObjectDictionary::read,
             py::arg("index"), py::arg("subindex"),
             "Read value from dictionary")
        .def("write", &ObjectDictionary::write,
             py::arg("index"), py::arg("subindex"), py::arg("data"),
             "Write value to dictionary");

    // ==== PDO Mapping ====
    py::class_<PDOMapping>(m, "PDOMapping")
        .def(py::init<>())
        .def("add_entry", &PDOMapping::addEntry,
             py::arg("index"), py::arg("subindex"), py::arg("bit_length"),
             "Add mapping entry")
        .def("clear", &PDOMapping::clear, "Clear mapping");

    // ==== Slave Core ====
    py::class_<SlaveCore>(m, "SlaveCore")
        .def(py::init<>())
        .def("configure", &SlaveCore::configure,
             py::arg("config"),
             "Configure slave with given parameters")
        .def("start", &SlaveCore::start,
             "Start slave operation")
        .def("stop", &SlaveCore::stop,
             "Stop slave operation")
        .def("process_frame", &SlaveCore::processFrame,
             py::arg("frame"), py::arg("length"),
             "Process incoming EtherCAT frame")
        .def("get_state", &SlaveCore::getState,
             "Get current slave state")
        .def("set_state_callback", &SlaveCore::setStateCallback,
             py::arg("callback"),
             "Set callback for state changes")
        .def("get_object_dictionary", &SlaveCore::getObjectDictionary,
             py::return_value_policy::reference_internal,
             "Get object dictionary");
}
