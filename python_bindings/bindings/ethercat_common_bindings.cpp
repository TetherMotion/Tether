/**
 * @file ethercat_common_bindings.cpp
 * @brief Python bindings for common EtherCAT types
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tether/ethercat/EtherCATTypes.hpp"
#include "tether/hal/HALTypes.hpp"

namespace py = pybind11;
using namespace EtherCAT;

PYBIND11_MODULE(_ethercat_common, m) {
    m.doc() = "Python bindings for common EtherCAT types";

    // ==== Slave State Enum ====
    py::enum_<SlaveState>(m, "SlaveState")
        .value("INIT", SlaveState::INIT)
        .value("PREOP", SlaveState::PREOP)
        .value("SAFEOP", SlaveState::SAFEOP)
        .value("OP", SlaveState::OP)
        .value("BOOTSTRAP", SlaveState::BOOTSTRAP);

    // ==== AL Status ====
    py::enum_<ALStatus>(m, "AlStatus")
        .value("INIT", ALStatus::INIT)
        .value("PREOP", ALStatus::PREOP)
        .value("SAFEOP", ALStatus::SAFEOP)
        .value("OP", ALStatus::OP);

    // ==== Error Code Enum ====
    py::enum_<hal::Error>(m, "ErrorCode")
        .value("OK", hal::Error::OK)
        .value("TIMEOUT", hal::Error::TIMEOUT)
        .value("INVALID_PARAM", hal::Error::INVALID_PARAM)
        .value("NOT_INITIALIZED", hal::Error::NOT_INITIALIZED)
        .value("BUSY", hal::Error::BUSY)
        .value("NO_MEMORY", hal::Error::NO_MEMORY);

    // ==== MAC Address ====
    py::class_<hal::MacAddress>(m, "MacAddress")
        .def(py::init<>())
        .def(py::init([](const std::array<uint8_t, 6>& bytes) {
            hal::MacAddress mac;
            std::copy(bytes.begin(), bytes.end(), mac.bytes.begin());
            return mac;
        }))
        .def("__str__", [](const hal::MacAddress& mac) {
            char buf[18];
            snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                     mac.bytes[0], mac.bytes[1], mac.bytes[2],
                     mac.bytes[3], mac.bytes[4], mac.bytes[5]);
            return std::string(buf);
        })
        .def_property("bytes",
            [](const hal::MacAddress& mac) { 
                return std::vector<uint8_t>(mac.bytes.begin(), mac.bytes.end()); 
            },
            [](hal::MacAddress& mac, const std::vector<uint8_t>& bytes) {
                if (bytes.size() == 6) {
                    std::copy(bytes.begin(), bytes.end(), mac.bytes.begin());
                }
            });

    // ==== EtherCAT Address ====
    py::class_<EtherCATAddress>(m, "EtherCATAddress")
        .def(py::init<>())
        .def(py::init<uint16_t>(), py::arg("configured_address"))
        .def_readwrite("configured_address", &EtherCATAddress::configuredAddress)
        .def_readwrite("auto_increment", &EtherCATAddress::autoIncrement);

    // ==== Slave Info ====
    py::class_<SlaveInfo>(m, "SlaveInfo")
        .def(py::init<>())
        .def_readonly("vendor_id", &SlaveInfo::vendorId)
        .def_readonly("product_code", &SlaveInfo::productCode)
        .def_readonly("revision", &SlaveInfo::revision)
        .def_readonly("serial_number", &SlaveInfo::serialNumber)
        .def_readonly("name", &SlaveInfo::name);
}
