/**
 * @file cia401_bindings.cpp
 * @brief Python bindings for CiA 401 I/O profile
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "tether/cia401/CiA401IO.hpp"

namespace py = pybind11;
using namespace EtherCAT;

PYBIND11_MODULE(_cia401, m) {
    m.doc() = "Python bindings for CiA 401 I/O profile";

    // ==== I/O Config ====
    py::class_<CiA401Config>(m, "IOConfig")
        .def(py::init<>())
        .def_readwrite("slave_index", &CiA401Config::slaveIndex)
        .def_readwrite("digital_inputs", &CiA401Config::digitalInputs)
        .def_readwrite("digital_outputs", &CiA401Config::digitalOutputs)
        .def_readwrite("analog_inputs", &CiA401Config::analogInputs)
        .def_readwrite("analog_outputs", &CiA401Config::analogOutputs);

    // ==== CiA 401 I/O Module ====
    py::class_<CiA401IO>(m, "CiA401IO")
        .def(py::init<uint16_t>(), py::arg("slave_index"))
        .def("configure", &CiA401IO::configure,
             py::arg("config"),
             "Configure I/O module")
        .def("read_digital_inputs", &CiA401IO::readDigitalInputs,
             "Read all digital inputs as byte array")
        .def("read_digital_input", &CiA401IO::readDigitalInput,
             py::arg("channel"),
             "Read single digital input")
        .def("write_digital_outputs", &CiA401IO::writeDigitalOutputs,
             py::arg("values"),
             "Write all digital outputs")
        .def("set_digital_output", &CiA401IO::setDigitalOutput,
             py::arg("channel"), py::arg("value"),
             "Set single digital output")
        .def("read_analog_inputs", &CiA401IO::readAnalogInputs,
             "Read all analog inputs")
        .def("read_analog_input", &CiA401IO::readAnalogInput,
             py::arg("channel"),
             "Read single analog input")
        .def("write_analog_outputs", &CiA401IO::writeAnalogOutputs,
             py::arg("values"),
             "Write all analog outputs")
        .def("set_analog_output", &CiA401IO::setAnalogOutput,
             py::arg("channel"), py::arg("value"),
             "Set single analog output")
        .def("update", &CiA401IO::update,
             "Update I/O state (call in cyclic loop)");
}
