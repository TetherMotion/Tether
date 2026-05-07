/**
 * @file motion_control_bindings.cpp
 * @brief Python bindings for real-time motion control
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "tether/motion/MotionGenerator.hpp"

namespace py = pybind11;
using namespace Motion;

PYBIND11_MODULE(_motion_control, m) {
    m.doc() = "Python bindings for real-time motion control";

    // ==== Generator State Enum ====
    py::enum_<GeneratorState>(m, "GeneratorState")
        .value("Idle", GeneratorState::Idle)
        .value("Running", GeneratorState::Running)
        .value("Paused", GeneratorState::Paused)
        .value("Complete", GeneratorState::Complete);

    // ==== Profile Type Enum ====
    py::enum_<ProfileType>(m, "ProfileType")
        .value("Trapezoidal", ProfileType::Trapezoidal)
        .value("SCurve", ProfileType::SCurve)
        .value("Sine", ProfileType::Sine);

    // ==== Motion Generator Base ====
    py::class_<MotionGenerator>(m, "MotionGenerator")
        .def("start", &MotionGenerator::start, "Start motion generation")
        .def("stop", &MotionGenerator::stop, "Stop motion generation")
        .def("pause", &MotionGenerator::pause, "Pause motion generation")
        .def("resume", &MotionGenerator::resume, "Resume motion generation")
        .def("update", &MotionGenerator::update, py::arg("dt_ms"),
             "Update generator state")
        .def("get_position", &MotionGenerator::getPosition, "Get current position")
        .def("get_velocity", &MotionGenerator::getVelocity, "Get current velocity")
        .def("get_acceleration", &MotionGenerator::getAcceleration, "Get current acceleration")
        .def("get_torque", &MotionGenerator::getTorque, "Get feedforward torque")
        .def("get_state", &MotionGenerator::getState, "Get generator state")
        .def("is_running", &MotionGenerator::isRunning, "Check if generator is running")
        .def("is_complete", &MotionGenerator::isComplete, "Check if motion is complete");

    // ==== Sine Motion Generator ====
    py::class_<SineMotionGenerator, MotionGenerator>(m, "SineMotionGenerator")
        .def(py::init<>())
        .def("set_amplitude", &SineMotionGenerator::setAmplitude, py::arg("amplitude"),
             "Set sine wave amplitude")
        .def("set_frequency", &SineMotionGenerator::setFrequency, py::arg("frequency"),
             "Set sine wave frequency in Hz")
        .def("set_offset", &SineMotionGenerator::setOffset, py::arg("offset"),
             "Set position offset")
        .def("set_phase", &SineMotionGenerator::setPhase, py::arg("phase"),
             "Set initial phase in radians")
        .def("get_amplitude", &SineMotionGenerator::getAmplitude)
        .def("get_frequency", &SineMotionGenerator::getFrequency)
        .def("get_elapsed_time", &SineMotionGenerator::getElapsedTime);

    // ==== Trapezoidal Profile Generator ====
    py::class_<TrapezoidalProfile, MotionGenerator>(m, "TrapezoidalProfile")
        .def(py::init<>())
        .def("configure", &TrapezoidalProfile::configure,
             py::arg("max_velocity"), py::arg("acceleration"),
             "Configure trapezoidal profile limits")
        .def("set_target", &TrapezoidalProfile::setTarget,
             py::arg("target_position"),
             "Set target position")
        .def("get_target", &TrapezoidalProfile::getTarget)
        .def("get_distance_to_go", &TrapezoidalProfile::getDistanceToGo)
        .def("get_time_to_go", &TrapezoidalProfile::getTimeToGo);

    // ==== S-Curve Profile Generator ====
    py::class_<SCurveProfile, MotionGenerator>(m, "SCurveProfile")
        .def(py::init<>())
        .def("configure", &SCurveProfile::configure,
             py::arg("max_velocity"), py::arg("max_acceleration"), py::arg("max_jerk"),
             "Configure S-curve profile limits")
        .def("set_target", &SCurveProfile::setTarget,
             py::arg("target_position"),
             "Set target position")
        .def("get_jerk", &SCurveProfile::getJerk, "Get current jerk");

    // ==== Motor Model ====
    py::class_<MotorModel>(m, "MotorModel")
        .def(py::init<>())
        .def("configure", &MotorModel::configure,
             py::arg("inertia"), py::arg("friction"), py::arg("damping"),
             "Configure motor model parameters")
        .def("simulate_step", &MotorModel::simulateStep,
             py::arg("torque"), py::arg("dt"),
             "Simulate one time step")
        .def("get_position", &MotorModel::getPosition)
        .def("get_velocity", &MotorModel::getVelocity)
        .def("reset", &MotorModel::reset, "Reset model state");
}
