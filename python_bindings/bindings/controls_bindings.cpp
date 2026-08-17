/**
 * @file controls_bindings.cpp
 * @brief Python bindings for PID and control algorithms
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "tether/control/ControllerBase.hpp"
#include "tether/control/PIDControllers.hpp"
#include "tether/control/StateSpaceControllers.hpp"
#include "tether/control/FractionalPID.hpp"
#include "tether/control/RobustControllers.hpp"
#include "tether/control/LearningControllers.hpp"
#include "tether/control/CompositeControllers.hpp"

namespace py = pybind11;
using namespace tether::control;

PYBIND11_MODULE(_controls, m) {
    m.doc() = "Python bindings for PID and control algorithms";

    // ==== Anti-Windup Enum ====
    py::enum_<AntiWindupMethod>(m, "AntiWindupMethod")
        .value("None", AntiWindupMethod::None)
        .value("Clamping", AntiWindupMethod::Clamping)
        .value("BackCalculation", AntiWindupMethod::BackCalculation)
        .value("ConditionalIntegration", AntiWindupMethod::ConditionalIntegration)
        .value("IntegratorLimiting", AntiWindupMethod::IntegratorLimiting);

    // ==== PID Config ====
    py::class_<PIDConfig>(m, "PIDConfig")
        .def(py::init<>())
        .def_readwrite("kp", &PIDConfig::kp)
        .def_readwrite("ki", &PIDConfig::ki)
        .def_readwrite("kd", &PIDConfig::kd)
        .def_readwrite("output_min", &PIDConfig::outputMin)
        .def_readwrite("output_max", &PIDConfig::outputMax)
        .def_readwrite("anti_windup", &PIDConfig::antiWindup)
        .def_readwrite("derivative_filter_tau", &PIDConfig::derivativeFilterTau)
        .def_readwrite("setpoint_weight_b", &PIDConfig::setpointWeightB)
        .def_readwrite("setpoint_weight_c", &PIDConfig::setpointWeightC);

    // ==== PID Controller ====
    py::class_<PIDController>(m, "PIDController")
        .def(py::init<>())
        .def(py::init<const PIDConfig&>(), py::arg("config"))
        .def("configure", &PIDController::configure, py::arg("config"))
        .def("update", &PIDController::update,
             py::arg("setpoint"), py::arg("measured"), py::arg("dt"),
             "Compute PID output")
        .def("reset", &PIDController::reset, "Reset controller state")
        .def("get_proportional", &PIDController::getProportional)
        .def("get_integral", &PIDController::getIntegral)
        .def("get_derivative", &PIDController::getDerivative)
        .def("get_output", &PIDController::getOutput)
        .def("set_gains", &PIDController::setGains,
             py::arg("kp"), py::arg("ki"), py::arg("kd"));

    // ==== P Controller ====
    py::class_<PController>(m, "PController")
        .def(py::init<double, double, double>(),
             py::arg("kp"), py::arg("output_min") = -1e9, py::arg("output_max") = 1e9)
        .def("update", &PController::update,
             py::arg("setpoint"), py::arg("measured"), py::arg("dt"))
        .def("reset", &PController::reset)
        .def("set_gain", &PController::setGain, py::arg("kp"));

    // ==== PD Controller ====
    py::class_<PDController>(m, "PDController")
        .def(py::init<double, double, double, double, double>(),
             py::arg("kp"), py::arg("kd"),
             py::arg("filter_tau") = 0.01,
             py::arg("output_min") = -1e9, py::arg("output_max") = 1e9)
        .def("update", &PDController::update,
             py::arg("setpoint"), py::arg("measured"), py::arg("dt"))
        .def("reset", &PDController::reset)
        .def("set_gains", &PDController::setGains, py::arg("kp"), py::arg("kd"));

    // ==== PI Controller ====
    py::class_<PIController>(m, "PIController")
        .def(py::init<double, double, double, double>(),
             py::arg("kp"), py::arg("ki"),
             py::arg("output_min") = -1e9, py::arg("output_max") = 1e9)
        .def("update", &PIController::update,
             py::arg("setpoint"), py::arg("measured"), py::arg("dt"))
        .def("reset", &PIController::reset)
        .def("set_gains", &PIController::setGains, py::arg("kp"), py::arg("ki"));

    // ==== LQR Controller ====
    py::class_<LQRController>(m, "LQRController")
        .def(py::init<>())
        .def("configure", &LQRController::configure,
             "Configure LQR with state-space matrices")
        .def("compute", &LQRController::compute,
             py::arg("state"),
             "Compute control input from state");

    // ==== Kalman Filter ====
    py::class_<KalmanFilter>(m, "KalmanFilter")
        .def(py::init<>())
        .def("configure", &KalmanFilter::configure,
             "Configure Kalman filter matrices")
        .def("predict", &KalmanFilter::predict,
             py::arg("u"), py::arg("dt"),
             "Prediction step")
        .def("update", &KalmanFilter::update,
             py::arg("z"),
             "Update step with measurement")
        .def("get_state", &KalmanFilter::getState,
             "Get current state estimate");

    // ==== FOPID (Fractional Order PID) ====
    py::class_<FOPIDController>(m, "FOPIDController")
        .def(py::init<>())
        .def("configure", &FOPIDController::configure,
             py::arg("kp"), py::arg("ki"), py::arg("kd"),
             py::arg("lambda_i"), py::arg("mu_d"),
             "Configure fractional PID (lambda=integral order, mu=derivative order)")
        .def("update", &FOPIDController::update,
             py::arg("setpoint"), py::arg("measured"), py::arg("dt"))
        .def("reset", &FOPIDController::reset);

    // ==== Iterative Learning Controller ====
    py::class_<IterativeLearningController>(m, "IterativeLearningController")
        .def(py::init<>())
        .def("configure", &IterativeLearningController::configure,
             "Configure ILC parameters")
        .def("start_trial", &IterativeLearningController::startTrial,
             "Start a new learning trial")
        .def("end_trial", &IterativeLearningController::endTrial,
             "End current trial and update learning")
        .def("update", &IterativeLearningController::update,
             py::arg("setpoint"), py::arg("measured"), py::arg("dt"),
             "Get feedforward signal for current position");
}
