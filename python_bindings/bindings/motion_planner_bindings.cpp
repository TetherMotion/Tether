/**
 * @file motion_planner_bindings.cpp
 * @brief Python bindings for Motion Replanner, Machine Tester, etc.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "tether/motion_replanner/MotionReplanner.hpp"
#include "tether/motion_replanner/MachineTester.hpp"
#include "tether/motion_replanner/SystemIdentifier.hpp"
#include "tether/motion_replanner/PerformanceHeatmap.hpp"
#include "tether/motion_replanner/GCodeGenerator.hpp"
#include "tether/motion_replanner/TestDataExporter.hpp"

namespace py = pybind11;
using namespace MotionReplanner;

PYBIND11_MODULE(_motion_planner, m) {
    m.doc() = "Python bindings for Motion Replanner and Machine Testing";

    // ==== Enums ====
    py::enum_<SingleAxisTestType>(m, "SingleAxisTestType")
        .value("Ramp", SingleAxisTestType::Ramp)
        .value("Sinusoid", SingleAxisTestType::Sinusoid)
        .value("SCurve", SingleAxisTestType::SCurve)
        .value("Step", SingleAxisTestType::Step)
        .value("Triangular", SingleAxisTestType::Triangular)
        .value("Trapezoidal", SingleAxisTestType::Trapezoidal);

    py::enum_<MultiAxisTestType>(m, "MultiAxisTestType")
        .value("Circle", MultiAxisTestType::Circle)
        .value("Ellipse", MultiAxisTestType::Ellipse)
        .value("Helix", MultiAxisTestType::Helix)
        .value("Lissajous", MultiAxisTestType::Lissajous)
        .value("Square", MultiAxisTestType::Square)
        .value("RoundedSquare", MultiAxisTestType::RoundedSquare)
        .value("DiagonalBox", MultiAxisTestType::DiagonalBox);

    // ==== Test Configurations ====
    py::class_<SingleAxisTestConfig>(m, "SingleAxisTestConfig")
        .def(py::init<>())
        .def_readwrite("axis", &SingleAxisTestConfig::axis)
        .def_readwrite("type", &SingleAxisTestConfig::type)
        .def_readwrite("amplitude", &SingleAxisTestConfig::amplitude)
        .def_readwrite("frequency", &SingleAxisTestConfig::frequency)
        .def_readwrite("velocity", &SingleAxisTestConfig::velocity)
        .def_readwrite("acceleration", &SingleAxisTestConfig::acceleration)
        .def_readwrite("jerk", &SingleAxisTestConfig::jerk)
        .def_readwrite("duration", &SingleAxisTestConfig::duration)
        .def_readwrite("cycles", &SingleAxisTestConfig::cycles)
        .def_readwrite("center_position", &SingleAxisTestConfig::centerPosition);

    py::class_<MultiAxisTestConfig>(m, "MultiAxisTestConfig")
        .def(py::init<>())
        .def_readwrite("type", &MultiAxisTestConfig::type)
        .def_readwrite("u_axis", &MultiAxisTestConfig::uAxis)
        .def_readwrite("v_axis", &MultiAxisTestConfig::vAxis);

    // ==== Machine Tester ====
    py::class_<MachineTester>(m, "MachineTester")
        .def(py::init<>())
        .def("run_single_axis_test", &MachineTester::runSingleAxisTest,
             py::arg("config"),
             "Run a single-axis test with the given configuration")
        .def("run_multi_axis_test", &MachineTester::runMultiAxisTest,
             py::arg("config"),
             "Run a multi-axis test with the given configuration");

    // ==== System Identifier ====
    py::class_<SystemIdentifier>(m, "SystemIdentifier")
        .def(py::init<>())
        .def("identify", &SystemIdentifier::identify,
             "Run system identification");

    // ==== Performance Heatmap ====
    py::class_<PerformanceHeatmap>(m, "PerformanceHeatmap")
        .def(py::init<>())
        .def("add_result", &PerformanceHeatmap::addResult,
             "Add a test result to the heatmap")
        .def("export_svg", &PerformanceHeatmap::exportSVG,
             py::arg("filename"),
             "Export heatmap to SVG file");

    // ==== G-code Generator ====
    py::class_<GCodeGenerator>(m, "GCodeGenerator")
        .def(py::init<>())
        .def("generate_test_pattern", &GCodeGenerator::generateTestPattern,
             py::arg("pattern_type"),
             "Generate G-code for a test pattern");
}
