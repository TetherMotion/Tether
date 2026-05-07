/**
 * @file export_bindings.cpp
 * @brief Python bindings for SVG and CSV export
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <vector>
#include <sstream> // For exportToString
#include "tether/export/SVGExporter.hpp"
#include "tether/export/CSVExporter.hpp"
#include "tether/export/TrajectoryAnalyzer.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"
#include "tether/gcode/GCodeTypes.hpp"

namespace py = pybind11;
using namespace GCodeExport;
using namespace GCode;

PYBIND11_MODULE(pyexport, m) {
    m.doc() = "Python bindings for SVG and CSV export";
    
    // SVG Config
    py::class_<SVGConfig>(m, "SVGConfig")
        .def(py::init<>())
        .def_readwrite("width", &SVGConfig::width)
        .def_readwrite("height", &SVGConfig::height)
        .def_readwrite("margin", &SVGConfig::margin)
        .def_readwrite("auto_scale", &SVGConfig::autoScale)
        .def_readwrite("primary_axis1", &SVGConfig::primaryAxis1)
        .def_readwrite("primary_axis2", &SVGConfig::primaryAxis2)
        .def_readwrite("flip_y", &SVGConfig::flipY)
        .def_readwrite("show_rapids", &SVGConfig::showRapids)
        .def_readwrite("show_grid", &SVGConfig::showGrid)
        .def_readwrite("grid_spacing", &SVGConfig::gridSpacing)
        .def_readwrite("show_direction_arrows", &SVGConfig::showDirectionArrows)
        .def_readwrite("color_by_velocity", &SVGConfig::colorByVelocity)
        .def_readwrite("color_by_acceleration", &SVGConfig::colorByAcceleration);
    
    // SVG Exporter
    py::class_<SVGExporter>(m, "SVGExporter")
        .def(py::init<const SVGConfig&>(), py::arg("config") = SVGConfig())
        .def("configure", &SVGExporter::configure, "Set SVG configuration")
        .def("export_to_file", 
             [](SVGExporter& self, const std::string& filename, 
                const std::vector<TrajectorySample>& trajectory) {
                 return self.exportToFile(trajectory, filename);
             },
             "Export trajectory to SVG file",
             py::arg("filename"), py::arg("trajectory"))
        .def("export_to_string", 
             [](SVGExporter& self, const std::vector<TrajectorySample>& trajectory) {
                 std::ostringstream oss;
                 self.exportToStream(trajectory, oss);
                 return oss.str();
             },
             "Export trajectory to SVG string",
             py::arg("trajectory"))
        .def("export_segments",
            [](SVGExporter& self, const std::string& filename, 
               const std::vector<PlanningSegment>& segments) {
                return self.exportSegments(segments, filename);
            },
            "Export segments directly to SVG file",
            py::arg("filename"), py::arg("segments"));

    // CSV Exporter
    py::class_<CSVExporter>(m, "CSVExporter")
        .def(py::init<>())
        .def("export_to_file", 
             [](CSVExporter& self, const std::string& filename, 
                const std::vector<TrajectorySample>& trajectory) {
                 return self.exportToFile(trajectory, filename);
             },
             "Export trajectory to CSV file",
             py::arg("filename"), py::arg("trajectory"));
             
    // TrajectoryPoint Binding
    py::class_<TrajectoryPoint>(m, "TrajectoryPoint")
        .def(py::init<>())
        .def_readwrite("time", &TrajectoryPoint::time)
        .def_readwrite("position", &TrajectoryPoint::position)
        .def_readwrite("velocity", &TrajectoryPoint::velocity)
        .def_readwrite("acceleration", &TrajectoryPoint::acceleration)
        .def_readwrite("jerk", &TrajectoryPoint::jerk)
        .def_readwrite("motion_type", &TrajectoryPoint::motionType)
        .def_readwrite("block_index", &TrajectoryPoint::blockIndex);

    // Position Binding
    py::class_<Position>(m, "Position")
        .def(py::init<>())
        .def(py::init([](double x, double y, double z) {
             Position p; p.x() = x; p.y() = y; p.z() = z; return p;
        }))
        .def_property("x", [](Position& p){ return p.x(); }, [](Position& p, double v){ p.x() = v; })
        .def_property("y", [](Position& p){ return p.y(); }, [](Position& p, double v){ p.y() = v; })
        .def_property("z", [](Position& p){ return p.z(); }, [](Position& p, double v){ p.z() = v; })
        .def("norm", &Position::magnitude, "Get magnitude")
        .def("__repr__", [](const Position& p) {
            return "<Position x=" + std::to_string(p.x()) + 
                   " y=" + std::to_string(p.y()) + 
                   " z=" + std::to_string(p.z()) + ">";
        });

    // Trajectory Statistics
    py::class_<TrajectoryStatistics>(m, "TrajectoryStatistics")
        .def(py::init<>())
        .def_readwrite("duration", &TrajectoryStatistics::duration)
        .def_readwrite("path_length", &TrajectoryStatistics::pathLength)
        .def_readwrite("max_velocity", &TrajectoryStatistics::maxLinearVelocity)
        .def_readwrite("max_acceleration", &TrajectoryStatistics::maxLinearAcceleration)
        .def_readwrite("max_jerk", &TrajectoryStatistics::maxLinearJerk)
        // Nested struct AxisStats binding if needed
        ;

    // Analysis Config (must be before TrajectoryAnalyzer)
    py::class_<AnalysisConfig>(m, "AnalysisConfig")
        .def(py::init<>())
        .def_readwrite("time_step", &AnalysisConfig::timeStep)
        .def_readwrite("max_chord_error", &AnalysisConfig::maxChordError)
        .def_readwrite("derivative_order", &AnalysisConfig::derivativeOrder);

    // Trajectory Analyzer
    py::class_<TrajectoryAnalyzer>(m, "TrajectoryAnalyzer")
        .def(py::init<const AnalysisConfig&>(), py::arg("config") = AnalysisConfig())
        .def("analyze", 
             [](TrajectoryAnalyzer& self, const std::vector<PlanningSegment>& segments) {
                 return self.analyze(segments);
             },
             "Analyze motion segments",
             py::arg("segments"));

    // TrajectorySample Binding
    py::class_<TrajectorySample>(m, "TrajectorySample")
        .def(py::init<>())
        .def_readwrite("time", &TrajectorySample::time)
        .def_readwrite("path_position", &TrajectorySample::pathPosition)
        .def_property("position", 
            [](const TrajectorySample& s) { 
                return std::vector<double>(s.position.begin(), s.position.end()); 
            }, nullptr)
        .def_property("velocity", 
            [](const TrajectorySample& s) { 
                return std::vector<double>(s.velocity.begin(), s.velocity.end()); 
            }, nullptr)
        .def_property("acceleration", 
            [](const TrajectorySample& s) { 
                return std::vector<double>(s.acceleration.begin(), s.acceleration.end()); 
            }, nullptr)
        .def_property("jerk", 
            [](const TrajectorySample& s) { 
                return std::vector<double>(s.jerk.begin(), s.jerk.end()); 
            }, nullptr);
}
