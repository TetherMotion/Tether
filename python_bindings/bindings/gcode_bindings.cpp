/**
 * @file gcode_bindings.cpp
 * @brief Python bindings for GCode parsing and trajectory generation
 *
 * Uses the full GCode namespace C++ API (GCodeParser, GCodeInterpreter, etc.)
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "tether/gcode/GCodeParser.hpp"
#include "tether/gcode/GCodeInterpreter.hpp"
#include "tether/gcode/GCodeTypes.hpp"
#include "tether/gcode/GCodeLexer.hpp"
#include "tether/gcode/motion/InterpolationStrategy.hpp"

#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <fstream>

namespace py = pybind11;

// Thread-local error tracking
static thread_local std::string g_lastError;
static thread_local int g_lastErrorCode = 0;

static const std::string& lastError() { return g_lastError; }
static int lastErrorCode() { return g_lastErrorCode; }
static void clearError() { g_lastError.clear(); g_lastErrorCode = 0; }
static void setError(const std::string& msg, int code = 1) { g_lastError = msg; g_lastErrorCode = code; }

// ---------------------------------------------------------------------------
// Wrapper class for Parser
// ---------------------------------------------------------------------------
class PyParser {
public:
    PyParser()
        : variables_()
        , lexer_(std::make_unique<GCode::Lexer>())
        , parser_(std::make_unique<GCode::Parser>(variables_))
    {
    }

    void configure(bool strict_modal, bool linux_cnc, bool fanuc, bool evaluate_expressions) {
        GCode::ParserConfig cfg;
        cfg.strictModalGroups = strict_modal;
        cfg.linuxCNCMode = linux_cnc;
        cfg.fanucMode = fanuc;
        cfg.evaluateExpressions = evaluate_expressions;
        // Reconfigure parser with new settings
        parser_ = std::make_unique<GCode::Parser>(variables_);
    }

    void parse_string(const std::string& gcode) {
        blocks_.clear();
        
        std::istringstream stream(gcode);
        std::string line;
        
        while (std::getline(stream, line)) {
            if (line.empty()) continue;
            
            GCode::Block block;
            GCode::Error err = parser_->parseLine(line.c_str(), block);
            if (err) {
                setError(std::string(err.message.data()), static_cast<int>(err.code));
                throw std::runtime_error("Parse error: " + std::string(err.message.data()));
            }
            
            blocks_.push_back(std::move(block));
        }
    }

    void parse_file(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            setError("Failed to open file: " + filename);
            throw std::runtime_error("Failed to open file: " + filename);
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        parse_string(buffer.str());
    }

    size_t block_count() const {
        return blocks_.size();
    }

    py::dict get_block(size_t index) {
        if (index >= blocks_.size()) {
            throw std::runtime_error("Block index out of range");
        }
        
        const auto& block = blocks_[index];
        
        py::dict result;
        result["line_number"] = block.lineNumber;
        result["source_line"] = block.sourceLineNumber;
        result["original_text"] = std::string(block.originalText.data());
        result["comment"] = std::string(block.comment.data());
        result["has_comment"] = block.hasComment;
        
        // Check for motion G-codes
        bool has_motion = block.hasMotion();
        int motion_mode = 1; // default linear
        for (uint8_t i = 0; i < block.gCodeCount; ++i) {
            int16_t gcode = block.gCodes[i];
            if (gcode >= 0 && gcode <= 3) {
                has_motion = true;
                motion_mode = gcode;
            }
        }
        result["has_motion"] = has_motion;
        result["motion_mode"] = motion_mode;
        
        // Feed rate
        double feed_rate = block.getWord(GCode::WordLetter::F, 0.0);
        result["feed_rate"] = feed_rate;
        
        // Axes
        py::list axes;
        for (size_t i = 0; i < GCode::MAX_AXES; ++i) {
            double val = 0.0;
            GCode::WordLetter letter;
            switch (i) {
                case 0: letter = GCode::WordLetter::X; break;
                case 1: letter = GCode::WordLetter::Y; break;
                case 2: letter = GCode::WordLetter::Z; break;
                case 3: letter = GCode::WordLetter::A; break;
                case 4: letter = GCode::WordLetter::B; break;
                case 5: letter = GCode::WordLetter::C; break;
                case 6: letter = GCode::WordLetter::U; break;
                case 7: letter = GCode::WordLetter::V; break;
                case 8: letter = GCode::WordLetter::W; break;
                default: letter = GCode::WordLetter::X; break;
            }
            if (block.hasWord(letter)) {
                val = block.getWord(letter);
            }
            axes.append(val);
        }
        result["axes"] = axes;
        
        // G-codes and M-codes
        py::list g_codes;
        for (uint8_t i = 0; i < block.gCodeCount; ++i) {
            g_codes.append(static_cast<int>(block.gCodes[i]));
        }
        result["g_codes"] = g_codes;
        
        py::list m_codes;
        for (uint8_t i = 0; i < block.mCodeCount; ++i) {
            m_codes.append(static_cast<int>(block.mCodes[i]));
        }
        result["m_codes"] = m_codes;
        
        // Arc parameters
        result["i"] = block.getWord(GCode::WordLetter::I, 0.0);
        result["j"] = block.getWord(GCode::WordLetter::J, 0.0);
        result["k"] = block.getWord(GCode::WordLetter::K, 0.0);
        result["r"] = block.getWord(GCode::WordLetter::R, 0.0);
        
        return result;
    }

    const std::vector<GCode::Block>& blocks() const { return blocks_; }

private:
    GCode::VariableSystem variables_;
    std::unique_ptr<GCode::Lexer> lexer_;
    std::unique_ptr<GCode::Parser> parser_;
    std::vector<GCode::Block> blocks_;
};

// ---------------------------------------------------------------------------
// Wrapper class for Interpreter (segment extraction)
// ---------------------------------------------------------------------------
class PyInterpreter {
public:
    PyInterpreter()
        : segments_()
        , currentPosition_()
        , defaultFeedRate_(1000.0)
        , rapidFeedRate_(6000.0)
    {
    }

    void configure(double max_vel, double max_accel, double max_jerk, double default_feed, double rapid_feed) {
        // Store limits for segment processing
        (void)max_vel;
        (void)max_accel;
        (void)max_jerk;
        defaultFeedRate_ = default_feed;
        rapidFeedRate_ = rapid_feed;
    }

    void set_position(const py::dict& pos) {
        if (pos.contains("X")) currentPosition_.coords[0] = pos["X"].cast<double>();
        if (pos.contains("Y")) currentPosition_.coords[1] = pos["Y"].cast<double>();
        if (pos.contains("Z")) currentPosition_.coords[2] = pos["Z"].cast<double>();
        if (pos.contains("A")) currentPosition_.coords[3] = pos["A"].cast<double>();
        if (pos.contains("B")) currentPosition_.coords[4] = pos["B"].cast<double>();
        if (pos.contains("C")) currentPosition_.coords[5] = pos["C"].cast<double>();
        if (pos.contains("U")) currentPosition_.coords[6] = pos["U"].cast<double>();
        if (pos.contains("V")) currentPosition_.coords[7] = pos["V"].cast<double>();
        if (pos.contains("W")) currentPosition_.coords[8] = pos["W"].cast<double>();
    }

    py::dict get_position() {
        py::dict result;
        result["X"] = currentPosition_.coords[0];
        result["Y"] = currentPosition_.coords[1];
        result["Z"] = currentPosition_.coords[2];
        result["A"] = currentPosition_.coords[3];
        result["B"] = currentPosition_.coords[4];
        result["C"] = currentPosition_.coords[5];
        result["U"] = currentPosition_.coords[6];
        result["V"] = currentPosition_.coords[7];
        result["W"] = currentPosition_.coords[8];
        return result;
    }

    void load_blocks(PyParser& parser) {
        segments_.clear();
        currentPosition_ = GCode::Position{};
        
        GCode::MotionMode activeMotionMode = GCode::MotionMode::LINEAR;
        double activeFeedRate = defaultFeedRate_;
        
        for (const auto& block : parser.blocks()) {
            // Update motion mode from G-codes
            for (uint8_t i = 0; i < block.gCodeCount; ++i) {
                int16_t gcode = block.gCodes[i];
                if (gcode == 0) activeMotionMode = GCode::MotionMode::RAPID;
                else if (gcode == 1) activeMotionMode = GCode::MotionMode::LINEAR;
                else if (gcode == 2) activeMotionMode = GCode::MotionMode::CW_ARC;
                else if (gcode == 3) activeMotionMode = GCode::MotionMode::CCW_ARC;
            }
            
            // Get feed rate if specified
            if (block.hasWord(GCode::WordLetter::F)) {
                activeFeedRate = block.getWord(GCode::WordLetter::F);
            }
            
            // Build target position from axis words
            GCode::Position targetPos = currentPosition_;
            bool hasAxisWords = false;
            
            if (block.hasWord(GCode::WordLetter::X)) { targetPos.coords[0] = block.getWord(GCode::WordLetter::X); hasAxisWords = true; }
            if (block.hasWord(GCode::WordLetter::Y)) { targetPos.coords[1] = block.getWord(GCode::WordLetter::Y); hasAxisWords = true; }
            if (block.hasWord(GCode::WordLetter::Z)) { targetPos.coords[2] = block.getWord(GCode::WordLetter::Z); hasAxisWords = true; }
            if (block.hasWord(GCode::WordLetter::A)) { targetPos.coords[3] = block.getWord(GCode::WordLetter::A); hasAxisWords = true; }
            if (block.hasWord(GCode::WordLetter::B)) { targetPos.coords[4] = block.getWord(GCode::WordLetter::B); hasAxisWords = true; }
            if (block.hasWord(GCode::WordLetter::C)) { targetPos.coords[5] = block.getWord(GCode::WordLetter::C); hasAxisWords = true; }
            
            if (hasAxisWords) {
                // Create a segment
                InternalSegment seg;
                seg.start = currentPosition_;
                seg.end = targetPos;
                
                double dx = seg.end.coords[0] - seg.start.coords[0];
                double dy = seg.end.coords[1] - seg.start.coords[1];
                double dz = seg.end.coords[2] - seg.start.coords[2];
                seg.segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                double effectiveFeed = (activeMotionMode == GCode::MotionMode::RAPID) ? rapidFeedRate_ : activeFeedRate;
                seg.feed_rate = effectiveFeed;
                seg.segment_time = (seg.segment_length > 0 && effectiveFeed > 0) ? (seg.segment_length / effectiveFeed * 60.0) : 0.0;
                
                seg.is_rapid = (activeMotionMode == GCode::MotionMode::RAPID);
                seg.motion_type = static_cast<uint8_t>(activeMotionMode);
                seg.block_index = static_cast<int32_t>(segments_.size());
                
                // Arc parameters
                if (block.hasWord(GCode::WordLetter::I)) seg.center.coords[0] = currentPosition_.coords[0] + block.getWord(GCode::WordLetter::I);
                if (block.hasWord(GCode::WordLetter::J)) seg.center.coords[1] = currentPosition_.coords[1] + block.getWord(GCode::WordLetter::J);
                if (block.hasWord(GCode::WordLetter::K)) seg.center.coords[2] = currentPosition_.coords[2] + block.getWord(GCode::WordLetter::K);
                
                segments_.push_back(seg);
                currentPosition_ = targetPos;
            }
        }
    }

    size_t segment_count() const {
        return segments_.size();
    }

    py::dict get_segment(size_t index) {
        if (index >= segments_.size()) {
            throw std::runtime_error("Segment index out of range");
        }
        
        const auto& seg = segments_[index];
        
        py::dict result;
        result["start"] = py::make_tuple(seg.start.coords[0], seg.start.coords[1], seg.start.coords[2]);
        result["end"] = py::make_tuple(seg.end.coords[0], seg.end.coords[1], seg.end.coords[2]);
        result["center"] = py::make_tuple(seg.center.coords[0], seg.center.coords[1], seg.center.coords[2]);
        result["feed_rate"] = seg.feed_rate;
        result["motion_type"] = seg.motion_type;
        result["segment_length"] = seg.segment_length;
        result["segment_time"] = seg.segment_time;
        result["is_rapid"] = seg.is_rapid;
        result["arc_radius"] = seg.arc_radius;
        result["arc_sweep"] = seg.arc_sweep;
        result["block_index"] = seg.block_index;
        result["plane"] = seg.plane;
        
        return result;
    }

    py::list get_all_segments() {
        py::list segments;
        for (size_t i = 0; i < segments_.size(); ++i) {
            segments.append(get_segment(i));
        }
        return segments;
    }

    // Internal segment structure for storage
    struct InternalSegment {
        GCode::Position start;
        GCode::Position end;
        GCode::Position center;
        double feed_rate = 0.0;
        double arc_radius = 0.0;
        double arc_sweep = 0.0;
        double segment_length = 0.0;
        double segment_time = 0.0;
        uint8_t motion_type = 1;
        int32_t block_index = -1;
        uint8_t plane = 0;
        bool is_rapid = false;
    };

    const std::vector<InternalSegment>& segments() const { return segments_; }

private:
    std::vector<InternalSegment> segments_;
    GCode::Position currentPosition_;
    double defaultFeedRate_;
    double rapidFeedRate_;
};

// ---------------------------------------------------------------------------
// Wrapper class for TrajectoryGenerator
// ---------------------------------------------------------------------------
class PyTrajectoryGenerator {
public:
    PyTrajectoryGenerator()
        : config_()
        , points_()
        , totalDuration_(0.0)
    {
        config_.timeResolution = 0.001;
        config_.maxChordDeviation = 0.01;
        config_.strategy = GCode::InterpolationStrategyType::FixedTime;
    }

    void configure(double time_step, double chord_deviation, bool use_s_curve, bool enable_jerk, double max_jerk) {
        config_.timeResolution = time_step;
        config_.maxChordDeviation = chord_deviation;
        config_.useSCurve = use_s_curve;
        config_.enableJerkLimit = enable_jerk;
        config_.limits.maxJerk = max_jerk;
    }

    void generate(PyInterpreter& interp) {
        points_.clear();
        totalDuration_ = 0.0;
        
        if (interp.segments().empty()) {
            return;
        }
        
        // Create strategy from factory
        auto strategy = GCode::InterpolationStrategyFactory::create(config_);
        if (!strategy) {
            setError("Failed to create interpolation strategy");
            throw std::runtime_error("Failed to create interpolation strategy");
        }
        
        GCode::InterpolationContext ctx;
        ctx.config = config_;
        
        // Build planning segments
        for (const auto& seg : interp.segments()) {
            GCode::PlanningSegment planSeg;
            planSeg.start = seg.start;
            planSeg.end = seg.end;
            planSeg.center = seg.center;
            planSeg.feedRate = seg.feed_rate;
            planSeg.segmentLength = seg.segment_length;
            planSeg.segmentTime = seg.segment_time;
            planSeg.isRapid = seg.is_rapid;
            planSeg.blockIndex = seg.block_index;
            
            switch (seg.motion_type) {
                case 0: planSeg.motionType = GCode::SegmentMotionType::Rapid; break;
                case 1: planSeg.motionType = GCode::SegmentMotionType::Linear; break;
                case 2: planSeg.motionType = GCode::SegmentMotionType::ArcCW; break;
                case 3: planSeg.motionType = GCode::SegmentMotionType::ArcCCW; break;
                default: planSeg.motionType = GCode::SegmentMotionType::Linear; break;
            }
            
            ctx.segments.push_back(planSeg);
        }
        
        std::vector<GCode::TrajectoryPoint> allPoints;
        GCode::InterpolationResult result = strategy->interpolateAll(ctx, allPoints);
        
        if (!result.success) {
            setError(result.errorMessage);
            throw std::runtime_error("Trajectory generation failed: " + result.errorMessage);
        }
        
        points_ = std::move(allPoints);
        totalDuration_ = result.totalDuration;
        minBounds_ = result.minBounds;
        maxBounds_ = result.maxBounds;
    }

    size_t point_count() const {
        return points_.size();
    }

    double duration() const {
        return totalDuration_;
    }

    py::dict get_point(size_t index) {
        if (index >= points_.size()) {
            throw std::runtime_error("Point index out of range");
        }
        
        const auto& pt = points_[index];
        
        py::dict result;
        result["time"] = pt.time;
        result["position"] = py::make_tuple(pt.position.coords[0], pt.position.coords[1], pt.position.coords[2]);
        result["velocity"] = py::make_tuple(pt.velocity.coords[0], pt.velocity.coords[1], pt.velocity.coords[2]);
        result["acceleration"] = py::make_tuple(pt.acceleration.coords[0], pt.acceleration.coords[1], pt.acceleration.coords[2]);
        result["motion_type"] = static_cast<uint8_t>(pt.motionType);
        result["block_index"] = pt.blockIndex;
        result["segment_index"] = pt.segmentIndex;
        result["is_interpolated"] = pt.isInterpolated;
        
        return result;
    }

    py::array_t<double> get_trajectory_array() {
        size_t count = points_.size();
        py::array_t<double> arr({static_cast<long>(count), 10l});
        auto r = arr.mutable_unchecked<2>();
        
        for (size_t i = 0; i < count; ++i) {
            const auto& pt = points_[i];
            r(i, 0) = pt.time;
            r(i, 1) = pt.position.coords[0];
            r(i, 2) = pt.position.coords[1];
            r(i, 3) = pt.position.coords[2];
            r(i, 4) = pt.velocity.coords[0];
            r(i, 5) = pt.velocity.coords[1];
            r(i, 6) = pt.velocity.coords[2];
            r(i, 7) = pt.acceleration.coords[0];
            r(i, 8) = pt.acceleration.coords[1];
            r(i, 9) = pt.acceleration.coords[2];
        }
        
        return arr;
    }

    py::tuple bounds() {
        auto min_tuple = py::make_tuple(minBounds_.coords[0], minBounds_.coords[1], minBounds_.coords[2]);
        auto max_tuple = py::make_tuple(maxBounds_.coords[0], maxBounds_.coords[1], maxBounds_.coords[2]);
        return py::make_tuple(min_tuple, max_tuple);
    }

private:
    GCode::InterpolationConfig config_;
    std::vector<GCode::TrajectoryPoint> points_;
    double totalDuration_;
    GCode::Position minBounds_;
    GCode::Position maxBounds_;
};

// ---------------------------------------------------------------------------
// Module definition
// ---------------------------------------------------------------------------
PYBIND11_MODULE(_gcode, m) {
    m.doc() = "Python bindings for GCode parsing and trajectory generation (C++ API)";

    // Error helpers
    m.def("last_error", &lastError, "Get last error message");
    m.def("last_error_code", &lastErrorCode, "Get last error code");
    m.def("clear_error", &clearError, "Clear error state");

    // Parser
    py::class_<PyParser>(m, "Parser")
        .def(py::init<>())
        .def("configure", &PyParser::configure,
             py::arg("strict_modal") = true,
             py::arg("linux_cnc") = true,
             py::arg("fanuc") = false,
             py::arg("evaluate_expressions") = true,
             "Configure parser options")
        .def("parse_string", &PyParser::parse_string, "Parse GCode from string")
        .def("parse_file", &PyParser::parse_file, "Parse GCode from file")
        .def("block_count", &PyParser::block_count, "Get number of parsed blocks")
        .def("get_block", &PyParser::get_block, "Get block info by index");

    // Interpreter
    py::class_<PyInterpreter>(m, "Interpreter")
        .def(py::init<>())
        .def("configure", &PyInterpreter::configure,
             py::arg("max_vel") = 6000.0,
             py::arg("max_accel") = 1000.0,
             py::arg("max_jerk") = 10000.0,
             py::arg("default_feed") = 1000.0,
             py::arg("rapid_feed") = 6000.0,
             "Configure machine limits")
        .def("set_position", &PyInterpreter::set_position, "Set current position")
        .def("get_position", &PyInterpreter::get_position, "Get current position")
        .def("load_blocks", &PyInterpreter::load_blocks, "Load blocks from parser")
        .def("segment_count", &PyInterpreter::segment_count, "Get segment count")
        .def("get_segment", &PyInterpreter::get_segment, "Get segment by index")
        .def("get_all_segments", &PyInterpreter::get_all_segments, "Get all segments as list");

    // TrajectoryGenerator
    py::class_<PyTrajectoryGenerator>(m, "TrajectoryGenerator")
        .def(py::init<>())
        .def("configure", &PyTrajectoryGenerator::configure,
             py::arg("time_step") = 0.001,
             py::arg("chord_deviation") = 0.01,
             py::arg("use_s_curve") = true,
             py::arg("enable_jerk") = true,
             py::arg("max_jerk") = 10000.0,
             "Configure trajectory generation")
        .def("generate", &PyTrajectoryGenerator::generate, "Generate trajectory from interpreter")
        .def("point_count", &PyTrajectoryGenerator::point_count, "Get number of points")
        .def("duration", &PyTrajectoryGenerator::duration, "Get total duration")
        .def("get_point", &PyTrajectoryGenerator::get_point, "Get point by index")
        .def("get_trajectory_array", &PyTrajectoryGenerator::get_trajectory_array,
             "Get full trajectory as numpy array (time, x, y, z, vx, vy, vz, ax, ay, az)")
        .def("bounds", &PyTrajectoryGenerator::bounds, "Get bounding box (min, max)");
}
