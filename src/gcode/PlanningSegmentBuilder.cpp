/**
 * @file PlanningSegmentBuilder.cpp
 * @brief Convert G-code text into PlanningSegments using the Interpreter
 */

#include "tether/gcode/PlanningSegmentBuilder.hpp"
#include "tether/gcode/GCodeInterpreter.hpp"

#include <cmath>
#include <sstream>
#include <unordered_map>

namespace GCode {

// ── Type mapping ─────────────────────────────────────────────────────────────

SegmentMotionType PlanningSegmentBuilder::mapMotionType(MotionSegment::Type type) {
    switch (type) {
        case MotionSegment::Type::RAPID:       return SegmentMotionType::Rapid;
        case MotionSegment::Type::LINEAR:      return SegmentMotionType::Linear;
        case MotionSegment::Type::ARC_CW:      return SegmentMotionType::ArcCW;
        case MotionSegment::Type::ARC_CCW:     return SegmentMotionType::ArcCCW;
        case MotionSegment::Type::SPLINE:      return SegmentMotionType::CubicSpline;
        case MotionSegment::Type::NURBS:       return SegmentMotionType::NURBS;
        case MotionSegment::Type::DWELL:       return SegmentMotionType::Dwell;
        case MotionSegment::Type::PROBE:       return SegmentMotionType::Linear;
        default:                               return SegmentMotionType::Linear;
    }
}

InterpolationPlane PlanningSegmentBuilder::mapPlane(Plane plane) {
    switch (plane) {
        case Plane::ZX: return InterpolationPlane::XZ;
        case Plane::YZ: return InterpolationPlane::YZ;
        case Plane::XY:
        default:        return InterpolationPlane::XY;
    }
}

// ── Main entry point ─────────────────────────────────────────────────────────

PlanningSegmentResult PlanningSegmentBuilder::fromText(
    const std::string& gcodeText)
{
    return fromText(gcodeText, InterpreterConfig{});
}

PlanningSegmentResult PlanningSegmentBuilder::fromText(
    const std::string& gcodeText,
    const InterpreterConfig& config)
{
    PlanningSegmentResult result;

    // Set up interpreter with arc-segment emission enabled
    Interpreter interpreter(config);
    interpreter.setEmitArcSegments(true);

    // Collect MotionSegments via callback
    std::vector<MotionSegment> motionSegments;
    interpreter.setMotionCallback([&motionSegments](const MotionSegment& seg) {
        motionSegments.push_back(seg);
        return Error{};
    });

    // Load and execute
    Error loadErr = interpreter.loadString(gcodeText);
    if (!loadErr.ok()) {
        result.error = loadErr;
        return result;
    }

    // Capture initial position BEFORE running
    Position currentPos = interpreter.getCurrentPosition();

    Error runErr = interpreter.run();
    if (!runErr.ok()) {
        result.error = runErr;
        return result;
    }

    // Also build block metadata by re-parsing the text line by line.
    // The Interpreter doesn't expose per-block metadata directly, so we
    // track line numbers and g-code text from the source.
    {
        std::istringstream stream(gcodeText);
        std::string line;
        int32_t blockIndex = 0;
        int lineNumber = 0;
        while (std::getline(stream, line)) {
            ++lineNumber;

            // Strip comment
            std::string stripped;
            bool inParen = false;
            for (char c : line) {
                if (c == '(') { inParen = true; continue; }
                if (c == ')') { inParen = false; continue; }
                if (c == ';' || c == '%') break;
                if (!inParen) stripped += c;
            }

            // Skip blank/comment-only lines
            bool hasContent = false;
            for (char c : stripped) {
                if (!std::isspace(static_cast<unsigned char>(c))) {
                    hasContent = true;
                    break;
                }
            }
            if (!hasContent) continue;

            // Determine motion type from the first G-code word
            uint8_t motionType = 255; // non-motion default
            for (size_t i = 0; i < stripped.size(); ++i) {
                char c = std::toupper(static_cast<unsigned char>(stripped[i]));
                if (c == 'G') {
                    // Parse the number after G
                    size_t j = i + 1;
                    while (j < stripped.size() && (std::isdigit(static_cast<unsigned char>(stripped[j])) ||
                           stripped[j] == '.')) ++j;
                    if (j > i + 1) {
                        double gval = std::atof(stripped.c_str() + i + 1);
                        int gvalInt = static_cast<int>(gval);
                        if (gvalInt == 0) motionType = 0;
                        else if (gvalInt == 1) motionType = 1;
                        else if (gvalInt == 2) motionType = 2;
                        else if (gvalInt == 3) motionType = 3;
                    }
                    break; // Only check first G-code
                }
            }

            BlockMetadata blk;
            blk.blockIndex = blockIndex++;
            blk.lineNumber = lineNumber;
            blk.motionType = motionType;
            // Trim leading whitespace from the stripped text
            size_t start = stripped.find_first_not_of(" \t");
            if (start != std::string::npos)
                blk.gcodeText = stripped.substr(start);
            result.blocks.push_back(std::move(blk));
        }
    }

    // Convert MotionSegments → PlanningSegments
    // MotionSegments don't carry start position; track from initial position.
    // currentPos was captured before run() above.

    // E-axis tracking: parse the G-code text to extract E values per line.
    // The Interpreter doesn't track E in MotionSegment, so we re-parse
    // to find E words and match them to segments by line number.
    // Modal state for E tracking:
    bool absoluteExtrude = true;  // M82 (absolute E), vs M83 (relative E)
    double currentE = 0.0;        // current extruder position (mm)
    double unitScaleE = 1.0;      // E axis doesn't use unit scale (always mm)

    // Build a map: lineNumber → E value (absolute or relative as per modal)
    std::unordered_map<int, double> lineToEdelta;
    {
        std::istringstream stream(gcodeText);
        std::string line;
        int lineNumber = 0;
        bool localAbsoluteExtrude = true;
        double localCurrentE = 0.0;
        bool localUnitsMm = true;
        while (std::getline(stream, line)) {
            ++lineNumber;

            // Strip comment
            std::string stripped;
            bool inParen = false;
            for (char c : line) {
                if (c == '(') { inParen = true; continue; }
                if (c == ')') { inParen = false; continue; }
                if (c == ';' || c == '%') break;
                if (!inParen) stripped += c;
            }

            // Check for M82/M83 (extruder mode)
            if (stripped.find("M82") != std::string::npos ||
                stripped.find("m82") != std::string::npos) {
                localAbsoluteExtrude = true;
            }
            if (stripped.find("M83") != std::string::npos ||
                stripped.find("m83") != std::string::npos) {
                localAbsoluteExtrude = false;
            }

            // Check for G20/G21 (units)
            if (stripped.find("G20") != std::string::npos ||
                stripped.find("g20") != std::string::npos) {
                localUnitsMm = false;
            }
            if (stripped.find("G21") != std::string::npos ||
                stripped.find("g21") != std::string::npos) {
                localUnitsMm = true;
            }

            // Check for G92 E (reset E)
            if (stripped.find("G92") != std::string::npos ||
                stripped.find("g92") != std::string::npos) {
                // Find E word
                for (size_t i = 0; i < stripped.size(); ++i) {
                    char c = std::toupper(static_cast<unsigned char>(stripped[i]));
                    if (c == 'E') {
                        size_t j = i + 1;
                        while (j < stripped.size() && (std::isdigit(static_cast<unsigned char>(stripped[j])) ||
                               stripped[j] == '.' || stripped[j] == '-' || stripped[j] == '+')) ++j;
                        if (j > i + 1) {
                            localCurrentE = std::atof(stripped.c_str() + i + 1);
                        }
                        break;
                    }
                }
            }

            // Find E word for motion lines (G0/G1/G2/G3)
            bool isMotion = false;
            for (size_t i = 0; i < stripped.size(); ++i) {
                char c = std::toupper(static_cast<unsigned char>(stripped[i]));
                if (c == 'G') {
                    size_t j = i + 1;
                    while (j < stripped.size() && (std::isdigit(static_cast<unsigned char>(stripped[j])) ||
                           stripped[j] == '.')) ++j;
                    if (j > i + 1) {
                        int gval = static_cast<int>(std::atof(stripped.c_str() + i + 1));
                        if (gval >= 0 && gval <= 3) isMotion = true;
                    }
                    break;
                }
            }

            if (isMotion) {
                for (size_t i = 0; i < stripped.size(); ++i) {
                    char c = std::toupper(static_cast<unsigned char>(stripped[i]));
                    if (c == 'E') {
                        size_t j = i + 1;
                        while (j < stripped.size() && (std::isdigit(static_cast<unsigned char>(stripped[j])) ||
                               stripped[j] == '.' || stripped[j] == '-' || stripped[j] == '+')) ++j;
                        if (j > i + 1) {
                            double eVal = std::atof(stripped.c_str() + i + 1);
                            double eDelta = 0.0;
                            if (localAbsoluteExtrude) {
                                eDelta = eVal - localCurrentE;
                            } else {
                                eDelta = eVal;
                            }
                            localCurrentE += eDelta;
                            lineToEdelta[lineNumber] = eDelta;
                        }
                        break;
                    }
                }
            }
        }
    }

    int32_t blockIdx = 0;
    for (const auto& mseg : motionSegments) {
        PlanningSegment pseg;
        pseg.start = currentPos;
        pseg.end = mseg.endPosition;
        pseg.motionType = mapMotionType(mseg.type);
        pseg.feedRate = mseg.feedRate;
        pseg.isRapid = (mseg.type == MotionSegment::Type::RAPID);
        pseg.blockIndex = blockIdx++;

        // Arc geometry
        if (mseg.type == MotionSegment::Type::ARC_CW ||
            mseg.type == MotionSegment::Type::ARC_CCW) {
            // ArcParams has absolute center; PlanningSegment.center is also absolute
            pseg.center = mseg.arc.center;
            pseg.arcRadius = mseg.arc.radius;
            pseg.arcSweep = mseg.arc.sweepAngle;
            pseg.plane = mapPlane(mseg.arc.plane);

            // Segment length = |sweep| * radius
            if (pseg.arcRadius > 1e-9) {
                pseg.segmentLength = std::abs(pseg.arcSweep) * pseg.arcRadius;
            }
        } else {
            // Linear / rapid
            pseg.segmentLength = currentPos.linearDistance(mseg.endPosition);
        }

        // Blend tolerance from interpreter modal state.
        pseg.blendTolerance = interpreter.getMachineState().blendTolerance;

        // E-axis delta: look up by source line number.
        // The MotionSegment.lineNumber is 1-based (sourceLineNumber).
        // Store E delta in exitVelocity (repurposed for visualization).
        auto eIt = lineToEdelta.find(mseg.lineNumber);
        if (eIt != lineToEdelta.end()) {
            pseg.exitVelocity = eIt->second;  // E delta (mm)
        } else {
            pseg.exitVelocity = 0.0;
        }

        // Compute segment time from feed rate
        pseg.computeTimeFromFeedRate();

        result.segments.push_back(std::move(pseg));
        currentPos = mseg.endPosition;
    }

    return result;
}

} // namespace GCode
