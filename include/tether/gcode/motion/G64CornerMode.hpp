/**
 * @file G64CornerMode.hpp
 * @brief Extended G64 Path Blending with Corner Mode Options
 *
 * @details
 * This extension to the G64 path blending system provides fine-grained control
 * over corner behavior during path blending. It supports:
 *
 * - **Inside Corner Mode**: Path stays completely inside the theoretical sharp corner
 *   (material is guaranteed to be cut)
 * - **Outside Corner Mode**: Path stays completely outside the theoretical corner
 *   (tool never cuts beyond programmed profile)
 * - **Approximate Inside**: Path stays approximately inside with tolerance
 * - **Approximate Outside**: Path stays approximately outside with tolerance
 * - **Centered**: Traditional G64 behavior (path may be inside or outside)
 *
 * Extended G64 Syntax:
 * @code
 *   G64 P<tolerance> I<inside_tol> O<outside_tol>
 * @endcode
 *
 * - P: Traditional path tolerance (maximum deviation from programmed path)
 * - I: Inside corner tolerance (0 = stay strictly inside, >0 = max inside overshoot)
 * - O: Outside corner tolerance (0 = stay strictly outside, >0 = max outside overshoot)
 *
 * If both I and O are specified, the system tries to balance between them.
 *
 * @author G-Code Export Tool
 * @version 1.0
 */

#pragma once

#include "InterpolationStrategy.hpp"
#include "BlendCoreGCodeAdapter.hpp"
#include <cmath>
#include <algorithm>
#include <string>

namespace GCode {

// ============================================================================
// Corner Mode Enumeration
// ============================================================================

/**
 * @brief Corner blending mode for G64 path control
 */
enum class G64CornerMode : uint8_t {
    Centered = 0,           ///< Traditional G64 - blend may be inside or outside
    InsideStrict = 1,       ///< Stay strictly inside corner (I0)
    InsideApproximate = 2,  ///< Stay approximately inside (I with tolerance)
    OutsideStrict = 3,      ///< Stay strictly outside corner (O0)
    OutsideApproximate = 4, ///< Stay approximately outside (O with tolerance)
    Balanced = 5,           ///< Balance inside and outside constraints
    Teardrop = 6,           ///< Teardrop strategy - continue past corner and make (360° - corner angle) turn
    ExactStop = 7,          ///< Exact stop mode - no blending
};

/**
 * @brief Corner geometry classification
 */
enum class CornerType : uint8_t {
    Convex = 0,             ///< Outside corner (turn < 180°, e.g., turning left on exterior)
    Concave = 1,            ///< Inside corner (turn > 180°, e.g., pocket corner)
    Straight = 2,           ///< No corner (segments collinear)
    Cusp = 3,               ///< Sharp reversal (180° turn)
};

// ============================================================================
// G64 Corner Configuration
// ============================================================================

/**
 * @brief Extended G64 configuration with corner mode options
 */
struct G64CornerConfig {
    // Traditional G64 parameters
    double pathTolerance = 0.05;            ///< G64 P value (mm) - max path deviation
    double naiveCamTolerance = 0.0;         ///< G64 Q value (mm) - naive cam

    // Extended corner mode parameters
    G64CornerMode cornerMode = G64CornerMode::Centered;
    double insideTolerance = 0.0;           ///< I parameter - inside tolerance (mm)
    double outsideTolerance = 0.0;          ///< O parameter - outside tolerance (mm)
    
    // Strategy string (e.g., "inside", "dogbone", "teardrop", "exact-stop")
    std::string strategyString;             ///< Optional strategy parameter
    
    // Tangential margin for outside/inside modes
    // When > 0, allows blend curve to come within this distance of corner point
    // When = 0, blend must be strictly tangent to corner point
    double tangentialMargin = 0.0;          ///< T parameter - tangential margin (mm)

    // Corner detection parameters
    double minCornerAngle = 5.0;            ///< Minimum angle to treat as corner (degrees)
    double maxBlendAngle = 175.0;           ///< Maximum angle for blending (degrees)

    // Blend curve parameters
    double blendRadiusFactor = 1.0;         ///< Multiplier for blend radius
    bool useBezierBlend = true;             ///< Use Bézier curves for smooth blends
    int bezierOrder = 5;                    ///< Bézier curve order (2=quadratic, 3=cubic, 5=quintic C2)
    double maxBlendFraction = 0.5;          ///< Max fraction of segment length consumed by blend

    // Velocity handling at corners
    bool reduceVelocityAtCorners = true;    ///< Slow down at sharp corners
    double cornerVelocityFactor = 0.8;      ///< Velocity reduction factor at corners
    
    // Kinematic limits for velocity planning
    double maxVelocity = 100.0;             ///< Maximum feed velocity (mm/s)
    double maxAcceleration = 1000.0;        ///< Maximum acceleration (mm/s²)
    double maxJerk = 50000.0;               ///< Maximum jerk (mm/s³)

    /**
     * @brief Parse G64 command parameters
     * @param P Path tolerance (standard G64 P)
     * @param Q Naive cam tolerance (standard G64 Q)
     * @param I Inside corner tolerance (extension)
     * @param O Outside corner tolerance (extension)
     * @param T Tangential margin (extension)
     * @param strategy Optional strategy string ("inside", "dogbone", "teardrop", "exact-stop")
     */
    void parseG64Parameters(double P = -1, double Q = -1, double I = -1, double O = -1, double T = -1, const std::string& strategy = "") {
        strategyString = strategy;
        
        // Handle exact-stop strategy first - P value is ignored
        if (!strategy.empty() && (strategy == "exact-stop" || strategy == "exactstop")) {
            cornerMode = G64CornerMode::ExactStop;
            pathTolerance = 0.0;
            return;
        }
        
        // Handle explicit strategy parameter (overrides sign semantics)
        bool hasExplicitStrategy = !strategy.empty();
        
        if (hasExplicitStrategy) {
            // Set pathTolerance to abs(P) regardless of sign when strategy is explicit
            if (P != -1) {
                pathTolerance = std::abs(P);
            }
            
            if (strategy == "inside") {
                cornerMode = G64CornerMode::InsideApproximate;
                insideTolerance = std::abs(P);
            } else if (strategy == "dogbone" || strategy == "outside") {
                cornerMode = G64CornerMode::OutsideApproximate;
                outsideTolerance = std::abs(P);
            } else if (strategy == "teardrop") {
                cornerMode = G64CornerMode::Teardrop;
                // For teardrop, sign doesn't matter
                pathTolerance = std::abs(P);
            }
        } else {
            // No explicit strategy - use sign semantics for P
            if (P != -1) {
                pathTolerance = std::abs(P);

                // Only interpret signed P as an inside/outside preference if I/O are not present
                if (I < 0 && O < 0) {
                    if (P > 0) {
                        cornerMode = G64CornerMode::InsideApproximate;
                        insideTolerance = P;
                    } else if (P < 0) {
                        cornerMode = G64CornerMode::OutsideApproximate;
                        outsideTolerance = std::abs(P);
                    } else {
                        cornerMode = G64CornerMode::Centered;
                    }
                }
            }
        }

        if (Q >= 0) naiveCamTolerance = Q;
        if (T >= 0) tangentialMargin = T;

        // Determine corner mode from I/O parameters (explicit overrides sign semantics)
        if (I >= 0 && O >= 0) {
            cornerMode = G64CornerMode::Balanced;
            insideTolerance = I;
            outsideTolerance = O;
        } else if (I >= 0) {
            cornerMode = (I == 0) ? G64CornerMode::InsideStrict
                                   : G64CornerMode::InsideApproximate;
            insideTolerance = I;
        } else if (O >= 0) {
            cornerMode = (O == 0) ? G64CornerMode::OutsideStrict
                                   : G64CornerMode::OutsideApproximate;
            outsideTolerance = O;
        } else if (I < 0 && O < 0 && P == -1) {
            // No explicit I/O or P - default to Centered
            cornerMode = G64CornerMode::Centered;
        }
    }
};

// ============================================================================
// Corner Geometry Analysis
// ============================================================================

/**
 * @brief Result of corner geometry analysis
 */
struct CornerAnalysis {
    CornerType type = CornerType::Straight;
    double angle = 0.0;                     ///< Corner angle (degrees, 0-180)
    double turnAngle = 0.0;                 ///< Direction change (degrees, -180 to +180)
    bool isCW = false;                      ///< True if clockwise turn (right turn)

    Position cornerPoint;                    ///< The corner vertex
    Position incomingDir;                    ///< Unit vector of incoming direction
    Position outgoingDir;                    ///< Unit vector of outgoing direction
    Position bisector;                       ///< Corner bisector direction

    double blendRadius = 0.0;               ///< Computed blend radius
    Position blendCenter;                    ///< Center of blend arc
    Position blendEntry;                     ///< Where blend starts (on incoming segment)
    Position blendExit;                      ///< Where blend ends (on outgoing segment)

    double maxInsideDeviation = 0.0;        ///< Maximum deviation toward inside
    double maxOutsideDeviation = 0.0;       ///< Maximum deviation toward outside
};

/**
 * @brief Analyze corner geometry between two segments
 */
class CornerAnalyzer {
public:
    /**
     * @brief Analyze the corner between two consecutive segments
     * @param seg1 Incoming segment
     * @param seg2 Outgoing segment
     * @return CornerAnalysis with geometric properties
     */
    static CornerAnalysis analyze(const PlanningSegment& seg1, const PlanningSegment& seg2) {
        CornerAnalysis result;
        result.cornerPoint = seg1.end;

        // Compute direction vectors using tangent at junction point
        // For arcs, the tangent is perpendicular to the radius at the endpoint
        result.incomingDir = computeExitTangent(seg1);
        result.outgoingDir = computeEntryTangent(seg2);

        // Compute angle between segments
        double dot = result.incomingDir[0] * result.outgoingDir[0] +
                     result.incomingDir[1] * result.outgoingDir[1] +
                     result.incomingDir[2] * result.outgoingDir[2];

        // Clamp to [-1, 1] for numerical stability
        dot = std::max(-1.0, std::min(1.0, dot));

        result.angle = std::acos(dot) * 180.0 / InterpolationConstants::PI;

        // Compute cross product for turn direction (2D in XY plane)
        double cross = result.incomingDir[0] * result.outgoingDir[1] -
                       result.incomingDir[1] * result.outgoingDir[0];

        result.isCW = cross < 0;
        result.turnAngle = result.isCW ? -result.angle : result.angle;

        // Classify corner type
        if (result.angle < 5.0) {
            result.type = CornerType::Straight;
        } else if (result.angle > 175.0) {
            result.type = CornerType::Cusp;
        } else {
            // Convex = outside corner, concave = inside corner
            // This depends on which side of the path we consider "inside"
            // For conventional milling (tool left of path), CW turn = convex
            result.type = result.isCW ? CornerType::Convex : CornerType::Concave;
        }

        // Compute bisector direction
        result.bisector[0] = result.incomingDir[0] + result.outgoingDir[0];
        result.bisector[1] = result.incomingDir[1] + result.outgoingDir[1];
        result.bisector[2] = result.incomingDir[2] + result.outgoingDir[2];
        normalizeVector(result.bisector);

        return result;
    }

    /**
     * @brief Compute blend geometry for a corner
     * @param analysis Corner analysis result (modified with blend data)
     * @param config G64 configuration
     * @return true if blend is feasible
     */
    static bool computeBlendGeometry(CornerAnalysis& analysis,
                                     const G64CornerConfig& config) {
        if (analysis.type == CornerType::Straight) {
            return false;  // No blend needed for straight path
        }

        // For a circular arc blend inscribed in the corner:
        // The arc is tangent to both incoming and outgoing path segments.
        // analysis.angle is the angle between direction vectors (the turn angle).
        // Half of this angle is used for all geometry calculations.
        // 
        // Key relationships for an inscribed circle tangent to both directions:
        // - halfAngle = analysis.angle / 2
        // - radius = tolerance * cos(halfAngle) / (1 - cos(halfAngle))
        // - tangentDist = radius * tan(halfAngle)
        // - center is at entry + perpendicular * radius

        double halfAngle = analysis.angle / 2.0 * InterpolationConstants::PI / 180.0;
        double sinHalf = std::sin(halfAngle);
        double cosHalf = std::cos(halfAngle);

        if (std::abs(sinHalf) < InterpolationConstants::EPSILON) {
            return false;  // Degenerate case
        }

        double tolerance = config.pathTolerance;

        // Adjust tolerance based on corner mode
        switch (config.cornerMode) {
            case G64CornerMode::InsideStrict:
            case G64CornerMode::InsideApproximate:
                // Blend must stay inside - reduce effective tolerance
                tolerance = std::min(tolerance, config.insideTolerance);
                break;

            case G64CornerMode::OutsideStrict:
            case G64CornerMode::OutsideApproximate:
                // Blend must stay outside - adjust accordingly
                tolerance = std::min(tolerance, config.outsideTolerance);
                break;

            case G64CornerMode::Balanced:
                // Use smaller of the two tolerances
                tolerance = std::min({tolerance, config.insideTolerance, config.outsideTolerance});
                break;

            case G64CornerMode::Teardrop:
                // For teardrop, use the path tolerance directly
                // The teardrop strategy continues past the corner and makes a turn
                // tangential to a circle of radius abs(P)
                tolerance = config.pathTolerance;
                break;

            case G64CornerMode::ExactStop:
                // No blending for exact stop
                return false;

            default:
                break;
        }

        // Maximum chord error at corner midpoint
        double maxDeviation = tolerance;

        // Compute blend radius from desired deviation
        // For a corner, the deviation d = r * (1 - cos(θ/2)) / cos(θ/2)
        //   or r = d * cos(θ/2) / (1 - cos(θ/2))
        if (std::abs(1.0 - cosHalf) < InterpolationConstants::EPSILON) {
            analysis.blendRadius = maxDeviation / InterpolationConstants::EPSILON;
        } else {
            analysis.blendRadius = maxDeviation * cosHalf / (1.0 - cosHalf);
        }

        analysis.blendRadius *= config.blendRadiusFactor;

        // Declare tangentDist that will be used throughout
        double tangentDist = 0.0;

        // Special handling for teardrop strategy
        if (config.cornerMode == G64CornerMode::Teardrop) {
            // Teardrop: Continue past corner, then make a (360° - cornerAngle) turn
            // The turn must be tangential to the circle of radius abs(P)
            
            // For teardrop:
            // 1. Continue straight along incoming direction past the corner
            // 2. Make a sweeping turn that's tangential to tolerance circle
            // 3. Exit in the outgoing direction
            
            // The radius for the teardrop arc is the tolerance itself
            analysis.blendRadius = tolerance;
            
            // Calculate how far past the corner to go before starting the turn
            // This is based on being tangential to the tolerance circle
            // Extension distance = radius / tan(halfAngle)
            double extensionDist = tolerance / std::tan(halfAngle);
            
            // Entry point is before the corner (same as standard)
            tangentDist = tolerance / std::tan(halfAngle);
            analysis.blendEntry[0] = analysis.cornerPoint[0] - analysis.incomingDir[0] * tangentDist;
            analysis.blendEntry[1] = analysis.cornerPoint[1] - analysis.incomingDir[1] * tangentDist;
            analysis.blendEntry[2] = analysis.cornerPoint[2] - analysis.incomingDir[2] * tangentDist;
            
            // Overshoot point: Continue past corner along incoming direction
            Position overshootPoint;
            overshootPoint[0] = analysis.cornerPoint[0] + analysis.incomingDir[0] * extensionDist;
            overshootPoint[1] = analysis.cornerPoint[1] + analysis.incomingDir[1] * extensionDist;
            overshootPoint[2] = analysis.cornerPoint[2] + analysis.incomingDir[2] * extensionDist;
            
            // Exit point: Calculate based on outgoing direction
            analysis.blendExit[0] = analysis.cornerPoint[0] + analysis.outgoingDir[0] * tangentDist;
            analysis.blendExit[1] = analysis.cornerPoint[1] + analysis.outgoingDir[1] * tangentDist;
            analysis.blendExit[2] = analysis.cornerPoint[2] + analysis.outgoingDir[2] * tangentDist;
            
            // Calculate the center of the teardrop arc
            // The arc center is perpendicular to incoming direction at overshoot point
            Position perpDir;
            if (analysis.isCW) {
                perpDir[0] = analysis.incomingDir[1];
                perpDir[1] = -analysis.incomingDir[0];
                perpDir[2] = 0;
            } else {
                perpDir[0] = -analysis.incomingDir[1];
                perpDir[1] = analysis.incomingDir[0];
                perpDir[2] = 0;
            }
            
            analysis.blendCenter[0] = overshootPoint[0] + perpDir[0] * tolerance;
            analysis.blendCenter[1] = overshootPoint[1] + perpDir[1] * tolerance;
            analysis.blendCenter[2] = overshootPoint[2];
            
            // For teardrop, the deviation is the extension distance
            if (analysis.isCW) {
                analysis.maxOutsideDeviation = extensionDist;
                analysis.maxInsideDeviation = 0.0;
            } else {
                analysis.maxInsideDeviation = extensionDist;
                analysis.maxOutsideDeviation = 0.0;
            }
            
            return true;
        }

        // Compute distance along each segment to entry/exit points
        // This is r * tan(θ/2)
        tangentDist = analysis.blendRadius * std::tan(halfAngle);

        // Compute entry and exit points
        analysis.blendEntry[0] = analysis.cornerPoint[0] - analysis.incomingDir[0] * tangentDist;
        analysis.blendEntry[1] = analysis.cornerPoint[1] - analysis.incomingDir[1] * tangentDist;
        analysis.blendEntry[2] = analysis.cornerPoint[2] - analysis.incomingDir[2] * tangentDist;

        analysis.blendExit[0] = analysis.cornerPoint[0] + analysis.outgoingDir[0] * tangentDist;
        analysis.blendExit[1] = analysis.cornerPoint[1] + analysis.outgoingDir[1] * tangentDist;
        analysis.blendExit[2] = analysis.cornerPoint[2] + analysis.outgoingDir[2] * tangentDist;

        // Compute blend center
        // The center lies perpendicular to the incoming direction at distance radius from entry.
        // For a proper inscribed circle tangent to both path segments:
        // - Center is at distance radius perpendicular to incoming direction from entry point
        // - This automatically places it at distance radius from exit point too
        // 
        // For CCW (left turn): center is to the left of incoming direction
        // For CW (right turn): center is to the right of incoming direction
        
        // Perpendicular to incoming direction, pointing toward center
        Position perpIncoming;
        if (analysis.isCW) {
            // For CW turn (right turn), center is to the right of travel
            // Right of incoming = (incoming.y, -incoming.x) for 2D
            perpIncoming[0] = analysis.incomingDir[1];
            perpIncoming[1] = -analysis.incomingDir[0];
            perpIncoming[2] = 0;
        } else {
            // For CCW turn (left turn), center is to the left of travel
            // Left of incoming = (-incoming.y, incoming.x) for 2D
            perpIncoming[0] = -analysis.incomingDir[1];
            perpIncoming[1] = analysis.incomingDir[0];
            perpIncoming[2] = 0;
        }
        
        // Center is at entry point + perpendicular * radius
        analysis.blendCenter[0] = analysis.blendEntry[0] + perpIncoming[0] * analysis.blendRadius;
        analysis.blendCenter[1] = analysis.blendEntry[1] + perpIncoming[1] * analysis.blendRadius;
        analysis.blendCenter[2] = analysis.blendEntry[2];  // Keep Z at entry level

        // Compute maximum deviations
        // At the midpoint of the blend, distance from corner vertex is:
        // d_mid = r / cos(θ/2)
        // Deviation from sharp corner = r / cos(θ/2) - r
        double midpointDist = analysis.blendRadius / cosHalf;
        double deviation = midpointDist - analysis.blendRadius;

        if (analysis.isCW) {
            // CW turn - blend is outside the corner (convex)
            analysis.maxOutsideDeviation = deviation;
            analysis.maxInsideDeviation = 0.0;
        } else {
            // CCW turn - blend is inside the corner (concave)
            analysis.maxInsideDeviation = deviation;
            analysis.maxOutsideDeviation = 0.0;
        }

        return true;
    }

    /**
     * @brief Adjust blend geometry for corner mode constraints
     */
    static void adjustForConstraints(CornerAnalysis& analysis, const G64CornerConfig& config) {
        const double tolerance = InterpolationConstants::EPSILON;
        namespace bc = tether::blend;

        switch (config.cornerMode) {
            case G64CornerMode::InsideStrict: {
                double prevRadius = analysis.blendRadius;
                bc::iterateConstraintReduction(analysis.blendRadius,
                    [&](double r) -> bool {
                        if (r != prevRadius) {
                            computeBlendGeometry(analysis, config);
                            prevRadius = r;
                        }
                        return analysis.maxOutsideDeviation <= tolerance;
                    }, 20, 0.5);
                computeBlendGeometry(analysis, config);
                break;
            }
            case G64CornerMode::OutsideStrict: {
                double prevRadius = analysis.blendRadius;
                bc::iterateConstraintReduction(analysis.blendRadius,
                    [&](double r) -> bool {
                        if (r != prevRadius) {
                            computeBlendGeometry(analysis, config);
                            prevRadius = r;
                        }
                        return analysis.maxInsideDeviation <= tolerance;
                    }, 20, 0.5);
                computeBlendGeometry(analysis, config);
                break;
            }
            default:
                break;
        }
    }

private:
    /**
     * @brief Get plane axis indices for arc tangent computation
     */
    static void getPlaneAxesLocal(InterpolationPlane plane, int& u, int& v, int& w) {
        switch (plane) {
            case InterpolationPlane::XZ:
                u = 0; v = 2; w = 1;
                break;
            case InterpolationPlane::YZ:
                u = 1; v = 2; w = 0;
                break;
            case InterpolationPlane::XY:
            default:
                u = 0; v = 1; w = 2;
                break;
        }
    }

    /**
     * @brief Compute exit tangent direction for a segment at its endpoint.
     * For arcs, this is perpendicular to the radius vector at the endpoint.
     * For lines, this is the chord direction (start -> end).
     */
    static Position computeExitTangent(const PlanningSegment& seg) {
        if (seg.isArc()) {
            int u, v, w;
            getPlaneAxesLocal(seg.plane, u, v, w);

            double cu = seg.center[u];
            double cv = seg.center[v];
            double eu = seg.end[u];
            double ev = seg.end[v];

            double angle = std::atan2(ev - cv, eu - cu);
            int dir = seg.arcDirection();

            // Tangent is perpendicular to the radius vector
            Position tangent{};
            tangent[u] = -dir * std::sin(angle);
            tangent[v] = dir * std::cos(angle);
            // Helical component
            double segLen = seg.segmentLength > InterpolationConstants::EPSILON ? seg.segmentLength : 1.0;
            tangent[w] = (seg.end[w] - seg.start[w]) / segLen;
            normalizeVector(tangent);
            return tangent;
        }
        return computeDirection(seg.start, seg.end);
    }

    /**
     * @brief Compute entry tangent direction for a segment at its start point.
     * For arcs, this is perpendicular to the radius vector at the start point.
     * For lines, this is the chord direction (start -> end).
     */
    static Position computeEntryTangent(const PlanningSegment& seg) {
        if (seg.isArc()) {
            int u, v, w;
            getPlaneAxesLocal(seg.plane, u, v, w);

            double cu = seg.center[u];
            double cv = seg.center[v];
            double su = seg.start[u];
            double sv = seg.start[v];

            double angle = std::atan2(sv - cv, su - cu);
            int dir = seg.arcDirection();

            // Tangent is perpendicular to the radius vector
            Position tangent{};
            tangent[u] = -dir * std::sin(angle);
            tangent[v] = dir * std::cos(angle);
            // Helical component
            double segLen = seg.segmentLength > InterpolationConstants::EPSILON ? seg.segmentLength : 1.0;
            tangent[w] = (seg.end[w] - seg.start[w]) / segLen;
            normalizeVector(tangent);
            return tangent;
        }
        return computeDirection(seg.start, seg.end);
    }

    static Position computeDirection(const Position& from, const Position& to) {
        Position dir;
        double len = 0.0;
        for (size_t i = 0; i < 3; ++i) {
            dir[i] = to[i] - from[i];
            len += dir[i] * dir[i];
        }
        len = std::sqrt(len);
        if (len > InterpolationConstants::EPSILON) {
            for (size_t i = 0; i < 3; ++i) {
                dir[i] /= len;
            }
        }
        return dir;
    }

    static void normalizeVector(Position& v) {
        double len = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (len > InterpolationConstants::EPSILON) {
            v[0] /= len;
            v[1] /= len;
            v[2] /= len;
        }
    }
};

// ============================================================================
// G64 Corner Blend Generator
// ============================================================================

/**
 * @brief Generates blend curves for G64 corner transitions
 */
class G64CornerBlendGenerator {
public:
    /**
     * @brief Generate blend points for a corner
     * @param analysis Corner analysis with blend geometry
     * @param config Blend configuration
     * @param numPoints Number of points to generate
     * @return Vector of trajectory points along the blend
     */
    static std::vector<TrajectoryPoint> generateBlendPoints(
            const CornerAnalysis& analysis,
            const G64CornerConfig& config,
            size_t numPoints = 20) {

        std::vector<TrajectoryPoint> points;

        if (analysis.type == CornerType::Straight) {
            return points;  // No blend needed
        }

        if (config.useBezierBlend) {
            return generateBezierBlend(analysis, config, numPoints);
        } else {
            return generateArcBlend(analysis, config, numPoints);
        }
    }

    /**
     * @brief Check if blend satisfies corner mode constraints
     * @param analysis Corner analysis
     * @param config G64 configuration
     * @return true if constraints are satisfied
     */
    static bool checkConstraints(const CornerAnalysis& analysis,
                                 const G64CornerConfig& config) {
        switch (config.cornerMode) {
            case G64CornerMode::InsideStrict:
                return analysis.maxInsideDeviation <= InterpolationConstants::EPSILON;

            case G64CornerMode::InsideApproximate:
                return analysis.maxInsideDeviation <= config.insideTolerance;

            case G64CornerMode::OutsideStrict:
                return analysis.maxOutsideDeviation <= InterpolationConstants::EPSILON;

            case G64CornerMode::OutsideApproximate:
                return analysis.maxOutsideDeviation <= config.outsideTolerance;

            case G64CornerMode::Balanced:
                return analysis.maxInsideDeviation <= config.insideTolerance &&
                       analysis.maxOutsideDeviation <= config.outsideTolerance;

            case G64CornerMode::Centered:
            default:
                // Traditional G64 - just check overall tolerance
                return std::max(analysis.maxInsideDeviation, analysis.maxOutsideDeviation)
                       <= config.pathTolerance;
        }
    }

    /**
     * @brief Adjust blend radius to satisfy corner mode constraints
     * @param analysis Corner analysis (modified)
     * @param config G64 configuration
     * @return true if adjustment was possible
     */
    static bool adjustForConstraints(CornerAnalysis& analysis,
                                     const G64CornerConfig& config) {
        // For inside-strict mode, we need zero inside deviation
        // This means we cannot use a circular arc - need sharp corner or special curve

        switch (config.cornerMode) {
            case G64CornerMode::InsideStrict:
                if (analysis.type == CornerType::Concave) {
                    // Inside corner - can't blend without going outside
                    analysis.blendRadius = 0.0;
                    return true;  // Will produce sharp corner
                }
                // Convex corner - blend goes outside, which is fine
                return true;

            case G64CornerMode::OutsideStrict:
                if (analysis.type == CornerType::Convex) {
                    // Outside corner - can't blend without going inside
                    analysis.blendRadius = 0.0;
                    return true;  // Will produce sharp corner
                }
                // Concave corner - blend goes inside, which is fine
                return true;

            default:
                return true;
        }
    }

private:
    /**
     * @brief Generate circular arc blend
     */
    static std::vector<TrajectoryPoint> generateArcBlend(
            const CornerAnalysis& analysis,
            const G64CornerConfig& config,
            size_t numPoints) {

        std::vector<TrajectoryPoint> points;
        points.reserve(numPoints);
        (void)config;  // Unused parameter

        // Compute arc parameters
        Position entry = analysis.blendEntry;
        Position center = analysis.blendCenter;

        // Start angle
        double startAngle = std::atan2(entry[1] - center[1], entry[0] - center[0]);

        // Sweep angle - equals the corner angle (turn angle between directions)
        double sweepAngle = analysis.angle * InterpolationConstants::PI / 180.0;
        if (analysis.isCW) sweepAngle = -sweepAngle;

        double radius = analysis.blendRadius;

        for (size_t i = 0; i <= numPoints; ++i) {
            double t = static_cast<double>(i) / numPoints;
            double angle = startAngle + t * sweepAngle;

            TrajectoryPoint pt;
            pt.position[0] = center[0] + radius * std::cos(angle);
            pt.position[1] = center[1] + radius * std::sin(angle);
            pt.position[2] = entry[2] + t * (analysis.blendExit[2] - entry[2]);
            pt.parameter = t;
            pt.isBlendPoint = true;
            pt.curvature = 1.0 / radius;

            points.push_back(pt);
        }

        return points;
    }

    /**
     * @brief Generate Bézier curve blend
     */
    static std::vector<TrajectoryPoint> generateBezierBlend(
            const CornerAnalysis& analysis,
            const G64CornerConfig& config,
            size_t numPoints) {

        std::vector<TrajectoryPoint> points;
        points.reserve(numPoints);

        if (config.bezierOrder == 2) {
            // Quadratic Bézier: entry -> corner -> exit
            Position P0 = analysis.blendEntry;
            Position P1 = analysis.cornerPoint;
            Position P2 = analysis.blendExit;

            for (size_t i = 0; i <= numPoints; ++i) {
                double t = static_cast<double>(i) / numPoints;
                double mt = 1.0 - t;

                TrajectoryPoint pt;
                for (size_t j = 0; j < 3; ++j) {
                    pt.position[j] = mt*mt*P0[j] + 2*mt*t*P1[j] + t*t*P2[j];
                }
                pt.parameter = t;
                pt.isBlendPoint = true;
                points.push_back(pt);
            }
        } else if (config.bezierOrder == 3) {
            // Cubic Bézier (C1 only) — fallback for compatibility
            Position P0 = analysis.blendEntry;
            Position P3 = analysis.blendExit;

            double tangentLen = analysis.blendRadius * 0.5;
            Position P1, P2;
            for (size_t j = 0; j < 3; ++j) {
                P1[j] = P0[j] + analysis.incomingDir[j] * tangentLen;
                P2[j] = P3[j] - analysis.outgoingDir[j] * tangentLen;
            }

            for (size_t i = 0; i <= numPoints; ++i) {
                double t = static_cast<double>(i) / numPoints;
                double mt = 1.0 - t;

                TrajectoryPoint pt;
                for (size_t j = 0; j < 3; ++j) {
                    pt.position[j] = mt*mt*mt*P0[j] + 3*mt*mt*t*P1[j] +
                                     3*mt*t*t*P2[j] + t*t*t*P3[j];
                }
                pt.parameter = t;
                pt.isBlendPoint = true;
                points.push_back(pt);
            }
        } else {
            // Quintic Bézier (C2 continuous) — default for higher continuity
            // Delegate control point computation to shared core
            namespace bc = tether::blend;
            bc::BlendVec entry{analysis.blendEntry[0], analysis.blendEntry[1], analysis.blendEntry[2]};
            bc::BlendVec exit{analysis.blendExit[0], analysis.blendExit[1], analysis.blendExit[2]};
            bc::BlendVec entryDir{analysis.incomingDir[0], analysis.incomingDir[1], analysis.incomingDir[2]};
            bc::BlendVec exitDir{analysis.outgoingDir[0], analysis.outgoingDir[1], analysis.outgoingDir[2]};

            auto cp = bc::quinticC2ControlPoints(entry, exit, entryDir, exitDir);

            for (size_t i = 0; i <= numPoints; ++i) {
                double t = static_cast<double>(i) / numPoints;
                bc::BlendVec pos = bc::evalQuintic(cp, t);

                TrajectoryPoint pt;
                pt.position[0] = pos.x;
                pt.position[1] = pos.y;
                pt.position[2] = pos.z;
                pt.parameter = t;
                pt.isBlendPoint = true;
                points.push_back(pt);
            }
        }

        return points;
    }
};

// ============================================================================
// Extended G64 Path Blender
// ============================================================================

/**
 * @brief Path blender with extended G64 corner mode support
 */
class G64PathBlender {
public:
    explicit G64PathBlender(const G64CornerConfig& config = {})
        : config_(config) {}

    /**
     * @brief Process a sequence of segments with G64 blending
     * @param segments Input motion segments
     * @return Processed segments with blended corners
     */
    std::vector<PlanningSegment> blend(const std::vector<PlanningSegment>& segments) {
        std::vector<PlanningSegment> result;

        if (segments.size() < 2) {
            return segments;
        }

        result.reserve(segments.size() * 2);  // May add blend segments

        // Track the adjusted start position for each segment
        Position nextSegmentStart = segments[0].start;
        bool nextStartAdjusted = false;

        for (size_t i = 0; i < segments.size() - 1; ++i) {
            PlanningSegment seg1 = segments[i];
            const auto& seg2 = segments[i + 1];

            // Adjust start of current segment if previous corner was blended
            if (nextStartAdjusted) {
                seg1.start = nextSegmentStart;
                seg1.segmentLength = seg1.start.linearDistance(seg1.end);
                nextStartAdjusted = false;
            }

            // Skip degenerate (zero-length) segments - just pass them through
            if (seg1.segmentLength < InterpolationConstants::EPSILON) {
                result.push_back(seg1);
                continue;
            }
            
            // Skip if next segment is degenerate
            if (seg2.segmentLength < InterpolationConstants::EPSILON) {
                result.push_back(seg1);
                continue;
            }

            // Analyze corner
            auto analysis = CornerAnalyzer::analyze(seg1, seg2);

            // Skip blending if angle is too small or too large
            if (analysis.angle < config_.minCornerAngle ||
                analysis.angle > config_.maxBlendAngle) {
                result.push_back(seg1);
                corners_.push_back(analysis);
                continue;
            }

            // Compute blend geometry
            if (!CornerAnalyzer::computeBlendGeometry(analysis, config_)) {
                result.push_back(seg1);
                corners_.push_back(analysis);
                continue;
            }

            // Adjust for corner mode constraints
            CornerAnalyzer::adjustForConstraints(analysis, config_);

            // Half-length constraint: clamp blend distances to maxBlendFraction
            // of each segment's available length (shared core)
            double halfAngle = analysis.angle / 2.0 * InterpolationConstants::PI / 180.0;
            double maxEntryDist = seg1.segmentLength * config_.maxBlendFraction;
            double maxExitDist  = seg2.segmentLength * config_.maxBlendFraction;
            double clampedRadius = tether::blend::clampBlendRadius(
                analysis.blendRadius, halfAngle, maxEntryDist, maxExitDist);
            
            if (clampedRadius != analysis.blendRadius) {
                analysis.blendRadius = clampedRadius;
                CornerAnalyzer::computeBlendGeometry(analysis, config_);
                CornerAnalyzer::adjustForConstraints(analysis, config_);
            }

            corners_.push_back(analysis);

            if (analysis.blendRadius <= InterpolationConstants::EPSILON) {
                // No blending - sharp corner
                result.push_back(seg1);
                continue;
            }

            // Truncate incoming segment at blend entry
            PlanningSegment truncated1 = seg1;
            truncated1.end = analysis.blendEntry;
            truncated1.segmentLength = truncated1.start.linearDistance(analysis.blendEntry);
            // Recalculate segment time based on new length
            if (seg1.segmentLength > InterpolationConstants::EPSILON) {
                truncated1.segmentTime = seg1.segmentTime * 
                    (truncated1.segmentLength / seg1.segmentLength);
            }
            result.push_back(truncated1);

            // Add blend segment (Bézier curve, not arc — for C2 continuity)
            PlanningSegment blendSeg;
            blendSeg.start = analysis.blendEntry;
            blendSeg.end = analysis.blendExit;
            blendSeg.center = analysis.blendCenter;
            blendSeg.arcRadius = analysis.blendRadius;
            blendSeg.motionType = SegmentMotionType::Linear;
            blendSeg.feedRate = seg1.feedRate * config_.cornerVelocityFactor;
            blendSeg.blockIndex = seg1.blockIndex;
            blendSeg.plane = InterpolationPlane::XY;

            // Blend segment length: approximate as chord length
            // (actual Bézier arc length is slightly longer but chord is a
            // reasonable approximation for velocity planning)
            blendSeg.segmentLength = analysis.blendEntry.linearDistance(analysis.blendExit);
            if (blendSeg.feedRate > InterpolationConstants::EPSILON) {
                double feedRatePerSec = blendSeg.feedRate / 60.0;
                blendSeg.segmentTime = blendSeg.segmentLength / feedRatePerSec;
            } else {
                blendSeg.segmentTime = 0.0;
            }

            result.push_back(blendSeg);

            // Mark that the next segment needs its start adjusted to blend exit
            nextSegmentStart = analysis.blendExit;
            nextStartAdjusted = true;
        }

        // Add last segment (adjusted if previous corner was blended)
        if (!segments.empty()) {
            PlanningSegment lastSeg = segments.back();
            if (nextStartAdjusted) {
                lastSeg.start = nextSegmentStart;
                lastSeg.segmentLength = lastSeg.start.linearDistance(lastSeg.end);
                // Recalculate segment time for last segment
                if (segments.back().segmentLength > InterpolationConstants::EPSILON) {
                    lastSeg.segmentTime = segments.back().segmentTime * 
                        (lastSeg.segmentLength / segments.back().segmentLength);
                }
            }
            result.push_back(lastSeg);
        }

        return result;
    }

    /**
     * @brief Get analysis for all corners
     */
    const std::vector<CornerAnalysis>& cornerAnalyses() const { return corners_; }

    /**
     * @brief Clear stored corner analyses
     */
    void clear() { corners_.clear(); }

private:
    G64CornerConfig config_;
    std::vector<CornerAnalysis> corners_;
};

} // namespace GCode
