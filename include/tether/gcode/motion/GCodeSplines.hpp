/**
 * @file GCodeSplines.hpp
 * @brief Spline Interpolation: G5, G5.1, G5.2, G5.3 (B-Spline & NURBS)
 * 
 * @details
 * ## Overview
 * 
 * Spline interpolation provides smooth curves through control points,
 * essential for:
 * - CAD/CAM generated toolpaths
 * - 5-axis contouring
 * - Artistic and organic shapes
 * - Reducing program size vs. linear approximation
 * 
 * ## G5 - Cubic B-Spline
 * 
 * Creates a cubic spline segment from current position to endpoint
 * using two control points.
 * 
 * ### Syntax
 * ```gcode
 * G5 X__ Y__ I__ J__ P__ Q__ [F__]
 * ```
 * 
 * ### Parameters
 * | Parameter | Description |
 * |-----------|-------------|
 * | X, Y | Endpoint |
 * | I, J | First control point (relative to start) |
 * | P, Q | Second control point (relative to end) |
 * | F | Feed rate (optional, modal) |
 * 
 * ### Curve Definition
 * ```
 *         Control 2 (End + P, Q)
 *              *
 *             /
 *            /  Curve follows this shape
 *   Start ---.....-----------------> End (X, Y)
 *            \
 *             \
 *              *
 *         Control 1 (Start + I, J)
 * ```
 * 
 * ### Continuity
 * 
 * For smooth transitions between G5 segments, control points should
 * form a straight line across the junction:
 * ```gcode
 * ; First segment
 * G5 X10 Y5 I2 J1 P-1 Q1
 * ; Second segment - P,Q of first mirrors I,J of second
 * G5 X20 Y8 I1 J-1 P-2 Q0
 * ```
 * 
 * G1 exit from G5:
 * - A G1 after G5 is treated as G5 with I=J=0 (mirrored tangent)
 * 
 * ---
 * 
 * ## G5.1 - Quadratic B-Spline
 * 
 * Simpler spline using single control point. Equivalent to a conic arc.
 * 
 * ### Syntax
 * ```gcode
 * G5.1 X__ Y__ I__ J__ [F__]
 * ```
 * 
 * ### Parameters
 * | Parameter | Description |
 * |-----------|-------------|
 * | X, Y | Endpoint |
 * | I, J | Control point (relative to start) |
 * | F | Feed rate |
 * 
 * ### Curve Shape
 * ```
 *              Control (Start + I, J)
 *                  *
 *                 / \
 *                /   \
 *   Start ------.     .------> End (X, Y)
 *              Quadratic curve
 * ```
 * 
 * ---
 * 
 * ## G5.2 - NURBS Block Start
 * ## G5.3 - NURBS Block End
 * 
 * Non-Uniform Rational B-Spline for complex curves.
 * 
 * ### Syntax
 * ```gcode
 * G5.2 [P__]                          ; Start NURBS, order P (default 3)
 * X__ Y__ P__ [L__]                   ; Control point, weight P, knot L
 * X__ Y__ P__ [L__]                   ; More control points...
 * ...
 * G5.3                                ; End NURBS and generate curve
 * ```
 * 
 * ### Parameters
 * | Context | Parameter | Description |
 * |---------|-----------|-------------|
 * | G5.2 | P | Spline order (default 3 = cubic) |
 * | Points | X, Y | Control point position |
 * | Points | P | Weight (1.0 = uniform, >1 = pull toward) |
 * | Points | L | Knot value (optional, auto-generated) |
 * 
 * ### NURBS Theory
 * 
 * NURBS are defined by:
 * 1. **Control points** - positions the curve is "attracted to"
 * 2. **Weights** - how strongly each point attracts the curve
 * 3. **Knot vector** - parameterization of the curve
 * 4. **Order** - degree + 1 (cubic = order 4, but LinuxCNC uses order 3 for cubic)
 * 
 * #### Weight Effects
 * ```
 * Weight = 1.0: Uniform influence
 * Weight > 1.0: Curve pulled toward point
 * Weight < 1.0: Curve pushed away (rare)
 * Weight = ∞:   Curve passes through point
 * ```
 * 
 * #### Knot Vector
 * 
 * The knot vector determines how the parameter maps to control points.
 * - Uniform: [0, 1, 2, 3, 4, ...]
 * - Clamped: [0, 0, 0, 1, 2, ..., n-2, n-1, n-1, n-1]
 * 
 * If L values are omitted, a clamped uniform knot vector is generated.
 * 
 * ### Example: Circle Approximation
 * 
 * ```gcode
 * ; NURBS circle (very close approximation)
 * G5.2 P3
 * X1 Y0 P1
 * X1 Y1 P0.707107    ; Weight = 1/sqrt(2) for circular arc
 * X0 Y1 P1
 * X-1 Y1 P0.707107
 * X-1 Y0 P1
 * X-1 Y-1 P0.707107
 * X0 Y-1 P1
 * X1 Y-1 P0.707107
 * X1 Y0 P1
 * G5.3
 * ```
 * 
 * ### Example: Free-form Curve
 * 
 * ```gcode
 * ; Organic shape
 * G0 X0 Y0
 * G5.2 P3
 * X0 Y0 P1
 * X5 Y10 P1
 * X15 Y15 P2      ; Weight 2 = curve pulled toward this point
 * X25 Y10 P1
 * X30 Y0 P1
 * G5.3
 * ```
 * 
 * ---
 * 
 * ## Implementation Notes
 * 
 * ### Feed Rate on Splines
 * 
 * Feed rate applies to the actual path length (arc length parametrization),
 * not the parameter. This requires computing arc length, which is done via
 * numerical integration.
 * 
 * ### Discretization
 * 
 * Splines are output as either:
 * 1. **Native spline segments** - for controllers supporting them
 * 2. **Linear approximation** - series of G1 moves
 * 
 * Configure via `outputNativeSplines` option.
 * 
 * ### Arc Length Parameterization
 * 
 * For constant feedrate, we need arc length parameterization:
 * ```
 * s(t) = ∫₀ᵗ |C'(u)| du
 * ```
 * 
 * This is computed numerically using Gaussian quadrature.
 * 
 * ### Curvature Analysis
 * 
 * Maximum curvature affects:
 * - Feed rate limits (high curvature = slow down)
 * - Accuracy of linear approximation
 * 
 * Curvature: κ = |C' × C''| / |C'|³
 * 
 * @see GCodeG2G3 for arc interpolation
 * @see GCodeG0G1 for linear motion
 */

#pragma once

#include "../GCodeTypes.hpp"
#include "../GCodeConfig.hpp"
#include <vector>
#include <optional>
#include <array>

namespace GCode {

// Forward declarations
class MachineState;
class VariableSystem;

// ============================================================================
// Spline Configuration
// ============================================================================

/**
 * @brief Configuration for spline interpolation
 */
struct SplineConfig {
    // === General ===
    
    /// Enable G5/G5.1 cubic/quadratic splines
    bool enableCubicSpline{true};
    bool enableQuadraticSpline{true};
    
    /// Enable G5.2/G5.3 NURBS
    bool enableNURBS{true};
    
    /// Output native spline segments (vs linear approximation)
    bool outputNativeSplines{false};
    
    // === Discretization ===
    
    /// Maximum chord error for linearization
    double maxChordError{0.001};  // mm
    
    /// Minimum segment length
    double minSegmentLength{0.01};  // mm
    
    /// Maximum segment length
    double maxSegmentLength{5.0};  // mm
    
    /// Minimum points per spline
    int32_t minPoints{4};
    
    /// Maximum points per spline
    int32_t maxPoints{10000};
    
    // === NURBS ===
    
    /// Default NURBS order (3 = cubic)
    int32_t defaultNURBSOrder{3};
    
    /// Maximum NURBS order
    int32_t maxNURBSOrder{6};
    
    /// Maximum control points
    int32_t maxControlPoints{1000};
    
    /// Auto-clamp knot vector
    bool autoClampKnots{true};
    
    // === Feed Rate ===
    
    /// Use arc length parameterization for feed
    bool arcLengthParam{true};
    
    /// Maximum curvature for feed limit
    double maxCurvature{100.0};  // 1/mm
    
    /// Feed reduction factor at max curvature
    double curvatureFeedFactor{0.5};
    
    // === Continuity ===
    
    /// Check G1 continuity at junctions
    bool checkContinuity{true};
    
    /// Tangent angle tolerance for continuity warning
    double continuityTolerance{0.1};  // radians
};

// ============================================================================
// Spline Data Structures
// ============================================================================

/**
 * @brief 2D control point with optional weight
 */
struct ControlPoint2D {
    double x{0};
    double y{0};
    double weight{1.0};  ///< NURBS weight (1.0 for B-spline)
    
    ControlPoint2D() = default;
    ControlPoint2D(double x_, double y_, double w = 1.0)
        : x(x_), y(y_), weight(w) {}
};

/**
 * @brief 3D control point with weight
 */
struct ControlPoint3D {
    double x{0};
    double y{0};
    double z{0};
    double weight{1.0};
    
    ControlPoint3D() = default;
    ControlPoint3D(double x_, double y_, double z_, double w = 1.0)
        : x(x_), y(y_), z(z_), weight(w) {}
};

/**
 * @brief Cubic spline specification (G5)
 */
struct CubicSplineSpec {
    Position start;
    Position end;
    
    // Control points (relative to start/end)
    double i{0}, j{0};  // First control point offset
    double p{0}, q{0};  // Second control point offset
    
    // Absolute control points (computed)
    ControlPoint2D ctrl1;
    ControlPoint2D ctrl2;
    
    double feedRate{0};
    
    // Computed values
    double length{0};
    double maxCurvature{0};
};

/**
 * @brief Quadratic spline specification (G5.1)
 */
struct QuadraticSplineSpec {
    Position start;
    Position end;
    
    double i{0}, j{0};  // Control point offset
    ControlPoint2D ctrl;
    
    double feedRate{0};
    double length{0};
    double maxCurvature{0};
};

/**
 * @brief NURBS specification (G5.2/G5.3)
 */
struct NURBSSpec {
    /// Control points
    std::vector<ControlPoint3D> controlPoints;
    
    /// Knot vector (size = controlPoints.size() + order)
    std::vector<double> knots;
    
    /// Spline order (degree + 1)
    int32_t order{3};
    
    /// Feed rate
    double feedRate{0};
    
    /// Computed length
    double length{0};
    
    /// Is closed curve
    bool closed{false};
};

// ============================================================================
// B-Spline Evaluation
// ============================================================================

/**
 * @brief Evaluate cubic Bezier curve
 * 
 * @param p0 Start point
 * @param p1 First control point
 * @param p2 Second control point
 * @param p3 End point
 * @param t Parameter [0, 1]
 * @return Point on curve
 */
ControlPoint2D evaluateCubicBezier(
    const ControlPoint2D& p0,
    const ControlPoint2D& p1,
    const ControlPoint2D& p2,
    const ControlPoint2D& p3,
    double t
);

/**
 * @brief Evaluate quadratic Bezier curve
 */
ControlPoint2D evaluateQuadraticBezier(
    const ControlPoint2D& p0,
    const ControlPoint2D& p1,
    const ControlPoint2D& p2,
    double t
);

/**
 * @brief Evaluate cubic Bezier derivative
 * 
 * Returns tangent vector at parameter t.
 */
ControlPoint2D evaluateCubicBezierDerivative(
    const ControlPoint2D& p0,
    const ControlPoint2D& p1,
    const ControlPoint2D& p2,
    const ControlPoint2D& p3,
    double t
);

/**
 * @brief Evaluate cubic Bezier second derivative
 */
ControlPoint2D evaluateCubicBezierSecondDerivative(
    const ControlPoint2D& p0,
    const ControlPoint2D& p1,
    const ControlPoint2D& p2,
    const ControlPoint2D& p3,
    double t
);

/**
 * @brief Calculate curvature at parameter t
 */
double calculateCurvature(
    const ControlPoint2D& tangent,
    const ControlPoint2D& secondDeriv
);

// ============================================================================
// NURBS Evaluation
// ============================================================================

/**
 * @brief Evaluate NURBS basis function
 * 
 * @param i Control point index
 * @param p Degree (order - 1)
 * @param u Parameter value
 * @param knots Knot vector
 * @return Basis function value N_{i,p}(u)
 */
double evaluateBasisFunction(
    int32_t i,
    int32_t p,
    double u,
    const std::vector<double>& knots
);

/**
 * @brief Evaluate NURBS curve
 * 
 * C(u) = Σ (N_{i,p}(u) * w_i * P_i) / Σ (N_{i,p}(u) * w_i)
 */
ControlPoint3D evaluateNURBS(
    const NURBSSpec& nurbs,
    double u
);

/**
 * @brief Evaluate NURBS derivative
 */
ControlPoint3D evaluateNURBSDerivative(
    const NURBSSpec& nurbs,
    double u
);

/**
 * @brief Generate clamped uniform knot vector
 * 
 * @param n Number of control points
 * @param order Spline order
 * @return Knot vector
 */
std::vector<double> generateClampedKnots(int32_t n, int32_t order);

// ============================================================================
// Arc Length Calculation
// ============================================================================

/**
 * @brief Calculate cubic spline arc length
 * 
 * Uses Gaussian quadrature for numerical integration.
 */
double calculateCubicSplineLength(const CubicSplineSpec& spec);

/**
 * @brief Calculate quadratic spline arc length
 */
double calculateQuadraticSplineLength(const QuadraticSplineSpec& spec);

/**
 * @brief Calculate NURBS arc length
 */
double calculateNURBSLength(const NURBSSpec& nurbs);

/**
 * @brief Find parameter for given arc length (inverse arc length)
 * 
 * @param spec Spline specification
 * @param targetLength Desired arc length from start
 * @return Parameter t such that arcLength(t) ≈ targetLength
 */
double findParameterForLength(
    const CubicSplineSpec& spec,
    double targetLength
);

// ============================================================================
// Spline Discretization
// ============================================================================

/**
 * @brief Discretize cubic spline into line segments
 */
Error discretizeCubicSpline(
    const CubicSplineSpec& spec,
    const SplineConfig& config,
    std::vector<Position>& points
);

/**
 * @brief Discretize quadratic spline
 */
Error discretizeQuadraticSpline(
    const QuadraticSplineSpec& spec,
    const SplineConfig& config,
    std::vector<Position>& points
);

/**
 * @brief Discretize NURBS
 */
Error discretizeNURBS(
    const NURBSSpec& nurbs,
    const SplineConfig& config,
    std::vector<Position>& points
);

// ============================================================================
// Motion Segment Generation
// ============================================================================

/**
 * @brief Generate motion segments for G5 (cubic spline)
 */
Error generateG5Segments(
    const Block& block,
    const MachineState& state,
    const SplineConfig& config,
    std::vector<MotionSegment>& segments
);

/**
 * @brief Generate motion segments for G5.1 (quadratic spline)
 */
Error generateG5_1Segments(
    const Block& block,
    const MachineState& state,
    const SplineConfig& config,
    std::vector<MotionSegment>& segments
);

// ============================================================================
// Spline Handler Class
// ============================================================================

/**
 * @brief Handler for spline commands
 */
class SplineHandler {
public:
    explicit SplineHandler(const SplineConfig& config = {});
    
    /**
     * @brief Process G5 (cubic spline)
     */
    Error processG5(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G5.1 (quadratic spline)
     */
    Error processG5_1(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Process G5.2 (start NURBS block)
     */
    Error processG5_2(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Add NURBS control point
     * 
     * Called for each line between G5.2 and G5.3.
     */
    Error addNURBSControlPoint(
        const Block& block,
        MachineState& state,
        VariableSystem& vars
    );
    
    /**
     * @brief Process G5.3 (end NURBS block)
     */
    Error processG5_3(
        const Block& block,
        MachineState& state,
        VariableSystem& vars,
        std::vector<MotionSegment>& segments
    );
    
    /**
     * @brief Check if in NURBS block
     */
    bool inNURBSBlock() const { return m_inNURBS; }
    
    /**
     * @brief Update configuration
     */
    void setConfig(const SplineConfig& config) { m_config = config; }
    
    /**
     * @brief Get current configuration
     */
    const SplineConfig& getConfig() const { return m_config; }
    
    /**
     * @brief Get last G5 spline (for continuity checking)
     */
    const CubicSplineSpec& getLastCubicSpline() const { return m_lastCubic; }
    
    /**
     * @brief Get current NURBS being built
     */
    const NURBSSpec& getCurrentNURBS() const { return m_currentNURBS; }
    
private:
    SplineConfig m_config;
    
    // State for G5 continuity
    CubicSplineSpec m_lastCubic;
    bool m_hadPreviousG5{false};
    
    // State for NURBS block
    bool m_inNURBS{false};
    NURBSSpec m_currentNURBS;
    
    Error finalizeNURBS(
        MachineState& state,
        std::vector<MotionSegment>& segments
    );
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Check G1 continuity between two splines
 * 
 * Returns angle between exit tangent of first and entry tangent of second.
 */
double checkSplineContinuity(
    const CubicSplineSpec& first,
    const CubicSplineSpec& second
);

/**
 * @brief Convert quadratic to cubic spline
 * 
 * Any quadratic Bezier can be exactly represented as a cubic.
 */
CubicSplineSpec quadraticToCubic(const QuadraticSplineSpec& quad);

/**
 * @brief Estimate control points for smooth curve through points
 * 
 * Given a series of pass-through points, estimate cubic spline
 * control points using Catmull-Rom or similar.
 */
std::vector<CubicSplineSpec> fitCubicSplines(
    const std::vector<Position>& points,
    double tension = 0.5
);

} // namespace GCode
