/**
 * @file MultiAxisPath.cpp
 * @brief Multi-axis path interpolation implementations
 */

#include "profiles/cia402/MultiAxisPath.hpp"
#include "tether/platform/EspCompat.hpp"
#include <numeric>

static const char* TAG = "MultiAxisPath";

namespace CiA402 {

// ============================================================================
// PathSegment Base
// ============================================================================

std::vector<PathPoint> PathSegment::sample(size_t numSamples) const {
    std::vector<PathPoint> points;
    points.reserve(numSamples);
    
    for (size_t i = 0; i < numSamples; i++) {
        double u = static_cast<double>(i) / (numSamples - 1);
        points.push_back(evaluate(u));
    }
    
    return points;
}

double PathSegment::arcLength(double u, size_t numSteps) const {
    if (u <= 0.0) return 0.0;
    // Clamp to [0,1] instead of calling getLength() to avoid infinite
    // recursion when getLength() itself delegates to arcLength(1.0, N).
    u = std::min(u, 1.0);
    
    // Numerical integration using trapezoidal rule
    double length = 0.0;
    double du = u / numSteps;
    
    PathPoint prev = evaluate(0.0);
    for (size_t i = 1; i <= numSteps; i++) {
        PathPoint curr = evaluate(i * du);
        double segmentLength = 0.0;
        
        for (size_t j = 0; j < getNumAxes(); j++) {
            double d = curr.position[j] - prev.position[j];
            segmentLength += d * d;
        }
        
        length += std::sqrt(segmentLength);
        prev = curr;
    }
    
    return length;
}

double PathSegment::parameterAtLength(double s, double tolerance) const {
    double totalLength = getLength();
    if (s <= 0.0) return 0.0;
    if (s >= totalLength) return 1.0;
    
    // Binary search
    double low = 0.0;
    double high = 1.0;
    
    while ((high - low) > tolerance) {
        double mid = (low + high) / 2.0;
        double midLength = arcLength(mid);
        
        if (midLength < s) {
            low = mid;
        } else {
            high = mid;
        }
    }
    
    return (low + high) / 2.0;
}

// ============================================================================
// Linear Path
// ============================================================================

LinearPath::LinearPath(const LinearConfig& config) {
    configure(config);
}

void LinearPath::configure(const LinearConfig& config) {
    m_config = config;
    
    // Calculate length
    m_length = 0.0;
    for (size_t i = 0; i < config.numAxes; i++) {
        double d = config.end[i] - config.start[i];
        m_length += d * d;
    }
    m_length = std::sqrt(m_length);
}

PathPoint LinearPath::evaluate(double u) const {
    PathPoint pt;
    pt.parameter = u;
    pt.numAxes = m_config.numAxes;
    
    // Clamp parameter
    u = std::clamp(u, 0.0, 1.0);
    
    // Linear interpolation
    for (size_t i = 0; i < m_config.numAxes; i++) {
        pt.position[i] = m_config.start[i] + u * (m_config.end[i] - m_config.start[i]);
    }
    
    // Velocity direction (constant along path)
    if (m_length > 1e-9) {
        for (size_t i = 0; i < m_config.numAxes; i++) {
            pt.velocity[i] = (m_config.end[i] - m_config.start[i]) / m_length;
        }
    }
    
    // Acceleration is zero for linear path
    pt.curvature = 0.0;
    
    return pt;
}

double LinearPath::getLength() const {
    return m_length;
}

// ============================================================================
// Circular Path
// ============================================================================

CircularPath::CircularPath(const CircularConfig& config) {
    configure(config);
}

void CircularPath::configure(const CircularConfig& config) {
    m_config = config;
    
    // Set axis indices based on plane
    switch (config.plane) {
        case Plane::XY:
            m_axisU = 0; m_axisV = 1; m_axisN = 2;
            break;
        case Plane::XZ:
            m_axisU = 0; m_axisV = 2; m_axisN = 1;
            break;
        case Plane::YZ:
            m_axisU = 1; m_axisV = 2; m_axisN = 0;
            break;
    }
    
    if (config.useAngles) {
        m_startAngle = config.startAngle;
        m_endAngle = config.endAngle;
        m_radius = config.radius;
    } else {
        calculateAngles();
    }
}

void CircularPath::calculateAngles() {
    // Calculate radius from center to start point
    double dx = m_config.start[m_axisU] - m_config.center[m_axisU];
    double dy = m_config.start[m_axisV] - m_config.center[m_axisV];
    m_radius = std::sqrt(dx * dx + dy * dy);
    
    if (m_radius < 1e-9) {
        TETHER_LOGW(TAG, "Degenerate circular path (zero radius)");
        return;
    }
    
    // Start angle
    m_startAngle = std::atan2(dy, dx);
    
    // End angle
    dx = m_config.end[m_axisU] - m_config.center[m_axisU];
    dy = m_config.end[m_axisV] - m_config.center[m_axisV];
    m_endAngle = std::atan2(dy, dx);
    
    // Adjust angles based on direction
    if (m_config.direction == ArcDirection::CCW) {
        if (m_endAngle <= m_startAngle) {
            m_endAngle += 2 * M_PI;
        }
    } else {
        if (m_endAngle >= m_startAngle) {
            m_endAngle -= 2 * M_PI;
        }
    }
}

void CircularPath::configureFromPoints(const std::array<double, 3>& center,
                                       const std::array<double, 3>& start,
                                       const std::array<double, 3>& end,
                                       ArcDirection dir,
                                       Plane plane) {
    CircularConfig cfg;
    cfg.center = center;
    cfg.start = start;
    cfg.end = end;
    cfg.plane = plane;
    cfg.direction = dir;
    cfg.useAngles = false;
    configure(cfg);
}

void CircularPath::configureFromRadius(const std::array<double, 3>& start,
                                       const std::array<double, 3>& end,
                                       double radius,
                                       bool largeArc,
                                       ArcDirection dir,
                                       Plane plane) {
    // Calculate center from endpoints and radius
    // Two solutions exist - choose based on direction and largeArc
    
    // Set plane first - initialize to XY plane by default
    int axisU = 0, axisV = 1;
    switch (plane) {
        case Plane::XY: axisU = 0; axisV = 1; break;
        case Plane::XZ: axisU = 0; axisV = 2; break;
        case Plane::YZ: axisU = 1; axisV = 2; break;
    }
    
    // Midpoint
    double mx = (start[axisU] + end[axisU]) / 2.0;
    double my = (start[axisV] + end[axisV]) / 2.0;
    
    // Half chord length
    double dx = end[axisU] - start[axisU];
    double dy = end[axisV] - start[axisV];
    double d = std::sqrt(dx * dx + dy * dy);
    double halfChord = d / 2.0;
    
    if (halfChord > radius) {
        TETHER_LOGW(TAG, "Chord longer than diameter, using chord as diameter");
        radius = halfChord;
    }
    
    // Distance from midpoint to center
    double h = std::sqrt(radius * radius - halfChord * halfChord);
    
    // Perpendicular direction
    double px = -dy / d;
    double py = dx / d;
    
    // Choose center based on direction and arc size
    int sign = ((dir == ArcDirection::CCW) ^ largeArc) ? 1 : -1;
    
    CircularConfig cfg;
    cfg.center[axisU] = mx + sign * h * px;
    cfg.center[axisV] = my + sign * h * py;
    cfg.start = start;
    cfg.end = end;
    cfg.radius = radius;
    cfg.plane = plane;
    cfg.direction = dir;
    cfg.useAngles = false;
    
    configure(cfg);
}

PathPoint CircularPath::evaluate(double u) const {
    PathPoint pt;
    pt.parameter = u;
    pt.numAxes = 3;
    
    u = std::clamp(u, 0.0, 1.0);
    
    // Current angle
    double angle = m_startAngle + u * (m_endAngle - m_startAngle);
    
    // Position
    pt.position[m_axisU] = m_config.center[m_axisU] + m_radius * std::cos(angle);
    pt.position[m_axisV] = m_config.center[m_axisV] + m_radius * std::sin(angle);
    pt.position[m_axisN] = m_config.start[m_axisN];  // Constant in normal direction
    
    // Velocity direction (tangent)
    double angularVel = (m_endAngle - m_startAngle);
    pt.velocity[m_axisU] = -m_radius * std::sin(angle) * angularVel;
    pt.velocity[m_axisV] = m_radius * std::cos(angle) * angularVel;
    pt.velocity[m_axisN] = 0.0;
    
    // Normalize velocity
    double velMag = std::sqrt(pt.velocity[m_axisU] * pt.velocity[m_axisU] + 
                              pt.velocity[m_axisV] * pt.velocity[m_axisV]);
    if (velMag > 1e-9) {
        pt.velocity[m_axisU] /= velMag;
        pt.velocity[m_axisV] /= velMag;
    }
    
    // Curvature
    pt.curvature = 1.0 / m_radius;
    
    return pt;
}

double CircularPath::getLength() const {
    return std::abs(m_endAngle - m_startAngle) * m_radius;
}

// ============================================================================
// Helical Path
// ============================================================================

HelicalPath::HelicalPath(const HelicalConfig& config) {
    configure(config);
}

void HelicalPath::configure(const HelicalConfig& config) {
    m_config = config;
    
    switch (config.plane) {
        case Plane::XY: m_axisU = 0; m_axisV = 1; m_axisN = 2; break;
        case Plane::XZ: m_axisU = 0; m_axisV = 2; m_axisN = 1; break;
        case Plane::YZ: m_axisU = 1; m_axisV = 2; m_axisN = 0; break;
    }
}

PathPoint HelicalPath::evaluate(double u) const {
    PathPoint pt;
    pt.parameter = u;
    pt.numAxes = 3;
    
    u = std::clamp(u, 0.0, 1.0);
    
    double angle = m_config.startAngle + u * m_config.totalAngle;
    if (m_config.direction == ArcDirection::CW) {
        angle = m_config.startAngle - u * std::abs(m_config.totalAngle);
    }
    
    // Radial position
    pt.position[m_axisU] = m_config.center[m_axisU] + m_config.radius * std::cos(angle);
    pt.position[m_axisV] = m_config.center[m_axisV] + m_config.radius * std::sin(angle);
    
    // Axial position (linear interpolation)
    double numRevolutions = m_config.totalAngle / (2 * M_PI);
    pt.position[m_axisN] = m_config.center[m_axisN] + u * numRevolutions * m_config.pitch;
    
    // Velocity direction
    double angularVel = m_config.totalAngle;
    pt.velocity[m_axisU] = -m_config.radius * std::sin(angle);
    pt.velocity[m_axisV] = m_config.radius * std::cos(angle);
    pt.velocity[m_axisN] = numRevolutions * m_config.pitch / m_config.totalAngle;
    
    // Normalize
    double velMag = std::sqrt(pt.velocity[0]*pt.velocity[0] + 
                              pt.velocity[1]*pt.velocity[1] + 
                              pt.velocity[2]*pt.velocity[2]);
    if (velMag > 1e-9) {
        for (int i = 0; i < 3; i++) {
            pt.velocity[i] /= velMag;
        }
    }
    
    return pt;
}

double HelicalPath::getLength() const {
    // Helix length = sqrt((r*theta)^2 + h^2)
    // where theta is total angle and h is total height
    double arcLength = m_config.radius * std::abs(m_config.totalAngle);
    double numRevolutions = std::abs(m_config.totalAngle) / (2 * M_PI);
    double height = numRevolutions * m_config.pitch;
    
    return std::sqrt(arcLength * arcLength + height * height);
}

// ============================================================================
// B-Spline Path
// ============================================================================

BSplinePath::BSplinePath(const BSplineConfig& config) {
    configure(config);
}

void BSplinePath::configure(const BSplineConfig& config) {
    m_config = config;
    m_cachedLength = -1.0;
    
    if (config.knots.empty()) {
        generateUniformKnots();
    }
}

void BSplinePath::addControlPoint(const std::array<double, MAX_PATH_AXES>& point) {
    m_config.controlPoints.push_back(point);
    m_cachedLength = -1.0;
    generateUniformKnots();
}

void BSplinePath::clearControlPoints() {
    m_config.controlPoints.clear();
    m_config.knots.clear();
    m_cachedLength = -1.0;
}

void BSplinePath::generateUniformKnots() {
    int n = static_cast<int>(m_config.controlPoints.size()) - 1;
    int p = m_config.degree;
    
    if (n < 0) return;
    
    m_config.knots.clear();
    m_config.knots.resize(n + p + 2);
    
    // Clamped uniform knot vector
    for (int i = 0; i <= p; i++) {
        m_config.knots[i] = 0.0;
    }
    
    for (int i = p + 1; i <= n; i++) {
        m_config.knots[i] = static_cast<double>(i - p) / (n - p + 1);
    }
    
    for (int i = n + 1; i <= n + p + 1; i++) {
        m_config.knots[i] = 1.0;
    }
}

double BSplinePath::basis(int i, int p, double u) const {
    if (p == 0) {
        if (m_config.knots[i] <= u && u < m_config.knots[i + 1]) {
            return 1.0;
        }
        // Handle endpoint
        if (i == static_cast<int>(m_config.controlPoints.size()) - 1 && 
            std::abs(u - 1.0) < 1e-9) {
            return 1.0;
        }
        return 0.0;
    }
    
    double left = 0.0, right = 0.0;
    
    double denom1 = m_config.knots[i + p] - m_config.knots[i];
    if (denom1 > 1e-9) {
        left = (u - m_config.knots[i]) / denom1 * basis(i, p - 1, u);
    }
    
    double denom2 = m_config.knots[i + p + 1] - m_config.knots[i + 1];
    if (denom2 > 1e-9) {
        right = (m_config.knots[i + p + 1] - u) / denom2 * basis(i + 1, p - 1, u);
    }
    
    return left + right;
}

PathPoint BSplinePath::evaluate(double u) const {
    PathPoint pt;
    pt.parameter = u;
    pt.numAxes = m_config.numAxes;
    
    u = std::clamp(u, 0.0, 1.0);
    
    int n = static_cast<int>(m_config.controlPoints.size());
    int p = m_config.degree;
    
    if (n == 0) return pt;
    
    // Calculate position
    for (size_t axis = 0; axis < m_config.numAxes; axis++) {
        double sum = 0.0;
        for (int i = 0; i < n; i++) {
            sum += basis(i, p, u) * m_config.controlPoints[i][axis];
        }
        pt.position[axis] = sum;
    }
    
    return pt;
}

double BSplinePath::getLength() const {
    if (m_cachedLength >= 0) {
        return m_cachedLength;
    }
    
    // Numerical integration
    m_cachedLength = arcLength(1.0, 200);
    return m_cachedLength;
}

// ============================================================================
// NURBS Path
// ============================================================================

NURBSPath::NURBSPath(const NURBSConfig& config) {
    configure(config);
}

void NURBSPath::configure(const NURBSConfig& config) {
    m_config = config;
    m_cachedLength = -1.0;
    
    // Default weights to 1
    if (m_config.weights.empty()) {
        m_config.weights.resize(config.controlPoints.size(), 1.0);
    }
    
    // Generate uniform knots if not provided
    if (m_config.knots.empty()) {
        int n = static_cast<int>(config.controlPoints.size()) - 1;
        int p = config.degree;
        
        m_config.knots.resize(n + p + 2);
        for (int i = 0; i <= p; i++) {
            m_config.knots[i] = 0.0;
        }
        for (int i = p + 1; i <= n; i++) {
            m_config.knots[i] = static_cast<double>(i - p) / (n - p + 1);
        }
        for (int i = n + 1; i <= n + p + 1; i++) {
            m_config.knots[i] = 1.0;
        }
    }
}

double NURBSPath::bSplineBasis(int i, int p, double u) const {
    if (p == 0) {
        if (m_config.knots[i] <= u && u < m_config.knots[i + 1]) {
            return 1.0;
        }
        if (i == static_cast<int>(m_config.controlPoints.size()) - 1 && 
            std::abs(u - 1.0) < 1e-9) {
            return 1.0;
        }
        return 0.0;
    }
    
    double left = 0.0, right = 0.0;
    
    double d1 = m_config.knots[i + p] - m_config.knots[i];
    if (d1 > 1e-9) {
        left = (u - m_config.knots[i]) / d1 * bSplineBasis(i, p - 1, u);
    }
    
    double d2 = m_config.knots[i + p + 1] - m_config.knots[i + 1];
    if (d2 > 1e-9) {
        right = (m_config.knots[i + p + 1] - u) / d2 * bSplineBasis(i + 1, p - 1, u);
    }
    
    return left + right;
}

double NURBSPath::rationalBasis(int i, double u) const {
    double num = bSplineBasis(i, m_config.degree, u) * m_config.weights[i];
    
    double denom = 0.0;
    for (size_t j = 0; j < m_config.controlPoints.size(); j++) {
        denom += bSplineBasis(j, m_config.degree, u) * m_config.weights[j];
    }
    
    return (denom > 1e-9) ? num / denom : 0.0;
}

PathPoint NURBSPath::evaluate(double u) const {
    PathPoint pt;
    pt.parameter = u;
    pt.numAxes = m_config.numAxes;
    
    u = std::clamp(u, 0.0, 1.0);
    
    int n = static_cast<int>(m_config.controlPoints.size());
    if (n == 0) return pt;
    
    for (size_t axis = 0; axis < m_config.numAxes; axis++) {
        double sum = 0.0;
        for (int i = 0; i < n; i++) {
            sum += rationalBasis(i, u) * m_config.controlPoints[i][axis];
        }
        pt.position[axis] = sum;
    }
    
    return pt;
}

double NURBSPath::getLength() const {
    if (m_cachedLength >= 0) return m_cachedLength;
    m_cachedLength = const_cast<NURBSPath*>(this)->arcLength(1.0, 200);
    return m_cachedLength;
}

// ============================================================================
// Bezier Path
// ============================================================================

BezierPath::BezierPath(const BezierConfig& config) {
    configure(config);
}

void BezierPath::configure(const BezierConfig& config) {
    m_config = config;
    m_cachedLength = -1.0;
}

void BezierPath::configureCubic(const std::array<double, MAX_PATH_AXES>& p0,
                                const std::array<double, MAX_PATH_AXES>& p1,
                                const std::array<double, MAX_PATH_AXES>& p2,
                                const std::array<double, MAX_PATH_AXES>& p3,
                                size_t numAxes) {
    m_config.controlPoints = {p0, p1, p2, p3};
    m_config.numAxes = numAxes;
    m_cachedLength = -1.0;
}

int BezierPath::binomial(int n, int k) {
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;
    
    int result = 1;
    for (int i = 0; i < k; i++) {
        result = result * (n - i) / (i + 1);
    }
    return result;
}

double BezierPath::bernstein(int n, int i, double t) {
    return binomial(n, i) * std::pow(t, i) * std::pow(1 - t, n - i);
}

PathPoint BezierPath::evaluate(double u) const {
    PathPoint pt;
    pt.parameter = u;
    pt.numAxes = m_config.numAxes;
    
    u = std::clamp(u, 0.0, 1.0);
    
    int n = static_cast<int>(m_config.controlPoints.size()) - 1;
    if (n < 0) return pt;
    
    // Position using Bernstein polynomials
    for (size_t axis = 0; axis < m_config.numAxes; axis++) {
        double sum = 0.0;
        for (int i = 0; i <= n; i++) {
            sum += bernstein(n, i, u) * m_config.controlPoints[i][axis];
        }
        pt.position[axis] = sum;
    }
    
    // Velocity (first derivative)
    if (n >= 1) {
        for (size_t axis = 0; axis < m_config.numAxes; axis++) {
            double sum = 0.0;
            for (int i = 0; i < n; i++) {
                double dp = m_config.controlPoints[i + 1][axis] - 
                           m_config.controlPoints[i][axis];
                sum += n * bernstein(n - 1, i, u) * dp;
            }
            pt.velocity[axis] = sum;
        }
    }
    
    return pt;
}

double BezierPath::getLength() const {
    if (m_cachedLength >= 0) return m_cachedLength;
    m_cachedLength = const_cast<BezierPath*>(this)->arcLength(1.0, 200);
    return m_cachedLength;
}

// ============================================================================
// Path Sampler
// ============================================================================

PathSampler::PathSampler(std::unique_ptr<PathSegment> path, double feedrate)
    : m_ownedPath(std::move(path))
    , m_feedrate(feedrate)
{
    if (m_ownedPath) {
        m_pathLength = m_ownedPath->getLength();
        // Simple duration calculation: time = distance / velocity
        m_duration = (feedrate > 0) ? (m_pathLength / feedrate) : 0.0;
    }
}

void PathSampler::configure(std::shared_ptr<PathSegment> path,
                           std::shared_ptr<MotionProfile> profile) {
    m_path = path;
    m_profile = profile;
    m_pathLength = path ? path->getLength() : 0.0;
}

void PathSampler::plan(double feedrate) {
    m_feedrate = feedrate;
    
    if (m_profile && m_pathLength > 0) {
        // Plan profile from 0 to path length
        m_profile->plan(0.0, m_pathLength, 0.0, 0.0);
    }
    
    // Update duration for simple mode
    if (!m_profile && feedrate > 0) {
        m_duration = m_pathLength / feedrate;
    }
}

PathPoint PathSampler::sample(double time) const {
    // Simple mode using owned path
    if (m_ownedPath && !m_profile) {
        if (m_pathLength < 1e-9 || m_duration < 1e-9) {
            return PathPoint{};
        }
        
        // Linear time parameterization
        double u = time / m_duration;
        u = std::clamp(u, 0.0, 1.0);
        
        PathPoint pt = m_ownedPath->evaluate(u);
        
        // Set velocity based on feedrate
        for (size_t i = 0; i < pt.numAxes; i++) {
            // Approximate by scaling tangent
            pt.velocity[i] *= m_feedrate;
        }
        pt.pathVelocity = m_feedrate;
        
        return pt;
    }
    
    // Profile-based mode using shared path
    if (!m_path || !m_profile || m_pathLength < 1e-9) {
        return PathPoint{};
    }
    
    // Get position along path from profile
    MotionState ms = m_profile->evaluate(time);
    
    // Convert position to parameter
    double u = ms.position / m_pathLength;
    u = std::clamp(u, 0.0, 1.0);
    
    // Get path point
    PathPoint pt = m_path->evaluate(u);
    
    // Scale velocity by profile velocity
    double pathVel = ms.velocity;
    for (size_t i = 0; i < pt.numAxes; i++) {
        pt.velocity[i] *= pathVel;
    }
    
    pt.pathVelocity = pathVel;
    
    return pt;
}

bool PathSampler::isComplete(double time) const {
    if (m_ownedPath && !m_profile) {
        return time >= m_duration;
    }
    return m_profile ? m_profile->isComplete(time) : true;
}

double PathSampler::getDuration() const {
    if (m_ownedPath && !m_profile) {
        return m_duration;
    }
    return m_profile ? m_profile->getDuration() : 0.0;
}

double PathSampler::getParameter(double time) const {
    if (!m_profile || m_pathLength < 1e-9) return 0.0;
    
    MotionState ms = m_profile->evaluate(time);
    return std::clamp(ms.position / m_pathLength, 0.0, 1.0);
}

// ============================================================================
// Multi-Segment Path
// ============================================================================

void MultiSegmentPath::addSegment(std::shared_ptr<PathSegment> segment) {
    SegmentInfo info;
    info.segment = segment;
    m_segments.push_back(info);
}

void MultiSegmentPath::plan(double feedrate, const MotionLimits& limits) {
    if (m_segments.empty()) {
        m_totalDuration = 0.0;
        return;
    }
    
    double currentTime = 0.0;
    
    for (auto& seg : m_segments) {
        seg.feedrate = feedrate;
        seg.startTime = currentTime;
        
        double length = seg.segment->getLength();
        double duration = length / feedrate;
        
        // TODO: Apply limits and blending
        
        seg.endTime = currentTime + duration;
        currentTime = seg.endTime;
    }
    
    m_totalDuration = currentTime;
}

PathPoint MultiSegmentPath::sample(double time) const {
    if (m_segments.empty()) {
        return PathPoint{};
    }
    
    // Find active segment
    for (const auto& seg : m_segments) {
        if (time >= seg.startTime && time <= seg.endTime) {
            double segTime = time - seg.startTime;
            double segDuration = seg.endTime - seg.startTime;
            double u = (segDuration > 1e-9) ? segTime / segDuration : 0.0;
            
            return seg.segment->evaluate(u);
        }
    }
    
    // Past end - return last point
    return m_segments.back().segment->evaluate(1.0);
}

PathPoint MultiSegmentPath::evaluate(double u) const {
    if (m_segments.empty()) {
        return PathPoint{};
    }
    
    // Map u [0,1] to overall path position
    double totalLen = getTotalLength();
    if (totalLen < 1e-9) {
        return m_segments.front().segment->evaluate(0.0);
    }
    
    double targetLen = u * totalLen;
    double accumulatedLen = 0.0;
    
    for (const auto& seg : m_segments) {
        double segLen = seg.segment->getLength();
        if (accumulatedLen + segLen >= targetLen) {
            // This segment contains our target
            double localLen = targetLen - accumulatedLen;
            double localU = (segLen > 1e-9) ? localLen / segLen : 0.0;
            PathPoint pt = seg.segment->evaluate(localU);
            pt.parameter = u;
            return pt;
        }
        accumulatedLen += segLen;
    }
    
    // End of path
    PathPoint pt = m_segments.back().segment->evaluate(1.0);
    pt.parameter = 1.0;
    return pt;
}

size_t MultiSegmentPath::getNumAxes() const {
    if (m_segments.empty()) {
        return 0;
    }
    return m_segments.front().segment->getNumAxes();
}

double MultiSegmentPath::getTotalLength() const {
    double total = 0.0;
    for (const auto& seg : m_segments) {
        total += seg.segment->getLength();
    }
    return total;
}

void MultiSegmentPath::clear() {
    m_segments.clear();
    m_totalDuration = 0.0;
}

// ============================================================================
// Factory
// ============================================================================

std::unique_ptr<PathSegment> createPathSegment(PathType type) {
    switch (type) {
        case PathType::Linear:
            return std::make_unique<LinearPath>();
        case PathType::Circular:
            return std::make_unique<CircularPath>();
        case PathType::Helical:
            return std::make_unique<HelicalPath>();
        case PathType::BSpline:
            return std::make_unique<BSplinePath>();
        case PathType::NURBS:
            return std::make_unique<NURBSPath>();
        case PathType::Bezier:
            return std::make_unique<BezierPath>();
        default:
            return std::make_unique<LinearPath>();
    }
}

} // namespace CiA402
