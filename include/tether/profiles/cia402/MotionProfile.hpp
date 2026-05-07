/**
 * @file MotionProfile.hpp
 * @brief Abstract motion profile interface and base implementations
 * 
 * @details
 * Provides motion profile generation for CiA 402 drives:
 * - Linear (constant velocity)
 * - Trapezoidal (acceleration/coast/deceleration)
 * - Triangular (no coast phase)
 * - S-curve (jerk-limited)
 * - Polynomial (arbitrary order)
 * 
 * ## Motion Profile Comparison
 * 
 * ```
 * Position:     Velocity:      Acceleration:   Jerk:
 * 
 * Linear:
 *   /           ─────          (none)          (none)
 *  /
 * 
 * Trapezoidal:
 *    ___        ─────           ┌───┐          (infinite)
 *   /   \      /     \          │   └───┐
 *  /     \    /       \    ─────┘       │
 *                          ─────────────┘
 * 
 * S-Curve:
 *    ___        ─────           ___            ┌──┐
 *   /   \      /     \         /   \        ┌──┘  │
 *  /     \    /       \    ───/     \───    │     └──┐
 *                          ───       ───  ──┘        └──
 * ```
 * 
 * ## Usage Example
 * 
 * ```cpp
 * #include "MotionProfile.hpp"
 * 
 * // Create trapezoidal profile
 * TrapezoidalProfile profile;
 * profile.setLimits(1000.0, 10000.0, 10000.0);  // vel, acc, dec
 * profile.plan(0, 10000);  // from 0 to 10000
 * 
 * // Sample at each time step
 * while (!profile.isComplete()) {
 *     MotionState state = profile.evaluate(currentTime);
 *     setTargetPosition(state.position);
 *     currentTime += cycleTime;
 * }
 * ```
 */

#pragma once

#include "CiA402Config.hpp"
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <memory>

namespace CiA402 {

/**
 * @brief Motion state at a given time
 */
struct MotionState {
    double position{0.0};       ///< Position [user units]
    double velocity{0.0};       ///< Velocity [user units/s]
    double acceleration{0.0};   ///< Acceleration [user units/s²]
    double jerk{0.0};           ///< Jerk [user units/s³]
    double time{0.0};           ///< Time from start [s]
    bool complete{false};       ///< Profile complete flag
};

/**
 * @brief Motion limits
 */
struct MotionLimits {
    double maxVelocity{CIA402_DEFAULT_MAX_VELOCITY};
    double maxAcceleration{CIA402_DEFAULT_MAX_ACCELERATION};
    double maxDeceleration{CIA402_DEFAULT_MAX_DECELERATION};
    double maxJerk{CIA402_DEFAULT_MAX_JERK};
};

/**
 * @brief Profile type enumeration
 */
enum class ProfileType {
    Linear,
    Trapezoidal,
    Triangular,
    SCurve,
    Polynomial,
    Custom
};

/**
 * @brief Abstract motion profile interface
 */
class MotionProfile {
public:
    virtual ~MotionProfile() = default;
    
    /**
     * @brief Get profile type
     */
    virtual ProfileType getType() const = 0;
    
    /**
     * @brief Set motion limits
     */
    virtual void setLimits(const MotionLimits& limits);
    
    /**
     * @brief Get motion limits
     */
    const MotionLimits& getLimits() const { return m_limits; }
    
    /**
     * @brief Set velocity limit
     */
    void setMaxVelocity(double velocity) { m_limits.maxVelocity = velocity; }
    
    /**
     * @brief Set acceleration limit
     */
    void setMaxAcceleration(double accel) { m_limits.maxAcceleration = accel; }
    
    /**
     * @brief Set deceleration limit
     */
    void setMaxDeceleration(double decel) { m_limits.maxDeceleration = decel; }
    
    /**
     * @brief Set jerk limit
     */
    void setMaxJerk(double jerk) { m_limits.maxJerk = jerk; }
    
    /**
     * @brief Plan motion from current to target position
     * 
     * @param startPos Starting position
     * @param endPos Target position
     * @param startVel Starting velocity (default 0)
     * @param endVel Target velocity (default 0)
     * @return Duration of profile in seconds
     */
    virtual double plan(double startPos, double endPos, 
                       double startVel = 0.0, double endVel = 0.0) = 0;
    
    /**
     * @brief Evaluate profile at given time
     * 
     * @param time Time from profile start [s]
     * @return Motion state at given time
     */
    virtual MotionState evaluate(double time) const = 0;
    
    /**
     * @brief Get total profile duration
     */
    virtual double getDuration() const = 0;
    
    /**
     * @brief Check if profile is complete
     * 
     * @param currentTime Current time from start
     */
    virtual bool isComplete(double currentTime) const {
        return currentTime >= getDuration();
    }
    
    /**
     * @brief Get start position
     */
    double getStartPosition() const { return m_startPos; }
    
    /**
     * @brief Get end position
     */
    double getEndPosition() const { return m_endPos; }
    
    /**
     * @brief Get distance (signed)
     */
    double getDistance() const { return m_endPos - m_startPos; }
    
    /**
     * @brief Apply global speed factor
     * 
     * @param factor Speed factor (0.0 to 1.0, negative for reverse)
     */
    void setSpeedFactor(double factor) { m_speedFactor = factor; }
    
    /**
     * @brief Get current speed factor
     */
    double getSpeedFactor() const { return m_speedFactor; }
    
protected:
    MotionLimits m_limits;
    double m_startPos{0.0};
    double m_endPos{0.0};
    double m_startVel{0.0};
    double m_endVel{0.0};
    double m_duration{0.0};
    double m_speedFactor{1.0};
    int m_direction{1};  // +1 or -1
};

// ============================================================================
// Linear Profile (Constant Velocity)
// ============================================================================

/**
 * @brief Linear motion profile (constant velocity)
 * 
 * Simple profile with instant acceleration/deceleration.
 * Useful for very slow motions or as building block.
 */
class LinearProfile : public MotionProfile {
public:
    ProfileType getType() const override { return ProfileType::Linear; }
    
    double plan(double startPos, double endPos, 
               double startVel = 0.0, double endVel = 0.0) override;
    
    MotionState evaluate(double time) const override;
    
    double getDuration() const override { return m_duration; }
    
private:
    double m_velocity{0.0};
};

// ============================================================================
// Trapezoidal Profile
// ============================================================================

/**
 * @brief Trapezoidal motion profile
 * 
 * Three phases: acceleration, constant velocity, deceleration.
 * May degrade to triangular if distance is too short.
 * 
 * ```
 * Velocity
 *    ^
 *    │     ┌─────────┐
 * Vm │    /│         │\
 *    │   / │         │ \
 *    │  /  │         │  \
 *    │ /   │         │   \
 * ───┴─────┴─────────┴────► Time
 *    t0   t1        t2   t3
 *     acc   coast     dec
 * ```
 */
class TrapezoidalProfile : public MotionProfile {
public:
    ProfileType getType() const override { return ProfileType::Trapezoidal; }
    
    double plan(double startPos, double endPos, 
               double startVel = 0.0, double endVel = 0.0) override;
    
    MotionState evaluate(double time) const override;
    
    double getDuration() const override { return m_duration; }
    
    /**
     * @brief Check if profile is triangular (no coast phase)
     */
    bool isTriangular() const { return m_isTriangular; }
    
    /**
     * @brief Get phase times
     */
    void getPhaseTimes(double& accelTime, double& coastTime, double& decelTime) const {
        accelTime = m_t1;
        coastTime = m_t2 - m_t1;
        decelTime = m_duration - m_t2;
    }
    
private:
    double m_t1{0.0};           // End of acceleration phase
    double m_t2{0.0};           // End of coast phase
    double m_peakVelocity{0.0}; // Velocity during coast (or peak for triangular)
    double m_accel{0.0};        // Acceleration used
    double m_decel{0.0};        // Deceleration used
    bool m_isTriangular{false};
};

// ============================================================================
// S-Curve Profile (Jerk-Limited)
// ============================================================================

/**
 * @brief S-Curve motion profile (jerk-limited)
 * 
 * Seven phases for smooth motion with bounded jerk:
 * 1. Increasing acceleration (positive jerk)
 * 2. Constant acceleration
 * 3. Decreasing acceleration (negative jerk)
 * 4. Constant velocity
 * 5. Increasing deceleration (negative jerk)
 * 6. Constant deceleration
 * 7. Decreasing deceleration (positive jerk)
 * 
 * ```
 * Acceleration
 *    ^
 * Am │   ┌─────┐
 *    │  /│     │\
 *    │ / │     │ \
 * ───┼──────────────────────► Time
 *    │           \│     │/
 *-Am │            └─────┘
 * ```
 */
class SCurveProfile : public MotionProfile {
public:
    ProfileType getType() const override { return ProfileType::SCurve; }
    
    double plan(double startPos, double endPos, 
               double startVel = 0.0, double endVel = 0.0) override;
    
    MotionState evaluate(double time) const override;
    
    double getDuration() const override { return m_duration; }
    
    /**
     * @brief Get number of phases actually used
     */
    int getPhaseCount() const { return m_phaseCount; }
    
    /**
     * @brief Get phase end times
     */
    const double* getPhaseTimes() const { return m_t; }
    
private:
    /**
     * @brief Calculate phase durations
     */
    void calculatePhases();
    
    /**
     * @brief Evaluate single phase
     */
    MotionState evaluatePhase(int phase, double t) const;
    
    // Phase end times (t[0] = 0, t[7] = duration)
    double m_t[8]{0};
    
    // Jerk values for each phase
    double m_j[7]{0};
    
    // Initial conditions at start of each phase
    double m_p0[7]{0};  // Position
    double m_v0[7]{0};  // Velocity
    double m_a0[7]{0};  // Acceleration
    
    int m_phaseCount{7};
    double m_jerk{0.0};
    double m_accel{0.0};
    double m_peakVel{0.0};
};

// ============================================================================
// Triangular Profile
// ============================================================================

/**
 * @brief Triangular motion profile
 * 
 * Two phases: acceleration and deceleration (no coast).
 * Used for short moves where max velocity cannot be reached.
 */
class TriangularProfile : public MotionProfile {
public:
    ProfileType getType() const override { return ProfileType::Triangular; }
    
    double plan(double startPos, double endPos, 
               double startVel = 0.0, double endVel = 0.0) override;
    
    MotionState evaluate(double time) const override;
    
    double getDuration() const override { return m_duration; }
    
private:
    double m_t1{0.0};           // Time at peak velocity
    double m_peakVelocity{0.0}; // Peak velocity reached
    double m_accel{0.0};
    double m_decel{0.0};
};

// ============================================================================
// Polynomial Profile
// ============================================================================

/**
 * @brief Polynomial motion profile
 * 
 * Uses polynomial interpolation between boundary conditions.
 * Common orders:
 * - 3rd order (cubic): Position and velocity at boundaries
 * - 5th order (quintic): Position, velocity, acceleration at boundaries
 * - 7th order: Position, velocity, acceleration, jerk at boundaries
 * 
 * ```
 * For 5th order polynomial:
 * p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
 * 
 * With boundary conditions:
 * p(0) = p0, v(0) = v0, a(0) = a0
 * p(T) = pf, v(T) = vf, a(T) = af
 * ```
 */
class PolynomialProfile : public MotionProfile {
public:
    /**
     * @brief Polynomial order
     */
    enum class Order {
        Cubic = 3,      ///< Position + velocity boundaries
        Quintic = 5,    ///< Position + velocity + acceleration boundaries
        Septic = 7      ///< Position + velocity + acceleration + jerk boundaries
    };
    
    /**
     * @brief Constructor
     * 
     * @param order Polynomial order (3, 5, or 7)
     */
    explicit PolynomialProfile(Order order = Order::Quintic);
    
    ProfileType getType() const override { return ProfileType::Polynomial; }
    
    /**
     * @brief Set desired duration
     */
    void setDuration(double duration) { m_desiredDuration = duration; }
    
    /**
     * @brief Set boundary accelerations (for quintic and above)
     */
    void setBoundaryAcceleration(double startAccel, double endAccel) {
        m_startAccel = startAccel;
        m_endAccel = endAccel;
    }
    
    /**
     * @brief Set boundary jerks (for septic)
     */
    void setBoundaryJerk(double startJerk, double endJerk) {
        m_startJerk = startJerk;
        m_endJerk = endJerk;
    }
    
    double plan(double startPos, double endPos, 
               double startVel = 0.0, double endVel = 0.0) override;
    
    MotionState evaluate(double time) const override;
    
    double getDuration() const override { return m_duration; }
    
    /**
     * @brief Get polynomial coefficients
     */
    const double* getCoefficients() const { return m_coeff; }
    
private:
    /**
     * @brief Calculate coefficients for cubic polynomial
     */
    void calculateCubic();
    
    /**
     * @brief Calculate coefficients for quintic polynomial
     */
    void calculateQuintic();
    
    /**
     * @brief Calculate coefficients for septic polynomial
     */
    void calculateSeptic();
    
    Order m_order;
    double m_coeff[8]{0};       // Polynomial coefficients
    double m_desiredDuration{0.0};
    double m_startAccel{0.0};
    double m_endAccel{0.0};
    double m_startJerk{0.0};
    double m_endJerk{0.0};
};

// ============================================================================
// Profile Factory
// ============================================================================

/**
 * @brief Create motion profile by type
 * 
 * @param type Profile type
 * @return Unique pointer to profile
 */
std::unique_ptr<MotionProfile> createProfile(ProfileType type);

/**
 * @brief Select optimal profile for given move
 * 
 * @param distance Move distance
 * @param limits Motion limits
 * @return Optimal profile type
 */
ProfileType selectOptimalProfile(double distance, const MotionLimits& limits);

} // namespace CiA402
