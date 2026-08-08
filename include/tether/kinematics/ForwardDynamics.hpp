/**
 * @file ForwardDynamics.hpp
 * @brief Comprehensive forward dynamics models for common robot types
 * 
 * This module provides forward dynamics (torque -> acceleration) for:
 * - Serial manipulators using Newton-Euler and Lagrangian methods
 * - 2-DOF, 3-DOF, 6-DOF configurations
 * - SCARA robots
 * - Mobile robots (differential drive, omnidirectional)
 * 
 * The dynamics equations compute:
 * - Mass/inertia matrices M(q)
 * - Coriolis/centrifugal terms C(q, q̇)
 * - Gravity terms g(q)
 * - Friction models
 * 
 * Forward dynamics: q̈ = M⁻¹(q) [τ - C(q, q̇)q̇ - g(q) - f(q̇)]
 * 
 * All implementations use SI units:
 * - Positions: meters, radians
 * - Velocities: m/s, rad/s
 * - Torques: Nm
 * - Forces: N
 * - Masses: kg
 * - Inertias: kg·m²
 * 
 * @copyright Public Domain (CC0)
 */
#pragma once

#include <cmath>
#include <array>
#include <cstdint>
#include <cstring>
#include "ForwardKinematics.hpp"
#include "LinearAlgebra.hpp"

namespace tether::kinematics {

// =============================================================================
// Physical Constants
// =============================================================================

constexpr float GRAVITY = 9.81f;  // m/s²

// =============================================================================
// Link Physical Properties
// =============================================================================

/**
 * @brief Physical properties of a robot link
 */
struct LinkProperties {
    float mass{1.0f};           // Mass (kg)
    float length{0.5f};         // Link length (m)
    float com_distance{0.25f};  // Distance from joint to center of mass (m)
    float inertia{0.01f};       // Moment of inertia about COM (kg·m²)
    
    // Motor/actuator properties
    float motor_inertia{0.001f};     // Motor rotor inertia (kg·m²)
    float gear_ratio{1.0f};          // Gear reduction ratio
    
    // Friction parameters
    float viscous_friction{0.1f};    // Viscous friction coefficient (Nm·s/rad)
    float coulomb_friction{0.05f};   // Coulomb friction (Nm)
    
    LinkProperties() = default;
    
    LinkProperties(float m, float L, float com, float I) 
        : mass(m), length(L), com_distance(com), inertia(I) {}
    
    /**
     * @brief Compute link inertia using parallel axis theorem
     * @return Inertia about joint axis
     */
    float jointInertia() const {
        return inertia + mass * com_distance * com_distance;
    }
    
    /**
     * @brief Compute effective inertia including motor
     * @return Total reflected inertia at joint
     */
    float effectiveInertia() const {
        return jointInertia() + motor_inertia * gear_ratio * gear_ratio;
    }
};

// =============================================================================
// Friction Models
// =============================================================================

/**
 * @brief Joint friction model
 * 
 * Computes friction torque: τ_f = b * q̇ + c * sign(q̇)
 * where b = viscous coefficient, c = coulomb friction
 */
class FrictionModel {
public:
    FrictionModel(float viscous = 0.1f, float coulomb = 0.05f, float stiction = 0.1f)
        : m_viscous(viscous), m_coulomb(coulomb), m_stiction(stiction) {}
    
    /**
     * @brief Compute friction torque
     * @param velocity Joint velocity (rad/s)
     * @return Friction torque (Nm)
     */
    float compute(float velocity) const {
        float viscous_term = m_viscous * velocity;
        
        // Smooth sign function to avoid discontinuity
        float sign_v = velocity / (std::abs(velocity) + 0.01f);
        float coulomb_term = m_coulomb * sign_v;
        
        return viscous_term + coulomb_term;
    }
    
    /**
     * @brief Compute friction with stiction (Stribeck model)
     * @param velocity Joint velocity
     * @param applied_torque Applied torque (for stiction check)
     * @return Friction torque
     */
    float computeStribeck(float velocity, float applied_torque) const {
        float abs_vel = std::abs(velocity);
        
        if (abs_vel < 0.001f) {
            // Near zero velocity - check stiction
            if (std::abs(applied_torque) < m_stiction) {
                return -applied_torque;  // Cancel applied torque
            }
        }
        
        // Stribeck effect: friction decreases from stiction to coulomb
        float stribeck_vel = 0.01f;  // Characteristic velocity
        float stribeck_factor = std::exp(-std::pow(abs_vel / stribeck_vel, 2));
        float effective_coulomb = m_coulomb + (m_stiction - m_coulomb) * stribeck_factor;
        
        float sign_v = velocity / (abs_vel + 0.001f);
        return m_viscous * velocity + effective_coulomb * sign_v;
    }
    
    float getViscous() const { return m_viscous; }
    float getCoulomb() const { return m_coulomb; }
    float getStiction() const { return m_stiction; }
    
    void setViscous(float v) { m_viscous = v; }
    void setCoulomb(float c) { m_coulomb = c; }
    void setStiction(float s) { m_stiction = s; }
    
private:
    float m_viscous;
    float m_coulomb;
    float m_stiction;
};

// =============================================================================
// 2-DOF Planar Arm Dynamics
// =============================================================================

/**
 * @brief 2-DOF planar arm dynamics
 * 
 * Complete dynamics model including:
 * - Inertia matrix M(q)
 * - Coriolis/centrifugal matrix C(q, q̇)
 * - Gravity vector g(q)
 * - Friction
 * 
 * Equations of motion:
 * M(q)q̈ + C(q, q̇)q̇ + g(q) + f(q̇) = τ
 */
class Planar2DOFDynamics {
public:
    /**
     * @brief Constructor
     * @param link1 Link 1 properties
     * @param link2 Link 2 properties
     * @param gravity_direction Gravity direction (default: -Y for vertical plane)
     */
    Planar2DOFDynamics(const LinkProperties& link1, const LinkProperties& link2,
                       const Position2D& gravity_dir = Position2D(0, -1))
        : m_link1(link1), m_link2(link2), m_gravity_dir(gravity_dir) {}
    
    /**
     * @brief Compute mass matrix M(q)
     * 
     * M = [M11  M12]
     *     [M21  M22]
     */
    void computeMassMatrix(float q1, float q2, float M[2][2]) const {
        float m1 = m_link1.mass;
        float m2 = m_link2.mass;
        float L1 = m_link1.length;
        float lc1 = m_link1.com_distance;
        float lc2 = m_link2.com_distance;
        float I1 = m_link1.inertia;
        float I2 = m_link2.inertia;
        
        float c2 = std::cos(q2);
        
        // Mass matrix elements
        M[0][0] = m1 * lc1 * lc1 + m2 * (L1 * L1 + lc2 * lc2 + 2 * L1 * lc2 * c2) + I1 + I2;
        M[0][1] = m2 * (lc2 * lc2 + L1 * lc2 * c2) + I2;
        M[1][0] = M[0][1];
        M[1][1] = m2 * lc2 * lc2 + I2;
    }
    
    /**
     * @brief Compute Coriolis/centrifugal matrix C(q, q̇)
     * 
     * Using Christoffel symbols approach
     */
    void computeCoriolisMatrix(float q1, float q2, float dq1, float dq2, 
                               float C[2][2]) const {
        float m2 = m_link2.mass;
        float L1 = m_link1.length;
        float lc2 = m_link2.com_distance;
        
        float s2 = std::sin(q2);
        float h = m2 * L1 * lc2 * s2;
        
        C[0][0] = -h * dq2;
        C[0][1] = -h * (dq1 + dq2);
        C[1][0] = h * dq1;
        C[1][1] = 0;
    }
    
    /**
     * @brief Compute gravity vector g(q)
     */
    void computeGravityVector(float q1, float q2, float g[2]) const {
        float m1 = m_link1.mass;
        float m2 = m_link2.mass;
        float L1 = m_link1.length;
        float lc1 = m_link1.com_distance;
        float lc2 = m_link2.com_distance;
        
        // For vertical plane motion (gravity in -Y direction)
        float g_mag = GRAVITY;
        
        float c1 = std::cos(q1);
        float c12 = std::cos(q1 + q2);
        
        g[0] = (m1 * lc1 + m2 * L1) * g_mag * c1 + m2 * lc2 * g_mag * c12;
        g[1] = m2 * lc2 * g_mag * c12;
    }
    
    /**
     * @brief Forward dynamics: compute joint accelerations
     * 
     * Given joint positions, velocities, and applied torques,
     * compute joint accelerations.
     * 
     * q̈ = M⁻¹(τ - Cq̇ - g - f)
     * 
     * @param q Joint positions [q1, q2]
     * @param dq Joint velocities [dq1, dq2]
     * @param tau Applied torques [τ1, τ2]
     * @param ddq Output accelerations [ddq1, ddq2]
     */
    void forwardDynamics(const float q[2], const float dq[2], 
                         const float tau[2], float ddq[2]) const {
        float M[2][2], C[2][2], g[2];
        
        computeMassMatrix(q[0], q[1], M);
        computeCoriolisMatrix(q[0], q[1], dq[0], dq[1], C);
        computeGravityVector(q[0], q[1], g);
        
        // Compute friction
        float friction[2] = {
            m_friction1.compute(dq[0]),
            m_friction2.compute(dq[1])
        };
        
        // Compute right-hand side: τ - Cq̇ - g - f
        float rhs[2];
        rhs[0] = tau[0] - (C[0][0] * dq[0] + C[0][1] * dq[1]) - g[0] - friction[0];
        rhs[1] = tau[1] - (C[1][0] * dq[0] + C[1][1] * dq[1]) - g[1] - friction[1];
        
        // Solve M * ddq = rhs using 2x2 matrix inverse
        float det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
        if (std::abs(det) < 1e-10f) {
            ddq[0] = ddq[1] = 0;
            return;
        }
        
        float inv_det = 1.0f / det;
        ddq[0] = inv_det * (M[1][1] * rhs[0] - M[0][1] * rhs[1]);
        ddq[1] = inv_det * (-M[1][0] * rhs[0] + M[0][0] * rhs[1]);
    }
    
    /**
     * @brief Inverse dynamics: compute required torques
     * 
     * Given joint positions, velocities, and desired accelerations,
     * compute required torques.
     * 
     * τ = M(q)q̈ + C(q,q̇)q̇ + g(q) + f(q̇)
     */
    void inverseDynamics(const float q[2], const float dq[2],
                         const float ddq[2], float tau[2]) const {
        float M[2][2], C[2][2], g[2];
        
        computeMassMatrix(q[0], q[1], M);
        computeCoriolisMatrix(q[0], q[1], dq[0], dq[1], C);
        computeGravityVector(q[0], q[1], g);
        
        // Compute friction
        float friction[2] = {
            m_friction1.compute(dq[0]),
            m_friction2.compute(dq[1])
        };
        
        // τ = Mq̈ + Cq̇ + g + f
        tau[0] = M[0][0] * ddq[0] + M[0][1] * ddq[1] + 
                 C[0][0] * dq[0] + C[0][1] * dq[1] + g[0] + friction[0];
        tau[1] = M[1][0] * ddq[0] + M[1][1] * ddq[1] + 
                 C[1][0] * dq[0] + C[1][1] * dq[1] + g[1] + friction[1];
    }
    
    /**
     * @brief Compute manipulator energy
     * @return [kinetic_energy, potential_energy]
     */
    std::array<float, 2> computeEnergy(const float q[2], const float dq[2]) const {
        float M[2][2];
        computeMassMatrix(q[0], q[1], M);
        
        // Kinetic energy: T = 0.5 * q̇ᵀMq̇
        float kinetic = 0.5f * (M[0][0] * dq[0] * dq[0] + 
                                2 * M[0][1] * dq[0] * dq[1] + 
                                M[1][1] * dq[1] * dq[1]);
        
        // Potential energy (assuming gravity in -Y)
        float m1 = m_link1.mass;
        float m2 = m_link2.mass;
        float L1 = m_link1.length;
        float lc1 = m_link1.com_distance;
        float lc2 = m_link2.com_distance;
        
        float h1 = lc1 * std::sin(q[0]);
        float h2 = L1 * std::sin(q[0]) + lc2 * std::sin(q[0] + q[1]);
        float potential = GRAVITY * (m1 * h1 + m2 * h2);
        
        return {kinetic, potential};
    }
    
    // Accessors
    const LinkProperties& getLink1() const { return m_link1; }
    const LinkProperties& getLink2() const { return m_link2; }
    void setLink1(const LinkProperties& link) { m_link1 = link; }
    void setLink2(const LinkProperties& link) { m_link2 = link; }
    
    FrictionModel& friction1() { return m_friction1; }
    FrictionModel& friction2() { return m_friction2; }
    
private:
    LinkProperties m_link1;
    LinkProperties m_link2;
    Position2D m_gravity_dir;
    FrictionModel m_friction1;
    FrictionModel m_friction2;
};

// =============================================================================
// 3-DOF Articulated Arm Dynamics
// =============================================================================

/**
 * @brief 3-DOF articulated arm dynamics
 * 
 * Typical configuration: base rotation + shoulder pitch + elbow pitch
 */
class Articulated3DOFDynamics {
public:
    Articulated3DOFDynamics() {
        // Default link properties
        m_links[0] = LinkProperties(2.0f, 0.3f, 0.15f, 0.02f);
        m_links[1] = LinkProperties(1.5f, 0.4f, 0.2f, 0.015f);
        m_links[2] = LinkProperties(1.0f, 0.3f, 0.15f, 0.01f);
    }
    
    void setLinkProperties(int index, const LinkProperties& props) {
        if (index >= 0 && index < 3) {
            m_links[index] = props;
        }
    }
    
    /**
     * @brief Compute 3x3 mass matrix
     */
    void computeMassMatrix(const float q[3], float M[3][3]) const {
        // Simplified computation for 3-DOF RRP arm
        float m1 = m_links[0].mass;
        float m2 = m_links[1].mass;
        float m3 = m_links[2].mass;
        float L1 = m_links[0].length;
        float L2 = m_links[1].length;
        float L3 = m_links[2].length;
        float lc1 = m_links[0].com_distance;
        float lc2 = m_links[1].com_distance;
        float lc3 = m_links[2].com_distance;
        float I1 = m_links[0].inertia;
        float I2 = m_links[1].inertia;
        float I3 = m_links[2].inertia;
        
        float c2 = std::cos(q[1]);
        float c3 = std::cos(q[2]);
        float c23 = std::cos(q[1] + q[2]);
        float s2 = std::sin(q[1]);
        float s3 = std::sin(q[2]);
        
        // Initialize to zero
        std::memset(M, 0, 9 * sizeof(float));
        
        // Diagonal terms (simplified)
        M[0][0] = I1 + I2 + I3 + m2 * lc2 * lc2 + m3 * (L2 * L2 + lc3 * lc3);
        M[1][1] = I2 + I3 + m2 * lc2 * lc2 + m3 * (L2 * L2 + lc3 * lc3 + 2 * L2 * lc3 * c3);
        M[2][2] = I3 + m3 * lc3 * lc3;
        
        // Off-diagonal terms
        M[1][2] = M[2][1] = I3 + m3 * (lc3 * lc3 + L2 * lc3 * c3);
    }
    
    /**
     * @brief Forward dynamics
     */
    void forwardDynamics(const float q[3], const float dq[3],
                         const float tau[3], float ddq[3]) const {
        float M[3][3], g[3];
        computeMassMatrix(q, M);
        computeGravityVector(q, g);
        
        // Compute Coriolis terms (simplified)
        float h = m_links[2].mass * m_links[1].length * m_links[2].com_distance * std::sin(q[2]);
        float C_vel[3] = {
            0,
            -h * dq[2] * (2 * dq[1] + dq[2]),
            h * dq[1] * dq[1]
        };
        
        // Compute friction
        float friction[3] = {
            m_friction[0].compute(dq[0]),
            m_friction[1].compute(dq[1]),
            m_friction[2].compute(dq[2])
        };
        
        // RHS: τ - C - g - f
        float rhs[3];
        for (int i = 0; i < 3; ++i) {
            rhs[i] = tau[i] - C_vel[i] - g[i] - friction[i];
        }
        
        // Solve 3x3 system (using Gaussian elimination with partial pivoting)
        tether::kinematics::solve3x3(M, rhs, ddq);
    }
    
    void inverseDynamics(const float q[3], const float dq[3],
                         const float ddq[3], float tau[3]) const {
        float M[3][3], g[3];
        computeMassMatrix(q, M);
        computeGravityVector(q, g);
        
        float h = m_links[2].mass * m_links[1].length * m_links[2].com_distance * std::sin(q[2]);
        float C_vel[3] = {
            0,
            -h * dq[2] * (2 * dq[1] + dq[2]),
            h * dq[1] * dq[1]
        };
        
        float friction[3] = {
            m_friction[0].compute(dq[0]),
            m_friction[1].compute(dq[1]),
            m_friction[2].compute(dq[2])
        };
        
        for (int i = 0; i < 3; ++i) {
            tau[i] = g[i] + friction[i] + C_vel[i];
            for (int j = 0; j < 3; ++j) {
                tau[i] += M[i][j] * ddq[j];
            }
        }
    }
    
    FrictionModel& friction(int index) { return m_friction[index]; }
    
private:
    LinkProperties m_links[3];
    FrictionModel m_friction[3];
    
    void computeGravityVector(const float q[3], float g[3]) const {
        float m1 = m_links[0].mass;
        float m2 = m_links[1].mass;
        float m3 = m_links[2].mass;
        float L1 = m_links[0].length;
        float L2 = m_links[1].length;
        float lc1 = m_links[0].com_distance;
        float lc2 = m_links[1].com_distance;
        float lc3 = m_links[2].com_distance;
        
        float c2 = std::cos(q[1]);
        float c23 = std::cos(q[1] + q[2]);
        
        g[0] = 0;  // Base rotation doesn't affect gravity
        g[1] = GRAVITY * ((m2 * lc2 + m3 * L2) * c2 + m3 * lc3 * c23);
        g[2] = GRAVITY * m3 * lc3 * c23;
    }
};

// =============================================================================
// Single Joint Dynamics (for Motor Control)
// =============================================================================

/**
 * @brief Single joint (motor) dynamics model
 * 
 * Useful for:
 * - Individual axis control
 * - CiA 402 drive simulation
 * - Motor tuning and identification
 * 
 * Equation: J_eff * θ̈ = τ - b * θ̇ - c * sign(θ̇) - τ_load
 */
class SingleJointDynamics {
public:
    /**
     * @brief Constructor
     * @param inertia Effective joint inertia (kg·m²)
     * @param viscous Viscous friction (Nm·s/rad)
     * @param coulomb Coulomb friction (Nm)
     */
    SingleJointDynamics(float inertia = 0.01f, float viscous = 0.1f, float coulomb = 0.05f)
        : m_inertia(inertia), m_friction(viscous, coulomb, coulomb * 1.5f) {}
    
    /**
     * @brief Forward dynamics: torque to acceleration
     * @param position Current position (rad)
     * @param velocity Current velocity (rad/s)
     * @param torque Applied motor torque (Nm)
     * @param load_torque External load torque (Nm)
     * @return Angular acceleration (rad/s²)
     */
    float forwardDynamics(float position, float velocity, 
                          float torque, float load_torque = 0) const {
        float friction_torque = m_friction.compute(velocity);
        float net_torque = torque - friction_torque - load_torque;
        return net_torque / m_inertia;
    }
    
    /**
     * @brief Inverse dynamics: desired acceleration to torque
     */
    float inverseDynamics(float position, float velocity,
                          float acceleration, float load_torque = 0) const {
        float friction_torque = m_friction.compute(velocity);
        return m_inertia * acceleration + friction_torque + load_torque;
    }
    
    /**
     * @brief Simulate one time step using Euler integration
     */
    void simulate(float& position, float& velocity, float torque, float dt,
                  float load_torque = 0) {
        float accel = forwardDynamics(position, velocity, torque, load_torque);
        velocity += accel * dt;
        position += velocity * dt;
    }
    
    /**
     * @brief Simulate with RK4 integration (more accurate)
     */
    void simulateRK4(float& position, float& velocity, float torque, float dt,
                     float load_torque = 0) {
        auto derivative = [this, torque, load_torque](float pos, float vel) {
            float accel = forwardDynamics(pos, vel, torque, load_torque);
            return std::make_pair(vel, accel);
        };
        
        auto [k1_v, k1_a] = derivative(position, velocity);
        auto [k2_v, k2_a] = derivative(position + 0.5f * dt * k1_v, velocity + 0.5f * dt * k1_a);
        auto [k3_v, k3_a] = derivative(position + 0.5f * dt * k2_v, velocity + 0.5f * dt * k2_a);
        auto [k4_v, k4_a] = derivative(position + dt * k3_v, velocity + dt * k3_a);
        
        position += dt * (k1_v + 2*k2_v + 2*k3_v + k4_v) / 6.0f;
        velocity += dt * (k1_a + 2*k2_a + 2*k3_a + k4_a) / 6.0f;
    }
    
    // Accessors
    float getInertia() const { return m_inertia; }
    void setInertia(float J) { m_inertia = J; }
    FrictionModel& friction() { return m_friction; }
    
    /**
     * @brief Compute mechanical time constant
     * @return Time constant τ_m = J / b (seconds)
     */
    float getMechanicalTimeConstant() const {
        float b = m_friction.getViscous();
        return (b > 1e-6f) ? m_inertia / b : 1e6f;
    }
    
private:
    float m_inertia;
    FrictionModel m_friction;
};

// =============================================================================
// SCARA Robot Dynamics
// =============================================================================

/**
 * @brief 4-DOF SCARA robot dynamics
 */
class SCARADynamics {
public:
    SCARADynamics() {
        m_link1 = LinkProperties(2.0f, 0.35f, 0.175f, 0.02f);
        m_link2 = LinkProperties(1.5f, 0.35f, 0.175f, 0.015f);
        m_z_axis = SingleJointDynamics(0.5f, 0.2f, 0.1f);  // Z slide
        m_tool = SingleJointDynamics(0.005f, 0.01f, 0.005f);  // Tool rotation
    }
    
    /**
     * @brief Forward dynamics for SCARA (RRPR configuration)
     * 
     * Joint 1, 2: Rotary (coupled planar dynamics)
     * Joint 3: Prismatic (Z axis)
     * Joint 4: Rotary (tool rotation, decoupled)
     */
    void forwardDynamics(const float q[4], const float dq[4],
                         const float tau[4], float ddq[4]) const {
        // Joints 1,2: Use planar 2-DOF dynamics
        float q_planar[2] = {q[0], q[1]};
        float dq_planar[2] = {dq[0], dq[1]};
        float tau_planar[2] = {tau[0], tau[1]};
        float ddq_planar[2];
        
        Planar2DOFDynamics planar(m_link1, m_link2, Position2D(0, 0));  // Horizontal
        planar.forwardDynamics(q_planar, dq_planar, tau_planar, ddq_planar);
        ddq[0] = ddq_planar[0];
        ddq[1] = ddq_planar[1];
        
        // Joint 3: Z axis (prismatic, decoupled)
        float z_load = (m_link1.mass + m_link2.mass) * GRAVITY;  // Weight
        ddq[2] = m_z_axis.forwardDynamics(q[2], dq[2], tau[2], -z_load);
        
        // Joint 4: Tool rotation (decoupled)
        ddq[3] = m_tool.forwardDynamics(q[3], dq[3], tau[3]);
    }
    
private:
    LinkProperties m_link1, m_link2;
    SingleJointDynamics m_z_axis;
    SingleJointDynamics m_tool;
};

// =============================================================================
// Mobile Robot Dynamics
// =============================================================================

/**
 * @brief Differential drive mobile robot dynamics
 * 
 * Models wheel dynamics and robot body dynamics including:
 * - Wheel motor dynamics
 * - Robot body inertia
 * - Ground friction
 */
class DifferentialDriveDynamics {
public:
    /**
     * @brief Constructor
     * @param robot_mass Robot mass (kg)
     * @param robot_inertia Robot moment of inertia about vertical axis (kg·m²)
     * @param wheel_radius Wheel radius (m)
     * @param wheel_base Distance between wheels (m)
     * @param wheel_inertia Wheel moment of inertia (kg·m²)
     */
    DifferentialDriveDynamics(float robot_mass = 10.0f, float robot_inertia = 0.5f,
                              float wheel_radius = 0.05f, float wheel_base = 0.3f,
                              float wheel_inertia = 0.001f)
        : m_mass(robot_mass), m_inertia(robot_inertia),
          m_wheel_radius(wheel_radius), m_wheel_base(wheel_base),
          m_wheel_inertia(wheel_inertia),
          m_left_wheel(wheel_inertia, 0.01f, 0.005f),
          m_right_wheel(wheel_inertia, 0.01f, 0.005f) {}
    
    /**
     * @brief Forward dynamics
     * 
     * Given wheel torques, compute robot accelerations.
     * 
     * @param v Current linear velocity (m/s)
     * @param omega Current angular velocity (rad/s)
     * @param tau_left Left wheel torque (Nm)
     * @param tau_right Right wheel torque (Nm)
     * @return [linear_accel, angular_accel]
     */
    std::array<float, 2> forwardDynamics(float v, float omega,
                                          float tau_left, float tau_right) const {
        // Convert wheel torques to forces at wheel contact points
        float F_left = tau_left / m_wheel_radius;
        float F_right = tau_right / m_wheel_radius;
        
        // Total force and torque on robot body
        float F_total = F_left + F_right;
        float T_total = (F_right - F_left) * m_wheel_base / 2.0f;
        
        // Add rolling resistance
        float rolling_resistance = 0.01f * m_mass * GRAVITY * (v > 0 ? 1 : -1);
        
        // Accelerations
        float a_linear = (F_total - rolling_resistance) / m_mass;
        float a_angular = T_total / m_inertia;
        
        return {a_linear, a_angular};
    }
    
    /**
     * @brief Simulate robot motion
     */
    void simulate(Pose2D& pose, float& v, float& omega,
                  float tau_left, float tau_right, float dt) {
        auto [a_linear, a_angular] = forwardDynamics(v, omega, tau_left, tau_right);
        
        // Update velocities
        v += a_linear * dt;
        omega += a_angular * dt;
        
        // Update pose
        DifferentialDrive kinematics(m_wheel_radius, m_wheel_base);
        pose = kinematics.updatePose(pose, v, omega, dt);
    }
    
private:
    float m_mass;
    float m_inertia;
    float m_wheel_radius;
    float m_wheel_base;
    float m_wheel_inertia;
    SingleJointDynamics m_left_wheel;
    SingleJointDynamics m_right_wheel;
};

// =============================================================================
// Newton-Euler Recursive Dynamics
// =============================================================================

/**
 * @brief Spatial velocity (twist)
 */
struct SpatialVelocity {
    Position3D angular;  // ω
    Position3D linear;   // v
};

/**
 * @brief Spatial acceleration
 */
struct SpatialAcceleration {
    Position3D angular;  // α
    Position3D linear;   // a
};

/**
 * @brief Spatial force (wrench)
 */
struct SpatialForce {
    Position3D torque;
    Position3D force;
};

/**
 * @brief Link spatial inertia
 */
struct SpatialInertia {
    float mass;
    Position3D com;      // Center of mass in link frame
    float Ixx, Iyy, Izz; // Principal moments of inertia
    float Ixy, Ixz, Iyz; // Products of inertia
    
    SpatialInertia() : mass(1.0f), Ixx(0.01f), Iyy(0.01f), Izz(0.01f),
                       Ixy(0), Ixz(0), Iyz(0) {}
};

/**
 * @brief Newton-Euler recursive dynamics for serial robots
 * 
 * Efficient O(n) algorithm for computing inverse dynamics.
 * Forward pass: compute link velocities and accelerations
 * Backward pass: compute link forces and joint torques
 * 
 * @tparam N Number of DOF (compile-time)
 */
template <size_t N>
class NewtonEulerDynamics {
public:
    NewtonEulerDynamics() {
        for (size_t i = 0; i < N; ++i) {
            m_joint_axis[i] = Position3D(0, 0, 1);  // Default: Z-axis rotation
            m_joint_type[i] = JointType::Revolute;
        }
    }
    
    enum class JointType { Revolute, Prismatic };
    
    void setJointAxis(size_t joint, const Position3D& axis) {
        if (joint < N) m_joint_axis[joint] = axis;
    }
    
    void setJointType(size_t joint, JointType type) {
        if (joint < N) m_joint_type[joint] = type;
    }
    
    void setLinkInertia(size_t link, const SpatialInertia& inertia) {
        if (link < N) m_inertias[link] = inertia;
    }
    
    /**
     * @brief Inverse dynamics using Newton-Euler algorithm
     * 
     * @param q Joint positions
     * @param dq Joint velocities
     * @param ddq Joint accelerations
     * @param tau Output joint torques
     * @param f_ext External forces on each link (optional)
     */
    void inverseDynamics(const float q[N], const float dq[N], const float ddq[N],
                         float tau[N], const SpatialForce* f_ext = nullptr) const {
        // Arrays for intermediate values
        SpatialVelocity v[N + 1];      // Link velocities
        SpatialAcceleration a[N + 1];  // Link accelerations
        SpatialForce f[N];             // Link forces
        
        // Base velocity and acceleration (stationary base)
        v[0].angular = Position3D(0, 0, 0);
        v[0].linear = Position3D(0, 0, 0);
        a[0].angular = Position3D(0, 0, 0);
        a[0].linear = Position3D(0, GRAVITY, 0);  // Gravity as base acceleration
        
        // Forward pass: compute velocities and accelerations
        for (size_t i = 0; i < N; ++i) {
            const Position3D& z = m_joint_axis[i];
            
            if (m_joint_type[i] == JointType::Revolute) {
                // Revolute joint
                v[i + 1].angular = v[i].angular + z * dq[i];
                v[i + 1].linear = v[i].linear;  // Simplified
                
                a[i + 1].angular = a[i].angular + z * ddq[i] + 
                                   v[i].angular.cross(z) * dq[i];
                a[i + 1].linear = a[i].linear + 
                                  a[i + 1].angular.cross(m_inertias[i].com) +
                                  v[i + 1].angular.cross(v[i + 1].angular.cross(m_inertias[i].com));
            } else {
                // Prismatic joint
                v[i + 1].angular = v[i].angular;
                v[i + 1].linear = v[i].linear + z * dq[i] + 
                                  v[i].angular.cross(z) * dq[i];
                
                a[i + 1].angular = a[i].angular;
                a[i + 1].linear = a[i].linear + z * ddq[i] + 
                                  2.0f * v[i].angular.cross(z) * dq[i];
            }
        }
        
        // Backward pass: compute forces and torques
        SpatialForce f_next = {Position3D(0, 0, 0), Position3D(0, 0, 0)};
        
        for (int i = N - 1; i >= 0; --i) {
            const SpatialInertia& I = m_inertias[i];
            
            // Link force
            f[i].force = I.mass * a[i + 1].linear;
            
            // Link torque (simplified inertia tensor)
            f[i].torque.x = I.Ixx * a[i + 1].angular.x;
            f[i].torque.y = I.Iyy * a[i + 1].angular.y;
            f[i].torque.z = I.Izz * a[i + 1].angular.z;
            
            // Add velocity-dependent terms
            f[i].torque = f[i].torque + v[i + 1].angular.cross(
                Position3D(I.Ixx * v[i + 1].angular.x,
                          I.Iyy * v[i + 1].angular.y,
                          I.Izz * v[i + 1].angular.z));
            
            // Add force from next link
            f[i].force = f[i].force + f_next.force;
            f[i].torque = f[i].torque + f_next.torque + m_inertias[i].com.cross(f_next.force);
            
            // Subtract external force if provided
            if (f_ext != nullptr) {
                f[i].force = f[i].force - f_ext[i].force;
                f[i].torque = f[i].torque - f_ext[i].torque;
            }
            
            // Joint torque
            const Position3D& z = m_joint_axis[i];
            if (m_joint_type[i] == JointType::Revolute) {
                tau[i] = f[i].torque.dot(z);
            } else {
                tau[i] = f[i].force.dot(z);
            }
            
            f_next = f[i];
        }
    }
    
private:
    Position3D m_joint_axis[N];
    JointType m_joint_type[N];
    SpatialInertia m_inertias[N];
};

// =============================================================================
// Computed Torque Controller
// =============================================================================

/**
 * @brief Computed torque controller for trajectory tracking
 * 
 * Uses inverse dynamics to compute feedforward torque, plus PD feedback.
 * Control law: τ = M(q)[q̈_d + Kp(q_d - q) + Kd(q̇_d - q̇)] + C(q,q̇)q̇ + g(q)
 * 
 * @tparam N Number of DOF
 */
template <size_t N>
class ComputedTorqueController {
public:
    ComputedTorqueController() {
        for (size_t i = 0; i < N; ++i) {
            m_Kp[i] = 100.0f;
            m_Kd[i] = 20.0f;
        }
    }
    
    void setGains(const float Kp[N], const float Kd[N]) {
        for (size_t i = 0; i < N; ++i) {
            m_Kp[i] = Kp[i];
            m_Kd[i] = Kd[i];
        }
    }
    
    void setGains(float Kp, float Kd) {
        for (size_t i = 0; i < N; ++i) {
            m_Kp[i] = Kp;
            m_Kd[i] = Kd;
        }
    }
    
    /**
     * @brief Compute control torque
     * 
     * @param q_desired Desired position
     * @param dq_desired Desired velocity
     * @param ddq_desired Desired acceleration (feedforward)
     * @param q_actual Actual position
     * @param dq_actual Actual velocity
     * @param tau Output control torque
     * @param dynamics_model Function to compute inverse dynamics
     */
    template <typename DynamicsFunc>
    void compute(const float q_desired[N], const float dq_desired[N],
                 const float ddq_desired[N],
                 const float q_actual[N], const float dq_actual[N],
                 float tau[N], DynamicsFunc& dynamics_model) const {
        
        // Compute error-corrected acceleration
        float ddq_cmd[N];
        for (size_t i = 0; i < N; ++i) {
            float pos_error = q_desired[i] - q_actual[i];
            float vel_error = dq_desired[i] - dq_actual[i];
            ddq_cmd[i] = ddq_desired[i] + m_Kp[i] * pos_error + m_Kd[i] * vel_error;
        }
        
        // Use inverse dynamics to compute required torque
        dynamics_model.inverseDynamics(q_actual, dq_actual, ddq_cmd, tau);
    }
    
private:
    float m_Kp[N];
    float m_Kd[N];
};

} // namespace tether::kinematics
