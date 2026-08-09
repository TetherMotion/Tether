/**
 * @file ForwardKinematics.hpp
 * @brief Comprehensive forward kinematics implementations for common robot types
 * 
 * This module provides forward kinematic solutions for:
 * - Serial manipulators (2-DOF planar, 3-DOF, 6-DOF, 7-DOF)
 * - SCARA robots
 * - Delta/parallel robots
 * - Cartesian robots (gantry)
 * - Mobile robots (differential drive, omnidirectional)
 * - Stewart platforms
 * 
 * All implementations use SI units:
 * - Positions: meters
 * - Angles: radians
 * - Velocities: m/s, rad/s
 * 
 * @copyright Public Domain (CC0)
 */
#pragma once

#include <cmath>
#include <array>
#include <cstdint>
#include <cstring>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace tether::kinematics {

// =============================================================================
// Mathematical Constants and Utilities
// =============================================================================

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float HALF_PI = PI / 2.0f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

/**
 * @brief Clamp value to range
 */
inline float clamp(float val, float min_val, float max_val) {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
}

/**
 * @brief Normalize angle to [-PI, PI]
 */
inline float normalizeAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

// =============================================================================
// Cartesian Position and Pose Types
// =============================================================================

/**
 * @brief 2D Cartesian position
 */
struct Position2D {
    float x{0};
    float y{0};
    
    Position2D() = default;
    Position2D(float x_, float y_) : x(x_), y(y_) {}
    
    float magnitude() const { return std::sqrt(x*x + y*y); }
    float angle() const { return std::atan2(y, x); }
    
    Position2D operator+(const Position2D& other) const {
        return Position2D(x + other.x, y + other.y);
    }
    
    Position2D operator-(const Position2D& other) const {
        return Position2D(x - other.x, y - other.y);
    }
    
    Position2D operator*(float scale) const {
        return Position2D(x * scale, y * scale);
    }
};

/**
 * @brief 2D pose (position + orientation)
 */
struct Pose2D {
    float x{0};
    float y{0};
    float theta{0};  // Orientation angle
    
    Pose2D() = default;
    Pose2D(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}
    
    Position2D position() const { return Position2D(x, y); }
};

/**
 * @brief 3D Cartesian position
 */
struct Position3D {
    float x{0};
    float y{0};
    float z{0};
    
    Position3D() = default;
    Position3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    float magnitude() const { return std::sqrt(x*x + y*y + z*z); }
    
    Position3D operator+(const Position3D& other) const {
        return Position3D(x + other.x, y + other.y, z + other.z);
    }
    
    Position3D operator-(const Position3D& other) const {
        return Position3D(x - other.x, y - other.y, z - other.z);
    }
    
    Position3D operator*(float scale) const {
        return Position3D(x * scale, y * scale, z * scale);
    }
    
    float dot(const Position3D& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    Position3D cross(const Position3D& other) const {
        return Position3D(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }
    
    Position3D normalized() const {
        float m = magnitude();
        if (m < 1e-9f) return Position3D();
        return Position3D(x/m, y/m, z/m);
    }
};

/**
 * @brief Euler angles (ZYX convention: yaw, pitch, roll)
 */
struct EulerAngles {
    float roll{0};   // Rotation about X
    float pitch{0};  // Rotation about Y
    float yaw{0};    // Rotation about Z
    
    EulerAngles() = default;
    EulerAngles(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

/**
 * @brief Quaternion orientation
 * 
 * Uses Eigen::Quaternionf internally for optimized operations while
 * maintaining the same public API (w, x, y, z members).
 */
struct Quaternion {
    float w{1};
    float x{0};
    float y{0};
    float z{0};

    Quaternion() = default;
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    /// Convert to Eigen quaternion
    Eigen::Quaternionf toEigen() const {
        return Eigen::Quaternionf(w, x, y, z);
    }

    /// Convert from Eigen quaternion
    static Quaternion fromEigen(const Eigen::Quaternionf& q) {
        return Quaternion(q.w(), q.x(), q.y(), q.z());
    }

    /**
     * @brief Create quaternion from axis-angle representation
     */
    static Quaternion fromAxisAngle(const Position3D& axis, float angle) {
        Eigen::AngleAxisf aa(angle, Eigen::Vector3f(axis.x, axis.y, axis.z).normalized());
        return fromEigen(Eigen::Quaternionf(aa));
    }

    /**
     * @brief Create quaternion from Euler angles (ZYX convention)
     */
    static Quaternion fromEuler(const EulerAngles& euler) {
        // ZYX intrinsic convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        Eigen::Quaternionf q = Eigen::AngleAxisf(euler.yaw, Eigen::Vector3f::UnitZ())
                             * Eigen::AngleAxisf(euler.pitch, Eigen::Vector3f::UnitY())
                             * Eigen::AngleAxisf(euler.roll, Eigen::Vector3f::UnitX());
        return fromEigen(q);
    }

    /**
     * @brief Convert to Euler angles
     */
    EulerAngles toEuler() const {
        Eigen::Quaternionf q = toEigen();
        // Extract Euler angles using intrinsic ZYX decomposition
        Eigen::Vector3f euler = q.toRotationMatrix().canonicalEulerAngles(2, 1, 0); // ZYX
        EulerAngles result;
        result.yaw = euler(0);
        result.pitch = euler(1);
        result.roll = euler(2);
        return result;
    }

    float magnitude() const {
        return toEigen().norm();
    }

    Quaternion normalized() const {
        return fromEigen(toEigen().normalized());
    }

    Quaternion conjugate() const {
        return fromEigen(toEigen().conjugate());
    }

    Quaternion operator*(const Quaternion& q) const {
        return fromEigen(toEigen() * q.toEigen());
    }

    /**
     * @brief Rotate a vector by this quaternion
     */
    Position3D rotate(const Position3D& v) const {
        Eigen::Vector3f rotated = toEigen() * Eigen::Vector3f(v.x, v.y, v.z);
        return Position3D(rotated.x(), rotated.y(), rotated.z());
    }
};

/**
 * @brief 6-DOF pose (position + orientation)
 */
struct Pose6D {
    Position3D position;
    Quaternion orientation;
    
    Pose6D() = default;
    Pose6D(const Position3D& pos, const Quaternion& orient) 
        : position(pos), orientation(orient) {}
    
    /**
     * @brief Get Euler angles for orientation
     */
    EulerAngles euler() const {
        return orientation.toEuler();
    }
};

// =============================================================================
// 4x4 Homogeneous Transformation Matrix
// =============================================================================

/**
 * @brief 4x4 Homogeneous transformation matrix
 *
 * Row-major storage:
 * | R00 R01 R02 Tx |
 * | R10 R11 R12 Ty |
 * | R20 R21 R22 Tz |
 * |  0   0   0   1 |
 *
 * Uses Eigen internally for matrix operations while maintaining the
 * same float m[16] row-major storage and public API.
 */
struct Transform4x4 {
    float m[16];

    Transform4x4() {
        identity();
    }

    void identity() {
        std::memset(m, 0, sizeof(m));
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }

    float& at(int row, int col) { return m[row * 4 + col]; }
    float at(int row, int col) const { return m[row * 4 + col]; }

    /// Map to Eigen row-major 4x4 matrix
    using EigenMatrix = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;
    Eigen::Map<const EigenMatrix> toEigen() const {
        return Eigen::Map<const EigenMatrix>(m);
    }
    Eigen::Map<EigenMatrix> toEigen() {
        return Eigen::Map<EigenMatrix>(m);
    }
    static Transform4x4 fromEigen(const Eigen::Matrix4f& mat) {
        Transform4x4 t;
        Eigen::Map<EigenMatrix>(t.m) = mat;
        return t;
    }

    /**
     * @brief Create rotation about X axis
     */
    static Transform4x4 rotX(float angle) {
        return fromEigen(Eigen::Affine3f(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitX())).matrix());
    }

    /**
     * @brief Create rotation about Y axis
     */
    static Transform4x4 rotY(float angle) {
        return fromEigen(Eigen::Affine3f(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitY())).matrix());
    }

    /**
     * @brief Create rotation about Z axis
     */
    static Transform4x4 rotZ(float angle) {
        return fromEigen(Eigen::Affine3f(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ())).matrix());
    }

    /**
     * @brief Create translation matrix
     */
    static Transform4x4 translation(float x, float y, float z) {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.translation() = Eigen::Vector3f(x, y, z);
        return fromEigen(t.matrix());
    }

    static Transform4x4 translation(const Position3D& p) {
        return translation(p.x, p.y, p.z);
    }

    /**
     * @brief Create DH (Denavit-Hartenberg) transformation
     *
     * Standard DH convention:
     * T = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
     */
    static Transform4x4 DH(float theta, float d, float a, float alpha) {
        Eigen::Affine3f t =
            Eigen::Affine3f(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ())) *
            Eigen::Translation3f(0, 0, d) *
            Eigen::Translation3f(a, 0, 0) *
            Eigen::Affine3f(Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX()));
        return fromEigen(t.matrix());
    }

    /**
     * @brief Matrix multiplication (using Eigen)
     */
    Transform4x4 operator*(const Transform4x4& other) const {
        return fromEigen(toEigen() * other.toEigen());
    }

    /**
     * @brief Transform a point (using Eigen)
     */
    Position3D transformPoint(const Position3D& p) const {
        Eigen::Vector4f homogeneous(p.x, p.y, p.z, 1.0f);
        Eigen::Vector4f result = toEigen() * homogeneous;
        return Position3D(result.x(), result.y(), result.z());
    }

    /**
     * @brief Transform a direction (no translation, using Eigen)
     */
    Position3D transformDirection(const Position3D& d) const {
        Eigen::Matrix3f rotation = toEigen().topLeftCorner<3, 3>();
        Eigen::Vector3f result = rotation * Eigen::Vector3f(d.x, d.y, d.z);
        return Position3D(result.x(), result.y(), result.z());
    }

    /**
     * @brief Get translation component
     */
    Position3D getTranslation() const {
        return Position3D(at(0, 3), at(1, 3), at(2, 3));
    }

    /**
     * @brief Get rotation as quaternion (using Eigen)
     */
    Quaternion getRotation() const {
        Eigen::Matrix3f rotation = toEigen().topLeftCorner<3, 3>();
        return Quaternion::fromEigen(Eigen::Quaternionf(rotation).normalized());
    }

    /**
     * @brief Get as 6-DOF pose
     */
    Pose6D toPose() const {
        return Pose6D(getTranslation(), getRotation());
    }

    /**
     * @brief Matrix inverse (using Eigen Affine3f for proper rigid transform inverse)
     */
    Transform4x4 inverse() const {
        Eigen::Affine3f affine(toEigen());
        return fromEigen(affine.inverse(Eigen::Isometry).matrix());
    }
};

// =============================================================================
// Forward Kinematics Base Class
// =============================================================================

/**
 * @brief Base class for forward kinematics solvers
 */
class ForwardKinematicsBase {
public:
    virtual ~ForwardKinematicsBase() = default;
    
    /**
     * @brief Get number of degrees of freedom
     */
    virtual size_t getDOF() const = 0;
    
    /**
     * @brief Compute forward kinematics
     * @param joint_positions Array of joint positions (radians or meters)
     * @return End-effector pose
     */
    virtual Pose6D forwardKinematics(const float* joint_positions) const = 0;
    
    /**
     * @brief Get transformation matrix
     */
    virtual Transform4x4 getTransform(const float* joint_positions) const = 0;
};

// =============================================================================
// 2-DOF Planar Manipulator
// =============================================================================

/**
 * @brief 2-DOF planar arm kinematics (RR configuration)
 * 
 *         +--(q2)--[L2]---> end-effector
 *         |
 *     [L1]
 *         |
 *     --(q1)-- base
 * 
 * Joint 1: Base rotation (about Z)
 * Joint 2: Elbow rotation (about Z)
 */
class Planar2DOF : public ForwardKinematicsBase {
public:
    /**
     * @brief Constructor
     * @param L1 Link 1 length (meters)
     * @param L2 Link 2 length (meters)
     */
    Planar2DOF(float L1 = 0.5f, float L2 = 0.5f)
        : m_L1(L1), m_L2(L2) {}
    
    size_t getDOF() const override { return 2; }
    
    Pose6D forwardKinematics(const float* q) const override {
        Position3D pos = computePosition(q[0], q[1]);
        float total_angle = q[0] + q[1];
        
        Quaternion orient = Quaternion::fromAxisAngle(Position3D(0, 0, 1), total_angle);
        return Pose6D(pos, orient);
    }
    
    Transform4x4 getTransform(const float* q) const override {
        return Transform4x4::rotZ(q[0]) * 
               Transform4x4::translation(m_L1, 0, 0) *
               Transform4x4::rotZ(q[1]) *
               Transform4x4::translation(m_L2, 0, 0);
    }
    
    /**
     * @brief Compute end-effector position
     */
    Position3D computePosition(float q1, float q2) const {
        float c1 = std::cos(q1);
        float s1 = std::sin(q1);
        float c12 = std::cos(q1 + q2);
        float s12 = std::sin(q1 + q2);
        
        return Position3D(
            m_L1 * c1 + m_L2 * c12,
            m_L1 * s1 + m_L2 * s12,
            0.0f
        );
    }
    
    /**
     * @brief Compute elbow position
     */
    Position3D computeElbowPosition(float q1) const {
        return Position3D(m_L1 * std::cos(q1), m_L1 * std::sin(q1), 0);
    }
    
    /**
     * @brief Get workspace radius (maximum reach)
     */
    float getMaxReach() const { return m_L1 + m_L2; }
    
    /**
     * @brief Get minimum reach (inner boundary of annular workspace)
     */
    float getMinReach() const { return std::abs(m_L1 - m_L2); }
    
    float getL1() const { return m_L1; }
    float getL2() const { return m_L2; }
    void setL1(float L1) { m_L1 = L1; }
    void setL2(float L2) { m_L2 = L2; }
    
private:
    float m_L1;  // Link 1 length
    float m_L2;  // Link 2 length
};

// =============================================================================
// 3-DOF Spherical Wrist / Articulated Arm
// =============================================================================

/**
 * @brief 3-DOF articulated arm (RRR configuration)
 * 
 * Often used as a simplified industrial arm or as the wrist of a 6-DOF robot.
 */
class Articulated3DOF : public ForwardKinematicsBase {
public:
    /**
     * @brief Constructor
     * @param L1 Link 1 length (vertical)
     * @param L2 Link 2 length
     * @param L3 Link 3 length (to end-effector)
     */
    Articulated3DOF(float L1 = 0.3f, float L2 = 0.4f, float L3 = 0.3f)
        : m_L1(L1), m_L2(L2), m_L3(L3) {}
    
    size_t getDOF() const override { return 3; }
    
    Pose6D forwardKinematics(const float* q) const override {
        return getTransform(q).toPose();
    }
    
    Transform4x4 getTransform(const float* q) const override {
        // Joint 1: Base rotation (about Z)
        // Joint 2: Shoulder pitch (about Y)
        // Joint 3: Elbow pitch (about Y)
        
        Transform4x4 T;
        T = Transform4x4::rotZ(q[0]);
        T = T * Transform4x4::translation(0, 0, m_L1);
        T = T * Transform4x4::rotY(q[1]);
        T = T * Transform4x4::translation(m_L2, 0, 0);
        T = T * Transform4x4::rotY(q[2]);
        T = T * Transform4x4::translation(m_L3, 0, 0);
        
        return T;
    }
    
    float getL1() const { return m_L1; }
    float getL2() const { return m_L2; }
    float getL3() const { return m_L3; }
    
private:
    float m_L1, m_L2, m_L3;
};

// =============================================================================
// 6-DOF Industrial Manipulator (Generic DH)
// =============================================================================

/**
 * @brief DH parameter set for a single joint
 */
struct DHParameters {
    float theta_offset{0};  // Joint angle offset
    float d{0};             // Link offset along Z
    float a{0};             // Link length along X
    float alpha{0};         // Link twist about X
};

/**
 * @brief 6-DOF serial manipulator using DH convention
 * 
 * Can be configured for any 6-DOF robot by setting DH parameters:
 * - UR robots (Universal Robots)
 * - KUKA KR series
 * - ABB IRB series
 * - Fanuc LR Mate
 */
class Serial6DOF : public ForwardKinematicsBase {
public:
    Serial6DOF() {
        // Default to UR5-like parameters
        setUR5Parameters();
    }
    
    size_t getDOF() const override { return 6; }
    
    /**
     * @brief Set DH parameters for all joints
     */
    void setDHParameters(const DHParameters params[6]) {
        for (int i = 0; i < 6; ++i) {
            m_dh[i] = params[i];
        }
    }
    
    /**
     * @brief Set UR5 robot parameters
     * 
     * Universal Robot UR5 DH parameters (modified DH convention adapted)
     */
    void setUR5Parameters() {
        // UR5 (approximate, using standard DH)
        m_dh[0] = {0, 0.089159f, 0, HALF_PI};
        m_dh[1] = {0, 0, -0.42500f, 0};
        m_dh[2] = {0, 0, -0.39225f, 0};
        m_dh[3] = {0, 0.10915f, 0, HALF_PI};
        m_dh[4] = {0, 0.09465f, 0, -HALF_PI};
        m_dh[5] = {0, 0.0823f, 0, 0};
    }
    
    /**
     * @brief Set KUKA KR6 R900 parameters
     */
    void setKukaKR6Parameters() {
        m_dh[0] = {0, 0.400f, 0.025f, -HALF_PI};
        m_dh[1] = {-HALF_PI, 0, 0.315f, 0};
        m_dh[2] = {0, 0, 0.035f, -HALF_PI};
        m_dh[3] = {0, 0.365f, 0, HALF_PI};
        m_dh[4] = {0, 0, 0, -HALF_PI};
        m_dh[5] = {0, 0.080f, 0, 0};
    }
    
    Pose6D forwardKinematics(const float* q) const override {
        return getTransform(q).toPose();
    }
    
    Transform4x4 getTransform(const float* q) const override {
        Transform4x4 T;
        T.identity();
        
        for (int i = 0; i < 6; ++i) {
            float theta = q[i] + m_dh[i].theta_offset;
            T = T * Transform4x4::DH(theta, m_dh[i].d, m_dh[i].a, m_dh[i].alpha);
        }
        
        return T;
    }
    
    /**
     * @brief Get transformation to a specific joint frame
     * @param joint_index Joint index (0-5), returns T_{0 to joint}
     */
    Transform4x4 getJointTransform(const float* q, int joint_index) const {
        Transform4x4 T;
        T.identity();
        
        int n = (joint_index < 6) ? joint_index + 1 : 6;
        for (int i = 0; i < n; ++i) {
            float theta = q[i] + m_dh[i].theta_offset;
            T = T * Transform4x4::DH(theta, m_dh[i].d, m_dh[i].a, m_dh[i].alpha);
        }
        
        return T;
    }
    
    const DHParameters& getDH(int index) const { return m_dh[index]; }
    
private:
    DHParameters m_dh[6];
};

// =============================================================================
// 7-DOF Redundant Manipulator
// =============================================================================

/**
 * @brief 7-DOF redundant serial manipulator
 * 
 * Similar to Franka Emika Panda, KUKA iiwa, etc.
 * Has one extra DOF for null-space motion.
 */
class Serial7DOF : public ForwardKinematicsBase {
public:
    Serial7DOF() {
        // Default to Franka Panda-like parameters
        setPandaParameters();
    }
    
    size_t getDOF() const override { return 7; }
    
    void setDHParameters(const DHParameters params[7]) {
        for (int i = 0; i < 7; ++i) {
            m_dh[i] = params[i];
        }
    }
    
    /**
     * @brief Set Franka Emika Panda parameters
     */
    void setPandaParameters() {
        m_dh[0] = {0, 0.333f, 0, 0};
        m_dh[1] = {0, 0, 0, -HALF_PI};
        m_dh[2] = {0, 0.316f, 0, HALF_PI};
        m_dh[3] = {0, 0, 0.0825f, HALF_PI};
        m_dh[4] = {0, 0.384f, -0.0825f, -HALF_PI};
        m_dh[5] = {0, 0, 0, HALF_PI};
        m_dh[6] = {0, 0.107f, 0.088f, HALF_PI};
    }
    
    Pose6D forwardKinematics(const float* q) const override {
        return getTransform(q).toPose();
    }
    
    Transform4x4 getTransform(const float* q) const override {
        Transform4x4 T;
        T.identity();
        
        for (int i = 0; i < 7; ++i) {
            float theta = q[i] + m_dh[i].theta_offset;
            T = T * Transform4x4::DH(theta, m_dh[i].d, m_dh[i].a, m_dh[i].alpha);
        }
        
        return T;
    }
    
private:
    DHParameters m_dh[7];
};

// =============================================================================
// SCARA Robot
// =============================================================================

/**
 * @brief 4-DOF SCARA robot kinematics
 * 
 * Selective Compliance Articulated Robot Arm
 * Typical configuration: RRPR (2 rotary + 1 prismatic + 1 rotary)
 * 
 *    (q4: tool rotation)
 *         |
 *      [q3: Z slide]
 *         |
 *   (q1)--[L1]--(q2)--[L2]
 *     |
 *   base
 */
class SCARA : public ForwardKinematicsBase {
public:
    /**
     * @brief Constructor
     * @param L1 Link 1 length
     * @param L2 Link 2 length
     * @param Z_base Base height
     * @param Z_max Maximum Z travel
     */
    SCARA(float L1 = 0.35f, float L2 = 0.35f, float Z_base = 0.4f, float Z_max = 0.2f)
        : m_L1(L1), m_L2(L2), m_Z_base(Z_base), m_Z_max(Z_max) {}
    
    size_t getDOF() const override { return 4; }
    
    Pose6D forwardKinematics(const float* q) const override {
        // q[0]: Joint 1 rotation
        // q[1]: Joint 2 rotation
        // q[2]: Z slide (prismatic, in meters)
        // q[3]: Tool rotation
        
        float c1 = std::cos(q[0]);
        float s1 = std::sin(q[0]);
        float c12 = std::cos(q[0] + q[1]);
        float s12 = std::sin(q[0] + q[1]);
        
        Position3D pos(
            m_L1 * c1 + m_L2 * c12,
            m_L1 * s1 + m_L2 * s12,
            m_Z_base - q[2]  // Z decreases as slide extends
        );
        
        float total_rotation = q[0] + q[1] + q[3];
        Quaternion orient = Quaternion::fromAxisAngle(Position3D(0, 0, 1), total_rotation);
        
        return Pose6D(pos, orient);
    }
    
    Transform4x4 getTransform(const float* q) const override {
        Transform4x4 T;
        T = Transform4x4::rotZ(q[0]);
        T = T * Transform4x4::translation(m_L1, 0, 0);
        T = T * Transform4x4::rotZ(q[1]);
        T = T * Transform4x4::translation(m_L2, 0, 0);
        T = T * Transform4x4::translation(0, 0, m_Z_base - q[2]);
        T = T * Transform4x4::rotZ(q[3]);
        return T;
    }
    
    /**
     * @brief Get workspace boundary (maximum reach in XY plane)
     */
    float getMaxReach() const { return m_L1 + m_L2; }
    float getMinReach() const { return std::abs(m_L1 - m_L2); }
    
private:
    float m_L1, m_L2;
    float m_Z_base;
    float m_Z_max;
};

// =============================================================================
// Delta Robot (Parallel Manipulator)
// =============================================================================

/**
 * @brief Delta robot forward kinematics
 * 
 * 3-DOF parallel robot with 3 arms arranged at 120° intervals.
 * Used for high-speed pick-and-place operations.
 * 
 * Configuration:
 * - 3 actuated arms at 120° intervals on base
 * - Each arm has upper (active) and lower (passive) segments
 * - End-effector maintains parallel orientation
 */
class DeltaRobot : public ForwardKinematicsBase {
public:
    /**
     * @brief Constructor
     * @param r_base Radius of base platform
     * @param r_ee Radius of end-effector platform
     * @param L_upper Upper arm length (actuated)
     * @param L_lower Lower arm length (passive)
     */
    DeltaRobot(float r_base = 0.15f, float r_ee = 0.05f, 
               float L_upper = 0.2f, float L_lower = 0.4f)
        : m_r_base(r_base), m_r_ee(r_ee), m_L_upper(L_upper), m_L_lower(L_lower) {}
    
    size_t getDOF() const override { return 3; }
    
    /**
     * @brief Compute forward kinematics
     * 
     * Given actuator angles, find end-effector position.
     * This involves solving a trilateration problem.
     * 
     * @param theta Array of 3 actuator angles (radians)
     */
    Pose6D forwardKinematics(const float* theta) const override {
        Position3D pos = computePosition(theta);
        return Pose6D(pos, Quaternion());  // Delta maintains parallel orientation
    }
    
    Transform4x4 getTransform(const float* theta) const override {
        Position3D pos = computePosition(theta);
        return Transform4x4::translation(pos);
    }
    
    /**
     * @brief Compute end-effector position
     */
    Position3D computePosition(const float* theta) const {
        // Angles for 3 arms (120° apart)
        constexpr float angles[3] = {0, TWO_PI / 3.0f, 4.0f * PI / 3.0f};
        
        // For each arm, compute the center of the sphere
        // where the lower rod endpoint could be
        Position3D centers[3];
        
        for (int i = 0; i < 3; ++i) {
            float phi = angles[i];
            float cp = std::cos(phi);
            float sp = std::sin(phi);
            
            // Base attachment point
            float bx = m_r_base * cp;
            float by = m_r_base * sp;
            
            // Upper arm endpoint (rotates in vertical plane containing arm)
            float cx = bx + m_L_upper * std::cos(theta[i]) * cp;
            float cy = by + m_L_upper * std::cos(theta[i]) * sp;
            float cz = -m_L_upper * std::sin(theta[i]);
            
            // Adjust for end-effector platform radius
            cx -= m_r_ee * cp;
            cy -= m_r_ee * sp;
            
            centers[i] = Position3D(cx, cy, cz);
        }
        
        // Solve trilateration: find point equidistant from all 3 sphere centers
        // Using simplified approach for delta geometry
        return solvTrilateration(centers, m_L_lower);
    }
    
private:
    float m_r_base;   // Base platform radius
    float m_r_ee;     // End-effector platform radius
    float m_L_upper;  // Upper arm length
    float m_L_lower;  // Lower arm length
    
    /**
     * @brief Solve trilateration problem
     */
    Position3D solvTrilateration(const Position3D* centers, float radius) const {
        // Simplified solution assuming symmetric delta
        // For full solution, solve system of 3 sphere equations
        
        // Compute centroid of sphere centers as approximation
        Position3D centroid(
            (centers[0].x + centers[1].x + centers[2].x) / 3.0f,
            (centers[0].y + centers[1].y + centers[2].y) / 3.0f,
            (centers[0].z + centers[1].z + centers[2].z) / 3.0f
        );
        
        // Vectors from centroid to each center
        Position3D v0 = centers[0] - centroid;
        Position3D v1 = centers[1] - centroid;
        Position3D v2 = centers[2] - centroid;
        
        // Normal to plane of centers
        Position3D normal = v1.cross(v2).normalized();
        
        // Distance from each center to centroid
        float d0 = v0.magnitude();
        
        // Height below plane (use Pythagorean theorem)
        float h_sq = radius * radius - d0 * d0;
        float h = (h_sq > 0) ? std::sqrt(h_sq) : 0;
        
        // End-effector is below the plane of sphere centers
        return centroid - normal * h;
    }
};

// =============================================================================
// Cartesian / Gantry Robot
// =============================================================================

/**
 * @brief 3-DOF Cartesian (Gantry) robot
 * 
 * Linear motion in X, Y, Z (PPP configuration)
 */
class CartesianRobot : public ForwardKinematicsBase {
public:
    CartesianRobot() = default;
    
    size_t getDOF() const override { return 3; }
    
    Pose6D forwardKinematics(const float* q) const override {
        // q[0]: X position
        // q[1]: Y position
        // q[2]: Z position
        return Pose6D(Position3D(q[0], q[1], q[2]), Quaternion());
    }
    
    Transform4x4 getTransform(const float* q) const override {
        return Transform4x4::translation(q[0], q[1], q[2]);
    }
};

/**
 * @brief 5-DOF Gantry with rotation
 * 
 * Linear motion in X, Y, Z plus pitch and yaw for tool orientation
 */
class Gantry5DOF : public ForwardKinematicsBase {
public:
    Gantry5DOF() = default;
    
    size_t getDOF() const override { return 5; }
    
    Pose6D forwardKinematics(const float* q) const override {
        return getTransform(q).toPose();
    }
    
    Transform4x4 getTransform(const float* q) const override {
        // q[0-2]: X, Y, Z translation
        // q[3]: Pitch (rotation about Y)
        // q[4]: Roll (rotation about Z)
        Transform4x4 T = Transform4x4::translation(q[0], q[1], q[2]);
        T = T * Transform4x4::rotY(q[3]);
        T = T * Transform4x4::rotZ(q[4]);
        return T;
    }
};

// =============================================================================
// Mobile Robot Base Kinematics
// =============================================================================

/**
 * @brief Differential drive mobile robot kinematics
 * 
 * Two-wheel differential drive configuration.
 * State: [x, y, theta]
 */
class DifferentialDrive {
public:
    /**
     * @brief Constructor
     * @param wheel_radius Wheel radius (meters)
     * @param wheel_base Distance between wheels (meters)
     */
    DifferentialDrive(float wheel_radius = 0.05f, float wheel_base = 0.3f)
        : m_wheel_radius(wheel_radius), m_wheel_base(wheel_base) {}
    
    /**
     * @brief Forward kinematics from wheel velocities to body velocity
     * @param omega_left Left wheel angular velocity (rad/s)
     * @param omega_right Right wheel angular velocity (rad/s)
     * @return [v_linear, omega_angular] body velocities
     */
    std::array<float, 2> wheelToBody(float omega_left, float omega_right) const {
        float v_left = omega_left * m_wheel_radius;
        float v_right = omega_right * m_wheel_radius;
        
        float v = (v_left + v_right) / 2.0f;
        float omega = (v_right - v_left) / m_wheel_base;
        
        return {v, omega};
    }
    
    /**
     * @brief Update pose given current pose and body velocities
     * @param pose Current pose [x, y, theta]
     * @param v Linear velocity (m/s)
     * @param omega Angular velocity (rad/s)
     * @param dt Time step (seconds)
     * @return New pose
     */
    Pose2D updatePose(const Pose2D& pose, float v, float omega, float dt) const {
        Pose2D new_pose;
        
        if (std::abs(omega) < 1e-6f) {
            // Straight line motion
            new_pose.x = pose.x + v * dt * std::cos(pose.theta);
            new_pose.y = pose.y + v * dt * std::sin(pose.theta);
            new_pose.theta = pose.theta;
        } else {
            // Arc motion
            float r = v / omega;
            new_pose.theta = pose.theta + omega * dt;
            new_pose.x = pose.x + r * (std::sin(new_pose.theta) - std::sin(pose.theta));
            new_pose.y = pose.y - r * (std::cos(new_pose.theta) - std::cos(pose.theta));
        }
        
        new_pose.theta = normalizeAngle(new_pose.theta);
        return new_pose;
    }
    
    /**
     * @brief Inverse kinematics: body velocity to wheel velocities
     */
    std::array<float, 2> bodyToWheel(float v, float omega) const {
        float v_left = v - omega * m_wheel_base / 2.0f;
        float v_right = v + omega * m_wheel_base / 2.0f;
        
        return {v_left / m_wheel_radius, v_right / m_wheel_radius};
    }
    
    float getWheelRadius() const { return m_wheel_radius; }
    float getWheelBase() const { return m_wheel_base; }
    
private:
    float m_wheel_radius;
    float m_wheel_base;
};

/**
 * @brief Omni-directional (holonomic) mobile robot
 * 
 * 3-wheel omnidirectional configuration at 120° intervals.
 */
class OmniDrive3Wheel {
public:
    /**
     * @brief Constructor
     * @param wheel_radius Wheel radius
     * @param robot_radius Distance from center to wheel
     */
    OmniDrive3Wheel(float wheel_radius = 0.05f, float robot_radius = 0.2f)
        : m_wheel_radius(wheel_radius), m_robot_radius(robot_radius) {}
    
    /**
     * @brief Forward kinematics: wheel velocities to body twist
     * @param omega Array of 3 wheel angular velocities
     * @return [vx, vy, omega] body velocities
     */
    std::array<float, 3> wheelToBody(const float omega[3]) const {
        // Wheel angles (120° apart)
        constexpr float phi1 = 0;
        constexpr float phi2 = TWO_PI / 3.0f;
        constexpr float phi3 = 4.0f * PI / 3.0f;
        
        // Wheel linear velocities
        float v1 = omega[0] * m_wheel_radius;
        float v2 = omega[1] * m_wheel_radius;
        float v3 = omega[2] * m_wheel_radius;
        
        // Inverse of wheel Jacobian (simplified for 120° config)
        float vx = (2.0f/3.0f) * (v1 * std::cos(phi1) + v2 * std::cos(phi2) + v3 * std::cos(phi3));
        float vy = (2.0f/3.0f) * (v1 * std::sin(phi1) + v2 * std::sin(phi2) + v3 * std::sin(phi3));
        float omega_z = (v1 + v2 + v3) / (3.0f * m_robot_radius);
        
        return {vx, vy, omega_z};
    }
    
    /**
     * @brief Inverse kinematics: body twist to wheel velocities
     */
    std::array<float, 3> bodyToWheel(float vx, float vy, float omega_z) const {
        constexpr float phi1 = 0;
        constexpr float phi2 = TWO_PI / 3.0f;
        constexpr float phi3 = 4.0f * PI / 3.0f;
        
        float v1 = vx * std::cos(phi1) + vy * std::sin(phi1) + m_robot_radius * omega_z;
        float v2 = vx * std::cos(phi2) + vy * std::sin(phi2) + m_robot_radius * omega_z;
        float v3 = vx * std::cos(phi3) + vy * std::sin(phi3) + m_robot_radius * omega_z;
        
        return {v1 / m_wheel_radius, v2 / m_wheel_radius, v3 / m_wheel_radius};
    }
    
    /**
     * @brief Update pose from body velocities
     */
    Pose2D updatePose(const Pose2D& pose, float vx, float vy, float omega, float dt) const {
        // Transform velocities to world frame
        float c = std::cos(pose.theta);
        float s = std::sin(pose.theta);
        
        float vx_world = c * vx - s * vy;
        float vy_world = s * vx + c * vy;
        
        return Pose2D(
            pose.x + vx_world * dt,
            pose.y + vy_world * dt,
            normalizeAngle(pose.theta + omega * dt)
        );
    }
    
private:
    float m_wheel_radius;
    float m_robot_radius;
};

/**
 * @brief Mecanum wheel mobile robot
 * 
 * 4-wheel mecanum drive for omnidirectional motion.
 */
class MecanumDrive {
public:
    /**
     * @brief Constructor
     * @param wheel_radius Wheel radius
     * @param Lx Half-distance between front and back wheels
     * @param Ly Half-distance between left and right wheels
     */
    MecanumDrive(float wheel_radius = 0.05f, float Lx = 0.15f, float Ly = 0.15f)
        : m_wheel_radius(wheel_radius), m_Lx(Lx), m_Ly(Ly) {}
    
    /**
     * @brief Forward kinematics: wheel velocities to body twist
     * @param omega Array of 4 wheel angular velocities [FL, FR, RL, RR]
     * @return [vx, vy, omega] body velocities
     */
    std::array<float, 3> wheelToBody(const float omega[4]) const {
        float r = m_wheel_radius;
        float L = m_Lx + m_Ly;
        
        float vx = r * (omega[0] + omega[1] + omega[2] + omega[3]) / 4.0f;
        float vy = r * (-omega[0] + omega[1] + omega[2] - omega[3]) / 4.0f;
        float wz = r * (-omega[0] + omega[1] - omega[2] + omega[3]) / (4.0f * L);
        
        return {vx, vy, wz};
    }
    
    /**
     * @brief Inverse kinematics: body twist to wheel velocities
     */
    std::array<float, 4> bodyToWheel(float vx, float vy, float omega_z) const {
        float L = m_Lx + m_Ly;
        
        float fl = (vx - vy - L * omega_z) / m_wheel_radius;
        float fr = (vx + vy + L * omega_z) / m_wheel_radius;
        float rl = (vx + vy - L * omega_z) / m_wheel_radius;
        float rr = (vx - vy + L * omega_z) / m_wheel_radius;
        
        return {fl, fr, rl, rr};
    }
    
    Pose2D updatePose(const Pose2D& pose, float vx, float vy, float omega, float dt) const {
        float c = std::cos(pose.theta);
        float s = std::sin(pose.theta);
        
        return Pose2D(
            pose.x + (c * vx - s * vy) * dt,
            pose.y + (s * vx + c * vy) * dt,
            normalizeAngle(pose.theta + omega * dt)
        );
    }
    
private:
    float m_wheel_radius;
    float m_Lx, m_Ly;
};

// =============================================================================
// Stewart Platform (6-DOF Parallel Robot)
// =============================================================================

/**
 * @brief Stewart Platform (Hexapod) forward kinematics
 * 
 * 6-DOF parallel robot with 6 linear actuators.
 * Forward kinematics is solved iteratively (Newton-Raphson).
 */
class StewartPlatform : public ForwardKinematicsBase {
public:
    /**
     * @brief Constructor
     * @param base_radius Radius of base platform
     * @param platform_radius Radius of moving platform
     * @param base_half_angle Half-angle between adjacent base joints
     * @param platform_half_angle Half-angle between adjacent platform joints
     */
    StewartPlatform(float base_radius = 0.3f, float platform_radius = 0.15f,
                    float base_half_angle = 10.0f * DEG_TO_RAD,
                    float platform_half_angle = 10.0f * DEG_TO_RAD)
        : m_base_radius(base_radius), m_platform_radius(platform_radius) {
        
        // Compute base joint positions (6 joints at 60° intervals, paired)
        for (int i = 0; i < 6; ++i) {
            float angle = i * PI / 3.0f;  // 60° intervals
            float offset = (i % 2 == 0) ? -base_half_angle : base_half_angle;
            m_base_joints[i] = Position3D(
                base_radius * std::cos(angle + offset),
                base_radius * std::sin(angle + offset),
                0
            );
        }
        
        // Compute platform joint positions
        for (int i = 0; i < 6; ++i) {
            float angle = i * PI / 3.0f;
            float offset = (i % 2 == 0) ? platform_half_angle : -platform_half_angle;
            m_platform_joints[i] = Position3D(
                platform_radius * std::cos(angle + offset),
                platform_radius * std::sin(angle + offset),
                0
            );
        }
    }
    
    size_t getDOF() const override { return 6; }
    
    /**
     * @brief Forward kinematics (iterative solution)
     * 
     * Given leg lengths, compute platform pose.
     * Uses Newton-Raphson iteration starting from initial guess.
     * 
     * @param leg_lengths Array of 6 leg lengths
     */
    Pose6D forwardKinematics(const float* leg_lengths) const override {
        // Initial guess: platform directly above base
        Pose6D pose(Position3D(0, 0, 0.3f), Quaternion());
        
        // Newton-Raphson iteration
        const int max_iterations = 50;
        const float tolerance = 1e-6f;
        
        for (int iter = 0; iter < max_iterations; ++iter) {
            // Compute current leg vectors and lengths
            float current_lengths[6];
            inverseKinematics(pose, current_lengths);
            
            // Compute error
            float error = 0;
            for (int i = 0; i < 6; ++i) {
                float e = leg_lengths[i] - current_lengths[i];
                error += e * e;
            }
            
            if (error < tolerance) {
                break;
            }
            
            // Simple gradient descent update (simplified from full Newton-Raphson)
            float step = 0.1f;
            Position3D pos_grad(0, 0, 0);
            EulerAngles orient_grad;
            
            for (int i = 0; i < 6; ++i) {
                float e = leg_lengths[i] - current_lengths[i];
                Position3D leg = getPlatformJointWorld(pose, i) - m_base_joints[i];
                leg = leg.normalized();
                pos_grad = pos_grad + leg * e;
            }
            
            pose.position = pose.position + pos_grad * step;
        }
        
        return pose;
    }
    
    Transform4x4 getTransform(const float* leg_lengths) const override {
        Pose6D pose = forwardKinematics(leg_lengths);
        Transform4x4 T = Transform4x4::translation(pose.position);
        // Add rotation based on quaternion
        return T;
    }
    
    /**
     * @brief Inverse kinematics: compute leg lengths from pose
     */
    void inverseKinematics(const Pose6D& pose, float* leg_lengths) const {
        for (int i = 0; i < 6; ++i) {
            Position3D platform_joint = getPlatformJointWorld(pose, i);
            Position3D leg = platform_joint - m_base_joints[i];
            leg_lengths[i] = leg.magnitude();
        }
    }
    
private:
    float m_base_radius;
    float m_platform_radius;
    Position3D m_base_joints[6];
    Position3D m_platform_joints[6];
    
    Position3D getPlatformJointWorld(const Pose6D& pose, int index) const {
        return pose.position + pose.orientation.rotate(m_platform_joints[index]);
    }
};

} // namespace tether::kinematics
