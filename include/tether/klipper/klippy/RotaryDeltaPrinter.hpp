#pragma once

/// @file RotaryDeltaPrinter.hpp
/// @brief Rotary delta printer configuration and kinematics.
///
/// @details
/// A rotary delta printer has three upper arms pivoting around fixed
/// shoulder joints at 120° spacing around the base. Each upper arm has
/// length **L1** (upper arm) and connects via a forearm of length **L2**
/// to the effector. The actuated quantity is the **shoulder angle**
/// (the angle of the upper arm relative to the horizontal plane).
///
/// Geometry (top view, looking down the Z axis):
///
/// @code
///        Tower C (90°)
///           *
///          / \
///         /   \
///        /     \
///       *-------*
///  Tower A      Tower B
///  (210°)       (330°)
/// @endcode
///
/// Forward kinematics (cartesian -> joint angles):
///   For each tower i at base position (bx_i, by_i):
///     1. Compute horizontal distance from effector to tower base:
///        d_i = sqrt((x - bx_i)^2 + (y - by_i)^2)
///     2. The effector joint is at height z. The upper-arm pivot is at
///        height baseHeight. The vertical offset is:
///        dz = z - baseHeight
///     3. The upper arm end is at (d_i - effectorRadius, dz) in the
///        (radial, vertical) plane relative to the pivot.
///     4. The shoulder angle theta_i satisfies:
///          L1 * cos(theta_i) = d_i - effectorRadius
///          L1 * sin(theta_i) = dz
///        => theta_i = atan2(dz, d_i - effectorRadius)
///     (When the arm is horizontal, theta = 0; when pointing up, theta = 90°.)
///
/// Inverse kinematics (joint angles -> cartesian):
///   For each tower i, the upper-arm end is at:
///     (bx_i + (effectorRadius + L1*cos(theta_i)) * cos(angle_i),
///      by_i + (effectorRadius + L1*cos(theta_i)) * sin(angle_i),
///      baseHeight + L1*sin(theta_i))
///   The effector position is at the intersection of three spheres of
///   radius L2 centered at each upper-arm end. We solve by trilateration
///   (same approach as the linear DeltaPrinter, adapted for the
///   upper-arm-end centers instead of fixed tower tops).

#include <cmath>
#include <array>

namespace tether::klipper::klippy {

/// @brief Rotary delta geometry parameters.
struct RotaryDeltaGeometry {
    double upperArmLength = 170.0;    ///< Upper arm length L1 (mm)
    double forearmLength = 320.0;     ///< Forearm length L2 (mm)
    double baseRadius = 90.0;         ///< Distance from center to shoulder pivot (mm)
    double effectorRadius = 24.0;     ///< Effector joint radius (mm)
    double baseHeight = 0.0;          ///< Shoulder pivot height above bed (mm)
    double towerAngleA = 210.0;       ///< Tower A angle (degrees)
    double towerAngleB = 330.0;       ///< Tower B angle (degrees)
    double towerAngleC = 90.0;        ///< Tower C angle (degrees)
};

/// @brief Rotary delta endstop angle adjustments (radians).
struct RotaryDeltaEndstopAdjust {
    double adjA = 0.0;  ///< Tower A endstop angle adjustment (rad)
    double adjB = 0.0;  ///< Tower B endstop angle adjustment (rad)
    double adjC = 0.0;  ///< Tower C endstop angle adjustment (rad)
};

/// @brief Rotary delta printer configuration and kinematics.
class RotaryDeltaPrinter {
public:
    /// @brief Set rotary delta geometry.
    void setGeometry(const RotaryDeltaGeometry& geo) { geometry_ = geo; }

    /// @brief Set endstop angle adjustments.
    void setEndstopAdjust(const RotaryDeltaEndstopAdjust& adj) { endstopAdjust_ = adj; }

    /// @return Current geometry.
    const RotaryDeltaGeometry& geometry() const { return geometry_; }

    /// @return Current endstop adjustments.
    const RotaryDeltaEndstopAdjust& endstopAdjust() const { return endstopAdjust_; }

    /// @brief Convert Cartesian (X, Y, Z) to shoulder angles (A, B, C) in radians.
    ///
    /// The angle is measured from the horizontal plane: 0 = arm horizontal,
    /// positive = arm pointing upward.
    std::array<double, 3> cartesianToTower(double x, double y, double z) const {
        const double angles[3] = {
            geometry_.towerAngleA * M_PI / 180.0,
            geometry_.towerAngleB * M_PI / 180.0,
            geometry_.towerAngleC * M_PI / 180.0,
        };
        const double adj[3] = {
            endstopAdjust_.adjA, endstopAdjust_.adjB, endstopAdjust_.adjC,
        };

        std::array<double, 3> result;
        double eff = geometry_.effectorRadius;
        double L1 = geometry_.upperArmLength;
        double L2 = geometry_.forearmLength;
        for (int i = 0; i < 3; ++i) {
            double ca = std::cos(angles[i]);
            double sa = std::sin(angles[i]);

            // The upper arm swings in a fixed vertical plane through the
            // shoulder and the printer center. In (radial, tangential,
            // vertical) coordinates relative to tower i:
            //   arm end radial  = baseRadius - L1*cos(theta)  (inward)
            //   arm end tangent = 0  (in the plane)
            //   arm end vert    = baseHeight + L1*sin(theta)
            //
            // Effector joint (radial, tangent, vert) relative to tower:
            //   joint radial  = x*ca + y*sa + eff  (projection + offset)
            //   joint tangent = -x*sa + y*ca
            //   joint vert    = z
            //
            // Forearm constraint:
            //   (armRadial - jointRadial)^2 + (0 - jointTangent)^2 +
            //   (armVert - z)^2 = L2^2

            double jointRadial = x * ca + y * sa + eff;
            double jointTangent = -x * sa + y * ca;
            double jointVert = z;

            // Let c = cos(theta), s = sin(theta):
            //   (baseRadius - L1*c - jointRadial)^2 + jointTangent^2 +
            //   (baseHeight + L1*s - jointVert)^2 = L2^2
            // Let P = baseRadius - jointRadial, Q = jointVert - baseHeight:
            //   (P - L1*c)^2 + jointTangent^2 + (L1*s - Q)^2 = L2^2
            //   P^2 - 2*P*L1*c + L1^2*c^2 + jt^2 + L1^2*s^2 - 2*Q*L1*s + Q^2 = L2^2
            //   P^2 + jt^2 + Q^2 + L1^2 - 2*L1*(P*c + Q*s) = L2^2
            //   P*c + Q*s = (P^2 + jt^2 + Q^2 + L1^2 - L2^2) / (2*L1) = K
            double P = geometry_.baseRadius - jointRadial;
            double Q = jointVert - geometry_.baseHeight;
            double jt2 = jointTangent * jointTangent;

            double K = (P * P + jt2 + Q * Q + L1 * L1 - L2 * L2) / (2.0 * L1);
            double denom2 = P * P + Q * Q - K * K;
            if (denom2 < 0.0) denom2 = 0.0;
            double denom = std::sqrt(denom2);

            // Solve P*cos(theta) + Q*sin(theta) = K:
            //   R*cos(theta - phi) = K  where R = sqrt(P²+Q²), phi = atan2(Q, P)
            //   theta = phi ± acos(K/R) = phi ± atan2(denom, K)
            double phi = std::atan2(Q, P);
            double alpha = std::atan2(denom, K); // = acos(K/R)
            double theta1 = phi + alpha;
            double theta2 = phi - alpha;

            // Pick the solution with the upper arm more horizontal
            // (larger cos(theta)), the typical working range.
            double theta = (std::cos(theta1) > std::cos(theta2)) ? theta1 : theta2;

            result[i] = theta + adj[i];
        }
        return result;
    }

    /// @brief Convert shoulder angles (A, B, C in radians) back to Cartesian.
    ///
    /// Computes the upper-arm-end positions, then trilaterates with forearm
    /// length L2 to find the effector center.
    std::array<double, 3> towerToCartesian(
        double thetaA, double thetaB, double thetaC) const {
        const double angles[3] = {
            geometry_.towerAngleA * M_PI / 180.0,
            geometry_.towerAngleB * M_PI / 180.0,
            geometry_.towerAngleC * M_PI / 180.0,
        };
        const double thetas[3] = {thetaA, thetaB, thetaC};
        const double adj[3] = {
            endstopAdjust_.adjA, endstopAdjust_.adjB, endstopAdjust_.adjC,
        };

        double L1 = geometry_.upperArmLength;
        double L2 = geometry_.forearmLength;

        // Upper-arm-end positions, then adjust for effector joint offset.
        // The forearm connects the upper-arm end to the effector joint,
        // which is at distance effectorRadius from the effector center
        // toward the tower. So the effective sphere center (for
        // trilateration in terms of the effector center) is:
        //   (cx_i - eff*cos(a_i), cy_i - eff*sin(a_i), cz_i)
        // because (x + eff*cos(a_i) - cx_i)^2 = (x - (cx_i - eff*cos(a_i)))^2
        double cx[3], cy[3], cz[3];
        double eff = geometry_.effectorRadius;
        for (int i = 0; i < 3; ++i) {
            double theta = thetas[i] - adj[i];
            double bx = geometry_.baseRadius * std::cos(angles[i]);
            double by = geometry_.baseRadius * std::sin(angles[i]);
            // Upper-arm end: arm extends inward from the shoulder.
            double reach = L1 * std::cos(theta);
            double armEndX = bx - reach * std::cos(angles[i]);
            double armEndY = by - reach * std::sin(angles[i]);
            double armEndZ = geometry_.baseHeight + L1 * std::sin(theta);
            // Effective sphere center = armEnd - effectorJointOffset
            // (so that the sphere equation is in terms of effector center)
            cx[i] = armEndX - eff * std::cos(angles[i]);
            cy[i] = armEndY - eff * std::sin(angles[i]);
            cz[i] = armEndZ;
        }

        // Trilateration: find (x, y, z) such that
        //   (x-cx_i)^2 + (y-cy_i)^2 + (z-cz_i)^2 = L2^2  for i=0,1,2
        // Subtract sphere 0 from spheres 1 and 2 to get two linear equations:
        //   2*(cx0-cx1)*x + 2*(cy0-cy1)*y + 2*(cz0-cz1)*z
        //     = cx0^2+cy0^2+cz0^2 - cx1^2-cy1^2-cz1^2  (+ L2^2 cancels)
        double a1 = 2.0 * (cx[0] - cx[1]);
        double b1 = 2.0 * (cy[0] - cy[1]);
        double c1 = 2.0 * (cz[0] - cz[1]);
        double d1 = (cx[0]*cx[0] + cy[0]*cy[0] + cz[0]*cz[0]) -
                    (cx[1]*cx[1] + cy[1]*cy[1] + cz[1]*cz[1]);

        double a2 = 2.0 * (cx[0] - cx[2]);
        double b2 = 2.0 * (cy[0] - cy[2]);
        double c2 = 2.0 * (cz[0] - cz[2]);
        double d2 = (cx[0]*cx[0] + cy[0]*cy[0] + cz[0]*cz[0]) -
                    (cx[2]*cx[2] + cy[2]*cy[2] + cz[2]*cz[2]);

        // Express x, y in terms of z:
        double detXY = a1 * b2 - a2 * b1;
        if (std::abs(detXY) < 1e-12) {
            return {0.0, 0.0, (cz[0] + cz[1] + cz[2]) / 3.0};
        }

        double p1 = d1 * b2 - d2 * b1;
        double q1 = -c1 * b2 + c2 * b1;
        double p2 = a1 * d2 - a2 * d1;
        double q2 = -a1 * c2 + a2 * c1;

        double u = p1 / detXY;
        double v = q1 / detXY;
        double w = p2 / detXY;
        double s = q2 / detXY;

        // Substitute into sphere 0: (x-cx0)^2 + (y-cy0)^2 + (z-cz0)^2 = L2^2
        double A = u - cx[0];
        double B = w - cy[0];
        double qa = v * v + s * s + 1.0;
        double qb = 2.0 * (A * v + B * s - cz[0]);
        double qc = A * A + B * B + cz[0] * cz[0] - L2 * L2;

        double disc = qb * qb - 4.0 * qa * qc;
        if (disc < 0.0) {
            return {u, w, (cz[0] + cz[1] + cz[2]) / 3.0};
        }

        double sqrtDisc = std::sqrt(disc);
        double z1 = (-qb + sqrtDisc) / (2.0 * qa);
        double z2 = (-qb - sqrtDisc) / (2.0 * qa);

        // For a rotary delta, the effector is below the arm ends.
        // Pick the lower (more negative) Z solution. Both mathematically
        // satisfy the sphere equations; the lower one is the physical
        // configuration where the effector hangs below the arms.
        double z = std::min(z1, z2);
        double x = u + v * z;
        double y = w + s * z;

        return {x, y, z};
    }

private:
    RotaryDeltaGeometry geometry_;
    RotaryDeltaEndstopAdjust endstopAdjust_;
};

} // namespace tether::klipper::klippy
