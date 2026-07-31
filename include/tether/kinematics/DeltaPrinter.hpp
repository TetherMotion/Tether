#pragma once

/// @file DeltaPrinter.hpp
/// @brief Delta printer configuration and kinematics

#include <cmath>
#include <array>

namespace tether::kinematics {

/// @brief Delta printer geometry parameters (M665).
struct DeltaGeometry {
    double armLength = 250.0;       ///< Delta arm length (mm)
    double deltaRadius = 125.0;     ///< Delta radius (mm)
    double towerAngleA = 0.0;       ///< Tower A angle offset (degrees)
    double towerAngleB = 0.0;       ///< Tower B angle offset (degrees)
    double towerAngleC = 0.0;       ///< Tower C angle offset (degrees)
};

/// @brief Delta endstop adjustments (M666).
struct DeltaEndstopAdjust {
    double adjX = 0.0;  ///< X tower endstop adjustment (mm)
    double adjY = 0.0;  ///< Y tower endstop adjustment (mm)
    double adjZ = 0.0;  ///< Z tower endstop adjustment (mm)
};

/// @brief Delta printer configuration and kinematics.
class DeltaPrinter {
public:
    /// @brief Set delta geometry (M665).
    void setGeometry(const DeltaGeometry& geo) { geometry_ = geo; }

    /// @brief Set delta endstop adjustments (M666).
    void setEndstopAdjust(const DeltaEndstopAdjust& adj) { endstopAdjust_ = adj; }

    /// @brief Get current geometry.
    const DeltaGeometry& geometry() const { return geometry_; }

    /// @brief Get current endstop adjustments.
    const DeltaEndstopAdjust& endstopAdjust() const { return endstopAdjust_; }

    /// @brief Convert Cartesian (X, Y, Z) to tower angles (A, B, C).
    /// @return Tower positions in steps for each tower.
    std::array<double, 3> forwardActuatorKinematics(double x, double y, double z) const {
        // Tower positions at 120-degree intervals with angle offsets
        double angleA = (210.0 + geometry_.towerAngleA) * M_PI / 180.0;
        double angleB = (330.0 + geometry_.towerAngleB) * M_PI / 180.0;
        double angleC = (90.0  + geometry_.towerAngleC) * M_PI / 180.0;

        // Tower base positions
        double baseXA = geometry_.deltaRadius * std::cos(angleA);
        double baseYA = geometry_.deltaRadius * std::sin(angleA);
        double baseXB = geometry_.deltaRadius * std::cos(angleB);
        double baseYB = geometry_.deltaRadius * std::sin(angleB);
        double baseXC = geometry_.deltaRadius * std::cos(angleC);
        double baseYC = geometry_.deltaRadius * std::sin(angleC);

        // Distance from effector to each tower base
        double distA = std::sqrt((x - baseXA) * (x - baseXA) +
                                 (y - baseYA) * (y - baseYA));
        double distB = std::sqrt((x - baseXB) * (x - baseXB) +
                                 (y - baseYB) * (y - baseYB));
        double distC = std::sqrt((x - baseXC) * (x - baseXC) +
                                 (y - baseYC) * (y - baseYC));

        // Tower heights: z + sqrt(armLength^2 - dist^2) + endstopAdjust
        double towerA = z + std::sqrt(std::max(0.0,
            geometry_.armLength * geometry_.armLength - distA * distA)) + endstopAdjust_.adjX;
        double towerB = z + std::sqrt(std::max(0.0,
            geometry_.armLength * geometry_.armLength - distB * distB)) + endstopAdjust_.adjY;
        double towerC = z + std::sqrt(std::max(0.0,
            geometry_.armLength * geometry_.armLength - distC * distC)) + endstopAdjust_.adjZ;

        return {towerA, towerB, towerC};
    }

    /// @brief Convert tower positions back to Cartesian (inverse kinematics).
    /// Uses trilateration: subtract pairs of sphere equations to get two
    /// linear equations, express x and y in terms of z, then substitute
    /// into one sphere equation to get a quadratic in z.
    std::array<double, 3> inverseActuatorKinematics(
        double towerA, double towerB, double towerC) const {
        // Tower base positions
        double angleA = (210.0 + geometry_.towerAngleA) * M_PI / 180.0;
        double angleB = (330.0 + geometry_.towerAngleB) * M_PI / 180.0;
        double angleC = (90.0  + geometry_.towerAngleC) * M_PI / 180.0;

        double baseXA = geometry_.deltaRadius * std::cos(angleA);
        double baseYA = geometry_.deltaRadius * std::sin(angleA);
        double baseXB = geometry_.deltaRadius * std::cos(angleB);
        double baseYB = geometry_.deltaRadius * std::sin(angleB);
        double baseXC = geometry_.deltaRadius * std::cos(angleC);
        double baseYC = geometry_.deltaRadius * std::sin(angleC);

        // Adjust for endstop offsets
        double zA = towerA - endstopAdjust_.adjX;
        double zB = towerB - endstopAdjust_.adjY;
        double zC = towerC - endstopAdjust_.adjZ;

        double L = geometry_.armLength;
        double L2 = L * L;

        // Linear equations from subtracting sphere A from B, and A from C:
        // 2*(baseXA-baseXB)*x + 2*(baseYA-baseYB)*y + 2*(zA-zB)*z
        //   = baseXA^2+baseYA^2+zA^2 - baseXB^2-baseYB^2-zB^2
        double a1 = 2.0 * (baseXA - baseXB);
        double b1 = 2.0 * (baseYA - baseYB);
        double c1 = 2.0 * (zA - zB);
        double d1 = (baseXA*baseXA + baseYA*baseYA + zA*zA) -
                    (baseXB*baseXB + baseYB*baseYB + zB*zB);

        double a2 = 2.0 * (baseXA - baseXC);
        double b2 = 2.0 * (baseYA - baseYC);
        double c2 = 2.0 * (zA - zC);
        double d2 = (baseXA*baseXA + baseYA*baseYA + zA*zA) -
                    (baseXC*baseXC + baseYC*baseYC + zC*zC);

        // Express x and y in terms of z:
        // a1*x + b1*y = d1 - c1*z
        // a2*x + b2*y = d2 - c2*z
        double detXY = a1 * b2 - a2 * b1;
        if (std::abs(detXY) < 1e-12) {
            // Degenerate — towers are collinear
            return {0.0, 0.0, (zA + zB + zC) / 3.0};
        }

        // x = (p1 + q1*z) / detXY, y = (p2 + q2*z) / detXY
        double p1 = d1 * b2 - d2 * b1;
        double q1 = -c1 * b2 + c2 * b1;
        double p2 = a1 * d2 - a2 * d1;
        double q2 = -a1 * c2 + a2 * c1;

        // Substitute into sphere A: (x-baseXA)^2 + (y-baseYA)^2 + (z-zA)^2 = L^2
        // x = (p1 + q1*z) / detXY, y = (p2 + q2*z) / detXY
        // Let u = p1/detXY, v = q1/detXY, w = p2/detXY, s = q2/detXY
        // x = u + v*z, y = w + s*z
        double u = p1 / detXY;
        double v = q1 / detXY;
        double w = p2 / detXY;
        double s = q2 / detXY;

        // (u + v*z - baseXA)^2 + (w + s*z - baseYA)^2 + (z - zA)^2 = L^2
        // Let A = u - baseXA, B = w - baseYA
        // (A + v*z)^2 + (B + s*z)^2 + (z - zA)^2 = L^2
        // A^2 + 2*A*v*z + v^2*z^2 + B^2 + 2*B*s*z + s^2*z^2 + z^2 - 2*zA*z + zA^2 = L^2
        // (v^2 + s^2 + 1)*z^2 + 2*(A*v + B*s - zA)*z + (A^2 + B^2 + zA^2 - L^2) = 0
        double A = u - baseXA;
        double B = w - baseYA;
        double qa = v * v + s * s + 1.0;
        double qb = 2.0 * (A * v + B * s - zA);
        double qc = A * A + B * B + zA * zA - L2;

        double disc = qb * qb - 4.0 * qa * qc;
        if (disc < 0) {
            // No solution — return best estimate
            return {u, w, (zA + zB + zC) / 3.0};
        }

        double sqrtDisc = std::sqrt(disc);
        double z1 = (-qb + sqrtDisc) / (2.0 * qa);
        double z2 = (-qb - sqrtDisc) / (2.0 * qa);

        // Verify both solutions against all three sphere equations and
        // pick the one with the smallest total error.
        auto trySolution = [&](double z) -> double {
            double x = u + v * z;
            double y = w + s * z;
            double errA = (x - baseXA) * (x - baseXA) +
                          (y - baseYA) * (y - baseYA) +
                          (z - zA) * (z - zA) - L2;
            double errB = (x - baseXB) * (x - baseXB) +
                          (y - baseYB) * (y - baseYB) +
                          (z - zB) * (z - zB) - L2;
            double errC = (x - baseXC) * (x - baseXC) +
                          (y - baseYC) * (y - baseYC) +
                          (z - zC) * (z - zC) - L2;
            return std::abs(errA) + std::abs(errB) + std::abs(errC);
        };

        double z = (trySolution(z1) < trySolution(z2)) ? z1 : z2;

        double x = u + v * z;
        double y = w + s * z;

        return {x, y, z};
    }

private:
    DeltaGeometry geometry_;
    DeltaEndstopAdjust endstopAdjust_;
};

} // namespace tether::kinematics
