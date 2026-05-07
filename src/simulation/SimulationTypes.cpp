#include "tether/simulation/SimulationTypes.hpp"
#include <cmath>

namespace Simulation {

double GeometryDesc::computeInertia() const {
    switch (shape) {
        case GeometryShape::PointMass:
            return 0.0;  // No rotational inertia
        case GeometryShape::Sphere:
            return 0.4 * mass * radius * radius;  // 2/5 * m * r^2
        case GeometryShape::SolidCylinder:
            return 0.5 * mass * radius * radius;  // 1/2 * m * r^2
        case GeometryShape::HollowCylinder:
            return 0.5 * mass * (radius * radius + innerRadius * innerRadius);
        case GeometryShape::SolidCuboid:
            return mass * (width * width + height * height) / 12.0;
        case GeometryShape::ThinRod:
            return mass * length * length / 12.0;
        case GeometryShape::ThinDisk:
            return 0.5 * mass * radius * radius;
        case GeometryShape::Cone:
            return 0.3 * mass * radius * radius;  // 3/10 * m * r^2
        case GeometryShape::Custom:
            return mass * radius * radius;  // Use radius as gyration radius
    }
    return 0.0;
}

double GeometryDesc::computeArea() const {
    switch (shape) {
        case GeometryShape::PointMass:
            return 0.0;
        case GeometryShape::Sphere:
            return M_PI * radius * radius;
        case GeometryShape::SolidCylinder:
        case GeometryShape::HollowCylinder:
            return 2.0 * radius * length;
        case GeometryShape::SolidCuboid:
            return width * height;
        case GeometryShape::ThinRod:
            return width * length;
        case GeometryShape::ThinDisk:
            return M_PI * radius * radius;
        case GeometryShape::Cone:
            return M_PI * radius * radius;
        case GeometryShape::Custom:
            return width * height;
    }
    return 0.0;
}

double FrictionParams::compute(double velocity, double normalForce, double dt) {
    if (model == FrictionModel::None) return 0.0;

    double absV = std::abs(velocity);
    double sign = (velocity > 0.0) ? 1.0 : ((velocity < 0.0) ? -1.0 : 0.0);

    switch (model) {
        case FrictionModel::Coulomb:
            return -sign * kineticFriction * normalForce;

        case FrictionModel::Viscous:
            return -viscousCoeff * velocity;

        case FrictionModel::CoulombViscous:
            return -sign * kineticFriction * normalForce - viscousCoeff * velocity;

        case FrictionModel::Stribeck: {
            // Stribeck effect: friction decreases from static to kinetic with velocity
            double stribeckFactor = std::exp(-std::pow(absV / stribeckVelocity, stribeckExponent));
            double frictionCoeff = kineticFriction + (staticFriction - kineticFriction) * stribeckFactor;
            return -sign * frictionCoeff * normalForce - viscousCoeff * velocity;
        }

        case FrictionModel::LuGre: {
            // LuGre dynamic friction model
            double g_v = kineticFriction + (staticFriction - kineticFriction) *
                         std::exp(-std::pow(absV / stribeckVelocity, stribeckExponent));
            g_v *= normalForce;
            double dz = velocity - sigma0 * absV / (g_v > 0 ? g_v : 1e-10) * lugreState;
            lugreState += dz * dt;
            return -(sigma0 * lugreState + sigma1 * dz + sigma2 * velocity);
        }

        case FrictionModel::Dahl: {
            // Dahl friction model (simplified LuGre without Stribeck)
            double fc = kineticFriction * normalForce;
            double dz = velocity * (1.0 - sigma0 * lugreState / (fc > 0 ? fc : 1e-10) * sign);
            lugreState += dz * dt;
            return -sigma0 * lugreState;
        }

        default:
            return 0.0;
    }
}

} // namespace Simulation
