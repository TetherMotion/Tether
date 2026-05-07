#pragma once

#include <cstddef>

#include <tether/identification/Common.hpp>

namespace Identification {

class FrictionIdentifier {
public:
    struct FrictionParams {
        float coulomb{0};
        float stiction{0};
        float stribeck_velocity{0};
        float viscous{0};
        float fit_error{0};
    };

    FrictionIdentifier();

    void clear();
    void addMeasurement(float velocity, float friction);
    FrictionParams identify() const;

private:
    static constexpr size_t MAX_DATA = 256;
    float m_velocities[MAX_DATA]{};
    float m_frictions[MAX_DATA]{};
    size_t m_data_count{0};

    static float predictFriction(float velocity, const FrictionParams& p);
};

} // namespace Identification