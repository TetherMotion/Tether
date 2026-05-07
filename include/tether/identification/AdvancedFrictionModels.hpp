#pragma once

#include <vector>

#include <tether/identification/DenseLinearAlgebra.hpp>

namespace Identification {

struct DynamicFrictionSample {
    double displacement{0.0};
    double velocity{0.0};
    double force{0.0};
    double dt{0.0};
};

struct LuGreParameters {
    double coulomb{0.0};
    double static_friction{0.0};
    double stribeck_velocity{0.0};
    double sigma0{0.0};
    double sigma1{0.0};
    double sigma2{0.0};
    double fit{0.0};
};

struct DahlParameters {
    double stiffness{0.0};
    double shape{0.0};
    double viscous{0.0};
    double fit{0.0};
};

struct BoucWenParameters {
    double alpha{0.0};
    double k{0.0};
    double a{1.0};
    double beta{0.0};
    double gamma{0.0};
    double exponent{1.0};
    double fit{0.0};
};

struct PreisachModel {
    Vector alpha_thresholds;
    Vector beta_thresholds;
    Vector weights;
    double fit{0.0};

    double evaluate(double input) const;
};

class LuGreIdentifier {
public:
    static LuGreParameters identify(const std::vector<DynamicFrictionSample>& samples);
    static Vector simulate(const std::vector<DynamicFrictionSample>& samples,
                           const LuGreParameters& parameters);
};

class DahlIdentifier {
public:
    static DahlParameters identify(const std::vector<DynamicFrictionSample>& samples);
    static Vector simulate(const std::vector<DynamicFrictionSample>& samples,
                           const DahlParameters& parameters);
};

class BoucWenIdentifier {
public:
    static BoucWenParameters identify(const std::vector<DynamicFrictionSample>& samples);
    static Vector simulate(const std::vector<DynamicFrictionSample>& samples,
                           const BoucWenParameters& parameters);
};

class PreisachIdentifier {
public:
    static PreisachModel identify(const std::vector<DynamicFrictionSample>& samples,
                                  size_t relay_count = 8);
    static Vector simulate(const std::vector<DynamicFrictionSample>& samples,
                           const PreisachModel& model);
};

} // namespace Identification