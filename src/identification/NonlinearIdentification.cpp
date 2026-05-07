#include <tether/identification/NonlinearIdentification.hpp>

#include <algorithm>
#include <cmath>

#include "IdentificationInternal.hpp"

namespace Identification {

namespace {

StaticNonlinearity makeStaticNonlinearity(const Vector& signal, StaticNonlinearityType type) {
    StaticNonlinearity non_linearity;
    non_linearity.type = type;
    non_linearity.gain = 1.0;
    non_linearity.offset = 0.0;
    const double limit = detail::percentileAbs(signal, 0.9);
    switch (type) {
        case StaticNonlinearityType::Saturation:
            non_linearity.lower_limit = -limit;
            non_linearity.upper_limit = limit;
            break;
        case StaticNonlinearityType::DeadZone:
            non_linearity.deadzone = detail::percentileAbs(signal, 0.15);
            break;
        case StaticNonlinearityType::Cubic:
            non_linearity.cubic = limit > detail::kEpsilon ? 0.1 / (limit * limit) : 0.0;
            break;
        case StaticNonlinearityType::Linear:
            break;
    }
    return non_linearity;
} // GCOVR_EXCL_LINE

} // namespace

double StaticNonlinearity::apply(double value) const {
    // Apply linear pre-transformation: y = m*x + b
    // This allows the non-linearity bounds to be scaled/shifted optimally.
    double transformed = gain * value + offset;
    switch (type) {
        case StaticNonlinearityType::Linear:
            return transformed;
        case StaticNonlinearityType::Saturation:
            // Clamp actuator inputs to their physical limitations to prevent windup 
            // and represent practical hardware ceilings (e.g., max voltage/PWM limit).
            return std::clamp(transformed, lower_limit, upper_limit);
        case StaticNonlinearityType::DeadZone:
            // Symmetrical deadzone logic: Discards values where |x| <= deadzone width.
            // Often used for modeling stick-slip friction start limits or sensor quantization zones where low-level noise shouldn't exert actuation.
            if (std::abs(transformed) <= deadzone) {
                return 0.0;
            }
            return transformed > 0.0 ? transformed - deadzone : transformed + deadzone;
        case StaticNonlinearityType::Cubic:
            // Represents progressive non-linear phenomena.
            // Function: y = x + c * x^3. It curves strongly for larger inputs, mimicking magnetic core saturation or stiffening mechanical springs.
            return transformed + cubic * transformed * transformed * transformed;
    }
    return transformed;
}

Vector HammersteinWienerModel::simulate(const Vector& input) const {
    // 1. Process data through the Hammerstein static memoryless non-linearity
    // For example: saturation applied by an amplifier on the actuator signal.
    Vector transformed_input(input.size(), 0.0);
    for (size_t i = 0; i < input.size(); ++i) {
        transformed_input[i] = input_non_linearity.apply(input[i]);
    }

    // 2. Drive the core Linear Time-Invariant (LTI) dynamics represented by the ARX/ARMAX difference equations.
    Vector linear_output = linear_block.simulate(transformed_input);

    // 3. Process the system state through the Wiener memoryless non-linearity
    // For instance: reading output using a sensor that itself has a non-linear characteristic reading the state.
    for (double& value : linear_output) {
        value = output_non_linearity.apply(value);
    }
    return linear_output;
}

HammersteinWienerModel HammersteinWienerIdentifier::identify(const Vector& input,
                                                             const Vector& output,
                                                             const PolynomialModelOrders& orders,
                                                             StaticNonlinearityType input_type,
                                                             StaticNonlinearityType output_type) {
    HammersteinWienerModel model;
    // Parameterize the fixed static boundaries using statistical limits from the raw dataset (min/max percentile approximations).
    model.input_non_linearity = makeStaticNonlinearity(input, input_type);
    model.output_non_linearity = makeStaticNonlinearity(output, output_type);

    // Apply the pre-figured input non-linearity to decouple it before identifying the dynamic blocks.
    Vector transformed_input(input.size(), 0.0);
    for (size_t i = 0; i < input.size(); ++i) {
        transformed_input[i] = model.input_non_linearity.apply(input[i]);
    }

    // Because the non-linearity is "stripped" from the input, a straightforward ARX model 
    // optimization can identify the internal linear relationship. Note: A fully correct Wiener system
    // identification requires more complex iterative Gauss-Newton solvers because the Wiener output nonlinearity 
    // cannot purely be inverted without noise propagation issues. Here, a simplified approach drives the ARX core.
    model.linear_block = ARXIdentifier::identify(transformed_input, output, orders);
    return model;
}

ETFEResult ETFEEstimator::estimate(const Vector& input,
                                   const Vector& output,
                                   double sample_time) {
    ETFEResult result;
    const size_t n = std::min(input.size(), output.size());
    // ETFE requires a minimum block size. A zero sample time is physically invalid.
    if (n < 8 || sample_time <= 0.0) {
        return result;
    }

    const size_t half = n / 2; // Only evaluate up to Nyquist frequency (fs/2) since signals are strictly real.
    result.frequencies.reserve(half - 1);
    result.magnitude.reserve(half - 1);
    result.phase.reserve(half - 1);
    result.coherence.reserve(half - 1);

    // Loop through discrete frequency bins k (1 to N/2 - 1). DC bin (k=0) is skipped.
    for (size_t k = 1; k < half; ++k) {
        std::complex<double> U(0.0, 0.0);
        std::complex<double> Y(0.0, 0.0);
        
        // Compute the Discrete Fourier Transform (DFT) specifically for frequency bin k.
        // Doing this manually rather than via FFT is computationally heavier O(N^2) but straightforward for arbitrary sizes.
        // Exponential basis: e^{-j * 2*pi*(k*t/N)}
        for (size_t t = 0; t < n; ++t) {
            const double angle = -2.0 * detail::kPi * static_cast<double>(k * t) / static_cast<double>(n);
            const std::complex<double> basis(std::cos(angle), std::sin(angle));
            U += input[t] * basis;
            Y += output[t] * basis;
        }

        // Power Spectral Density components (Auto-power and Cross-power)
        // A tiny scalar (1e-12) avoids numerical divide-by-zero blowups in unexcited frequency ranges.
        const double Suu = std::norm(U) + 1e-12; // Input spectral power: |U(f)|^2
        const double Syy = std::norm(Y) + 1e-12; // Output spectral power: |Y(f)|^2
        const std::complex<double> Syu = Y * std::conj(U); // Cross-power spectrum: Y(f) * U*(f) 
        
        // By spectral definition: Transfer Function G(f) = Syu(f) / Suu(f) = Y(f)/U(f) given deterministic bounds.
        const std::complex<double> G = Syu / Suu;

        // Map discrete index k to physical Hertz converting to physical Radians/sec
        // fs = 1/sample_time. freq in Hz = k * (fs / N).
        result.frequencies.push_back(static_cast<double>(k) / (static_cast<double>(n) * sample_time));
        result.magnitude.push_back(std::abs(G));
        result.phase.push_back(std::arg(G));
        
        // Coherence (gamma^2) measures linearity between input/output. 
        // 1.0 means perfectly linear/causal at that frequency. ~0 means noise/non-linearity dominates.
        result.coherence.push_back(std::norm(Syu) / (Suu * Syy));
    }

    return result; // GCOVR_EXCL_LINE
}

} // namespace Identification