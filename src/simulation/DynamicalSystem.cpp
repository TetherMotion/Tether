#include "tether/simulation/DynamicalSystem.hpp"

#include <iomanip>
#include <sstream>
#include <cmath>

namespace Simulation {

namespace {

std::string trim_copy(std::string text) {
    auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };

    auto begin = std::find_if_not(text.begin(), text.end(), is_space);
    auto end = std::find_if_not(text.rbegin(), text.rend(), is_space).base();
    if (begin >= end) {
        return {};
    }
    return std::string(begin, end);
}

std::string format_number(double value) {
    std::ostringstream stream;
    stream << std::setprecision(6) << std::defaultfloat << value;
    return stream.str();
}

bool ends_with_sentence_punctuation(const std::string& text) {
    if (text.empty()) {
        return false;
    }

    const char last = text.back();
    return last == '.' || last == '!' || last == '?';
}

std::string make_detailed_description(const DynamicalSystem& system, const ParamDescriptor& descriptor) {
    std::string base = trim_copy(descriptor.description);
    if (base.empty() || base == descriptor.name || base.size() < 4) {
        base = descriptor.name + " parameter";
    }
    if (!ends_with_sentence_punctuation(base)) {
        base.push_back('.');
    }

    std::ostringstream text;
    text << base << ' ';
    text << "Used by the " << system.name() << " model";
    if (!descriptor.unit.empty()) {
        text << " with units [" << descriptor.unit << ']';
    }
    text << ". Default " << format_number(descriptor.defaultValue);
    if (!descriptor.unit.empty()) {
        text << ' ' << descriptor.unit;
    }
    text << ". Range " << format_number(descriptor.minValue) << " to "
         << format_number(descriptor.maxValue);
    if (!descriptor.unit.empty()) {
        text << ' ' << descriptor.unit;
    }
    text << ". Recommended adjustment step " << format_number(descriptor.step);
    if (!descriptor.unit.empty()) {
        text << ' ' << descriptor.unit;
    }
    text << '.';
    if (descriptor.logarithmic) {
        text << " This parameter is best adjusted on a logarithmic scale.";
    }

    return text.str();
}

} // namespace

std::vector<ParamDescriptor> DynamicalSystem::parameterDescriptorsDetailed() const {
    auto descriptors = parameterDescriptors();
    for (auto& descriptor : descriptors) {
        descriptor.description = make_detailed_description(*this, descriptor);
    }
    return descriptors;
}

StateVector DynamicalSystem::defaultInitialStateForUi() const {
    auto initial_state = defaultInitialState();
    const bool has_nonzero_component = std::any_of(
        initial_state.begin(),
        initial_state.end(),
        [](double value) { return std::abs(value) > 1e-12; }
    );

    auto perturb_value = [](double value, double minimum_delta) {
        if (std::abs(value) <= 1e-12) {
            return minimum_delta;
        }

        const double direction = value >= 0.0 ? 1.0 : -1.0;
        const double delta = std::clamp(std::abs(value) * 0.05, minimum_delta, minimum_delta * 10.0);
        return value + (direction * delta);
    };

    if (initial_state.empty()) {
        return initial_state;
    }

    if (!has_nonzero_component) {
        initial_state.front() = 1e-3;
        for (size_t index = 1; index < initial_state.size(); ++index) {
            initial_state[index] = index == 1 ? 1e-3 : 1e-4;
        }
        return initial_state;
    }

    initial_state.front() = perturb_value(initial_state.front(), 1e-3);
    for (size_t index = 1; index < initial_state.size(); ++index) {
        const double minimum_delta = index == 1 ? 1e-3 : 1e-4;
        initial_state[index] = perturb_value(initial_state[index], minimum_delta);
    }

    return initial_state;
}

void DynamicalSystem::linearize(const StateVector& x0, const StateVector& u0,
                                  std::vector<double>& A, std::vector<double>& B,
                                  std::vector<double>& C, std::vector<double>& D) const {
    int n = stateDim();
    int m = inputDim();
    int p = outputDim();

    A.resize(n * n, 0.0);
    B.resize(n * m, 0.0);
    C.resize(p * n, 0.0);
    D.resize(p * m, 0.0);

    // Numerical Jacobian via central differences
    const double eps = 1e-6;
    StateVector f0 = dynamics(0.0, x0, u0);
    StateVector y0 = output(0.0, x0, u0);

    // dF/dx -> A matrix
    for (int j = 0; j < n; ++j) {
        StateVector xp = x0, xm = x0;
        xp[j] += eps;
        xm[j] -= eps;
        StateVector fp = dynamics(0.0, xp, u0);
        StateVector fm = dynamics(0.0, xm, u0);
        for (int i = 0; i < n; ++i) {
            A[i * n + j] = (fp[i] - fm[i]) / (2.0 * eps);
        }
        // dG/dx -> C matrix
        StateVector yp = output(0.0, xp, u0);
        StateVector ym = output(0.0, xm, u0);
        for (int i = 0; i < p; ++i) {
            C[i * n + j] = (yp[i] - ym[i]) / (2.0 * eps);
        }
    }

    // dF/du -> B matrix
    for (int j = 0; j < m; ++j) {
        StateVector up = u0, um = u0;
        up[j] += eps;
        um[j] -= eps;
        StateVector fp = dynamics(0.0, x0, up);
        StateVector fm = dynamics(0.0, x0, um);
        for (int i = 0; i < n; ++i) {
            B[i * m + j] = (fp[i] - fm[i]) / (2.0 * eps);
        }
        // dG/du -> D matrix
        StateVector yp = output(0.0, x0, up);
        StateVector ym = output(0.0, x0, um);
        for (int i = 0; i < p; ++i) {
            D[i * m + j] = (yp[i] - ym[i]) / (2.0 * eps);
        }
    }
}

} // namespace Simulation
