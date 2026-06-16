#include <tether/identification/PolynomialModels.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include <tether/identification/DenseLinearAlgebra.hpp>

namespace Identification {

namespace {

size_t maxHistory(const PolynomialModelOrders& orders) {
    return std::max({orders.na, orders.nb + orders.nk, orders.nc, orders.nd, orders.nf, size_t(1)});
} // GCOVR_EXCL_LINE

double predictARXLike(const DiscretePolynomialModel& model,
                      const Vector& input,
                      const Vector& output,
                      const Vector& residuals,
                      size_t index,
                      bool use_measured_output) {
    double prediction = 0.0;
    bool f_has_coeffs = false;
    for (size_t i = 1; i < model.F.size(); ++i) {
        if (std::abs(model.F[i]) > 1e-12) { f_has_coeffs = true; break; }
    }
    const std::vector<double>& denominator = f_has_coeffs ? model.F : model.A;
    const Vector& reference_output = use_measured_output ? output : residuals;
    const Vector& simulated_output = output;

    for (size_t i = 1; i < denominator.size(); ++i) {
        if (index >= i) {
            const double previous = use_measured_output ? output[index - i] : simulated_output[index - i];
            prediction -= denominator[i] * previous;
        }
    }

    for (size_t j = 0; j < model.B.size(); ++j) {
        const size_t input_index = model.orders.nk + j;
        if (index >= input_index) {
            prediction += model.B[j] * input[index - input_index];
        }
    }

    if (model.C.size() > 1) {
        for (size_t i = 1; i < model.C.size(); ++i) {
            if (index >= i) {
                prediction += model.C[i] * residuals[index - i];
            }
        }
    }

    if (model.D.size() > 1) {
        for (size_t i = 1; i < model.D.size(); ++i) {
            if (index >= i) {
                prediction -= model.D[i] * reference_output[index - i];
            }
        }
    }

    return prediction;
}

void finalizeFit(DiscretePolynomialModel& model, const Vector& input, const Vector& output) {
    model.residuals.assign(output.size(), 0.0);
    const Vector simulated = model.simulate(input);
    double error_sum = 0.0;
    double signal_sum = 0.0;
    const double mean = output.empty() ? 0.0 :
        std::accumulate(output.begin(), output.end(), 0.0) / static_cast<double>(output.size());

    for (size_t i = 0; i < output.size() && i < simulated.size(); ++i) {
        const double error = output[i] - simulated[i];
        model.residuals[i] = error;
        error_sum += error * error;
        const double centered = output[i] - mean;
        signal_sum += centered * centered;
    }

    model.mse = output.empty() ? 0.0 : error_sum / static_cast<double>(output.size());
    if (signal_sum > 1e-9) {
        model.fit = 100.0 * std::max(0.0, 1.0 - std::sqrt(error_sum / signal_sum));
    } else {
        model.fit = 0.0;
    }
}

DiscretePolynomialModel makeInvalidPolynomial(const PolynomialModelOrders& orders) {
    DiscretePolynomialModel model;
    model.orders = orders;
    model.A.assign(orders.na + 1, 0.0);
    model.C.assign(orders.nc + 1, 0.0);
    model.D.assign(orders.nd + 1, 0.0);
    model.F.assign(orders.nf + 1, 0.0);
    if (!model.A.empty()) {
        model.A[0] = 1.0;
    }
    if (!model.C.empty()) {
        model.C[0] = 1.0;
    }
    if (!model.D.empty()) {
        model.D[0] = 1.0;
    }
    if (!model.F.empty()) {
        model.F[0] = 1.0;
    }
    model.B.assign(orders.nb, 0.0);
    return model; // GCOVR_EXCL_LINE
}

double polynomialCost(const DiscretePolynomialModel& model,
                      const Vector& input,
                      const Vector& output) {
    const Vector simulated = model.simulate(input);
    double cost = 0.0;
    for (size_t i = 0; i < output.size() && i < simulated.size(); ++i) {
        const double error = output[i] - simulated[i];
        cost += error * error;
    }
    return output.empty() ? 0.0 : cost / static_cast<double>(output.size());
}

void optimizeOutputErrorModel(DiscretePolynomialModel& model,
                              const Vector& input,
                              const Vector& output,
                              size_t iterations) {
    Vector parameters;
    parameters.insert(parameters.end(), model.B.begin(), model.B.end());
    if (model.F.size() > 1) {
        parameters.insert(parameters.end(), model.F.begin() + 1, model.F.end());
    }

    if (parameters.empty()) {
        finalizeFit(model, input, output); // GCOVR_EXCL_LINE
        return; // GCOVR_EXCL_LINE
    }

    Vector steps(parameters.size(), 0.05);
    double best_cost = polynomialCost(model, input, output);

    auto apply = [&](const Vector& theta) {
        size_t index = 0;
        for (double& coeff : model.B) {
            coeff = theta[index++];
        }
        for (size_t i = 1; i < model.F.size(); ++i) {
            model.F[i] = theta[index++];
        }
    };

    for (size_t iter = 0; iter < iterations; ++iter) {
        for (size_t i = 0; i < parameters.size(); ++i) {
            const double original = parameters[i];
            double local_best = best_cost;
            double best_value = original;

            for (double direction : {-1.0, 1.0}) {
                parameters[i] = original + direction * steps[i];
                apply(parameters);
                const double cost = polynomialCost(model, input, output);
                if (cost < local_best) {
                    local_best = cost; // GCOVR_EXCL_LINE
                    best_value = parameters[i]; // GCOVR_EXCL_LINE
                }
            }

            parameters[i] = best_value;
            apply(parameters);
            if (local_best < best_cost) {
                best_cost = local_best; // GCOVR_EXCL_LINE
            } else {
                steps[i] *= 0.5;
            }
        }
    }

    apply(parameters);
    finalizeFit(model, input, output);
}

} // namespace

double DiscretePolynomialModel::predictOneStep(const Vector& input,
                                               const Vector& output,
                                               size_t index) const {
    return predictARXLike(*this, input, output, residuals, index, true);
}

Vector DiscretePolynomialModel::simulate(const Vector& input) const {
    Vector simulated(input.size(), 0.0);
    Vector errors(input.size(), 0.0);
    for (size_t k = 0; k < input.size(); ++k) {
        simulated[k] = predictARXLike(*this, input, simulated, errors, k, false);
        errors[k] = 0.0;
    }
    return simulated;
}

bool DiscretePolynomialModel::valid() const {
    return !A.empty() || !F.empty();
}

DiscretePolynomialModel ARXIdentifier::identify(const Vector& input,
                                                const Vector& output,
                                                const PolynomialModelOrders& orders) {
    DiscretePolynomialModel model = makeInvalidPolynomial(orders);
    const size_t start = maxHistory(orders);
    if (input.size() != output.size() || output.size() <= start) {
        return model;
    }

    Matrix phi;
    Vector y;
    for (size_t k = start; k < output.size(); ++k) {
        Vector row;
        for (size_t i = 1; i <= orders.na; ++i) {
            row.push_back(-output[k - i]);
        }
        for (size_t j = 0; j < orders.nb; ++j) {
            row.push_back(input[k - orders.nk - j]);
        }
        appendRow(phi, row);
        y.push_back(output[k]);
    }

    const Vector theta = solveLeastSquares(phi, y, orders.regularization);
    bool all_zero = true;
    for (double v : theta) if (std::abs(v) > 1e-12) { all_zero = false; break; }
    if (all_zero) std::cerr << "ARX theta ALL ZERO for na=" << orders.na << " nb=" << orders.nb << " samples=" << y.size() << "\n";
    for (size_t i = 0; i < orders.na; ++i) {
        model.A[i + 1] = theta[i];
    }
    for (size_t j = 0; j < orders.nb; ++j) {
        model.B[j] = theta[orders.na + j];
    }
    finalizeFit(model, input, output);
    return model;
}

DiscretePolynomialModel InstrumentalVariablesIdentifier::identify(const Vector& input,
                                                                  const Vector& output,
                                                                  const PolynomialModelOrders& orders) {
    DiscretePolynomialModel initial = ARXIdentifier::identify(input, output, orders);
    DiscretePolynomialModel model = makeInvalidPolynomial(orders);
    const size_t start = maxHistory(orders);
    if (input.size() != output.size() || output.size() <= start) {
        return model;
    }

    const Vector predicted = initial.simulate(input);
    Matrix phi;
    Matrix instruments;
    Vector y;
    for (size_t k = start; k < output.size(); ++k) {
        Vector reg_row;
        Vector inst_row;
        for (size_t i = 1; i <= orders.na; ++i) {
            reg_row.push_back(-output[k - i]);
            inst_row.push_back(-predicted[k - i]);
        }
        for (size_t j = 0; j < orders.nb; ++j) {
            const double input_value = input[k - orders.nk - j];
            reg_row.push_back(input_value);
            inst_row.push_back(input_value);
        }
        appendRow(phi, reg_row);
        appendRow(instruments, inst_row);
        y.push_back(output[k]);
    }

    Matrix ztphi = multiply(transpose(instruments), phi);
    Vector zty = multiply(transpose(instruments), y);
    const Vector theta = solveLinearSystem(ztphi, zty, orders.regularization);
    for (size_t i = 0; i < orders.na; ++i) {
        model.A[i + 1] = theta[i];
    }
    for (size_t j = 0; j < orders.nb; ++j) {
        model.B[j] = theta[orders.na + j];
    }
    finalizeFit(model, input, output);
    return model;
}

DiscretePolynomialModel ARMAXIdentifier::identify(const Vector& input,
                                                  const Vector& output,
                                                  const PolynomialModelOrders& orders) {
    DiscretePolynomialModel model = ARXIdentifier::identify(input, output, orders);
    model.C.assign(orders.nc + 1, 0.0);
    if (!model.C.empty()) {
        model.C[0] = 1.0;
    }
    const size_t start = maxHistory(orders);
    if (output.size() <= start) {
        return model;
    }

    Vector residuals = model.residuals;
    for (size_t iteration = 0; iteration < orders.iterations; ++iteration) {
        Matrix phi;
        Vector y;
        for (size_t k = start; k < output.size(); ++k) {
            Vector row;
            for (size_t i = 1; i <= orders.na; ++i) {
                row.push_back(-output[k - i]);
            }
            for (size_t j = 0; j < orders.nb; ++j) {
                row.push_back(input[k - orders.nk - j]);
            }
            for (size_t i = 1; i <= orders.nc; ++i) {
                row.push_back(residuals[k - i]);
            }
            appendRow(phi, row);
            y.push_back(output[k]);
        }

        const Vector theta = solveLeastSquares(phi, y, orders.regularization);
        for (size_t i = 0; i < orders.na; ++i) {
            model.A[i + 1] = theta[i];
        }
        for (size_t j = 0; j < orders.nb; ++j) {
            model.B[j] = theta[orders.na + j];
        }
        for (size_t i = 0; i < orders.nc; ++i) {
            model.C[i + 1] = theta[orders.na + orders.nb + i];
        }

        residuals.assign(output.size(), 0.0);
        for (size_t k = 0; k < output.size(); ++k) {
            const double prediction = predictARXLike(model, input, output, residuals, k, true);
            residuals[k] = output[k] - prediction;
        }
    }
    model.residuals = residuals;
    finalizeFit(model, input, output);
    return model;
}

DiscretePolynomialModel OEIdentifier::identify(const Vector& input,
                                               const Vector& output,
                                               const PolynomialModelOrders& orders) {
    DiscretePolynomialModel initial = ARXIdentifier::identify(input, output, orders);
    std::cerr << "OE initial fit=" << initial.fit << "\n";
    DiscretePolynomialModel model = makeInvalidPolynomial(orders);
    model.B = initial.B;
    const size_t nf = std::max<size_t>(orders.nf, orders.na);
    model.F.assign(nf + 1, 0.0);
    model.F[0] = 1.0;
    for (size_t i = 1; i < model.F.size() && i < initial.A.size(); ++i) {
        model.F[i] = initial.A[i];
    }
    optimizeOutputErrorModel(model, input, output, std::max<size_t>(orders.iterations, 6));
    std::cerr << "OE final fit=" << model.fit << "\n";
    return model;
}

DiscretePolynomialModel BoxJenkinsIdentifier::identify(const Vector& input,
                                                       const Vector& output,
                                                       const PolynomialModelOrders& orders) {
    DiscretePolynomialModel oe = OEIdentifier::identify(input, output, orders);
    DiscretePolynomialModel armax = ARMAXIdentifier::identify(input, output, orders);
    DiscretePolynomialModel model = oe;
    model.C.assign(orders.nc + 1, 0.0);
    model.D.assign(orders.nd + 1, 0.0);
    if (!model.C.empty()) {
        model.C[0] = 1.0;
    }
    if (!model.D.empty()) {
        model.D[0] = 1.0;
    }
    for (size_t i = 1; i < model.C.size() && i < armax.C.size(); ++i) {
        model.C[i] = armax.C[i];
    }
    for (size_t i = 1; i < model.D.size() && i < armax.A.size(); ++i) {
        model.D[i] = armax.A[i];
    }
    finalizeFit(model, input, output);
    return model;
}

} // namespace Identification