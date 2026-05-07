#include <tether/identification/SubspaceIdentification.hpp>

#include <algorithm>

#include "IdentificationInternal.hpp"

namespace Identification {

namespace {

StateSpaceModel identifySubspaceVariant(const Matrix& inputs,
                                        const Matrix& outputs,
                                        size_t block_rows,
                                        size_t model_order,
                                        int variant) {
    StateSpaceModel model;
    const size_t sample_count = std::min(inputs.size(), outputs.size());
    if (sample_count < 2 * block_rows + 2 || inputs.empty() || outputs.empty()) {
        return model;
    }

    const size_t input_dim = inputs.front().size();
    const size_t output_dim = outputs.front().size();
    Matrix past;
    Matrix future_y;
    Matrix future_u;

    for (size_t k = block_rows; k + block_rows < sample_count; ++k) {
        Vector past_row = detail::extractWindow(inputs, k - block_rows, block_rows);
        Vector past_outputs = detail::extractWindow(outputs, k - block_rows, block_rows);
        past_row.insert(past_row.end(), past_outputs.begin(), past_outputs.end());
        appendRow(past, past_row);
        appendRow(future_y, detail::extractWindow(outputs, k, block_rows));
        appendRow(future_u, detail::extractWindow(inputs, k, block_rows));
    }

    Matrix feature_space;
    if (variant == 0) {
        feature_space = multiply(past, detail::leastSquaresMatrix(past, future_y));
    } else if (variant == 1) {
        const Matrix future_u_effect = multiply(future_u, detail::leastSquaresMatrix(future_u, future_y));
        feature_space = subtract(future_y, future_u_effect);
    } else {
        const Matrix past_whitening = detail::inverseSqrtSymmetric(detail::covariance(past));
        const Matrix future_whitening = detail::inverseSqrtSymmetric(detail::covariance(future_y));
        const Matrix past_w = multiply(past, past_whitening);
        const Matrix future_w = multiply(future_y, future_whitening);
        feature_space = multiply(past_w, detail::leastSquaresMatrix(past_w, future_w));
    }

    if (feature_space.empty()) {
        return model; // GCOVR_EXCL_LINE
    }

    const Matrix gram = multiply(transpose(feature_space), feature_space);
    const EigenDecomposition eig = jacobiEigenDecomposition(gram);
    model.singular_values.resize(eig.values.size(), 0.0);
    for (size_t i = 0; i < eig.values.size(); ++i) {
        model.singular_values[i] = std::sqrt(std::max(eig.values[i], 0.0));
    }

    size_t order = model_order;
    if (order == 0 && !model.singular_values.empty()) {
        const double threshold = 0.05 * model.singular_values.front();
        for (double singular_value : model.singular_values) {
            if (singular_value > std::max(threshold, 1e-6)) {
                ++order;
            }
        }
        order = std::max<size_t>(1, order);
    }
    order = std::min(order, detail::colCount(feature_space));
    model.order = order;

    Matrix basis = makeMatrix(detail::colCount(feature_space), order, 0.0);
    for (size_t c = 0; c < order; ++c) {
        Vector eigenvector = column(eig.vectors, c);
        for (size_t r = 0; r < eigenvector.size(); ++r) {
            basis[r][c] = eigenvector[r];
        }
    }

    Matrix state_rows = multiply(feature_space, basis);
    if (state_rows.size() < 2) {
        return model;
    }

    Matrix Xk(state_rows.size() - 1, Vector(order, 0.0));
    Matrix Xk1(state_rows.size() - 1, Vector(order, 0.0));
    Matrix Uk(state_rows.size() - 1, Vector(input_dim, 0.0));
    Matrix Yk(state_rows.size() - 1, Vector(output_dim, 0.0));

    for (size_t i = 0; i + 1 < state_rows.size(); ++i) {
        Xk[i] = state_rows[i];
        Xk1[i] = state_rows[i + 1];
        Uk[i] = inputs[block_rows + i];
        Yk[i] = outputs[block_rows + i];
    }

    const Matrix state_input = detail::concatHorizontal(Xk, Uk);
    const Matrix ab = detail::leastSquaresMatrix(state_input, Xk1);
    const Matrix cd = detail::leastSquaresMatrix(state_input, Yk);

    model.A = makeMatrix(order, order, 0.0);
    model.B = makeMatrix(order, input_dim, 0.0);
    model.C = makeMatrix(output_dim, order, 0.0);
    model.D = makeMatrix(output_dim, input_dim, 0.0);

    for (size_t r = 0; r < order; ++r) {
        for (size_t c = 0; c < order; ++c) {
            model.A[r][c] = ab[c][r];
        }
        for (size_t c = 0; c < input_dim; ++c) {
            model.B[r][c] = ab[order + c][r];
        }
    }
    for (size_t r = 0; r < output_dim; ++r) {
        for (size_t c = 0; c < order; ++c) {
            model.C[r][c] = cd[c][r];
        }
        for (size_t c = 0; c < input_dim; ++c) {
            model.D[r][c] = cd[order + c][r];
        }
    }

    Vector x(order, 0.0);
    Vector measured_signal;
    Vector predicted_signal;
    measured_signal.reserve(sample_count * output_dim);
    predicted_signal.reserve(sample_count * output_dim);
    for (size_t k = 0; k < sample_count; ++k) {
        const Vector yhat = model.output(x, inputs[k]);
        predicted_signal.insert(predicted_signal.end(), yhat.begin(), yhat.end());
        measured_signal.insert(measured_signal.end(), outputs[k].begin(), outputs[k].end());
        x = model.propagate(x, inputs[k]);
    }
    model.fit = detail::computeFitPercent(measured_signal, predicted_signal);
    return model;
}

} // namespace

bool StateSpaceModel::valid() const {
    return !A.empty() && !B.empty() && !C.empty();
}

Vector StateSpaceModel::propagate(const Vector& state, const Vector& input) const {
    return detail::addVector(multiply(A, state), multiply(B, input));
}

Vector StateSpaceModel::output(const Vector& state, const Vector& input) const {
    return detail::addVector(multiply(C, state), multiply(D, input));
}

StateSpaceModel N4SIDIdentifier::identify(const Matrix& inputs,
                                          const Matrix& outputs,
                                          size_t block_rows,
                                          size_t model_order) {
    return identifySubspaceVariant(inputs, outputs, block_rows, model_order, 0);
}

StateSpaceModel MOESPIdentifier::identify(const Matrix& inputs,
                                          const Matrix& outputs,
                                          size_t block_rows,
                                          size_t model_order) {
    return identifySubspaceVariant(inputs, outputs, block_rows, model_order, 1);
}

StateSpaceModel CVAIdentifier::identify(const Matrix& inputs,
                                        const Matrix& outputs,
                                        size_t block_rows,
                                        size_t model_order) {
    return identifySubspaceVariant(inputs, outputs, block_rows, model_order, 2);
}

} // namespace Identification