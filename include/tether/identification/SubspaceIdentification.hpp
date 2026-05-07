#pragma once

#include <cstddef>

#include <tether/identification/DenseLinearAlgebra.hpp>

namespace Identification {

struct StateSpaceModel {
    Matrix A;
    Matrix B;
    Matrix C;
    Matrix D;
    Vector singular_values;
    size_t order{0};
    double fit{0.0};

    bool valid() const;
    Vector propagate(const Vector& state, const Vector& input) const;
    Vector output(const Vector& state, const Vector& input) const;
};

class N4SIDIdentifier {
public:
    static StateSpaceModel identify(const Matrix& inputs,
                                    const Matrix& outputs,
                                    size_t block_rows,
                                    size_t model_order = 0);
};

class MOESPIdentifier {
public:
    static StateSpaceModel identify(const Matrix& inputs,
                                    const Matrix& outputs,
                                    size_t block_rows,
                                    size_t model_order = 0);
};

class CVAIdentifier {
public:
    static StateSpaceModel identify(const Matrix& inputs,
                                    const Matrix& outputs,
                                    size_t block_rows,
                                    size_t model_order = 0);
};

} // namespace Identification