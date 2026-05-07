#pragma once

#include <cstddef>
#include <vector>

#include <tether/identification/DenseLinearAlgebra.hpp>

namespace Identification {

struct PolynomialModelOrders {
    size_t na{1};
    size_t nb{1};
    size_t nc{0};
    size_t nd{0};
    size_t nf{0};
    size_t nk{1};
    size_t iterations{8};
    double regularization{1e-6};
};

struct DiscretePolynomialModel {
    PolynomialModelOrders orders{};
    std::vector<double> A{1.0};
    std::vector<double> B{};
    std::vector<double> C{1.0};
    std::vector<double> D{1.0};
    std::vector<double> F{1.0};
    std::vector<double> residuals{};
    double mse{0.0};
    double fit{0.0};

    double predictOneStep(const Vector& input, const Vector& output, size_t index) const;
    Vector simulate(const Vector& input) const;
    bool valid() const;
};

class ARXIdentifier {
public:
    static DiscretePolynomialModel identify(const Vector& input,
                                            const Vector& output,
                                            const PolynomialModelOrders& orders);
};

class InstrumentalVariablesIdentifier {
public:
    static DiscretePolynomialModel identify(const Vector& input,
                                            const Vector& output,
                                            const PolynomialModelOrders& orders);
};

class ARMAXIdentifier {
public:
    static DiscretePolynomialModel identify(const Vector& input,
                                            const Vector& output,
                                            const PolynomialModelOrders& orders);
};

class OEIdentifier {
public:
    static DiscretePolynomialModel identify(const Vector& input,
                                            const Vector& output,
                                            const PolynomialModelOrders& orders);
};

class BoxJenkinsIdentifier {
public:
    static DiscretePolynomialModel identify(const Vector& input,
                                            const Vector& output,
                                            const PolynomialModelOrders& orders);
};

} // namespace Identification