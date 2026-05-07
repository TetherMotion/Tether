#pragma once
#include "SimulationTypes.hpp"
#include <functional>

namespace Simulation {

/// ODE right-hand side function type: f(t, y) -> dydt
using OdeFunction = std::function<StateVector(double, const StateVector&)>;

/// Integrator result
struct IntegratorStep {
    StateVector state;
    double dt_used;
    double error_estimate;
    bool accepted;
};

/// Abstract integrator interface
class Integrator {
public:
    virtual ~Integrator() = default;
    virtual IntegratorStep step(const OdeFunction& f, double t,
                                 const StateVector& y, double dt) = 0;
    virtual const char* name() const = 0;
    virtual bool isAdaptive() const { return false; }
    virtual bool isImplicit() const { return false; }
};

/// Forward Euler
class EulerForwardIntegrator : public Integrator {
public:
    IntegratorStep step(const OdeFunction& f, double t,
                         const StateVector& y, double dt) override;
    const char* name() const override { return "Forward Euler"; }
};

/// Classical 4th-order Runge-Kutta
class RungeKutta4Integrator : public Integrator {
public:
    IntegratorStep step(const OdeFunction& f, double t,
                         const StateVector& y, double dt) override;
    const char* name() const override { return "RK4"; }
};

/// Dormand-Prince RK45 (adaptive)
class DormandPrinceRK45Integrator : public Integrator {
public:
    DormandPrinceRK45Integrator(double atol = 1e-6, double rtol = 1e-6);
    IntegratorStep step(const OdeFunction& f, double t,
                         const StateVector& y, double dt) override;
    const char* name() const override { return "Dormand-Prince RK45"; }
    bool isAdaptive() const override { return true; }

    void setTolerances(double atol, double rtol) { atol_ = atol; rtol_ = rtol; }

private:
    double atol_, rtol_;
};

/// Bogacki-Shampine RK23 (adaptive)
class BogackiShampineRK23Integrator : public Integrator {
public:
    BogackiShampineRK23Integrator(double atol = 1e-6, double rtol = 1e-6);
    IntegratorStep step(const OdeFunction& f, double t,
                         const StateVector& y, double dt) override;
    const char* name() const override { return "Bogacki-Shampine RK23"; }
    bool isAdaptive() const override { return true; }

    void setTolerances(double atol, double rtol) { atol_ = atol; rtol_ = rtol; }

private:
    double atol_, rtol_;
};

/// Implicit trapezoidal method (for stiff systems)
class ImplicitTrapezoidalIntegrator : public Integrator {
public:
    ImplicitTrapezoidalIntegrator(int maxIter = 20, double tol = 1e-10);
    IntegratorStep step(const OdeFunction& f, double t,
                         const StateVector& y, double dt) override;
    const char* name() const override { return "Implicit Trapezoidal"; }
    bool isImplicit() const override { return true; }

private:
    int maxIter_;
    double tol_;
};

/// BDF2 (Backward Differentiation Formula, order 2) for stiff systems
class BDF2Integrator : public Integrator {
public:
    BDF2Integrator(int maxIter = 20, double tol = 1e-10);
    IntegratorStep step(const OdeFunction& f, double t,
                         const StateVector& y, double dt) override;
    const char* name() const override { return "BDF2"; }
    bool isImplicit() const override { return true; }

    void setPreviousState(const StateVector& yPrev) { yPrev_ = yPrev; hasPrev_ = true; }

private:
    int maxIter_;
    double tol_;
    StateVector yPrev_;
    bool hasPrev_ = false;
};

/// Factory for creating integrators
std::unique_ptr<Integrator> createIntegrator(IntegrationMethod method,
                                               double atol = 1e-6,
                                               double rtol = 1e-6);

} // namespace Simulation
