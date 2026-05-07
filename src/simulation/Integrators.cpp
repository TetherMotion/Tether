#include "tether/simulation/Integrators.hpp"
#include <cmath>
#include <algorithm>

namespace Simulation {

// ============================================================================
// Forward Euler
// ============================================================================
IntegratorStep EulerForwardIntegrator::step(const OdeFunction& f, double t,
                                              const StateVector& y, double dt) {
    StateVector dydt = f(t, y);
    StateVector ynew(y.size());
    for (size_t i = 0; i < y.size(); ++i) {
        ynew[i] = y[i] + dt * dydt[i];
    }
    return {ynew, dt, 0.0, true};
}

// ============================================================================
// Classical RK4
// ============================================================================
IntegratorStep RungeKutta4Integrator::step(const OdeFunction& f, double t,
                                             const StateVector& y, double dt) {
    size_t n = y.size();
    StateVector k1 = f(t, y);
    StateVector ytmp(n);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + 0.5 * dt * k1[i];
    StateVector k2 = f(t + 0.5 * dt, ytmp);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + 0.5 * dt * k2[i];
    StateVector k3 = f(t + 0.5 * dt, ytmp);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + dt * k3[i];
    StateVector k4 = f(t + dt, ytmp);

    StateVector ynew(n);
    for (size_t i = 0; i < n; ++i) {
        ynew[i] = y[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    return {ynew, dt, 0.0, true};
}

// ============================================================================
// Dormand-Prince RK45 (adaptive)
// ============================================================================
DormandPrinceRK45Integrator::DormandPrinceRK45Integrator(double atol, double rtol)
    : atol_(atol), rtol_(rtol) {}

IntegratorStep DormandPrinceRK45Integrator::step(const OdeFunction& f, double t,
                                                    const StateVector& y, double dt) {
    // Dormand-Prince coefficients
    static const double a2 = 1.0/5.0, a3 = 3.0/10.0, a4 = 4.0/5.0, a5 = 8.0/9.0;
    static const double b21 = 1.0/5.0;
    static const double b31 = 3.0/40.0, b32 = 9.0/40.0;
    static const double b41 = 44.0/45.0, b42 = -56.0/15.0, b43 = 32.0/9.0;
    static const double b51 = 19372.0/6561.0, b52 = -25360.0/2187.0, b53 = 64448.0/6561.0, b54 = -212.0/729.0;
    static const double b61 = 9017.0/3168.0, b62 = -355.0/33.0, b63 = 46732.0/5247.0, b64 = 49.0/176.0, b65 = -5103.0/18656.0;

    // 5th order coefficients
    static const double c1 = 35.0/384.0, c3 = 500.0/1113.0, c4 = 125.0/192.0, c5 = -2187.0/6784.0, c6 = 11.0/84.0;

    // Error coefficients (5th - 4th order)
    static const double e1 = 71.0/57600.0, e3 = -71.0/16695.0, e4 = 71.0/1920.0, e5 = -17253.0/339200.0, e6 = 22.0/525.0, e7 = -1.0/40.0;

    size_t n = y.size();
    StateVector ytmp(n);

    StateVector k1 = f(t, y);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + dt * b21 * k1[i];
    StateVector k2 = f(t + a2 * dt, ytmp);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + dt * (b31 * k1[i] + b32 * k2[i]);
    StateVector k3 = f(t + a3 * dt, ytmp);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + dt * (b41 * k1[i] + b42 * k2[i] + b43 * k3[i]);
    StateVector k4 = f(t + a4 * dt, ytmp);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + dt * (b51 * k1[i] + b52 * k2[i] + b53 * k3[i] + b54 * k4[i]);
    StateVector k5 = f(t + a5 * dt, ytmp);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + dt * (b61 * k1[i] + b62 * k2[i] + b63 * k3[i] + b64 * k4[i] + b65 * k5[i]);
    StateVector k6 = f(t + dt, ytmp);

    // 5th order solution
    StateVector ynew(n);
    for (size_t i = 0; i < n; ++i) {
        ynew[i] = y[i] + dt * (c1 * k1[i] + c3 * k3[i] + c4 * k4[i] + c5 * k5[i] + c6 * k6[i]);
    }

    // Compute k7 for error estimate
    StateVector k7 = f(t + dt, ynew);

    // Error estimate
    double maxErr = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double ei = dt * (e1 * k1[i] + e3 * k3[i] + e4 * k4[i] + e5 * k5[i] + e6 * k6[i] + e7 * k7[i]);
        double sc = atol_ + rtol_ * std::max(std::abs(y[i]), std::abs(ynew[i]));
        maxErr = std::max(maxErr, std::abs(ei) / sc);
    }

    bool accepted = maxErr <= 1.0;
    return {ynew, dt, maxErr, accepted};
}

// ============================================================================
// Bogacki-Shampine RK23 (adaptive)
// ============================================================================
BogackiShampineRK23Integrator::BogackiShampineRK23Integrator(double atol, double rtol)
    : atol_(atol), rtol_(rtol) {}

IntegratorStep BogackiShampineRK23Integrator::step(const OdeFunction& f, double t,
                                                      const StateVector& y, double dt) {
    size_t n = y.size();
    StateVector ytmp(n);

    StateVector k1 = f(t, y);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + dt * 0.5 * k1[i];
    StateVector k2 = f(t + 0.5 * dt, ytmp);

    for (size_t i = 0; i < n; ++i) ytmp[i] = y[i] + dt * 0.75 * k2[i];
    StateVector k3 = f(t + 0.75 * dt, ytmp);

    // 3rd order solution
    StateVector ynew(n);
    for (size_t i = 0; i < n; ++i) {
        ynew[i] = y[i] + dt * (2.0/9.0 * k1[i] + 1.0/3.0 * k2[i] + 4.0/9.0 * k3[i]);
    }

    StateVector k4 = f(t + dt, ynew);

    // 2nd order solution for error estimate
    double maxErr = 0.0;
    for (size_t i = 0; i < n; ++i) {
        double y2 = y[i] + dt * (7.0/24.0 * k1[i] + 0.25 * k2[i] + 1.0/3.0 * k3[i] + 1.0/8.0 * k4[i]);
        double ei = ynew[i] - y2;
        double sc = atol_ + rtol_ * std::max(std::abs(y[i]), std::abs(ynew[i]));
        maxErr = std::max(maxErr, std::abs(ei) / sc);
    }

    bool accepted = maxErr <= 1.0;
    return {ynew, dt, maxErr, accepted};
}

// ============================================================================
// Implicit Trapezoidal
// ============================================================================
ImplicitTrapezoidalIntegrator::ImplicitTrapezoidalIntegrator(int maxIter, double tol)
    : maxIter_(maxIter), tol_(tol) {}

IntegratorStep ImplicitTrapezoidalIntegrator::step(const OdeFunction& f, double t,
                                                      const StateVector& y, double dt) {
    size_t n = y.size();
    StateVector fn = f(t, y);

    // Initial guess: Forward Euler
    StateVector ynew(n);
    for (size_t i = 0; i < n; ++i) {
        ynew[i] = y[i] + dt * fn[i];
    }

    // Fixed-point iteration
    for (int iter = 0; iter < maxIter_; ++iter) {
        StateVector fnew = f(t + dt, ynew);
        StateVector ynext(n);
        double maxDiff = 0.0;
        for (size_t i = 0; i < n; ++i) {
            ynext[i] = y[i] + 0.5 * dt * (fn[i] + fnew[i]);
            maxDiff = std::max(maxDiff, std::abs(ynext[i] - ynew[i]));
        }
        ynew = ynext;
        if (maxDiff < tol_) break;
    }

    return {ynew, dt, 0.0, true};
}

// ============================================================================
// BDF2
// ============================================================================
BDF2Integrator::BDF2Integrator(int maxIter, double tol)
    : maxIter_(maxIter), tol_(tol) {}

IntegratorStep BDF2Integrator::step(const OdeFunction& f, double t,
                                       const StateVector& y, double dt) {
    size_t n = y.size();

    if (!hasPrev_) {
        // First step: fall back to implicit trapezoidal
        ImplicitTrapezoidalIntegrator trap(maxIter_, tol_);
        auto result = trap.step(f, t, y, dt);
        yPrev_ = y;
        hasPrev_ = true;
        return result;
    }

    // BDF2: (4/3)*y_{n+1} - (1/3)*y_{n-1} = (2/3)*dt*f(t+dt, y_{n+1}) + y_n - ...
    // Simplified: y_{n+1} = (4/3)*y_n - (1/3)*y_{n-1} + (2/3)*dt*f(t+dt, y_{n+1})

    // Initial guess
    StateVector ynew(n);
    for (size_t i = 0; i < n; ++i) {
        ynew[i] = (4.0/3.0) * y[i] - (1.0/3.0) * yPrev_[i];
    }

    // Fixed-point iteration
    for (int iter = 0; iter < maxIter_; ++iter) {
        StateVector fnew = f(t + dt, ynew);
        StateVector ynext(n);
        double maxDiff = 0.0;
        for (size_t i = 0; i < n; ++i) {
            ynext[i] = (4.0/3.0) * y[i] - (1.0/3.0) * yPrev_[i] + (2.0/3.0) * dt * fnew[i];
            maxDiff = std::max(maxDiff, std::abs(ynext[i] - ynew[i]));
        }
        ynew = ynext;
        if (maxDiff < tol_) break;
    }

    yPrev_ = y;
    return {ynew, dt, 0.0, true};
}

// ============================================================================
// Factory
// ============================================================================
std::unique_ptr<Integrator> createIntegrator(IntegrationMethod method, double atol, double rtol) {
    switch (method) {
        case IntegrationMethod::EulerForward:
            return std::make_unique<EulerForwardIntegrator>();
        case IntegrationMethod::RungeKutta4:
            return std::make_unique<RungeKutta4Integrator>();
        case IntegrationMethod::DormandPrinceRK45:
            return std::make_unique<DormandPrinceRK45Integrator>(atol, rtol);
        case IntegrationMethod::BogackiShampineRK23:
            return std::make_unique<BogackiShampineRK23Integrator>(atol, rtol);
        case IntegrationMethod::ImplicitTrapezoidal:
            return std::make_unique<ImplicitTrapezoidalIntegrator>();
        case IntegrationMethod::BDF2:
            return std::make_unique<BDF2Integrator>();
        default:
            return std::make_unique<RungeKutta4Integrator>();
    }
}

} // namespace Simulation
