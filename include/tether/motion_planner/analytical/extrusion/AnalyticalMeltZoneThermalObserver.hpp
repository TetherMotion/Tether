/**
 * @file AnalyticalMeltZoneThermalObserver.hpp
 * @brief Analytical three-state melt-zone thermal observer on WSS arcs.
 *
 * @details
 * The three-state thermal model:
 *
 *   C_h · dT_h/dt = P_heater - G_hs·(T_h - T_s)
 *   C_s · dT_s/dt = G_hs·(T_h - T_s) - G_sm·(T_s - T_m)
 *   C_m · dT_m/dt = G_sm·(T_s - T_m) - ρ·c_p·Q(t)·(T_m - T_inlet)
 *
 * In matrix form: dT/dt = A_th · T + b(t), where T = [T_h, T_s, T_m]^T.
 *
 * The matrix A_th is constant (thermal properties don't change), but the
 * enthalpy drain term -ρ·c_p·Q(t)·T_m makes the system linear time-varying
 * because Q(t) multiplies the state T_m.
 *
 * **Piecewise-constant Q approximation (per arc):**
 *
 * Within each WSS arc, Q(t) is approximated as constant (the arc's
 * time-averaged flow Q̄).  The system becomes LTI:
 *
 *   dT/dt = A_arc · T + b_arc
 *
 * where A_arc incorporates Q̄ in the (3,3) entry and b_arc contains the
 * heater power and Q̄·T_inlet terms.  The solution is:
 *
 *   T(t) = exp(A_arc·Δt) · T₀ + A_arc⁻¹·(exp(A_arc·Δt) - I) · b_arc
 *
 * This is a matrix exponential — computed once per arc via Eigen.
 *
 * **Luenberger correction:**
 *
 * At measurement times, the state is corrected:
 *   T += L · (T_s_measured - T_s) · Δt
 *
 * In the analytical version, this is applied as an impulsive correction.
 *
 * @see docs/extrusion/AnalyticalExtrusionCompensation.md §9
 */

#pragma once

#include "AnalyticalExtrusionTypes.hpp"

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <cmath>
#include <vector>

namespace MotionPlanner::analytical::extrusion {

/**
 * @brief Parameters for the analytical melt-zone thermal observer.
 */
struct AnalyticalThermalParams {
    // --- Thermal capacitances [J/K] ---
    double heaterBlockCapacitance = 8.0;   ///< C_h
    double sensorCapacitance      = 1.0;   ///< C_s
    double meltZoneCapacitance    = 2.0;   ///< C_m

    // --- Thermal conductances [W/K] ---
    double heaterSensorConductance = 2.0;  ///< G_hs
    double sensorMeltConductance   = 1.5;  ///< G_sm

    // --- Filament properties ---
    double filamentHeatCapacity = 2.1;     ///< ρ·c_p [J/(mm³·K)]
    double inletTempC           = 25.0;    ///< T_inlet [°C]

    // --- Heater ---
    double heaterPowerScale = 40.0;        ///< [W] at PWM=1
    double heaterPWM        = 0.0;         ///< Heater PWM fraction [0,1]

    // --- Luenberger gains [1/s] ---
    double luenbergerGainHeater = 0.5;
    double luenbergerGainSensor = 2.0;
    double luenbergerGainMelt   = 0.3;

    // --- Filament diameter for flow computation [mm] ---
    double filamentDiameterMm = 1.75;
};

/**
 * @brief Analytical three-state melt-zone thermal observer.
 *
 * Propagates the three thermal states (T_h, T_s, T_m) through the WSS
 * trajectory using matrix exponentials per arc.  Provides closed-form
 * temperature evaluation at any time t.
 */
template<size_t Dim, typename T = double>
class AnalyticalMeltZoneThermalObserver {
public:
    using Traj = ExtrusionTrajectory<Dim, T>;
    using StateVec = Eigen::Vector3d;
    using StateMat = Eigen::Matrix3d;

    /**
     * @brief Per-arc precomputed thermal solution.
     */
    struct ArcThermalSolution {
        double t0 = 0.0;         ///< Arc start time
        double duration = 0.0;   ///< Arc duration
        StateVec stateStart;     ///< State at arc start
        StateMat expA;           ///< exp(A_arc · Δt)
        StateVec steadyState;    ///< A_arc⁻¹ · b_arc (steady-state offset)
        double Qbar = 0.0;       ///< Average flow for this arc

        /**
         * @brief Evaluate the thermal state at local time tau ∈ [0, duration].
         *
         * T(τ) = exp(A·τ) · T₀ + A⁻¹·(exp(A·τ) - I) · b
         *
         * We precompute exp(A·Δt) and use the property:
         *   exp(A·τ) = exp(A·Δt)^(τ/Δt)
         * But for arbitrary τ, we compute exp(A·τ) directly via scaling.
         * For efficiency, we use the interpolation:
         *   T(τ) ≈ (1-α)·T₀ + α·T_end  (linear in state, valid for short arcs)
         * where α = τ/Δt and T_end = expA·T₀ + (expA - I)·steadyState.
         *
         * For higher accuracy, we compute exp(A·τ) via Eigen.
         */
        StateVec stateAt(double tau, const StateMat& A_arc) const {
            if (tau <= 0.0) return stateStart;
            if (tau >= duration) return stateAtEnd();

            // Compute exp(A·τ) — for short arcs, linear interpolation suffices
            // For accuracy, use the matrix exponential
            StateMat expAtau = (A_arc * tau).exp();
            return expAtau * stateStart + (expAtau - StateMat::Identity())
                   * steadyState;
        }

        StateVec stateAtEnd() const {
            return expA * stateStart + (expA - StateMat::Identity())
                   * steadyState;
        }
    };

    /**
     * @brief Construct from trajectory and parameters.
     */
    AnalyticalMeltZoneThermalObserver(const Traj& traj,
                                       AnalyticalThermalParams params)
        : traj_(&traj), params_(params) {
        filamentAreaMm2_ = M_PI * params_.filamentDiameterMm
                           * params_.filamentDiameterMm / 4.0;
        // Initialize all states to inlet temperature
        state_ << params_.inletTempC, params_.inletTempC, params_.inletTempC;
        precompute();
    }

    /**
     * @brief Initialize all states to a known temperature.
     */
    void initialize(double heaterBlockTempC, double sensorTempC,
                    double meltTempC) {
        state_ << heaterBlockTempC, sensorTempC, meltTempC;
        precompute();
    }

    /// Convenience: initialize all to the same temperature
    void initialize(double tempC) {
        initialize(tempC, tempC, tempC);
    }

    /**
     * @brief Evaluate the melt-zone temperature at time t [°C].
     */
    double meltTempAt(double t) const {
        return stateAt(t)[2];
    }

    /**
     * @brief Evaluate the heater-block temperature at time t [°C].
     */
    double heaterBlockTempAt(double t) const {
        return stateAt(t)[0];
    }

    /**
     * @brief Evaluate the sensor-point temperature at time t [°C].
     */
    double sensorTempAt(double t) const {
        return stateAt(t)[1];
    }

    /**
     * @brief Get the full thermal state [T_h, T_s, T_m] at time t.
     */
    StateVec stateAt(double t) const {
        if (solutions_.empty()) return state_;
        if (t <= 0.0) return solutions_.front().stateStart;

        // Find the arc containing t
        size_t idx = findArc(t);
        const auto& sol = solutions_[idx];
        double tau = std::clamp(t - sol.t0, 0.0, sol.duration);
        return sol.stateAt(tau, arcMatrices_[idx]);
    }

    /**
     * @brief Apply a Luenberger correction at time t.
     *
     * T += L · (T_s_measured - T_s(t)) · dt
     *
     * This modifies the state at the correction time and recomputes
     * all subsequent arc solutions.
     *
     * @param t Correction time [s]
     * @param measuredSensorTempC Measured thermistor reading [°C]
     * @param dt Correction time step [s]
     */
    void applyLuenbergerCorrection(double t, double measuredSensorTempC,
                                    double dt) {
        StateVec current = stateAt(t);
        double innovation = measuredSensorTempC - current[1];
        current[0] += params_.luenbergerGainHeater * innovation * dt;
        current[1] += params_.luenbergerGainSensor * innovation * dt;
        current[2] += params_.luenbergerGainMelt * innovation * dt;

        // Recompute from the correction point forward
        size_t idx = findArc(t);
        if (idx < solutions_.size()) {
            solutions_[idx].stateStart = current;
            recomputeFrom(idx);
        }
    }

    /**
     * @brief Get the melt temperature at multiple time points.
     */
    std::vector<double> meltTempSeries(const std::vector<double>& times) const {
        std::vector<double> result;
        result.reserve(times.size());
        for (double t : times)
            result.push_back(meltTempAt(t));
        return result;
    }

    /// Parameters
    const AnalyticalThermalParams& params() const { return params_; }

    /// Trajectory
    const Traj& trajectory() const { return *traj_; }

    /// Current state at t=0
    StateVec initialState() const { return state_; }

private:
    const Traj* traj_;
    AnalyticalThermalParams params_;
    double filamentAreaMm2_ = 0.0;
    StateVec state_;

    std::vector<ArcThermalSolution> solutions_;
    std::vector<StateMat> arcMatrices_;  ///< A_arc for each arc

    size_t findArc(double t) const {
        if (solutions_.empty()) return 0;
        size_t lo = 0, hi = solutions_.size();
        while (lo < hi) {
            size_t mid = (lo + hi) / 2;
            if (solutions_[mid].t0 + solutions_[mid].duration < t)
                lo = mid + 1;
            else
                hi = mid;
        }
        return std::min(lo, solutions_.size() - 1);
    }

    /**
     * @brief Build the A_arc matrix and b_arc vector for a given average flow.
     *
     * dT_h/dt = (P_heater - G_hs·(T_h - T_s)) / C_h
     * dT_s/dt = (G_hs·(T_h - T_s) - G_sm·(T_s - T_m)) / C_s
     * dT_m/dt = (G_sm·(T_s - T_m) - ρ·c_p·Q·(T_m - T_inlet)) / C_m
     */
    void buildArcSystem(double Qbar, StateMat& A, StateVec& b) const {
        double Ch = params_.heaterBlockCapacitance;
        double Cs = params_.sensorCapacitance;
        double Cm = params_.meltZoneCapacitance;
        double Ghs = params_.heaterSensorConductance;
        double Gsm = params_.sensorMeltConductance;
        double rhoCp = params_.filamentHeatCapacity;
        double Pheater = params_.heaterPWM * params_.heaterPowerScale;
        double Tinlet = params_.inletTempC;

        A = StateMat::Zero();
        A(0, 0) = -Ghs / Ch;
        A(0, 1) =  Ghs / Ch;
        A(1, 0) =  Ghs / Cs;
        A(1, 1) = -(Ghs + Gsm) / Cs;
        A(1, 2) =  Gsm / Cs;
        A(2, 1) =  Gsm / Cm;
        A(2, 2) = -(Gsm + rhoCp * Qbar) / Cm;

        b = StateVec::Zero();
        b[0] = Pheater / Ch;
        b[2] = rhoCp * Qbar * Tinlet / Cm;
    }

    void precompute() {
        const auto& arcs = traj_->arcs();
        solutions_.clear();
        arcMatrices_.clear();
        solutions_.reserve(arcs.size());
        arcMatrices_.reserve(arcs.size());

        StateVec currentState = state_;
        for (const auto& a : arcs) {
            // Compute average flow for this arc
            double avgVE = a.avgExtruderVelocity();
            double Qbar = avgVE * filamentAreaMm2_;

            StateMat A;
            StateVec b;
            buildArcSystem(Qbar, A, b);

            // Compute exp(A·Δt) and steady-state = A⁻¹·b
            double dt = a.duration;
            StateMat expAdt = (A * dt).exp();

            // Steady state: A⁻¹·b (if A is invertible)
            StateVec steadyState = A.colPivHouseholderQr().solve(b);

            ArcThermalSolution sol;
            sol.t0 = a.t0;
            sol.duration = dt;
            sol.stateStart = currentState;
            sol.expA = expAdt;
            sol.steadyState = steadyState;
            sol.Qbar = Qbar;
            solutions_.push_back(sol);
            arcMatrices_.push_back(A);

            // Propagate state to end of arc
            currentState = sol.stateAtEnd();
        }
    }

    void recomputeFrom(size_t startIdx) {
        if (startIdx >= solutions_.size()) return;
        StateVec currentState = solutions_[startIdx].stateStart;
        for (size_t i = startIdx; i < solutions_.size(); ++i) {
            solutions_[i].stateStart = currentState;
            currentState = solutions_[i].stateAtEnd();
        }
    }
};

} // namespace MotionPlanner::analytical::extrusion
