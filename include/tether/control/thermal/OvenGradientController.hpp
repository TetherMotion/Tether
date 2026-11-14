/**
 * @file OvenGradientController.hpp
 * @brief Multi-zone temperature-gradient controller built on
 *        OvenController.
 *
 * @details
 * Supervisory layer for ovens with several spatially ordered thermal
 * zones (reflow/conveyor ovens, multi-chamber systems). Each zone runs
 * a full `OvenController` — physical FOPDT model, SIMC-PI or LQI
 * backend, bounded online adaptation, feasibility governor — and this
 * class adds the gradient-specific machinery on top:
 *
 *  - **Gradient references**: one master `TemperatureProfile` is
 *    transformed per zone by `tempOffsetC` (static spatial gradient)
 *    and `timeShiftS` (conveyor-style stagger — the profile reaches
 *    each zone later/earlier), via `TemperatureProfile::transformed`.
 *    Independent per-zone profiles and plain setpoints are also
 *    supported.
 *  - **Gradient constraint**: `maxZoneDeltaC` clamps adjacent-zone
 *    reference offsets at profile-build time and adjacent setpoints at
 *    setpoint-install time, enforcing a maximum temperature difference
 *    between neighboring zones (thermal-shock / product-stress limit).
 *    Runtime measured/setpoint deltas are reported for monitoring.
 *  - **Coupling feedforward**: zones exchange heat through thermal
 *    resistances. The instantaneous coupling heat flow
 *    `d_i = γL·(T_{i-1}−T_i) + γR·(T_{i+1}−T_i)` [°C/s] is injected
 *    into each zone's feedforward via
 *    `OvenController::setCouplingDisturbance`, giving transient-leading
 *    decoupling while each zone's estimator still owns the residual.
 *
 * Real-time notes: `computeInto()` performs one
 * `OvenController::compute` per zone with pre-sized scratch — no
 * per-cycle allocation. Zone count is fixed after zones are added;
 * profiles/setpoints are reference data, swappable freely.
 */

#pragma once

#include "tether/control/thermal/OvenController.hpp"
#include "tether/control/thermal/TemperatureProfile.hpp"

#include <memory>
#include <span>
#include <vector>

namespace tether::control::thermal {

/**@brief Per-zone configuration for OvenGradientController. */
struct GradientZoneConfig {
    OvenControllerConfig controller; /**< Per-zone controller config */

    /**@brief Coupling rate from the LEFT neighbor into this zone [1/s].
     *        dT_i/dt += couplingLeft·(T_{i-1} − T_i). In lumped RC
     *        terms this is 1/(R_{i-1,i}·C_i) — e.g. R=0.5 K/W,
     *        C=3000 J/K → γ≈6.7e-4 1/s. 0 disables. */
    double couplingLeft{0.0};

    /**@brief Coupling rate from the RIGHT neighbor into this zone
     *        [1/s]: dT_i/dt += couplingRight·(T_{i+1} − T_i). */
    double couplingRight{0.0};
};

/**@brief Gradient-profile specification: one master profile transformed
 *        per zone. */
struct GradientSpec {
    TemperatureProfile base;          /**< Master profile (zone 0 shape) */

    /**@brief Per-zone static temperature offset [°C]. Zone i follows
     *        `base(t − shift_i) + offset_i`. Missing entries = 0. */
    std::vector<double> tempOffsetC;

    /**@brief Per-zone time shift [s]. Positive = zone reaches each
     *        profile point later (conveyor downstream zones lag).
     *        Missing entries = 0. */
    std::vector<double> timeShiftS;

    /**@brief Maximum allowed adjacent-zone reference difference [°C].
     *        Offsets are clamped so |offset_i − offset_{i−1}| ≤ this
     *        (forward pass from zone 0). <=0 disables clamping. */
    double maxZoneDeltaC{15.0};
};

class OvenGradientController {
public:
    /**@brief Diagnostics snapshot per zone. */
    struct ZoneStatus {
        OvenControllerStatus controller{}; /**< Zone controller status */
        double couplingDisturb{0.0};       /**< Last injected coupling [°C/s] */
        double refDeltaPrev{0.0};  /**< Governed ref_i − ref_{i-1} [°C] */
        double measDeltaPrev{0.0}; /**< Measured T_i − T_{i-1} [°C] */
    };

    /**@brief Add a zone. Zones MUST be added in spatial order — zone i's
     *        couplingLeft/couplingRight refer to zones i-1/i+1. Zone
     *        count is fixed once compute() has run. */
    void addZone(const GradientZoneConfig& cfg);

    size_t zoneCount() const { return m_zones.size(); }

    /**@brief Install an independent profile on one zone (overrides any
     *        gradient-spec profile for that zone). */
    void setZoneProfile(size_t zone, const TemperatureProfile& profile);

    /**@brief Set a constant setpoint for one zone (clears its profile).
     *        For adjacent-zone delta clamping use setZoneSetpoints. */
    void setZoneSetpoint(size_t zone, double tempC);

    /**@brief Install constant setpoints for all zones at once.
     *        `maxDeltaC` > 0 clamps adjacent deltas
     *        |sp_i − sp_{i-1}| ≤ maxDeltaC (forward pass from zone 0).
     *        Clears all zone profiles. */
    void setZoneSetpoints(const std::vector<double>& tempsC,
                          double maxDeltaC = -1.0);

    /**@brief Build per-zone profiles from a GradientSpec and install
     *        them. Returns false when no zones are configured or the
     *        base profile is empty. */
    bool setGradientProfile(const GradientSpec& spec);

    /**@brief Advance all zones one control cycle (allocating
     *        convenience form — see computeInto for the RT path).
     * @param measured Measured zone temperatures [°C], spatial order,
     *                 size == zoneCount().
     * @param dt       Cycle time [s].
     * @return Per-zone actuator commands u_i in [uMin,uMax]. */
    std::vector<double> compute(std::span<const double> measured,
                                double dt);

    /**@brief Allocation-free form of compute(): writes per-zone
     *        commands into `out` (size == zoneCount()). Returns false
     *        without touching state when sizes mismatch. */
    bool computeInto(std::span<const double> measured, double dt,
                     std::span<double> out);

    /**@brief Largest absolute measured adjacent-zone delta [°C]. */
    double maxMeasuredDelta() const;

    /**@brief Largest absolute governed-reference adjacent-zone delta
     *        [°C] from the last compute(). */
    double maxSetpointDelta() const;

    /**@brief True when the largest measured adjacent-zone delta exceeds
     *        `alarmLimitC` (set via setGradientAlarmLimit). */
    bool gradientAlarm() const { return m_alarm; }

    /**@brief Set the measured-delta alarm threshold [°C]; <=0 disables. */
    void setGradientAlarmLimit(double deltaC) { m_alarmLimitC = deltaC; }

    /**@brief Per-zone status from the last compute(). */
    std::span<const ZoneStatus> zoneStatus() const { return m_status; }

    /**@brief Access a zone controller (tuning, autotune, backend swap). */
    OvenController& zone(size_t i) { return *m_zones.at(i)->ctl; }
    const OvenController& zone(size_t i) const {
        return *m_zones.at(i)->ctl;
    }

    void reset();

private:
    struct Zone {
        GradientZoneConfig cfg;
        std::unique_ptr<OvenController> ctl;
        double setpoint{25.0};
        bool hasProfile{false};
        TemperatureProfile profile;
    };

    std::vector<std::unique_ptr<Zone>> m_zones;
    std::vector<ZoneStatus> m_status;
    std::vector<double> m_lastU;
    std::vector<double> m_lastMeasured;
    std::vector<double> m_coupleD;   // per-cycle coupling scratch
    double m_maxRefDelta{0.0};
    double m_alarmLimitC{0.0};
    bool m_alarm{false};
};

} // namespace tether::control::thermal
