/**
 * @file OvenCoupledModel.hpp
 * @brief Two-state coupled thermal model for non-collocated sensing.
 *
 * @details
 * When the temperature sensor is NOT collocated with the thermal mass that
 * matters (e.g. a chamber-air thermistor while the product/load temperature
 * is the controlled variable), a single-state FOPDT model is structurally
 * inadequate: it cannot represent the unmeasured product state, so no
 * observer-based feedback is possible.
 *
 * This header defines the physical parameter set for a two-state lumped
 * model:
 *
 * ```
 *        heater u(t-θ)         ambient Tamb
 *             │                  │
 *             ▼                  ▼
 *   ┌─────────────────┐   ┌──────────────────┐
 *   │  zone 1 (sensor)│   │ zone 2 (product) │
 *   │   T_s measured  │──►│   T_p controlled │
 *   └─────────────────┘   └──────────────────┘
 *        C_s·dT_s/dt         C_p·dT_p/dt
 * ```
 *
 * Continuous-time dynamics (all rates in 1/s, temperatures in °C):
 *
 * ```
 * dx1/dt = α·u_d − βS·(x1−Tamb) − γSP·(x1−x2)
 * dx2/dt = γPS·(x1−x2) − βP·(x2−Tamb) + d
 * ```
 *
 * where x1 = sensor-zone temperature, x2 = product-zone temperature,
 * u_d = delayed actuator input, and d = additive disturbance (°C/s,
 * e.g. endothermic/exothermic load reactions) acting on the product zone.
 *
 * Parameter semantics:
 *  - α     : heater power coefficient into zone 1 [°C/s per unit u]
 *  - βS    : zone-1 ambient loss rate [1/s]  (= G_amb1/C_s)
 *  - βP    : zone-2 ambient loss rate [1/s]  (= G_amb2/C_p)
 *  - γSP   : coupling zone1→zone2 rate [1/s] (= G_couple/C_s)
 *  - γPS   : coupling zone2→zone1 rate [1/s] (= G_couple/C_p)
 *
 * Note γSP·C_s = γPS·C_p = G_couple: the two coupling rates are NOT
 * independent — their ratio is the inverse capacitance ratio. Both are
 * exposed because commissioning data (e.g. cool-down curves with/without
 * product) constrains them separately in practice.
 *
 * State-space form used by the observer/LQI backend:
 *   A = [[-(βS+γSP),  γSP ],
 *        [  γPS,    -(γPS+βP)]]
 *   B = [α, 0]ᵀ          (input enters zone 1 only)
 *   c = [βS·Tamb, βP·Tamb + d]ᵀ   (known affine term)
 *   C_meas  = [1, 0]     (sensor measures zone 1)
 *   C_ctrl  = [0, 1]     (controlled variable is zone 2 / product)
 */

#pragma once

#include "tether/control/thermal/OvenThermalModel.hpp"

namespace tether::control::thermal {

/**@brief Physical parameters of the two-state coupled oven model. */
struct OvenCoupledParams {
    double alpha{0.5};       /**< Heater coefficient into zone 1 [°C/s/u] */
    double betaSensor{0.01}; /**< Zone-1 ambient loss [1/s] */
    double betaProduct{0.003}; /**< Zone-2 ambient loss [1/s] */
    double gammaSP{0.01};    /**< Coupling zone1→zone2 [1/s] */
    double gammaPS{0.004};   /**< Coupling zone2→zone1 [1/s] */
    double delay{4.0};       /**< Input dead time [s] */
    double disturbance{0.0}; /**< Additive product-zone disturbance [°C/s] */
    double ambient{25.0};    /**< Ambient temperature [°C] */
    double uMin{0.0};        /**< Minimum actuator output */
    double uMax{1.0};        /**< Maximum actuator output */
};

/**
 * @brief Build nominal coupled parameters from a FOPDT parameter set.
 *
 * Maps the single-state {α, β, θ, d} identification onto the two-state
 * model: α and the dominant loss rate carry over to zone 1, the product
 * zone gets a fraction `productLossRatio` of the loss and fixed coupling
 * rates. The coupling defaults produce a product-zone time constant
 * roughly 3-5x the sensor-zone constant — refine via `coupled` config or
 * dedicated commissioning experiments (see docs/control/OvenThermalControl.md).
 *
 * @param m              FOPDT parameters (e.g. from autotune/estimator)
 * @param productLossRatio  βP = β·ratio
 * @param couplingSP        γSP override [1/s]; <=0 keeps default
 * @param couplingPS        γPS override [1/s]; <=0 keeps default
 */
inline OvenCoupledParams makeCoupledParams(
    const OvenModelParams& m,
    double productLossRatio = 0.3,
    double couplingSP = -1.0,
    double couplingPS = -1.0) {
    OvenCoupledParams c;
    c.alpha = m.alpha;
    c.betaSensor = m.beta;
    c.betaProduct = m.beta * productLossRatio;
    c.gammaSP = (couplingSP > 0.0) ? couplingSP : 0.4 * m.beta;
    c.gammaPS = (couplingPS > 0.0) ? couplingPS : 0.15 * m.beta;
    c.delay = m.delay;
    c.disturbance = m.disturbance;
    c.ambient = m.ambient;
    c.uMin = m.uMin;
    c.uMax = m.uMax;
    return c;
}

} // namespace tether::control::thermal
