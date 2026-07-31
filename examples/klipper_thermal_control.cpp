/**
 * @file klipper_thermal_control.cpp
 * @brief Example: thermal control with thermistor, heater, and PID.
 *
 * @details
 * Demonstrates the thermal control subsystem:
 *   1. Create a Thermistor with custom parameters and a simulated ADC.
 *   2. Create a Heater with PID parameters.
 *   3. Set a target temperature and run PID control iterations.
 *   4. Monitor temperature over time as it approaches the target.
 *   5. Simulate PID tuning (M303) by measuring heating/cooling response.
 */

#include "tether/klipper/objects/Thermal.hpp"

#include <cstdio>
#include <cmath>
#include <memory>

using namespace tether::klipper::objects;

int main() {
    // 1. Create a Thermistor with custom parameters
    // Simulate a 100K NTC thermistor (beta 3950) on a 12-bit ADC
    // We model the ADC value as a function of temperature.
    Thermistor::Params params;
    params.pullupResistor = 4700.0;
    params.referenceVoltage = 3.3;
    params.adcMax = 4095.0;
    params.resistanceAt25C = 100000.0;
    params.beta = 3950.0;

    // Simulated temperature state (starts at room temp)
    double simTemp = 24.0;

    // ADC read function: compute ADC value from simulated temperature
    auto adcRead = [&]() -> double {
        // R_therm = R25 * exp(beta * (1/T - 1/T0))
        double t0 = 298.15;
        double tempK = simTemp + 273.15;
        double rTherm = params.resistanceAt25C *
            std::exp(params.beta * (1.0 / tempK - 1.0 / t0));
        // V_adc = V_ref * R_therm / (R_pullup + R_therm)
        double vadc = params.referenceVoltage * rTherm /
                      (params.pullupResistor + rTherm);
        return vadc * params.adcMax / params.referenceVoltage;
    };

    auto thermistor = std::make_shared<Thermistor>(0, params, adcRead);
    std::printf("Thermistor OID=%u type=%s initial temp=%.2f°C\n",
                thermistor->oid(), thermistor->type().c_str(),
                thermistor->read());

    // 2. Create a Heater with PID parameters
    double pwmOutput = 0.0;
    auto heater = std::make_shared<Heater>(
        0, [&](double pwm) { pwmOutput = pwm; },
        [&]() { return thermistor->read(); });

    HeaterPidParams pid;
    pid.Kp = 14.0;
    pid.Ki = 0.5;
    pid.Kd = 50.0;
    pid.imax = 100.0;
    pid.pwmMin = 0.0;
    pid.pwmMax = 1.0;
    heater->setPidParams(pid);
    heater->setSafetyLimits(0.0, 300.0);
    heater->setControlInterval(0.1); // 100ms control loop

    // 3. Set target temperature and run PID control
    const double targetTemp = 200.0;
    heater->setTarget(targetTemp);
    std::printf("\nSet target temperature: %.1f°C\n", targetTemp);

    // 4. Monitor temperature over time (simulated thermal model)
    // Simple thermal model: dT/dt = heatingPower * pwm - coolingRate * (T - ambient)
    const double ambient = 24.0;
    const double heatingPower = 8.0;   // °C/s at full PWM
    const double coolingRate = 0.02;   // 1/s cooling constant
    const double dt = 0.1;             // 100ms per iteration

    std::printf("\n%-8s %10s %10s %10s\n", "Time(s)", "Temp(°C)", "Target", "PWM");
    std::printf("%-8.1f %10.2f %10.1f %10.2f\n", 0.0, simTemp, targetTemp, 0.0);

    for (int i = 1; i <= 100; ++i) { // 10 seconds
        double pwm = heater->control();
        // Update simulated temperature
        simTemp += (heatingPower * pwm - coolingRate * (simTemp - ambient)) * dt;
        if (i % 10 == 0) {
            std::printf("%-8.1f %10.2f %10.1f %10.2f\n",
                        i * dt, simTemp, targetTemp, pwmOutput);
        }
    }

    std::printf("\nFinal: temp=%.2f°C target=%.1f°C atTarget=%d\n",
                simTemp, targetTemp, heater->atTarget(5.0));

    // 5. Simulate PID tuning (M303) — measure heating response
    std::printf("\n--- PID Autotune (M303) simulation ---\n");
    heater->reset();
    simTemp = ambient;

    // Measure heating rate at full power
    heater->setTarget(250.0);
    double startTemp = simTemp;
    for (int i = 0; i < 50; ++i) {
        heater->control();
        simTemp += (heatingPower * pwmOutput - coolingRate * (simTemp - ambient)) * dt;
    }
    double heatingRate = (simTemp - startTemp) / 5.0;
    std::printf("Heating rate: %.2f°C/s (from %.1f to %.1f in 5s)\n",
                heatingRate, startTemp, simTemp);

    // Estimate PID via Ziegler-Nichols-like heuristic
    double Pu = 20.0; // assumed oscillation period
    double Ku = heatingRate > 0 ? heatingRate * 5.0 : 10.0;
    double kp = 0.6 * Ku;
    double ki = 2.0 * kp / Pu;
    double kd = kp * Pu / 8.0;
    std::printf("Tuned PID: Kp=%.3f Ki=%.4f Kd=%.3f\n", kp, ki, kd);

    std::printf("\nDone\n");
    return 0;
}
