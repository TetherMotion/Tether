#pragma once
#include "SimulationTypes.hpp"
#include <random>

namespace Simulation {

/**Noise generator for sensor simulation */
class NoiseGenerator {
public:
    NoiseGenerator();
    explicit NoiseGenerator(uint64_t seed);

    /**
     * @brief Generate noise sample given configuration and current time.
     * @param params          Noise characteristics (type, amplitude, color).
     * @param time            Sample time in seconds.
     * @param actuatorOutput  Coupled actuator output for dependent noise.
     * @return noise value to add to a measurement.
     */
    double generate(const NoiseParams& params, double time, double actuatorOutput = 0.0);

    /**Reset internal state */
    void reset();
    void setSeed(uint64_t seed);

private:
    std::mt19937_64 rng_;
    double brownState_ = 0.0;
    double purpleState_ = 0.0;
    double prevWhite_ = 0.0;
};

/**Sensor model with noise, delay, quantization, saturation */
class SensorModel {
public:
    SensorModel();
    explicit SensorModel(const SensorConfig& config);

    /**
     * @brief Process a true measurement through sensor model.
     * @param trueValue       Ideal measurement before degradation.
     * @param time            Sample time in seconds.
     * @param actuatorOutput  Coupled actuator output fed to the noise model.
     * @return measured value after noise, delay, quantization, saturation.
     */
    double measure(double trueValue, double time, double actuatorOutput = 0.0);

    /**
     * @brief Configure sensor.
     * @param config  New sensor configuration; applies to the next measure().
     */
    void setConfig(const SensorConfig& config) { config_ = config; }
    const SensorConfig& config() const { return config_; }

    void reset();

private:
    SensorConfig config_;
    NoiseGenerator noise_;
    std::vector<std::pair<double, double>> delayBuffer_; // (time, value)
    std::mt19937_64 delayRng_;
    double lastSampleTime_ = -1e9;
    double lastSampledValue_ = 0.0;
};

/**Actuator model with nonlinearities */
class ActuatorModel {
public:
    ActuatorModel();
    explicit ActuatorModel(const ActuatorConfig& config);

    /**
     * @brief Process a control command through actuator model.
     * @param command  Commanded output.
     * @param dt       Step duration in seconds.
     * @return effective actuator output after nonlinearities/failure.
     */
    double apply(double command, double dt);

    /**
     * @brief Configure actuator.
     * @param config  New actuator configuration; applies to the next apply().
     */
    void setConfig(const ActuatorConfig& config) { config_ = config; }
    const ActuatorConfig& config() const { return config_; }

    /**
     * @brief Check if actuator has failed.
     * @return true while the injected failure is active.
     */
    bool hasFailed() const { return failed_; }

    void reset();

private:
    ActuatorConfig config_;
    double prevOutput_ = 0.0;
    double prevCommand_ = 0.0;
    double backlashState_ = 0.0;
    double hysteresisState_ = 0.0;
    bool failed_ = false;
};

} // namespace Simulation
