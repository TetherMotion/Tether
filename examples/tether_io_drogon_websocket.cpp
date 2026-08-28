/**
 * @file tether_io_drogon_websocket.cpp
 * @brief Minimal Drogon binary WebSocket endpoint for the Tether IO protocol.
 */
#include "TetherIOWebSocketController.hpp"
#include <drogon/drogon.h>
#include <cstring>
#include <chrono>
#include <cmath>
#include <atomic>

using namespace tether::io;
using tether::io::example::TetherIOWebSocketController;

int main() {
    Registry registry;
    double temperature = 21.5;
    std::atomic<double> amplitude{1.0};
    std::atomic<double> frequency{0.5};
    const auto started = std::chrono::steady_clock::now();
    const auto elapsed = [&started]() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    };
    ParamEntry parameter;
    parameter.id = 1;
    parameter.name = "temperature";
    parameter.description = "Example temperature parameter";
    parameter.group = "example";
    parameter.valueType = ValueType::F64;
    parameter.readFn = [&temperature](void* destination) {
        std::memcpy(destination, &temperature, sizeof(temperature));
    };
    parameter.writeFn = [&temperature](const void* source) {
        std::memcpy(&temperature, source, sizeof(temperature));
    };
    registry.addParam(std::move(parameter));

    registry.addParam({2, "wave_amplitude", "Sine and cosine amplitude", "demo", ValueType::F64,
        [&amplitude](void* dst) { const double value = amplitude.load(); std::memcpy(dst, &value, sizeof(value)); },
        [&amplitude](const void* src) { double value{}; std::memcpy(&value, src, sizeof(value)); amplitude.store(value); }});
    registry.addParam({3, "wave_frequency", "Wave frequency in Hz", "demo", ValueType::F64,
        [&frequency](void* dst) { const double value = frequency.load(); std::memcpy(dst, &value, sizeof(value)); },
        [&frequency](const void* src) { double value{}; std::memcpy(&value, src, sizeof(value)); frequency.store(value); }});

    const auto addSignal = [&registry, &elapsed, &amplitude, &frequency](uint64_t id, const char* name,
            const char* description, auto callback) {
        registry.addSignal({id, name, description, "demo", ValueType::F64,
            [callback, &elapsed, &amplitude, &frequency](void* dst) {
                const double value = callback(elapsed(), amplitude.load(), frequency.load());
                std::memcpy(dst, &value, sizeof(value));
            }});
    };
    addSignal(10, "sine_wave", "Deterministic sine wave", [](double t, double a, double f) { return a * std::sin(2.0 * M_PI * f * t); });
    addSignal(11, "cosine_wave", "Deterministic cosine wave", [](double t, double a, double f) { return a * std::cos(2.0 * M_PI * f * t); });
    addSignal(12, "ramp_seconds", "Monotonic elapsed-time ramp", [](double t, double, double) { return std::fmod(t, 10.0); });
    addSignal(13, "sample_counter", "Elapsed time in milliseconds", [](double t, double, double) { return std::floor(t * 1000.0); });
    registry.addSignal({14, "healthy", "Example health flag", "demo", ValueType::Bool,
        [&elapsed](void* dst) { const bool value = true; (void)elapsed; std::memcpy(dst, &value, sizeof(value)); }});

    FunctionEntry function;
    function.id = 20;
    function.name = "reset_demo";
    function.description = "Reset the demo temperature to its nominal value";
    function.group = "demo";
    function.callback = [&temperature](const std::vector<FunctionArgument>&) {
        temperature = 21.5;
        return FunctionCallResult{true, ErrorCode::None, {}, {}};
    };
    registry.addFunction(std::move(function));

    drogon::app().registerWebSocketController(
        "/tether-io", "tether::io::example::TetherIOWebSocketController", {});
    drogon::DrClassMap::setSingleInstance(
        std::make_shared<TetherIOWebSocketController>(registry));

    drogon::app().addListener("0.0.0.0", 8080);
    drogon::app().run();
}
