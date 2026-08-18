#include "WebSocketTransport.hpp"

#include "tether/control/ControllerBase.hpp"
#include "tether/control/PIDControllers.hpp"
#include "tether/io/ParameterExposer.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/simulation/AllSystems.hpp"
#include "tether/simulation/SimulationEngine.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using namespace tether::io;

std::atomic<bool> gStopRequested{false};

uint64_t nowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        Clock::now().time_since_epoch()).count());
}

void logFn(const char* tag, const char* fmt, ...) {
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    std::cerr << "[" << tag << "] " << buffer << '\n';
}

void onSignal(int) {
    gStopRequested.store(true, std::memory_order_relaxed);
}

template <typename ReadFn>
SignalEntry makeFixedSignal(
    uint64_t id,
    std::string name,
    std::string description,
    std::string group,
    ValueType valueType,
    ReadFn readFn,
    std::map<std::string, std::string> metadata = {}) {
    SignalEntry entry{};
    entry.id = id;
    entry.name = std::move(name);
    entry.description = std::move(description);
    entry.group = std::move(group);
    entry.valueType = valueType;
    entry.readFn = std::move(readFn);
    entry.metadata = std::move(metadata);
    return entry;
}

template <typename ReadFn, typename WriteFn>
ParamEntry makeFixedParam(
    uint64_t id,
    std::string name,
    std::string description,
    std::string group,
    ValueType valueType,
    ReadFn readFn,
    WriteFn writeFn,
    std::map<std::string, std::string> metadata = {}) {
    ParamEntry entry{};
    entry.id = id;
    entry.name = std::move(name);
    entry.description = std::move(description);
    entry.group = std::move(group);
    entry.valueType = valueType;
    entry.readFn = std::move(readFn);
    entry.writeFn = std::move(writeFn);
    entry.metadata = std::move(metadata);
    return entry;
}

std::shared_ptr<Simulation::DynamicalSystem> makeSharedSystem(int systemId) {
    auto system = Simulation::createSystem(systemId);
    return std::shared_ptr<Simulation::DynamicalSystem>(system.release());
}

class ScalarPIDController final : public Simulation::SimController {
public:
    explicit ScalarPIDController(size_t outputIndex)
        : outputIndex_(outputIndex) {
        tether::control::SaturationLimits limits;
        limits.outputMin = -1.0e6;
        limits.outputMax = 1.0e6;
        pid_.setSaturationLimits(limits);
        pid_.setDerivativeFilter(0.01);
    }

    Simulation::StateVector compute(
        double,
        const Simulation::StateVector& measured,
        const Simulation::StateVector& reference,
        double dt) override {
        const double measuredValue = outputIndex_ < measured.size() ? measured[outputIndex_] : 0.0;
        const double referenceValue = outputIndex_ < reference.size() ? reference[outputIndex_] : 0.0;
        tether::control::ControllerInput input;
        input.reference = referenceValue;
        input.measured = measuredValue;
        input.dt = dt;
        const auto output = pid_.compute(input);
        return {output.control};
    }

    void reset() override {
        pid_.reset();
    }

    const char* name() const override {
        return "Scalar PID";
    }

    void setGains(double kp, double ki, double kd) {
        pid_.setGains(kp, ki, kd);
    }

    double kp() const { return pid_.getKp(); }
    double ki() const { return pid_.getKi(); }
    double kd() const { return pid_.getKd(); }

private:
    size_t outputIndex_;
    tether::control::PIDController pid_;
};

struct SystemSpec {
    std::string key;
    int systemId;
    size_t controlledOutputIndex;
    double defaultSetpoint;
    double defaultKp;
    double defaultKi;
    double defaultKd;
};

class SystemRunner {
public:
    explicit SystemRunner(SystemSpec spec)
        : spec_(std::move(spec))
        , system_(makeSharedSystem(spec_.systemId))
        , controller_(std::make_shared<ScalarPIDController>(spec_.controlledOutputIndex)) {
        controller_->setGains(spec_.defaultKp, spec_.defaultKi, spec_.defaultKd);
        setupEngineLocked();
        refreshCachesLocked();
    }

    const std::string& key() const { return spec_.key; }
    const char* systemName() const { return system_->name(); }
    const char* systemDescription() const { return system_->description(); }

    std::vector<std::string> stateNames() const { return system_->stateNames(); }
    std::vector<std::string> outputNames() const { return system_->outputNames(); }
    std::vector<std::string> inputNames() const { return system_->inputNames(); }
    std::vector<Simulation::ParamDescriptor> parameterDescriptors() const { return system_->parameterDescriptorsDetailed(); }

    void tick() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) {
            return;
        }
        stepLocked();
    }

    bool running() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return running_;
    }

    double time() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastTime_;
    }

    double stateAt(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return index < lastState_.size() ? lastState_[index] : 0.0;
    }

    double outputAt(size_t index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return index < lastOutput_.size() ? lastOutput_[index] : 0.0;
    }

    double controlSignal() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastControl_;
    }

    double errorSignal() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastError_;
    }

    double setpoint() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return setpoint_;
    }

    void setSetpoint(double value) {
        std::lock_guard<std::mutex> lock(mutex_);
        setpoint_ = value;
        updateReferenceLocked();
    }

    double stepDelta() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stepDelta_;
    }

    void setStepDelta(double value) {
        std::lock_guard<std::mutex> lock(mutex_);
        stepDelta_ = value;
    }

    double speed() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return speed_;
    }

    void setSpeed(double value) {
        std::lock_guard<std::mutex> lock(mutex_);
        speed_ = std::max(0.05, value);
        config_.dt = baseDt_ * speed_;
        engine_.setConfig(config_);
        updateReferenceLocked();
    }

    uint8_t lastCommand() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastCommand_;
    }

    void command(uint8_t value) {
        std::lock_guard<std::mutex> lock(mutex_);
        lastCommand_ = value;
        switch (value) {
            case 1:
                running_ = true;
                break;
            case 2:
                running_ = false;
                break;
            case 3:
                running_ = false;
                resetLocked();
                break;
            case 4:
                stepLocked();
                break;
            case 5:
                setpoint_ += stepDelta_;
                updateReferenceLocked();
                break;
            default:
                break;
        }
    }

    double parameter(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return system_->getParameter(name);
    }

    void setParameter(const std::string& name, double value) {
        std::lock_guard<std::mutex> lock(mutex_);
        system_->setParameter(name, value);
        refreshCachesLocked();
    }

    double kp() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return controller_->kp();
    }

    double ki() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return controller_->ki();
    }

    double kd() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return controller_->kd();
    }

    void setKp(double value) {
        std::lock_guard<std::mutex> lock(mutex_);
        controller_->setGains(value, controller_->ki(), controller_->kd());
    }

    void setKi(double value) {
        std::lock_guard<std::mutex> lock(mutex_);
        controller_->setGains(controller_->kp(), value, controller_->kd());
    }

    void setKd(double value) {
        std::lock_guard<std::mutex> lock(mutex_);
        controller_->setGains(controller_->kp(), controller_->ki(), value);
    }

private:
    void setupEngineLocked() {
        config_.dt = baseDt_;
        config_.totalTime = 1.0e12;
        config_.adaptiveStep = false;
        engine_.setSystem(system_);
        engine_.setController(controller_);
        engine_.setConfig(config_);
        setpoint_ = spec_.defaultSetpoint;
        updateReferenceLocked();
        resetLocked();
    }

    void updateReferenceLocked() {
        Simulation::StateVector reference(system_->outputDim(), 0.0);
        if (spec_.controlledOutputIndex < reference.size()) {
            reference[spec_.controlledOutputIndex] = setpoint_;
        }
        engine_.setReference(reference);
    }

    void resetLocked() {
        controller_->reset();
        engine_.reset();
        engine_.setSystem(system_);
        engine_.setController(controller_);
        engine_.setConfig(config_);
        engine_.setInitialState(system_->defaultInitialStateForUi());
        updateReferenceLocked();
        engine_.initialize();
        refreshCachesLocked();
    }

    void refreshCachesLocked() {
        lastTime_ = engine_.currentTime();
        lastState_ = engine_.currentState();
        lastOutput_ = system_->output(lastTime_, lastState_, system_->defaultInput());
    }

    void stepLocked() {
        const auto result = engine_.step();
        lastTime_ = result.time;
        lastState_ = result.state;
        lastOutput_ = result.output;
        lastControl_ = result.controlSignal;
        lastError_ = result.error;
    }

    SystemSpec spec_;
    std::shared_ptr<Simulation::DynamicalSystem> system_;
    std::shared_ptr<ScalarPIDController> controller_;
    mutable std::mutex mutex_;
    Simulation::SimulationEngine engine_;
    Simulation::SimConfig config_{};
    Simulation::StateVector lastState_;
    Simulation::StateVector lastOutput_;
    double lastTime_ = 0.0;
    double lastControl_ = 0.0;
    double lastError_ = 0.0;
    double setpoint_ = 0.0;
    double stepDelta_ = 1.0;
    double speed_ = 1.0;
    const double baseDt_ = 0.002;
    bool running_ = false;
    uint8_t lastCommand_ = 0;
};

class ControlMeBackend {
public:
    ControlMeBackend() {
        runners_.emplace_back(std::make_unique<SystemRunner>(SystemSpec{"pendulum", 3, 1, 0.0, 120.0, 4.0, 18.0}));
        runners_.emplace_back(std::make_unique<SystemRunner>(SystemSpec{"servo", 21, 0, 1.0, 12.0, 0.5, 0.2}));
        runners_.emplace_back(std::make_unique<SystemRunner>(SystemSpec{"thermal", 33, 0, 50.0, 10.0, 0.1, 0.0}));
    }

    void expose(Registry& registry) {
        uint32_t nextLocalId = 1;
        for (const auto& runner : runners_) {
            exposeRunner(registry, *runner, nextLocalId);
            nextLocalId += 0x100;
        }
    }

    void start() {
        running_.store(true, std::memory_order_relaxed);
        worker_ = std::thread([this]() {
            while (running_.load(std::memory_order_relaxed)) {
                for (const auto& runner : runners_) {
                    runner->tick();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        });
    }

    void stop() {
        running_.store(false, std::memory_order_relaxed);
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    static std::map<std::string, std::string> paramMetadata(const Simulation::ParamDescriptor& descriptor) {
        return {
            {"unit", descriptor.unit},
            {"label", descriptor.description},
            {"min", std::to_string(descriptor.minValue)},
            {"max", std::to_string(descriptor.maxValue)},
            {"step", std::to_string(descriptor.step)}
        };
    }

    void exposeRunner(Registry& registry, SystemRunner& runner, uint32_t idBase) {
        const auto prefix = std::string("controlme.") + runner.key();
        const auto stateNames = runner.stateNames();
        const auto outputNames = runner.outputNames();

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::Simulation, idBase + 0x01),
            prefix + ".running",
            std::string(runner.systemDescription()) + " running flag",
            prefix,
            ValueType::Bool,
            [&runner](void* dest) {
                const uint8_t value = runner.running() ? 1 : 0;
                std::memcpy(dest, &value, sizeof(value));
            },
            {{"label", "Running"}}
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::Simulation, idBase + 0x02),
            prefix + ".time",
            "Simulation time",
            prefix,
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.time();
                std::memcpy(dest, &value, sizeof(value));
            },
            {{"unit", "s"}, {"label", "Simulation time"}}
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::Simulation, idBase + 0x03),
            prefix + ".control",
            "Current controller output",
            prefix,
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.controlSignal();
                std::memcpy(dest, &value, sizeof(value));
            },
            {{"label", "Control signal"}}
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::Simulation, idBase + 0x04),
            prefix + ".error",
            "Current control error magnitude",
            prefix,
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.errorSignal();
                std::memcpy(dest, &value, sizeof(value));
            },
            {{"label", "Error"}}
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::Simulation, idBase + 0x10),
            prefix + ".command",
            "Simulation command (1=run,2=pause,3=reset,4=step,5=apply step)",
            prefix,
            ValueType::U8,
            [&runner](void* dest) {
                const uint8_t value = runner.lastCommand();
                std::memcpy(dest, &value, sizeof(value));
            },
            [&runner](const void* src) {
                uint8_t value = 0;
                std::memcpy(&value, src, sizeof(value));
                runner.command(value);
            },
            {{"label", "Command"}}
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::Simulation, idBase + 0x11),
            prefix + ".setpoint",
            "Controller setpoint",
            prefix,
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.setpoint();
                std::memcpy(dest, &value, sizeof(value));
            },
            [&runner](const void* src) {
                double value = 0.0;
                std::memcpy(&value, src, sizeof(value));
                runner.setSetpoint(value);
            },
            {{"label", "Setpoint"}}
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::Simulation, idBase + 0x12),
            prefix + ".step_delta",
            "Step-response setpoint increment",
            prefix,
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.stepDelta();
                std::memcpy(dest, &value, sizeof(value));
            },
            [&runner](const void* src) {
                double value = 0.0;
                std::memcpy(&value, src, sizeof(value));
                runner.setStepDelta(value);
            },
            {{"label", "Step amplitude"}}
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::Simulation, idBase + 0x13),
            prefix + ".speed",
            "Simulation speed multiplier",
            prefix,
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.speed();
                std::memcpy(dest, &value, sizeof(value));
            },
            [&runner](const void* src) {
                double value = 0.0;
                std::memcpy(&value, src, sizeof(value));
                runner.setSpeed(value);
            },
            {{"label", "Speed"}, {"min", "0.1"}, {"max", "10"}, {"step", "0.1"}}
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::PIDController, idBase + 0x20),
            prefix + ".controller.kp",
            "Proportional gain",
            prefix + ".controller",
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.kp();
                std::memcpy(dest, &value, sizeof(value));
            },
            [&runner](const void* src) {
                double value = 0.0;
                std::memcpy(&value, src, sizeof(value));
                runner.setKp(value);
            },
            {{"label", "Kp"}}
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::PIDController, idBase + 0x21),
            prefix + ".controller.ki",
            "Integral gain",
            prefix + ".controller",
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.ki();
                std::memcpy(dest, &value, sizeof(value));
            },
            [&runner](const void* src) {
                double value = 0.0;
                std::memcpy(&value, src, sizeof(value));
                runner.setKi(value);
            },
            {{"label", "Ki"}}
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::PIDController, idBase + 0x22),
            prefix + ".controller.kd",
            "Derivative gain",
            prefix + ".controller",
            ValueType::F64,
            [&runner](void* dest) {
                const double value = runner.kd();
                std::memcpy(dest, &value, sizeof(value));
            },
            [&runner](const void* src) {
                double value = 0.0;
                std::memcpy(&value, src, sizeof(value));
                runner.setKd(value);
            },
            {{"label", "Kd"}}
        ));

        const auto descriptors = runner.parameterDescriptors();
        for (size_t index = 0; index < descriptors.size(); ++index) {
            const auto descriptor = descriptors[index];
            registry.addParam(makeFixedParam(
                makeId(ModuleId::Simulation, idBase + 0x40 + static_cast<uint32_t>(index)),
                prefix + ".param." + descriptor.name,
                descriptor.description,
                prefix + ".param",
                ValueType::F64,
                [&runner, name = descriptor.name](void* dest) {
                    const double value = runner.parameter(name);
                    std::memcpy(dest, &value, sizeof(value));
                },
                [&runner, name = descriptor.name](const void* src) {
                    double value = 0.0;
                    std::memcpy(&value, src, sizeof(value));
                    runner.setParameter(name, value);
                },
                paramMetadata(descriptor)
            ));
        }

        for (size_t index = 0; index < stateNames.size(); ++index) {
            registry.addSignal(makeFixedSignal(
                makeId(ModuleId::Simulation, idBase + 0x80 + static_cast<uint32_t>(index)),
                prefix + ".state." + std::to_string(index),
                "System state",
                prefix + ".state",
                ValueType::F64,
                [&runner, index](void* dest) {
                    const double value = runner.stateAt(index);
                    std::memcpy(dest, &value, sizeof(value));
                },
                {{"label", stateNames[index]}, {"index", std::to_string(index)}}
            ));
        }

        for (size_t index = 0; index < outputNames.size(); ++index) {
            registry.addSignal(makeFixedSignal(
                makeId(ModuleId::Simulation, idBase + 0xA0 + static_cast<uint32_t>(index)),
                prefix + ".output." + std::to_string(index),
                "System output",
                prefix + ".output",
                ValueType::F64,
                [&runner, index](void* dest) {
                    const double value = runner.outputAt(index);
                    std::memcpy(dest, &value, sizeof(value));
                },
                {{"label", outputNames[index]}, {"index", std::to_string(index)}}
            ));
        }
    }

    std::vector<std::unique_ptr<SystemRunner>> runners_;
    std::atomic<bool> running_{false};
    std::thread worker_;
};

int runBackend(uint16_t port) {
    Registry registry;
    ControlMeBackend backend;
    backend.expose(registry);
    backend.start();

    ServerConfig config;
    config.maxClients = 8;
    config.timestampFn = nowUs;
    config.logFn = logFn;

    Server server(
        registry,
        std::make_unique<tether::examples::WebSocketTransportServer>(port, "/"),
        config);

    if (!server.start()) {
        std::cerr << "Failed to start controlme_ui backend on port " << port << '\n';
        backend.stop();
        return 1;
    }

    std::cout << "controlme_ui backend listening on ws://0.0.0.0:" << port << "/\n";
    while (!gStopRequested.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    server.stop();
    backend.stop();
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    uint16_t port = 4002;
    if (argc >= 3 && std::string(argv[1]) == "--port") {
        port = static_cast<uint16_t>(std::stoi(argv[2]));
    }
    return runBackend(port);
}