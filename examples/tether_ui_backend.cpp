#include "WebSocketTransport.hpp"

#include "tether/gcode/GCodeInterpreter.hpp"
#include "tether/gcode/GCodeParser.hpp"
#include "tether/gcode/GCodeTypes.hpp"
#include "tether/gcode/GCodeVariables.hpp"
#include "tether/io/ParameterExposer.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/Server.hpp"
#include "tether/slave/core/SlaveTypes.hpp"
#include "tether/slave/profiles/CiA402Slave.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;
using namespace tether::io;

std::atomic<bool> gStopRequested{false};

template <size_t N>
std::string arrayToString(const std::array<char, N>& value) {
    return std::string(value.data(), strnlen(value.data(), value.size()));
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

template <typename VarRead, typename VarWrite>
ParamEntry makeStringParam(
    uint64_t id,
    std::string name,
    std::string description,
    std::string group,
    VarRead readFn,
    VarWrite writeFn,
    uint16_t maxValueSize,
    std::map<std::string, std::string> metadata = {}) {
    ParamEntry entry{};
    entry.id = id;
    entry.name = std::move(name);
    entry.description = std::move(description);
    entry.group = std::move(group);
    entry.valueType = ValueType::String;
    entry.varReadFn = std::move(readFn);
    entry.varWriteFn = std::move(writeFn);
    entry.maxValueSize = maxValueSize;
    entry.metadata = std::move(metadata);
    return entry;
}

template <typename VarRead>
SignalEntry makeStringSignal(
    uint64_t id,
    std::string name,
    std::string description,
    std::string group,
    VarRead readFn,
    uint16_t maxValueSize,
    std::map<std::string, std::string> metadata = {}) {
    SignalEntry entry{};
    entry.id = id;
    entry.name = std::move(name);
    entry.description = std::move(description);
    entry.group = std::move(group);
    entry.valueType = ValueType::String;
    entry.varReadFn = std::move(readFn);
    entry.maxValueSize = maxValueSize;
    entry.metadata = std::move(metadata);
    return entry;
}

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

constexpr size_t kRxPDOControlwordOffset = 0;
constexpr size_t kRxPDOTargetPositionOffset = 2;
constexpr size_t kRxPDOTargetVelocityOffset = 6;
constexpr size_t kRxPDOTargetTorqueOffset = 10;
constexpr size_t kRxPDOModeOffset = 12;

class NativeGCodeProgram {
public:
    NativeGCodeProgram()
        : parser_(variables_) {}

    void setMode(GCode::InterpreterMode mode) {
        mode_ = mode;
    }

    void setDryRun(bool dryRun) {
        dryRun_ = dryRun;
    }

    bool isDryRun() const {
        return dryRun_;
    }

    bool isProgramLoaded() const {
        return !lines_.empty();
    }

    bool isFinished() const {
        return state_ == GCode::InterpreterState::FINISHED;
    }

    GCode::InterpreterState getState() const {
        return state_;
    }

    uint32_t getCurrentLine() const {
        return currentLine_;
    }

    uint32_t getTotalLines() const {
        return static_cast<uint32_t>(lines_.size());
    }

    GCode::Position getCurrentPosition() const {
        return position_;
    }

    GCode::Error loadString(const std::string& program) {
        lines_.clear();
        std::string current;
        for (char ch : program) {
            if (ch == '\r') {
                continue;
            }
            if (ch == '\n') {
                lines_.push_back(current);
                current.clear();
                continue;
            }
            current.push_back(ch);
        }
        if (!current.empty() || (!program.empty() && program.back() == '\n')) {
            lines_.push_back(current);
        }

        resetExecutionState();
        if (lines_.empty()) {
            state_ = GCode::InterpreterState::IDLE;
            return GCode::Error{};
        }

        for (size_t index = 0; index < lines_.size(); ++index) {
            GCode::Block block;
            auto err = parser_.parseLine(lines_[index].c_str(), block);
            if (!err.ok()) {
                state_ = GCode::InterpreterState::ERROR;
                return err;
            }
        }

        state_ = GCode::InterpreterState::READY;
        return GCode::Error{};
    }

    GCode::Error step() {
        if (state_ == GCode::InterpreterState::FINISHED) {
            GCode::Error err;
            err.code = GCode::ErrorCode::END;
            return err;
        }
        if (state_ == GCode::InterpreterState::ERROR || state_ == GCode::InterpreterState::STOPPED) {
            GCode::Error err;
            err.code = GCode::ErrorCode::INTERLOCK_ERROR;
            std::snprintf(err.message.data(), err.message.size(), "%s", "Program not runnable in current state");
            return err;
        }
        if (lines_.empty()) {
            state_ = GCode::InterpreterState::IDLE;
            return GCode::Error{};
        }

        state_ = GCode::InterpreterState::RUNNING;

        while (nextLineIndex_ < lines_.size()) {
            GCode::Block block;
            auto err = parser_.parseLine(lines_[nextLineIndex_].c_str(), block);
            currentLine_ = static_cast<uint32_t>(nextLineIndex_ + 1);
            ++nextLineIndex_;
            if (!err.ok()) {
                state_ = GCode::InterpreterState::ERROR;
                return err;
            }

            applyModalState(block);
            applyMotion(block);

            if (block.hasMCode(0) || block.hasMCode(1)) {
                state_ = GCode::InterpreterState::PAUSED;
                return GCode::Error{};
            }
            if (block.hasMCode(2) || block.hasMCode(30)) {
                state_ = GCode::InterpreterState::FINISHED;
                return GCode::Error{};
            }

            if (mode_ == GCode::InterpreterMode::STEP) {
                state_ = nextLineIndex_ >= lines_.size() ? GCode::InterpreterState::FINISHED : GCode::InterpreterState::READY;
                return GCode::Error{};
            }
        }

        state_ = GCode::InterpreterState::FINISHED;
        return GCode::Error{};
    }

    void pause() {
        if (state_ == GCode::InterpreterState::RUNNING) {
            state_ = GCode::InterpreterState::PAUSED;
        }
    }

    void stop() {
        state_ = GCode::InterpreterState::STOPPED;
    }

    void reset() {
        resetExecutionState();
        state_ = lines_.empty() ? GCode::InterpreterState::IDLE : GCode::InterpreterState::READY;
    }

private:
    void resetExecutionState() {
        nextLineIndex_ = 0;
        currentLine_ = 0;
        position_ = GCode::Position{};
        absoluteMode_ = true;
        state_ = GCode::InterpreterState::IDLE;
    }

    void applyModalState(const GCode::Block& block) {
        if (block.hasGCode(900)) {
            absoluteMode_ = true;
        }
        if (block.hasGCode(910)) {
            absoluteMode_ = false;
        }
    }

    void applyMotion(const GCode::Block& block) {
        if (!block.hasMotion()) {
            return;
        }

        updateAxis(block, GCode::WordLetter::X, position_.x());
        updateAxis(block, GCode::WordLetter::Y, position_.y());
        updateAxis(block, GCode::WordLetter::Z, position_.z());
    }

    void updateAxis(const GCode::Block& block, GCode::WordLetter axis, double& currentValue) {
        if (!block.hasWord(axis)) {
            return;
        }
        const double value = block.getWord(axis);
        currentValue = absoluteMode_ ? value : currentValue + value;
    }

    GCode::VariableSystem variables_;
    GCode::Parser parser_;
    std::vector<std::string> lines_;
    GCode::Position position_{};
    GCode::InterpreterMode mode_ = GCode::InterpreterMode::STEP;
    GCode::InterpreterState state_ = GCode::InterpreterState::IDLE;
    size_t nextLineIndex_ = 0;
    uint32_t currentLine_ = 0;
    bool absoluteMode_ = true;
    bool dryRun_ = true;
};

class TetherUiBackend {
public:
    TetherUiBackend()
        : drive_(EtherCAT::slave::createServoDrive(131072)) {
        configureInterpreter();
        initializeDrive();
    }

    void expose(Registry& registry) {
        using ValueType = tether::io::ValueType;

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::EtherCATMaster, 0x0001),
            "discovered_slave_count",
            "Number of simulated EtherCAT slaves",
            "ethercat.master",
            ValueType::U16,
            [this](void* dest) {
                const uint16_t value = 1;
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::EtherCATMaster, 0x0002),
            "is_running",
            "Whether the simulated EtherCAT bus is active",
            "ethercat.master",
            ValueType::Bool,
            [this](void* dest) {
                const uint8_t value = isRunning() ? 1 : 0;
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::EtherCATMaster, 0x1001),
            "slave0.ec_state",
            "Current EtherCAT AL state of the simulated servo slave",
            "ethercat.slave0",
            ValueType::U8,
            [this](void* dest) {
                const auto state = static_cast<uint8_t>(drive_->getCore().getState());
                std::memcpy(dest, &state, sizeof(state));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::CiA402Drive, 0x0001),
            "drive0.drive_state",
            "Current CiA402 drive state",
            "cia402.drive0",
            ValueType::U8,
            [this](void* dest) {
                const auto value = static_cast<uint8_t>(drive_->getDriveState());
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::CiA402Drive, 0x0002),
            "drive0.actual_position",
            "Actual simulated servo position",
            "cia402.drive0",
            ValueType::I32,
            [this](void* dest) {
                const auto value = drive_->getActualPosition();
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::CiA402Drive, 0x0003),
            "drive0.actual_velocity",
            "Actual simulated servo velocity",
            "cia402.drive0",
            ValueType::I32,
            [this](void* dest) {
                const auto value = drive_->getActualVelocity();
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::CiA402Drive, 0x0004),
            "drive0.actual_torque",
            "Actual simulated servo torque",
            "cia402.drive0",
            ValueType::I16,
            [this](void* dest) {
                const auto value = drive_->getActualTorque();
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::CiA402Drive, 0x0005),
            "drive0.target_position",
            "Current commanded target position",
            "cia402.drive0",
            ValueType::I32,
            [this](void* dest) {
                const auto value = targetPositionCounts_;
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::CiA402Drive, 0x0081),
            "drive0.target_position_cmd",
            "Write target position for the simulated servo",
            "cia402.drive0",
            ValueType::I32,
            [this](void* dest) {
                const auto value = targetPositionCounts_;
                std::memcpy(dest, &value, sizeof(value));
            },
            [this](const void* src) {
                int32_t value = 0;
                std::memcpy(&value, src, sizeof(value));
                std::lock_guard<std::mutex> lock(mutex_);
                targetPositionCounts_ = value;
                writeDriveCommandLocked();
            }
        ));

        registry.addParam(makeStringParam(
            makeId(ModuleId::GCodeInterpreter, 0x0001),
            "program_text",
            "Loaded G-code program text",
            "gcode",
            [this](void* dest, size_t maxLen) -> size_t {
                std::lock_guard<std::mutex> lock(mutex_);
                const size_t len = std::min(maxLen, programText_.size());
                std::memcpy(dest, programText_.data(), len);
                return len;
            },
            [this](const void* src, size_t len) {
                std::lock_guard<std::mutex> lock(mutex_);
                programText_.assign(static_cast<const char*>(src), len);
                const auto err = interpreter_.loadString(programText_);
                if (!err.ok()) {
                    lastProgramError_ = arrayToString(err.message);
                } else {
                    lastProgramError_.clear();
                }
                gcodeRunning_ = false;
            },
            65535,
            {{"unit", "gcode"}}
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::GCodeInterpreter, 0x0002),
            "command",
            "G-code transport command (0=idle,1=run,2=pause,3=stop,4=reset)",
            "gcode",
            ValueType::U8,
            [this](void* dest) {
                std::memcpy(dest, &lastCommand_, sizeof(lastCommand_));
            },
            [this](const void* src) {
                uint8_t command = 0;
                std::memcpy(&command, src, sizeof(command));
                std::lock_guard<std::mutex> lock(mutex_);
                lastCommand_ = command;
                switch (command) {
                    case 1:
                        gcodeRunning_ = true;
                        break;
                    case 2:
                        gcodeRunning_ = false;
                        interpreter_.pause();
                        break;
                    case 3:
                        gcodeRunning_ = false;
                        interpreter_.stop();
                        break;
                    case 4:
                        gcodeRunning_ = false;
                        interpreter_.reset();
                        break;
                    default:
                        break;
                }
            }
        ));

        registry.addParam(makeFixedParam(
            makeId(ModuleId::GCodeInterpreter, 0x0003),
            "dry_run",
            "Enable dry-run mode in the interpreter",
            "gcode",
            ValueType::Bool,
            [this](void* dest) {
                const uint8_t value = interpreter_.isDryRun() ? 1 : 0;
                std::memcpy(dest, &value, sizeof(value));
            },
            [this](const void* src) {
                uint8_t value = 0;
                std::memcpy(&value, src, sizeof(value));
                std::lock_guard<std::mutex> lock(mutex_);
                interpreter_.setDryRun(value != 0);
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::GCodeInterpreter, 0x0081),
            "state",
            "Current interpreter state",
            "gcode",
            ValueType::U8,
            [this](void* dest) {
                const uint8_t value = static_cast<uint8_t>(interpreter_.getState());
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::GCodeInterpreter, 0x0082),
            "current_line",
            "Current line number",
            "gcode",
            ValueType::U32,
            [this](void* dest) {
                const uint32_t value = interpreter_.getCurrentLine();
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::GCodeInterpreter, 0x0083),
            "total_lines",
            "Loaded line count",
            "gcode",
            ValueType::U32,
            [this](void* dest) {
                const uint32_t value = interpreter_.getTotalLines();
                std::memcpy(dest, &value, sizeof(value));
            }
        ));

        addPositionSignal(registry, 0x0090, "position.x", [this]() -> double { return interpreter_.getCurrentPosition().x(); });
        addPositionSignal(registry, 0x0091, "position.y", [this]() -> double { return interpreter_.getCurrentPosition().y(); });
        addPositionSignal(registry, 0x0092, "position.z", [this]() -> double { return interpreter_.getCurrentPosition().z(); });

        registry.addSignal(makeStringSignal(
            makeId(ModuleId::GCodeInterpreter, 0x00A0),
            "last_error_message",
            "Last G-code load/execute error",
            "gcode",
            [this](void* dest, size_t maxLen) -> size_t {
                std::lock_guard<std::mutex> lock(mutex_);
                const size_t len = std::min(maxLen, lastProgramError_.size());
                std::memcpy(dest, lastProgramError_.data(), len);
                return len;
            },
            1024
        ));
    }

    void start() {
        running_.store(true, std::memory_order_relaxed);
        worker_ = std::thread([this]() { runLoop(); });
    }

    void stop() {
        running_.store(false, std::memory_order_relaxed);
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    bool isRunning() const {
        return running_.load(std::memory_order_relaxed);
    }

private:
    template <typename Fn>
    void addPositionSignal(Registry& registry, uint32_t localId, const std::string& name, Fn fn) {
        registry.addSignal(makeFixedSignal(
            makeId(ModuleId::GCodeInterpreter, localId),
            name,
            "Current G-code position component",
            "gcode.position",
            ValueType::F64,
            [fn](void* dest) {
                const double value = fn();
                std::memcpy(dest, &value, sizeof(value));
            }
        ));
    }

    void configureInterpreter() {
        interpreter_.setMode(GCode::InterpreterMode::STEP);
        interpreter_.setDryRun(true);
        programText_ = "G0 X0 Y0 Z0\nG1 X50 Y0 F1000\nG1 X50 Y50\nG1 X0 Y50\nG1 X0 Y0\nM30\n";
        const auto err = interpreter_.loadString(programText_);
        if (!err.ok()) {
            lastProgramError_ = arrayToString(err.message);
        }
    }

    void initializeDrive() {
        auto& core = drive_->getCore();
        core.requestStateChange(EtherCAT::slave::ALControl{EtherCAT::slave::SlaveState::PRE_OP, false, false});
        core.requestStateChange(EtherCAT::slave::ALControl{EtherCAT::slave::SlaveState::SAFE_OP, false, false});
        writeDriveCommandLocked();
        drive_->processRxPDO();
        core.requestStateChange(EtherCAT::slave::ALControl{EtherCAT::slave::SlaveState::OP, false, false});
        drive_->processControlWord(0x0006);
        drive_->processControlWord(0x0007);
        drive_->processControlWord(0x000F);
    }

    void writeDriveCommandLocked() {
        uint8_t* rx = drive_->getCore().getRxPDOData();
        if (!rx) {
            return;
        }
        const uint16_t controlword = 0x000F;
        const int32_t targetPosition = targetPositionCounts_;
        const int32_t targetVelocity = 0;
        const int16_t targetTorque = 0;
        const int8_t mode = 8;
        std::memcpy(rx + kRxPDOControlwordOffset, &controlword, sizeof(controlword));
        std::memcpy(rx + kRxPDOTargetPositionOffset, &targetPosition, sizeof(targetPosition));
        std::memcpy(rx + kRxPDOTargetVelocityOffset, &targetVelocity, sizeof(targetVelocity));
        std::memcpy(rx + kRxPDOTargetTorqueOffset, &targetTorque, sizeof(targetTorque));
        std::memcpy(rx + kRxPDOModeOffset, &mode, sizeof(mode));
        drive_->processRxPDO();
    }

    void runLoop() {
        auto last = Clock::now();
        while (running_.load(std::memory_order_relaxed)) {
            const auto now = Clock::now();
            const auto delta = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last);
            last = now;

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (gcodeRunning_ && interpreter_.isProgramLoaded() && !interpreter_.isFinished()) {
                    const auto err = interpreter_.step();
                    if (!err.ok()) {
                        lastProgramError_ = arrayToString(err.message);
                        gcodeRunning_ = false;
                    }
                }

                const auto pos = interpreter_.getCurrentPosition();
                targetPositionCounts_ = static_cast<int32_t>(pos.x() * 1000.0);
                writeDriveCommandLocked();
                drive_->simulate(static_cast<uint64_t>(delta.count()));
                drive_->updateTxPDO();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    mutable std::mutex mutex_;
    std::unique_ptr<EtherCAT::slave::CiA402Slave> drive_;
    NativeGCodeProgram interpreter_;
    std::string programText_;
    std::string lastProgramError_;
    int32_t targetPositionCounts_ = 0;
    uint8_t lastCommand_ = 0;
    bool gcodeRunning_ = false;
    std::atomic<bool> running_{false};
    std::thread worker_;
};

int runBackend(uint16_t port) {
    Registry registry;
    TetherUiBackend demo;
    demo.expose(registry);
    demo.start();

    ServerConfig config;
    config.maxClients = 8;
    config.timestampFn = nowUs;
    config.logFn = logFn;

    Server server(
        registry,
        std::make_unique<tether::examples::WebSocketTransportServer>(port, "/"),
        config);

    if (!server.start()) {
        std::cerr << "Failed to start tether_ui backend on port " << port << '\n';
        demo.stop();
        return 1;
    }

    std::cout << "tether_ui backend listening on ws://0.0.0.0:" << port << "/\n";
    while (!gStopRequested.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    server.stop();
    demo.stop();
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    uint16_t port = 4001;
    if (argc >= 3 && std::string(argv[1]) == "--port") {
        port = static_cast<uint16_t>(std::stoi(argv[2]));
    }
    return runBackend(port);
}