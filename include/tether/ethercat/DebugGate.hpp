/**
 * @file DebugGate.hpp
 * @brief Conditional debug gating framework — start/stop conditions for debug output.
 */

#pragma once

#include "tether/ethercat/TetherConfig.hpp"

#if TETHER_DEBUG_GATE_ENABLED

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include "tether/ethercat/Types.hpp"

namespace EtherCAT {

// ============================================================================
// Condition types
// ============================================================================

enum class DebugConditionType {
    State,              ///< Triggered by EtherCAT state transitions
    Checkpoint,         ///< Triggered by named program-flow checkpoints
    RegisterIntercept,  ///< Triggered by intercepting register reads
    CoEIntercept,       ///< Triggered by intercepting CoE/SDO reads
    Custom,             ///< User-defined callback
};

enum class CompareOp {
    Eq,      ///< ==
    Ne,      ///< !=
    Bitmask, ///< (read & value) == value
    Ge,      ///< >=
    Le,      ///< <=
    Gt,      ///< >
    Lt,      ///< <
};

// ============================================================================
// Abstract condition base
// ============================================================================

class DebugCondition {
public:
    virtual ~DebugCondition() = default;
    virtual DebugConditionType type() const = 0;
    virtual bool hasFired() const = 0;
    virtual void reset() = 0;

    // Intercept hooks — called by Master, default no-op
    virtual void onRegisterRead(uint16_t slave_index, uint16_t addr,
                                const uint8_t* data, uint16_t len) {
        (void)slave_index; (void)addr; (void)data; (void)len;
    }
    virtual void onCoERead(uint16_t slave_index, uint16_t index, uint8_t sub,
                           const uint8_t* data, size_t len) {
        (void)slave_index; (void)index; (void)sub; (void)data; (void)len;
    }
    virtual void onCheckpoint(const std::string& name, uint16_t slave_index) {
        (void)name; (void)slave_index;
    }

    virtual const char* description() const = 0;
};

// ============================================================================
// Concrete condition types
// ============================================================================

class StateCondition : public DebugCondition {
public:
    explicit StateCondition(SlaveState state, uint16_t slave_index = 0xFFFF)
        : state_(state), slave_index_(slave_index), fired_(false) {}

    DebugConditionType type() const override { return DebugConditionType::State; }
    bool hasFired() const override { return fired_; }
    void reset() override { fired_ = false; }

    void onCheckpoint(const std::string& name, uint16_t slave_index) override;

    const char* description() const override { return desc_.c_str(); }

    SlaveState state() const { return state_; }
    uint16_t slaveIndex() const { return slave_index_; }

private:
    SlaveState state_;
    uint16_t slave_index_;
    bool fired_;
    std::string desc_;
};

class CheckpointCondition : public DebugCondition {
public:
    explicit CheckpointCondition(const std::string& checkpoint_name,
                                 uint16_t slave_index = 0xFFFF)
        : checkpoint_name_(checkpoint_name), slave_index_(slave_index), fired_(false) {}

    DebugConditionType type() const override { return DebugConditionType::Checkpoint; }
    bool hasFired() const override { return fired_; }
    void reset() override { fired_ = false; }

    void onCheckpoint(const std::string& name, uint16_t slave_index) override;

    const char* description() const override { return desc_.c_str(); }

    const std::string& checkpointName() const { return checkpoint_name_; }

private:
    std::string checkpoint_name_;
    uint16_t slave_index_;
    bool fired_;
    std::string desc_;
};

class RegisterInterceptCondition : public DebugCondition {
public:
    RegisterInterceptCondition(uint16_t reg_addr, CompareOp op, uint64_t value,
                               uint16_t slave_index = 0xFFFF)
        : reg_addr_(reg_addr), op_(op), value_(value),
          slave_index_(slave_index), fired_(false) {}

    DebugConditionType type() const override { return DebugConditionType::RegisterIntercept; }
    bool hasFired() const override { return fired_; }
    void reset() override { fired_ = false; }

    void onRegisterRead(uint16_t slave_index, uint16_t addr,
                        const uint8_t* data, uint16_t len) override;

    const char* description() const override { return desc_; }

private:
    uint16_t reg_addr_;
    CompareOp op_;
    uint64_t value_;
    uint16_t slave_index_;
    bool fired_;
    const char* desc_ = "";
};

class CoEInterceptCondition : public DebugCondition {
public:
    CoEInterceptCondition(uint16_t obj_index, uint8_t sub_index, bool has_sub,
                          CompareOp op, uint64_t value,
                          uint16_t slave_index = 0xFFFF)
        : obj_index_(obj_index), sub_index_(sub_index), has_sub_(has_sub),
          op_(op), value_(value), slave_index_(slave_index), fired_(false) {}

    DebugConditionType type() const override { return DebugConditionType::CoEIntercept; }
    bool hasFired() const override { return fired_; }
    void reset() override { fired_ = false; }

    void onCoERead(uint16_t slave_index, uint16_t index, uint8_t sub,
                   const uint8_t* data, size_t len) override;

    const char* description() const override { return desc_; }

private:
    uint16_t obj_index_;
    uint8_t sub_index_;
    bool has_sub_;
    CompareOp op_;
    uint64_t value_;
    uint16_t slave_index_;
    bool fired_;
    const char* desc_ = "";
};

class CustomCondition : public DebugCondition {
public:
    using Callback = std::function<bool()>;

    explicit CustomCondition(Callback cb)
        : cb_(std::move(cb)), fired_(false) {}

    DebugConditionType type() const override { return DebugConditionType::Custom; }
    bool hasFired() const override;
    void reset() override { fired_ = false; }

    const char* description() const override { return "custom"; }

private:
    Callback cb_;
    bool fired_;
};

// ============================================================================
// DebugGate — manages start/stop conditions for debug output
// ============================================================================

class DebugGate {
public:
    using GateChangedCallback = std::function<void()>;

    // ---- Global conditions (from CLI) ----
    void addGlobalStart(std::unique_ptr<DebugCondition> cond);
    void addGlobalStop(std::unique_ptr<DebugCondition> cond);

    // ---- Per-flag conditions (from code API) ----
    void addStartCondition(const std::string& flagName, std::unique_ptr<DebugCondition> cond);
    void addStopCondition(const std::string& flagName, std::unique_ptr<DebugCondition> cond);

    // ---- Query ----
    bool isActive(const std::string& flagName) const;
    bool isActiveForSlave(uint16_t slave_index) const;
    bool hasAnyConditions() const;

    // ---- Intercept hooks (called by Master) ----
    void onRegisterRead(uint16_t slave_index, uint16_t addr,
                        const uint8_t* data, uint16_t len);
    void onCoERead(uint16_t slave_index, uint16_t index, uint8_t sub,
                   const uint8_t* data, size_t len);
    void notifyCheckpoint(const std::string& name, uint16_t slave_index = 0xFFFF);

    // ---- Callback when gate state changes ----
    void setGateChangedCallback(GateChangedCallback cb);

    // ---- Parse CLI condition string → DebugCondition ----
    static std::unique_ptr<DebugCondition> parseCondition(const std::string& spec);

    // ---- Print help for condition syntax ----
    static void printHelp();

private:
    std::vector<std::unique_ptr<DebugCondition>> global_starts_;
    std::vector<std::unique_ptr<DebugCondition>> global_stops_;
    std::map<std::string, std::vector<std::unique_ptr<DebugCondition>>> flag_starts_;
    std::map<std::string, std::vector<std::unique_ptr<DebugCondition>>> flag_stops_;

    mutable bool global_active_ = false;
    mutable std::map<std::string, bool> flag_active_;
    GateChangedCallback gate_changed_cb_;

    void checkAndNotifyChange();
    bool computeGlobalActive() const;
    bool computeFlagActive(const std::string& flagName) const;
};

} // namespace EtherCAT

#else // !TETHER_DEBUG_GATE_ENABLED — no-op stubs with zero overhead

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace EtherCAT {

// Forward-declare SlaveState for the stub API
enum class SlaveState : uint8_t;

// Stub DebugCondition — empty type so unique_ptr<DebugCondition> still works
class DebugCondition {
public:
    virtual ~DebugCondition() = default;
};

// Stub DebugGate — all methods are inline no-ops
class DebugGate {
public:
    using GateChangedCallback = std::function<void()>;

    void addGlobalStart(std::unique_ptr<DebugCondition>) {}
    void addGlobalStop(std::unique_ptr<DebugCondition>) {}
    void addStartCondition(const std::string&, std::unique_ptr<DebugCondition>) {}
    void addStopCondition(const std::string&, std::unique_ptr<DebugCondition>) {}

    bool isActive(const std::string&) const { return true; }
    bool isActiveForSlave(uint16_t) const { return true; }
    bool hasAnyConditions() const { return false; }

    void onRegisterRead(uint16_t, uint16_t, const uint8_t*, uint16_t) {}
    void onCoERead(uint16_t, uint16_t, uint8_t, const uint8_t*, size_t) {}
    void notifyCheckpoint(const std::string&, uint16_t = 0xFFFF) {}

    void setGateChangedCallback(GateChangedCallback) {}

    static std::unique_ptr<DebugCondition> parseCondition(const std::string&) { return nullptr; }
    static void printHelp() {}
};

} // namespace EtherCAT

#endif // TETHER_DEBUG_GATE_ENABLED
