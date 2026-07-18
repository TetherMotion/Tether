/**
 * @file DebugGate.cpp
 * @brief Implementation of conditional debug gating framework.
 */

#include "tether/ethercat/DebugGate.hpp"

#if TETHER_DEBUG_GATE_ENABLED

#include "tether/platform/Platform.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>

namespace EtherCAT {

static const char* TAG = "DebugGate";

// ============================================================================
// Helpers
// ============================================================================

static const char* compareOpStr(CompareOp op) {
    switch (op) {
        case CompareOp::Eq:      return "==";
        case CompareOp::Ne:      return "!=";
        case CompareOp::Bitmask: return "&";
        case CompareOp::Ge:      return ">=";
        case CompareOp::Le:      return "<=";
        case CompareOp::Gt:      return ">";
        case CompareOp::Lt:      return "<";
        default:                 return "?";
    }
}

static bool compareValues(uint64_t read_val, CompareOp op, uint64_t expected) {
    switch (op) {
        case CompareOp::Eq:      return read_val == expected;
        case CompareOp::Ne:      return read_val != expected;
        case CompareOp::Bitmask: return (read_val & expected) == expected;
        case CompareOp::Ge:      return read_val >= expected;
        case CompareOp::Le:      return read_val <= expected;
        case CompareOp::Gt:      return read_val >  expected;
        case CompareOp::Lt:      return read_val <  expected;
        default:                 return false;
    }
}

static uint64_t readLEValue(const uint8_t* data, size_t len) {
    uint64_t val = 0;
    size_t copy_len = len < sizeof(uint64_t) ? len : sizeof(uint64_t);
    for (size_t i = 0; i < copy_len; ++i) {
        val |= static_cast<uint64_t>(data[i]) << (8 * i);
    }
    return val;
}

static const char* stateCheckpointName(SlaveState state) {
    switch (state) {
        case SlaveState::INIT:    return "state:init";
        case SlaveState::PRE_OP:  return "state:pre-op";
        case SlaveState::BOOT:    return "state:boot";
        case SlaveState::SAFE_OP: return "state:safe-op";
        case SlaveState::OP:      return "state:op";
        default:                  return "state:unknown";
    }
}

// ============================================================================
// StateCondition
// ============================================================================

void StateCondition::onCheckpoint(const std::string& name, uint16_t slave_index) {
    if (fired_) return;
    if (name == stateCheckpointName(state_)) {
        if (slave_index_ == 0xFFFF || slave_index_ == slave_index) {
            fired_ = true;
        }
    }
}

// ============================================================================
// CheckpointCondition
// ============================================================================

void CheckpointCondition::onCheckpoint(const std::string& name, uint16_t slave_index) {
    if (fired_) return;
    if (name == checkpoint_name_) {
        if (slave_index_ == 0xFFFF || slave_index_ == slave_index) {
            fired_ = true;
        }
    }
}

// ============================================================================
// RegisterInterceptCondition
// ============================================================================

void RegisterInterceptCondition::onRegisterRead(uint16_t slave_index, uint16_t addr,
                                                 const uint8_t* data, uint16_t len) {
    if (fired_) return;
    if (addr != reg_addr_) return;
    if (slave_index_ != 0xFFFF && slave_index_ != slave_index) return;
    if (len == 0 || !data) return;

    uint64_t read_val = readLEValue(data, len);
    if (compareValues(read_val, op_, value_)) {
        fired_ = true;
    }
}

// ============================================================================
// CoEInterceptCondition
// ============================================================================

void CoEInterceptCondition::onCoERead(uint16_t slave_index, uint16_t index, uint8_t sub,
                                       const uint8_t* data, size_t len) {
    if (fired_) return;
    if (index != obj_index_) return;
    if (has_sub_ && sub != sub_index_) return;
    if (slave_index_ != 0xFFFF && slave_index_ != slave_index) return;
    if (len == 0 || !data) return;

    uint64_t read_val = readLEValue(data, len);
    if (compareValues(read_val, op_, value_)) {
        fired_ = true;
    }
}

// ============================================================================
// CustomCondition
// ============================================================================

bool CustomCondition::hasFired() const {
    if (fired_) return true;
    if (cb_ && cb_()) {
        const_cast<CustomCondition*>(this)->fired_ = true;
        return true;
    }
    return false;
}

// ============================================================================
// DebugGate
// ============================================================================

void DebugGate::addGlobalStart(std::unique_ptr<DebugCondition> cond) {
    global_starts_.push_back(std::move(cond));
}

void DebugGate::addGlobalStop(std::unique_ptr<DebugCondition> cond) {
    global_stops_.push_back(std::move(cond));
}

void DebugGate::addStartCondition(const std::string& flagName,
                                  std::unique_ptr<DebugCondition> cond) {
    flag_starts_[flagName].push_back(std::move(cond));
}

void DebugGate::addStopCondition(const std::string& flagName,
                                 std::unique_ptr<DebugCondition> cond) {
    flag_stops_[flagName].push_back(std::move(cond));
}

bool DebugGate::hasAnyConditions() const {
    return !global_starts_.empty() || !global_stops_.empty() ||
           !flag_starts_.empty() || !flag_stops_.empty();
}

bool DebugGate::computeGlobalActive() const {
    // If no global start conditions, global gate is open
    if (global_starts_.empty()) return true;

    // Any global start condition firing opens the gate
    bool any_start = false;
    for (const auto& c : global_starts_) {
        if (c->hasFired()) { any_start = true; break; }
    }
    if (!any_start) return false;

    // Any global stop condition firing closes the gate
    for (const auto& c : global_stops_) {
        if (c->hasFired()) return false;
    }

    return true;
}

bool DebugGate::computeFlagActive(const std::string& flagName) const {
    // If no per-flag conditions for this flag, it follows the global gate
    auto sit = flag_starts_.find(flagName);
    auto eit = flag_stops_.find(flagName);
    bool has_starts = (sit != flag_starts_.end() && !sit->second.empty());
    bool has_stops = (eit != flag_stops_.end() && !eit->second.empty());

    if (!has_starts && !has_stops) {
        return computeGlobalActive();
    }

    // Per-flag start conditions: any must fire
    if (has_starts) {
        bool any_start = false;
        for (const auto& c : sit->second) {
            if (c->hasFired()) { any_start = true; break; }
        }
        if (!any_start) return false;
    }

    // Per-flag stop conditions: any firing closes
    if (has_stops) {
        for (const auto& c : eit->second) {
            if (c->hasFired()) return false;
        }
    }

    // Also respect global gate
    return computeGlobalActive();
}

bool DebugGate::isActive(const std::string& flagName) const {
    if (!hasAnyConditions()) return true;
    return computeFlagActive(flagName);
}

bool DebugGate::isActiveForSlave(uint16_t slave_index) const {
    if (!hasAnyConditions()) return true;

    // Check global gate first
    if (!computeGlobalActive()) return false;

    // If any per-flag conditions exist, check if any flag is active for this slave
    // We don't have slave-specific flag info here, so we check all flags
    // If no per-flag conditions at all, global gate is sufficient
    if (flag_starts_.empty() && flag_stops_.empty()) return true;

    // With per-flag conditions, we can't know which flags apply to this slave
    // without the full flag set. Return true — the per-flag check happens
    // in computeForSlave() which calls isActive(flagName).
    return true;
}

void DebugGate::onRegisterRead(uint16_t slave_index, uint16_t addr,
                               const uint8_t* data, uint16_t len) {
    if (!hasAnyConditions()) return;

    bool was_active = computeGlobalActive();
    for (auto& c : global_starts_) c->onRegisterRead(slave_index, addr, data, len);
    for (auto& c : global_stops_) c->onRegisterRead(slave_index, addr, data, len);
    for (auto& [name, conds] : flag_starts_) {
        for (auto& c : conds) c->onRegisterRead(slave_index, addr, data, len);
    }
    for (auto& [name, conds] : flag_stops_) {
        for (auto& c : conds) c->onRegisterRead(slave_index, addr, data, len);
    }
    checkAndNotifyChange();
}

void DebugGate::onCoERead(uint16_t slave_index, uint16_t index, uint8_t sub,
                          const uint8_t* data, size_t len) {
    if (!hasAnyConditions()) return;

    for (auto& c : global_starts_) c->onCoERead(slave_index, index, sub, data, len);
    for (auto& c : global_stops_) c->onCoERead(slave_index, index, sub, data, len);
    for (auto& [name, conds] : flag_starts_) {
        for (auto& c : conds) c->onCoERead(slave_index, index, sub, data, len);
    }
    for (auto& [name, conds] : flag_stops_) {
        for (auto& c : conds) c->onCoERead(slave_index, index, sub, data, len);
    }
    checkAndNotifyChange();
}

void DebugGate::notifyCheckpoint(const std::string& name, uint16_t slave_index) {
    if (!hasAnyConditions()) return;

    for (auto& c : global_starts_) c->onCheckpoint(name, slave_index);
    for (auto& c : global_stops_) c->onCheckpoint(name, slave_index);
    for (auto& [n, conds] : flag_starts_) {
        for (auto& c : conds) c->onCheckpoint(name, slave_index);
    }
    for (auto& [n, conds] : flag_stops_) {
        for (auto& c : conds) c->onCheckpoint(name, slave_index);
    }
    checkAndNotifyChange();
}

void DebugGate::setGateChangedCallback(GateChangedCallback cb) {
    gate_changed_cb_ = std::move(cb);
}

void DebugGate::checkAndNotifyChange() {
    bool new_global = computeGlobalActive();
    if (new_global != global_active_) {
        global_active_ = new_global;
        if (gate_changed_cb_) gate_changed_cb_();
        return;
    }

    // Check per-flag changes
    bool any_changed = false;
    for (const auto& [name, conds] : flag_starts_) {
        bool new_active = computeFlagActive(name);
        auto it = flag_active_.find(name);
        if (it == flag_active_.end() || it->second != new_active) {
            flag_active_[name] = new_active;
            any_changed = true;
        }
    }
    if (any_changed && gate_changed_cb_) {
        gate_changed_cb_();
    }
}

// ============================================================================
// CLI condition parsing
// ============================================================================

static CompareOp parseCompareOp(const std::string& s) {
    if (s == "==") return CompareOp::Eq;
    if (s == "!=") return CompareOp::Ne;
    if (s == "&")  return CompareOp::Bitmask;
    if (s == ">=") return CompareOp::Ge;
    if (s == "<=") return CompareOp::Le;
    if (s == ">")  return CompareOp::Gt;
    if (s == "<")  return CompareOp::Lt;
    return CompareOp::Eq;
}

static uint64_t parseValue(const std::string& s) {
    if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        return std::stoull(s.substr(2), nullptr, 16);
    }
    return std::stoull(s, nullptr, 10);
}

static std::unique_ptr<DebugCondition> parseStateCondition(const std::string& spec) {
    // state:pre-op, state:safe-op, state:op, state:init, state:boot
    // Optional: state:pre-op:0 (slave index)
    std::istringstream iss(spec);
    std::string token;
    std::vector<std::string> parts;
    while (std::getline(iss, token, ':')) {
        parts.push_back(token);
    }

    if (parts.size() < 2) return nullptr;

    const std::string& state_str = parts[1];
    SlaveState state = SlaveState::INIT;
    if (state_str == "init" || state_str == "INIT") {
        state = SlaveState::INIT;
    } else if (state_str == "pre-op" || state_str == "PRE-OP" || state_str == "preop") {
        state = SlaveState::PRE_OP;
    } else if (state_str == "boot" || state_str == "BOOT") {
        state = SlaveState::BOOT;
    } else if (state_str == "safe-op" || state_str == "SAFE-OP" || state_str == "safeop") {
        state = SlaveState::SAFE_OP;
    } else if (state_str == "op" || state_str == "OP") {
        state = SlaveState::OP;
    } else {
        TETHER_LOGE(TAG, "Unknown state in condition: '%s'", state_str.c_str());
        return nullptr;
    }

    uint16_t slave_index = 0xFFFF;
    if (parts.size() >= 3) {
        slave_index = static_cast<uint16_t>(std::stoul(parts[2], nullptr, 10));
    }

    return std::make_unique<StateCondition>(state, slave_index);
}

static std::unique_ptr<DebugCondition> parseCheckpointCondition(const std::string& spec) {
    // checkpoint:name or checkpoint:name:slave_index
    std::istringstream iss(spec);
    std::string token;
    std::vector<std::string> parts;
    while (std::getline(iss, token, ':')) {
        parts.push_back(token);
    }

    if (parts.size() < 2) return nullptr;

    const std::string& name = parts[1];
    uint16_t slave_index = 0xFFFF;
    if (parts.size() >= 3) {
        slave_index = static_cast<uint16_t>(std::stoul(parts[2], nullptr, 10));
    }

    return std::make_unique<CheckpointCondition>(name, slave_index);
}

static std::unique_ptr<DebugCondition> parseRegisterCondition(const std::string& spec) {
    // reg:[slave:]addr:op:value
    // e.g. reg:0x0130:==:0x0002
    //      reg:0:0x0130:==:0x0002
    std::istringstream iss(spec);
    std::string token;
    std::vector<std::string> parts;
    while (std::getline(iss, token, ':')) {
        parts.push_back(token);
    }

    if (parts.size() < 4) return nullptr;

    // Determine if first part after "reg" is a slave index or an address
    // Addresses typically start with 0x, slave indices are small decimal numbers
    uint16_t slave_index = 0xFFFF;
    uint16_t addr = 0;
    size_t op_idx = 0;

    // Check if parts[1] looks like an address (starts with 0x) or a slave index
    if (parts[1].size() >= 2 && parts[1][0] == '0' && (parts[1][1] == 'x' || parts[1][1] == 'X')) {
        // parts[1] is address, parts[2] is op, parts[3] is value
        addr = static_cast<uint16_t>(std::stoul(parts[1], nullptr, 16));
        op_idx = 2;
    } else if (parts.size() >= 5) {
        // parts[1] is slave index, parts[2] is address, parts[3] is op, parts[4] is value
        slave_index = static_cast<uint16_t>(std::stoul(parts[1], nullptr, 10));
        addr = static_cast<uint16_t>(std::stoul(parts[2], nullptr, 16));
        op_idx = 3;
    } else {
        return nullptr;
    }

    if (op_idx + 1 >= parts.size()) return nullptr;

    CompareOp op = parseCompareOp(parts[op_idx]);
    uint64_t value = parseValue(parts[op_idx + 1]);

    return std::make_unique<RegisterInterceptCondition>(addr, op, value, slave_index);
}

static std::unique_ptr<DebugCondition> parseCoECondition(const std::string& spec) {
    // coe:[slave:]index[:sub]:op:value
    // e.g. coe:0x6051:0x00:==:0x0001
    //      coe:0x6051:==:0x0001  (any subindex)
    //      coe:0:0x6051:0x00:==:0x0001  (slave 0)
    std::istringstream iss(spec);
    std::string token;
    std::vector<std::string> parts;
    while (std::getline(iss, token, ':')) {
        parts.push_back(token);
    }

    if (parts.size() < 4) return nullptr;

    uint16_t slave_index = 0xFFFF;
    uint16_t obj_index = 0;
    uint8_t sub_index = 0;
    bool has_sub = false;
    size_t op_idx = 0;

    // Try to figure out the layout
    // If parts[1] starts with 0x → it's the object index
    if (parts[1].size() >= 2 && parts[1][0] == '0' && (parts[1][1] == 'x' || parts[1][1] == 'X')) {
        // parts[1] = object index
        obj_index = static_cast<uint16_t>(std::stoul(parts[1], nullptr, 16));
        // Now check if parts[2] is a subindex or an operator
        if (parts[2] == "==" || parts[2] == "!=" || parts[2] == "&" ||
            parts[2] == ">=" || parts[2] == "<=" || parts[2] == ">" || parts[2] == "<") {
            // No subindex, parts[2] is op, parts[3] is value
            op_idx = 2;
        } else {
            // parts[2] is subindex
            sub_index = static_cast<uint8_t>(std::stoul(parts[2], nullptr, 16));
            has_sub = true;
            op_idx = 3;
        }
    } else {
        // parts[1] is slave index
        slave_index = static_cast<uint16_t>(std::stoul(parts[1], nullptr, 10));
        if (parts.size() < 5) return nullptr;
        obj_index = static_cast<uint16_t>(std::stoul(parts[2], nullptr, 16));
        if (parts[3] == "==" || parts[3] == "!=" || parts[3] == "&" ||
            parts[3] == ">=" || parts[3] == "<=" || parts[3] == ">" || parts[3] == "<") {
            op_idx = 3;
        } else {
            sub_index = static_cast<uint8_t>(std::stoul(parts[3], nullptr, 16));
            has_sub = true;
            op_idx = 4;
        }
    }

    if (op_idx + 1 >= parts.size()) return nullptr;

    CompareOp op = parseCompareOp(parts[op_idx]);
    uint64_t value = parseValue(parts[op_idx + 1]);

    return std::make_unique<CoEInterceptCondition>(obj_index, sub_index, has_sub,
                                                    op, value, slave_index);
}

std::unique_ptr<DebugCondition> DebugGate::parseCondition(const std::string& spec) {
    if (spec.empty()) return nullptr;

    // Determine condition type by prefix
    if (spec.substr(0, 6) == "state:") {
        return parseStateCondition(spec);
    } else if (spec.substr(0, 11) == "checkpoint:") {
        return parseCheckpointCondition(spec);
    } else if (spec.substr(0, 4) == "reg:") {
        return parseRegisterCondition(spec);
    } else if (spec.substr(0, 4) == "coe:") {
        return parseCoECondition(spec);
    } else {
        TETHER_LOGE(TAG, "Unknown condition type: '%s'", spec.c_str());
        TETHER_LOGI(TAG, "Valid prefixes: state:, checkpoint:, reg:, coe:");
        return nullptr;
    }
}

void DebugGate::printHelp() {
    TETHER_LOGI(TAG, "=== Debug Start/Stop Condition Syntax ===");
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "Condition types (type-prefixed, colon-separated):");
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "  state:<state>[:<slave>]       - EtherCAT state transition");
    TETHER_LOGI(TAG, "    States: init, pre-op, boot, safe-op, op");
    TETHER_LOGI(TAG, "    Example: state:pre-op");
    TETHER_LOGI(TAG, "    Example: state:safe-op:0  (slave 0 only)");
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "  checkpoint:<name>[:<slave>]   - Named program-flow checkpoint");
    TETHER_LOGI(TAG, "    Available checkpoints:");
    TETHER_LOGI(TAG, "      state:init, state:pre-op, state:safe-op, state:op");
    TETHER_LOGI(TAG, "      discovery-complete, mailbox-configured");
    TETHER_LOGI(TAG, "      first-txpdo, first-rxpdo");
    TETHER_LOGI(TAG, "    Example: checkpoint:first-txpdo");
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "  reg:[<slave>:]<addr>:<op>:<value>  - Register read intercept");
    TETHER_LOGI(TAG, "    Ops: ==, !=, &, >=, <=, >, <");
    TETHER_LOGI(TAG, "    Example: reg:0x0130:==:0x0002  (any slave)");
    TETHER_LOGI(TAG, "    Example: reg:0:0x0130:==:0x0002  (slave 0 only)");
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "  coe:[<slave>:]<index>[:<sub>]:<op>:<value>  - CoE/SDO read intercept");
    TETHER_LOGI(TAG, "    Example: coe:0x6051:0x00:==:0x0001");
    TETHER_LOGI(TAG, "    Example: coe:0x6051:==:0x0001  (any subindex)");
    TETHER_LOGI(TAG, "    Example: coe:0:0x6051:0x00:==:0x0001  (slave 0 only)");
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "Usage:");
    TETHER_LOGI(TAG, "  --debug-start <condition>   Start debug output when condition fires");
    TETHER_LOGI(TAG, "  --debug-stop  <condition>   Stop debug output when condition fires");
    TETHER_LOGI(TAG, "");
    TETHER_LOGI(TAG, "If --debug-start is not given, debug is active immediately (current behavior).");
    TETHER_LOGI(TAG, "If --debug-stop is not given, debug stays active once started.");
}

} // namespace EtherCAT

#endif // TETHER_DEBUG_GATE_ENABLED
