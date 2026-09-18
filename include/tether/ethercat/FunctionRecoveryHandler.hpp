#pragma once

#include <functional>

#include "tether/ethercat/SlaveSupervisor.hpp"

namespace EtherCAT {

/// ISlaveRecoveryHandler adapter around a std::function — lets recovery
/// logic be a lambda capturing local state (PDO mappings, mailbox config,
/// init helpers) instead of a dedicated class.
///
///   supervisor.setRecoveryHandler(
///       std::make_unique<FunctionRecoveryHandler>(
///           [&](uint16_t idx) { return doFullInit(idx); }));
class FunctionRecoveryHandler : public ISlaveRecoveryHandler {
public:
    explicit FunctionRecoveryHandler(std::function<bool(uint16_t)> fn)
        : fn_(std::move(fn)) {}

    bool reinitializeSlave(uint16_t slave_index) override {
        return fn_ ? fn_(slave_index) : false;
    }

private:
    std::function<bool(uint16_t)> fn_;
};

} // namespace EtherCAT
