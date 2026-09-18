#include "tether/ethercat/SdoCommandChannel.hpp"

#include <thread>
#include <vector>

#include "logging/Logger.hpp"

namespace EtherCAT {

const char* sdoPollResultToString(SdoPollResult r) {
    switch (r) {
        case SdoPollResult::Success:   return "Success";
        case SdoPollResult::Rejected:  return "Rejected";
        case SdoPollResult::Timeout:   return "Timeout";
        case SdoPollResult::SDOError:  return "SDOError";
        case SdoPollResult::Cancelled: return "Cancelled";
    }
    return "Unknown";
}

// ---------------------------------------------------------------------------
// SdoReadFn
// ---------------------------------------------------------------------------

SdoReadFn makeSdoReadFn(Slave& slave) {
    return [&slave](uint16_t index, uint8_t subindex, uint8_t bytes,
                    uint32_t& out) -> SlaveError {
        switch (bytes) {
            case 1: {
                uint8_t u8 = 0;
                const auto err = slave.sdoReadU8(index, subindex, u8);
                out = u8;
                return err;
            }
            case 2: {
                uint16_t u16 = 0;
                const auto err = slave.sdoReadU16(index, subindex, u16);
                out = u16;
                return err;
            }
            default:
                return slave.sdoReadU32(index, subindex, out);
        }
    };
}

// ---------------------------------------------------------------------------
// waitForSdoValues
// ---------------------------------------------------------------------------

bool waitForSdoValues(const SdoReadFn& read,
                      std::span<const SdoExpectation> expectations,
                      std::chrono::milliseconds poll_interval,
                      std::chrono::milliseconds timeout,
                      std::stop_token stop,
                      const char* tag) {
    if (expectations.empty()) return true;

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::vector<bool> matched(expectations.size(), false);

    // One state-change logger per expectation: logs on change or every 10th
    // consecutive unchanged poll, to avoid flooding the log with identical
    // MISMATCH lines.  Mismatch → match is a state change, so the logger
    // handles the MATCH log line automatically.
    std::vector<StateChangeLogger<uint32_t>> loggers;
    loggers.reserve(expectations.size());
    for (size_t i = 0; i < expectations.size(); ++i) {
        const auto& exp = expectations[i];
        const char* name = exp.name ? exp.name : "?";
        const uint32_t expected = exp.expected;
        loggers.emplace_back(
            [name, expected, tag](uint32_t device_value, int attempt,
                                  bool changed) {
                if (device_value == expected) {
                    TETHER_LOGI(tag,
                        "{}: device=0x{:08X}, expected=0x{:08X} "
                        "[MATCH after {} poll(s)]",
                        name, device_value, expected, attempt);
                } else if (changed) {
                    TETHER_LOGI(tag,
                        "{}: device=0x{:08X}, expected=0x{:08X} "
                        "[MISMATCH, poll {}]",
                        name, device_value, expected, attempt);
                } else {
                    TETHER_LOGI(tag,
                        "{}: device=0x{:08X}, expected=0x{:08X} "
                        "[MISMATCH, poll {}] (unchanged)",
                        name, device_value, expected, attempt);
                }
            });
    }

    for (int attempt = 0;; ++attempt) {
        if (stop.stop_requested()) {
            TETHER_LOGI(tag, "waitForSdoValues: cancelled by stop request");
            return false;
        }
        bool all_matched = true;
        for (size_t i = 0; i < expectations.size(); ++i) {
            if (matched[i]) continue;

            uint32_t device_value = 0;
            auto err = read(expectations[i].index,
                            expectations[i].subindex, 4, device_value);
            if (err != SlaveError::Ok) {
                TETHER_LOGE(tag, "Failed to read {} (0x{:04X}:{}): {}",
                            expectations[i].name ? expectations[i].name : "?",
                            expectations[i].index,
                            static_cast<int>(expectations[i].subindex),
                            slaveErrorToString(err));
                return false;
            }

            loggers[i].update(device_value, attempt + 1);
            matched[i] = (device_value == expectations[i].expected);
            all_matched &= matched[i];
        }

        if (all_matched) return true;
        if (std::chrono::steady_clock::now() >= deadline) {
            for (size_t i = 0; i < expectations.size(); ++i) {
                if (!matched[i]) {
                    TETHER_LOGE(tag,
                        "{}: timed out waiting for match "
                        "(expected=0x{:08X})",
                        expectations[i].name ? expectations[i].name : "?",
                        expectations[i].expected);
                }
            }
            return false;
        }

        std::this_thread::sleep_for(poll_interval);
    }
}

bool waitForSdoValues(Slave& slave,
                      std::span<const SdoExpectation> expectations,
                      std::chrono::milliseconds poll_interval,
                      std::chrono::milliseconds timeout,
                      std::stop_token stop,
                      const char* tag) {
    return waitForSdoValues(makeSdoReadFn(slave), expectations,
                            poll_interval, timeout, stop, tag);
}

// ---------------------------------------------------------------------------
// pollSdoObjects
// ---------------------------------------------------------------------------

size_t pollSdoObjects(
    const SdoReadFn& read,
    std::span<const SdoObjectEntry> objects,
    std::span<uint32_t> values,
    std::span<bool> ok,
    const std::function<bool()>& shouldAbort,
    const std::function<void(size_t, const SdoObjectEntry&, SlaveError)>&
        onError) {
    size_t read_ok = 0;
    for (size_t i = 0; i < objects.size(); ++i) {
        if (shouldAbort && shouldAbort()) {
            break;
        }
        const auto& obj = objects[i];
        uint32_t v = 0;
        const auto err = read(obj.index, obj.subindex, obj.bytes, v);
        if (i < ok.size()) ok[i] = (err == SlaveError::Ok);
        if (err == SlaveError::Ok) {
            if (i < values.size()) values[i] = v;
            ++read_ok;
        } else if (onError) {
            onError(i, obj, err);
        }
    }
    return read_ok;
}

size_t pollSdoObjects(
    Slave& slave,
    std::span<const SdoObjectEntry> objects,
    std::span<uint32_t> values,
    std::span<bool> ok,
    const std::function<bool()>& shouldAbort,
    const std::function<void(size_t, const SdoObjectEntry&, SlaveError)>&
        onError) {
    return pollSdoObjects(makeSdoReadFn(slave), objects, values, ok,
                          shouldAbort, onError);
}

// ---------------------------------------------------------------------------
// SdoCommandChannel
// ---------------------------------------------------------------------------

SdoCommandChannel::SdoCommandChannel(Slave& slave, Config config,
                                     const char* tag)
    : slave_(slave), config_(config), tag_(tag),
      poll_logger_(
          [this](uint32_t response, int /*attempt*/, bool /*changed*/) {
              TETHER_LOGI(tag_,
                          "Command response poll (0x{:04X}:{}): {} "
                          "(0x{:08X}) [{}]",
                          config_.response_index,
                          static_cast<int>(config_.response_subindex),
                          response, response, responseName(response));
          }) {}

std::string SdoCommandChannel::responseName(uint32_t value) const {
    if (response_namer_) return response_namer_(value);
    return std::to_string(value);
}

// --- Command protocol ---

bool SdoCommandChannel::resetResponse() {
    if (!config_.pre_command_reset) return true;
    TETHER_LOGI(tag_, "Resetting command response (cmd register = {})...",
                config_.reset_value);
    auto err = slave_.sdoWriteU32(config_.command_index,
                                  config_.command_subindex,
                                  config_.reset_value);
    if (err != SlaveError::Ok) {
        TETHER_LOGE(tag_, "Failed to write reset value {}: {}",
                    config_.reset_value, slaveErrorToString(err));
        return false;
    }
    if (config_.reset_delay.count() > 0) {
        std::this_thread::sleep_for(config_.reset_delay);
    }
    return true;
}

bool SdoCommandChannel::sendCommand(uint32_t cmd) {
    TETHER_LOGI(tag_, "Writing command 0x{:04X}:{} = {} (0x{:08X})",
                config_.command_index,
                static_cast<int>(config_.command_subindex), cmd, cmd);
    auto err = slave_.sdoWriteU32(config_.command_index,
                                  config_.command_subindex, cmd);
    if (err != SlaveError::Ok) {
        TETHER_LOGE(tag_, "Failed to write command {}: {}",
                    cmd, slaveErrorToString(err));
        return false;
    }
    return true;
}

bool SdoCommandChannel::pollResponse(uint32_t& out) {
    auto err = slave_.sdoReadU32(config_.response_index,
                                 config_.response_subindex, out);
    if (err != SlaveError::Ok) {
        TETHER_LOGW(tag_, "Failed to read command response (0x{:04X}:{}): {}",
                    config_.response_index,
                    static_cast<int>(config_.response_subindex),
                    slaveErrorToString(err));
        return false;
    }
    last_response_.store(out);
    poll_logger_.update(out, 0);
    if (poll_observer_) poll_observer_();
    return true;
}

SdoCommandChannel::Result
SdoCommandChannel::waitForResponse(uint32_t cmd,
                                   const ResponseMatcher& matcher,
                                   std::chrono::milliseconds timeout) {
    const auto effective_timeout =
        timeout.count() > 0 ? timeout : config_.command_timeout;
    const auto deadline = std::chrono::steady_clock::now() + effective_timeout;
    uint32_t last_resp = 0xFFFFFFFF;
    poll_logger_.reset();  // first poll of each command always logs

    while (std::chrono::steady_clock::now() < deadline) {
        if (stop_token_.stop_requested()) {
            TETHER_LOGW(tag_, "waitForResponse cancelled by stop request");
            return Result::Cancelled;
        }

        uint32_t response = 0;
        if (!pollResponse(response)) {
            std::this_thread::sleep_for(config_.poll_interval);
            continue;
        }
        last_resp = response;

        if (response == config_.failure_response) {
            const uint32_t errCode = readErrorCode();
            TETHER_LOGE(tag_,
                        "Command {} failed — response={}, error=0x{:08X}",
                        cmd, response, errCode);
            return Result::Rejected;
        }

        if (matcher && matcher(cmd, response, slave_)) {
            TETHER_LOGI(tag_, "Command {}: success (response={} [{}])",
                        cmd, response, responseName(response));
            return Result::Success;
        }

        std::this_thread::sleep_for(config_.poll_interval);
    }

    const uint32_t errCode = readErrorCode();
    TETHER_LOGE(tag_,
                "Command {} timed out — last response={} (0x{:08X}), "
                "error=0x{:08X}",
                cmd, last_resp, last_resp, errCode);
    return Result::Timeout;
}

SdoCommandChannel::Result
SdoCommandChannel::sendAndWait(uint32_t cmd,
                               const ResponseMatcher& matcher,
                               std::chrono::milliseconds timeout) {
    if (!resetResponse()) {
        return Result::SDOError;
    }
    if (!sendCommand(cmd)) {
        return Result::SDOError;
    }
    auto result = waitForResponse(cmd, matcher, timeout);

    if (config_.post_command_reset) {
        TETHER_LOGI(tag_, "Post-command reset (={}) after command {}",
                    config_.reset_value, cmd);
        slave_.sdoWriteU32(config_.command_index, config_.command_subindex,
                           config_.reset_value);
        if (config_.reset_delay.count() > 0) {
            std::this_thread::sleep_for(config_.reset_delay);
        }
    }

    return result;
}

uint32_t SdoCommandChannel::readErrorCode() {
    if (config_.error_index == 0) return 0;
    uint32_t errCode = 0;
    slave_.sdoReadU32(config_.error_index, config_.error_subindex, errCode);
    return errCode;
}

// --- Generic polling ---

SdoCommandChannel::Result
SdoCommandChannel::pollUntil(uint16_t index, uint8_t subindex,
                             const std::function<bool(uint32_t)>& match,
                             std::chrono::milliseconds timeout,
                             const std::function<void(uint32_t)>& onValue) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    uint32_t last_value = 0;
    uint32_t sdo_errors = 0;

    while (std::chrono::steady_clock::now() < deadline) {
        if (stop_token_.stop_requested()) {
            TETHER_LOGW(tag_, "pollUntil aborted (stop requested)");
            return Result::Cancelled;
        }
        uint32_t value = 0;
        const auto err = slave_.sdoReadU32(index, subindex, value);
        if (err != SlaveError::Ok) {
            ++sdo_errors;
            if (sdo_errors == 1 || sdo_errors % 25 == 0) {
                TETHER_LOGW(tag_,
                            "pollUntil: read 0x{:04X}:{} failed ({}), "
                            "retrying",
                            index, static_cast<int>(subindex),
                            slaveErrorToString(err));
            }
        } else {
            last_value = value;
            last_response_.store(value);
            if (onValue) onValue(value);
            if (match && match(value)) {
                return Result::Success;
            }
        }
        if (poll_observer_) poll_observer_();
        std::this_thread::sleep_for(config_.poll_interval);
    }
    TETHER_LOGE(tag_,
                "pollUntil: timeout waiting on 0x{:04X}:{} "
                "(last value 0x{:08X}, {} SDO error(s))",
                index, static_cast<int>(subindex), last_value, sdo_errors);
    return Result::Timeout;
}

bool SdoCommandChannel::waitUntilMatch(
    std::span<const SdoExpectation> expectations,
    std::chrono::milliseconds timeout) {
    return waitForSdoValues(slave_, expectations, config_.poll_interval,
                            timeout, stop_token_, tag_);
}

} // namespace EtherCAT
