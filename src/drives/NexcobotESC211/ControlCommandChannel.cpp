#include "tether/drives/NexcobotESC211/ControlCommandChannel.hpp"

#include "logging/Logger.hpp"

namespace EtherCAT {
namespace Drives {
namespace ESC211 {

namespace UserSystem = EtherCAT::Drives::Registers::NexcobotESC211::UserSystem;

// --- Pre-built matchers ---

bool ControlCommandChannel::matchDone(uint32_t /*cmdCode*/, uint32_t response,
                                      EtherCAT::Slave& /*slave*/) {
    return response == static_cast<uint32_t>(CommandResponseCode::Done);
}

bool ControlCommandChannel::matchCmd4(uint32_t /*cmdCode*/, uint32_t response,
                                      EtherCAT::Slave& slave) {
    if (response == static_cast<uint32_t>(CommandResponseCode::Done))
        return true;
    // Command 4 returns to Standby (0) with error code 0 on success.
    if (response == static_cast<uint32_t>(CommandResponseCode::Standby)) {
        uint32_t errCode = 0;
        slave.sdoReadU32(UserSystem::UserControlIndex, 0x03, errCode);
        return errCode == 0;
    }
    return false;
}

bool ControlCommandChannel::matchOngoingOrDone(uint32_t /*cmdCode*/, uint32_t response,
                                               EtherCAT::Slave& /*slave*/) {
    return response == static_cast<uint32_t>(CommandResponseCode::Ongoing) ||
           response == static_cast<uint32_t>(CommandResponseCode::Done);
}

// --- Constructor ---

ControlCommandChannel::ControlCommandChannel(EtherCAT::Slave& slave,
                                             Config config,
                                             const char* tag)
    : channel_(
          slave,
          EtherCAT::SdoCommandChannel::Config{
              .command_index      = UserSystem::UserControlIndex,
              .command_subindex   = 0x01,
              .response_index     = UserSystem::UserControlIndex,
              .response_subindex  = 0x02,
              .error_index        = UserSystem::UserControlIndex,
              .error_subindex     = 0x03,
              .command_timeout    = config.command_timeout,
              .poll_interval      = config.poll_interval,
              // Manufacturer requirement: CMD=0 + delay before every
              // command, or the device silently ignores it.
              .pre_command_reset  = true,
              .reset_value        =
                  static_cast<uint32_t>(ControlCommandCode::ResetResponseToStandby),
              .reset_delay        = config.pre_command_reset_delay,
              .post_command_reset = config.post_command_reset,
              .failure_response   =
                  static_cast<uint32_t>(CommandResponseCode::Failed),
          },
          tag),
      tag_(tag) {
    channel_.setResponseNamer(
        [](uint32_t v) { return std::string(responseName(v)); });
}

// --- Low-level primitives ---

bool ControlCommandChannel::resetResponseToStandby() {
    return channel_.resetResponse();
}

bool ControlCommandChannel::sendCommand(ControlCommandCode cmd) {
    TETHER_LOGI(tag_,
                "Sending control command {} (0x{:08X}) [{}]",
                static_cast<uint32_t>(cmd), static_cast<uint32_t>(cmd),
                commandName(cmd));
    return channel_.sendCommand(static_cast<uint32_t>(cmd));
}

bool ControlCommandChannel::pollResponse(uint32_t& out) {
    return channel_.pollResponse(out);
}

ControlCommandChannel::Result
ControlCommandChannel::waitForResponse(ControlCommandCode cmd,
                                       const ResponseMatcher& matcher,
                                       std::chrono::milliseconds timeout) {
    const auto effective = matcher ? matcher : ResponseMatcher(matchDone);
    return mapResult(channel_.waitForResponse(static_cast<uint32_t>(cmd),
                                              effective, timeout));
}

// --- Convenience ---

ControlCommandChannel::Result
ControlCommandChannel::sendAndWait(ControlCommandCode cmd,
                                   const ResponseMatcher& matcher,
                                   std::chrono::milliseconds timeout) {
    const auto effective = matcher ? matcher : ResponseMatcher(matchDone);
    return mapResult(channel_.sendAndWait(static_cast<uint32_t>(cmd),
                                          effective, timeout));
}

uint32_t ControlCommandChannel::readErrorCode() {
    return channel_.readErrorCode();
}

ControlCommandChannel::Result
ControlCommandChannel::pollUntil(
    uint16_t index, uint8_t subindex,
    const std::function<bool(uint32_t)>& match,
    std::chrono::milliseconds timeout,
    const std::function<void(uint32_t)>& onValue) {
    return mapResult(channel_.pollUntil(index, subindex, match, timeout,
                                        onValue));
}

// --- Result mapping ---

ControlCommandChannel::Result
ControlCommandChannel::mapResult(EtherCAT::SdoPollResult r) {
    switch (r) {
        case EtherCAT::SdoPollResult::Success:   return Result::Success;
        case EtherCAT::SdoPollResult::Rejected:  return Result::Failed;
        case EtherCAT::SdoPollResult::SDOError:  return Result::SDOError;
        // The pre-Tether implementation reported a cancelled wait as a
        // timeout — keep that observable behavior.
        case EtherCAT::SdoPollResult::Cancelled:
        case EtherCAT::SdoPollResult::Timeout:   return Result::Timeout;
    }
    return Result::SDOError;
}

// --- Strings ---

const char* ControlCommandChannel::resultToString(Result r) {
    switch (r) {
        case Result::Success:  return "Success";
        case Result::Failed:   return "Failed";
        case Result::Timeout:  return "Timeout";
        case Result::SDOError: return "SDOError";
    }
    return "Unknown";
}

const char* ControlCommandChannel::commandName(ControlCommandCode cmd) {
    switch (cmd) {
        case ControlCommandCode::ResetResponseToStandby:    return "Reset response to standby";
        case ControlCommandCode::LoadFNIFromFlash:          return "Load FNI from flash";
        case ControlCommandCode::LoadRSPAndSDDFromFlash:    return "Load RSP and SDD from flash";
        case ControlCommandCode::UpdateFNIFromObject:       return "Update FNI from object";
        case ControlCommandCode::DownloadRSPAndSDDByObject: return "Download RSP and SDD by object";
        case ControlCommandCode::SaveFNIToFlash:            return "Save FNI to Flash";
        case ControlCommandCode::SaveRSPAndSDDToFlash:      return "Save RSP and SDD to flash";
        case ControlCommandCode::ClearApplicationErrorState:return "Clear application error state";
        case ControlCommandCode::ClearLastErrorCode:        return "Clear last error code";
        case ControlCommandCode::StopSafetyAppAndFSoE:      return "Stop safety app and FSoE";
        case ControlCommandCode::StartFSoEAndSafetyApp:     return "Start FSoE and safety app";
        case ControlCommandCode::StartFSoEConnectionOnly:   return "Start FSoE connection only";
        case ControlCommandCode::FlashMemoryReset:          return "Flash memory reset";
    }
    return "Unknown";
}

const char* ControlCommandChannel::responseName(uint32_t code) {
    switch (static_cast<CommandResponseCode>(code)) {
        case CommandResponseCode::Standby: return "Standby";
        case CommandResponseCode::Ongoing: return "Ongoing";
        case CommandResponseCode::Done:    return "Done";
        case CommandResponseCode::Failed:  return "Failed";
    }
    return "Unknown";
}

} // namespace ESC211
} // namespace Drives
} // namespace EtherCAT
