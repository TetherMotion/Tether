/**
 * @file KlipperDevice.cpp
 * @brief KlipperDevice implementation.
 */

#include "tether/klipper/device/KlipperDevice.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/protocol/Crc16.hpp"
#include "tether/klipper/reliability/SequenceCounter.hpp"

#include <cstring>

namespace tether::klipper::device {

KlipperDevice::KlipperDevice(std::shared_ptr<transport::IByteStreamTransport> transport,
                               protocol::DataDictionary dict,
                               KlipperDeviceConfig config)
    : transport_(std::move(transport))
    , dict_(std::move(dict))
    , config_(std::move(config))
    , mcuClock_(config_.clockFreqHz)
    , blockReader_(*transport_) {
    commandTable_ = std::make_unique<protocol::CommandTable>(dict_);
    // Build the identify server from the dictionary's wire blob.
    identifyServer_ = std::make_unique<protocol::IdentifyServer>(dict_.toWire());
    // Create the real-time step scheduler if enabled.
    if (config_.useStepScheduler) {
        stepScheduler_ = std::make_unique<motion::StepScheduler>(config_.clockFreqHz);
    }
    // Compute config CRC from the dictionary wire bytes.
    auto wire = dict_.toWire();
    configCrc_ = protocol::crc16Ccitt(wire);
    // Register default handlers for core commands.
    enableDefaultCommands();
}

KlipperDevice::~KlipperDevice() = default;

bool KlipperDevice::start() {
    if (!transport_->open()) return false;
    started_ = true;
    return true;
}

void KlipperDevice::pump() {
    if (!started_) return;
    // Use the streaming BlockReader: it reads from the transport and parses
    // complete blocks, handling partial data and (in recovery mode) corrupt
    // blocks transparently. This replaces the manual readAll()+parseBlock()
    // loop and surfaces parse statistics via blockParseStats().
    protocol::MessageBlock block;
    while (blockReader_.readNext(block)) {
        processBlock(block);
    }
}

void KlipperDevice::reset() {
    lastRecvSeq_ = 0;
    shutdown_ = false;
    configFinalized_ = false;
    oidAllocator_.reset();
    steppers_.clear();
    peripherals_.clear();
    stepperBaseClocks_.clear();
    blockReader_.reset();
    // Re-register default command handlers (clears any application-registered
    // handlers that may have referenced now-stale state).
    enableDefaultCommands();
}

void KlipperDevice::processBlock(const protocol::MessageBlock& block) {
    // Update last received in-order sequence.
    // Simple in-order check: accept if seq == lastRecvSeq_ + 1 (mod 16) or
    // if this is the first block.
    uint8_t expected = (lastRecvSeq_ + 1) & 0x0F;
    if (block.sequence == expected || lastRecvSeq_ == 0 && block.sequence == 0) {
        lastRecvSeq_ = block.sequence;
    }
    // Send ack for the received block.
    sendAck(lastRecvSeq_);

    if (block.content.empty()) return;

    // Check for identify command (msgid 1) before dictionary-based decoding,
    // since the identify handshake happens before the dictionary is available
    // on the host side and the identify msgids are hardcoded.
    const uint8_t* p = block.content.data();
    const uint8_t* end = p + block.content.size();
    auto firstMsgid = protocol::decodeMsgId(p, end);
    if (firstMsgid && *firstMsgid == protocol::kMsgIdIdentify) {
        // Parse identify params: offset=%u count=%c
        auto offsetOpt = protocol::decodeParam(p, end);
        auto countOpt = protocol::decodeParam(p, end);
        if (offsetOpt && countOpt) {
            uint32_t offset = static_cast<uint32_t>(*offsetOpt);
            uint8_t count = static_cast<uint8_t>(*countOpt);
            auto respContent = identifyServer_->buildResponseContent(offset, count);
            auto blockBytes = protocol::buildBlockVec(block.sequence, respContent);
            transport_->write(blockBytes);
        }
        return;
    }

    // Decode and dispatch commands.
    auto msgs = protocol::decodeMessages(dict_, block.content);
    for (const auto& msg : msgs) {
        // Check for identify command (msgid 1).
        if (msg.msgid == protocol::kMsgIdIdentify) {
            // Build identify_response and send it.
            if (msg.params.size() >= 2) {
                uint32_t offset = static_cast<uint32_t>(msg.params[0].integer);
                uint8_t count = static_cast<uint8_t>(msg.params[1].integer);
                auto respContent = identifyServer_->buildResponseContent(offset, count);
                // Build a response block and send it.
                std::vector<uint8_t> blockBytes = protocol::buildBlockVec(
                    block.sequence, respContent);
                transport_->write(blockBytes);
            }
            continue;
        }
        // Check for get_clock command.
        auto getClockOpt = dict_.lookupCommand("get_clock");
        if (getClockOpt && msg.msgid == *getClockOpt) {
            // Respond with current clock.
            sendResponse("clock clock=%u", {protocol::ParamValue{static_cast<int32_t>(mcuClock_.ticks32())}});
            continue;
        }
        // Dispatch to registered handlers.
        if (commandTable_) commandTable_->dispatchCommand(msg);
    }
}

void KlipperDevice::sendAck(uint8_t seq) {
    auto ack = protocol::buildAckBlock(seq);
    transport_->write(ack);
}

void KlipperDevice::advanceClock(uint32_t deltaTicks) {
    mcuClock_.advanceTo(mcuClock_.ticks32() + deltaTicks);
}

void KlipperDevice::registerPeripheral(uint8_t oid, std::shared_ptr<void> peripheral) {
    peripherals_[oid] = std::move(peripheral);
}

uint8_t KlipperDevice::registerStepper(std::shared_ptr<objects::Stepper> stepper) {
    uint8_t oid = stepper->oid();
    peripherals_[oid] = stepper;
    steppers_[oid] = std::move(stepper);
    return oid;
}

void KlipperDevice::enableDefaultCommands() {
    // allocate_oids oid=%c — allocate a block of OIDs.
    onCommand("allocate_oids oid=%c",
        [this](const std::vector<protocol::ParamValue>& params) {
            if (params.size() < 1) return;
            uint8_t count = static_cast<uint8_t>(params[0].integer);
            oidAllocator_.allocateBlock(count);
        });

    // get_config — respond with oid_count and config_crc.
    onCommand("get_config",
        [this](const std::vector<protocol::ParamValue>&) {
            sendResponse("config_result oid_count=%c config_crc=%u",
                {protocol::ParamValue{static_cast<int32_t>(oidAllocator_.nextOid())},
                 protocol::ParamValue{static_cast<int32_t>(configCrc_)}});
        });

    // get_status — respond with current clock and status byte.
    onCommand("get_status",
        [this](const std::vector<protocol::ParamValue>&) {
            uint8_t status = shutdown_ ? 1 : 0;
            sendResponse("status clock=%u status=%c",
                {protocol::ParamValue{static_cast<int32_t>(mcuClock_.ticks32())},
                 protocol::ParamValue{static_cast<int32_t>(status)}});
        });

    // shutdown — enter shutdown state.
    onCommand("shutdown",
        [this](const std::vector<protocol::ParamValue>&) {
            shutdown_ = true;
        });

    // finalize_config crc=%u — lock config and verify CRC.
    onCommand("finalize_config crc=%u",
        [this](const std::vector<protocol::ParamValue>& params) {
            if (params.size() < 1) return;
            uint32_t crc = static_cast<uint32_t>(params[0].integer);
            configFinalized_ = true;
            if (crc != 0) configCrc_ = crc;
            sendResponse("config_result oid_count=%c config_crc=%u",
                {protocol::ParamValue{static_cast<int32_t>(oidAllocator_.nextOid())},
                 protocol::ParamValue{static_cast<int32_t>(configCrc_)}});
        });
}

void KlipperDevice::enableStepperMotion() {
    // If the StepScheduler is enabled, wire its step callback to update
    // the Stepper's position counter (simulating real hardware step pulses).
    if (stepScheduler_) {
        stepScheduler_->setStepCallback([this](uint8_t oid, int8_t dir) {
            auto it = steppers_.find(oid);
            if (it == steppers_.end()) return;
            it->second->step(dir);
        });
        stepScheduler_->start();
    }

    // queue_step oid=%c interval=%u count=%hu add=%hi
    onCommand("queue_step oid=%c interval=%u count=%hu add=%hi",
        [this](const std::vector<protocol::ParamValue>& params) {
            if (params.size() < 4) return;
            uint8_t oid = static_cast<uint8_t>(params[0].integer);
            auto it = steppers_.find(oid);
            if (it == steppers_.end()) return;
            objects::StepCommand cmd;
            cmd.interval = static_cast<uint32_t>(params[1].integer);
            cmd.count    = static_cast<uint16_t>(params[2].integer);
            cmd.add      = static_cast<int16_t>(params[3].integer);
            cmd.dir      = it->second->direction();
            uint32_t start = stepperBaseClocks_.count(oid) ? stepperBaseClocks_[oid] : 0;
            it->second->enqueueStep(cmd, start);
            // Also forward to the real-time StepScheduler (if enabled).
            if (stepScheduler_) {
                stepScheduler_->schedule(oid, cmd, start);
            }
            // Advance the base clock by the total duration of this command so
            // the next queue_step continues seamlessly. With `add`, the
            // interval changes each step: duration = count*interval +
            // add*(count-1)*count/2.
            uint64_t dur = static_cast<uint64_t>(cmd.interval) * cmd.count;
            if (cmd.count > 1) {
                dur += static_cast<uint64_t>(static_cast<int64_t>(cmd.add)) *
                       (cmd.count - 1) * cmd.count / 2;
            }
            stepperBaseClocks_[oid] = static_cast<uint32_t>(
                static_cast<uint64_t>(start) + dur);
        });

    // set_next_step_dir oid=%c dir=%c
    onCommand("set_next_step_dir oid=%c dir=%c",
        [this](const std::vector<protocol::ParamValue>& params) {
            if (params.size() < 2) return;
            uint8_t oid = static_cast<uint8_t>(params[0].integer);
            auto it = steppers_.find(oid);
            if (it == steppers_.end()) return;
            int8_t dir = (params[1].integer == 0) ? -1 : 1;
            it->second->setDirection(dir);
        });

    // reset_step_clock oid=%c clock=%u
    onCommand("reset_step_clock oid=%c clock=%u",
        [this](const std::vector<protocol::ParamValue>& params) {
            if (params.size() < 2) return;
            uint8_t oid = static_cast<uint8_t>(params[0].integer);
            stepperBaseClocks_[oid] = static_cast<uint32_t>(params[1].integer);
        });
}

size_t KlipperDevice::tickStepScheduler() {
    if (!stepScheduler_) return 0;
    return stepScheduler_->tick();
}

void KlipperDevice::onCommand(const std::string& formatStr, protocol::CommandHandler handler) {
    auto msgidOpt = dict_.lookupCommand(formatStr);
    if (!msgidOpt || !commandTable_) return;
    commandTable_->registerCommand(*msgidOpt, std::move(handler));
}

bool KlipperDevice::sendResponse(const std::string& formatStr,
                                   const std::vector<protocol::ParamValue>& params) {
    auto msgidOpt = dict_.lookupResponse(formatStr);
    if (!msgidOpt) return false;
    std::vector<uint8_t> content;
    if (!protocol::encodeMessage(dict_, *msgidOpt, params, content)) return false;
    auto blockBytes = protocol::buildBlockVec(lastRecvSeq_, content);
    transport_->write(blockBytes);
    return true;
}

} // namespace tether::klipper::device
