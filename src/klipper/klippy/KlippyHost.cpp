/**
 * @file KlippyHost.cpp
 * @brief KlippyHost implementation.
 */

#include "tether/klipper/klippy/KlippyHost.hpp"
#include "tether/klipper/protocol/MessageBlock.hpp"
#include "tether/klipper/KlipperLog.hpp"

#include <chrono>
#include <format>
#include <functional>

namespace tether::klipper::klippy {

KlippyHost::KlippyHost(std::shared_ptr<transport::IByteStreamTransport> transport)
    : transport_(std::move(transport)) {
    serialQueue_ = std::make_unique<reliability::SerialQueue>(*transport_);
}

KlippyHost::~KlippyHost() = default;

bool KlippyHost::connect() {
    if (!transport_->open()) {
        KLIPPER_LOG_ERROR("KlippyHost::connect() - transport open failed");
        return false;
    }
    connected_ = true;
    commandTable_ = std::make_unique<protocol::CommandTable>(dict_);
    return true;
}

bool KlippyHost::downloadDictionary(std::function<void()> devicePump) {
    if (!connected_) {
        KLIPPER_LOG_ERROR("KlippyHost::downloadDictionary() - not connected");
        return false;
    }
    protocol::IdentifyClient client;
    int rounds = 0;
    while (!client.complete() && rounds < 10000) {
        auto reqContent = client.buildRequestContent();
        auto seq = serialQueue_->send(reqContent);
        if (!seq) {
            pump();
            if (devicePump) devicePump();
            continue;
        }
        // Pump both sides until we get a response or timeout.
        for (int i = 0; i < 100 && !client.complete(); ++i) {
            if (devicePump) devicePump();
            // Read and parse blocks directly (pump() dispatches to
            // commandTable which isn't set up yet during identify).
            auto rd = transport_->readAll();
            if (rd.empty()) continue;
            size_t off = 0;
            while (off < rd.size()) {
                auto pb = protocol::parseBlock(
                    std::span<const uint8_t>(rd.data() + off, rd.size() - off));
                if (pb.status != protocol::BlockParseStatus::Ok) break;
                if (pb.block.content.empty()) {
                    serialQueue_->processAck(pb.block);
                } else {
                    client.consumeResponseContent(pb.block.content);
                }
                off += pb.consumedBytes;
            }
        }
        ++rounds;
    }
    if (!client.complete()) {
        KLIPPER_LOG_ERROR(std::format("KlippyHost::downloadDictionary() - identify protocol did not complete after {} rounds", rounds));
        return false;
    }
    auto dictOpt = client.decodeDictionary();
    if (!dictOpt) {
        KLIPPER_LOG_ERROR("KlippyHost::downloadDictionary() - failed to decode dictionary");
        return false;
    }
    dict_ = std::move(*dictOpt);
    commandTable_ = std::make_unique<protocol::CommandTable>(dict_);
    dictDownloaded_ = true;
    return true;
}

bool KlippyHost::syncClock(std::function<void()> devicePump) {
    if (!dictDownloaded_) {
        KLIPPER_LOG_ERROR("KlippyHost::syncClock() - dictionary not downloaded");
        return false;
    }
    auto msgidOpt = dict_.lookupCommand("get_clock");
    if (!msgidOpt) {
        KLIPPER_LOG_ERROR("KlippyHost::syncClock() - 'get_clock' not in dictionary");
        return false;
    }
    std::vector<uint8_t> content;
    std::vector<protocol::ParamValue> params;
    if (!protocol::encodeMessage(dict_, *msgidOpt, params, content)) {
        KLIPPER_LOG_ERROR("KlippyHost::syncClock() - failed to encode get_clock");
        return false;
    }
    getClockSendTime_ = clock::HostClock::now();
    getClockPending_ = true;
    auto seq = serialQueue_->send(content);
    if (!seq) {
        KLIPPER_LOG_ERROR("KlippyHost::syncClock() - failed to send get_clock");
        return false;
    }
    for (int i = 0; i < 1000 && getClockPending_; ++i) {
        if (devicePump) devicePump();
        pump();
    }
    return !getClockPending_;
}

uint8_t KlippyHost::allocateOid(const std::string& type) {
    uint8_t oid = oidAllocator_.allocate();
    oidAllocator_.assign(oid, type);
    return oid;
}

bool KlippyHost::sendCommand(const std::string& formatStr,
                              const std::vector<protocol::ParamValue>& params) {
    if (!dictDownloaded_) {
        KLIPPER_LOG_ERROR("KlippyHost::sendCommand() - dictionary not downloaded");
        return false;
    }
    auto msgidOpt = dict_.lookupCommand(formatStr);
    if (!msgidOpt) {
        KLIPPER_LOG_ERROR("KlippyHost::sendCommand() - unknown command: " + formatStr);
        return false;
    }
    std::vector<uint8_t> content;
    if (!protocol::encodeMessage(dict_, *msgidOpt, params, content)) {
        KLIPPER_LOG_ERROR("KlippyHost::sendCommand() - failed to encode: " + formatStr);
        return false;
    }
    auto seq = serialQueue_->send(content);
    if (!seq) {
        KLIPPER_LOG_WARN("KlippyHost::sendCommand() - serial queue full for: " + formatStr);
        return false;
    }
    return true;
}

size_t KlippyHost::sendStepSequence(const motion::AxisStepSequence& seq,
                                     std::function<void()> pump) {
    if (!dictDownloaded_) return 0;
    size_t enqueued = 0;

    // Helper: send a command, pumping the transport when the window is full
    // so acks flow back and free window space.
    auto sendWithPump = [&](const std::string& fmt,
                            const std::vector<protocol::ParamValue>& params) -> bool {
        // If the window is full, pump until there's room (or give up).
        int tries = 0;
        while (!serialQueue_->canSend() && tries < 1000) {
            if (pump) pump();
            pump(); // drain host-side acks
            ++tries;
        }
        return sendCommand(fmt, params);
    };

    // reset_step_clock oid=%c clock=%u  (once, at the sequence start)
    sendWithPump("reset_step_clock oid=%c clock=%u", {
        protocol::ParamValue{static_cast<int32_t>(seq.oid)},
        protocol::ParamValue{static_cast<int32_t>(seq.startClock)},
    });

    int8_t lastDir = -100; // forces an initial set_next_step_dir
    for (const auto& step : seq.steps) {
        // Update direction when it changes.
        int8_t wireDir = (step.dir < 0) ? 0 : 1;
        if (wireDir != lastDir) {
            sendWithPump("set_next_step_dir oid=%c dir=%c", {
                protocol::ParamValue{static_cast<int32_t>(seq.oid)},
                protocol::ParamValue{static_cast<int32_t>(wireDir)},
            });
            lastDir = wireDir;
        }
        // queue_step oid=%c interval=%u count=%hu add=%hi
        bool ok = sendWithPump("queue_step oid=%c interval=%u count=%hu add=%hi", {
            protocol::ParamValue{static_cast<int32_t>(seq.oid)},
            protocol::ParamValue{static_cast<int32_t>(step.interval)},
            protocol::ParamValue{static_cast<int32_t>(step.count)},
            protocol::ParamValue{static_cast<int32_t>(step.add)},
        });
        if (ok) ++enqueued;
    }
    return enqueued;
}

size_t KlippyHost::sendStepSequences(const std::vector<motion::AxisStepSequence>& seqs,
                                     std::function<void()> pump) {
    size_t total = 0;
    for (const auto& s : seqs) total += sendStepSequence(s, pump);
    return total;
}

void KlippyHost::onResponse(const std::string& formatStr, protocol::ResponseHandler handler) {
    auto msgidOpt = dict_.lookupResponse(formatStr);
    if (!msgidOpt || !commandTable_) return;
    commandTable_->registerResponse(*msgidOpt, std::move(handler));
}

void KlippyHost::pump() {
    auto rd = transport_->readAll();
    if (rd.empty()) return;
    size_t off = 0;
    while (off < rd.size()) {
        auto pb = protocol::parseBlock(
            std::span<const uint8_t>(rd.data() + off, rd.size() - off));
        if (pb.status != protocol::BlockParseStatus::Ok) break;
        if (pb.block.content.empty()) {
            serialQueue_->processAck(pb.block);
        } else {
            auto msgs = protocol::decodeMessages(dict_, pb.block.content);
            for (const auto& msg : msgs) {
                if (getClockPending_) {
                    auto clockRespOpt = dict_.lookupResponse("clock clock=%u");
                    if (clockRespOpt && msg.msgid == *clockRespOpt && !msg.params.empty()) {
                        uint32_t mcuClock = static_cast<uint32_t>(msg.params[0].integer);
                        auto recvTime = clock::HostClock::now();
                        clockSync_.addSample(getClockSendTime_, recvTime, mcuClock);
                        getClockPending_ = false;
                    }
                }
                if (commandTable_) commandTable_->dispatchResponse(msg);
            }
        }
        off += pb.consumedBytes;
    }
}

void KlippyHost::checkTimeouts() {
    serialQueue_->checkTimeouts();
}

} // namespace tether::klipper::klippy
