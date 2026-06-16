/**
 * @file PCAPLogger.cpp
 * @brief PCAP packet logger adapter implementation
 */

#include "packetloggers/pcap/PCAPLogger.hpp"
#include "packetloggers/pcap/PCAPWriter.hpp"

#include <chrono>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

PCAPLogger::PCAPLogger()
    : writer_(createPCAPWriter())
{}

PCAPLogger::~PCAPLogger() = default;

Error PCAPLogger::init(const PCAPLoggerConfig& config) {
    config_ = config;

    if (!writer_->open(config.filename)) {
        return Error::PermissionDenied;
    }

    // Set section header / application info
    SectionHeaderInfo sectionInfo;
    sectionInfo.application = "Tether PCAP Logger";
    writer_->setSectionHeader(sectionInfo);

    // Add the interface description
    InterfaceInfo iface;
    iface.name = config.interfaceName;
    iface.description = config.interfaceDescription;
    iface.linkType = 1;  // LINKTYPE_ETHERNET
    iface.snapLength = 65535;
    writer_->addInterface(iface);

    frameCount_ = 0;
    totalBytes_ = 0;
    droppedFrames_ = 0;

    return Error::OK;
}

void PCAPLogger::close() {
    writer_->close();
}

bool PCAPLogger::isOpen() const {
    return writer_->isOpen();
}

Error PCAPLogger::logFrame(const uint8_t* frame, size_t length,
                            FrameDirection direction,
                            uint64_t timestamp) {
    if (!config_.logTx && direction == FrameDirection::Tx) {
        return Error::OK;
    }
    if (!config_.logRx && direction == FrameDirection::Rx) {
        return Error::OK;
    }

    PacketMetadata metadata;
    metadata.timestampNs = timestamp > 0 ? timestamp * 1000 : getPCAPTimestampNs();
    metadata.capturedLength = static_cast<uint32_t>(length);
    metadata.originalLength = static_cast<uint32_t>(length);
    metadata.direction = (direction == FrameDirection::Rx)
                         ? PacketDirection::Inbound
                         : PacketDirection::Outbound;
    metadata.interfaceId = 0;

    if (!writer_->logPacket(frame, length, metadata)) {
        droppedFrames_++;
        return Error::InternalError;
    }

    frameCount_++;
    totalBytes_ += length;

    return Error::OK;
}

void PCAPLogger::flush() {
    writer_->flush();
}

uint64_t PCAPLogger::getFrameCount() const {
    return frameCount_;
}

size_t PCAPLogger::getFileSize() const {
    // The underlying writer does not currently expose file size directly.
    return 0;
}

Stats PCAPLogger::getStats() const {
    Stats stats;
    // We do not track direction separately at this adapter level, so count
    // everything according to the last known direction when relevant.
    stats.totalBytes = totalBytes_;
    stats.droppedFrames = droppedFrames_;
    return stats;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::shared_ptr<PacketLogger> createPCAPLogger() {
    return std::make_shared<PCAPLogger>();
}

std::shared_ptr<PacketLogger> createPCAPLogger(const PCAPLoggerConfig& config) {
    auto logger = std::make_shared<PCAPLogger>();
    logger->init(config);
    return logger;
}

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
