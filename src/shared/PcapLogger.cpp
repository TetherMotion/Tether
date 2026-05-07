/**
 * @file PcapLogger.cpp
 * @brief Shared PcapNG logging implementation for EtherCAT Master and Slave
 */

#include "shared/PcapLogger.hpp"

#include <chrono>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <iomanip>

#ifdef __linux__
#include <sys/utsname.h>
#endif

namespace EtherCAT {

// ============================================================================
// PcapNgLogger Implementation
// ============================================================================

PcapNgLogger::PcapNgLogger() {
    writeBuffer_.reserve(bufferSize_);
    
    // Set default section info
    sectionInfo_.application = "EtherCAT Logger";
    sectionInfo_.hardware = "Unknown";
    
#ifdef __linux__
    struct utsname uname_data;
    if (uname(&uname_data) == 0) {
        sectionInfo_.os = std::string(uname_data.sysname) + " " + uname_data.release;
    }
#else
    sectionInfo_.os = "Unknown OS";
#endif
}

PcapNgLogger::~PcapNgLogger() {
    close();
}

bool PcapNgLogger::open(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (file_.is_open()) {
        file_.close();
    }
    
    filename_ = filename;
    file_.open(filename, std::ios::binary | std::ios::trunc);
    
    if (!file_.is_open()) {
        return false;
    }
    
    // Write Section Header Block
    writeSectionHeaderBlock();
    
    // Write Interface Description Blocks for any pre-registered interfaces
    for (const auto& iface : interfaces_) {
        writeInterfaceDescriptionBlock(iface);
    }
    
    packetCount_ = 0;
    byteCount_ = 0;
    dropCount_ = 0;
    
    return true;
}

void PcapNgLogger::close() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (file_.is_open()) {
        // Flush any buffered data
        if (!writeBuffer_.empty()) {
            file_.write(reinterpret_cast<const char*>(writeBuffer_.data()), writeBuffer_.size());
            writeBuffer_.clear();
        }
        file_.close();
    }
}

bool PcapNgLogger::isOpen() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return file_.is_open();
}

bool PcapNgLogger::flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!file_.is_open()) {
        return false;
    }
    
    if (!writeBuffer_.empty()) {
        file_.write(reinterpret_cast<const char*>(writeBuffer_.data()), writeBuffer_.size());
        writeBuffer_.clear();
    }
    
    file_.flush();
    return true;
}

void PcapNgLogger::setSectionHeader(const SectionHeaderInfo& info) {
    std::lock_guard<std::mutex> lock(mutex_);
    sectionInfo_ = info;
}

uint8_t PcapNgLogger::addInterface(const InterfaceInfo& info) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    uint8_t interfaceId = static_cast<uint8_t>(interfaces_.size());
    interfaces_.push_back(info);
    
    if (file_.is_open()) {
        writeInterfaceDescriptionBlock(info);
    }
    
    return interfaceId;
}

bool PcapNgLogger::logPacket(const uint8_t* data, size_t length,
                            const PacketMetadata& metadata) {
    if (!enabled_) {
        return true;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!file_.is_open()) {
        dropCount_++;
        return false;
    }
    
    // Check file size for rotation
    if (rotation_ && byteCount_ > maxFileSize_) {
        rotateFile();
    }
    
    writeEnhancedPacketBlock(data, length, metadata);
    
    packetCount_++;
    byteCount_ += length;
    
    return true;
}

bool PcapNgLogger::logPacket(const uint8_t* data, size_t length,
                            PacketDirection direction, uint8_t interfaceId) {
    PacketMetadata metadata;
    metadata.timestampNs = getCurrentTimestampNs();
    metadata.capturedLength = length;
    metadata.originalLength = length;
    metadata.direction = direction;
    metadata.interfaceId = interfaceId;
    
    return logPacket(data, length, metadata);
}

void PcapNgLogger::setBufferSize(size_t bytes) {
    std::lock_guard<std::mutex> lock(mutex_);
    bufferSize_ = bytes;
    writeBuffer_.reserve(bufferSize_);
}

size_t PcapNgLogger::getBufferUsage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return writeBuffer_.size();
}

// ============================================================================
// Private Implementation
// ============================================================================

void PcapNgLogger::writeSectionHeaderBlock() {
    std::vector<uint8_t> content;
    content.reserve(256);
    
    // Byte order magic
    uint32_t magic = PcapNg::BYTE_ORDER_MAGIC;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&magic),
                  reinterpret_cast<uint8_t*>(&magic) + 4);
    
    // Version
    uint16_t versionMajor = PcapNg::VERSION_MAJOR;
    uint16_t versionMinor = PcapNg::VERSION_MINOR;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&versionMajor),
                  reinterpret_cast<uint8_t*>(&versionMajor) + 2);
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&versionMinor),
                  reinterpret_cast<uint8_t*>(&versionMinor) + 2);
    
    // Section length (-1 = unknown)
    int64_t sectionLength = -1;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&sectionLength),
                  reinterpret_cast<uint8_t*>(&sectionLength) + 8);
    
    // Options
    std::vector<uint8_t> options;
    
    if (!sectionInfo_.hardware.empty()) {
        writeOptionString(options, PcapNg::SHB_HARDWARE, sectionInfo_.hardware);
    }
    if (!sectionInfo_.os.empty()) {
        writeOptionString(options, PcapNg::SHB_OS, sectionInfo_.os);
    }
    if (!sectionInfo_.application.empty()) {
        writeOptionString(options, PcapNg::SHB_USERAPPL, sectionInfo_.application);
    }
    if (!sectionInfo_.comment.empty()) {
        writeOptionString(options, PcapNg::OPT_COMMENT, sectionInfo_.comment);
    }
    writeEndOfOptions(options);
    
    content.insert(content.end(), options.begin(), options.end());
    
    writeBlock(PcapNg::BLOCK_TYPE_SHB, content);
}

void PcapNgLogger::writeInterfaceDescriptionBlock(const InterfaceInfo& info) {
    std::vector<uint8_t> content;
    content.reserve(256);
    
    // Link type
    uint16_t linkType = info.linkType;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&linkType),
                  reinterpret_cast<uint8_t*>(&linkType) + 2);
    
    // Reserved
    uint16_t reserved = 0;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&reserved),
                  reinterpret_cast<uint8_t*>(&reserved) + 2);
    
    // Snap length
    uint32_t snapLen = info.snapLength;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&snapLen),
                  reinterpret_cast<uint8_t*>(&snapLen) + 4);
    
    // Options
    std::vector<uint8_t> options;
    
    if (!info.name.empty()) {
        writeOptionString(options, PcapNg::IDB_NAME, info.name);
    }
    if (!info.description.empty()) {
        writeOptionString(options, PcapNg::IDB_DESCRIPTION, info.description);
    }
    
    // MAC address
    bool hasMac = false;
    for (auto b : info.macAddress) {
        if (b != 0) { hasMac = true; break; }
    }
    if (hasMac) {
        writeOption(options, PcapNg::IDB_MACADDR, info.macAddress.data(), 6);
    }
    
    // Speed
    if (info.speed > 0) {
        uint64_t speed = info.speed;
        writeOption(options, PcapNg::IDB_SPEED, &speed, 8);
    }
    
    // Timestamp resolution (10^-9 = nanoseconds)
    uint8_t tsResol = 9;  // 10^-9 seconds
    writeOption(options, PcapNg::IDB_TSRESOL, &tsResol, 1);
    
    // Filter
    if (!info.filter.empty()) {
        // Filter string format: 1 byte filter type + string
        std::vector<uint8_t> filterData;
        filterData.push_back(0);  // String filter
        filterData.insert(filterData.end(), info.filter.begin(), info.filter.end());
        writeOption(options, PcapNg::IDB_FILTER, filterData.data(), filterData.size());
    }
    
    writeEndOfOptions(options);
    content.insert(content.end(), options.begin(), options.end());
    
    writeBlock(PcapNg::BLOCK_TYPE_IDB, content);
}

void PcapNgLogger::writeEnhancedPacketBlock(const uint8_t* data, size_t length,
                                            const PacketMetadata& metadata) {
    std::vector<uint8_t> content;
    content.reserve(length + 128);
    
    // Interface ID
    uint32_t interfaceId = metadata.interfaceId;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&interfaceId),
                  reinterpret_cast<uint8_t*>(&interfaceId) + 4);
    
    // Timestamp
    uint64_t timestamp = metadata.timestampNs;
    if (timestamp == 0) {
        timestamp = getCurrentTimestampNs();
    }
    
    uint32_t tsHigh = timestamp >> 32;
    uint32_t tsLow = timestamp & 0xFFFFFFFF;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&tsHigh),
                  reinterpret_cast<uint8_t*>(&tsHigh) + 4);
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&tsLow),
                  reinterpret_cast<uint8_t*>(&tsLow) + 4);
    
    // Captured length
    uint32_t capturedLen = metadata.capturedLength > 0 ? metadata.capturedLength : length;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&capturedLen),
                  reinterpret_cast<uint8_t*>(&capturedLen) + 4);
    
    // Original length
    uint32_t originalLen = metadata.originalLength > 0 ? metadata.originalLength : length;
    content.insert(content.end(), reinterpret_cast<uint8_t*>(&originalLen),
                  reinterpret_cast<uint8_t*>(&originalLen) + 4);
    
    // Packet data
    content.insert(content.end(), data, data + capturedLen);
    
    // Pad to 32-bit boundary
    uint32_t paddedLen = padTo32(capturedLen);
    while (content.size() < 20 + paddedLen) {
        content.push_back(0);
    }
    
    // Options
    std::vector<uint8_t> options;
    
    // Packet flags
    if (metadata.hasFlags || metadata.direction != PacketDirection::Unknown) {
        uint32_t flags = metadata.packetFlags;
        
        // Set direction in flags (bits 0-1)
        flags &= ~0x03;
        switch (metadata.direction) {
            case PacketDirection::Inbound:
                flags |= 0x01;
                break;
            case PacketDirection::Outbound:
                flags |= 0x02;
                break;
            case PacketDirection::Loopback:
                flags |= 0x03;
                break;
            default:
                break;
        }
        
        writeOption(options, PcapNg::EPB_FLAGS, &flags, 4);
    }
    
    // Hash
    if (metadata.hasHash) {
        struct {
            uint8_t algorithm;
            uint8_t padding[3];
            uint32_t hash;
        } hashData = {2, {0, 0, 0}, metadata.packetHash};  // 2 = CRC32
        writeOption(options, PcapNg::EPB_HASH, &hashData, sizeof(hashData));
    }
    
    // Drop count
    if (metadata.hasDropCount && metadata.dropCount > 0) {
        writeOption(options, PcapNg::EPB_DROPCOUNT, &metadata.dropCount, 8);
    }
    
    // EtherCAT-specific options (custom)
    if (metadata.slaveAddress != 0) {
        writeOption(options, PcapNg::EPB_ETHERCAT_SLAVE, &metadata.slaveAddress, 2);
    }
    if (metadata.isProcessData) {
        uint8_t pdo = 1;
        writeOption(options, PcapNg::EPB_ETHERCAT_PDO, &pdo, 1);
    }
    if (metadata.workingCounter != 0) {
        writeOption(options, PcapNg::EPB_ETHERCAT_WC, &metadata.workingCounter, 1);
    }
    
    writeEndOfOptions(options);
    content.insert(content.end(), options.begin(), options.end());
    
    writeBlock(PcapNg::BLOCK_TYPE_EPB, content);
}

void PcapNgLogger::writeBlock(uint32_t blockType, const std::vector<uint8_t>& content) {
    // Block total length = type(4) + length(4) + content + length(4)
    uint32_t totalLength = 4 + 4 + content.size() + 4;
    totalLength = padTo32(totalLength);
    
    // Write block type
    writeBuffer_.insert(writeBuffer_.end(), reinterpret_cast<uint8_t*>(&blockType),
                       reinterpret_cast<uint8_t*>(&blockType) + 4);
    
    // Write total length
    writeBuffer_.insert(writeBuffer_.end(), reinterpret_cast<uint8_t*>(&totalLength),
                       reinterpret_cast<uint8_t*>(&totalLength) + 4);
    
    // Write content
    writeBuffer_.insert(writeBuffer_.end(), content.begin(), content.end());
    
    // Pad to 32-bit boundary
    while (writeBuffer_.size() % 4 != 0) {
        writeBuffer_.push_back(0);
    }
    
    // Write total length again
    writeBuffer_.insert(writeBuffer_.end(), reinterpret_cast<uint8_t*>(&totalLength),
                       reinterpret_cast<uint8_t*>(&totalLength) + 4);
    
    // Flush if buffer is getting full
    if (writeBuffer_.size() >= bufferSize_ * 3 / 4) {
        file_.write(reinterpret_cast<const char*>(writeBuffer_.data()), writeBuffer_.size());
        writeBuffer_.clear();
    }
}

void PcapNgLogger::writeOption(std::vector<uint8_t>& options, uint16_t code,
                               const void* data, uint16_t length) {
    options.insert(options.end(), reinterpret_cast<uint8_t*>(&code),
                  reinterpret_cast<uint8_t*>(&code) + 2);
    options.insert(options.end(), reinterpret_cast<uint8_t*>(&length),
                  reinterpret_cast<uint8_t*>(&length) + 2);
    
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    options.insert(options.end(), bytes, bytes + length);
    
    // Pad to 32-bit boundary
    while (options.size() % 4 != 0) {
        options.push_back(0);
    }
}

void PcapNgLogger::writeOptionString(std::vector<uint8_t>& options, uint16_t code,
                                     const std::string& str) {
    writeOption(options, code, str.data(), str.length());
}

void PcapNgLogger::writeEndOfOptions(std::vector<uint8_t>& options) {
    uint16_t endCode = PcapNg::OPT_ENDOFOPT;
    uint16_t endLength = 0;
    options.insert(options.end(), reinterpret_cast<uint8_t*>(&endCode),
                  reinterpret_cast<uint8_t*>(&endCode) + 2);
    options.insert(options.end(), reinterpret_cast<uint8_t*>(&endLength),
                  reinterpret_cast<uint8_t*>(&endLength) + 2);
}

uint32_t PcapNgLogger::padTo32(uint32_t length) {
    return (length + 3) & ~3;
}

uint64_t PcapNgLogger::getCurrentTimestampNs() const {
    auto now = std::chrono::high_resolution_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
    return static_cast<uint64_t>(ns);
}

void PcapNgLogger::rotateFile() {
    // Close current file
    if (!writeBuffer_.empty()) {
        file_.write(reinterpret_cast<const char*>(writeBuffer_.data()), writeBuffer_.size());
        writeBuffer_.clear();
    }
    file_.close();
    
    // Rotate file names
    currentRotationIndex_++;
    if (currentRotationIndex_ >= maxRotationFiles_) {
        currentRotationIndex_ = 0;
    }
    
    // Open new file
    std::string rotatedName = getRotatedFilename(currentRotationIndex_);
    file_.open(rotatedName, std::ios::binary | std::ios::trunc);
    
    if (file_.is_open()) {
        byteCount_ = 0;
        writeSectionHeaderBlock();
        for (const auto& iface : interfaces_) {
            writeInterfaceDescriptionBlock(iface);
        }
    }
}

std::string PcapNgLogger::getRotatedFilename(uint32_t index) const {
    // Insert index before extension
    size_t dotPos = filename_.rfind('.');
    if (dotPos != std::string::npos) {
        return filename_.substr(0, dotPos) + "_" + std::to_string(index) + 
               filename_.substr(dotPos);
    }
    return filename_ + "_" + std::to_string(index);
}

// ============================================================================
// Null Logger (discards all packets)
// ============================================================================

class NullPcapLogger : public IPcapLogger {
public:
    bool open(const std::string&) override { return true; }
    void close() override {}
    bool isOpen() const override { return true; }
    bool flush() override { return true; }
    
    void setSectionHeader(const SectionHeaderInfo&) override {}
    uint8_t addInterface(const InterfaceInfo&) override { return 0; }
    
    bool logPacket(const uint8_t*, size_t, const PacketMetadata&) override { return true; }
    bool logPacket(const uint8_t*, size_t, PacketDirection, uint8_t) override { return true; }
    
    uint64_t getPacketCount() const override { return 0; }
    uint64_t getByteCount() const override { return 0; }
    uint64_t getDropCount() const override { return 0; }
    
    void setEnabled(bool) override {}
    bool isEnabled() const override { return false; }
    
    void setBufferSize(size_t) override {}
    size_t getBufferUsage() const override { return 0; }
};

// ============================================================================
// Memory Logger
// ============================================================================

class MemoryPcapLogger : public PcapNgLogger {
public:
    explicit MemoryPcapLogger(size_t maxBytes)
        : maxBytes_(maxBytes)
    {}
    
    bool open(const std::string&) override {
        buffer_.clear();
        buffer_.reserve(maxBytes_);
        return true;
    }
    
    void close() override {
        // Keep data in memory
    }
    
    bool isOpen() const override {
        return true;
    }
    
    const std::vector<uint8_t>& getData() const {
        return buffer_;
    }
    
    void clear() {
        buffer_.clear();
    }
    
private:
    std::vector<uint8_t> buffer_;
    size_t maxBytes_;
};

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<IPcapLogger> createPcapLogger() {
    return std::make_unique<PcapNgLogger>();
}

std::unique_ptr<IPcapLogger> createPcapLogger(const std::string& filename,
                                              const SectionHeaderInfo& sectionInfo) {
    auto logger = std::make_unique<PcapNgLogger>();
    logger->setSectionHeader(sectionInfo);
    logger->open(filename);
    return logger;
}

std::unique_ptr<IPcapLogger> createNullPcapLogger() {
    return std::make_unique<NullPcapLogger>();
}

std::unique_ptr<IPcapLogger> createMemoryPcapLogger(size_t maxBytes) {
    return std::make_unique<MemoryPcapLogger>(maxBytes);
}

// ============================================================================
// Helper Functions
// ============================================================================

uint64_t getPcapTimestampNs() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
}

PacketMetadata extractEtherCATMetadata(const uint8_t* frame, size_t length) {
    PacketMetadata meta;
    meta.timestampNs = getPcapTimestampNs();
    meta.capturedLength = length;
    meta.originalLength = length;
    
    // Check for EtherCAT frame (EtherType 0x88A4)
    if (length >= 14) {
        uint16_t etherType = (frame[12] << 8) | frame[13];
        if (etherType == 0x88A4 && length >= 16) {
            // EtherCAT header
            uint16_t ecatHeader = frame[14] | (frame[15] << 8);
            uint16_t dataLen = ecatHeader & 0x07FF;
            uint8_t type = (ecatHeader >> 11) & 0x0F;
            
            meta.isProcessData = (type == 0x01);  // Type 1 = process data
            
            // Extract working counter if present
            if (length >= 18 + dataLen) {
                // WC is at end of datagram
            }
        }
    }
    
    return meta;
}

}  // namespace EtherCAT
