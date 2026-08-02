/**
 * @file PCAPNGReader.cpp
 * @brief Full-featured PCAPNG reader implementation
 */

#include "packetloggers/pcap/PCAPNGReader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

namespace {

inline uint16_t read16_be(const uint8_t* p) {
    return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

inline uint32_t read32_be(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

constexpr uint32_t kPcapngByteOrderMagic = 0x1A2B3C4D;
constexpr uint32_t kPcapngByteOrderMagicSwapped = 0x4D3C2B1A;

constexpr uint16_t kEtherTypeVlan = 0x8100;
constexpr uint16_t kEtherTypeVlan8021ad = 0x88A8;  // Service-tag (QinQ outer)
constexpr uint16_t kEtherTypeIPv4 = 0x0800;
constexpr uint16_t kEtherTypeIPv6 = 0x86DD;
constexpr uint8_t  kIpProtocolUDP = 0x11;
constexpr uint16_t kEtherCATOverUdpPort = 0x88A4; // 34980

// LINKTYPE_* constants relevant for dispatch.
constexpr uint16_t kLinkTypeNull       = 0;   // BSD loopback
constexpr uint16_t kLinkTypeEthernet   = 1;   // LINKTYPE_ETHERNET
constexpr uint16_t kLinkTypeRaw        = 101; // LINKTYPE_RAW (old)
constexpr uint16_t kLinkTypeLinuxSll   = 113; // LINKTYPE_LINUX_SLL
constexpr uint16_t kLinkTypeRaw228     = 228; // LINKTYPE_RAW (new)

// PCAPNG block types already declared in PCAPWriter.hpp, but keep local
// constexprs for clarity in switch statements.
constexpr uint32_t kBlockTypeShb = 0x0A0D0D0A;
constexpr uint32_t kBlockTypeIdb = 0x00000001;
constexpr uint32_t kBlockTypePb  = 0x00000002; // obsolete Packet Block
constexpr uint32_t kBlockTypeEpb = 0x00000006;
constexpr uint32_t kBlockTypeSpb = 0x00000003;
constexpr uint32_t kBlockTypeNrb = 0x00000004;
constexpr uint32_t kBlockTypeIsb = 0x00000005;
constexpr uint32_t kBlockTypeDsb = 0x0000000A;
constexpr uint32_t kBlockTypeCb1 = 0x00000BAD;
constexpr uint32_t kBlockTypeCb2 = 0x40000BAD;

// ISB option codes.
constexpr uint16_t kIsbStarttime     = 2;
constexpr uint16_t kIsbEndtime       = 3;
constexpr uint16_t kIsbIfrecv        = 4;
constexpr uint16_t kIsbIfdrop        = 5;
constexpr uint16_t kIsbFilteraccept  = 6;
constexpr uint16_t kIsbFilterdrop    = 7;
constexpr uint16_t kIsbOsdrop        = 8;
constexpr uint16_t kIsbUsrdeliv      = 9;

// NRB record type codes.
constexpr uint16_t kNrbEndRecord = 0;
constexpr uint16_t kNrbIpv4      = 1;
constexpr uint16_t kNrbIpv6      = 2;

// Simple byte-swap helpers that are independent of host endianness.
inline uint16_t swap16(uint16_t v) {
    return static_cast<uint16_t>(((v & 0xFF00u) >> 8) | ((v & 0x00FFu) << 8));
}

inline uint32_t swap32(uint32_t v) {
    return ((v & 0xFF000000u) >> 24) |
           ((v & 0x00FF0000u) >> 8) |
           ((v & 0x0000FF00u) << 8) |
           ((v & 0x000000FFu) << 24);
}

inline uint64_t swap64(uint64_t v) {
    return ((v & 0xFF00000000000000ull) >> 56) |
           ((v & 0x00FF000000000000ull) >> 40) |
           ((v & 0x0000FF0000000000ull) >> 24) |
           ((v & 0x000000FF00000000ull) >> 8) |
           ((v & 0x00000000FF000000ull) << 8) |
           ((v & 0x0000000000FF0000ull) << 24) |
           ((v & 0x000000000000FF00ull) << 40) |
           ((v & 0x00000000000000FFull) << 56);
}

inline uint16_t read16_le(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

inline uint32_t read32_le(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline uint64_t read64_le(const uint8_t* p) {
    return static_cast<uint64_t>(read32_le(p)) |
           (static_cast<uint64_t>(read32_le(p + 4)) << 32);
}

inline std::string jsonEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

} // anonymous namespace

// ============================================================================
// PCAPNGReader
// ============================================================================

PCAPNGReader::PCAPNGReader() = default;

PCAPNGReader::~PCAPNGReader() {
    close();
}

bool PCAPNGReader::open(const std::string& path) {
    close();

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return false;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (size < 0 || static_cast<size_t>(size) < 12) {
        return false;
    }

    buffer_.resize(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char*>(buffer_.data()), size)) {
        buffer_.clear();
        return false;
    }

    ownsBuffer_ = true;
    isOpen_ = true;
    fileSize_ = buffer_.size();
    return true;
}

bool PCAPNGReader::open(const std::vector<uint8_t>& data) {
    close();

    if (data.size() < 12) {
        return false;
    }

    buffer_ = data;
    ownsBuffer_ = true;
    isOpen_ = true;
    fileSize_ = buffer_.size();
    return true;
}

void PCAPNGReader::close() {
    buffer_.clear();
    ownsBuffer_ = false;
    isOpen_ = false;
    fileSize_ = 0;
    currentOffset_ = 0;
    haveSectionHeader_ = false;
    section_ = PCAPNGSectionInfo{};
    interfaces_.clear();
    interfaceStats_.clear();
    nameRecords_.clear();
    decryptionSecrets_.clear();
}

bool PCAPNGReader::isOpen() const {
    return isOpen_;
}

bool PCAPNGReader::readAll(PacketCallback cb) {
    if (!isOpen_) {
        return false;
    }
    return parseBuffer(cb);
}

std::vector<InterpretedFrame> PCAPNGReader::readAll() {
    std::vector<InterpretedFrame> result;
    if (!isOpen_) {
        return result;
    }
    readAll([&result](const InterpretedFrame& frame) {
        result.push_back(frame);
    });
    return result;
}

// ============================================================================
// Private parsing
// ============================================================================

uint16_t PCAPNGReader::read16(const uint8_t* p) const {
    return maybeSwap16(read16_le(p));
}

uint32_t PCAPNGReader::read32(const uint8_t* p) const {
    return maybeSwap32(read32_le(p));
}

uint64_t PCAPNGReader::read64(const uint8_t* p) const {
    return maybeSwap64(read64_le(p));
}

uint16_t PCAPNGReader::maybeSwap16(uint16_t v) const {
    return section_.byteOrderSwapped ? swap16(v) : v;
}

uint32_t PCAPNGReader::maybeSwap32(uint32_t v) const {
    return section_.byteOrderSwapped ? swap32(v) : v;
}

uint64_t PCAPNGReader::maybeSwap64(uint64_t v) const {
    return section_.byteOrderSwapped ? swap64(v) : v;
}

uint32_t PCAPNGReader::padTo32(uint32_t length) {
    return (length + 3u) & ~3u;
}

bool PCAPNGReader::processBlock(size_t offset, const BlockHeader& header,
                                PacketCallback cb) {
    switch (header.type) {
        case kBlockTypeShb:
            return parseSectionHeaderBlock(offset, header);
        case kBlockTypeIdb:
            return parseInterfaceDescriptionBlock(offset, header);
        case kBlockTypePb:
            return parsePacketBlock(offset, header, cb);
        case kBlockTypeEpb:
            return parseEnhancedPacketBlock(offset, header, cb);
        case kBlockTypeSpb:
            return parseSimplePacketBlock(offset, header, cb);
        case kBlockTypeIsb:
            return parseInterfaceStatisticsBlock(offset, header);
        case kBlockTypeNrb:
            return parseNameResolutionBlock(offset, header);
        case kBlockTypeDsb:
            return parseDecryptionSecretsBlock(offset, header);
        case kBlockTypeCb1:
        case kBlockTypeCb2:
        default:
            return skipBlock(offset, header);
    }
}

bool PCAPNGReader::parseBuffer(PacketCallback cb) {
    currentOffset_ = 0;
    haveSectionHeader_ = false;
    interfaces_.clear();
    interfaceStats_.clear();
    nameRecords_.clear();
    decryptionSecrets_.clear();
    section_ = PCAPNGSectionInfo{};
    skippedBlockCount_ = 0;

    while (currentOffset_ < fileSize_) {
        BlockHeader header;
        if (!readBlockHeader(currentOffset_, header)) {
            if (recoveryMode_) {
                size_t bad = currentOffset_;
                currentOffset_ = findNextBlockOffset(currentOffset_ + 4);
                ++skippedBlockCount_;
                if (errorCallback_) errorCallback_(bad, "invalid block header");
                continue;
            }
            return false;
        }

        if (!processBlock(currentOffset_, header, cb)) {
            if (recoveryMode_) {
                size_t bad = currentOffset_;
                currentOffset_ = findNextBlockOffset(currentOffset_ + header.totalLength);
                ++skippedBlockCount_;
                if (errorCallback_) errorCallback_(bad, "block parse error");
                continue;
            }
            return false;
        }

        currentOffset_ += header.totalLength;
    }

    return true;
}

void PCAPNGReader::reset() {
    currentOffset_ = 0;
    haveSectionHeader_ = false;
    interfaces_.clear();
    interfaceStats_.clear();
    nameRecords_.clear();
    decryptionSecrets_.clear();
    section_ = PCAPNGSectionInfo{};
    skippedBlockCount_ = 0;
}

void PCAPNGReader::setRecoveryMode(bool enabled, ErrorCallback cb) {
    recoveryMode_ = enabled;
    errorCallback_ = std::move(cb);
}

size_t PCAPNGReader::findNextBlockOffset(size_t from) const {
    // Scan forward in 4-byte increments for a plausible block header.
    // A plausible header has a known block type and a total length that
    // fits within the remaining file.
    for (size_t off = from; off + 12 <= fileSize_; off += 4) {
        BlockHeader hdr;
        if (readBlockHeader(off, hdr)) {
            return off;
        }
    }
    return fileSize_;
}

bool PCAPNGReader::readNext(InterpretedFrame& out) {
    if (!isOpen_) {
        return false;
    }

    // If this is the first call, initialise state (like parseBuffer does).
    if (currentOffset_ == 0 && !haveSectionHeader_) {
        reset();
    }

    while (currentOffset_ < fileSize_) {
        BlockHeader header;
        if (!readBlockHeader(currentOffset_, header)) {
            if (recoveryMode_) {
                size_t bad = currentOffset_;
                currentOffset_ = findNextBlockOffset(currentOffset_ + 4);
                ++skippedBlockCount_;
                if (errorCallback_) errorCallback_(bad, "invalid block header");
                continue;
            }
            return false;
        }

        bool gotFrame = false;
        bool ok = processBlock(currentOffset_, header,
            [&out, &gotFrame](const InterpretedFrame& frame) {
                out = frame;
                gotFrame = true;
            });

        currentOffset_ += header.totalLength;

        if (!ok) {
            if (recoveryMode_) {
                size_t bad = currentOffset_ - header.totalLength;
                ++skippedBlockCount_;
                if (errorCallback_) errorCallback_(bad, "block parse error");
                // currentOffset_ already advanced past the bad block; continue.
                continue;
            }
            return false;
        }

        if (gotFrame) {
            return true;
        }
    }

    return false; // EOF
}

bool PCAPNGReader::readBlockHeader(size_t offset, BlockHeader& header) const {
    if (offset + 8 > fileSize_) {
        return false;
    }
    // Block type is always in the byte order of the section.  However, at the
    // very first block (SHB) we do not yet know the byte order.  The SHB type
    // value 0x0A0D0D0A is symmetric, so reading it raw works either way.
    uint32_t rawType = read32_le(buffer_.data() + offset);

    // For total length we need to know the byte order.  If we have not parsed
    // the SHB yet, peek at the byte-order magic inside the SHB body.
    bool swap = section_.byteOrderSwapped;
    if (!haveSectionHeader_ && rawType == kBlockTypeShb && offset + 12 <= fileSize_) {
        uint32_t magic = read32_le(buffer_.data() + offset + 8);
        if (magic == kPcapngByteOrderMagic) {
            swap = false;
        } else if (magic == kPcapngByteOrderMagicSwapped) {
            swap = true;
        } else {
            return false;
        }
    }

    header.type = swap ? swap32(rawType) : rawType;

    uint32_t rawLen = read32_le(buffer_.data() + offset + 4);
    header.totalLength = swap ? swap32(rawLen) : rawLen;

    if (header.totalLength < 12 || header.totalLength > fileSize_ - offset) {
        return false;
    }
    return true;
}

bool PCAPNGReader::parseSectionHeaderBlock(size_t offset, const BlockHeader& header) {
    if (offset + 28 > fileSize_) {
        return false;
    }

    uint32_t magic = read32_le(buffer_.data() + offset + 8);
    if (magic == kPcapngByteOrderMagic) {
        section_.byteOrderSwapped = false;
    } else if (magic == kPcapngByteOrderMagicSwapped) {
        section_.byteOrderSwapped = true;
    } else {
        return false;
    }
    haveSectionHeader_ = true;

    // A new Section Header Block starts a fresh section: reset interface
    // state so subsequent EPBs do not resolve timestamps against stale IDBs
    // from the previous section.
    if (!interfaces_.empty()) {
        interfaces_.clear();
    }
    section_.sectionLength = -1;
    section_.hardware.clear();
    section_.os.clear();
    section_.application.clear();
    section_.comment.clear();

    // Major/minor version
    [[maybe_unused]] uint16_t majorVersion = read16(buffer_.data() + offset + 12);
    [[maybe_unused]] uint16_t minorVersion = read16(buffer_.data() + offset + 14);

    // Section length
    section_.sectionLength = static_cast<int64_t>(read64(buffer_.data() + offset + 16));

    // Options start at offset + 24 and run until end-of-block minus trailing length
    size_t optionsOffset = offset + 24;
    size_t optionsEnd = offset + header.totalLength - 4;
    if (optionsEnd < optionsOffset) {
        return true; // no options
    }

    return parseOptions(optionsOffset, optionsEnd - optionsOffset,
        [this](uint16_t code, const uint8_t* data, uint16_t len) -> bool {
            (void)len;
            switch (code) {
                case PCAPNG::SHB_HARDWARE:
                    section_.hardware.assign(reinterpret_cast<const char*>(data), len);
                    break;
                case PCAPNG::SHB_OS:
                    section_.os.assign(reinterpret_cast<const char*>(data), len);
                    break;
                case PCAPNG::SHB_USERAPPL:
                    section_.application.assign(reinterpret_cast<const char*>(data), len);
                    break;
                case PCAPNG::OPT_COMMENT:
                    section_.comment.assign(reinterpret_cast<const char*>(data), len);
                    break;
                default:
                    break;
            }
            return true;
        });
}

bool PCAPNGReader::parseInterfaceDescriptionBlock(size_t offset, const BlockHeader& header) {
    if (offset + 16 > fileSize_) {
        return false;
    }

    PCAPNGInterfaceInfo iface;
    iface.id = static_cast<uint32_t>(interfaces_.size());
    iface.linkType = read16(buffer_.data() + offset + 8);
    // reserved at offset + 10
    iface.snapLen = read32(buffer_.data() + offset + 12);

    size_t optionsOffset = offset + 16;
    size_t optionsEnd = offset + header.totalLength - 4;

    if (optionsEnd > optionsOffset) {
        if (!parseOptions(optionsOffset, optionsEnd - optionsOffset,
            [&iface, this](uint16_t code, const uint8_t* data, uint16_t len) -> bool {
                switch (code) {
                    case PCAPNG::IDB_NAME:
                        iface.name.assign(reinterpret_cast<const char*>(data), len);
                        break;
                    case PCAPNG::IDB_DESCRIPTION:
                        iface.description.assign(reinterpret_cast<const char*>(data), len);
                        break;
                    case PCAPNG::IDB_IPV4ADDR:
                        break; // not interpreted
                    case PCAPNG::IDB_IPV6ADDR:
                        break;
                    case PCAPNG::IDB_MACADDR:
                        if (len >= 6) {
                            std::memcpy(iface.macAddress.data(), data, 6);
                        }
                        break;
                    case PCAPNG::IDB_EUIADDR:
                        break;
                    case PCAPNG::IDB_SPEED:
                        if (len >= 8) {
                            iface.speed = this->read64(data);
                        }
                        break;
                    case PCAPNG::IDB_TSRESOL:
                        if (len >= 1) {
                            iface.tsResol = data[0];
                        }
                        break;
                    case PCAPNG::IDB_TZONE:
                        break;
                    case PCAPNG::IDB_FILTER:
                        if (len > 0) {
                            // First byte is filter type (0 = string), rest is the filter.
                            if (data[0] == 0 && len > 1) {
                                iface.filter.assign(reinterpret_cast<const char*>(data + 1), len - 1);
                            }
                        }
                        break;
                    case PCAPNG::IDB_OS:
                        iface.os.assign(reinterpret_cast<const char*>(data), len);
                        break;
                    case PCAPNG::IDB_FCSLEN:
                        if (len >= 1) {
                            iface.fcsLen = data[0];
                        }
                        break;
                    case PCAPNG::IDB_TSOFFSET:
                        if (len >= 8) {
                            iface.tsOffset = static_cast<int64_t>(this->read64(data));
                        }
                        break;
                    case PCAPNG::OPT_COMMENT:
                        iface.comment.assign(reinterpret_cast<const char*>(data), len);
                        break;
                    default:
                        break;
                }
                return true;
            })) {
            return false;
        }
    }

    interfaces_.push_back(iface);
    return true;
}

bool PCAPNGReader::parseEnhancedPacketBlock(size_t offset, const BlockHeader& header,
                                            PacketCallback cb) {
    if (offset + 32 > fileSize_) {
        return false;
    }

    InterpretedFrame frame;
    frame.interfaceId = read32(buffer_.data() + offset + 8);
    uint32_t tsHigh = read32(buffer_.data() + offset + 12);
    uint32_t tsLow = read32(buffer_.data() + offset + 16);
    uint64_t rawTs = (static_cast<uint64_t>(tsHigh) << 32) | tsLow;
    frame.capturedLength = read32(buffer_.data() + offset + 20);
    frame.originalLength = read32(buffer_.data() + offset + 24);

    // Convert timestamp to nanoseconds using interface resolution.
    uint8_t tsResol = 9;
    int64_t tsOffset = 0;
    if (frame.interfaceId < interfaces_.size()) {
        tsResol = interfaces_[frame.interfaceId].tsResol;
        tsOffset = interfaces_[frame.interfaceId].tsOffset;
    }

    int64_t signedRawTs = static_cast<int64_t>(rawTs) + tsOffset;
    if (signedRawTs < 0) {
        signedRawTs = 0;
    }

    // Convert resolution to nanoseconds per tick.
    uint64_t nsPerTick = 1;
    if ((tsResol & 0x80) == 0) {
        // Power of ten: tsResol = n means tick = 10^-n seconds.
        uint8_t n = tsResol;
        if (n <= 9) {
            for (uint8_t i = 0; i < 9 - n; ++i) {
                nsPerTick *= 10;
            }
            frame.timestampNs = static_cast<uint64_t>(signedRawTs) * nsPerTick;
        } else {
            uint64_t div = 1;
            for (uint8_t i = 0; i < n - 9; ++i) {
                div *= 10;
            }
            frame.timestampNs = static_cast<uint64_t>(signedRawTs) / div;
        }
    } else {
        // Power of two: lower 7 bits = n, tick = 2^-n seconds.
        uint8_t n = tsResol & 0x7F;
        // Approximate 10^9 / 2^n using integer division.
        uint64_t pow2n = 1ull << n; // safe for n < 64
        frame.timestampNs = static_cast<uint64_t>(signedRawTs) * 1000000000ull / pow2n;
    }

    size_t dataOffset = offset + 28;
    size_t paddedLength = padTo32(frame.capturedLength);
    if (dataOffset + paddedLength > offset + header.totalLength - 4) {
        return false;
    }

    if (frame.capturedLength > 0) {
        frame.frameData.assign(buffer_.data() + dataOffset,
                               buffer_.data() + dataOffset + frame.capturedLength);
        // Strip trailing FCS bytes declared by the interface (if_fcslen).
        frame.fcsLength = interfaceFcsLen(frame.interfaceId);
        if (frame.fcsLength > 0 && frame.frameData.size() > frame.fcsLength) {
            frame.frameData.resize(frame.frameData.size() - frame.fcsLength);
        }
        interpretFrameByLinkType(frame.frameData.data(), frame.frameData.size(), frame);
    }

    // Parse options.
    size_t optionsOffset = dataOffset + paddedLength;
    size_t optionsEnd = offset + header.totalLength - 4;
    if (optionsEnd > optionsOffset) {
        if (!parseOptions(optionsOffset, optionsEnd - optionsOffset,
            [&frame, this](uint16_t code, const uint8_t* data, uint16_t len) -> bool {
                switch (code) {
                    case PCAPNG::EPB_FLAGS:
                        if (len >= 4) {
                            frame.packetFlags = this->read32(data);
                            uint32_t dir = frame.packetFlags & 0x03;
                            if (dir == 0x01) {
                                frame.direction = PacketDirection::Inbound;
                            } else if (dir == 0x02) {
                                frame.direction = PacketDirection::Outbound;
                            } else if (dir == 0x03) {
                                frame.direction = PacketDirection::Loopback;
                            } else {
                                frame.direction = PacketDirection::Unknown;
                            }
                        }
                        break;
                    case PCAPNG::EPB_HASH:
                        if (len >= 1) {
                            InterpretedFrame::PacketHash h;
                            h.type = data[0];
                            h.data.assign(data + 1, data + len);
                            frame.hash = std::move(h);
                        }
                        break;
                    case PCAPNG::EPB_DROPCOUNT:
                        if (len >= 8) {
                            frame.dropCount = this->read64(data);
                        }
                        break;
                    case PCAPNG::EPB_PACKETID:
                        if (len >= 8) {
                            frame.packetId = this->read64(data);
                        }
                        break;
                    case PCAPNG::EPB_QUEUE:
                        if (len >= 4) {
                            frame.queue = this->read32(data);
                        }
                        break;
                    case PCAPNG::EPB_VERDICT:
                        if (len >= 2) {
                            InterpretedFrame::PacketVerdict v;
                            v.type = this->read16(data);
                            v.text.assign(reinterpret_cast<const char*>(data + 2), len - 2);
                            frame.verdict = std::move(v);
                        }
                        break;
                    case PCAPNG::EPB_ETHERCAT_SLAVE:
                        if (len >= 2) {
                            frame.slaveAddress = this->read16(data);
                        }
                        break;
                    case PCAPNG::EPB_ETHERCAT_PDO:
                        if (len >= 1) {
                            frame.isProcessData = data[0] != 0;
                        }
                        break;
                    case PCAPNG::EPB_ETHERCAT_WC:
                        if (len >= 1) {
                            frame.workingCounter = data[0];
                        }
                        break;
                    case PCAPNG::OPT_COMMENT:
                        frame.comment.assign(reinterpret_cast<const char*>(data), len);
                        break;
                    default:
                        break;
                }
                return true;
            })) {
            return false;
        }
    }

    if (cb) {
        cb(frame);
    }
    return true;
}

bool PCAPNGReader::parsePacketBlock(size_t offset, const BlockHeader& header,
                                    PacketCallback cb) {
    // Obsolete Packet Block (type 0x00000002) layout:
    //   0: block type (4)
    //   4: total length (4)
    //   8: interface ID (2)
    //  10: drops (2)
    //  12: timestamp seconds (4)
    //  16: captured length (4)
    //  20: original length (4)
    //  24: packet data (padded to 32-bit)
    //     options
    //     total length (4)
    if (offset + 24 > fileSize_) {
        return false;
    }

    InterpretedFrame frame;
    frame.interfaceId = read16(buffer_.data() + offset + 8);
    frame.dropCount = read16(buffer_.data() + offset + 10);
    uint32_t tsSeconds = read32(buffer_.data() + offset + 12);
    frame.capturedLength = read32(buffer_.data() + offset + 16);
    frame.originalLength = read32(buffer_.data() + offset + 20);

    // PB timestamps are in seconds; convert to nanoseconds.
    frame.timestampNs = static_cast<uint64_t>(tsSeconds) * 1000000000ull;

    size_t dataOffset = offset + 24;
    size_t paddedLength = padTo32(frame.capturedLength);
    if (dataOffset + paddedLength > offset + header.totalLength - 4) {
        return false;
    }

    if (frame.capturedLength > 0) {
        frame.frameData.assign(buffer_.data() + dataOffset,
                               buffer_.data() + dataOffset + frame.capturedLength);
        frame.fcsLength = interfaceFcsLen(frame.interfaceId);
        if (frame.fcsLength > 0 && frame.frameData.size() > frame.fcsLength) {
            frame.frameData.resize(frame.frameData.size() - frame.fcsLength);
        }
        interpretFrameByLinkType(frame.frameData.data(), frame.frameData.size(), frame);
    }

    // Parse options (same option set as EPB).
    size_t optionsOffset = dataOffset + paddedLength;
    size_t optionsEnd = offset + header.totalLength - 4;
    if (optionsEnd > optionsOffset) {
        if (!parseOptions(optionsOffset, optionsEnd - optionsOffset,
            [&frame, this](uint16_t code, const uint8_t* data, uint16_t len) -> bool {
                switch (code) {
                    case PCAPNG::EPB_FLAGS:
                        if (len >= 4) {
                            frame.packetFlags = this->read32(data);
                            uint32_t dir = frame.packetFlags & 0x03;
                            if (dir == 0x01) frame.direction = PacketDirection::Inbound;
                            else if (dir == 0x02) frame.direction = PacketDirection::Outbound;
                            else if (dir == 0x03) frame.direction = PacketDirection::Loopback;
                            else frame.direction = PacketDirection::Unknown;
                        }
                        break;
                    case PCAPNG::EPB_DROPCOUNT:
                        if (len >= 8) frame.dropCount = this->read64(data);
                        break;
                    case PCAPNG::OPT_COMMENT:
                        frame.comment.assign(reinterpret_cast<const char*>(data), len);
                        break;
                    default:
                        break;
                }
                return true;
            })) {
            return false;
        }
    }

    if (cb) {
        cb(frame);
    }
    return true;
}

bool PCAPNGReader::parseSimplePacketBlock(size_t offset, const BlockHeader& header,
                                          PacketCallback cb) {
    if (offset + 12 > fileSize_) {
        return false;
    }

    InterpretedFrame frame;
    frame.interfaceId = 0;
    frame.timestampNs = 0;
    frame.originalLength = read32(buffer_.data() + offset + 8);
    uint32_t dataPaddedLength = header.totalLength - 16; // minus header, original length, trailing length
    frame.capturedLength = std::min(frame.originalLength, dataPaddedLength);

    size_t dataOffset = offset + 12;
    size_t paddedLength = padTo32(frame.capturedLength);
    if (dataOffset + paddedLength > offset + header.totalLength - 4) {
        return false;
    }

    if (frame.capturedLength > 0) {
        frame.frameData.assign(buffer_.data() + dataOffset,
                               buffer_.data() + dataOffset + frame.capturedLength);
        // Strip trailing FCS bytes declared by the interface (if_fcslen).
        frame.fcsLength = interfaceFcsLen(frame.interfaceId);
        if (frame.fcsLength > 0 && frame.frameData.size() > frame.fcsLength) {
            frame.frameData.resize(frame.frameData.size() - frame.fcsLength);
        }
        interpretFrameByLinkType(frame.frameData.data(), frame.frameData.size(), frame);
    }

    if (cb) {
        cb(frame);
    }
    return true;
}

bool PCAPNGReader::parseInterfaceStatisticsBlock(size_t offset, const BlockHeader& header) {
    // ISB body: interface ID (4), timestamp high (4), timestamp low (4) = 12 bytes.
    if (offset + 20 > fileSize_) {
        return false;
    }

    PCAPNGInterfaceStats stats;
    stats.interfaceId = read32(buffer_.data() + offset + 8);
    uint32_t tsHigh = read32(buffer_.data() + offset + 12);
    uint32_t tsLow = read32(buffer_.data() + offset + 16);
    stats.timestampNs = (static_cast<uint64_t>(tsHigh) << 32) | tsLow;

    size_t optionsOffset = offset + 20;
    size_t optionsEnd = offset + header.totalLength - 4;
    if (optionsEnd > optionsOffset) {
        if (!parseOptions(optionsOffset, optionsEnd - optionsOffset,
            [&stats, this](uint16_t code, const uint8_t* data, uint16_t len) -> bool {
                switch (code) {
                    case kIsbStarttime:    if (len >= 8) stats.startTime = this->read64(data); break;
                    case kIsbEndtime:      if (len >= 8) stats.endTime = this->read64(data); break;
                    case kIsbIfrecv:       if (len >= 8) stats.ifRecv = this->read64(data); break;
                    case kIsbIfdrop:       if (len >= 8) stats.ifDrop = this->read64(data); break;
                    case kIsbFilteraccept: if (len >= 8) stats.filterAccept = this->read64(data); break;
                    case kIsbFilterdrop:   if (len >= 8) stats.filterDrop = this->read64(data); break;
                    case kIsbOsdrop:       if (len >= 8) stats.osDrop = this->read64(data); break;
                    case kIsbUsrdeliv:     if (len >= 8) stats.usrDeliv = this->read64(data); break;
                    case PCAPNG::OPT_COMMENT:
                        stats.comment.assign(reinterpret_cast<const char*>(data), len);
                        break;
                    default: break;
                }
                return true;
            })) {
            return false;
        }
    }

    interfaceStats_.push_back(std::move(stats));
    return true;
}

bool PCAPNGReader::parseNameResolutionBlock(size_t offset, const BlockHeader& header) {
    // NRB has no fixed body; it is a sequence of records followed by options.
    // Records use the same TLV format as options but with their own type codes.
    // We scan records until nrb_end_record, then the rest are options.
    size_t pos = offset + 8;
    size_t end = offset + header.totalLength - 4;

    while (pos + 4 <= end) {
        uint16_t recType = read16(buffer_.data() + pos);
        uint16_t recLen = read16(buffer_.data() + pos + 2);
        size_t paddedLen = padTo32(recLen);

        if (recType == kNrbEndRecord && recLen == 0) {
            // Remaining bytes (if any) are options.
            size_t optionsOffset = pos + 4;
            if (optionsOffset < end) {
                parseOptions(optionsOffset, end - optionsOffset,
                    [this](uint16_t code, const uint8_t* data, uint16_t len) -> bool {
                        if (code == PCAPNG::OPT_COMMENT) {
                            PCAPNGNameResolutionRecord rec;
                            rec.type = PCAPNGNameResolutionRecord::Type::Comment;
                            rec.name.assign(reinterpret_cast<const char*>(data), len);
                            nameRecords_.push_back(std::move(rec));
                        }
                        return true;
                    });
            }
            return true;
        }

        if (pos + 4 + paddedLen > end) {
            break; // truncated
        }

        const uint8_t* recData = buffer_.data() + pos + 4;
        PCAPNGNameResolutionRecord rec;
        if (recType == kNrbIpv4 && recLen >= 4) {
            rec.type = PCAPNGNameResolutionRecord::Type::Ipv4;
            std::memcpy(rec.ipv4.data(), recData, 4);
            rec.name.assign(reinterpret_cast<const char*>(recData + 4), recLen - 4);
            nameRecords_.push_back(std::move(rec));
        } else if (recType == kNrbIpv6 && recLen >= 16) {
            rec.type = PCAPNGNameResolutionRecord::Type::Ipv6;
            std::memcpy(rec.ipv6.data(), recData, 16);
            rec.name.assign(reinterpret_cast<const char*>(recData + 16), recLen - 16);
            nameRecords_.push_back(std::move(rec));
        }

        pos += 4 + paddedLen;
    }

    return true;
}

bool PCAPNGReader::parseDecryptionSecretsBlock(size_t offset, const BlockHeader& header) {
    // DSB body: secrets type (2), secrets length (2), secrets data (padded to 4).
    if (offset + 16 > fileSize_) {
        return false;
    }

    PCAPNGDecryptionSecrets dsb;
    dsb.secretsType = read16(buffer_.data() + offset + 8);
    uint16_t secretsLen = read16(buffer_.data() + offset + 10);
    size_t paddedSecretsLen = padTo32(secretsLen);

    size_t dataStart = offset + 12;
    if (dataStart + paddedSecretsLen > offset + header.totalLength - 4) {
        return false;
    }

    dsb.secretsData.assign(buffer_.data() + dataStart,
                           buffer_.data() + dataStart + secretsLen);

    // Options follow the padded secrets data.
    size_t optionsOffset = dataStart + paddedSecretsLen;
    size_t optionsEnd = offset + header.totalLength - 4;
    if (optionsEnd > optionsOffset) {
        parseOptions(optionsOffset, optionsEnd - optionsOffset,
            [&dsb](uint16_t code, const uint8_t* data, uint16_t len) -> bool {
                if (code == PCAPNG::OPT_COMMENT) {
                    dsb.comment.assign(reinterpret_cast<const char*>(data), len);
                }
                return true;
            });
    }

    decryptionSecrets_.push_back(std::move(dsb));
    return true;
}

bool PCAPNGReader::skipBlock(size_t, const BlockHeader&) {
    return true;
}

bool PCAPNGReader::parseOptions(size_t offset, size_t length,
                                std::function<bool(uint16_t code, const uint8_t* data, uint16_t len)> handler) {
    size_t pos = 0;
    while (pos + 4 <= length) {
        uint16_t code = read16(buffer_.data() + offset + pos);
        uint16_t optLen = read16(buffer_.data() + offset + pos + 2);
        size_t paddedLen = padTo32(optLen);

        if (code == PCAPNG::OPT_ENDOFOPT && optLen == 0) {
            return true;
        }

        if (pos + 4 + paddedLen > length) {
            return false;
        }

        if (!handler(code, buffer_.data() + offset + pos + 4, optLen)) {
            return false;
        }

        pos += 4 + paddedLen;
    }

    return true;
}

uint8_t PCAPNGReader::interfaceFcsLen(uint32_t interfaceId) const {
    if (interfaceId < interfaces_.size()) {
        return interfaces_[interfaceId].fcsLen;
    }
    return 0;
}

uint16_t PCAPNGReader::interfaceLinkType(uint32_t interfaceId) const {
    if (interfaceId < interfaces_.size()) {
        return interfaces_[interfaceId].linkType;
    }
    return kLinkTypeEthernet;
}

void PCAPNGReader::interpretPayload(uint16_t etherType, const uint8_t* data,
                                    size_t length, size_t payloadOffset,
                                    InterpretedFrame& frame) const {
    frame.innerEtherType = etherType;

    // Direct EtherCAT via EtherType 0x88A4.
    if (etherType == EtherCAT::kEtherTypeEtherCAT) {
        frame.isEtherCAT = true;
        parseEtherCATDatagrams(data, length, payloadOffset, frame);
        return;
    }

    // EtherCAT-over-UDP encapsulation: IPv4 or IPv6 -> UDP ->
    // dst port 34980 (0x88A4) -> EtherCAT frame as UDP payload.
    if (etherType == kEtherTypeIPv4 || etherType == kEtherTypeIPv6) {
        parseEtherCATOverUDP(data, length, payloadOffset, frame);
    }
}

bool PCAPNGReader::interpretEthernetFrame(const uint8_t* data, size_t length,
                                          InterpretedFrame& frame) const {
    if (length < sizeof(EtherCAT::EthernetHeader)) {
        return false;
    }

    EtherCAT::EthernetHeader ethHdr;
    std::memcpy(&ethHdr, data, sizeof(ethHdr));
    std::memcpy(frame.dstMac.data(), ethHdr.dst, 6);
    std::memcpy(frame.srcMac.data(), ethHdr.src, 6);

    uint16_t etherType = read16_be(reinterpret_cast<const uint8_t*>(&ethHdr.etherType_be));
    size_t payloadOffset = sizeof(EtherCAT::EthernetHeader);

    // Handle 802.1Q / 802.1ad VLAN tag(s).  The outer EtherType field already
    // held the TPID, so payloadOffset points at the TCI word.
    while ((etherType == kEtherTypeVlan || etherType == kEtherTypeVlan8021ad) &&
           payloadOffset + 4 <= length) {
        uint16_t tci = read16_be(data + payloadOffset);
        uint16_t inner = read16_be(data + payloadOffset + 2);
        frame.vlanId = tci & 0x0FFF;
        frame.vlanPcp = static_cast<uint8_t>((tci >> 13) & 0x07);
        frame.vlanDei = ((tci >> 12) & 0x01) != 0;
        etherType = inner;
        payloadOffset += 4;
    }

    interpretPayload(etherType, data, length, payloadOffset, frame);
    return true; // parsed but not necessarily EtherCAT
}

void PCAPNGReader::interpretFrameByLinkType(const uint8_t* data, size_t length,
                                            InterpretedFrame& frame) const {
    frame.linkType = interfaceLinkType(frame.interfaceId);
    switch (frame.linkType) {
        case kLinkTypeEthernet:
            interpretEthernetFrame(data, length, frame);
            break;
        case kLinkTypeLinuxSll:
            interpretLinuxSllFrame(data, length, frame);
            break;
        case kLinkTypeRaw:
        case kLinkTypeRaw228:
            interpretRawIpFrame(data, length, frame);
            break;
        case kLinkTypeNull:
            interpretNullFrame(data, length, frame);
            break;
        default:
            // Unknown link type: leave frameData as-is, do not interpret.
            break;
    }
}

void PCAPNGReader::interpretLinuxSllFrame(const uint8_t* data, size_t length,
                                          InterpretedFrame& frame) const {
    // SLL header (16 bytes):
    //   0: packet type (2 bytes, BE)
    //   2: ARPHRD type (2 bytes, BE)
    //   4: address length (2 bytes, BE)
    //   6: address (8 bytes)
    //  14: protocol / EtherType (2 bytes, BE)
    if (length < 16) return;

    uint16_t etherType = read16_be(data + 14);
    // The 8-byte address field holds the source MAC (padded/truncated).
    std::memcpy(frame.srcMac.data(), data + 6, 6);
    interpretPayload(etherType, data, length, 16, frame);
}

void PCAPNGReader::interpretRawIpFrame(const uint8_t* data, size_t length,
                                       InterpretedFrame& frame) const {
    if (length < 1) return;
    uint8_t version = (data[0] >> 4) & 0x0F;
    if (version == 4) {
        frame.innerEtherType = kEtherTypeIPv4;
        parseEtherCATOverUDP(data, length, 0, frame);
    } else if (version == 6) {
        frame.innerEtherType = kEtherTypeIPv6;
        parseEtherCATOverUDP(data, length, 0, frame);
    }
}

void PCAPNGReader::interpretNullFrame(const uint8_t* data, size_t length,
                                      InterpretedFrame& frame) const {
    // BSD loopback header: 4-byte address family (host byte order on the
    // capturing machine; on a little-endian host AF_INET=2, AF_INET6=30).
    if (length < 4) return;
    uint32_t family = read32_le(data);
    if (family == 2 || family == 0x02000000u) {
        frame.innerEtherType = kEtherTypeIPv4;
        parseEtherCATOverUDP(data, length, 4, frame);
    } else if (family == 30 || family == 0x1E000000u ||
               family == 24 || family == 0x18000000u) { // AF_INET6 on BSDs varies
        frame.innerEtherType = kEtherTypeIPv6;
        parseEtherCATOverUDP(data, length, 4, frame);
    }
}

void PCAPNGReader::parseEtherCATOverUDP(const uint8_t* data, size_t length,
                                        size_t ipOffset,
                                        InterpretedFrame& frame) const {
    // Minimum IPv4 header is 20 bytes.
    if (ipOffset + 20 > length) return;

    const uint8_t* ip = data + ipOffset;
    uint8_t versionIhl = ip[0];
    if ((versionIhl >> 4) != 4) return; // not IPv4

    uint8_t ihl = (versionIhl & 0x0F) * 4;
    if (ihl < 20 || ipOffset + ihl > length) return;

    uint8_t protocol = ip[9];
    if (protocol != kIpProtocolUDP) return; // not UDP

    // IPv4 src/dst addresses (offset 12 and 16, network byte order).
    frame.srcIp = read32_be(ip + 12);
    frame.dstIp = read32_be(ip + 16);

    size_t udpOffset = ipOffset + ihl;
    if (udpOffset + 8 > length) return;

    const uint8_t* udp = data + udpOffset;
    frame.srcPort = read16_be(udp + 0);
    frame.dstPort = read16_be(udp + 2);
    uint16_t udpLen = read16_be(udp + 4);
    if (udpLen < 8) return;

    // EtherCAT-over-UDP uses destination port 34980 (0x88A4).
    if (frame.dstPort != kEtherCATOverUdpPort) return;

    size_t ecatOffset = udpOffset + 8;
    size_t ecatAvail = length - ecatOffset;
    // Clamp to UDP length minus header.
    size_t udpPayloadLen = udpLen - 8;
    if (udpPayloadLen > ecatAvail) udpPayloadLen = ecatAvail;
    if (udpPayloadLen < sizeof(EtherCAT::FrameHeader)) return;

    frame.isEtherCAT = true;
    frame.isEtherCATOverUDP = true;
    parseEtherCATDatagrams(data, ecatOffset + udpPayloadLen, ecatOffset, frame);
}

void PCAPNGReader::parseEtherCATDatagrams(const uint8_t* data, size_t dataEnd,
                                          size_t ecatOffset,
                                          InterpretedFrame& frame) const {
    if (ecatOffset + sizeof(EtherCAT::FrameHeader) > dataEnd) {
        return;
    }

    const uint8_t* ecatHdrBytes = data + ecatOffset;
    frame.ecatFrameLength = static_cast<uint16_t>(ecatHdrBytes[0]) |
                            static_cast<uint16_t>((ecatHdrBytes[1] & 0x07) << 8);
    frame.ecatFrameType = static_cast<uint8_t>((ecatHdrBytes[1] >> 4) & 0x0F);

    size_t datagramOffset = ecatOffset + sizeof(EtherCAT::FrameHeader);
    size_t remaining = frame.ecatFrameLength;

    while (remaining >= sizeof(EtherCAT::DatagramHeader) + sizeof(uint16_t)) {
        if (datagramOffset + sizeof(EtherCAT::DatagramHeader) > dataEnd) {
            break;
        }

        const uint8_t* dg = data + datagramOffset;
        EtherCATDatagramInfo info;
        info.cmd = static_cast<EtherCAT::Command>(dg[0]);
        info.idx = dg[1];
        info.adp = read16_le(dg + 2);
        info.ado = read16_le(dg + 4);
        uint16_t lenFlags = read16_le(dg + 6);
        info.dataLength = lenFlags & 0x07FF;
        info.more = (lenFlags & 0x8000) != 0;
        info.circulating = (lenFlags & 0x4000) != 0;
        info.irq = read16_le(dg + 8);

        size_t dgTotalSize = sizeof(EtherCAT::DatagramHeader) + info.dataLength + sizeof(uint16_t);
        if (datagramOffset + dgTotalSize > dataEnd || dgTotalSize > remaining) {
            break;
        }

        const uint8_t* payload = dg + sizeof(EtherCAT::DatagramHeader);
        info.data.assign(payload, payload + info.dataLength);
        info.wkc = read16_le(payload + info.dataLength);

        frame.datagrams.push_back(std::move(info));

        datagramOffset += dgTotalSize;
        remaining -= dgTotalSize;
    }
}

// ============================================================================
// Formatting helpers
// ============================================================================

std::string macToString(const std::array<uint8_t, 6>& mac) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < mac.size(); ++i) {
        if (i > 0) oss << ':';
        oss << std::setw(2) << static_cast<int>(mac[i]);
    }
    return oss.str();
}

std::string ipToString(uint32_t ip) {
    std::ostringstream oss;
    oss << ((ip >> 24) & 0xFF) << "."
        << ((ip >> 16) & 0xFF) << "."
        << ((ip >> 8) & 0xFF) << "."
        << (ip & 0xFF);
    return oss.str();
}

std::string bytesToHex(const uint8_t* data, size_t length, const std::string& separator) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < length; ++i) {
        if (i > 0) oss << separator;
        oss << std::setw(2) << static_cast<int>(data[i]);
    }
    return oss.str();
}

static const char* directionString(PacketDirection dir) {
    switch (dir) {
        case PacketDirection::Inbound: return "RX";
        case PacketDirection::Outbound: return "TX";
        case PacketDirection::Loopback: return "LOOPBACK";
        default: return "?";
    }
}

std::string formatInterpretedFrame(const InterpretedFrame& frame, bool verbose, size_t maxDataBytes) {
    std::ostringstream oss;
    oss << "[" << directionString(frame.direction) << "] "
        << "ts=" << frame.timestampNs << " ns"
        << "  iface=" << frame.interfaceId
        << "  cap=" << frame.capturedLength
        << "  orig=" << frame.originalLength;
    if (frame.dropCount > 0) {
        oss << "  drops=" << frame.dropCount;
    }
    if (!frame.comment.empty()) {
        oss << "  comment=\"" << frame.comment << "\"";
    }
    oss << "\n";

    oss << "  Dst: " << macToString(frame.dstMac)
        << "  Src: " << macToString(frame.srcMac);
    if (frame.vlanId.has_value()) {
        oss << "  VLAN: " << *frame.vlanId
            << " PCP: " << static_cast<int>(frame.vlanPcp)
            << " DEI: " << (frame.vlanDei ? "1" : "0");
    }
    oss << "  EtherType: 0x" << std::hex << std::setfill('0') << std::setw(4) << frame.innerEtherType
        << std::dec << "\n";

    if (frame.isEtherCATOverUDP) {
        oss << "  EtherCAT-over-UDP  "
            << ipToString(frame.srcIp) << ":" << frame.srcPort
            << " -> " << ipToString(frame.dstIp) << ":" << frame.dstPort
            << "\n";
    }

    if (frame.isEtherCAT) {
        oss << "  EtherCAT frame  length=" << frame.ecatFrameLength
            << "  type=" << static_cast<int>(frame.ecatFrameType);
        if (frame.slaveAddress != 0) {
            oss << "  slave=" << frame.slaveAddress;
        }
        if (frame.isProcessData) {
            oss << "  PDO";
        }
        if (frame.workingCounter != 0) {
            oss << "  wc=" << static_cast<int>(frame.workingCounter);
        }
        oss << "\n";

        for (size_t i = 0; i < frame.datagrams.size(); ++i) {
            const auto& dg = frame.datagrams[i];
            oss << "    DG" << i << ": "
                << EtherCAT::commandToString(dg.cmd)
                << " idx=" << static_cast<int>(dg.idx);

            switch (dg.cmd) {
                case EtherCAT::Command::LRD:
                case EtherCAT::Command::LWR:
                case EtherCAT::Command::LRW:
                    oss << " logAddr=0x" << std::hex << std::setfill('0') << std::setw(8)
                        << dg.logicalAddress() << std::dec;
                    break;
                default:
                    oss << " adp=" << dg.adp << " ado=0x" << std::hex << std::setfill('0')
                        << std::setw(4) << dg.ado << std::dec;
                    break;
            }

            oss << " len=" << dg.dataLength
                << " wkc=" << dg.wkc;
            if (dg.irq != 0) {
                oss << " irq=0x" << std::hex << std::setfill('0')
                    << std::setw(4) << dg.irq << std::dec;
            }
            oss << (dg.more ? " [M]" : "")
                << (dg.circulating ? " [C]" : "")
                << "\n";

            if (verbose && !dg.data.empty()) {
                size_t dumpLen = maxDataBytes > 0 ? std::min(dg.data.size(), maxDataBytes) : dg.data.size();
                oss << "      Data: " << bytesToHex(dg.data.data(), dumpLen);
                if (maxDataBytes > 0 && dg.data.size() > maxDataBytes) {
                    oss << " ... (" << (dg.data.size() - maxDataBytes) << " more bytes)";
                }
                oss << "\n";
            }
        }
    }

    return oss.str();
}

std::string frameToJson(const InterpretedFrame& frame) {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"timestampNs\": " << frame.timestampNs << ",\n";
    oss << "  \"interfaceId\": " << frame.interfaceId << ",\n";
    oss << "  \"capturedLength\": " << frame.capturedLength << ",\n";
    oss << "  \"originalLength\": " << frame.originalLength << ",\n";
    oss << "  \"direction\": \"" << directionString(frame.direction) << "\",\n";
    oss << "  \"packetFlags\": " << frame.packetFlags << ",\n";
    oss << "  \"dropCount\": " << frame.dropCount << ",\n";
    oss << "  \"comment\": \"" << jsonEscape(frame.comment) << "\",\n";
    oss << "  \"dstMac\": \"" << macToString(frame.dstMac) << "\",\n";
    oss << "  \"srcMac\": \"" << macToString(frame.srcMac) << "\",\n";
    if (frame.vlanId.has_value()) {
        oss << "  \"vlanId\": " << *frame.vlanId << ",\n";
        oss << "  \"vlanPcp\": " << static_cast<int>(frame.vlanPcp) << ",\n";
        oss << "  \"vlanDei\": " << (frame.vlanDei ? "true" : "false") << ",\n";
    }
    oss << "  \"innerEtherType\": " << frame.innerEtherType << ",\n";
    oss << "  \"isEtherCAT\": " << (frame.isEtherCAT ? "true" : "false") << ",\n";
    oss << "  \"isEtherCATOverUDP\": " << (frame.isEtherCATOverUDP ? "true" : "false") << ",\n";
    if (frame.isEtherCATOverUDP) {
        oss << "  \"srcIp\": \"" << ipToString(frame.srcIp) << "\",\n";
        oss << "  \"dstIp\": \"" << ipToString(frame.dstIp) << "\",\n";
        oss << "  \"srcPort\": " << frame.srcPort << ",\n";
        oss << "  \"dstPort\": " << frame.dstPort << ",\n";
    }
    if (frame.isEtherCAT) {
        oss << "  \"ecatFrameLength\": " << frame.ecatFrameLength << ",\n";
        oss << "  \"ecatFrameType\": " << static_cast<int>(frame.ecatFrameType) << ",\n";
        oss << "  \"slaveAddress\": " << frame.slaveAddress << ",\n";
        oss << "  \"isProcessData\": " << (frame.isProcessData ? "true" : "false") << ",\n";
        oss << "  \"workingCounter\": " << static_cast<int>(frame.workingCounter) << ",\n";
        oss << "  \"datagrams\": [\n";
        for (size_t i = 0; i < frame.datagrams.size(); ++i) {
            const auto& dg = frame.datagrams[i];
            oss << "    {\n";
            oss << "      \"cmd\": \"" << EtherCAT::commandToString(dg.cmd) << "\",\n";
            oss << "      \"idx\": " << static_cast<int>(dg.idx) << ",\n";
            oss << "      \"adp\": " << dg.adp << ",\n";
            oss << "      \"ado\": " << dg.ado << ",\n";
            oss << "      \"dataLength\": " << dg.dataLength << ",\n";
            oss << "      \"irq\": " << dg.irq << ",\n";
            oss << "      \"wkc\": " << dg.wkc << ",\n";
            oss << "      \"more\": " << (dg.more ? "true" : "false") << ",\n";
            oss << "      \"circulating\": " << (dg.circulating ? "true" : "false") << ",\n";
            oss << "      \"data\": \"" << bytesToHex(dg.data.data(), dg.data.size(), "") << "\"\n";
            oss << "    }";
            if (i + 1 < frame.datagrams.size()) oss << ",";
            oss << "\n";
        }
        oss << "  ]\n";
    } else {
        oss << "  \"frameData\": \"" << bytesToHex(frame.frameData.data(), frame.frameData.size(), "") << "\"\n";
    }
    oss << "}\n";
    return oss.str();
}

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
