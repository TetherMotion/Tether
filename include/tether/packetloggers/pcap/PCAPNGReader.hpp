/**
 * @file PCAPNGReader.hpp
 * @brief Full-featured PCAPNG reader with EtherCAT frame interpretation
 *
 * Reads PCAPNG capture files, parses Section Header, Interface Description,
 * Enhanced Packet, Simple Packet, Interface Statistics, Name Resolution, and
 * Decryption Secrets blocks, and interprets captured Ethernet frames.  VLAN
 * tags (802.1Q) are stripped automatically, and EtherCAT frames (EtherType
 * 0x88A4) are decoded into individual datagrams.
 */

#pragma once

#include "packetloggers/pcap/PCAPWriter.hpp"
#include "ethercat/Types.hpp"

#include <array>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace Tether {
namespace PacketLoggers {
namespace PCAP {

// ============================================================================
// Section Header Information (read from file)
// ============================================================================

struct PCAPNGSectionInfo {
    bool byteOrderSwapped = false;   ///< True if file byte order differs from host
    int64_t sectionLength = -1;      ///< -1 if unknown
    std::string hardware;
    std::string os;
    std::string application;
    std::string comment;
};

// ============================================================================
// Interface Description Information (read from file)
// ============================================================================

struct PCAPNGInterfaceInfo {
    uint32_t id = 0;                 ///< Interface index (0-based)
    uint16_t linkType = 1;           ///< LINKTYPE_ETHERNET = 1
    uint32_t snapLen = 65535;        ///< Max captured bytes
    uint64_t speed = 0;              ///< Bits/sec, 0 if unknown
    uint8_t tsResol = 9;             ///< 10^-tsResol seconds (default nanoseconds)
    int64_t tsOffset = 0;            ///< Timestamp offset in units of tsResol
    uint8_t fcsLen = 0;              ///< FCS length in bytes
    std::string name;
    std::string description;
    std::string filter;
    std::string os;
    std::array<uint8_t, 6> macAddress{};
    std::string comment;
};

// ============================================================================
// Interface Statistics (from Interface Statistics Block)
// ============================================================================

struct PCAPNGInterfaceStats {
    uint32_t interfaceId = 0;
    uint64_t timestampNs = 0;        ///< Timestamp of the statistics record
    uint64_t startTime = 0;          ///< isb_starttime (raw ticks)
    uint64_t endTime = 0;            ///< isb_endtime (raw ticks)
    uint64_t ifRecv = 0;             ///< isb_ifrecv — packets received
    uint64_t ifDrop = 0;             ///< isb_ifdrop — dropped by interface
    uint64_t filterAccept = 0;       ///< isb_filteraccept
    uint64_t filterDrop = 0;         ///< isb_filterdrop
    uint64_t osDrop = 0;             ///< isb_osdrop — dropped by OS
    uint64_t usrDeliv = 0;           ///< isb_usrdeliv — delivered to user
    std::string comment;
};

// ============================================================================
// Name Resolution Record (from Name Resolution Block)
// ============================================================================

struct PCAPNGNameResolutionRecord {
    enum class Type : uint8_t { Ipv4, Ipv6, Comment };
    Type type = Type::Ipv4;
    std::array<uint8_t, 4> ipv4{};   ///< Valid when type == Ipv4
    std::array<uint8_t, 16> ipv6{};  ///< Valid when type == Ipv6
    std::string name;                ///< Hostname (or comment text)
};

// ============================================================================
// Decryption Secrets (from Decryption Secrets Block)
// ============================================================================

struct PCAPNGDecryptionSecrets {
    uint16_t secretsType = 0;        ///< Secrets type code
    std::vector<uint8_t> secretsData;
    std::string comment;
};

// ============================================================================
// Parsed EtherCAT Datagram
// ============================================================================

struct EtherCATDatagramInfo {
    EtherCAT::Command cmd = EtherCAT::Command::NOP;
    uint8_t idx = 0;
    uint16_t adp = 0;
    uint16_t ado = 0;
    uint16_t dataLength = 0;
    uint16_t irq = 0;              ///< Interrupt request register
    uint16_t wkc = 0;
    bool more = false;
    bool circulating = false;
    std::vector<uint8_t> data;       ///< Copy of datagram payload

    /// @return 32-bit logical address for LRD/LWR/LRW datagrams
    uint32_t logicalAddress() const {
        return (static_cast<uint32_t>(ado) << 16) | adp;
    }
};

// ============================================================================
// Interpreted Ethernet Frame
// ============================================================================

struct InterpretedFrame {
    // Packet-block metadata
    uint64_t timestampNs = 0;
    uint32_t interfaceId = 0;
    uint32_t capturedLength = 0;   ///< Raw on-wire captured length (incl. FCS)
    uint32_t originalLength = 0;   ///< Raw on-wire original length (incl. FCS)
    uint8_t  fcsLength = 0;        ///< FCS bytes stripped from frameData
    PacketDirection direction = PacketDirection::Unknown;
    uint32_t packetFlags = 0;
    uint64_t dropCount = 0;
    std::string comment;

    // EPB optional metadata
    std::optional<uint64_t> packetId;  ///< epb_packetid
    std::optional<uint32_t> queue;     ///< epb_queue
    struct PacketHash {
        uint8_t type = 0;              ///< Hash algorithm (e.g. 1=IPv4, 2=IPv6)
        std::vector<uint8_t> data;
    };
    std::optional<PacketHash> hash;    ///< epb_hash
    struct PacketVerdict {
        uint16_t type = 0;             ///< Verdict type code
        std::string text;              ///< Verdict description
    };
    std::optional<PacketVerdict> verdict;  ///< epb_verdict

    // Custom EtherCAT options stored by the writer
    uint16_t slaveAddress = 0;
    bool isProcessData = false;
    uint8_t workingCounter = 0;

    /// Full captured Ethernet frame (including header, VLAN tag, and payload).
    /// For non-Ethernet link types, this is the link-layer payload after FCS
    /// stripping (e.g. the SLL header + payload, or the raw IP packet).
    std::vector<uint8_t> frameData;

    // Link-layer information
    uint16_t linkType = 1;          ///< LINKTYPE_* of the capturing interface

    // Ethernet header
    std::array<uint8_t, 6> dstMac{};
    std::array<uint8_t, 6> srcMac{};

    // VLAN decapsulation
    std::optional<uint16_t> vlanId;
    uint8_t vlanPcp = 0;
    bool vlanDei = false;

    // Inner protocol
    uint16_t innerEtherType = 0;
    bool isEtherCAT = false;

    // EtherCAT-over-UDP encapsulation (set when EtherCAT is tunneled via UDP
    // port 34980/0x88A4 instead of using EtherType 0x88A4 directly).
    bool isEtherCATOverUDP = false;
    uint32_t srcIp = 0;   ///< IPv4 source address (host byte order)
    uint32_t dstIp = 0;   ///< IPv4 destination address (host byte order)
    uint16_t srcPort = 0; ///< UDP source port (host byte order)
    uint16_t dstPort = 0; ///< UDP destination port (host byte order)

    // EtherCAT frame header (valid when isEtherCAT == true)
    uint16_t ecatFrameLength = 0;
    uint8_t ecatFrameType = 0;

    // Decoded datagrams
    std::vector<EtherCATDatagramInfo> datagrams;

    /// @return true if this frame carried at least one EtherCAT datagram
    bool hasEtherCAT() const { return isEtherCAT && !datagrams.empty(); }
};

// ============================================================================
// PCAPNG Reader
// ============================================================================

class PCAPNGReader {
public:
    PCAPNGReader();
    ~PCAPNGReader();

    /// Open a pcapng file from disk.
    bool open(const std::string& path);

    /// Open a pcapng capture from an in-memory buffer (useful for tests).
    bool open(const std::vector<uint8_t>& data);

    /// Release any open resources.
    void close();

    /// True when a file or memory buffer is loaded.
    bool isOpen() const;

    /// Parsed Section Header Block information.
    const PCAPNGSectionInfo& sectionInfo() const { return section_; }

    /// Parsed Interface Description Blocks.
    const std::vector<PCAPNGInterfaceInfo>& interfaces() const { return interfaces_; }

    /// Parsed Interface Statistics Blocks.
    const std::vector<PCAPNGInterfaceStats>& interfaceStats() const { return interfaceStats_; }

    /// Parsed Name Resolution records (from all NRBs).
    const std::vector<PCAPNGNameResolutionRecord>& nameResolutionRecords() const { return nameRecords_; }

    /// Parsed Decryption Secrets Blocks.
    const std::vector<PCAPNGDecryptionSecrets>& decryptionSecrets() const { return decryptionSecrets_; }

    /// Callback invoked once for every interpreted frame.
    using PacketCallback = std::function<void(const InterpretedFrame&)>;

    /// Read and interpret all packets, invoking @p cb for each one.
    /// @return true if the file was parsed successfully to the end.
    bool readAll(PacketCallback cb);

    /// Convenience overload that collects all interpreted frames.
    std::vector<InterpretedFrame> readAll();

    /// Read the next interpreted frame incrementally.  Processes metadata
    /// blocks (SHB/IDB/ISB/NRB/DSB) as encountered and returns the next
    /// packet block (EPB/PB/SPB) via @p out.
    /// @return true if a frame was produced; false at EOF or on error.
    bool readNext(InterpretedFrame& out);

    /// Reset the read cursor to the beginning of the capture without
    /// reloading the buffer.  Useful for re-iterating with readNext().
    void reset();

    /// Error callback invoked when a malformed block is encountered in
    /// recovery mode.  Receives the block offset and a human-readable message.
    using ErrorCallback = std::function<void(size_t offset, const std::string& msg)>;

    /// Enable or disable recovery mode.  When enabled, malformed blocks are
    /// skipped (the reader scans forward for the next valid block) instead of
    /// aborting the parse.  @p cb is invoked for each skipped block.
    void setRecoveryMode(bool enabled, ErrorCallback cb = nullptr);

    /// @return number of blocks skipped due to errors in recovery mode.
    size_t skippedBlockCount() const { return skippedBlockCount_; }

private:
    struct BlockHeader {
        uint32_t type = 0;
        uint32_t totalLength = 0;
    };

    bool parseBuffer(PacketCallback cb);
    /// Process the block at @p offset, invoking @p cb for packet blocks.
    /// @return true on success.  Does not advance currentOffset_.
    bool processBlock(size_t offset, const BlockHeader& header, PacketCallback cb);
    bool readBlockHeader(size_t offset, BlockHeader& header) const;
    /// Scan forward from @p from for the next plausible block header.
    /// @return offset of the next valid block, or fileSize_ if none found.
    size_t findNextBlockOffset(size_t from) const;
    bool parseSectionHeaderBlock(size_t offset, const BlockHeader& header);
    bool parseInterfaceDescriptionBlock(size_t offset, const BlockHeader& header);
    bool parseEnhancedPacketBlock(size_t offset, const BlockHeader& header,
                                  PacketCallback cb);
    bool parsePacketBlock(size_t offset, const BlockHeader& header,
                          PacketCallback cb);
    bool parseSimplePacketBlock(size_t offset, const BlockHeader& header,
                                PacketCallback cb);
    bool parseInterfaceStatisticsBlock(size_t offset, const BlockHeader& header);
    bool parseNameResolutionBlock(size_t offset, const BlockHeader& header);
    bool parseDecryptionSecretsBlock(size_t offset, const BlockHeader& header);
    bool skipBlock(size_t offset, const BlockHeader& header);

    bool parseOptions(size_t offset, size_t length,
                      std::function<bool(uint16_t code, const uint8_t* data, uint16_t len)> handler);

    bool interpretEthernetFrame(const uint8_t* data, size_t length,
                                InterpretedFrame& frame) const;

    /// Dispatch frame interpretation based on the interface link type.
    void interpretFrameByLinkType(const uint8_t* data, size_t length,
                                  InterpretedFrame& frame) const;

    /// Interpret a Linux cooked-capture (SLL) frame (LINKTYPE_LINUX_SLL=113).
    void interpretLinuxSllFrame(const uint8_t* data, size_t length,
                                InterpretedFrame& frame) const;

    /// Interpret a raw IP frame (LINKTYPE_RAW=101/228, no link-layer header).
    void interpretRawIpFrame(const uint8_t* data, size_t length,
                             InterpretedFrame& frame) const;

    /// Interpret a BSD loopback frame (LINKTYPE_NULL=0): 4-byte family + IP.
    void interpretNullFrame(const uint8_t* data, size_t length,
                            InterpretedFrame& frame) const;

    /// Common post-EtherType payload interpretation (VLAN already stripped).
    void interpretPayload(uint16_t etherType, const uint8_t* data, size_t length,
                          size_t payloadOffset, InterpretedFrame& frame) const;

    /// @return FCS length (bytes) for the given interface, or 0 if unknown.
    uint8_t interfaceFcsLen(uint32_t interfaceId) const;

    /// @return LINKTYPE_* for the given interface, or LINKTYPE_ETHERNET if unknown.
    uint16_t interfaceLinkType(uint32_t interfaceId) const;

    /// Parse EtherCAT datagrams starting at @p ecatOffset within @p data.
    /// @p dataEnd is the exclusive end boundary (data + dataEnd is one-past-the-last valid byte).
    void parseEtherCATDatagrams(const uint8_t* data, size_t dataEnd,
                                size_t ecatOffset,
                                InterpretedFrame& frame) const;

    /// Detect and parse EtherCAT-over-UDP encapsulation (UDP dst port 34980).
    /// @p ipOffset points to the start of the IPv4 header within @p data.
    void parseEtherCATOverUDP(const uint8_t* data, size_t length,
                              size_t ipOffset,
                              InterpretedFrame& frame) const;

    // Byte-order helpers
    uint16_t read16(const uint8_t* p) const;
    uint32_t read32(const uint8_t* p) const;
    uint64_t read64(const uint8_t* p) const;
    uint16_t maybeSwap16(uint16_t v) const;
    uint32_t maybeSwap32(uint32_t v) const;
    uint64_t maybeSwap64(uint64_t v) const;

    static uint32_t padTo32(uint32_t length);

private:
    std::vector<uint8_t> buffer_;
    bool ownsBuffer_ = false;
    bool isOpen_ = false;
    size_t fileSize_ = 0;

    PCAPNGSectionInfo section_;
    std::vector<PCAPNGInterfaceInfo> interfaces_;
    std::vector<PCAPNGInterfaceStats> interfaceStats_;
    std::vector<PCAPNGNameResolutionRecord> nameRecords_;
    std::vector<PCAPNGDecryptionSecrets> decryptionSecrets_;

    // Parsing state
    size_t currentOffset_ = 0;
    bool haveSectionHeader_ = false;

    // Error recovery
    bool recoveryMode_ = false;
    size_t skippedBlockCount_ = 0;
    ErrorCallback errorCallback_;
};

// ============================================================================
// Human-Readable Formatting
// ============================================================================

/**
 * @brief Format an interpreted frame as human-readable text.
 * @param frame         Frame to format.
 * @param verbose       If true, include full payload hex dumps.
 * @param maxDataBytes  Maximum payload bytes to dump per datagram (0 = no limit).
 */
std::string formatInterpretedFrame(const InterpretedFrame& frame,
                                   bool verbose = false,
                                   size_t maxDataBytes = 64);

/**
 * @brief Format an interpreted frame as compact JSON.
 */
std::string frameToJson(const InterpretedFrame& frame);

/**
 * @brief Convert a MAC address to a colon-separated hex string.
 */
std::string macToString(const std::array<uint8_t, 6>& mac);

/**
 * @brief Convert a byte span to a hex string with optional separator.
 */
std::string bytesToHex(const uint8_t* data, size_t length,
                       const std::string& separator = " ");

} // namespace PCAP
} // namespace PacketLoggers
} // namespace Tether
