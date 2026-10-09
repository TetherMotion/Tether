/**
 * @file EtherCATWireFormatStructures.hpp
 * @brief EtherCAT types: Wire-Format Structures (Packed)
 *
 * Split out of Types.hpp.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <functional>

namespace EtherCAT {

// ============================================================================
// Wire-Format Structures (Packed)
// ============================================================================

// Backwards-compatible alias used by older code/tests that reference Raw::EtherCATCommand
namespace Raw {
    using EtherCATCommand = Command;
}

/**
 * @brief Ethernet frame header
 */
struct __attribute__((packed)) EthernetHeader {
    uint8_t dst[6];         ///< Destination MAC address
    uint8_t src[6];         ///< Source MAC address
    uint16_t etherType_be;  ///< EtherType in big-endian
};
static_assert(sizeof(EthernetHeader) == 14, "EthernetHeader must be 14 bytes");

/**
 * @brief EtherCAT frame header (2 bytes after Ethernet header)
 */
struct __attribute__((packed)) FrameHeader {
    uint16_t raw_le;  ///< [0:10]=length, [11]=reserved, [12:15]=type (must be 1)
    
    uint16_t length() const { return raw_le & 0x07FF; }
    uint16_t type() const { return (raw_le >> 12) & 0x0F; }
    
    void setLength(uint16_t len) {
        raw_le = (raw_le & 0xF800) | (len & 0x07FF);
    }
    void setType(uint16_t t) {
        raw_le = (raw_le & 0x0FFF) | ((t & 0x0F) << 12);
    }
    void set(uint16_t len, uint16_t type = 1) {
        raw_le = (len & 0x07FF) | ((type & 0x0F) << 12);
    }
};
static_assert(sizeof(FrameHeader) == 2, "FrameHeader must be 2 bytes");

/**
 * @brief EtherCAT datagram header (10 bytes)
 */
struct __attribute__((packed)) DatagramHeader {
    Command cmd;        ///< Command type
    uint8_t idx;        ///< Index for request/response matching
    uint16_t adp_le;    ///< Address Position
    uint16_t ado_le;    ///< Address Offset
    uint16_t lenFlags_le; ///< [0:10]=length, [14]=C, [15]=M
    uint16_t irq_le;    ///< Interrupt request
    
    // Accessors
    uint16_t dataLength() const { return lenFlags_le & 0x07FF; }
    bool more() const { return (lenFlags_le & 0x8000) != 0; }
    bool circulating() const { return (lenFlags_le & 0x4000) != 0; }
    
    void setDataLength(uint16_t len) {
        lenFlags_le = (lenFlags_le & 0xF800) | (len & 0x07FF);
    }
    void setMore(bool m) {
        if (m) lenFlags_le |= 0x8000;
        else lenFlags_le &= ~0x8000;
    }
    void setCirculating(bool c) {
        if (c) lenFlags_le |= 0x4000;
        else lenFlags_le &= ~0x4000;
    }
    
    // Get 32-bit logical address (for LRD/LWR/LRW)
    uint32_t logicalAddress() const {
        return (static_cast<uint32_t>(ado_le) << 16) | adp_le;
    }
    void setLogicalAddress(uint32_t addr) {
        adp_le = addr & 0xFFFF;
        ado_le = (addr >> 16) & 0xFFFF;
    }
};
static_assert(sizeof(DatagramHeader) == 10, "DatagramHeader must be 10 bytes");

/**
 * @brief Complete datagram with header, data, and WKC
 * 
 * This is a variable-size structure. The data array size is a maximum.
 */
struct Datagram {
    DatagramHeader header;
    uint8_t data[kMaxDatagramDataSize];
    uint16_t wkc = 0;
    
    // Actual data size based on header
    size_t totalSize() const {
        return sizeof(DatagramHeader) + header.dataLength() + sizeof(uint16_t);
    }
    
    // Copy data into the datagram
    void setData(const void* src, size_t len) {
        if (len > kMaxDatagramDataSize) len = kMaxDatagramDataSize;
        std::memcpy(data, src, len);
        header.setDataLength(static_cast<uint16_t>(len));
    }
    
    // Get pointer to WKC (after data)
    uint16_t* wkcPtr() {
        return reinterpret_cast<uint16_t*>(data + header.dataLength());
    }
    const uint16_t* wkcPtr() const {
        return reinterpret_cast<const uint16_t*>(data + header.dataLength());
    }
    
    // Read WKC from wire position
    uint16_t readWkc() const {
        uint16_t w;
        std::memcpy(&w, data + header.dataLength(), sizeof(w));
        return w;
    }
    
    // Write WKC to wire position
    void writeWkc(uint16_t w) {
        std::memcpy(data + header.dataLength(), &w, sizeof(w));
        wkc = w;
    }
};

/**
 * @brief Lightweight datagram reference for parsing received frames
 * 
 * Points into a frame buffer without copying data.
 */
struct DatagramView {
    Command cmd;
    uint8_t idx;
    uint16_t adp;
    uint16_t ado;
    uint16_t dataLength;
    const uint8_t* data;
    uint16_t wkc;
    bool more;
    
    // Get 32-bit logical address
    uint32_t logicalAddress() const {
        return (static_cast<uint32_t>(ado) << 16) | adp;
    }
    
    // Parse from wire format
    static DatagramView parse(const uint8_t* ptr, size_t available) {
        DatagramView v{};
        if (available < sizeof(DatagramHeader)) {
            return v;
        }
        
        // Use memcpy instead of reinterpret_cast for safety
        DatagramHeader hdr;
        std::memcpy(&hdr, ptr, sizeof(hdr));
        v.cmd = hdr.cmd;
        v.idx = hdr.idx;
        v.adp = hdr.adp_le;
        v.ado = hdr.ado_le;
        v.dataLength = hdr.dataLength();
        v.more = hdr.more();
        
        if (available < sizeof(DatagramHeader) + v.dataLength + 2) {
            v.dataLength = 0;
            return v;
        }
        
        v.data = ptr + sizeof(DatagramHeader);
        std::memcpy(&v.wkc, v.data + v.dataLength, sizeof(v.wkc));
        
        return v;
    }
    
    // Total size in bytes
    size_t totalSize() const {
        return sizeof(DatagramHeader) + dataLength + sizeof(uint16_t);
    }
};

} // namespace EtherCAT
