/**
 * @file EoEHandler.cpp
 * @brief Ethernet over EtherCAT (EoE) Mailbox Handler Implementation
 */

#include "slave/mailbox/IMailboxHandler.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/logging/SlaveLogger.hpp"

#include <cstring>
#include <functional>
#include <vector>
#include <queue>
#include <array>
#include <map>

namespace EtherCAT {
namespace slave {

// ============================================================================
// EoE Constants
// ============================================================================

namespace {
    // EoE Frame Types
    constexpr uint8_t EOE_TYPE_FRAGMENT_DATA = 0x00;
    constexpr uint8_t EOE_TYPE_TIMESTAMP_REQUEST = 0x01;
    constexpr uint8_t EOE_TYPE_TIMESTAMP_RESPONSE = 0x02;
    constexpr uint8_t EOE_TYPE_SET_IP_PARAM_REQ = 0x03;
    constexpr uint8_t EOE_TYPE_SET_IP_PARAM_RESP = 0x04;
    constexpr uint8_t EOE_TYPE_SET_FILTER_REQ = 0x05;
    constexpr uint8_t EOE_TYPE_SET_FILTER_RESP = 0x06;
    constexpr uint8_t EOE_TYPE_GET_IP_PARAM_REQ = 0x07;
    constexpr uint8_t EOE_TYPE_GET_IP_PARAM_RESP = 0x08;
    constexpr uint8_t EOE_TYPE_GET_FILTER_REQ = 0x09;
    constexpr uint8_t EOE_TYPE_GET_FILTER_RESP = 0x0A;
    
    // EoE Result Codes
    constexpr uint16_t EOE_RESULT_SUCCESS = 0x0000;
    constexpr uint16_t EOE_RESULT_UNSUPPORTED_FRAME = 0x0001;
    constexpr uint16_t EOE_RESULT_UNSPECIFIED_ERROR = 0x0002;
    constexpr uint16_t EOE_RESULT_NO_IP_SUPPORT = 0x0201;
    constexpr uint16_t EOE_RESULT_NO_FILTER_SUPPORT = 0x0301;
    
    // Maximum Ethernet frame size
    constexpr size_t MAX_ETHERNET_FRAME = 1518;
}

// ============================================================================
// EoE IP Parameters
// ============================================================================

struct EoEIPParams {
    bool macIncluded{false};
    bool ipIncluded{false};
    bool subnetIncluded{false};
    bool gatewayIncluded{false};
    bool dnsIncluded{false};
    bool dnsNameIncluded{false};
    
    std::array<uint8_t, 6> macAddress{};
    std::array<uint8_t, 4> ipAddress{};
    std::array<uint8_t, 4> subnetMask{};
    std::array<uint8_t, 4> gateway{};
    std::array<uint8_t, 4> dnsServer{};
    std::string dnsName;
};

// ============================================================================
// EoE Fragment Reassembly
// ============================================================================

struct EoEFragment {
    std::vector<uint8_t> data;
    uint8_t fragmentNumber{0};
    uint16_t frameOffset{0};
    uint16_t frameNumber{0};
    bool complete{false};
    bool lastFragment{false};
    uint64_t timestamp{0};
};

// ============================================================================
// EoEHandler Implementation
// ============================================================================

class EoEHandler : public IMailboxHandler {
public:
    explicit EoEHandler(SlaveCore* core)
        : core_(core)
    {
        // Initialize default IP parameters
        ipParams_.macAddress = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
        ipParams_.ipAddress = {192, 168, 1, 100};
        ipParams_.subnetMask = {255, 255, 255, 0};
        ipParams_.gateway = {192, 168, 1, 1};
        ipParams_.dnsServer = {8, 8, 8, 8};
        ipParams_.dnsName = "ethercat-slave";
    }
    
    ~EoEHandler() override = default;
    
    MailboxProtocol getProtocol() const override { return MailboxProtocol::EoE; }
    const char* getProtocolName() const override { return "EoE"; }
    
    void reset() override {
        rxFragments_.clear();
        txQueue_ = std::queue<std::vector<uint8_t>>();
        currentTxFrame_.clear();
    }
    
    bool processRequest(const uint8_t* data, size_t length,
                       uint8_t* response, size_t& responseLength) override {
        if (length < 4) {
            responseLength = 0;
            return false;
        }
        
        // Parse EoE header
        uint8_t frameType = (data[0] >> 4) & 0x0F;
        uint8_t port = data[0] & 0x0F;
        bool lastFragment = (data[1] & 0x80) != 0;
        bool timeAppend = (data[1] & 0x40) != 0;
        // bool timeRequest = (data[1] & 0x20) != 0; // Not used
        uint8_t fragmentNumber = data[1] & 0x3F;
        uint16_t frameOffset = data[2] | ((data[3] & 0x3F) << 8);
        uint16_t frameNumber = (data[3] >> 6) | (length > 4 ? (data[4] << 2) : 0);
        
        switch (frameType) {
            case EOE_TYPE_FRAGMENT_DATA:
                return handleFragmentData(data, length, port, lastFragment, 
                                         fragmentNumber, frameOffset, frameNumber,
                                         response, responseLength);
                
            case EOE_TYPE_TIMESTAMP_REQUEST:
                return handleTimestampRequest(response, responseLength);
                
            case EOE_TYPE_SET_IP_PARAM_REQ:
                return handleSetIPParam(data + 4, length - 4, response, responseLength);
                
            case EOE_TYPE_GET_IP_PARAM_REQ:
                return handleGetIPParam(response, responseLength);
                
            case EOE_TYPE_SET_FILTER_REQ:
                return handleSetFilter(data + 4, length - 4, response, responseLength);
                
            case EOE_TYPE_GET_FILTER_REQ:
                return handleGetFilter(response, responseLength);
                
            default:
                return sendResponse(EOE_TYPE_FRAGMENT_DATA, EOE_RESULT_UNSUPPORTED_FRAME,
                                   response, responseLength);
        }
    }
    
    // Transfer management (not part of IMailboxHandler)
    bool isTransferActive() const {
        return !rxFragments_.empty();
    }
    
    void abortTransfer() {
        rxFragments_.clear();
        txQueue_ = std::queue<std::vector<uint8_t>>();
        currentTxFrame_.clear();
    }
    
    // Network interface
    void setFrameReceivedCallback(std::function<void(const uint8_t*, size_t)> callback) {
        frameReceivedCallback_ = callback;
    }
    
    bool sendEthernetFrame(const uint8_t* data, size_t length) {
        if (length > MAX_ETHERNET_FRAME) {
            return false;
        }
        
        std::vector<uint8_t> frame(data, data + length);
        txQueue_.push(std::move(frame));
        return true;
    }
    
    bool hasPendingTxFrame() const {
        return !txQueue_.empty();
    }
    
    bool getNextTxFragment(uint8_t* data, size_t maxLength, size_t& length) {
        if (currentTxFrame_.empty()) {
            if (txQueue_.empty()) {
                return false;
            }
            currentTxFrame_ = std::move(txQueue_.front());
            txQueue_.pop();
            txOffset_ = 0;
            txFragmentNumber_ = 0;
            txFrameNumber_++;
        }
        
        size_t headerSize = 4;
        size_t maxDataSize = maxLength - headerSize;
        size_t remaining = currentTxFrame_.size() - txOffset_;
        size_t dataSize = std::min(remaining, maxDataSize);
        bool lastFragment = (dataSize == remaining);
        
        // Build EoE header
        data[0] = (EOE_TYPE_FRAGMENT_DATA << 4);  // Frame type
        data[1] = (lastFragment ? 0x80 : 0x00) | (txFragmentNumber_ & 0x3F);
        data[2] = (txOffset_ / 32) & 0xFF;
        data[3] = ((txOffset_ / 32) >> 8) & 0x3F;
        data[3] |= (txFrameNumber_ & 0x03) << 6;
        
        memcpy(data + headerSize, currentTxFrame_.data() + txOffset_, dataSize);
        
        txOffset_ += dataSize;
        txFragmentNumber_++;
        
        if (lastFragment) {
            currentTxFrame_.clear();
        }
        
        length = headerSize + dataSize;
        return true;
    }
    
    // IP Parameters
    void setIPParams(const EoEIPParams& params) {
        ipParams_ = params;
    }
    
    EoEIPParams getIPParams() const {
        return ipParams_;
    }
    
    // Statistics
    struct Stats {
        uint64_t framesReceived{0};
        uint64_t framesSent{0};
        uint64_t fragmentsReceived{0};
        uint64_t fragmentsSent{0};
        uint64_t reassemblyErrors{0};
        uint64_t ipParamRequests{0};
    };
    
    Stats getStats() const { return stats_; }
    
private:
    bool handleFragmentData(const uint8_t* data, size_t length, uint8_t port,
                           bool lastFragment, uint8_t fragmentNumber,
                           uint16_t frameOffset, uint16_t frameNumber,
                           uint8_t* response, size_t& responseLength) {
        stats_.fragmentsReceived++;
        
        size_t headerSize = 4;
        size_t dataSize = length - headerSize;
        const uint8_t* frameData = data + headerSize;
        
        // Find or create fragment entry
        auto& fragments = rxFragments_[frameNumber];
        
        EoEFragment frag;
        frag.data.assign(frameData, frameData + dataSize);
        frag.fragmentNumber = fragmentNumber;
        frag.frameOffset = frameOffset * 32;  // Offset is in 32-byte units
        frag.frameNumber = frameNumber;
        frag.lastFragment = lastFragment;
        frag.timestamp = core_ ? core_->getDCSystemTime() : 0;
        
        fragments.push_back(frag);
        
        // Check if frame is complete
        if (lastFragment) {
            if (tryReassembleFrame(frameNumber)) {
                stats_.framesReceived++;
            } else {
                stats_.reassemblyErrors++;
            }
        }
        
        // No response needed for data fragments
        responseLength = 0;
        return true;
    }
    
    bool tryReassembleFrame(uint16_t frameNumber) {
        auto it = rxFragments_.find(frameNumber);
        if (it == rxFragments_.end()) {
            return false;
        }
        
        auto& fragments = it->second;
        
        // Sort fragments by offset
        std::sort(fragments.begin(), fragments.end(),
                 [](const EoEFragment& a, const EoEFragment& b) {
                     return a.frameOffset < b.frameOffset;
                 });
        
        // Calculate total size and check for gaps
        size_t totalSize = 0;
        uint16_t expectedOffset = 0;
        
        for (const auto& frag : fragments) {
            if (frag.frameOffset != expectedOffset) {
                // Gap in fragments
                rxFragments_.erase(it);
                return false;
            }
            expectedOffset = frag.frameOffset + frag.data.size();
            totalSize += frag.data.size();
        }
        
        // Reassemble frame
        std::vector<uint8_t> frame;
        frame.reserve(totalSize);
        
        for (const auto& frag : fragments) {
            frame.insert(frame.end(), frag.data.begin(), frag.data.end());
        }
        
        // Clean up fragments
        rxFragments_.erase(it);
        
        // Deliver frame
        if (frameReceivedCallback_) {
            frameReceivedCallback_(frame.data(), frame.size());
        }
        
        return true;
    }
    
    bool handleTimestampRequest(uint8_t* response, size_t& responseLength) {
        // Build timestamp response
        response[0] = (EOE_TYPE_TIMESTAMP_RESPONSE << 4);
        response[1] = 0;
        response[2] = 0;
        response[3] = 0;
        
        // Add 64-bit timestamp
        uint64_t timestamp = core_ ? core_->getDCSystemTime() : 0;
        for (int i = 0; i < 8; i++) {
            response[4 + i] = (timestamp >> (i * 8)) & 0xFF;
        }
        
        responseLength = 12;
        return true;
    }
    
    bool handleSetIPParam(const uint8_t* data, size_t length,
                         uint8_t* response, size_t& responseLength) {
        stats_.ipParamRequests++;
        
        if (length < 2) {
            return sendResponse(EOE_TYPE_SET_IP_PARAM_RESP, EOE_RESULT_UNSPECIFIED_ERROR,
                              response, responseLength);
        }
        
        uint16_t flags = data[0] | (data[1] << 8);
        size_t offset = 2;
        
        // Parse IP parameters based on flags
        if (flags & 0x01) {  // MAC included
            if (offset + 6 > length) {
                return sendResponse(EOE_TYPE_SET_IP_PARAM_RESP, EOE_RESULT_UNSPECIFIED_ERROR,
                                  response, responseLength);
            }
            memcpy(ipParams_.macAddress.data(), data + offset, 6);
            ipParams_.macIncluded = true;
            offset += 6;
        }
        
        if (flags & 0x02) {  // IP included
            if (offset + 4 > length) {
                return sendResponse(EOE_TYPE_SET_IP_PARAM_RESP, EOE_RESULT_UNSPECIFIED_ERROR,
                                  response, responseLength);
            }
            memcpy(ipParams_.ipAddress.data(), data + offset, 4);
            ipParams_.ipIncluded = true;
            offset += 4;
        }
        
        if (flags & 0x04) {  // Subnet included
            if (offset + 4 > length) {
                return sendResponse(EOE_TYPE_SET_IP_PARAM_RESP, EOE_RESULT_UNSPECIFIED_ERROR,
                                  response, responseLength);
            }
            memcpy(ipParams_.subnetMask.data(), data + offset, 4);
            ipParams_.subnetIncluded = true;
            offset += 4;
        }
        
        if (flags & 0x08) {  // Gateway included
            if (offset + 4 > length) {
                return sendResponse(EOE_TYPE_SET_IP_PARAM_RESP, EOE_RESULT_UNSPECIFIED_ERROR,
                                  response, responseLength);
            }
            memcpy(ipParams_.gateway.data(), data + offset, 4);
            ipParams_.gatewayIncluded = true;
            offset += 4;
        }
        
        if (flags & 0x10) {  // DNS included
            if (offset + 4 > length) {
                return sendResponse(EOE_TYPE_SET_IP_PARAM_RESP, EOE_RESULT_UNSPECIFIED_ERROR,
                                  response, responseLength);
            }
            memcpy(ipParams_.dnsServer.data(), data + offset, 4);
            ipParams_.dnsIncluded = true;
            offset += 4;
        }
        
        if (flags & 0x20) {  // DNS name included
            ipParams_.dnsName = std::string(reinterpret_cast<const char*>(data + offset));
            ipParams_.dnsNameIncluded = true;
        }
        
        return sendResponse(EOE_TYPE_SET_IP_PARAM_RESP, EOE_RESULT_SUCCESS,
                           response, responseLength);
    }
    
    bool handleGetIPParam(uint8_t* response, size_t& responseLength) {
        stats_.ipParamRequests++;
        
        // Build response header
        response[0] = (EOE_TYPE_GET_IP_PARAM_RESP << 4);
        response[1] = 0;
        response[2] = EOE_RESULT_SUCCESS & 0xFF;
        response[3] = (EOE_RESULT_SUCCESS >> 8) & 0xFF;
        
        // Calculate flags
        uint16_t flags = 0;
        size_t offset = 6;
        
        // MAC address
        flags |= 0x01;
        memcpy(response + offset, ipParams_.macAddress.data(), 6);
        offset += 6;
        
        // IP address
        flags |= 0x02;
        memcpy(response + offset, ipParams_.ipAddress.data(), 4);
        offset += 4;
        
        // Subnet mask
        flags |= 0x04;
        memcpy(response + offset, ipParams_.subnetMask.data(), 4);
        offset += 4;
        
        // Gateway
        flags |= 0x08;
        memcpy(response + offset, ipParams_.gateway.data(), 4);
        offset += 4;
        
        // DNS server
        flags |= 0x10;
        memcpy(response + offset, ipParams_.dnsServer.data(), 4);
        offset += 4;
        
        // DNS name
        if (!ipParams_.dnsName.empty()) {
            flags |= 0x20;
            memcpy(response + offset, ipParams_.dnsName.c_str(), 
                   ipParams_.dnsName.length() + 1);
            offset += ipParams_.dnsName.length() + 1;
        }
        
        // Write flags
        response[4] = flags & 0xFF;
        response[5] = (flags >> 8) & 0xFF;
        
        responseLength = offset;
        return true;
    }
    
    bool handleSetFilter(const uint8_t* data, size_t length,
                        uint8_t* response, size_t& responseLength) {
        // Filter support not implemented
        return sendResponse(EOE_TYPE_SET_FILTER_RESP, EOE_RESULT_NO_FILTER_SUPPORT,
                           response, responseLength);
    }
    
    bool handleGetFilter(uint8_t* response, size_t& responseLength) {
        // Filter support not implemented
        return sendResponse(EOE_TYPE_GET_FILTER_RESP, EOE_RESULT_NO_FILTER_SUPPORT,
                           response, responseLength);
    }
    
    bool sendResponse(uint8_t frameType, uint16_t result,
                     uint8_t* response, size_t& responseLength) {
        response[0] = (frameType << 4);
        response[1] = 0;
        response[2] = result & 0xFF;
        response[3] = (result >> 8) & 0xFF;
        responseLength = 4;
        return true;
    }
    
private:
    SlaveCore* core_;
    EoEIPParams ipParams_;
    
    // RX fragment reassembly
    std::map<uint16_t, std::vector<EoEFragment>> rxFragments_;
    
    // TX fragmentation
    std::queue<std::vector<uint8_t>> txQueue_;
    std::vector<uint8_t> currentTxFrame_;
    size_t txOffset_{0};
    uint8_t txFragmentNumber_{0};
    uint16_t txFrameNumber_{0};
    
    // Callback
    std::function<void(const uint8_t*, size_t)> frameReceivedCallback_;
    
    // Statistics
    Stats stats_;
};

// Factory function
std::unique_ptr<IMailboxHandler> createEoEHandler(SlaveCore* core) {
    return std::make_unique<EoEHandler>(core);
}

// Factory overload matching header declaration (no SlaveCore)
std::unique_ptr<IMailboxHandler> createEoEHandler() {
    return std::make_unique<EoEHandler>(nullptr);
}

}  // namespace slave
}  // namespace EtherCAT
