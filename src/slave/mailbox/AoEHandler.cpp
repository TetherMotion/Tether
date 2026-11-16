/**
 * @file AoEHandler.cpp
 * @brief ADS over EtherCAT (AoE) Mailbox Handler Implementation
 * 
 * Implements Beckhoff ADS protocol over EtherCAT for TwinCAT compatibility.
 */

#include "slave/mailbox/IMailboxHandler.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/logging/SlaveLogger.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>
#include <map>

namespace EtherCAT {
namespace slave {

// ============================================================================
// AoE / ADS Constants
// ============================================================================

namespace {
    // ADS Command IDs
    constexpr uint16_t ADS_CMD_INVALID = 0x0000;
    constexpr uint16_t ADS_CMD_READ_DEVICE_INFO = 0x0001;
    constexpr uint16_t ADS_CMD_READ = 0x0002;
    constexpr uint16_t ADS_CMD_WRITE = 0x0003;
    constexpr uint16_t ADS_CMD_READ_STATE = 0x0004;
    constexpr uint16_t ADS_CMD_WRITE_CONTROL = 0x0005;
    constexpr uint16_t ADS_CMD_ADD_DEVICE_NOTIFICATION = 0x0006;
    constexpr uint16_t ADS_CMD_DEL_DEVICE_NOTIFICATION = 0x0007;
    constexpr uint16_t ADS_CMD_DEVICE_NOTIFICATION = 0x0008;
    constexpr uint16_t ADS_CMD_READ_WRITE = 0x0009;
    
    // ADS State Flags
    constexpr uint16_t ADS_STATE_INVALID = 0x0000;
    constexpr uint16_t ADS_STATE_IDLE = 0x0001;
    constexpr uint16_t ADS_STATE_RESET = 0x0002;
    constexpr uint16_t ADS_STATE_INIT = 0x0003;
    constexpr uint16_t ADS_STATE_START = 0x0004;
    constexpr uint16_t ADS_STATE_RUN = 0x0005;
    constexpr uint16_t ADS_STATE_STOP = 0x0006;
    constexpr uint16_t ADS_STATE_SAVECFG = 0x0007;
    constexpr uint16_t ADS_STATE_LOADCFG = 0x0008;
    constexpr uint16_t ADS_STATE_POWERFAILURE = 0x0009;
    constexpr uint16_t ADS_STATE_POWERGOOD = 0x000A;
    constexpr uint16_t ADS_STATE_ERROR = 0x000B;
    constexpr uint16_t ADS_STATE_SHUTDOWN = 0x000C;
    constexpr uint16_t ADS_STATE_SUSPEND = 0x000D;
    constexpr uint16_t ADS_STATE_RESUME = 0x000E;
    constexpr uint16_t ADS_STATE_CONFIG = 0x000F;
    constexpr uint16_t ADS_STATE_RECONFIG = 0x0010;
    
    // ADS Error Codes
    constexpr uint32_t ADS_ERR_NOERROR = 0x0000;
    constexpr uint32_t ADS_ERR_INTERNAL = 0x0001;
    constexpr uint32_t ADS_ERR_NO_RTIME = 0x0002;
    constexpr uint32_t ADS_ERR_ALLOCLOCKEDMEM = 0x0003;
    constexpr uint32_t ADS_ERR_INSERT_MAILBOX = 0x0004;
    constexpr uint32_t ADS_ERR_WRONGRECEIVEHMSG = 0x0005;
    constexpr uint32_t ADS_ERR_TARGETPORT_NOT_FOUND = 0x0006;
    constexpr uint32_t ADS_ERR_TARGETMACHINE_NOT_FOUND = 0x0007;
    constexpr uint32_t ADS_ERR_UNKNOWNCMD_ID = 0x0008;
    constexpr uint32_t ADS_ERR_BADTASK_ID = 0x0009;
    constexpr uint32_t ADS_ERR_NO_IO = 0x000A;
    constexpr uint32_t ADS_ERR_UNKNOWN_AMS_CMD = 0x000B;
    constexpr uint32_t ADS_ERR_WIN32_ERROR = 0x000C;
    constexpr uint32_t ADS_ERR_PORTNOTCONNECTED = 0x000D;
    constexpr uint32_t ADS_ERR_INVALIDAMSPORT = 0x000E;
    constexpr uint32_t ADS_ERR_INVALIDAMSNETID = 0x000F;
    constexpr uint32_t ADS_ERR_LOWINSTLEVEL = 0x0010;
    constexpr uint32_t ADS_ERR_NODEBUGINTAVAILABLE = 0x0011;
    constexpr uint32_t ADS_ERR_PORTDISABLED = 0x0012;
    constexpr uint32_t ADS_ERR_PORTNOTCONNECTED2 = 0x0013;
    constexpr uint32_t ADS_ERR_SYNCPORT_LOCKED = 0x0014;
    
    // Index groups for standard operations
    constexpr uint32_t ADSIGRP_SYM_HANDLE = 0xF003;
    constexpr uint32_t ADSIGRP_SYM_VAL_BYNAME = 0xF004;
    constexpr uint32_t ADSIGRP_SYM_VAL_BYHANDLE = 0xF005;
    constexpr uint32_t ADSIGRP_SYM_REL_HANDLE = 0xF006;
    constexpr uint32_t ADSIGRP_SYM_INFO_BYNAME = 0xF007;
    constexpr uint32_t ADSIGRP_SYM_VERSION = 0xF008;
    constexpr uint32_t ADSIGRP_SYM_INFO_BYADDR = 0xF009;
    constexpr uint32_t ADSIGRP_SYM_DOWNLOAD = 0xF00A;
    constexpr uint32_t ADSIGRP_SYM_UPLOAD = 0xF00B;
    constexpr uint32_t ADSIGRP_SYM_UPLOAD_INFO = 0xF00C;
}

// ============================================================================
// AMS/ADS Header Structures
// ============================================================================

#pragma pack(push, 1)

struct AmsNetId {
    uint8_t b[6];
    
    bool operator==(const AmsNetId& other) const {
        if (this == &other) return true;
        return memcmp(b, other.b, 6) == 0;
    }
};

struct AmsAddr {
    AmsNetId netId;
    uint16_t port;
};

struct AmsHeader {
    AmsAddr target;
    AmsAddr source;
    uint16_t cmdId;
    uint16_t stateFlags;
    uint32_t cbData;
    uint32_t errorCode;
    uint32_t invokeId;
};

struct AdsReadRequest {
    uint32_t indexGroup;
    uint32_t indexOffset;
    uint32_t cbLength;
};

struct AdsWriteRequest {
    uint32_t indexGroup;
    uint32_t indexOffset;
    uint32_t cbLength;
    // Data follows
};

struct AdsReadWriteRequest {
    uint32_t indexGroup;
    uint32_t indexOffset;
    uint32_t cbReadLength;
    uint32_t cbWriteLength;
    // Write data follows
};

struct AdsWriteControlRequest {
    uint16_t adsState;
    uint16_t deviceState;
    uint32_t cbLength;
    // Data follows
};

// AddDeviceNotification request (cmd 0x0006).  indexGroup/indexOffset
// address the monitored variable in the same space used by READ/WRITE;
// for SYM_VAL_BYNAME a NUL-terminated symbol name may follow the struct.
struct AdsAddDeviceNotification {
    uint32_t indexGroup;
    uint32_t indexOffset;
    uint32_t length;      // monitored data length
    uint32_t time;        // reserved (dead time) [100 ns]
    uint32_t cycle;       // cycle time [100 ns]; 0 = on-change only
    uint16_t flags;       // bit0: include timestamp in notifications
    uint16_t reserved;
    uint32_t maxDelay;    // max delay [100 ns]
};

// DelDeviceNotification request (cmd 0x0007): notification handle.
struct AdsDelDeviceNotification {
    uint32_t notificationHandle;
};

// DeviceNotification payload (cmd 0x0008, sent to the subscriber):
//   errorCode u32 | notificationHandle u32 | cbLength u32 | data | [timestamp u64]

#pragma pack(pop)

// ============================================================================
// ADS Symbol Table Entry
// ============================================================================

struct AdsSymbol {
    std::string name;
    uint32_t indexGroup;
    uint32_t indexOffset;
    uint32_t size;
    uint16_t dataType;
    std::vector<uint8_t> data;
    bool readOnly{false};
    
    // Callbacks
    std::function<std::vector<uint8_t>()> readCallback;
    std::function<bool(const std::vector<uint8_t>&)> writeCallback;
};

// ============================================================================
// AoEHandler Implementation
// ============================================================================

class AoEHandler : public IMailboxHandler {
public:
    explicit AoEHandler(SlaveCore* core)
        : core_(core)
    {
        // Initialize default AMS Net ID (based on MAC or configured)
        localNetId_ = {{192, 168, 1, 100, 1, 1}};
        localPort_ = 851;  // Default TwinCAT port
        
        adsState_ = ADS_STATE_RUN;
        deviceState_ = 0;
        
        // Register some example symbols
        registerSymbol("MAIN.Counter", ADSIGRP_SYM_VAL_BYNAME, 0, 4, 0x0004);
        registerSymbol("MAIN.Position", ADSIGRP_SYM_VAL_BYNAME, 4, 4, 0x0004);
        registerSymbol("MAIN.Velocity", ADSIGRP_SYM_VAL_BYNAME, 8, 4, 0x0004);
        registerSymbol("MAIN.Status", ADSIGRP_SYM_VAL_BYNAME, 12, 2, 0x0003);
    }
    
    ~AoEHandler() override {
        stopNotificationThread();
    }
    
    MailboxProtocol getProtocol() const override { return MailboxProtocol::AoE; }
    const char* getProtocolName() const override { return "AoE"; }
    
    void reset() override {
        {
            std::lock_guard<std::mutex> lk(notifMutex_);
            symbols_.clear();
            symbolsByAddr_.clear();
            for (auto& n : notifications_) n.symbolName.clear();
        }
        adsState_ = ADS_STATE_RUN;
        deviceState_ = 0;
    }
    
    bool processRequest(const uint8_t* data, size_t length,
                       uint8_t* response, size_t& responseLength) override {
        // Every handler's fixed response fits in sizeof(AmsHeader) + 32 bytes
        // (device-info is the largest); reject tiny buffers up front.
        if (responseLength < sizeof(AmsHeader) + 32) {
            responseLength = 0;
            return false;
        }
        if (length < sizeof(AmsHeader)) {
            return sendError(ADS_ERR_INTERNAL, {}, response, responseLength);
        }
        
        // Parse AMS header
        AmsHeader header;
        memcpy(&header, data, sizeof(header));
        
        // Verify target
        if (header.target.port != localPort_ && header.target.port != 0) {
            return sendError(ADS_ERR_TARGETPORT_NOT_FOUND, header, response, responseLength);
        }
        
        // Store source for response
        currentSource_ = header.source;
        currentInvokeId_ = header.invokeId;
        
        // Process based on command
        const uint8_t* cmdData = data + sizeof(AmsHeader);
        size_t cmdLength = length - sizeof(AmsHeader);
        
        switch (header.cmdId) {
            case ADS_CMD_READ_DEVICE_INFO:
                return handleReadDeviceInfo(header, response, responseLength);
                
            case ADS_CMD_READ:
                return handleRead(header, cmdData, cmdLength, response, responseLength);
                
            case ADS_CMD_WRITE:
                return handleWrite(header, cmdData, cmdLength, response, responseLength);
                
            case ADS_CMD_READ_STATE:
                return handleReadState(header, response, responseLength);
                
            case ADS_CMD_WRITE_CONTROL:
                return handleWriteControl(header, cmdData, cmdLength, response, responseLength);
                
            case ADS_CMD_READ_WRITE:
                return handleReadWrite(header, cmdData, cmdLength, response, responseLength);
                
            case ADS_CMD_ADD_DEVICE_NOTIFICATION:
                return handleAddNotification(header, cmdData, cmdLength, response, responseLength);
                
            case ADS_CMD_DEL_DEVICE_NOTIFICATION:
                return handleDelNotification(header, cmdData, cmdLength, response, responseLength);
                
            default:
                return sendError(ADS_ERR_UNKNOWNCMD_ID, header, response, responseLength);
        }
    }
    
    // isTransferActive and abortTransfer are not in IMailboxHandler interface
    // Provide them as regular methods without override
    bool isTransferActive() const {
        return false;  // AoE doesn't have multi-packet transfers
    }
    
    void abortTransfer() {
        // Nothing to abort
    }

    // Pending (unsolicited) responses — used for ADS device notifications.
    bool hasPendingResponse() const override {
        std::lock_guard<std::mutex> lk(pendingMutex_);
        return !pendingResponses_.empty();
    }

    bool getPendingResponse(uint8_t* response, size_t& responseLen) override {
        std::lock_guard<std::mutex> lk(pendingMutex_);
        if (pendingResponses_.empty()) return false;
        const auto& msg = pendingResponses_.front();
        if (msg.size() > responseLen) return false;
        memcpy(response, msg.data(), msg.size());
        responseLen = msg.size();
        pendingResponses_.pop_front();
        return true;
    }
    
    // Configuration
    void setLocalNetId(const AmsNetId& netId) { localNetId_ = netId; }
    void setLocalPort(uint16_t port) { localPort_ = port; }
    
    // Symbol management
    void registerSymbol(const std::string& name, uint32_t indexGroup, uint32_t indexOffset,
                       uint32_t size, uint16_t dataType) {
        AdsSymbol sym;
        sym.name = name;
        sym.indexGroup = indexGroup;
        sym.indexOffset = indexOffset;
        sym.size = size;
        sym.dataType = dataType;
        sym.data.resize(size, 0);

        std::lock_guard<std::mutex> lk(notifMutex_);
        symbols_[name] = sym;
        symbolsByAddr_[{indexGroup, indexOffset}] = &symbols_[name];
    }
    
    void setSymbolData(const std::string& name, const std::vector<uint8_t>& data) {
        std::lock_guard<std::mutex> lk(notifMutex_);
        auto it = symbols_.find(name);
        if (it != symbols_.end() && data.size() == it->second.size) {
            it->second.data = data;
        }
    }
    
    std::vector<uint8_t> getSymbolData(const std::string& name) const {
        std::lock_guard<std::mutex> lk(notifMutex_);
        auto it = symbols_.find(name);
        if (it != symbols_.end()) {
            if (it->second.readCallback) {
                return it->second.readCallback();
            }
            return it->second.data;
        }
        return {};
    }
    
    // State
    void setAdsState(uint16_t state) { adsState_ = state; }
    uint16_t getAdsState() const { return adsState_; }
    
private:
    bool handleReadDeviceInfo(const AmsHeader& request,
                             uint8_t* response, size_t& responseLength) {
        // Build response header
        AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_READ_DEVICE_INFO);
        
        // Device info response
        struct {
            uint8_t majorVersion;
            uint8_t minorVersion;
            uint16_t versionBuild;
            char deviceName[16];
        } deviceInfo;
        
        deviceInfo.majorVersion = 1;
        deviceInfo.minorVersion = 0;
        deviceInfo.versionBuild = 1;
        strncpy(deviceInfo.deviceName, "EtherCAT Slave", sizeof(deviceInfo.deviceName) - 1);
        
        respHeader.cbData = sizeof(uint32_t) + sizeof(deviceInfo);  // Error code + data
        
        memcpy(response, &respHeader, sizeof(respHeader));
        
        uint32_t errorCode = ADS_ERR_NOERROR;
        memcpy(response + sizeof(respHeader), &errorCode, sizeof(errorCode));
        memcpy(response + sizeof(respHeader) + sizeof(errorCode), &deviceInfo, sizeof(deviceInfo));
        
        responseLength = sizeof(respHeader) + sizeof(errorCode) + sizeof(deviceInfo);
        return true;
    }
    
    bool handleRead(const AmsHeader& request, const uint8_t* data, size_t length,
                   uint8_t* response, size_t& responseLength) {
        // responseLength on input holds the response buffer capacity.
        size_t bufCap = responseLength;
        if (length < sizeof(AdsReadRequest)) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }
        
        AdsReadRequest readReq;
        memcpy(&readReq, data, sizeof(readReq));
        
        // Find data by index group/offset
        std::vector<uint8_t> readData;
        
        // Symbol-name reads: when indexGroup is SYM_VAL_BYNAME and the
        // request carries a trailing NUL-terminated symbol name after the
        // fixed AdsReadRequest, resolve by name.  Requests without a name
        // fall through to the flat {group, offset} address lookup.
        AdsSymbol* sym = nullptr;
        size_t nameBytes = 0;
        bool nameProvided = false;
        if (readReq.indexGroup == ADSIGRP_SYM_VAL_BYNAME &&
            length > sizeof(AdsReadRequest)) {
            const char* name =
                reinterpret_cast<const char*>(data + sizeof(AdsReadRequest));
            const size_t avail = length - sizeof(AdsReadRequest);
            const size_t nl = strnlen(name, avail);
            if (nl > 0 && nl < avail) {
                nameProvided = true;
                nameBytes = nl + 1;
                auto nit = symbols_.find(std::string(name, nl));
                if (nit != symbols_.end()) sym = &nit->second;
            }
        }
        if (nameProvided && !sym) {
            // A name was supplied but does not resolve — per ADS this is
            // an error, not a fall-back to flat addressing.
            return sendError(ADS_ERR_NO_IO, request, response, responseLength);
        }
        if (!sym) {
            if (readReq.indexGroup == ADSIGRP_SYM_VAL_BYHANDLE) {
                // Handle encodes (indexGroup << 16) | (indexOffset & 0xFFFF)
                const uint32_t handle = readReq.indexOffset;
                auto hit = symbolsByAddr_.find(
                    {handle >> 16, handle & 0xFFFFu});
                if (hit != symbolsByAddr_.end()) sym = hit->second;
            } else {
                auto it = symbolsByAddr_.find(
                    {readReq.indexGroup, readReq.indexOffset});
                if (it != symbolsByAddr_.end()) sym = it->second;
            }
        }

        // Apply the in-variable byte offset (indexOffset for by-name reads
        // is an offset into the symbol's data, not an address).
        uint32_t varOffset = 0;
        if (sym && nameBytes > 0) varOffset = readReq.indexOffset;

        if (sym) {
            if (sym->readCallback) {
                readData = sym->readCallback();
            } else {
                readData = sym->data;
            }
            if (varOffset < readData.size()) {
                readData.erase(readData.begin(),
                               readData.begin() + varOffset);
            } else {
                readData.clear();
            }
            if (readData.size() > readReq.cbLength) {
                readData.resize(readReq.cbLength);
            }
        } else {
            return sendError(ADS_ERR_NO_IO, request, response, responseLength);
        }
        
        // Build response
        AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_READ);
        respHeader.cbData = sizeof(uint32_t) + sizeof(uint32_t) + readData.size();

        size_t totalNeeded = sizeof(respHeader) + sizeof(uint32_t) + sizeof(uint32_t) + readData.size();
        if (totalNeeded > bufCap) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }

        memcpy(response, &respHeader, sizeof(respHeader));
        
        size_t offset = sizeof(respHeader);
        uint32_t errorCode = ADS_ERR_NOERROR;
        memcpy(response + offset, &errorCode, sizeof(errorCode));
        offset += sizeof(errorCode);
        
        uint32_t cbLength = readData.size();
        memcpy(response + offset, &cbLength, sizeof(cbLength));
        offset += sizeof(cbLength);
        
        memcpy(response + offset, readData.data(), readData.size());
        offset += readData.size();
        
        responseLength = offset;
        return true;
    }
    
    bool handleWrite(const AmsHeader& request, const uint8_t* data, size_t length,
                    uint8_t* response, size_t& responseLength) {
        if (length < sizeof(AdsWriteRequest)) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }
        
        AdsWriteRequest writeReq;
        memcpy(&writeReq, data, sizeof(writeReq));
        
        if (length < sizeof(AdsWriteRequest) + writeReq.cbLength) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }
        
        const uint8_t* writeData = data + sizeof(AdsWriteRequest);
        const size_t writeAvail = length - sizeof(AdsWriteRequest);

        // Resolve target symbol: trailing NUL-terminated name for
        // SYM_VAL_BYNAME, handle decode for SYM_VAL_BYHANDLE, otherwise
        // the flat {group, offset} address space.
        AdsSymbol* sym = nullptr;
        size_t nameBytes = 0;
        bool nameProvided = false;
        if (writeReq.indexGroup == ADSIGRP_SYM_VAL_BYNAME &&
            writeAvail > writeReq.cbLength) {
            const char* name = reinterpret_cast<const char*>(
                writeData + writeReq.cbLength);
            const size_t avail = writeAvail - writeReq.cbLength;
            const size_t nl = strnlen(name, avail);
            if (nl > 0 && nl < avail) {
                nameProvided = true;
                nameBytes = nl + 1;
                auto nit = symbols_.find(std::string(name, nl));
                if (nit != symbols_.end()) sym = &nit->second;
            }
        }
        if (nameProvided && !sym) {
            return sendError(ADS_ERR_NO_IO, request, response, responseLength);
        }
        if (!sym) {
            if (writeReq.indexGroup == ADSIGRP_SYM_VAL_BYHANDLE) {
                const uint32_t handle = writeReq.indexOffset;
                auto hit = symbolsByAddr_.find(
                    {handle >> 16, handle & 0xFFFFu});
                if (hit != symbolsByAddr_.end()) sym = hit->second;
            } else {
                auto it = symbolsByAddr_.find(
                    {writeReq.indexGroup, writeReq.indexOffset});
                if (it != symbolsByAddr_.end()) sym = it->second;
            }
        }

        if (sym) {
            if (sym->readOnly) {
                return sendError(ADS_ERR_NO_IO, request, response, responseLength);
            }

            // For by-name writes indexOffset is an in-variable offset.
            const uint8_t* payload = writeData;
            size_t payloadLen = writeReq.cbLength;
            uint32_t varOffset = 0;
            if (nameBytes > 0) {
                varOffset = writeReq.indexOffset;
                if (varOffset >= sym->data.size()) {
                    return sendError(ADS_ERR_NO_IO, request, response, responseLength);
                }
            }

            std::vector<uint8_t> newData(payload, payload + payloadLen);

            if (sym->writeCallback) {
                if (!sym->writeCallback(newData)) {
                    return sendError(ADS_ERR_NO_IO, request, response, responseLength);
                }
            } else {
                const size_t room = sym->data.size() - varOffset;
                const size_t n = std::min(room, newData.size());
                if (n > 0) {
                    memcpy(sym->data.data() + varOffset, newData.data(), n);
                }
            }
        } else {
            return sendError(ADS_ERR_NO_IO, request, response, responseLength);
        }
        
        // Build success response
        AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_WRITE);
        respHeader.cbData = sizeof(uint32_t);
        
        memcpy(response, &respHeader, sizeof(respHeader));
        
        uint32_t errorCode = ADS_ERR_NOERROR;
        memcpy(response + sizeof(respHeader), &errorCode, sizeof(errorCode));
        
        responseLength = sizeof(respHeader) + sizeof(errorCode);
        return true;
    }
    
    bool handleReadState(const AmsHeader& request,
                        uint8_t* response, size_t& responseLength) {
        AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_READ_STATE);
        respHeader.cbData = sizeof(uint32_t) + sizeof(uint16_t) + sizeof(uint16_t);
        
        memcpy(response, &respHeader, sizeof(respHeader));
        
        size_t offset = sizeof(respHeader);
        uint32_t errorCode = ADS_ERR_NOERROR;
        memcpy(response + offset, &errorCode, sizeof(errorCode));
        offset += sizeof(errorCode);
        
        memcpy(response + offset, &adsState_, sizeof(adsState_));
        offset += sizeof(adsState_);
        
        memcpy(response + offset, &deviceState_, sizeof(deviceState_));
        offset += sizeof(deviceState_);
        
        responseLength = offset;
        return true;
    }
    
    bool handleWriteControl(const AmsHeader& request, const uint8_t* data, size_t length,
                           uint8_t* response, size_t& responseLength) {
        if (length < sizeof(AdsWriteControlRequest)) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }
        
        AdsWriteControlRequest writeCtrl;
        memcpy(&writeCtrl, data, sizeof(writeCtrl));
        
        // Update state
        adsState_ = writeCtrl.adsState;
        deviceState_ = writeCtrl.deviceState;
        
        // Build success response
        AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_WRITE_CONTROL);
        respHeader.cbData = sizeof(uint32_t);
        
        memcpy(response, &respHeader, sizeof(respHeader));
        
        uint32_t errorCode = ADS_ERR_NOERROR;
        memcpy(response + sizeof(respHeader), &errorCode, sizeof(errorCode));
        
        responseLength = sizeof(respHeader) + sizeof(errorCode);
        return true;
    }
    
    bool handleReadWrite(const AmsHeader& request, const uint8_t* data, size_t length,
                        uint8_t* response, size_t& responseLength) {
        if (length < sizeof(AdsReadWriteRequest)) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }
        
        AdsReadWriteRequest rwReq;
        memcpy(&rwReq, data, sizeof(rwReq));
        
        // The declared write-data length must fit within the actual request —
        // otherwise the symbol-name read below runs past the buffer.
        if (rwReq.cbWriteLength > length - sizeof(AdsReadWriteRequest)) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }
        
        // Handle special index groups
        if (rwReq.indexGroup == ADSIGRP_SYM_HANDLE) {
            // Get handle by name
            // Write data is the symbol name
            std::string symbolName(reinterpret_cast<const char*>(data + sizeof(AdsReadWriteRequest)),
                                   rwReq.cbWriteLength);
            
            auto it = symbols_.find(symbolName);
            if (it == symbols_.end()) {
                return sendError(ADS_ERR_NO_IO, request, response, responseLength);
            }
            
            // Return a handle (use address as handle)
            uint32_t handle = (it->second.indexGroup << 16) | (it->second.indexOffset & 0xFFFF);
            
            AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_READ_WRITE);
            respHeader.cbData = sizeof(uint32_t) + sizeof(uint32_t) + sizeof(handle);
            
            memcpy(response, &respHeader, sizeof(respHeader));
            
            size_t offset = sizeof(respHeader);
            uint32_t errorCode = ADS_ERR_NOERROR;
            memcpy(response + offset, &errorCode, sizeof(errorCode));
            offset += sizeof(errorCode);
            
            uint32_t cbLength = sizeof(handle);
            memcpy(response + offset, &cbLength, sizeof(cbLength));
            offset += sizeof(cbLength);
            
            memcpy(response + offset, &handle, sizeof(handle));
            offset += sizeof(handle);
            
            responseLength = offset;
            return true;
        }

        // Generic read+write against the symbol address space: write the
        // write-data at (indexGroup, indexOffset), then read cbReadLength
        // bytes back from the same address (post-write value per ADS).
        AdsSymbol* sym = nullptr;
        if (rwReq.indexGroup == ADSIGRP_SYM_VAL_BYHANDLE) {
            const uint32_t handle = rwReq.indexOffset;
            auto hit = symbolsByAddr_.find({handle >> 16, handle & 0xFFFFu});
            if (hit != symbolsByAddr_.end()) sym = hit->second;
        } else {
            auto it = symbolsByAddr_.find(
                {rwReq.indexGroup, rwReq.indexOffset});
            if (it != symbolsByAddr_.end()) sym = it->second;
        }
        if (!sym) {
            return sendError(ADS_ERR_NO_IO, request, response, responseLength);
        }
        if (sym->readOnly) {
            return sendError(ADS_ERR_NO_IO, request, response, responseLength);
        }

        const uint8_t* wdata = data + sizeof(AdsReadWriteRequest);
        if (rwReq.cbWriteLength > 0) {
            std::vector<uint8_t> newData(wdata, wdata + rwReq.cbWriteLength);
            if (sym->writeCallback) {
                if (!sym->writeCallback(newData)) {
                    return sendError(ADS_ERR_NO_IO, request, response,
                                     responseLength);
                }
            } else {
                const size_t n = std::min(sym->data.size(), newData.size());
                if (n > 0) memcpy(sym->data.data(), newData.data(), n);
            }
        }

        std::vector<uint8_t> readData;
        if (sym->readCallback) {
            readData = sym->readCallback();
        } else {
            readData = sym->data;
        }
        if (readData.size() > rwReq.cbReadLength) {
            readData.resize(rwReq.cbReadLength);
        }

        // Response: errorCode + cbLength + read data (same layout as READ).
        size_t bufCap = responseLength;
        const size_t totalNeeded = sizeof(AmsHeader) + 8 + readData.size();
        if (totalNeeded > bufCap) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }

        AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_READ_WRITE);
        respHeader.cbData = sizeof(uint32_t) + sizeof(uint32_t) +
                            static_cast<uint32_t>(readData.size());
        memcpy(response, &respHeader, sizeof(respHeader));

        size_t offset = sizeof(respHeader);
        uint32_t errorCode = ADS_ERR_NOERROR;
        memcpy(response + offset, &errorCode, sizeof(errorCode));
        offset += sizeof(errorCode);
        uint32_t cbLen = static_cast<uint32_t>(readData.size());
        memcpy(response + offset, &cbLen, sizeof(cbLen));
        offset += sizeof(cbLen);
        if (!readData.empty()) {
            memcpy(response + offset, readData.data(), readData.size());
            offset += readData.size();
        }
        responseLength = offset;
        return true;
    }

    bool handleAddNotification(const AmsHeader& request, const uint8_t* data, size_t length,
                              uint8_t* response, size_t& responseLength) {
        if (length < sizeof(AdsAddDeviceNotification)) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }

        AdsAddDeviceNotification addReq;
        memcpy(&addReq, data, sizeof(addReq));

        // Resolve the monitored symbol (flat address space, handle decode,
        // or trailing NUL-terminated name for SYM_VAL_BYNAME).
        AdsSymbol* sym = resolveSymbol(addReq.indexGroup, addReq.indexOffset,
                                       data + sizeof(addReq),
                                       length - sizeof(addReq));
        if (!sym) {
            return sendError(ADS_ERR_NO_IO, request, response, responseLength);
        }
        if (addReq.length == 0 || addReq.length > sym->data.size() + 4096) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }

        Notification sub;
        sub.handle = nextNotificationHandle_++;
        sub.target = request.source;
        sub.indexGroup = addReq.indexGroup;
        sub.indexOffset = addReq.indexOffset;
        sub.length = std::min<uint32_t>(addReq.length,
                                        static_cast<uint32_t>(sym->data.size()));
        sub.cycle100ns = addReq.cycle;
        sub.flags = addReq.flags;
        sub.symbolName = sym->name;
        sub.lastData.assign(sym->data.begin(),
                            sym->data.begin() + sub.length);
        sub.hasSample = true;
        sub.nextDue = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lk(notifMutex_);
            notifications_.push_back(sub);
        }
        ensureNotificationThread();

        // Response: errorCode + notificationHandle
        AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_ADD_DEVICE_NOTIFICATION);
        respHeader.cbData = 2 * sizeof(uint32_t);
        memcpy(response, &respHeader, sizeof(respHeader));
        size_t offset = sizeof(respHeader);
        uint32_t errorCode = ADS_ERR_NOERROR;
        memcpy(response + offset, &errorCode, sizeof(errorCode));
        offset += sizeof(errorCode);
        memcpy(response + offset, &sub.handle, sizeof(sub.handle));
        offset += sizeof(sub.handle);
        responseLength = offset;
        return true;
    }

    bool handleDelNotification(const AmsHeader& request, const uint8_t* data, size_t length,
                              uint8_t* response, size_t& responseLength) {
        if (length < sizeof(AdsDelDeviceNotification)) {
            return sendError(ADS_ERR_INTERNAL, request, response, responseLength);
        }

        AdsDelDeviceNotification delReq;
        memcpy(&delReq, data, sizeof(delReq));

        bool found = false;
        {
            std::lock_guard<std::mutex> lk(notifMutex_);
            auto it = std::find_if(notifications_.begin(), notifications_.end(),
                [&](const Notification& n) {
                    return n.handle == delReq.notificationHandle &&
                           n.target.netId == request.source.netId &&
                           n.target.port == request.source.port;
                });
            if (it != notifications_.end()) {
                notifications_.erase(it);
                found = true;
            }
        }

        AmsHeader respHeader = buildResponseHeader(request, ADS_CMD_DEL_DEVICE_NOTIFICATION);
        respHeader.cbData = sizeof(uint32_t);
        memcpy(response, &respHeader, sizeof(respHeader));
        uint32_t errorCode = found ? ADS_ERR_NOERROR : ADS_ERR_INTERNAL;
        memcpy(response + sizeof(respHeader), &errorCode, sizeof(errorCode));
        responseLength = sizeof(respHeader) + sizeof(errorCode);
        return true;
    }
    
    AmsHeader buildResponseHeader(const AmsHeader& request, uint16_t cmdId) {
        AmsHeader resp;
        resp.target = request.source;
        resp.source.netId = localNetId_;
        resp.source.port = localPort_;
        resp.cmdId = cmdId;
        resp.stateFlags = 0x0005;  // Response + ADS command
        resp.cbData = 0;
        resp.errorCode = 0;
        resp.invokeId = request.invokeId;
        return resp;
    }
    
    bool sendError(uint32_t errorCode, const AmsHeader& request,
                  uint8_t* response, size_t& responseLength) {
        AmsHeader respHeader = buildResponseHeader(request, request.cmdId);
        respHeader.errorCode = errorCode;
        respHeader.cbData = sizeof(uint32_t);
        
        memcpy(response, &respHeader, sizeof(respHeader));
        memcpy(response + sizeof(respHeader), &errorCode, sizeof(errorCode));
        
        responseLength = sizeof(respHeader) + sizeof(errorCode);
        return true;
    }

    // Resolve a symbol from {indexGroup, indexOffset} plus optional
    // trailing NUL-terminated name bytes (used by SYM_VAL_BYNAME).
    AdsSymbol* resolveSymbol(uint32_t indexGroup, uint32_t indexOffset,
                             const uint8_t* nameData, size_t nameAvail) {
        if (indexGroup == ADSIGRP_SYM_VAL_BYNAME && nameAvail > 0) {
            const size_t nl = strnlen(reinterpret_cast<const char*>(nameData),
                                      nameAvail);
            if (nl > 0 && nl < nameAvail) {
                auto it = symbols_.find(std::string(
                    reinterpret_cast<const char*>(nameData), nl));
                if (it != symbols_.end()) return &it->second;
            }
        }
        if (indexGroup == ADSIGRP_SYM_VAL_BYHANDLE) {
            // Handle value (carried in indexOffset) encodes
            // (group << 16) | offset.
            const uint32_t handle = indexOffset;
            auto hit = symbolsByAddr_.find({handle >> 16,
                                            handle & 0xFFFFu});
            if (hit != symbolsByAddr_.end()) return hit->second;
        }
        auto it = symbolsByAddr_.find({indexGroup, indexOffset});
        return (it != symbolsByAddr_.end()) ? it->second : nullptr;
    }

    // ---- Device notification machinery ----

    struct Notification {
        uint32_t handle{0};
        AmsAddr target{};
        uint32_t indexGroup{0};
        uint32_t indexOffset{0};
        uint32_t length{0};
        uint32_t cycle100ns{0};   // 0 = on-change only
        uint16_t flags{0};        // bit0: append timestamp
        std::string symbolName;
        std::vector<uint8_t> lastData;
        bool hasSample{false};
        std::chrono::steady_clock::time_point nextDue{};
    };

    void ensureNotificationThread() {
        std::lock_guard<std::mutex> lk(notifMutex_);
        if (notifThread_.joinable()) return;
        notifStop_.store(false);
        notifThread_ = std::thread([this] { notificationWorker(); });
    }

    void stopNotificationThread() {
        notifStop_.store(true);
        if (notifThread_.joinable()) notifThread_.join();
    }

    void notificationWorker() {
        while (!notifStop_.load()) {
            const auto now = std::chrono::steady_clock::now();
            std::vector<Notification> due;
            {
                std::lock_guard<std::mutex> lk(notifMutex_);
                for (auto& n : notifications_) {
                    if (n.cycle100ns == 0 && n.hasSample) {
                        // On-change subscriptions are polled at the worker
                        // rate; emit when the sampled value differs.
                        n.nextDue = now + std::chrono::milliseconds(10);
                    }
                    if (now >= n.nextDue) due.push_back(n);
                }
            }
            for (const auto& n : due) {
                // Re-resolve by name each cycle so re-registered symbols
                // keep working; fall back to the stored address.
                AdsSymbol* sym = nullptr;
                {
                    std::lock_guard<std::mutex> lk(notifMutex_);
                    auto it = symbols_.find(n.symbolName);
                    if (it != symbols_.end()) sym = &it->second;
                }
                if (!sym) continue;

                std::vector<uint8_t> cur;
                if (sym->readCallback) cur = sym->readCallback();
                else cur = sym->data;
                if (n.length <= cur.size()) cur.resize(n.length);
                else cur.resize(n.length, 0);

                const bool changed = !n.hasSample || cur != n.lastData;
                const bool cyclic = (n.cycle100ns != 0);
                if (!changed && !cyclic) {
                    // schedule next poll
                    std::lock_guard<std::mutex> lk(notifMutex_);
                    for (auto& m : notifications_) {
                        if (m.handle == n.handle) {
                            m.nextDue = now + std::chrono::milliseconds(10);
                        }
                    }
                    continue;
                }

                // Build the unsolicited DEVICE_NOTIFICATION AMS message.
                std::vector<uint8_t> msg(sizeof(AmsHeader) + 12 + cur.size() + 8);
                AmsHeader hdr{};
                hdr.target = n.target;
                hdr.source.netId = localNetId_;
                hdr.source.port = localPort_;
                hdr.cmdId = ADS_CMD_DEVICE_NOTIFICATION;
                hdr.stateFlags = 0x0005;
                hdr.cbData = 12 + static_cast<uint32_t>(cur.size()) +
                             ((n.flags & 0x01) ? 8 : 0);
                hdr.invokeId = n.handle;
                memcpy(msg.data(), &hdr, sizeof(hdr));
                size_t off = sizeof(hdr);
                uint32_t err = ADS_ERR_NOERROR;
                memcpy(msg.data() + off, &err, 4); off += 4;
                memcpy(msg.data() + off, &n.handle, 4); off += 4;
                uint32_t cl = static_cast<uint32_t>(cur.size());
                memcpy(msg.data() + off, &cl, 4); off += 4;
                memcpy(msg.data() + off, cur.data(), cur.size());
                off += cur.size();
                if (n.flags & 0x01) {
                    const uint64_t ts = static_cast<uint64_t>(
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            now.time_since_epoch()).count()) * 10ULL;
                    memcpy(msg.data() + off, &ts, 8);
                    off += 8;
                }
                msg.resize(off);

                {
                    std::lock_guard<std::mutex> lk(pendingMutex_);
                    pendingResponses_.push_back(std::move(msg));
                }

                // Update last-sample and schedule next emission.
                std::lock_guard<std::mutex> lk(notifMutex_);
                for (auto& m : notifications_) {
                    if (m.handle == n.handle) {
                        m.lastData = cur;
                        m.hasSample = true;
                        m.nextDue = cyclic
                            ? now + std::chrono::nanoseconds(
                                  static_cast<int64_t>(n.cycle100ns) * 100)
                            : now + std::chrono::milliseconds(10);
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

private:
    SlaveCore* core_;
    
    AmsNetId localNetId_;
    uint16_t localPort_;
    
    uint16_t adsState_;
    uint16_t deviceState_;
    
    AmsAddr currentSource_;
    uint32_t currentInvokeId_;
    
    std::map<std::string, AdsSymbol> symbols_;
    std::map<std::pair<uint32_t, uint32_t>, AdsSymbol*> symbolsByAddr_;

    // Device notifications
    std::vector<Notification> notifications_;
    std::deque<std::vector<uint8_t>> pendingResponses_;
    mutable std::mutex notifMutex_;
    mutable std::mutex pendingMutex_;
    std::thread notifThread_;
    std::atomic<bool> notifStop_{false};
    uint32_t nextNotificationHandle_{1};
};

// Factory function
std::unique_ptr<IMailboxHandler> createAoEHandler(SlaveCore* core) {
    return std::make_unique<AoEHandler>(core);
}

// Factory overload matching header declaration (no SlaveCore)
std::unique_ptr<IMailboxHandler> createAoEHandler() {
    return std::make_unique<AoEHandler>(nullptr);
}

}  // namespace slave
}  // namespace EtherCAT
