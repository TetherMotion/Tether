/**
 * @file AoEHandler.cpp
 * @brief ADS over EtherCAT (AoE) Mailbox Handler Implementation
 * 
 * Implements Beckhoff ADS protocol over EtherCAT for TwinCAT compatibility.
 */

#include "slave/mailbox/IMailboxHandler.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/logging/SlaveLogger.hpp"

#include <cstring>
#include <functional>
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
    
    ~AoEHandler() override = default;
    
    MailboxProtocol getProtocol() const override { return MailboxProtocol::AoE; }
    const char* getProtocolName() const override { return "AoE"; }
    
    void reset() override {
        symbols_.clear();
        symbolsByAddr_.clear();
        adsState_ = ADS_STATE_RUN;
        deviceState_ = 0;
    }
    
    bool processRequest(const uint8_t* data, size_t length,
                       uint8_t* response, size_t& responseLength) override {
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
        
        symbols_[name] = sym;
        symbolsByAddr_[{indexGroup, indexOffset}] = &symbols_[name];
    }
    
    void setSymbolData(const std::string& name, const std::vector<uint8_t>& data) {
        auto it = symbols_.find(name);
        if (it != symbols_.end() && data.size() == it->second.size) {
            it->second.data = data;
        }
    }
    
    std::vector<uint8_t> getSymbolData(const std::string& name) const {
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
        
        if (readReq.indexGroup == ADSIGRP_SYM_VAL_BYNAME) {
            // Read by symbol name (name follows the request)
            // Not implemented in this simple version
            return sendError(ADS_ERR_NO_IO, request, response, responseLength);
        }
        
        auto it = symbolsByAddr_.find({readReq.indexGroup, readReq.indexOffset});
        if (it != symbolsByAddr_.end() && it->second) {
            AdsSymbol* sym = it->second;
            if (sym->readCallback) {
                readData = sym->readCallback();
            } else {
                readData = sym->data;
            }
            
            if (readData.size() > readReq.cbLength) {
                readData.resize(readReq.cbLength);
            }
        } else {
            // Try direct memory access
            // For now, return error
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
        
        // Find symbol and write
        auto it = symbolsByAddr_.find({writeReq.indexGroup, writeReq.indexOffset});
        if (it != symbolsByAddr_.end() && it->second) {
            AdsSymbol* sym = it->second;
            
            if (sym->readOnly) {
                return sendError(ADS_ERR_NO_IO, request, response, responseLength);
            }
            
            std::vector<uint8_t> newData(writeData, writeData + writeReq.cbLength);
            
            if (sym->writeCallback) {
                if (!sym->writeCallback(newData)) {
                    return sendError(ADS_ERR_NO_IO, request, response, responseLength);
                }
            } else {
                if (newData.size() <= sym->data.size()) {
                    memcpy(sym->data.data(), newData.data(), newData.size());
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
        
        // Generic read/write - not fully implemented
        return sendError(ADS_ERR_NO_IO, request, response, responseLength);
    }
    
    bool handleAddNotification(const AmsHeader& request, const uint8_t* data, size_t length,
                              uint8_t* response, size_t& responseLength) {
        // Notifications not implemented
        return sendError(ADS_ERR_NO_IO, request, response, responseLength);
    }
    
    bool handleDelNotification(const AmsHeader& request, const uint8_t* data, size_t length,
                              uint8_t* response, size_t& responseLength) {
        // Notifications not implemented
        return sendError(ADS_ERR_NO_IO, request, response, responseLength);
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
