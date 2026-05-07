/**
 * @file VoEHandler.cpp
 * @brief Vendor-specific over EtherCAT (VoE) Mailbox Handler Implementation
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
// VoE Message Structure
// ============================================================================

/**
 * VoE message header structure
 */
struct VoEHeader {
    uint32_t vendorId;        // Vendor ID (from SII)
    uint16_t vendorType;      // Vendor-specific type/subcode
    uint8_t flags;            // Message flags
    uint8_t reserved;
};

/**
 * Registered VoE handler
 */
struct VoEHandlerEntry {
    uint16_t vendorType;
    std::function<bool(const uint8_t*, size_t, uint8_t*, size_t&)> handler;
    std::string description;
};

// ============================================================================
// VoEHandler Implementation
// ============================================================================

class VoEHandler : public IMailboxHandler {
public:
    explicit VoEHandler(SlaveCore* core, uint32_t vendorId)
        : core_(core)
        , vendorId_(vendorId)
    {
        // Register some example vendor-specific handlers
        registerVendorHandler(0x0001, "Echo",
            [](const uint8_t* data, size_t length, uint8_t* response, size_t& responseLength) {
                // Simple echo: return same data
                memcpy(response, data, length);
                responseLength = length;
                return true;
            });
        
        registerVendorHandler(0x0002, "Device Info",
            [this](const uint8_t* data, size_t length, uint8_t* response, size_t& responseLength) {
                return handleDeviceInfoRequest(data, length, response, responseLength);
            });
        
        registerVendorHandler(0x0003, "Debug Command",
            [this](const uint8_t* data, size_t length, uint8_t* response, size_t& responseLength) {
                return handleDebugCommand(data, length, response, responseLength);
            });
        
        registerVendorHandler(0x0004, "Firmware Update",
            [this](const uint8_t* data, size_t length, uint8_t* response, size_t& responseLength) {
                return handleFirmwareUpdate(data, length, response, responseLength);
            });
        
        registerVendorHandler(0x0005, "Configuration",
            [this](const uint8_t* data, size_t length, uint8_t* response, size_t& responseLength) {
                return handleConfiguration(data, length, response, responseLength);
            });
    }
    
    ~VoEHandler() override = default;
    
    MailboxProtocol getProtocol() const override { return MailboxProtocol::VoE; }
    const char* getProtocolName() const override { return "VoE"; }
    
    void reset() override {
        pendingTransfer_ = false;
        transferBuffer_.clear();
    }
    
    bool processRequest(const uint8_t* data, size_t length,
                       uint8_t* response, size_t& responseLength) override {
        if (length < sizeof(VoEHeader)) {
            responseLength = 0;
            return false;
        }
        
        // Parse VoE header
        VoEHeader header;
        memcpy(&header, data, sizeof(header));
        
        // Verify vendor ID
        if (header.vendorId != vendorId_ && header.vendorId != 0) {
            // Vendor ID mismatch - ignore or reject
            responseLength = 0;
            return false;
        }
        
        // Find handler for this vendor type
        auto it = handlers_.find(header.vendorType);
        if (it == handlers_.end()) {
            // No handler registered - send error response
            return sendErrorResponse(header.vendorType, 0x0001, "Unknown vendor type",
                                    response, responseLength);
        }
        
        // Prepare response header
        VoEHeader respHeader;
        respHeader.vendorId = vendorId_;
        respHeader.vendorType = header.vendorType;
        respHeader.flags = 0;
        respHeader.reserved = 0;
        
        memcpy(response, &respHeader, sizeof(respHeader));
        
        // Call vendor handler
        size_t dataResponseLength = 0;
        const uint8_t* requestData = data + sizeof(VoEHeader);
        size_t requestLength = length - sizeof(VoEHeader);
        
        bool result = it->second.handler(requestData, requestLength,
                                        response + sizeof(VoEHeader),
                                        dataResponseLength);
        
        responseLength = sizeof(VoEHeader) + dataResponseLength;
        return result;
    }
    
    // Transfer management (not part of IMailboxHandler interface)
    bool isTransferActive() const {
        return pendingTransfer_;
    }
    
    void abortTransfer() {
        pendingTransfer_ = false;
        transferBuffer_.clear();
    }
    
    // Handler registration
    void registerVendorHandler(uint16_t vendorType, const std::string& description,
                              std::function<bool(const uint8_t*, size_t, 
                                                uint8_t*, size_t&)> handler) {
        VoEHandlerEntry entry;
        entry.vendorType = vendorType;
        entry.description = description;
        entry.handler = handler;
        handlers_[vendorType] = entry;
    }
    
    void unregisterVendorHandler(uint16_t vendorType) {
        handlers_.erase(vendorType);
    }
    
    // Vendor ID
    uint32_t getVendorId() const { return vendorId_; }
    void setVendorId(uint32_t vendorId) { vendorId_ = vendorId; }
    
    // Custom data
    void setCustomData(const std::string& key, const std::vector<uint8_t>& data) {
        customData_[key] = data;
    }
    
    std::vector<uint8_t> getCustomData(const std::string& key) const {
        auto it = customData_.find(key);
        if (it != customData_.end()) {
            return it->second;
        }
        return {};
    }
    
    // Notification callback
    void setNotificationCallback(std::function<void(uint16_t, const uint8_t*, size_t)> callback) {
        notificationCallback_ = callback;
    }
    
    // Send unsolicited VoE notification
    bool sendNotification(uint16_t vendorType, const uint8_t* data, size_t length) {
        if (!core_) return false;
        
        // Build notification message
        std::vector<uint8_t> message(sizeof(VoEHeader) + length);
        
        VoEHeader header;
        header.vendorId = vendorId_;
        header.vendorType = vendorType;
        header.flags = 0x01;  // Notification flag
        header.reserved = 0;
        
        memcpy(message.data(), &header, sizeof(header));
        memcpy(message.data() + sizeof(header), data, length);
        
        // Queue for transmission
        // Note: Implementation depends on SlaveCore mailbox TX support
        return true;
    }
    
private:
    bool handleDeviceInfoRequest(const uint8_t* data, size_t length,
                                 uint8_t* response, size_t& responseLength) {
        // Return device information
        struct DeviceInfo {
            uint32_t vendorId;
            uint32_t productCode;
            uint32_t revisionNumber;
            uint32_t serialNumber;
            char deviceName[32];
            char firmwareVersion[16];
            char hardwareVersion[16];
        } __attribute__((packed));
        
        DeviceInfo info{};
        info.vendorId = vendorId_;
        info.productCode = 0x12345678;
        info.revisionNumber = 0x00010002;
        info.serialNumber = 0xABCD1234;
        strncpy(info.deviceName, "EtherCAT Slave Emulator", sizeof(info.deviceName) - 1);
        strncpy(info.firmwareVersion, "1.0.0", sizeof(info.firmwareVersion) - 1);
        strncpy(info.hardwareVersion, "Rev.A", sizeof(info.hardwareVersion) - 1);
        
        memcpy(response, &info, sizeof(info));
        responseLength = sizeof(info);
        return true;
    }
    
    bool handleDebugCommand(const uint8_t* data, size_t length,
                           uint8_t* response, size_t& responseLength) {
        if (length < 1) {
            return sendInlineError(0x0002, "Missing command", response, responseLength);
        }
        
        uint8_t command = data[0];
        
        switch (command) {
            case 0x01:  // Get debug flags
                response[0] = debugFlags_;
                responseLength = 1;
                return true;
                
            case 0x02:  // Set debug flags
                if (length >= 2) {
                    debugFlags_ = data[1];
                    response[0] = 0;  // OK
                    responseLength = 1;
                    return true;
                }
                break;
                
            case 0x03:  // Get statistics
                {
                    struct Stats {
                        uint64_t framesProcessed;
                        uint64_t errorsDetected;
                        uint64_t uptimeSeconds;
                    } stats{};
                    
                    stats.framesProcessed = framesProcessed_;
                    stats.errorsDetected = errorsDetected_;
                    stats.uptimeSeconds = core_ ? core_->getDCSystemTime() / 1000000000ULL : 0;
                    
                    memcpy(response, &stats, sizeof(stats));
                    responseLength = sizeof(stats);
                    return true;
                }
                
            case 0x04:  // Reset statistics
                framesProcessed_ = 0;
                errorsDetected_ = 0;
                response[0] = 0;
                responseLength = 1;
                return true;
                
            case 0x10:  // Memory dump
                {
                    if (length < 9) {
                        return sendInlineError(0x0003, "Invalid parameters", response, responseLength);
                    }
                    
                    uint32_t address = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
                    uint32_t size = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);
                    
                    if (size > 256) {
                        // size = 256;  // Limit dump size - not used
                    }
                    
                    // Return ESC register content if within range
                    if (address < 0x1000) {
                        // Register dump not supported via VoE
                        return sendInlineError(0x0004, "Register dump not supported", response, responseLength);
                    }
                    
                    return sendInlineError(0x0004, "Invalid address", response, responseLength);
                }
                
            default:
                return sendInlineError(0x0001, "Unknown command", response, responseLength);
        }
        
        return sendInlineError(0x0002, "Invalid parameters", response, responseLength);
    }
    
    bool handleFirmwareUpdate(const uint8_t* data, size_t length,
                             uint8_t* response, size_t& responseLength) {
        if (length < 1) {
            return sendInlineError(0x0010, "Missing subcommand", response, responseLength);
        }
        
        uint8_t subCommand = data[0];
        
        switch (subCommand) {
            case 0x01:  // Start update
                if (core_ && core_->getState() != SlaveState::BOOT) {
                    return sendInlineError(0x0011, "Not in Bootstrap mode", response, responseLength);
                }
                pendingTransfer_ = true;
                transferBuffer_.clear();
                response[0] = 0;
                responseLength = 1;
                return true;
                
            case 0x02:  // Data chunk
                if (!pendingTransfer_) {
                    return sendInlineError(0x0012, "No transfer active", response, responseLength);
                }
                transferBuffer_.insert(transferBuffer_.end(), data + 1, data + length);
                response[0] = 0;
                responseLength = 1;
                return true;
                
            case 0x03:  // End update
                if (!pendingTransfer_) {
                    return sendInlineError(0x0012, "No transfer active", response, responseLength);
                }
                // Process firmware data...
                pendingTransfer_ = false;
                response[0] = 0;
                responseLength = 1;
                return true;
                
            case 0x04:  // Abort update
                pendingTransfer_ = false;
                transferBuffer_.clear();
                response[0] = 0;
                responseLength = 1;
                return true;
                
            default:
                return sendInlineError(0x0010, "Unknown subcommand", response, responseLength);
        }
    }
    
    bool handleConfiguration(const uint8_t* data, size_t length,
                            uint8_t* response, size_t& responseLength) {
        if (length < 1) {
            return sendInlineError(0x0020, "Missing subcommand", response, responseLength);
        }
        
        uint8_t subCommand = data[0];
        
        switch (subCommand) {
            case 0x01:  // Read config
                {
                    auto configData = getCustomData("config");
                    memcpy(response, configData.data(), configData.size());
                    responseLength = configData.size();
                    return true;
                }
                
            case 0x02:  // Write config
                {
                    std::vector<uint8_t> configData(data + 1, data + length);
                    setCustomData("config", configData);
                    response[0] = 0;
                    responseLength = 1;
                    return true;
                }
                
            case 0x03:  // Reset to defaults
                customData_.erase("config");
                response[0] = 0;
                responseLength = 1;
                return true;
                
            case 0x04:  // Save to persistent storage
                // Implementation depends on hardware
                response[0] = 0;
                responseLength = 1;
                return true;
                
            default:
                return sendInlineError(0x0020, "Unknown subcommand", response, responseLength);
        }
    }
    
    bool sendErrorResponse(uint16_t vendorType, uint16_t errorCode, const char* errorText,
                          uint8_t* response, size_t& responseLength) {
        VoEHeader header;
        header.vendorId = vendorId_;
        header.vendorType = vendorType;
        header.flags = 0x80;  // Error flag
        header.reserved = 0;
        
        memcpy(response, &header, sizeof(header));
        
        response[sizeof(header)] = errorCode & 0xFF;
        response[sizeof(header) + 1] = (errorCode >> 8) & 0xFF;
        
        size_t textLen = strlen(errorText);
        memcpy(response + sizeof(header) + 2, errorText, textLen + 1);
        
        responseLength = sizeof(header) + 2 + textLen + 1;
        return true;
    }
    
    bool sendInlineError(uint16_t errorCode, const char* errorText,
                        uint8_t* response, size_t& responseLength) {
        response[0] = 0xFF;  // Error marker
        response[1] = errorCode & 0xFF;
        response[2] = (errorCode >> 8) & 0xFF;
        
        size_t textLen = strlen(errorText);
        memcpy(response + 3, errorText, textLen + 1);
        
        responseLength = 3 + textLen + 1;
        return true;
    }
    
private:
    SlaveCore* core_;
    uint32_t vendorId_;
    
    std::map<uint16_t, VoEHandlerEntry> handlers_;
    std::map<std::string, std::vector<uint8_t>> customData_;
    
    // Transfer state
    bool pendingTransfer_{false};
    std::vector<uint8_t> transferBuffer_;
    
    // Debug/stats
    uint8_t debugFlags_{0};
    uint64_t framesProcessed_{0};
    uint64_t errorsDetected_{0};
    
    // Notification callback
    std::function<void(uint16_t, const uint8_t*, size_t)> notificationCallback_;
};

// Factory function
std::unique_ptr<IMailboxHandler> createVoEHandler(SlaveCore* core, uint32_t vendorId) {
    return std::make_unique<VoEHandler>(core, vendorId);
}

// Factory overload matching header declaration (no SlaveCore)
std::unique_ptr<IMailboxHandler> createVoEHandler() {
    return std::make_unique<VoEHandler>(nullptr, 0);
}

}  // namespace slave
}  // namespace EtherCAT
