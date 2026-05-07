/**
 * @file FoEHandler.cpp
 * @brief File over EtherCAT (FoE) Mailbox Handler Implementation
 */

#include "slave/mailbox/IMailboxHandler.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/core/SlaveTypes.hpp"
#include "slave/logging/SlaveLogger.hpp"

#include <cstring>
#include <functional>
#include <vector>
#include <map>

namespace EtherCAT {
namespace slave {

// ============================================================================
// FoE Constants
// ============================================================================

namespace {
    // FoE OpCodes
    constexpr uint8_t FOE_OP_READ_REQUEST = 1;
    constexpr uint8_t FOE_OP_WRITE_REQUEST = 2;
    constexpr uint8_t FOE_OP_DATA = 3;
    constexpr uint8_t FOE_OP_ACK = 4;
    constexpr uint8_t FOE_OP_ERROR = 5;
    constexpr uint8_t FOE_OP_BUSY = 6;
    
    // FoE Error Codes
    constexpr uint32_t FOE_ERR_NOT_DEFINED = 0x8000;
    constexpr uint32_t FOE_ERR_NOT_FOUND = 0x8001;
    constexpr uint32_t FOE_ERR_ACCESS_DENIED = 0x8002;
    constexpr uint32_t FOE_ERR_DISK_FULL = 0x8003;
    constexpr uint32_t FOE_ERR_ILLEGAL = 0x8004;
    constexpr uint32_t FOE_ERR_PACKET_NUMBER = 0x8005;
    constexpr uint32_t FOE_ERR_ALREADY_EXISTS = 0x8006;
    constexpr uint32_t FOE_ERR_NO_USER = 0x8007;
    constexpr uint32_t FOE_ERR_BOOTSTRAP_ONLY = 0x8008;
    constexpr uint32_t FOE_ERR_NOT_IN_BOOTSTRAP = 0x8009;
    constexpr uint32_t FOE_ERR_NO_RIGHTS = 0x800A;
    constexpr uint32_t FOE_ERR_PROGRAM_ERROR = 0x800B;
}

// ============================================================================
// Virtual File System
// ============================================================================

/**
 * Virtual file for FoE transfers
 */
struct VirtualFile {
    std::string name;
    std::vector<uint8_t> data;
    bool readOnly;
    bool bootstrapOnly;
    
    // Callbacks for dynamic files
    std::function<std::vector<uint8_t>()> readCallback;
    std::function<bool(const std::vector<uint8_t>&)> writeCallback;
};

// ============================================================================
// FoE Transfer State
// ============================================================================

struct FoETransferState {
    bool active{false};
    bool isRead{false};
    std::string fileName;
    std::vector<uint8_t> data;
    size_t offset{0};
    uint32_t packetNumber{0};
    uint32_t password{0};
};

// ============================================================================
// FoEHandler Implementation
// ============================================================================

class FoEHandler : public IMailboxHandler {
public:
    explicit FoEHandler(SlaveCore* core)
        : core_(core)
    {
        // Register standard files
        registerFile("firmware.bin", {}, false, true);  // Firmware (bootstrap only)
        registerFile("config.bin", {}, false, false);   // Configuration
        registerFile("eeprom.bin", {}, false, false);   // EEPROM content
    }
    
    ~FoEHandler() override = default;
    
    MailboxProtocol getProtocol() const override { return MailboxProtocol::FoE; }
    const char* getProtocolName() const override { return "FoE"; }
    
    void reset() override {
        transfer_.active = false;
        transfer_.data.clear();
        transfer_.offset = 0;
        transfer_.packetNumber = 0;
        transfer_.fileName.clear();
        transfer_.isRead = false;
    }
    
    bool processRequest(const uint8_t* data, size_t length,
                       uint8_t* response, size_t& responseLength) override {
        if (length < 6) {
            return sendError(FOE_ERR_ILLEGAL, "Invalid request", response, responseLength);
        }
        
        uint8_t opCode = data[0];
        
        switch (opCode) {
            case FOE_OP_READ_REQUEST:
                return handleReadRequest(data, length, response, responseLength);
                
            case FOE_OP_WRITE_REQUEST:
                return handleWriteRequest(data, length, response, responseLength);
                
            case FOE_OP_DATA:
                return handleData(data, length, response, responseLength);
                
            case FOE_OP_ACK:
                return handleAck(data, length, response, responseLength);
                
            default:
                return sendError(FOE_ERR_ILLEGAL, "Unknown opcode", response, responseLength);
        }
    }
    
    // Transfer management (not part of IMailboxHandler, but useful)
    bool isTransferActive() const {
        return transfer_.active;
    }
    
    void abortTransfer() {
        transfer_.active = false;
        transfer_.data.clear();
        transfer_.offset = 0;
    }
    
    // File management
    void registerFile(const std::string& name, const std::vector<uint8_t>& data,
                     bool readOnly = false, bool bootstrapOnly = false) {
        VirtualFile file;
        file.name = name;
        file.data = data;
        file.readOnly = readOnly;
        file.bootstrapOnly = bootstrapOnly;
        files_[name] = file;
    }
    
    void registerDynamicFile(const std::string& name,
                            std::function<std::vector<uint8_t>()> readCallback,
                            std::function<bool(const std::vector<uint8_t>&)> writeCallback) {
        VirtualFile file;
        file.name = name;
        file.readOnly = (writeCallback == nullptr);
        file.bootstrapOnly = false;
        file.readCallback = readCallback;
        file.writeCallback = writeCallback;
        files_[name] = file;
    }
    
    void setFileData(const std::string& name, const std::vector<uint8_t>& data) {
        auto it = files_.find(name);
        if (it != files_.end()) {
            it->second.data = data;
        }
    }
    
    std::vector<uint8_t> getFileData(const std::string& name) const {
        auto it = files_.find(name);
        if (it != files_.end()) {
            if (it->second.readCallback) {
                return it->second.readCallback();
            }
            return it->second.data;
        }
        return {};
    }
    
private:
    bool handleReadRequest(const uint8_t* data, size_t length,
                          uint8_t* response, size_t& responseLength) {
        // Extract password (4 bytes after opcode)
        uint32_t password = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
        
        // Extract filename (null-terminated string after password)
        std::string fileName;
        if (length > 6) {
            fileName = std::string(reinterpret_cast<const char*>(data + 6), length - 6);
            // Remove null terminator if present
            size_t nullPos = fileName.find('\0');
            if (nullPos != std::string::npos) {
                fileName = fileName.substr(0, nullPos);
            }
        }
        
        // Find file
        auto it = files_.find(fileName);
        if (it == files_.end()) {
            return sendError(FOE_ERR_NOT_FOUND, "File not found", response, responseLength);
        }
        
        VirtualFile& file = it->second;
        
        // Check bootstrap restriction
        if (file.bootstrapOnly && core_ && core_->getState() != SlaveState::BOOT) {
            return sendError(FOE_ERR_NOT_FOUND, "File only available in Bootstrap", 
                           response, responseLength);
        }
        
        // Start transfer
        transfer_.active = true;
        transfer_.isRead = true;
        transfer_.fileName = fileName;
        transfer_.password = password;
        transfer_.packetNumber = 1;
        transfer_.offset = 0;
        
        // Get file data
        if (file.readCallback) {
            transfer_.data = file.readCallback();
        } else {
            transfer_.data = file.data;
        }
        
        log(SlaveLogLevel::Info, SlaveLogCategory::FoE, 
            "Read request for '{}', {} bytes", fileName, transfer_.data.size());
        
        // Send first data packet
        return sendDataPacket(response, responseLength);
    }
    
    bool handleWriteRequest(const uint8_t* data, size_t length,
                           uint8_t* response, size_t& responseLength) {
        // Extract password
        uint32_t password = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
        
        // Extract filename
        std::string fileName;
        if (length > 6) {
            fileName = std::string(reinterpret_cast<const char*>(data + 6), length - 6);
            size_t nullPos = fileName.find('\0');
            if (nullPos != std::string::npos) {
                fileName = fileName.substr(0, nullPos);
            }
        }
        
        // Find or create file
        auto it = files_.find(fileName);
        if (it != files_.end()) {
            VirtualFile& file = it->second;
            
            if (file.readOnly) {
                return sendError(FOE_ERR_ACCESS_DENIED, "File is read-only", 
                               response, responseLength);
            }
            
            if (file.bootstrapOnly && core_ && core_->getState() != SlaveState::BOOT) {
                return sendError(FOE_ERR_ACCESS_DENIED, "File only writable in Bootstrap",
                               response, responseLength);
            }
        }
        
        // Start transfer
        transfer_.active = true;
        transfer_.isRead = false;
        transfer_.fileName = fileName;
        transfer_.password = password;
        transfer_.packetNumber = 0;
        transfer_.offset = 0;
        transfer_.data.clear();
        
        log(SlaveLogLevel::Info, SlaveLogCategory::FoE, "Write request for '{}'", fileName);
        
        // Send ACK for packet 0 (ready to receive)
        return sendAck(0, response, responseLength);
    }
    
    bool handleData(const uint8_t* data, size_t length,
                   uint8_t* response, size_t& responseLength) {
        if (!transfer_.active || transfer_.isRead) {
            return sendError(FOE_ERR_ILLEGAL, "No write transfer active", 
                           response, responseLength);
        }
        
        // Extract packet number
        uint32_t packetNum = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
        
        // Check packet number
        if (packetNum != transfer_.packetNumber + 1) {
            return sendError(FOE_ERR_PACKET_NUMBER, "Wrong packet number",
                           response, responseLength);
        }
        
        transfer_.packetNumber = packetNum;
        
        // Append data
        size_t dataLen = length - 6;
        const uint8_t* fileData = data + 6;
        transfer_.data.insert(transfer_.data.end(), fileData, fileData + dataLen);
        
        // Check if transfer complete (short packet)
        size_t maxDataSize = 512 - 6;  // Typical mailbox size - header
        if (dataLen < maxDataSize) {
            // Transfer complete
            completeWriteTransfer();
        }
        
        // Send ACK
        return sendAck(packetNum, response, responseLength);
    }
    
    bool handleAck(const uint8_t* data, size_t length,
                  uint8_t* response, size_t& responseLength) {
        if (!transfer_.active || !transfer_.isRead) {
            return sendError(FOE_ERR_ILLEGAL, "No read transfer active",
                           response, responseLength);
        }
        
        // Extract acknowledged packet number
        uint32_t ackPacket = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24);
        
        if (ackPacket != transfer_.packetNumber) {
            return sendError(FOE_ERR_PACKET_NUMBER, "Wrong ACK number",
                           response, responseLength);
        }
        
        // Check if transfer complete
        if (transfer_.offset >= transfer_.data.size()) {
            log(SlaveLogLevel::Info, SlaveLogCategory::FoE, 
                "Read transfer complete: '{}', {} bytes", 
                transfer_.fileName, transfer_.data.size());
            transfer_.active = false;
            responseLength = 0;
            return true;
        }
        
        // Send next data packet
        transfer_.packetNumber++;
        return sendDataPacket(response, responseLength);
    }
    
    void completeWriteTransfer() {
        // Find file and update
        auto it = files_.find(transfer_.fileName);
        if (it != files_.end()) {
            VirtualFile& file = it->second;
            
            if (file.writeCallback) {
                if (!file.writeCallback(transfer_.data)) {
                    log(SlaveLogLevel::Error, SlaveLogCategory::FoE,
                        "Write callback failed for '{}'", transfer_.fileName);
                }
            } else {
                file.data = transfer_.data;
            }
        } else {
            // Create new file
            VirtualFile file;
            file.name = transfer_.fileName;
            file.data = transfer_.data;
            file.readOnly = false;
            file.bootstrapOnly = false;
            files_[transfer_.fileName] = file;
        }
        
        log(SlaveLogLevel::Info, SlaveLogCategory::FoE,
            "Write transfer complete: '{}', {} bytes",
            transfer_.fileName, transfer_.data.size());
        
        transfer_.active = false;
    }
    
    bool sendDataPacket(uint8_t* response, size_t& responseLength) {
        size_t maxDataSize = 512 - 6;  // Adjust based on mailbox size
        size_t remaining = transfer_.data.size() - transfer_.offset;
        size_t dataLen = std::min(remaining, maxDataSize);
        
        response[0] = FOE_OP_DATA;
        response[1] = 0;  // Reserved
        response[2] = transfer_.packetNumber & 0xFF;
        response[3] = (transfer_.packetNumber >> 8) & 0xFF;
        response[4] = (transfer_.packetNumber >> 16) & 0xFF;
        response[5] = (transfer_.packetNumber >> 24) & 0xFF;
        
        memcpy(response + 6, transfer_.data.data() + transfer_.offset, dataLen);
        transfer_.offset += dataLen;
        
        responseLength = 6 + dataLen;
        return true;
    }
    
    bool sendAck(uint32_t packetNumber, uint8_t* response, size_t& responseLength) {
        response[0] = FOE_OP_ACK;
        response[1] = 0;
        response[2] = packetNumber & 0xFF;
        response[3] = (packetNumber >> 8) & 0xFF;
        response[4] = (packetNumber >> 16) & 0xFF;
        response[5] = (packetNumber >> 24) & 0xFF;
        
        responseLength = 6;
        return true;
    }
    
    bool sendError(uint32_t errorCode, const char* errorText,
                  uint8_t* response, size_t& responseLength) {
        response[0] = FOE_OP_ERROR;
        response[1] = 0;
        response[2] = errorCode & 0xFF;
        response[3] = (errorCode >> 8) & 0xFF;
        response[4] = (errorCode >> 16) & 0xFF;
        response[5] = (errorCode >> 24) & 0xFF;
        
        size_t textLen = strlen(errorText);
        memcpy(response + 6, errorText, textLen + 1);
        
        responseLength = 6 + textLen + 1;
        
        transfer_.active = false;
        
        log(SlaveLogLevel::Warning, SlaveLogCategory::FoE,
            "FoE error 0x{:08X}: {}", errorCode, errorText);
        
        return true;
    }
    
    void log(SlaveLogLevel level, SlaveLogCategory category, 
            const char* fmt, auto&&... args) {
        if (core_) {
            // Use core's logger
        }
    }
    
private:
    SlaveCore* core_;
    std::map<std::string, VirtualFile> files_;
    FoETransferState transfer_;
};

// Factory function
std::unique_ptr<IMailboxHandler> createFoEHandler(SlaveCore* core) {
    return std::make_unique<FoEHandler>(core);
}

// Factory overload matching header declaration (no SlaveCore)
std::unique_ptr<IMailboxHandler> createFoEHandler(const std::string& rootPath) {
    (void)rootPath;
    return std::make_unique<FoEHandler>(nullptr);
}

}  // namespace slave
}  // namespace EtherCAT
