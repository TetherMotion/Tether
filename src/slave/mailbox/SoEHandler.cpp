/**
 * @file SoEHandler.cpp
 * @brief Servo Drive Profile over EtherCAT (SoE) Mailbox Handler Implementation
 * 
 * Implements SERCOS-over-EtherCAT for servo drive communication.
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
// SoE Constants
// ============================================================================

namespace {
    // SoE OpCodes
    constexpr uint8_t SOE_OP_READ_REQUEST = 1;
    constexpr uint8_t SOE_OP_READ_RESPONSE = 2;
    constexpr uint8_t SOE_OP_WRITE_REQUEST = 3;
    constexpr uint8_t SOE_OP_WRITE_RESPONSE = 4;
    constexpr uint8_t SOE_OP_NOTIFICATION = 5;
    constexpr uint8_t SOE_OP_EMERGENCY = 6;
    
    // SoE Error Codes
    constexpr uint16_t SOE_ERR_NO_IDN = 0x1001;
    constexpr uint16_t SOE_ERR_NO_NAME = 0x1009;
    constexpr uint16_t SOE_ERR_NO_ATTRIBUTE = 0x100A;
    constexpr uint16_t SOE_ERR_NO_UNIT = 0x100B;
    constexpr uint16_t SOE_ERR_NO_MIN = 0x100C;
    constexpr uint16_t SOE_ERR_NO_MAX = 0x100D;
    constexpr uint16_t SOE_ERR_NO_DATA = 0x100E;
    constexpr uint16_t SOE_ERR_NAME_SHORT = 0x2001;
    constexpr uint16_t SOE_ERR_NAME_LONG = 0x2002;
    constexpr uint16_t SOE_ERR_ATTRIBUTE_RO = 0x2003;
    constexpr uint16_t SOE_ERR_DATA_MIN = 0x2011;
    constexpr uint16_t SOE_ERR_DATA_MAX = 0x2012;
    constexpr uint16_t SOE_ERR_COMMAND = 0x3001;
    
    // SERCOS element flags
    constexpr uint8_t SOE_ELEM_DATA = 0x01;
    constexpr uint8_t SOE_ELEM_NAME = 0x02;
    constexpr uint8_t SOE_ELEM_ATTRIBUTE = 0x04;
    constexpr uint8_t SOE_ELEM_UNIT = 0x08;
    constexpr uint8_t SOE_ELEM_MIN = 0x10;
    constexpr uint8_t SOE_ELEM_MAX = 0x20;
    constexpr uint8_t SOE_ELEM_VALUE = 0x40;
}

// ============================================================================
// IDN (Identification Number) Structure
// ============================================================================

struct SoEIDN {
    uint16_t idn;                  // Identification number
    std::string name;              // Parameter name
    uint32_t attribute;            // Attribute (data type, length, etc.)
    std::string unit;              // Physical unit
    std::vector<uint8_t> minValue; // Minimum value
    std::vector<uint8_t> maxValue; // Maximum value
    std::vector<uint8_t> data;     // Current value
    bool readOnly{false};
    
    // Callback for dynamic IDNs
    std::function<std::vector<uint8_t>()> readCallback;
    std::function<bool(const std::vector<uint8_t>&)> writeCallback;
};

// ============================================================================
// SoEHandler Implementation
// ============================================================================

class SoEHandler : public IMailboxHandler {
public:
    explicit SoEHandler(SlaveCore* core)
        : core_(core)
    {
        // Register standard SERCOS IDNs
        initializeStandardIDNs();
    }
    
    ~SoEHandler() override = default;
    
    MailboxProtocol getProtocol() const override { return MailboxProtocol::SoE; }
    const char* getProtocolName() const override { return "SoE"; }
    
    void reset() override {
        fragmentedTransfer_ = false;
        fragmentBuffer_.clear();
    }
    
    bool processRequest(const uint8_t* data, size_t length,
                       uint8_t* response, size_t& responseLength) override {
        if (length < 4) {
            return sendError(0, SOE_ERR_COMMAND, response, responseLength);
        }
        
        // Parse SoE header
        uint8_t opCode = data[0] & 0x07;
        bool incomplete = (data[0] & 0x08) != 0;
        bool error = (data[0] & 0x10) != 0;
        uint8_t driveNo = data[1];
        uint8_t elements = data[2];
        uint16_t idn = data[3] | (data[4] << 8);
        
        switch (opCode) {
            case SOE_OP_READ_REQUEST:
                return handleReadRequest(driveNo, idn, elements, data + 6, length - 6,
                                        response, responseLength);
                
            case SOE_OP_WRITE_REQUEST:
                return handleWriteRequest(driveNo, idn, elements, data + 6, length - 6,
                                         response, responseLength);
                
            default:
                return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
        }
    }
    
    // Transfer management (not part of IMailboxHandler interface)
    bool isTransferActive() const {
        return fragmentedTransfer_;
    }
    
    void abortTransfer() {
        fragmentedTransfer_ = false;
        fragmentBuffer_.clear();
    }
    
    // IDN Management
    void registerIDN(uint16_t idn, const std::string& name, uint32_t attribute,
                    const std::string& unit, const std::vector<uint8_t>& data,
                    bool readOnly = false) {
        SoEIDN entry;
        entry.idn = idn;
        entry.name = name;
        entry.attribute = attribute;
        entry.unit = unit;
        entry.data = data;
        entry.readOnly = readOnly;
        idnMap_[idn] = entry;
    }
    
    void registerDynamicIDN(uint16_t idn, const std::string& name, uint32_t attribute,
                           std::function<std::vector<uint8_t>()> readCallback,
                           std::function<bool(const std::vector<uint8_t>&)> writeCallback) {
        SoEIDN entry;
        entry.idn = idn;
        entry.name = name;
        entry.attribute = attribute;
        entry.readCallback = readCallback;
        entry.writeCallback = writeCallback;
        entry.readOnly = (writeCallback == nullptr);
        idnMap_[idn] = entry;
    }
    
    void setIDNData(uint16_t idn, const std::vector<uint8_t>& data) {
        auto it = idnMap_.find(idn);
        if (it != idnMap_.end()) {
            it->second.data = data;
        }
    }
    
    std::vector<uint8_t> getIDNData(uint16_t idn) const {
        auto it = idnMap_.find(idn);
        if (it != idnMap_.end()) {
            if (it->second.readCallback) {
                return it->second.readCallback();
            }
            return it->second.data;
        }
        return {};
    }
    
private:
    void initializeStandardIDNs() {
        // P-0-0000: IDN list
        registerIDN(0x0000, "IDN list", 0x00400001, "", {}, true);
        
        // S-0-0001: NC cycle time
        std::vector<uint8_t> cycleTime = {0xE8, 0x03, 0x00, 0x00};  // 1000 us
        registerIDN(0x0001, "NC cycle time", 0x00070001, "us", cycleTime, true);
        
        // S-0-0002: Communication cycle time
        registerIDN(0x0002, "Communication cycle time", 0x00070001, "us", cycleTime, true);
        
        // S-0-0017: IDN of list operation data
        registerIDN(0x0011, "IDN list of operation data", 0x00400001, "", {}, true);
        
        // S-0-0032: Primary operation mode
        registerIDN(0x0020, "Primary operation mode", 0x00030001, "", {0x00}, false);
        
        // S-0-0033: Secondary operation mode 1
        registerIDN(0x0021, "Secondary operation mode 1", 0x00030001, "", {0x00}, false);
        
        // S-0-0034: Secondary operation mode 2
        registerIDN(0x0022, "Secondary operation mode 2", 0x00030001, "", {0x00}, false);
        
        // S-0-0036: Velocity command
        std::vector<uint8_t> velocity = {0x00, 0x00, 0x00, 0x00};
        registerIDN(0x0024, "Velocity command", 0x00070001, "rpm", velocity, false);
        
        // S-0-0037: Velocity feedback
        registerIDN(0x0025, "Velocity feedback", 0x00070001, "rpm", velocity, true);
        
        // S-0-0040: Position command
        std::vector<uint8_t> position = {0x00, 0x00, 0x00, 0x00};
        registerIDN(0x0028, "Position command", 0x00070001, "inc", position, false);
        
        // S-0-0041: Position feedback
        registerIDN(0x0029, "Position feedback", 0x00070001, "inc", position, true);
        
        // S-0-0047: Position feedback 1
        registerIDN(0x002F, "Position feedback 1", 0x00070001, "inc", position, true);
        
        // S-0-0051: Torque command
        std::vector<uint8_t> torque = {0x00, 0x00};
        registerIDN(0x0033, "Torque command", 0x00030001, "0.1%", torque, false);
        
        // S-0-0084: Torque feedback
        registerIDN(0x0054, "Torque feedback", 0x00030001, "0.1%", torque, true);
        
        // S-0-0091: Amplifier switching on
        registerIDN(0x005B, "Amplifier switching on", 0x00070001, "", {0x00}, false);
        
        // S-0-0099: Control word
        std::vector<uint8_t> controlWord = {0x00, 0x00};
        registerIDN(0x0063, "Control word", 0x00030001, "", controlWord, false);
        
        // S-0-0134: Status word
        registerIDN(0x0086, "Status word", 0x00030001, "", controlWord, true);
        
        // S-0-0135: Error code
        registerIDN(0x0087, "Error code", 0x00030001, "", {0x00, 0x00}, true);
        
        // S-0-0140: Manufacturer version
        registerIDN(0x008C, "Manufacturer version", 0x00800001, "", {}, true);
    }
    
    bool handleReadRequest(uint8_t driveNo, uint16_t idn, uint8_t elements,
                          const uint8_t* data, size_t length,
                          uint8_t* response, size_t& responseLength) {
        // responseLength on input holds the response buffer capacity.
        size_t bufCap = responseLength;
        auto it = idnMap_.find(idn);
        if (it == idnMap_.end()) {
            return sendError(idn, SOE_ERR_NO_IDN, response, responseLength);
        }

        SoEIDN& entry = it->second;

        // Build response
        size_t offset = 0;
        response[offset++] = SOE_OP_READ_RESPONSE;
        response[offset++] = driveNo;
        response[offset++] = elements;
        response[offset++] = idn & 0xFF;
        response[offset++] = (idn >> 8) & 0xFF;
        response[offset++] = 0;  // Reserved

        // Add requested elements
        if (elements & SOE_ELEM_DATA) {
            // Data length header (2 bytes)
            std::vector<uint8_t> value;
            if (entry.readCallback) {
                value = entry.readCallback();
            } else {
                value = entry.data;
            }

            uint16_t dataLen = value.size();
            if (offset + 2 + dataLen > bufCap) {
                return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
            }
            response[offset++] = dataLen & 0xFF;
            response[offset++] = (dataLen >> 8) & 0xFF;

            memcpy(response + offset, value.data(), value.size());
            offset += value.size();
        }

        if (elements & SOE_ELEM_NAME) {
            if (entry.name.empty()) {
                return sendError(idn, SOE_ERR_NO_NAME, response, responseLength);
            }

            uint16_t nameLen = entry.name.length();
            if (offset + 2 + nameLen > bufCap) {
                return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
            }
            response[offset++] = nameLen & 0xFF;
            response[offset++] = (nameLen >> 8) & 0xFF;
            memcpy(response + offset, entry.name.c_str(), nameLen);
            offset += nameLen;
        }

        if (elements & SOE_ELEM_ATTRIBUTE) {
            if (offset + 4 > bufCap) {
                return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
            }
            response[offset++] = entry.attribute & 0xFF;
            response[offset++] = (entry.attribute >> 8) & 0xFF;
            response[offset++] = (entry.attribute >> 16) & 0xFF;
            response[offset++] = (entry.attribute >> 24) & 0xFF;
        }

        if (elements & SOE_ELEM_UNIT) {
            if (entry.unit.empty()) {
                if (offset + 2 > bufCap) {
                    return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
                }
                // No unit - send empty
                response[offset++] = 0;
                response[offset++] = 0;
            } else {
                uint16_t unitLen = entry.unit.length();
                if (offset + 2 + unitLen > bufCap) {
                    return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
                }
                response[offset++] = unitLen & 0xFF;
                response[offset++] = (unitLen >> 8) & 0xFF;
                memcpy(response + offset, entry.unit.c_str(), unitLen);
                offset += unitLen;
            }
        }

        if (elements & SOE_ELEM_MIN) {
            if (entry.minValue.empty()) {
                return sendError(idn, SOE_ERR_NO_MIN, response, responseLength);
            }

            uint16_t minLen = entry.minValue.size();
            if (offset + 2 + minLen > bufCap) {
                return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
            }
            response[offset++] = minLen & 0xFF;
            response[offset++] = (minLen >> 8) & 0xFF;
            memcpy(response + offset, entry.minValue.data(), minLen);
            offset += minLen;
        }

        if (elements & SOE_ELEM_MAX) {
            if (entry.maxValue.empty()) {
                return sendError(idn, SOE_ERR_NO_MAX, response, responseLength);
            }

            uint16_t maxLen = entry.maxValue.size();
            if (offset + 2 + maxLen > bufCap) {
                return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
            }
            response[offset++] = maxLen & 0xFF;
            response[offset++] = (maxLen >> 8) & 0xFF;
            memcpy(response + offset, entry.maxValue.data(), maxLen);
            offset += maxLen;
        }

        responseLength = offset;
        return true;
    }
    
    bool handleWriteRequest(uint8_t driveNo, uint16_t idn, uint8_t elements,
                           const uint8_t* data, size_t length,
                           uint8_t* response, size_t& responseLength) {
        auto it = idnMap_.find(idn);
        if (it == idnMap_.end()) {
            return sendError(idn, SOE_ERR_NO_IDN, response, responseLength);
        }
        
        SoEIDN& entry = it->second;
        
        // Check if writable
        if (entry.readOnly) {
            return sendError(idn, SOE_ERR_ATTRIBUTE_RO, response, responseLength);
        }
        
        // Parse write data
        size_t offset = 0;
        
        if (elements & SOE_ELEM_DATA) {
            if (offset + 2 > length) {
                return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
            }
            
            uint16_t dataLen = data[offset] | (data[offset + 1] << 8);
            offset += 2;
            
            if (offset + dataLen > length) {
                return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
            }
            
            std::vector<uint8_t> newData(data + offset, data + offset + dataLen);
            // offset += dataLen; // Not used
            
            // Check limits
            if (!entry.minValue.empty() && newData < entry.minValue) {
                return sendError(idn, SOE_ERR_DATA_MIN, response, responseLength);
            }
            if (!entry.maxValue.empty() && newData > entry.maxValue) {
                return sendError(idn, SOE_ERR_DATA_MAX, response, responseLength);
            }
            
            // Write data
            if (entry.writeCallback) {
                if (!entry.writeCallback(newData)) {
                    return sendError(idn, SOE_ERR_COMMAND, response, responseLength);
                }
            } else {
                entry.data = newData;
            }
        }
        
        // Build success response
        response[0] = SOE_OP_WRITE_RESPONSE;
        response[1] = driveNo;
        response[2] = elements;
        response[3] = idn & 0xFF;
        response[4] = (idn >> 8) & 0xFF;
        response[5] = 0;
        
        responseLength = 6;
        return true;
    }
    
    bool sendError(uint16_t idn, uint16_t errorCode,
                  uint8_t* response, size_t& responseLength) {
        response[0] = SOE_OP_READ_RESPONSE | 0x10;  // Error flag
        response[1] = 0;
        response[2] = 0;
        response[3] = idn & 0xFF;
        response[4] = (idn >> 8) & 0xFF;
        response[5] = 0;
        response[6] = errorCode & 0xFF;
        response[7] = (errorCode >> 8) & 0xFF;
        
        responseLength = 8;
        return true;
    }
    
private:
    SlaveCore* core_;
    std::map<uint16_t, SoEIDN> idnMap_;
    
    bool fragmentedTransfer_{false};
    std::vector<uint8_t> fragmentBuffer_;
};

// Factory function
std::unique_ptr<IMailboxHandler> createSoEHandler(SlaveCore* core) {
    return std::make_unique<SoEHandler>(core);
}

// Factory overload matching header declaration (no SlaveCore)
std::unique_ptr<IMailboxHandler> createSoEHandler() {
    return std::make_unique<SoEHandler>(nullptr);
}

}  // namespace slave
}  // namespace EtherCAT
