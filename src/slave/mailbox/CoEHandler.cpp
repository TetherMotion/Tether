/**
 * @file CoEHandler.cpp
 * @brief CANopen over EtherCAT (CoE) Handler Implementation
 */

#include "tether/slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <map>
#include <vector>

namespace EtherCAT {
namespace slave {

// ============================================================================
// CoE Protocol Constants
// ============================================================================

namespace {
    // Mailbox types
    constexpr uint8_t MBOX_TYPE_COE = 0x03;
    
    // CoE header service types
    constexpr uint8_t COE_EMERGENCY = 0x01;
    constexpr uint8_t COE_SDO_REQ   = 0x02;
    constexpr uint8_t COE_SDO_RES   = 0x03;
    constexpr uint8_t COE_TX_PDO    = 0x04;
    constexpr uint8_t COE_RX_PDO    = 0x05;
    constexpr uint8_t COE_TX_PDO_REMOTE = 0x06;
    constexpr uint8_t COE_RX_PDO_REMOTE = 0x07;
    constexpr uint8_t COE_SDO_INFO  = 0x08;
    
    // SDO command specifiers
    constexpr uint8_t SDO_CCS_DOWNLOAD_INIT = 1;
    constexpr uint8_t SDO_CCS_DOWNLOAD_SEG  = 0;
    constexpr uint8_t SDO_CCS_UPLOAD_INIT   = 2;
    constexpr uint8_t SDO_CCS_UPLOAD_SEG    = 3;
    constexpr uint8_t SDO_CCS_ABORT         = 4;
    
    constexpr uint8_t SDO_SCS_DOWNLOAD_INIT = 3;
    constexpr uint8_t SDO_SCS_DOWNLOAD_SEG  = 1;
    constexpr uint8_t SDO_SCS_UPLOAD_INIT   = 2;
    constexpr uint8_t SDO_SCS_UPLOAD_SEG    = 0;
    
    // SDO Info opcodes
    constexpr uint8_t SDO_INFO_GET_OD_LIST_REQ  = 0x01;
    constexpr uint8_t SDO_INFO_GET_OD_LIST_RES  = 0x02;
    constexpr uint8_t SDO_INFO_GET_OBJ_DESC_REQ = 0x03;
    constexpr uint8_t SDO_INFO_GET_OBJ_DESC_RES = 0x04;
    constexpr uint8_t SDO_INFO_GET_ENTRY_DESC_REQ = 0x05;
    constexpr uint8_t SDO_INFO_GET_ENTRY_DESC_RES = 0x06;
    constexpr uint8_t SDO_INFO_ERROR = 0x07;
}

// ============================================================================
// ObjectDictionary Implementation
// ============================================================================

/**
 * @brief Basic object dictionary implementation
 * Stores objects with their data and provides read/write access
 */
class ObjectDictionaryImpl : public IObjectDictionary {
public:
    ObjectDictionaryImpl() = default;
    
    // Object Access
    SDOAbortCode read(uint16_t index, uint8_t subindex,
                      uint8_t* data, size_t& dataLen) override {
        auto key = makeKey(index, subindex);
        auto it = objects_.find(key);
        if (it == objects_.end()) {
            return SDOAbortCode::ObjectNotFound;
        }
        
        auto& entry = it->second;
        
        // Check access
        if (entry.accessType == static_cast<uint8_t>(ODAccessType::WriteOnly)) {
            return SDOAbortCode::ReadOnlyObject;
        }
        
        // If there's a read callback, use it
        if (entry.readCallback) {
            return entry.readCallback(data, dataLen);
        }
        
        // Otherwise, copy from stored data
        size_t copyLen = std::min(dataLen, entry.data.size());
        std::memcpy(data, entry.data.data(), copyLen);
        dataLen = copyLen;
        
        return SDOAbortCode::Success;
    }
    
    SDOAbortCode write(uint16_t index, uint8_t subindex,
                       const uint8_t* data, size_t dataLen) override {
        auto key = makeKey(index, subindex);
        auto it = objects_.find(key);
        if (it == objects_.end()) {
            return SDOAbortCode::ObjectNotFound;
        }
        
        auto& entry = it->second;
        
        // Check access
        if (entry.accessType == static_cast<uint8_t>(ODAccessType::ReadOnly)) {
            return SDOAbortCode::WriteOnlyObject;
        }
        
        // If there's a write callback, use it
        if (entry.writeCallback) {
            return entry.writeCallback(data, dataLen);
        }
        
        // Otherwise, copy to stored data
        size_t copyLen = std::min(dataLen, entry.data.size());
        std::memcpy(entry.data.data(), data, copyLen);
        
        return SDOAbortCode::Success;
    }
    
    // Object Information
    bool hasObject(uint16_t index, uint8_t subindex) const override {
        return objects_.find(makeKey(index, subindex)) != objects_.end();
    }
    
    bool getObjectInfo(uint16_t index, uint8_t subindex,
                       ODEntryInfo& info) const override {
        auto key = makeKey(index, subindex);
        auto it = objects_.find(key);
        if (it == objects_.end()) {
            return false;
        }
        
        const auto& entry = it->second;
        info.index = index;
        info.subindex = subindex;
        info.dataType = entry.dataType;
        info.bitLength = entry.bitLength;
        info.accessType = entry.accessType;
        info.name = entry.name;
        info.defaultValue = entry.defaultValue;
        
        return true;
    }
    
    uint8_t getSubindexCount(uint16_t index) const override {
        uint8_t maxSubindex = 0;
        for (const auto& [key, entry] : objects_) {
            uint16_t objIndex = key >> 8;
            uint8_t objSubindex = key & 0xFF;
            if (objIndex == index && objSubindex > maxSubindex) {
                maxSubindex = objSubindex;
            }
        }
        return maxSubindex;
    }
    
    // Object Registration
    bool registerObject(const ODEntryInfo& info) override {
        auto key = makeKey(info.index, info.subindex);
        
        ObjectData objData;
        objData.dataType = info.dataType;
        objData.bitLength = info.bitLength;
        objData.accessType = info.accessType;
        objData.name = info.name;
        objData.defaultValue = info.defaultValue;
        objData.data.resize((info.bitLength + 7) / 8, 0);
        
        // Initialize with default value
        if (objData.data.size() >= 4) {
            objData.data[0] = info.defaultValue & 0xFF;
            objData.data[1] = (info.defaultValue >> 8) & 0xFF;
            objData.data[2] = (info.defaultValue >> 16) & 0xFF;
            objData.data[3] = (info.defaultValue >> 24) & 0xFF;
        }
        
        objects_[key] = std::move(objData);
        
        // Track indices
        if (std::find(indices_.begin(), indices_.end(), info.index) == indices_.end()) {
            indices_.push_back(info.index);
        }
        
        return true;
    }
    
    bool registerObject(const ODEntryInfo& info,
                        ReadCallback readCb,
                        WriteCallback writeCb) override {
        if (!registerObject(info)) {
            return false;
        }
        
        auto key = makeKey(info.index, info.subindex);
        auto it = objects_.find(key);
        if (it != objects_.end()) {
            it->second.readCallback = std::move(readCb);
            it->second.writeCallback = std::move(writeCb);
        }
        
        return true;
    }
    
    // PDO Mapping
    bool getPDOMapping(uint16_t pdoIndex,
                       std::vector<ODEntryInfo>& entries) const override {
        // PDO mapping is stored in objects 0x1600-0x17FF (RxPDO) and 0x1A00-0x1BFF (TxPDO)
        entries.clear();
        
        // Get number of entries (subindex 0)
        if (!hasObject(pdoIndex, 0)) {
            return false;
        }
        
        uint8_t numEntries = getSubindexCount(pdoIndex);
        
        for (uint8_t i = 1; i <= numEntries; ++i) {
            ODEntryInfo info;
            if (getObjectInfo(pdoIndex, i, info)) {
                entries.push_back(info);
            }
        }
        
        return true;
    }
    
    // Serialization
    bool save(std::vector<uint8_t>& data) const override {
        // Simple serialization: not implemented for now
        data.clear();
        return true;
    }
    
    bool load(const std::vector<uint8_t>& data) override {
        // Simple deserialization: not implemented for now
        return true;
    }
    
    // Additional helpers for internal use
    const std::vector<uint16_t>& getIndices() const { return indices_; }

private:
    struct ObjectData {
        ObjectDictionaryDataType dataType;
        uint16_t bitLength;
        uint8_t accessType;
        std::string name;
        uint32_t defaultValue;
        std::vector<uint8_t> data;
        ReadCallback readCallback;
        WriteCallback writeCallback;
    };
    
    static uint32_t makeKey(uint16_t index, uint8_t subindex) {
        return (static_cast<uint32_t>(index) << 8) | subindex;
    }
    
    std::map<uint32_t, ObjectData> objects_;
    std::vector<uint16_t> indices_;
};

// ============================================================================
// CoEHandler Implementation
// ============================================================================

class CoEHandler : public IMailboxHandler {
public:
    explicit CoEHandler(std::shared_ptr<IObjectDictionary> objectDictionary)
        : objectDictionary_(objectDictionary)
    {
    }
    
    MailboxProtocol getProtocol() const override {
        return MailboxProtocol::CoE;
    }
    
    const char* getProtocolName() const override {
        return "CoE";
    }
    
    void reset() override {
        segmentedTransferActive_ = false;
        segmentedData_.clear();
    }
    
    bool processRequest(const uint8_t* request, size_t requestLength,
                        uint8_t* response, size_t& responseLength) override {
        if (requestLength < 8) {  // Minimum mailbox + CoE header
            return false;
        }
        
        // Parse mailbox header (6 bytes)
        uint16_t mbxLength = request[0] | (request[1] << 8);
        uint16_t mbxAddress = request[2] | (request[3] << 8);
        uint8_t mbxChannel = request[4] & 0x3F;
        uint8_t mbxPriority = (request[4] >> 6) & 0x03;
        uint8_t mbxType = request[5] & 0x0F;
        uint8_t mbxCount = (request[5] >> 4) & 0x0F;
        
        if (mbxType != MBOX_TYPE_COE) {
            return false;  // Not CoE
        }
        
        // Parse CoE header (2 bytes)
        const uint8_t* coeData = request + 6;
        uint16_t coeHeader = coeData[0] | (coeData[1] << 8);
        uint8_t coeService = (coeHeader >> 12) & 0x0F;
        
        // Build response mailbox header
        response[2] = mbxAddress & 0xFF;
        response[3] = (mbxAddress >> 8) & 0xFF;
        response[4] = mbxChannel | (mbxPriority << 6);
        response[5] = MBOX_TYPE_COE | (mbxCount << 4);
        
        bool result = false;
        
        switch (coeService) {
            case COE_SDO_REQ:
                result = processSDORequest(coeData + 2, mbxLength - 2,
                                           response + 8, responseLength);
                if (result) {
                    // Add CoE header to response
                    response[6] = ((COE_SDO_RES & 0x0F) << 4);
                    response[7] = 0;
                    
                    // Update mailbox length
                    uint16_t respLen = responseLength + 2;  // +2 for CoE header
                    response[0] = respLen & 0xFF;
                    response[1] = (respLen >> 8) & 0xFF;
                    
                    responseLength += 8;  // Total response includes mailbox + CoE header
                }
                break;
                
            case COE_SDO_INFO:
                result = processSDOInfo(coeData + 2, mbxLength - 2,
                                        response + 8, responseLength);
                if (result) {
                    response[6] = ((COE_SDO_INFO & 0x0F) << 4);
                    response[7] = 0;
                    
                    uint16_t respLen = responseLength + 2;
                    response[0] = respLen & 0xFF;
                    response[1] = (respLen >> 8) & 0xFF;
                    
                    responseLength += 8;
                }
                break;
                
            default:
                break;
        }
        
        return result;
    }

private:
    std::shared_ptr<IObjectDictionary> objectDictionary_;
    
    // Segmented transfer state
    bool segmentedTransferActive_ = false;
    bool segmentedUpload_ = false;
    uint16_t segmentedIndex_ = 0;
    uint8_t segmentedSubindex_ = 0;
    size_t segmentedOffset_ = 0;
    uint8_t segmentedToggle_ = 0;
    std::vector<uint8_t> segmentedData_;
    
    bool processSDORequest(const uint8_t* data, size_t length,
                            uint8_t* response, size_t& responseLength) {
        if (length < 8) {
            return false;
        }
        
        uint8_t ccs = (data[0] >> 5) & 0x07;
        
        switch (ccs) {
            case SDO_CCS_UPLOAD_INIT:
                return processUploadInit(data, length, response, responseLength);
                
            case SDO_CCS_UPLOAD_SEG:
                return processUploadSegment(data, length, response, responseLength);
                
            case SDO_CCS_DOWNLOAD_INIT:
                return processDownloadInit(data, length, response, responseLength);
                
            case SDO_CCS_DOWNLOAD_SEG:
                return processDownloadSegment(data, length, response, responseLength);
                
            case SDO_CCS_ABORT:
                segmentedTransferActive_ = false;
                return true;
                
            default:
                return false;
        }
    }
    
    bool processUploadInit(const uint8_t* data, size_t length,
                           uint8_t* response, size_t& responseLength) {
        uint16_t index = data[1] | (data[2] << 8);
        uint8_t subindex = data[3];
        
        if (!objectDictionary_) {
            return sendAbort(index, subindex, static_cast<uint32_t>(SDOAbortCode::InternalError),
                            response, responseLength);
        }
        
        // Check if object exists
        if (!objectDictionary_->hasObject(index, subindex)) {
            return sendAbort(index, subindex, static_cast<uint32_t>(SDOAbortCode::ObjectNotFound),
                            response, responseLength);
        }
        
        // Read the data
        std::vector<uint8_t> objData(256);
        size_t dataLen = objData.size();
        
        SDOAbortCode result = objectDictionary_->read(index, subindex, objData.data(), dataLen);
        if (result != SDOAbortCode::Success) {
            return sendAbort(index, subindex, static_cast<uint32_t>(result),
                            response, responseLength);
        }
        
        objData.resize(dataLen);
        
        if (dataLen <= 4) {
            // Expedited transfer
            uint8_t n = 4 - dataLen;
            response[0] = (SDO_SCS_UPLOAD_INIT << 5) | (n << 2) | 0x02 | 0x01;  // e=1, s=1
            response[1] = index & 0xFF;
            response[2] = (index >> 8) & 0xFF;
            response[3] = subindex;
            
            std::memcpy(response + 4, objData.data(), dataLen);
            std::memset(response + 4 + dataLen, 0, 4 - dataLen);
            
            responseLength = 8;
        } else {
            // Segmented transfer
            segmentedTransferActive_ = true;
            segmentedUpload_ = true;
            segmentedIndex_ = index;
            segmentedSubindex_ = subindex;
            segmentedOffset_ = 0;
            segmentedToggle_ = 0;
            segmentedData_ = std::move(objData);
            
            response[0] = (SDO_SCS_UPLOAD_INIT << 5) | 0x01;  // s=1, e=0
            response[1] = index & 0xFF;
            response[2] = (index >> 8) & 0xFF;
            response[3] = subindex;
            
            // Size (4 bytes)
            uint32_t totalSize = segmentedData_.size();
            response[4] = totalSize & 0xFF;
            response[5] = (totalSize >> 8) & 0xFF;
            response[6] = (totalSize >> 16) & 0xFF;
            response[7] = (totalSize >> 24) & 0xFF;
            
            responseLength = 8;
        }
        
        return true;
    }
    
    bool processUploadSegment(const uint8_t* data, size_t length,
                               uint8_t* response, size_t& responseLength) {
        if (!segmentedTransferActive_ || !segmentedUpload_) {
            return sendAbort(segmentedIndex_, segmentedSubindex_,
                            static_cast<uint32_t>(SDOAbortCode::InvalidCommand),
                            response, responseLength);
        }
        
        uint8_t toggle = (data[0] >> 4) & 0x01;
        if (toggle != segmentedToggle_) {
            return sendAbort(segmentedIndex_, segmentedSubindex_,
                            static_cast<uint32_t>(SDOAbortCode::ToggleBitNotChanged),
                            response, responseLength);
        }
        
        // Calculate remaining data
        size_t remaining = segmentedData_.size() - segmentedOffset_;
        size_t segmentSize = std::min(remaining, size_t(7));
        bool lastSegment = (remaining <= 7);
        uint8_t n = 7 - segmentSize;
        
        response[0] = (SDO_SCS_UPLOAD_SEG << 5) | (toggle << 4) |
                     (n << 1) | (lastSegment ? 0x01 : 0x00);
        std::memcpy(response + 1, segmentedData_.data() + segmentedOffset_, segmentSize);
        std::memset(response + 1 + segmentSize, 0, 7 - segmentSize);
        
        responseLength = 8;
        
        segmentedOffset_ += segmentSize;
        segmentedToggle_ ^= 1;
        
        if (lastSegment) {
            segmentedTransferActive_ = false;
            segmentedData_.clear();
        }
        
        return true;
    }
    
    bool processDownloadInit(const uint8_t* data, size_t length,
                              uint8_t* response, size_t& responseLength) {
        uint16_t index = data[1] | (data[2] << 8);
        uint8_t subindex = data[3];
        
        if (!objectDictionary_) {
            return sendAbort(index, subindex, static_cast<uint32_t>(SDOAbortCode::InternalError),
                            response, responseLength);
        }
        
        // Check if object exists
        if (!objectDictionary_->hasObject(index, subindex)) {
            return sendAbort(index, subindex, static_cast<uint32_t>(SDOAbortCode::ObjectNotFound),
                            response, responseLength);
        }
        
        uint8_t cs = data[0];
        bool expedited = (cs & 0x02) != 0;
        bool sizeIndicated = (cs & 0x01) != 0;
        uint8_t n = (cs >> 2) & 0x03;
        
        if (expedited) {
            // Expedited transfer
            size_t dataSize = sizeIndicated ? (4 - n) : 4;
            
            SDOAbortCode result = objectDictionary_->write(index, subindex,
                                                          data + 4, dataSize);
            if (result != SDOAbortCode::Success) {
                return sendAbort(index, subindex, static_cast<uint32_t>(result),
                                response, responseLength);
            }
            
            // Response
            response[0] = (SDO_SCS_DOWNLOAD_INIT << 5);
            response[1] = index & 0xFF;
            response[2] = (index >> 8) & 0xFF;
            response[3] = subindex;
            std::memset(response + 4, 0, 4);
            
            responseLength = 8;
        } else {
            // Segmented transfer
            uint32_t totalSize = 0;
            if (sizeIndicated) {
                totalSize = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24);
            }
            
            segmentedTransferActive_ = true;
            segmentedUpload_ = false;
            segmentedIndex_ = index;
            segmentedSubindex_ = subindex;
            segmentedOffset_ = 0;
            segmentedToggle_ = 0;
            segmentedData_.clear();
            if (totalSize > 0 && totalSize < 65536) {
                segmentedData_.reserve(totalSize);
            }
            
            // Response
            response[0] = (SDO_SCS_DOWNLOAD_INIT << 5);
            response[1] = index & 0xFF;
            response[2] = (index >> 8) & 0xFF;
            response[3] = subindex;
            std::memset(response + 4, 0, 4);
            
            responseLength = 8;
        }
        
        return true;
    }
    
    bool processDownloadSegment(const uint8_t* data, size_t length,
                                 uint8_t* response, size_t& responseLength) {
        if (!segmentedTransferActive_ || segmentedUpload_) {
            return sendAbort(segmentedIndex_, segmentedSubindex_,
                            static_cast<uint32_t>(SDOAbortCode::InvalidCommand),
                            response, responseLength);
        }
        
        uint8_t toggle = (data[0] >> 4) & 0x01;
        if (toggle != segmentedToggle_) {
            return sendAbort(segmentedIndex_, segmentedSubindex_,
                            static_cast<uint32_t>(SDOAbortCode::ToggleBitNotChanged),
                            response, responseLength);
        }
        
        uint8_t n = (data[0] >> 1) & 0x07;
        bool lastSegment = (data[0] & 0x01) != 0;
        size_t segmentSize = 7 - n;
        
        segmentedData_.insert(segmentedData_.end(), data + 1, data + 1 + segmentSize);
        
        // Response
        response[0] = (SDO_SCS_DOWNLOAD_SEG << 5) | (toggle << 4);
        std::memset(response + 1, 0, 7);
        responseLength = 8;
        
        segmentedToggle_ ^= 1;
        
        if (lastSegment) {
            // Write complete data to object
            SDOAbortCode result = objectDictionary_->write(segmentedIndex_, segmentedSubindex_,
                                                          segmentedData_.data(), segmentedData_.size());
            
            segmentedTransferActive_ = false;
            segmentedData_.clear();
            
            if (result != SDOAbortCode::Success) {
                return sendAbort(segmentedIndex_, segmentedSubindex_,
                                static_cast<uint32_t>(result),
                                response, responseLength);
            }
        }
        
        return true;
    }
    
    bool processSDOInfo(const uint8_t* data, size_t length,
                         uint8_t* response, size_t& responseLength) {
        if (length < 2) {
            return false;
        }
        
        uint8_t opcode = data[0] & 0x7F;
        
        switch (opcode) {
            case SDO_INFO_GET_OD_LIST_REQ:
                return processGetODList(data, length, response, responseLength);
                
            case SDO_INFO_GET_OBJ_DESC_REQ:
                return processGetObjDesc(data, length, response, responseLength);
                
            case SDO_INFO_GET_ENTRY_DESC_REQ:
                return processGetEntryDesc(data, length, response, responseLength);
                
            default:
                return false;
        }
    }
    
    bool processGetODList(const uint8_t* data, size_t length,
                           uint8_t* response, size_t& responseLength) {
        // This requires knowing all indices - we can iterate if needed
        // For now, return a minimal response
        uint16_t listType = data[2] | (data[3] << 8);
        
        response[0] = SDO_INFO_GET_OD_LIST_RES;
        response[1] = 0;
        response[2] = listType & 0xFF;
        response[3] = (listType >> 8) & 0xFF;
        
        responseLength = 4;
        return true;
    }
    
    bool processGetObjDesc(const uint8_t* data, size_t length,
                            uint8_t* response, size_t& responseLength) {
        uint16_t index = data[2] | (data[3] << 8);
        
        if (!objectDictionary_->hasObject(index, 0)) {
            return false;
        }
        
        ODEntryInfo info;
        if (!objectDictionary_->getObjectInfo(index, 0, info)) {
            return false;
        }
        
        uint8_t maxSub = objectDictionary_->getSubindexCount(index);
        
        response[0] = SDO_INFO_GET_OBJ_DESC_RES;
        response[1] = 0;
        response[2] = index & 0xFF;
        response[3] = (index >> 8) & 0xFF;
        response[4] = static_cast<uint8_t>(info.dataType);
        response[5] = maxSub;
        response[6] = 0x07;  // Object code (VAR)
        
        // Copy name
        size_t nameLen = std::min(info.name.size(), size_t(240));
        std::memcpy(response + 7, info.name.c_str(), nameLen);
        
        responseLength = 7 + nameLen;
        return true;
    }
    
    bool processGetEntryDesc(const uint8_t* data, size_t length,
                              uint8_t* response, size_t& responseLength) {
        uint16_t index = data[2] | (data[3] << 8);
        uint8_t subindex = data[4];
        uint8_t valueInfo = data[5];
        
        if (!objectDictionary_->hasObject(index, subindex)) {
            return false;
        }
        
        ODEntryInfo info;
        if (!objectDictionary_->getObjectInfo(index, subindex, info)) {
            return false;
        }
        
        response[0] = SDO_INFO_GET_ENTRY_DESC_RES;
        response[1] = 0;
        response[2] = index & 0xFF;
        response[3] = (index >> 8) & 0xFF;
        response[4] = subindex;
        response[5] = valueInfo;
        response[6] = static_cast<uint8_t>(info.dataType);
        response[7] = info.bitLength & 0xFF;
        response[8] = (info.bitLength >> 8) & 0xFF;
        response[9] = info.accessType;
        
        // Name
        size_t nameLen = std::min(info.name.size(), size_t(236));
        std::memcpy(response + 10, info.name.c_str(), nameLen);
        
        responseLength = 10 + nameLen;
        return true;
    }
    
    bool sendAbort(uint16_t index, uint8_t subindex, uint32_t abortCode,
                   uint8_t* response, size_t& responseLength) {
        response[0] = (SDO_CCS_ABORT << 5);
        response[1] = index & 0xFF;
        response[2] = (index >> 8) & 0xFF;
        response[3] = subindex;
        response[4] = abortCode & 0xFF;
        response[5] = (abortCode >> 8) & 0xFF;
        response[6] = (abortCode >> 16) & 0xFF;
        response[7] = (abortCode >> 24) & 0xFF;
        
        responseLength = 8;
        return true;
    }
};

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<IObjectDictionary> createObjectDictionary() {
    return std::make_unique<ObjectDictionaryImpl>();
}

std::unique_ptr<IMailboxHandler> createCoEHandler(std::shared_ptr<IObjectDictionary> od) {
    return std::make_unique<CoEHandler>(od);
}

}  // namespace slave
}  // namespace EtherCAT
