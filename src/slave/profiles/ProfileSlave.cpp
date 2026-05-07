/**
 * @file ProfileSlave.cpp
 * @brief Base Profile Slave Implementation
 */

#include "slave/profiles/ProfileSlave.hpp"
#include "slave/core/SlaveCore.hpp"
#include "slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <map>

namespace EtherCAT {
namespace slave {

// ============================================================================
// Simple Object Dictionary Implementation
// ============================================================================

class ProfileObjectDictionary : public IObjectDictionary {
public:
    SDOAbortCode read(uint16_t index, uint8_t subindex,
                      uint8_t* data, size_t& dataLen) override {
        auto it = objects_.find(makeKey(index, subindex));
        if (it == objects_.end()) {
            return SDOAbortCode::ObjectNotFound;
        }
        
        const auto& entry = it->second;
        size_t copyLen = std::min(dataLen, entry.data.size());
        if (copyLen > 0) {
            std::memcpy(data, entry.data.data(), copyLen);
        }
        dataLen = copyLen;
        
        // Invoke read callback if present
        if (entry.readCallback) {
            return entry.readCallback(data, dataLen);
        }
        
        return SDOAbortCode::Success;
    }
    
    SDOAbortCode write(uint16_t index, uint8_t subindex,
                       const uint8_t* data, size_t dataLen) override {
        auto it = objects_.find(makeKey(index, subindex));
        if (it == objects_.end()) {
            return SDOAbortCode::ObjectNotFound;
        }
        
        auto& entry = it->second;
        
        // Check access type (bit 0 = read, bit 1 = write)
        if ((entry.info.accessType & 0x02) == 0) {
            return SDOAbortCode::ReadOnlyObject;
        }
        
        // Invoke write callback if present
        if (entry.writeCallback) {
            return entry.writeCallback(data, dataLen);
        }
        
        // Copy data
        size_t copyLen = std::min(dataLen, entry.data.size());
        if (copyLen > 0) {
            std::memcpy(entry.data.data(), data, copyLen);
        }
        
        return SDOAbortCode::Success;
    }
    
    bool hasObject(uint16_t index, uint8_t subindex) const override {
        return objects_.find(makeKey(index, subindex)) != objects_.end();
    }
    
    bool getObjectInfo(uint16_t index, uint8_t subindex,
                       ODEntryInfo& info) const override {
        auto it = objects_.find(makeKey(index, subindex));
        if (it == objects_.end()) {
            return false;
        }
        info = it->second.info;
        return true;
    }
    
    uint8_t getSubindexCount(uint16_t index) const override {
        auto it = indexMaxSubindex_.find(index);
        return (it != indexMaxSubindex_.end()) ? it->second : 0;
    }
    
    bool registerObject(const ODEntryInfo& info) override {
        return registerObject(info, nullptr, nullptr);
    }
    
    bool registerObject(const ODEntryInfo& info,
                        ReadCallback readCb,
                        WriteCallback writeCb) override {
        uint32_t key = makeKey(info.index, info.subindex);
        
        ObjectEntry entry;
        entry.info = info;
        entry.data.resize((info.bitLength + 7) / 8, 0);
        entry.readCallback = readCb;
        entry.writeCallback = writeCb;
        
        // Initialize data from defaultValue (little-endian)
        if (entry.data.size() >= 4) {
            entry.data[0] = info.defaultValue & 0xFF;
            entry.data[1] = (info.defaultValue >> 8) & 0xFF;
            entry.data[2] = (info.defaultValue >> 16) & 0xFF;
            entry.data[3] = (info.defaultValue >> 24) & 0xFF;
        } else if (entry.data.size() >= 2) {
            entry.data[0] = info.defaultValue & 0xFF;
            entry.data[1] = (info.defaultValue >> 8) & 0xFF;
        } else if (entry.data.size() >= 1) {
            entry.data[0] = info.defaultValue & 0xFF;
        }
        
        objects_[key] = std::move(entry);
        
        // Track max subindex
        auto& maxSub = indexMaxSubindex_[info.index];
        if (info.subindex > maxSub) {
            maxSub = info.subindex;
        }
        
        return true;
    }
    
    bool getPDOMapping(uint16_t /*pdoIndex*/,
                       std::vector<ODEntryInfo>& /*entries*/) const override {
        // Not implemented for simple profile
        return false;
    }
    
    bool save(std::vector<uint8_t>& /*data*/) const override {
        // Not implemented for simple profile
        return false;
    }
    
    bool load(const std::vector<uint8_t>& /*data*/) override {
        // Not implemented for simple profile
        return false;
    }
    
private:
    static uint32_t makeKey(uint16_t index, uint8_t subindex) {
        return (static_cast<uint32_t>(index) << 8) | subindex;
    }
    
    struct ObjectEntry {
        ODEntryInfo info;
        std::vector<uint8_t> data;
        ReadCallback readCallback;
        WriteCallback writeCallback;
    };
    
    std::map<uint32_t, ObjectEntry> objects_;
    std::map<uint16_t, uint8_t> indexMaxSubindex_;
};

// ============================================================================
// ProfileSlave Implementation
// ============================================================================

ProfileSlave::ProfileSlave(CiAProfile profile, const SlaveConfig& config)
    : profile_(profile)
    , core_(std::make_unique<SlaveCore>(config))
    , objectDictionary_(std::make_shared<ProfileObjectDictionary>())
{
}

ProfileSlave::~ProfileSlave() {
    stop();
}

void ProfileSlave::setHAL(std::shared_ptr<ISlaveHAL> hal) {
    // Store HAL reference - core_ may have a setHAL method
    (void)hal;
}

bool ProfileSlave::start() {
    // Initialize object dictionary
    initObjectDictionary();
    
    // Initialize PDO mappings
    initPDOMappings();
    
    return true;
}

void ProfileSlave::stop() {
    // Cleanup
}

bool ProfileSlave::isRunning() const {
    return core_ != nullptr;
}

void ProfileSlave::simulate(uint64_t deltaNs) {
    (void)deltaNs;
    // Default: no simulation
}

void ProfileSlave::onStateChange(SlaveState oldState, SlaveState newState) {
    (void)oldState;
    (void)newState;
    // Default: no action
}

void ProfileSlave::onSync(int syncNum, uint64_t timestamp) {
    (void)syncNum;
    (void)timestamp;
    // Default: no action
}

void ProfileSlave::registerCiA301Objects() {
    // Register standard communication objects
    ODEntryInfo info;
    
    // 0x1000 Device Type
    info.index = 0x1000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned32;
    info.bitLength = 32;
    info.accessType = 0x01;  // Read-only
    info.name = "Device Type";
    info.defaultValue = getDeviceType();
    objectDictionary_->registerObject(info);
    
    // 0x1018 Identity Object
    info.index = 0x1018;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned8;
    info.bitLength = 8;
    info.accessType = 0x01;
    info.name = "Identity Object";
    info.defaultValue = 4;  // 4 subindices
    objectDictionary_->registerObject(info);
}

void ProfileSlave::registerPDOMapping(uint16_t index, const std::vector<uint32_t>& entries) {
    // Register PDO mapping object
    ODEntryInfo info;
    info.index = index;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned8;
    info.bitLength = 8;
    info.accessType = 0x03;  // Read-write
    info.name = "PDO Mapping";
    info.defaultValue = static_cast<uint32_t>(entries.size());
    objectDictionary_->registerObject(info);
    
    // Register each entry
    for (size_t i = 0; i < entries.size(); i++) {
        info.subindex = static_cast<uint8_t>(i + 1);
        info.dataType = ObjectDictionaryDataType::Unsigned32;
        info.bitLength = 32;
        info.defaultValue = entries[i];
        objectDictionary_->registerObject(info);
    }
}

}  // namespace slave
}  // namespace EtherCAT
