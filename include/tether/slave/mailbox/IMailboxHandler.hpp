/**
 * @file IMailboxHandler.hpp
 * @brief Interface for mailbox protocol handlers
 *
 * @details
 * EtherCAT mailbox supports multiple protocols:
 * - CoE (CANopen over EtherCAT) - Object dictionary access
 * - FoE (File over EtherCAT) - File transfers
 * - EoE (Ethernet over EtherCAT) - Tunneled Ethernet
 * - SoE (Servo over EtherCAT) - SERCOS parameters
 * - AoE (ADS over EtherCAT) - Beckhoff ADS
 * - VoE (Vendor over EtherCAT) - Vendor-specific
 *
 * Each protocol is implemented as a separate handler that processes
 * mailbox requests and generates responses.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include "tether/ethercat/ObjectDictionary.hpp"

namespace EtherCAT {
namespace slave {

// Forward declarations
class SlaveCore;
class IObjectDictionary;

// ============================================================================
// Mailbox Protocol Types
// ============================================================================

/**
 * @brief Mailbox protocol identifiers
 */
enum class MailboxProtocol : uint8_t {
    Error = 0x00,  ///< Error response
    AoE   = 0x01,  ///< ADS over EtherCAT
    EoE   = 0x02,  ///< Ethernet over EtherCAT
    CoE   = 0x03,  ///< CANopen over EtherCAT
    FoE   = 0x04,  ///< File over EtherCAT
    SoE   = 0x05,  ///< Servo over EtherCAT
    VoE   = 0x0F,  ///< Vendor over EtherCAT
};

/**
 * @brief Mailbox header structure (6 bytes)
 */
struct MailboxHeader {
    uint16_t length;      ///< Data length (excluding header)
    uint16_t address;     ///< Station address
    uint8_t  channel : 6; ///< Channel
    uint8_t  priority : 2;///< Priority
    uint8_t  type : 4;    ///< Protocol type
    uint8_t  counter : 4; ///< Counter for duplicate detection
    
    /**
     * @brief Parse header from bytes
     */
    static MailboxHeader fromBytes(const uint8_t* data) {
        MailboxHeader h;
        h.length = data[0] | (data[1] << 8);
        h.address = data[2] | (data[3] << 8);
        h.channel = data[4] & 0x3F;
        h.priority = (data[4] >> 6) & 0x03;
        h.type = data[5] & 0x0F;
        h.counter = (data[5] >> 4) & 0x0F;
        return h;
    }
    
    /**
     * @brief Serialize header to bytes
     */
    void toBytes(uint8_t* data) const {
        data[0] = length & 0xFF;
        data[1] = (length >> 8) & 0xFF;
        data[2] = address & 0xFF;
        data[3] = (address >> 8) & 0xFF;
        data[4] = (channel & 0x3F) | ((priority & 0x03) << 6);
        data[5] = (type & 0x0F) | ((counter & 0x0F) << 4);
    }
};

constexpr size_t kMailboxHeaderSize = 6;

// ============================================================================
// Mailbox Error Codes
// ============================================================================

/**
 * @brief Mailbox error codes
 */
enum class MailboxError : uint16_t {
    NoError              = 0x0000,
    SyntaxError          = 0x0001,
    UnsupportedProtocol  = 0x0002,
    InvalidChannel       = 0x0003,
    ServiceNotSupported  = 0x0004,
    InvalidHeader        = 0x0005,
    SizeTooShort         = 0x0006,
    NoMoreMemory         = 0x0007,
    InvalidSize          = 0x0008,
    ServiceInWork        = 0x0009,
};

// ============================================================================
// IMailboxHandler Interface
// ============================================================================

/**
 * @brief Interface for mailbox protocol handlers
 *
 * Each mailbox protocol (CoE, FoE, etc.) implements this interface
 * to handle protocol-specific requests.
 */
class IMailboxHandler {
public:
    virtual ~IMailboxHandler() = default;
    
    /**
     * @brief Get the protocol type this handler supports
     */
    virtual MailboxProtocol getProtocol() const = 0;
    
    /**
     * @brief Get protocol name for logging
     */
    virtual const char* getProtocolName() const = 0;
    
    /**
     * @brief Process a mailbox request
     *
     * @param request Request data (after mailbox header)
     * @param requestLen Request data length
     * @param response Buffer for response data
     * @param responseLen Response data length (in/out)
     * @return true if response generated, false if no response needed
     */
    virtual bool processRequest(const uint8_t* request, size_t requestLen,
                                uint8_t* response, size_t& responseLen) = 0;
    
    /**
     * @brief Check if handler has pending response
     *
     * Some protocols (like FoE) may need multiple exchanges.
     */
    virtual bool hasPendingResponse() const { return false; }
    
    /**
     * @brief Get pending response
     *
     * @param response Buffer for response
     * @param responseLen Response length (in/out)
     * @return true if there was a pending response
     */
    virtual bool getPendingResponse(uint8_t* response, size_t& responseLen) {
        (void)response; (void)responseLen;
        return false;
    }
    
    /**
     * @brief Reset handler state
     */
    virtual void reset() = 0;
    
    /**
     * @brief Set slave core reference
     */
    virtual void setSlaveCore(SlaveCore* slave) { slave_ = slave; }
    
protected:
    SlaveCore* slave_ = nullptr;
};

// ============================================================================
// IObjectDictionary Interface
// ============================================================================

/**
 * @brief Object dictionary access type
 */
enum class ODAccessType : uint8_t {
    ReadOnly   = 0x01,
    WriteOnly  = 0x02,
    ReadWrite  = 0x03,
    RxPDO      = 0x04,
    TxPDO      = 0x08,
};

// Backward-compatible aliases — keep the old `ODDataType` name while
// exposing the new canonical `ObjectDictionaryDataType`.
using ObjectDictionaryDataType = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;
using ODDataType = ::EtherCAT::ObjectDictionary::ObjectDictionaryDataType;

/**
 * @brief Object dictionary entry description
 */
struct ODEntryInfo {
    uint16_t index;
    uint8_t subindex;
    ODDataType dataType;
    uint16_t bitLength;
    uint8_t accessType;
    std::string name;
    uint32_t defaultValue;
};

/**
 * @brief SDO abort codes (CoE)
 */
enum class SDOAbortCode : uint32_t {
    Success                  = 0x00000000,
    ToggleBitNotChanged      = 0x05030000,
    Timeout                  = 0x05040000,
    InvalidCommand           = 0x05040001,
    OutOfMemory              = 0x05040005,
    UnsupportedAccess        = 0x06010000,
    ReadOnlyObject           = 0x06010001,
    WriteOnlyObject          = 0x06010002,
    ObjectNotFound           = 0x06020000,
    ParameterIncompatible    = 0x06040043,
    InternalError            = 0x06040047,
    DataTypeMismatch         = 0x06070010,
    SubindexNotFound         = 0x06090011,
    InvalidValue             = 0x06090030,
    ValueTooHigh             = 0x06090031,
    ValueTooLow              = 0x06090032,
    GeneralError             = 0x08000000,
    TransferAborted          = 0x08000020,
    DeviceStateError         = 0x08000022,
};

/**
 * @brief Interface for object dictionary
 *
 * The object dictionary provides structured access to slave parameters
 * and data, following the CANopen object dictionary model.
 */
class IObjectDictionary {
public:
    virtual ~IObjectDictionary() = default;
    
    // ========================================================================
    // Object Access
    // ========================================================================
    
    /**
     * @brief Read object value
     *
     * @param index Object index
     * @param subindex Object subindex
     * @param data Buffer for data
     * @param dataLen Data buffer length (in/out: actual length)
     * @return SDO abort code
     */
    virtual SDOAbortCode read(uint16_t index, uint8_t subindex,
                              uint8_t* data, size_t& dataLen) = 0;
    
    /**
     * @brief Write object value
     *
     * @param index Object index
     * @param subindex Object subindex
     * @param data Data to write
     * @param dataLen Data length
     * @return SDO abort code
     */
    virtual SDOAbortCode write(uint16_t index, uint8_t subindex,
                               const uint8_t* data, size_t dataLen) = 0;
    
    // ========================================================================
    // Object Information
    // ========================================================================
    
    /**
     * @brief Check if object exists
     */
    virtual bool hasObject(uint16_t index, uint8_t subindex) const = 0;
    
    /**
     * @brief Get object information
     */
    virtual bool getObjectInfo(uint16_t index, uint8_t subindex,
                               ODEntryInfo& info) const = 0;
    
    /**
     * @brief Get number of subindices for an object
     */
    virtual uint8_t getSubindexCount(uint16_t index) const = 0;
    
    // ========================================================================
    // Object Registration
    // ========================================================================
    
    /**
     * @brief Register a new object
     */
    virtual bool registerObject(const ODEntryInfo& info) = 0;
    
    /**
     * @brief Register object with callback
     *
     * Callback is invoked on read/write for dynamic objects.
     */
    using ReadCallback = std::function<SDOAbortCode(uint8_t* data, size_t& len)>;
    using WriteCallback = std::function<SDOAbortCode(const uint8_t* data, size_t len)>;
    
    virtual bool registerObject(const ODEntryInfo& info,
                                ReadCallback readCb,
                                WriteCallback writeCb) = 0;
    
    // ========================================================================
    // PDO Mapping
    // ========================================================================
    
    /**
     * @brief Get PDO mapping entries
     *
     * @param pdoIndex PDO mapping object index (0x1600-0x17FF for RxPDO, 0x1A00-0x1BFF for TxPDO)
     * @param entries Output: list of mapped objects
     * @return true if PDO exists
     */
    virtual bool getPDOMapping(uint16_t pdoIndex,
                               std::vector<ODEntryInfo>& entries) const = 0;
    
    // ========================================================================
    // Serialization
    // ========================================================================
    
    /**
     * @brief Save object dictionary to file/memory
     */
    virtual bool save(std::vector<uint8_t>& data) const = 0;
    
    /**
     * @brief Load object dictionary from file/memory
     */
    virtual bool load(const std::vector<uint8_t>& data) = 0;
};

// ============================================================================
// Factory Functions for Mailbox Handlers
// ============================================================================

/**
 * @brief Create CoE (CANopen over EtherCAT) handler
 *
 * @param od Object dictionary to use
 */
std::unique_ptr<IMailboxHandler> createCoEHandler(std::shared_ptr<IObjectDictionary> od);

/**
 * @brief Create FoE (File over EtherCAT) handler
 *
 * @param rootPath Root path for file operations
 */
std::unique_ptr<IMailboxHandler> createFoEHandler(const std::string& rootPath);

/**
 * @brief Create EoE (Ethernet over EtherCAT) handler
 */
std::unique_ptr<IMailboxHandler> createEoEHandler();

/**
 * @brief Create VoE (Vendor over EtherCAT) handler
 */
std::unique_ptr<IMailboxHandler> createVoEHandler();

/**
 * @brief Create SoE (Servo over EtherCAT) handler
 */
std::unique_ptr<IMailboxHandler> createSoEHandler();

/**
 * @brief Create AoE (ADS over EtherCAT) handler
 */
std::unique_ptr<IMailboxHandler> createAoEHandler();

// ============================================================================
// Factory Function for Object Dictionary
// ============================================================================

/**
 * @brief Create a basic object dictionary
 */
std::unique_ptr<IObjectDictionary> createObjectDictionary();

}  // namespace slave
}  // namespace EtherCAT
