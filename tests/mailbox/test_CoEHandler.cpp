/**
 * @file test_CoEHandler.cpp
 * @brief Tests for ObjectDictionaryImpl and CoEHandler
 *        Covers CoEHandler.cpp (0% → target 100%).
 *
 * Tests object dictionary CRUD operations, SDO upload/download protocol
 * processing, abort codes, and edge cases.
 */
#include <gtest/gtest.h>
#include <tether/slave/mailbox/IMailboxHandler.hpp>
#include <cstring>
#include <memory>
#include <vector>

using namespace EtherCAT::slave;

// ============================================================================
// Helper: build mailbox+CoE+SDO request
// ============================================================================
static constexpr size_t kMbxHdrSize = 6;
static constexpr size_t kCoeHdrSize = 2;
static constexpr size_t kSdoDataSize = 8;
static constexpr size_t kMinRequestSize = kMbxHdrSize + kCoeHdrSize + kSdoDataSize;

// SDO Client Command Specifiers
static constexpr uint8_t SDO_CCS_DOWNLOAD_INIT = 1;
static constexpr uint8_t SDO_CCS_UPLOAD_INIT = 2;
static constexpr uint8_t SDO_CCS_ABORT = 4;

// SDO Server Command Specifiers
static constexpr uint8_t SDO_SCS_UPLOAD_INIT = 2;
static constexpr uint8_t SDO_SCS_DOWNLOAD_INIT = 3;

// CoE service types
static constexpr uint8_t COE_SDO_REQ = 0x02;

static void buildSdoUploadRequest(uint8_t* buf, uint16_t index, uint8_t subindex, uint8_t counter = 1) {
    std::memset(buf, 0, kMinRequestSize);
    // Mailbox header
    uint16_t dataLen = kCoeHdrSize + kSdoDataSize;
    buf[0] = dataLen & 0xFF;
    buf[1] = (dataLen >> 8) & 0xFF;
    buf[2] = 0; buf[3] = 0; // address
    buf[4] = 0; // channel | priority
    buf[5] = 0x03 | ((counter & 0x0F) << 4); // type=CoE(0x03), counter

    // CoE header: service = SDO_REQ (0x02) in bits 12-15
    uint16_t coeHdr = (COE_SDO_REQ << 12);
    buf[6] = coeHdr & 0xFF;
    buf[7] = (coeHdr >> 8) & 0xFF;

    // SDO data
    buf[8] = (SDO_CCS_UPLOAD_INIT << 5); // CCS upload init
    buf[9] = index & 0xFF;
    buf[10] = (index >> 8) & 0xFF;
    buf[11] = subindex;
}

static void buildSdoDownloadRequest(uint8_t* buf, uint16_t index, uint8_t subindex,
                                     const uint8_t* data, size_t dataLen, uint8_t counter = 1) {
    std::memset(buf, 0, kMinRequestSize);
    // Mailbox header
    uint16_t mboxDataLen = kCoeHdrSize + kSdoDataSize;
    buf[0] = mboxDataLen & 0xFF;
    buf[1] = (mboxDataLen >> 8) & 0xFF;
    buf[4] = 0;
    buf[5] = 0x03 | ((counter & 0x0F) << 4);

    // CoE header
    uint16_t coeHdr = (COE_SDO_REQ << 12);
    buf[6] = coeHdr & 0xFF;
    buf[7] = (coeHdr >> 8) & 0xFF;

    // SDO data: expedited download
    size_t n = (dataLen <= 4) ? (4 - dataLen) : 0;
    buf[8] = (SDO_CCS_DOWNLOAD_INIT << 5) | (static_cast<uint8_t>(n) << 2) | 0x02 | 0x01;
    buf[9] = index & 0xFF;
    buf[10] = (index >> 8) & 0xFF;
    buf[11] = subindex;
    size_t copyLen = std::min(dataLen, size_t(4));
    std::memcpy(&buf[12], data, copyLen);
}

// ============================================================================
// ObjectDictionary Tests
// ============================================================================
class ObjectDictionaryTest : public ::testing::Test {
protected:
    std::unique_ptr<IObjectDictionary> od_;

    void SetUp() override {
        od_ = createObjectDictionary();
    }
};

TEST_F(ObjectDictionaryTest, EmptyDictionary) {
    EXPECT_FALSE(od_->hasObject(0x2000, 0));
    EXPECT_EQ(od_->getSubindexCount(0x2000), 0u);
}

TEST_F(ObjectDictionaryTest, RegisterAndHasObject) {
    ODEntryInfo info;
    info.index = 0x2000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned32;
    info.bitLength = 32;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.name = "TestObject";
    info.defaultValue = 0x12345678;

    EXPECT_TRUE(od_->registerObject(info));
    EXPECT_TRUE(od_->hasObject(0x2000, 0));
    EXPECT_FALSE(od_->hasObject(0x2001, 0));
}

TEST_F(ObjectDictionaryTest, ReadAfterRegister) {
    ODEntryInfo info;
    info.index = 0x2000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned32;
    info.bitLength = 32;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.name = "TestU32";
    info.defaultValue = 0x42;

    od_->registerObject(info);

    uint8_t data[4] = {};
    size_t len = 4;
    auto code = od_->read(0x2000, 0, data, len);
    EXPECT_EQ(code, SDOAbortCode::Success);
    EXPECT_EQ(len, 4u);
    // Default value stored LE
    uint32_t val = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
    EXPECT_EQ(val, 0x42u);
}

TEST_F(ObjectDictionaryTest, WriteAndReadBack) {
    ODEntryInfo info;
    info.index = 0x2000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned32;
    info.bitLength = 32;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.name = "TestRW";
    info.defaultValue = 0;

    od_->registerObject(info);

    // Write a value
    uint8_t writeData[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    auto wcode = od_->write(0x2000, 0, writeData, 4);
    EXPECT_EQ(wcode, SDOAbortCode::Success);

    // Read it back
    uint8_t readData[4] = {};
    size_t len = 4;
    auto rcode = od_->read(0x2000, 0, readData, len);
    EXPECT_EQ(rcode, SDOAbortCode::Success);
    EXPECT_EQ(readData[0], 0xAAu);
    EXPECT_EQ(readData[1], 0xBBu);
    EXPECT_EQ(readData[2], 0xCCu);
    EXPECT_EQ(readData[3], 0xDDu);
}

TEST_F(ObjectDictionaryTest, ReadNonExistent) {
    uint8_t data[4] = {};
    size_t len = 4;
    auto code = od_->read(0x9999, 0, data, len);
    EXPECT_EQ(code, SDOAbortCode::ObjectNotFound);
}

TEST_F(ObjectDictionaryTest, WriteNonExistent) {
    uint8_t data[4] = {1, 2, 3, 4};
    auto code = od_->write(0x9999, 0, data, 4);
    EXPECT_EQ(code, SDOAbortCode::ObjectNotFound);
}

TEST_F(ObjectDictionaryTest, WriteToReadOnlyObject) {
    ODEntryInfo info;
    info.index = 0x2000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned16;
    info.bitLength = 16;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadOnly);
    info.name = "ReadOnlyObj";
    info.defaultValue = 42;

    od_->registerObject(info);
    uint8_t data[2] = {0xFF, 0xFF};
    auto code = od_->write(0x2000, 0, data, 2);
    // Should reject write to read-only
    EXPECT_NE(code, SDOAbortCode::Success);
}

TEST_F(ObjectDictionaryTest, ReadFromWriteOnlyObject) {
    ODEntryInfo info;
    info.index = 0x2000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned16;
    info.bitLength = 16;
    info.accessType = static_cast<uint8_t>(ODAccessType::WriteOnly);
    info.name = "WriteOnlyObj";
    info.defaultValue = 0;

    od_->registerObject(info);
    uint8_t data[2] = {};
    size_t len = 2;
    auto code = od_->read(0x2000, 0, data, len);
    // Should reject read from write-only
    EXPECT_NE(code, SDOAbortCode::Success);
}

TEST_F(ObjectDictionaryTest, GetObjectInfo) {
    ODEntryInfo info;
    info.index = 0x2000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned32;
    info.bitLength = 32;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.name = "InfoTest";
    info.defaultValue = 0;

    od_->registerObject(info);

    ODEntryInfo retrieved;
    EXPECT_TRUE(od_->getObjectInfo(0x2000, 0, retrieved));
    EXPECT_EQ(retrieved.index, 0x2000u);
    EXPECT_EQ(retrieved.subindex, 0u);
    EXPECT_EQ(retrieved.bitLength, 32u);
    EXPECT_EQ(retrieved.name, "InfoTest");
}

TEST_F(ObjectDictionaryTest, GetObjectInfoNonExistent) {
    ODEntryInfo retrieved;
    EXPECT_FALSE(od_->getObjectInfo(0x9999, 0, retrieved));
}

TEST_F(ObjectDictionaryTest, SubindexCount) {
    ODEntryInfo info;
    info.index = 0x2000;
    info.dataType = ObjectDictionaryDataType::Unsigned8;
    info.bitLength = 8;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.defaultValue = 0;

    info.subindex = 0; info.name = "Sub0";
    od_->registerObject(info);
    info.subindex = 1; info.name = "Sub1";
    od_->registerObject(info);
    info.subindex = 2; info.name = "Sub2";
    od_->registerObject(info);

    EXPECT_GE(od_->getSubindexCount(0x2000), 2u);
    EXPECT_EQ(od_->getSubindexCount(0x9999), 0u);
}

TEST_F(ObjectDictionaryTest, RegisterWithCallbacks) {
    ODEntryInfo info;
    info.index = 0x3000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned32;
    info.bitLength = 32;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.name = "CallbackObj";
    info.defaultValue = 0;

    uint32_t customValue = 0xDEADBEEF;
    auto readCb = [&customValue](uint8_t* data, size_t& len) -> SDOAbortCode {
        len = std::min(len, size_t(4));
        std::memcpy(data, &customValue, len);
        return SDOAbortCode::Success;
    };
    auto writeCb = [&customValue](const uint8_t* data, size_t len) -> SDOAbortCode {
        if (len >= 4) std::memcpy(&customValue, data, 4);
        return SDOAbortCode::Success;
    };

    EXPECT_TRUE(od_->registerObject(info, readCb, writeCb));

    // Read via callback
    uint8_t buf[4] = {};
    size_t len = 4;
    auto code = od_->read(0x3000, 0, buf, len);
    EXPECT_EQ(code, SDOAbortCode::Success);
    uint32_t readVal;
    std::memcpy(&readVal, buf, 4);
    EXPECT_EQ(readVal, 0xDEADBEEFu);

    // Write via callback
    uint32_t newVal = 0x12345678;
    code = od_->write(0x3000, 0, reinterpret_cast<const uint8_t*>(&newVal), 4);
    EXPECT_EQ(code, SDOAbortCode::Success);
    EXPECT_EQ(customValue, 0x12345678u);
}

TEST_F(ObjectDictionaryTest, SaveAndLoad) {
    ODEntryInfo info;
    info.index = 0x2000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned32;
    info.bitLength = 32;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.name = "SaveObj";
    info.defaultValue = 0;
    od_->registerObject(info);

    // Write some data
    uint8_t data[4] = {0x11, 0x22, 0x33, 0x44};
    od_->write(0x2000, 0, data, 4);

    // Save
    std::vector<uint8_t> savedData;
    EXPECT_TRUE(od_->save(savedData));

    // Create new OD and load
    auto od2 = createObjectDictionary();
    EXPECT_TRUE(od2->load(savedData));
}

TEST_F(ObjectDictionaryTest, Unsigned8ObjectWriteRead) {
    ODEntryInfo info;
    info.index = 0x2001;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned8;
    info.bitLength = 8;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.name = "U8";
    info.defaultValue = 0;
    od_->registerObject(info);

    // Write a value then read it back
    uint8_t writeData[1] = {0xFF};
    auto wcode = od_->write(0x2001, 0, writeData, 1);
    EXPECT_EQ(wcode, SDOAbortCode::Success);

    uint8_t data[1] = {};
    size_t len = 1;
    auto rcode = od_->read(0x2001, 0, data, len);
    EXPECT_EQ(rcode, SDOAbortCode::Success);
    EXPECT_EQ(data[0], 0xFFu);
}

TEST_F(ObjectDictionaryTest, Unsigned16ObjectWriteRead) {
    ODEntryInfo info;
    info.index = 0x2002;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned16;
    info.bitLength = 16;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.name = "U16";
    info.defaultValue = 0;
    od_->registerObject(info);

    uint8_t writeData[2] = {0x34, 0x12};
    auto wcode = od_->write(0x2002, 0, writeData, 2);
    EXPECT_EQ(wcode, SDOAbortCode::Success);

    uint8_t data[2] = {};
    size_t len = 2;
    auto rcode = od_->read(0x2002, 0, data, len);
    EXPECT_EQ(rcode, SDOAbortCode::Success);
    uint16_t val = data[0] | (data[1] << 8);
    EXPECT_EQ(val, 0x1234u);
}

TEST_F(ObjectDictionaryTest, GetPDOMappingEmpty) {
    std::vector<ODEntryInfo> entries;
    EXPECT_FALSE(od_->getPDOMapping(0x1600, entries));
}

// ============================================================================
// CoEHandler Tests
// ============================================================================
class CoEHandlerTest : public ::testing::Test {
protected:
    std::shared_ptr<IObjectDictionary> od_;
    std::unique_ptr<IMailboxHandler> handler_;

    void SetUp() override {
        od_ = std::shared_ptr<IObjectDictionary>(createObjectDictionary().release());
        handler_ = createCoEHandler(od_);

        // Register a test object
        ODEntryInfo info;
        info.index = 0x2000;
        info.subindex = 0;
        info.dataType = ObjectDictionaryDataType::Unsigned32;
        info.bitLength = 32;
        info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
        info.name = "TestObj";
        info.defaultValue = 0x42;
        od_->registerObject(info);
    }
};

TEST_F(CoEHandlerTest, GetProtocol) {
    EXPECT_EQ(handler_->getProtocol(), MailboxProtocol::CoE);
}

TEST_F(CoEHandlerTest, GetProtocolName) {
    const char* name = handler_->getProtocolName();
    EXPECT_NE(name, nullptr);
    EXPECT_STREQ(name, "CoE");
}

TEST_F(CoEHandlerTest, Reset) {
    handler_->reset(); // should not crash
}

TEST_F(CoEHandlerTest, SdoUploadInit) {
    uint8_t request[kMinRequestSize] = {};
    buildSdoUploadRequest(request, 0x2000, 0);

    uint8_t response[256] = {};
    size_t responseLen = sizeof(response);
    bool ok = handler_->processRequest(request, kMinRequestSize, response, responseLen);
    EXPECT_TRUE(ok);
    EXPECT_GT(responseLen, kMbxHdrSize + kCoeHdrSize);

    // Verify response is SDO upload response
    // SDO data starts at offset 8
    uint8_t scs = (response[8] >> 5) & 0x07;
    EXPECT_EQ(scs, SDO_SCS_UPLOAD_INIT);
}

TEST_F(CoEHandlerTest, SdoUploadNonExistent) {
    uint8_t request[kMinRequestSize] = {};
    buildSdoUploadRequest(request, 0x9999, 0);

    uint8_t response[256] = {};
    size_t responseLen = sizeof(response);
    bool ok = handler_->processRequest(request, kMinRequestSize, response, responseLen);
    EXPECT_TRUE(ok);

    // Should be an abort response
    uint8_t ccs = (response[8] >> 5) & 0x07;
    EXPECT_EQ(ccs, SDO_CCS_ABORT);
}

TEST_F(CoEHandlerTest, SdoDownloadInit) {
    uint8_t request[kMinRequestSize] = {};
    uint8_t data[4] = {0xAA, 0xBB, 0xCC, 0xDD};
    buildSdoDownloadRequest(request, 0x2000, 0, data, 4);

    uint8_t response[256] = {};
    size_t responseLen = sizeof(response);
    bool ok = handler_->processRequest(request, kMinRequestSize, response, responseLen);
    EXPECT_TRUE(ok);

    // Verify it was a download init response
    uint8_t scs = (response[8] >> 5) & 0x07;
    EXPECT_EQ(scs, SDO_SCS_DOWNLOAD_INIT);

    // Verify the data was written
    uint8_t readBuf[4] = {};
    size_t readLen = 4;
    auto code = od_->read(0x2000, 0, readBuf, readLen);
    EXPECT_EQ(code, SDOAbortCode::Success);
    EXPECT_EQ(readBuf[0], 0xAAu);
}

TEST_F(CoEHandlerTest, SdoDownloadNonExistent) {
    uint8_t request[kMinRequestSize] = {};
    uint8_t data[4] = {1, 2, 3, 4};
    buildSdoDownloadRequest(request, 0x9999, 0, data, 4);

    uint8_t response[256] = {};
    size_t responseLen = sizeof(response);
    bool ok = handler_->processRequest(request, kMinRequestSize, response, responseLen);
    EXPECT_TRUE(ok);

    uint8_t ccs = (response[8] >> 5) & 0x07;
    EXPECT_EQ(ccs, SDO_CCS_ABORT);
}

TEST_F(CoEHandlerTest, RequestTooShort) {
    uint8_t request[4] = {};
    uint8_t response[256] = {};
    size_t responseLen = sizeof(response);
    bool ok = handler_->processRequest(request, 4, response, responseLen);
    EXPECT_FALSE(ok);
}

TEST_F(CoEHandlerTest, WrongMailboxType) {
    uint8_t request[kMinRequestSize] = {};
    buildSdoUploadRequest(request, 0x2000, 0);
    request[5] = 0x05; // FoE type instead of CoE
    
    uint8_t response[256] = {};
    size_t responseLen = sizeof(response);
    bool ok = handler_->processRequest(request, kMinRequestSize, response, responseLen);
    EXPECT_FALSE(ok);
}

TEST_F(CoEHandlerTest, MultipleSubindices) {
    // Register subindices 1 and 2
    ODEntryInfo info;
    info.index = 0x2000;
    info.dataType = ObjectDictionaryDataType::Unsigned16;
    info.bitLength = 16;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadWrite);
    info.defaultValue = 0;

    info.subindex = 1; info.name = "Sub1";
    od_->registerObject(info);
    info.subindex = 2; info.name = "Sub2";
    od_->registerObject(info);

    // Upload sub1
    uint8_t req[kMinRequestSize] = {};
    buildSdoUploadRequest(req, 0x2000, 1);
    uint8_t resp[256] = {};
    size_t respLen = sizeof(resp);
    bool ok = handler_->processRequest(req, kMinRequestSize, resp, respLen);
    EXPECT_TRUE(ok);
}

TEST_F(CoEHandlerTest, DownloadToReadOnly) {
    ODEntryInfo info;
    info.index = 0x3000;
    info.subindex = 0;
    info.dataType = ObjectDictionaryDataType::Unsigned32;
    info.bitLength = 32;
    info.accessType = static_cast<uint8_t>(ODAccessType::ReadOnly);
    info.name = "RO";
    info.defaultValue = 123;
    od_->registerObject(info);

    uint8_t request[kMinRequestSize] = {};
    uint8_t data[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    buildSdoDownloadRequest(request, 0x3000, 0, data, 4);

    uint8_t response[256] = {};
    size_t responseLen = sizeof(response);
    handler_->processRequest(request, kMinRequestSize, response, responseLen);

    // Should be an abort (write to read-only)
    uint8_t ccs = (response[8] >> 5) & 0x07;
    EXPECT_EQ(ccs, SDO_CCS_ABORT);
}

TEST_F(CoEHandlerTest, HasPendingResponseDefault) {
    EXPECT_FALSE(handler_->hasPendingResponse());
}

TEST_F(CoEHandlerTest, GetPendingResponseDefault) {
    uint8_t resp[256] = {};
    size_t respLen = sizeof(resp);
    EXPECT_FALSE(handler_->getPendingResponse(resp, respLen));
}

// ============================================================================
// MailboxHeader Tests
// ============================================================================
TEST(MailboxHeaderTest, FromBytes) {
    uint8_t data[6] = {};
    data[0] = 0x0A; data[1] = 0x00; // length = 10
    data[2] = 0x00; data[3] = 0x00; // address = 0
    data[4] = 0x00;                  // channel=0, priority=0
    data[5] = 0x03;                  // type=3 (CoE), counter=0

    auto hdr = MailboxHeader::fromBytes(data);
    EXPECT_EQ(hdr.length, 10u);
    EXPECT_EQ(hdr.type, 3u);
}

TEST(MailboxHeaderTest, ToBytes) {
    MailboxHeader hdr;
    hdr.length = 10;
    hdr.address = 0x1234;
    hdr.channel = 5;
    hdr.priority = 2;
    hdr.type = 3;
    hdr.counter = 7;

    uint8_t data[6] = {};
    hdr.toBytes(data);

    auto decoded = MailboxHeader::fromBytes(data);
    EXPECT_EQ(decoded.length, 10u);
    EXPECT_EQ(decoded.address, 0x1234u);
    EXPECT_EQ(decoded.channel, 5u);
    EXPECT_EQ(decoded.priority, 2u);
    EXPECT_EQ(decoded.type, 3u);
    EXPECT_EQ(decoded.counter, 7u);
}

TEST(MailboxHeaderTest, RoundTrip) {
    MailboxHeader hdr;
    hdr.length = 256;
    hdr.address = 0xABCD;
    hdr.channel = 0x3F;
    hdr.priority = 3;
    hdr.type = 0x0F;
    hdr.counter = 0x0F;

    uint8_t data[6] = {};
    hdr.toBytes(data);
    auto decoded = MailboxHeader::fromBytes(data);
    EXPECT_EQ(decoded.length, 256u);
    EXPECT_EQ(decoded.address, 0xABCDu);
    EXPECT_EQ(decoded.channel, 0x3Fu);
    EXPECT_EQ(decoded.priority, 3u);
    EXPECT_EQ(decoded.type, 0x0Fu);
    EXPECT_EQ(decoded.counter, 0x0Fu);
}

// ============================================================================
// SDOAbortCode enum values
// ============================================================================
TEST(SDOAbortCodeTest, Values) {
    EXPECT_EQ(static_cast<uint32_t>(SDOAbortCode::Success), 0x00000000u);
    EXPECT_EQ(static_cast<uint32_t>(SDOAbortCode::ObjectNotFound), 0x06020000u);
    EXPECT_EQ(static_cast<uint32_t>(SDOAbortCode::ReadOnlyObject), 0x06010001u);
    EXPECT_EQ(static_cast<uint32_t>(SDOAbortCode::WriteOnlyObject), 0x06010002u);
}

// ============================================================================
// Mailbox header size constant
// ============================================================================
TEST(MailboxConstantsTest, HeaderSize) {
    EXPECT_EQ(kMailboxHeaderSize, 6u);
}
