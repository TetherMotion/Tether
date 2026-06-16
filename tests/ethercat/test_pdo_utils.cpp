#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "tether/ethercat/utils/PDO.hpp"
#include "tether/ethercat/SDOManager.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Utils;
using namespace EtherCAT::SDO;
using ::testing::_;
using ::testing::Invoke;
using ::testing::Return;

// reuse MockSDOTransport from existing test file; duplicate minimal version
namespace {

class MockSDOTransport : public ISDOTransport {
public:
    MOCK_METHOD(bool, sdoUpload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 uint8_t* out, size_t out_cap, size_t* out_len),
                (override));
    MOCK_METHOD(bool, sdoDownload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 const uint8_t* data, size_t data_len),
                (override));
    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
};

// Helper from test_sdo_async (simplified) -----------------------------------
static auto UploadOk(const void* data, size_t len) {
    return [data, len](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                       uint16_t, uint8_t, uint8_t* out, size_t out_cap, size_t* out_len) -> bool {
        size_t cp = std::min(len, out_cap);
        std::memcpy(out, data, cp);
        if (out_len) *out_len = cp;
        return true;
    };
}

static auto UploadFail() {
    return [](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
              uint16_t, uint8_t, uint8_t*, size_t, size_t*) -> bool {
        return false;
    };
}

} // namespace

// -----------------------------------------------------------------------------
TEST(PDOUtils, ReadMappingSuccess)
{
    MockSDOTransport transport;
    // count = 2 entries
    uint8_t count = 2;
    // two 32-bit raw mapping values
    uint32_t entry1 = (0x6040u << 16) | (0u << 8) | 16u; // 0x6040:00 16bits
    uint32_t entry2 = (0x607Au << 16) | (0u << 8) | 32u; // 0x607A:00 32bits

    // first call (subindex 0) returns count
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&count, sizeof(count))));
    // subsequent calls for subindex 1 and 2
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 1, _, _, _))
        .WillByDefault(Invoke(UploadOk(&entry1, sizeof(entry1))));
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 2, _, _, _))
        .WillByDefault(Invoke(UploadOk(&entry2, sizeof(entry2))));

    SDOManager mgr(transport);
    mgr.init();
    mgr.configureSlaveMailbox(0, 0x1000, 128, 0x1400, 128);

    std::vector<PDOMappingEntry> entries;
    bool ok = readPDOMapping(mgr, 0, 0x1705, entries, 100);
    EXPECT_TRUE(ok);
    ASSERT_EQ(entries.size(), 2u);
    EXPECT_EQ(entries[0].index, 0x6040);
    EXPECT_EQ(entries[0].subindex, 0);
    EXPECT_EQ(entries[0].bit_length, 16);
    EXPECT_EQ(entries[0].byte_offset, 0);

    EXPECT_EQ(entries[1].index, 0x607A);
    EXPECT_EQ(entries[1].subindex, 0);
    EXPECT_EQ(entries[1].bit_length, 32);
    EXPECT_EQ(entries[1].byte_offset, 2); // 16 bits / 8 = 2 bytes

    mgr.deinit();
}

TEST(PDOUtils, PrintMappingSuccess)
{
    MockSDOTransport transport;
    uint8_t count = 1;
    uint32_t entry = (0x6040u << 16) | (0u << 8) | 16u;
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _))
        .WillByDefault(Invoke(UploadOk(&count, sizeof(count))));
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 1, _, _, _))
        .WillByDefault(Invoke(UploadOk(&entry, sizeof(entry))));

    SDOManager mgr(transport);
    mgr.init();
    mgr.configureSlaveMailbox(0, 0x1000, 128, 0x1400, 128);

    // exercise logging wrapper; we don't assert on stdout, just ensure it runs
    printPDOMapping(mgr, 0, false, 0x1705, "TEST", 100);

    mgr.deinit();
}

TEST(PDOUtils, PrintMappingFail)
{
    MockSDOTransport transport;
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _))
        .WillByDefault(UploadFail());
    SDOManager mgr(transport);
    mgr.init();
    mgr.configureSlaveMailbox(0, 0x1000, 128, 0x1400, 128);

    // call with failure path – should log a warning but not crash
    printPDOMapping(mgr, 0, true, 0x1B04, "TEST", 100);

    mgr.deinit();
}

TEST(PDOUtils, ReadMappingCountFail)
{
    MockSDOTransport transport;
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _))
        .WillByDefault(UploadFail());
    SDOManager mgr(transport);
    mgr.init();
    mgr.configureSlaveMailbox(0, 0x1000, 128, 0x1400, 128);

    std::vector<PDOMappingEntry> entries = {{0}};
    bool ok = readPDOMapping(mgr, 0, 0x1B04, entries, 100);
    EXPECT_FALSE(ok);
    EXPECT_TRUE(entries.empty());

    mgr.deinit();
}

TEST(PDOUtils, MappingToStringFormatting)
{
    std::vector<PDOMappingEntry> entries;
    entries.push_back({0x6040, 0, 16, 0});
    entries.push_back({0x607A, 1, 32, 2});

    std::string out = pdoMappingToString(true, 0x1B04, entries);
    EXPECT_NE(out.find("TxPDO"), std::string::npos);
    EXPECT_NE(out.find("0x1B04"), std::string::npos);
    EXPECT_NE(out.find("0x6040"), std::string::npos);
    EXPECT_NE(out.find("0x607A"), std::string::npos);
    EXPECT_NE(out.find("bits"), std::string::npos);
    EXPECT_NE(out.find("offset 2 bytes"), std::string::npos);
}
