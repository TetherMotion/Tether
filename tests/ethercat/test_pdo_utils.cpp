/**
 * @file test_pdo_utils.cpp
 * @brief Tests for the PDO diagnostics utilities — readPDOMapping(),
 *        pdoMappingToString() and the printPDOMapping() logging wrapper.
 *
 * Uses a GMock ISDOTransport behind a CoE::CoEManager so no real mailbox
 * traffic is required.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cstring>
#include <vector>

#include "tether/ethercat/CoEManager.hpp"
#include "tether/ethercat/utils/PDO.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Utils;
using namespace EtherCAT::SDO;
using ::testing::_;
using ::testing::Invoke;

namespace {

class MockSDOTransport : public ISDOTransport {
public:
    MOCK_METHOD(bool, sdoUpload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 uint8_t* out, size_t out_cap, size_t* out_len,
                 bool diag_enabled,
                 unsigned int poll_interval_ms,
                 unsigned int transaction_timeout_ms),
                (override));

    MOCK_METHOD(bool, sdoDownload,
                (uint16_t slave_index, uint8_t* mbx_counter,
                 uint16_t mbx_wr_addr, uint16_t mbx_wr_len,
                 uint16_t mbx_rd_addr, uint16_t mbx_rd_len,
                 uint16_t index, uint8_t sub,
                 const uint8_t* data, size_t data_len,
                 bool diag_enabled,
                 unsigned int poll_interval_ms,
                 unsigned int transaction_timeout_ms),
                (override));

    MOCK_METHOD(uint64_t, getMicroseconds, (), (override));
};

static auto UploadOk(const void* data, size_t len) {
    return [data, len](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
                       uint16_t, uint8_t, uint8_t* out, size_t out_cap,
                       size_t* out_len, bool, unsigned int, unsigned int) -> bool {
        const size_t cp = std::min(len, out_cap);
        std::memcpy(out, data, cp);
        if (out_len) *out_len = cp;
        return true;
    };
}

static auto UploadFail() {
    return [](uint16_t, uint8_t*, uint16_t, uint16_t, uint16_t, uint16_t,
              uint16_t, uint8_t, uint8_t*, size_t, size_t*, bool,
              unsigned int, unsigned int) -> bool {
        return false;
    };
}

/// Configure and start a CoE manager for the mocked mailbox.
void prime(CoE::CoEManager& mgr) {
    mgr.configureMailbox(0x1000, 128, 0x1400, 128);
    ASSERT_TRUE(mgr.init());
}

}  // namespace

TEST(PDOUtils, ReadMappingSuccess)
{
    MockSDOTransport transport;
    const uint8_t count = 2;
    const uint32_t entry1 = (0x6040u << 16) | (0u << 8) | 16u;  // 0x6040:00
    const uint32_t entry2 = (0x607Au << 16) | (1u << 8) | 32u;  // 0x607A:01

    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&count, sizeof(count))));
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 1, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&entry1, sizeof(entry1))));
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 2, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&entry2, sizeof(entry2))));

    CoE::CoEManager mgr(0, transport);
    prime(mgr);

    std::vector<PDOMappingEntry> entries;
    ASSERT_TRUE(readPDOMapping(mgr, 0x1705, entries, 100));
    ASSERT_EQ(entries.size(), 2u);
    EXPECT_EQ(entries[0].index, 0x6040);
    EXPECT_EQ(entries[0].subindex, 0);
    EXPECT_EQ(entries[0].bit_length, 16);
    EXPECT_EQ(entries[0].byte_offset, 0);

    EXPECT_EQ(entries[1].index, 0x607A);
    EXPECT_EQ(entries[1].subindex, 1);
    EXPECT_EQ(entries[1].bit_length, 32);
    EXPECT_EQ(entries[1].byte_offset, 2);  // 16 bits / 8 = 2 bytes
}

TEST(PDOUtils, ReadMappingCountFail)
{
    MockSDOTransport transport;
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoE::CoEManager mgr(0, transport);
    prime(mgr);

    std::vector<PDOMappingEntry> entries{{0}};
    EXPECT_FALSE(readPDOMapping(mgr, 0x1B04, entries, 100));
    EXPECT_TRUE(entries.empty());
}

TEST(PDOUtils, ReadMappingStopsOnEntryFailure)
{
    MockSDOTransport transport;
    const uint8_t count = 3;
    const uint32_t entry1 = (0x6040u << 16) | (0u << 8) | 16u;

    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&count, sizeof(count))));
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 1, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&entry1, sizeof(entry1))));
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 2, _, _, _, _, _, _))
        .WillByDefault(UploadFail());
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 3, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoE::CoEManager mgr(0, transport);
    prime(mgr);

    // The count read succeeds, so the helper reports success but keeps only
    // the entries gathered before the first failure.
    std::vector<PDOMappingEntry> entries;
    EXPECT_TRUE(readPDOMapping(mgr, 0x1B04, entries, 100));
    ASSERT_EQ(entries.size(), 1u);
    EXPECT_EQ(entries[0].index, 0x6040);
}

TEST(PDOUtils, ReadMappingZeroEntries)
{
    MockSDOTransport transport;
    const uint8_t count = 0;
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&count, sizeof(count))));

    CoE::CoEManager mgr(0, transport);
    prime(mgr);

    std::vector<PDOMappingEntry> entries;
    EXPECT_TRUE(readPDOMapping(mgr, 0x1B04, entries, 100));
    EXPECT_TRUE(entries.empty());
}

TEST(PDOUtils, PrintMappingSuccess)
{
    MockSDOTransport transport;
    const uint8_t count = 1;
    const uint32_t entry = (0x6040u << 16) | (0u << 8) | 16u;

    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&count, sizeof(count))));
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 1, _, _, _, _, _, _))
        .WillByDefault(Invoke(UploadOk(&entry, sizeof(entry))));

    CoE::CoEManager mgr(0, transport);
    prime(mgr);

    // Exercise the logging wrapper; it only formats and logs.
    printPDOMapping(mgr, false, 0x1705, "TEST", 100);
}

TEST(PDOUtils, PrintMappingFail)
{
    MockSDOTransport transport;
    ON_CALL(transport, sdoUpload(_, _, _, _, _, _, _, 0, _, _, _, _, _, _))
        .WillByDefault(UploadFail());

    CoE::CoEManager mgr(0, transport);
    prime(mgr);

    printPDOMapping(mgr, true, 0x1B04, "TEST", 100);
}

TEST(PDOUtils, MappingToStringFormatting)
{
    std::vector<PDOMappingEntry> entries;
    entries.push_back({0x6040, 0, 16, 0});
    entries.push_back({0x607A, 1, 32, 2});

    const std::string out = pdoMappingToString(true, 0x1B04, entries);
    EXPECT_NE(out.find("TxPDO"), std::string::npos);
    EXPECT_NE(out.find("0x1B04"), std::string::npos);
    EXPECT_NE(out.find("0x6040"), std::string::npos);
    EXPECT_NE(out.find("0x607A"), std::string::npos);
    EXPECT_NE(out.find("bits"), std::string::npos);
    EXPECT_NE(out.find("offset 2 bytes"), std::string::npos);
    EXPECT_NE(out.find("Total mapped: 6 bytes"), std::string::npos);
}

TEST(PDOUtils, MappingToStringEmptyList)
{
    const std::string out = pdoMappingToString(false, 0x1600, {});
    EXPECT_NE(out.find("RxPDO 0x1600 Mapping Entries"), std::string::npos);
    EXPECT_NE(out.find("Total mapped: 0 bytes"), std::string::npos);
}
