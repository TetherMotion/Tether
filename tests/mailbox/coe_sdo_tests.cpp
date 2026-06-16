#include <cstddef>
#include <gtest/gtest.h>
#include "ethercat/raw/internal.hpp"
#include "tether/ethercat/Master.hpp"

using namespace EtherCAT::Raw;

TEST(CoeHelpers, MbxTypeAndCoeRawEncodeDecode) {
    uint8_t t = mbx_type_with_cnt(0x0Au, 0x05u);
    EXPECT_EQ((t & 0x0Fu), 0x0Au);
    EXPECT_EQ(((t >> 4) & 0x0Fu), 0x05u);

    uint16_t raw = coe_make_raw(0x15Au, 0x0Bu);
    EXPECT_EQ((raw & 0x01FFu), 0x15Au);
    EXPECT_EQ(((raw >> 12) & 0x0Fu), 0x0Bu);
}

TEST(CoeStructs, SdoStructSizes) {
    EXPECT_EQ(sizeof(MbxHeader), 6u);
    EXPECT_EQ(sizeof(CoeHeader), 2u);
    EXPECT_EQ(sizeof(SdoInitUploadReq), 8u);
    EXPECT_EQ(sizeof(SdoInitUploadRes), 8u);
    EXPECT_EQ(sizeof(SdoInitDownloadReq), 8u);
    EXPECT_EQ(sizeof(SdoAbort), 8u);
}

TEST(CoeSDO, Upload_SmallMailboxWrite_ReturnsFalse) {
    // Provide tiny mailbox lengths to trigger early error path
    uint8_t mbx_cnt = 0;
    uint8_t outbuf[8] = {0};
    size_t out_len = 0;
    EtherCAT::Master master;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt, /*mbx_write_addr=*/0x1000, /*mbx_write_len=*/4,
                             /*mbx_read_addr=*/0x1080, /*mbx_read_len=*/8, /*index=*/0x2000, /*sub=*/0, outbuf, sizeof(outbuf), &out_len);
    EXPECT_FALSE(ok);
}

TEST(CoeSDO, Download_InvalidParams_ReturnsFalse) {
    uint8_t mbx_cnt = 0;
    uint8_t data[5] = {1,2,3,4,5};
    EtherCAT::Master master;

    // Null data
    EXPECT_FALSE(coe_sdo_download(master, 0x0000, &mbx_cnt, 0x1000, 128, 0x1080, 128, 0x2000, 0x0, nullptr, 4));

    // Zero length
    EXPECT_FALSE(coe_sdo_download(master, 0x0000, &mbx_cnt, 0x1000, 128, 0x1080, 128, 0x2000, 0x0, data, 0));

    // Length > 4
    EXPECT_FALSE(coe_sdo_download(master, 0x0000, &mbx_cnt, 0x1000, 128, 0x1080, 128, 0x2000, 0x0, data, 5));
}

TEST(CoeSDO, Download_SmallMailboxSize_ReturnsFalse) {
    uint8_t mbx_cnt = 0;
    uint8_t data[4] = {1,2,3,4};
    EtherCAT::Master master;
    // mailbox write len too small to hold header+SdoInitDownloadReq
    EXPECT_FALSE(coe_sdo_download(master, 0x0000, &mbx_cnt, 2, 2, 4, 4, 0x2000, 0x0, data, 4));
}

// ============================================================================
// SyncManager register address and bit mask tests (ETG.1000.4 conformance)
// ============================================================================

TEST(SyncManagerRegs, StatusAddressCalculation) {
    // ETG.1000.4 Table 59: each SM channel occupies 8 bytes starting at 0x0800.
    // The Status register is at offset 5 within each 8-byte block.
    // Test all 16 entries of the SM vtable.
    for (uint8_t sm = 0; sm <= 15; ++sm) {
        const uint16_t base = static_cast<uint16_t>(0x0800 + (sm * 8u));
        const uint16_t expected_status = static_cast<uint16_t>(base + 5u);
        EXPECT_EQ(sm_status_address(sm), expected_status)
            << "SM" << static_cast<int>(sm) << " status address mismatch";
    }
}

TEST(SyncManagerRegs, StatusBitMasksMatchSpec) {
    // ETG.1000.4 Figure 36: TSYNCMAN.status bit layout
    // Bit 0: WriteEvent
    // Bit 1: ReadEvent
    // Bit 3: mailboxState (buffer full)
    EXPECT_EQ(EC_SM_STATUS_WRITE_EVENT, 0x01u);
    EXPECT_EQ(EC_SM_STATUS_READ_EVENT,  0x02u);
    EXPECT_EQ(EC_SM_STATUS_MBXFULL,     0x08u);

    // Verify bit positions
    EXPECT_EQ(EC_SM_STATUS_WRITE_EVENT & 0xFE, 0x00u); // only bit 0 set
    EXPECT_EQ(EC_SM_STATUS_READ_EVENT  & 0xFD, 0x00u); // only bit 1 set
    EXPECT_EQ(EC_SM_STATUS_MBXFULL     & 0xF7, 0x00u); // only bit 3 set
}

TEST(SyncManagerRegs, StructOffsets) {
    using namespace EtherCAT::Raw;
    EXPECT_EQ(offsetof(SyncManagerRegs, physStart_le), 0u);
    EXPECT_EQ(offsetof(SyncManagerRegs, length_le),     2u);
    EXPECT_EQ(offsetof(SyncManagerRegs, control),       4u);
    EXPECT_EQ(offsetof(SyncManagerRegs, status),        5u); // <-- critical for SM polling
    EXPECT_EQ(offsetof(SyncManagerRegs, activate),       6u);
    EXPECT_EQ(offsetof(SyncManagerRegs, pdiControl),     7u);
    EXPECT_EQ(sizeof(SyncManagerRegs),                  8u);
}
