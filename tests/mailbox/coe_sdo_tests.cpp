#include <gtest/gtest.h>
#include "ethercat/raw/internal.hpp"
#include "tether/ethercat/EtherCATMaster.hpp"

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
    EtherCAT::EtherCATMaster master;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt, /*mbx_write_addr=*/0x1000, /*mbx_write_len=*/4,
                             /*mbx_read_addr=*/0x1080, /*mbx_read_len=*/8, /*index=*/0x2000, /*sub=*/0, outbuf, sizeof(outbuf), &out_len);
    EXPECT_FALSE(ok);
}

TEST(CoeSDO, Download_InvalidParams_ReturnsFalse) {
    uint8_t mbx_cnt = 0;
    uint8_t data[5] = {1,2,3,4,5};
    EtherCAT::EtherCATMaster master;

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
    EtherCAT::EtherCATMaster master;
    // mailbox write len too small to hold header+SdoInitDownloadReq
    EXPECT_FALSE(coe_sdo_download(master, 0x0000, &mbx_cnt, 2, 2, 4, 4, 0x2000, 0x0, data, 4));
}
