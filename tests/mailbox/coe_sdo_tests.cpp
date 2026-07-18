#include <cstddef>
#include <cstring>
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
    EXPECT_EQ(sizeof(SdoDownloadSegReq), 8u);
    EXPECT_EQ(sizeof(SdoDownloadSegRes), 8u);
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

// ============================================================================
// Stale response rejection tests
// ============================================================================

// Helper: build a CoE SDO upload response in a mailbox buffer
static void buildSdoUploadResponse(uint8_t* buf, uint8_t mbx_cnt,
                                   uint16_t index, uint8_t sub,
                                   uint32_t data) {
    // Mailbox header (6 bytes)
    MbxHeader mbx{};
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoInitUploadRes)));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);
    std::memcpy(buf, &mbx, sizeof(mbx));

    // CoE header (2 bytes)
    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDORES));
    std::memcpy(buf + sizeof(mbx), &coe, sizeof(coe));

    // SDO upload response (8 bytes): expedited, 4 bytes, CCS=2
    SdoInitUploadRes sdo{};
    sdo.cmd = 0x43; // CCS=2 (upload response), expedited, size indicated, n=0
    sdo.index_le = host_to_le16(index);
    sdo.sub = sub;
    sdo.data_or_size_le = host_to_le32(data);
    std::memcpy(buf + sizeof(mbx) + sizeof(coe), &sdo, sizeof(sdo));
}

// Mock that serves SM status reads and mailbox data reads for coe_sdo_upload.
// It first returns a stale response (wrong counter), then the correct response.
class StaleResponseMock {
public:
    static constexpr uint16_t SM0_STATUS_ADDR = 0x0805;
    static constexpr uint16_t SM1_STATUS_ADDR = 0x0805 + 8;
    static constexpr uint16_t MBX_WRITE_ADDR = 0x1000;
    static constexpr uint16_t MBX_READ_ADDR = 0x1080;
    static constexpr uint16_t MBX_LEN = 128;

    uint8_t expected_cnt = 1;
    uint16_t expected_index = 0x1000;
    uint8_t expected_sub = 0;
    uint32_t response_data = 0xDEADBEEF;

    // Stale response params
    uint8_t stale_cnt = 7;
    uint16_t stale_index = 0x2000;
    uint8_t stale_sub = 1;

    int sm1_full_reads = 0;
    int mailbox_data_reads = 0;
    bool write_happened = false;
    EtherCAT::Master* master_ = nullptr;

    void install(EtherCAT::Master& master) {
        master_ = &master;

        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
                                          void* out, uint16_t len, unsigned int ms) {
            (void)adp; (void)ms;

            // SM0 status read — always not full (0x00)
            if (ado == SM0_STATUS_ADDR && len >= 1) {
                uint8_t val = 0x00;
                std::memcpy(out, &val, 1);
                return true;
            }

            // SM1 status read — only report full after the write has occurred
            // (so mbx_drain_all_stale before the write sees SM1 empty)
            if (ado == SM1_STATUS_ADDR && len >= 1) {
                uint8_t val = write_happened ? EC_SM_STATUS_MBXFULL : 0x00;
                std::memcpy(out, &val, 1);
                if (write_happened) sm1_full_reads++;
                return true;
            }

            // Mailbox data read (MBX_READ_ADDR)
            if (ado == MBX_READ_ADDR && len >= 16) {
                // First read after write: return stale response (wrong counter/index)
                if (mailbox_data_reads == 0) {
                    buildSdoUploadResponse(static_cast<uint8_t*>(out), stale_cnt,
                                           stale_index, stale_sub, 0xAAAAAAAA);
                } else {
                    // Second read: return correct response
                    buildSdoUploadResponse(static_cast<uint8_t*>(out), expected_cnt,
                                           expected_index, expected_sub, response_data);
                }
                mailbox_data_reads++;
                return true;
            }

            // Default: return zeros
            std::memset(out, 0, len);
            return true;
        });

        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                                          const void* data, uint16_t len, unsigned int ms) {
            (void)adp; (void)ado; (void)data; (void)len; (void)ms;
            write_happened = true;
            return true; // Accept all writes
        });
    }

    void remove() {
        if (master_) {
            master_->setAprdTestCallback(nullptr);
            master_->setApwrTestCallback(nullptr);
        }
    }
};

TEST(CoeSDO, Upload_RejectsStaleResponse_WrongCounter) {
    EtherCAT::Master master;
    StaleResponseMock mock;
    mock.expected_cnt = 1;
    mock.expected_index = 0x1000;
    mock.expected_sub = 0;
    mock.response_data = 0xDEADBEEF;
    mock.stale_cnt = 7;        // Different counter
    mock.stale_index = 0x1000;  // Same index — isolate counter check
    mock.stale_sub = 0;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t outbuf[256] = {0};
    size_t out_len = 0;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt,
                             StaleResponseMock::MBX_WRITE_ADDR, StaleResponseMock::MBX_LEN,
                             StaleResponseMock::MBX_READ_ADDR, StaleResponseMock::MBX_LEN,
                             0x1000, 0, outbuf, sizeof(outbuf), &out_len,
                             false, 5, 200);

    // Should succeed — stale response skipped, correct response accepted
    EXPECT_TRUE(ok);
    EXPECT_GE(mock.mailbox_data_reads, 2) << "Should have read at least twice (stale + correct)";

    mock.remove();
}

TEST(CoeSDO, Upload_RejectsStaleResponse_WrongIndex) {
    EtherCAT::Master master;
    StaleResponseMock mock;
    mock.expected_cnt = 1;
    mock.expected_index = 0x1000;
    mock.expected_sub = 0;
    mock.response_data = 0x12345678;
    mock.stale_cnt = 1;         // Same counter — isolate index check
    mock.stale_index = 0x2000;  // Different index
    mock.stale_sub = 0;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t outbuf[256] = {0};
    size_t out_len = 0;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt,
                             StaleResponseMock::MBX_WRITE_ADDR, StaleResponseMock::MBX_LEN,
                             StaleResponseMock::MBX_READ_ADDR, StaleResponseMock::MBX_LEN,
                             0x1000, 0, outbuf, sizeof(outbuf), &out_len,
                             false, 5, 200);

    EXPECT_TRUE(ok);
    EXPECT_GE(mock.mailbox_data_reads, 2) << "Should have read at least twice (stale + correct)";

    mock.remove();
}

// ============================================================================
// Download stale response tests
// ============================================================================

static void buildSdoDownloadResponse(uint8_t* buf, uint8_t mbx_cnt,
                                     uint16_t index, uint8_t sub) {
    MbxHeader mbx{};
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoInitDownloadRes)));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);
    std::memcpy(buf, &mbx, sizeof(mbx));

    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDORES));
    std::memcpy(buf + sizeof(mbx), &coe, sizeof(coe));

    SdoInitDownloadRes sdo{};
    sdo.cmd = 0x60;
    sdo.index_le = host_to_le16(index);
    sdo.sub = sub;
    std::memcpy(buf + sizeof(mbx) + sizeof(coe), &sdo, sizeof(sdo));
}

static void buildSdoAbortResponse(uint8_t* buf, uint8_t mbx_cnt,
                                  uint16_t index, uint8_t sub,
                                  uint32_t abort_code) {
    MbxHeader mbx{};
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoAbort)));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);
    std::memcpy(buf, &mbx, sizeof(mbx));

    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDORES));
    std::memcpy(buf + sizeof(mbx), &coe, sizeof(coe));

    SdoAbort abort{};
    abort.cmd = EC_SDO_ABORT;
    abort.index_le = host_to_le16(index);
    abort.sub = sub;
    abort.abortCode_le = host_to_le32(abort_code);
    std::memcpy(buf + sizeof(mbx) + sizeof(coe), &abort, sizeof(abort));
}

static void buildSdoDownloadSegmentResponse(uint8_t* buf, uint8_t mbx_cnt,
                                            bool toggle) {
    MbxHeader mbx{};
    mbx.length_le = host_to_le16(static_cast<uint16_t>(sizeof(CoeHeader) + sizeof(SdoDownloadSegRes)));
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_COE, mbx_cnt);
    std::memcpy(buf, &mbx, sizeof(mbx));

    CoeHeader coe{};
    coe.raw_le = host_to_le16(coe_make_raw(0, EC_COES_SDORES));
    std::memcpy(buf + sizeof(mbx), &coe, sizeof(coe));

    SdoDownloadSegRes sdo{};
    sdo.cmd = static_cast<uint8_t>(0x20u | (toggle ? 0x10u : 0x00u));
    std::memcpy(buf + sizeof(mbx) + sizeof(coe), &sdo, sizeof(sdo));
}

// Extended mock supporting upload, download, abort, and timeout scenarios.
class StaleResponseMockExt {
public:
    static constexpr uint16_t SM0_STATUS_ADDR = 0x0805;
    static constexpr uint16_t SM1_STATUS_ADDR = 0x0805 + 8;
    static constexpr uint16_t MBX_WRITE_ADDR = 0x1000;
    static constexpr uint16_t MBX_READ_ADDR = 0x1080;
    static constexpr uint16_t MBX_LEN = 128;

    int num_stale_responses = 1;
    uint8_t stale_cnt = 7;
    uint16_t stale_index = 0x2000;
    uint8_t stale_sub = 1;

    uint8_t final_cnt = 1;
    uint16_t final_index = 0x1000;
    uint8_t final_sub = 0;
    uint32_t final_data = 0xDEADBEEF;
    bool final_is_download = false;
    bool final_is_abort = false;
    uint32_t abort_code = 0x06020000;
    bool no_final_response = false;

    int mailbox_data_reads = 0;
    bool write_happened = false;
    EtherCAT::Master* master_ = nullptr;

    void install(EtherCAT::Master& master) {
        master_ = &master;

        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
                                          void* out, uint16_t len, unsigned int ms) {
            (void)adp; (void)ms;

            if (ado == SM0_STATUS_ADDR && len >= 1) {
                uint8_t val = 0x00;
                std::memcpy(out, &val, 1);
                return true;
            }

            if (ado == SM1_STATUS_ADDR && len >= 1) {
                uint8_t val = write_happened ? EC_SM_STATUS_MBXFULL : 0x00;
                std::memcpy(out, &val, 1);
                return true;
            }

            if (ado == MBX_READ_ADDR && len >= 16) {
                int idx = mailbox_data_reads++;
                if (idx < num_stale_responses || no_final_response) {
                    if (final_is_download || no_final_response) {
                        buildSdoDownloadResponse(static_cast<uint8_t*>(out), stale_cnt,
                                                 stale_index, stale_sub);
                    } else {
                        buildSdoUploadResponse(static_cast<uint8_t*>(out), stale_cnt,
                                               stale_index, stale_sub, 0xAAAAAAAA);
                    }
                } else {
                    if (final_is_abort) {
                        buildSdoAbortResponse(static_cast<uint8_t*>(out), final_cnt,
                                              final_index, final_sub, abort_code);
                    } else if (final_is_download) {
                        buildSdoDownloadResponse(static_cast<uint8_t*>(out), final_cnt,
                                                 final_index, final_sub);
                    } else {
                        buildSdoUploadResponse(static_cast<uint8_t*>(out), final_cnt,
                                               final_index, final_sub, final_data);
                    }
                }
                return true;
            }

            std::memset(out, 0, len);
            return true;
        });

        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                                          const void* data, uint16_t len, unsigned int ms) {
            (void)adp; (void)ado; (void)data; (void)len; (void)ms;
            write_happened = true;
            return true;
        });
    }

    void remove() {
        if (master_) {
            master_->setAprdTestCallback(nullptr);
            master_->setApwrTestCallback(nullptr);
        }
    }
};

TEST(CoeSDO, Download_RejectsStaleResponse_WrongCounter) {
    EtherCAT::Master master;
    StaleResponseMockExt mock;
    mock.num_stale_responses = 1;
    mock.stale_cnt = 7;
    mock.stale_index = 0x1000;
    mock.stale_sub = 0;
    mock.final_cnt = 1;
    mock.final_index = 0x1000;
    mock.final_sub = 0;
    mock.final_is_download = true;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               StaleResponseMockExt::MBX_WRITE_ADDR, StaleResponseMockExt::MBX_LEN,
                               StaleResponseMockExt::MBX_READ_ADDR, StaleResponseMockExt::MBX_LEN,
                               0x1000, 0, data, 4,
                               false, 5, 200);

    EXPECT_TRUE(ok);
    EXPECT_GE(mock.mailbox_data_reads, 2);

    mock.remove();
}

TEST(CoeSDO, Download_RejectsStaleResponse_WrongIndex) {
    EtherCAT::Master master;
    StaleResponseMockExt mock;
    mock.num_stale_responses = 1;
    mock.stale_cnt = 1;
    mock.stale_index = 0x2000;
    mock.stale_sub = 0;
    mock.final_cnt = 1;
    mock.final_index = 0x1000;
    mock.final_sub = 0;
    mock.final_is_download = true;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               StaleResponseMockExt::MBX_WRITE_ADDR, StaleResponseMockExt::MBX_LEN,
                               StaleResponseMockExt::MBX_READ_ADDR, StaleResponseMockExt::MBX_LEN,
                               0x1000, 0, data, 4,
                               false, 5, 200);

    EXPECT_TRUE(ok);
    EXPECT_GE(mock.mailbox_data_reads, 2);

    mock.remove();
}

// ============================================================================
// Upload edge case tests
// ============================================================================

TEST(CoeSDO, Upload_RejectsStaleResponse_WrongSubindex) {
    EtherCAT::Master master;
    StaleResponseMockExt mock;
    mock.num_stale_responses = 1;
    mock.stale_cnt = 1;
    mock.stale_index = 0x1000;
    mock.stale_sub = 5;
    mock.final_cnt = 1;
    mock.final_index = 0x1000;
    mock.final_sub = 0;
    mock.final_data = 0xCAFEBABE;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t outbuf[256] = {0};
    size_t out_len = 0;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt,
                             StaleResponseMockExt::MBX_WRITE_ADDR, StaleResponseMockExt::MBX_LEN,
                             StaleResponseMockExt::MBX_READ_ADDR, StaleResponseMockExt::MBX_LEN,
                             0x1000, 0, outbuf, sizeof(outbuf), &out_len,
                             false, 5, 200);

    EXPECT_TRUE(ok);
    EXPECT_GE(mock.mailbox_data_reads, 2);

    mock.remove();
}

TEST(CoeSDO, Upload_AllStaleResponses_TimeoutReturnsFalse) {
    EtherCAT::Master master;
    StaleResponseMockExt mock;
    mock.no_final_response = true;
    mock.stale_cnt = 7;
    mock.stale_index = 0x1000;
    mock.stale_sub = 0;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t outbuf[256] = {0};
    size_t out_len = 0;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt,
                             StaleResponseMockExt::MBX_WRITE_ADDR, StaleResponseMockExt::MBX_LEN,
                             StaleResponseMockExt::MBX_READ_ADDR, StaleResponseMockExt::MBX_LEN,
                             0x1000, 0, outbuf, sizeof(outbuf), &out_len,
                             false, 5, 100);

    EXPECT_FALSE(ok) << "Should time out when all responses are stale";

    mock.remove();
}

TEST(CoeSDO, Upload_StaleThenAbort_ReturnsFalse) {
    EtherCAT::Master master;
    StaleResponseMockExt mock;
    mock.num_stale_responses = 1;
    mock.stale_cnt = 7;
    mock.stale_index = 0x1000;
    mock.stale_sub = 0;
    mock.final_cnt = 1;
    mock.final_index = 0x1000;
    mock.final_sub = 0;
    mock.final_is_abort = true;
    mock.abort_code = 0x06020000;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t outbuf[256] = {0};
    size_t out_len = 0;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt,
                             StaleResponseMockExt::MBX_WRITE_ADDR, StaleResponseMockExt::MBX_LEN,
                             StaleResponseMockExt::MBX_READ_ADDR, StaleResponseMockExt::MBX_LEN,
                             0x1000, 0, outbuf, sizeof(outbuf), &out_len,
                             false, 5, 200);

    EXPECT_FALSE(ok) << "SDO abort should cause upload to return false";
    EXPECT_GE(mock.mailbox_data_reads, 2);

    mock.remove();
}

// ============================================================================
// Multi-stale response tests: verify re-send and recovery
// ============================================================================

// Test that multiple stale responses are cleared via re-send and the correct
// response is eventually accepted.  This simulates the real-world lockup
// scenario where the slave has several queued stale responses.
TEST(CoeSDO, Upload_MultipleStaleResponses_ClearsAndResendsUntilSuccess) {
    EtherCAT::Master master;
    StaleResponseMockExt mock;
    mock.num_stale_responses = 5;   // 5 stale responses before the correct one
    mock.stale_cnt = 7;
    mock.stale_index = 0x1000;
    mock.stale_sub = 0;
    mock.final_cnt = 1;
    mock.final_index = 0x1000;
    mock.final_sub = 0;
    mock.final_data = 0xCAFEBABE;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t outbuf[256] = {0};
    size_t out_len = 0;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt,
                             StaleResponseMockExt::MBX_WRITE_ADDR, StaleResponseMockExt::MBX_LEN,
                             StaleResponseMockExt::MBX_READ_ADDR, StaleResponseMockExt::MBX_LEN,
                             0x1000, 0, outbuf, sizeof(outbuf), &out_len,
                             false, 5, 200);

    EXPECT_TRUE(ok) << "Should succeed after clearing multiple stale responses via re-send";
    EXPECT_GE(mock.mailbox_data_reads, 6) << "Should have read 5 stale + 1 correct response";
    if (ok && out_len >= 4) {
        uint32_t val = 0;
        std::memcpy(&val, outbuf, 4);
        EXPECT_EQ(val, 0xCAFEBABE);
    }

    mock.remove();
}

// Test that multiple stale download responses are cleared via re-send.
TEST(CoeSDO, Download_MultipleStaleResponses_ClearsAndResendsUntilSuccess) {
    EtherCAT::Master master;
    StaleResponseMockExt mock;
    mock.num_stale_responses = 3;
    mock.stale_cnt = 7;
    mock.stale_index = 0x1000;
    mock.stale_sub = 0;
    mock.final_cnt = 1;
    mock.final_index = 0x1000;
    mock.final_sub = 0;
    mock.final_is_download = true;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               StaleResponseMockExt::MBX_WRITE_ADDR, StaleResponseMockExt::MBX_LEN,
                               StaleResponseMockExt::MBX_READ_ADDR, StaleResponseMockExt::MBX_LEN,
                               0x1000, 0, data, 4,
                               false, 5, 200);

    EXPECT_TRUE(ok) << "Should succeed after clearing multiple stale responses via re-send";
    EXPECT_GE(mock.mailbox_data_reads, 4) << "Should have read 3 stale + 1 correct response";

    mock.remove();
}

// ============================================================================
// Segmented download tests
// ============================================================================

// Mock that serves an init download response followed by segment download
// responses.  Responses are returned in the order the master reads them.
class SegmentedDownloadMock {
public:
    static constexpr uint16_t SM0_STATUS_ADDR = 0x0805;
    static constexpr uint16_t SM1_STATUS_ADDR = 0x0805 + 8;
    static constexpr uint16_t MBX_WRITE_ADDR = 0x1000;
    static constexpr uint16_t MBX_READ_ADDR = 0x1080;
    static constexpr uint16_t MBX_LEN = 128;

    uint16_t index = 0x1000;
    uint8_t sub = 0;
    int abort_on_segment = -1;
    uint32_t abort_code = 0x06070010;
    uint8_t sm1_full_flag = EC_SM_STATUS_MBXFULL;

    int mailbox_data_reads = 0;
    bool write_happened = false;
    EtherCAT::Master* master_ = nullptr;

    void install(EtherCAT::Master& master) {
        master_ = &master;

        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
                                          void* out, uint16_t len, unsigned int ms) {
            (void)adp; (void)ms;

            if (ado == SM0_STATUS_ADDR && len >= 1) {
                uint8_t val = 0x00;
                std::memcpy(out, &val, 1);
                return true;
            }

            if (ado == SM1_STATUS_ADDR && len >= 1) {
                uint8_t val = write_happened ? sm1_full_flag : 0x00;
                std::memcpy(out, &val, 1);
                return true;
            }

            if (ado == MBX_READ_ADDR && len >= 16) {
                int idx = mailbox_data_reads++;
                if (idx == 0) {
                    // Init download response (counter matches init request: 0)
                    buildSdoDownloadResponse(static_cast<uint8_t*>(out), 0, index, sub);
                } else {
                    // Segment download response. Segment N uses counter N and
                    // alternates toggle starting with false for segment 1.
                    int seg_idx = idx - 1;
                    bool toggle = (seg_idx % 2) != 0;
                    if (seg_idx == abort_on_segment) {
                        buildSdoAbortResponse(static_cast<uint8_t*>(out),
                                              static_cast<uint8_t>(idx),
                                              index, sub, abort_code);
                    } else {
                        buildSdoDownloadSegmentResponse(static_cast<uint8_t*>(out),
                                                        static_cast<uint8_t>(idx), toggle);
                    }
                }
                return true;
            }

            std::memset(out, 0, len);
            return true;
        });

        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                                          const void* data, uint16_t len, unsigned int ms) {
            (void)adp; (void)ado; (void)data; (void)len; (void)ms;
            write_happened = true;
            return true;
        });
    }

    void remove() {
        if (master_) {
            master_->setAprdTestCallback(nullptr);
            master_->setApwrTestCallback(nullptr);
        }
    }
};

TEST(CoeSDO, Download_Segmented_5Bytes_Succeeds) {
    EtherCAT::Master master;
    SegmentedDownloadMock mock;
    mock.index = 0x1000;
    mock.sub = 0;
    mock.install(master);

    uint8_t mbx_cnt = 0;
    uint8_t data[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               SegmentedDownloadMock::MBX_WRITE_ADDR, SegmentedDownloadMock::MBX_LEN,
                               SegmentedDownloadMock::MBX_READ_ADDR, SegmentedDownloadMock::MBX_LEN,
                               0x1000, 0, data, sizeof(data),
                               false, 5, 200);

    EXPECT_TRUE(ok) << "5-byte segmented download should succeed";
    EXPECT_EQ(mock.mailbox_data_reads, 2) << "Expected init response + 1 segment response";

    mock.remove();
}

TEST(CoeSDO, Download_Segmented_9Bytes_Succeeds) {
    EtherCAT::Master master;
    SegmentedDownloadMock mock;
    mock.index = 0x1000;
    mock.sub = 0;
    mock.install(master);

    uint8_t mbx_cnt = 0;
    uint8_t data[9] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               SegmentedDownloadMock::MBX_WRITE_ADDR, SegmentedDownloadMock::MBX_LEN,
                               SegmentedDownloadMock::MBX_READ_ADDR, SegmentedDownloadMock::MBX_LEN,
                               0x1000, 0, data, sizeof(data),
                               false, 5, 200);

    EXPECT_TRUE(ok) << "9-byte segmented download should succeed";
    EXPECT_EQ(mock.mailbox_data_reads, 3) << "Expected init response + 2 segment responses";

    mock.remove();
}

TEST(CoeSDO, Download_Segmented_AbortInSegment_Fails) {
    EtherCAT::Master master;
    SegmentedDownloadMock mock;
    mock.index = 0x1000;
    mock.sub = 0;
    mock.abort_on_segment = 0;
    mock.abort_code = 0x06070010;
    mock.install(master);

    uint8_t mbx_cnt = 0;
    uint8_t data[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               SegmentedDownloadMock::MBX_WRITE_ADDR, SegmentedDownloadMock::MBX_LEN,
                               SegmentedDownloadMock::MBX_READ_ADDR, SegmentedDownloadMock::MBX_LEN,
                               0x1000, 0, data, sizeof(data),
                               false, 5, 200);

    EXPECT_FALSE(ok) << "Segment abort should cause download to fail";
    EXPECT_GE(mock.mailbox_data_reads, 2) << "Expected init response + abort response";

    mock.remove();
}

// Verify that segmented download succeeds when the ESC signals a full SM1 via
// the WRITE_BUFFER_FULL flag (bit 7) instead of the traditional MBXFULL flag.
TEST(CoeSDO, Download_Segmented_SM1WriteBufferFull_Succeeds) {
    EtherCAT::Master master;
    SegmentedDownloadMock mock;
    mock.index = 0x1000;
    mock.sub = 0;
    mock.sm1_full_flag = EC_SM_STATUS_WRITE_BUFFER_FULL;
    mock.install(master);

    uint8_t mbx_cnt = 0;
    uint8_t data[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               SegmentedDownloadMock::MBX_WRITE_ADDR, SegmentedDownloadMock::MBX_LEN,
                               SegmentedDownloadMock::MBX_READ_ADDR, SegmentedDownloadMock::MBX_LEN,
                               0x1000, 0, data, sizeof(data),
                               false, 5, 200);

    EXPECT_TRUE(ok) << "Segmented download should succeed with WRITE_BUFFER_FULL flag";
    EXPECT_EQ(mock.mailbox_data_reads, 2) << "Expected init response + 1 segment response";

    mock.remove();
}

// ============================================================================
// Mailbox error response (type 0) tests — RZ/T2M infinite loop regression
// ============================================================================

// Helper: build a mailbox error response (type 0x00, not CoE)
static void buildMailboxErrorResponse(uint8_t* buf, uint8_t mbx_cnt,
                                       uint16_t err_code, uint16_t err_detail) {
    MbxHeader mbx{};
    mbx.length_le = host_to_le16(4); // 4 bytes of error payload
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(EC_MBXT_ERR, mbx_cnt); // type 0x00
    std::memcpy(buf, &mbx, sizeof(mbx));

    // Error code + detail (4 bytes)
    uint16_t err_le = host_to_le16(err_code);
    uint16_t detail_le = host_to_le16(err_detail);
    std::memcpy(buf + sizeof(mbx), &err_le, 2);
    std::memcpy(buf + sizeof(mbx) + 2, &detail_le, 2);
}

// Mock that returns a mailbox error response (type 0) instead of a CoE SDO response.
class MailboxErrorMock {
public:
    static constexpr uint16_t SM0_STATUS_ADDR = 0x0805;
    static constexpr uint16_t SM1_STATUS_ADDR = 0x0805 + 8;
    static constexpr uint16_t MBX_WRITE_ADDR = 0x1000;
    static constexpr uint16_t MBX_READ_ADDR = 0x1080;
    static constexpr uint16_t MBX_LEN = 128;

    uint8_t mbx_cnt = 1;
    uint16_t err_code = 0x0001;
    uint16_t err_detail = 0x0000;

    int mailbox_data_reads = 0;
    int sm1_status_reads = 0;
    bool write_happened = false;
    EtherCAT::Master* master_ = nullptr;

    void install(EtherCAT::Master& master) {
        master_ = &master;

        master.setAprdTestCallback([this](uint16_t adp, uint16_t ado,
                                          void* out, uint16_t len, unsigned int ms) {
            (void)adp; (void)ms;

            if (ado == SM0_STATUS_ADDR && len >= 1) {
                uint8_t val = 0x00;
                std::memcpy(out, &val, 1);
                return true;
            }

            if (ado == SM1_STATUS_ADDR && len >= 1) {
                sm1_status_reads++;
                uint8_t val = write_happened ? EC_SM_STATUS_MBXFULL : 0x00;
                std::memcpy(out, &val, 1);
                return true;
            }

            if (ado == MBX_READ_ADDR && len >= 16) {
                mailbox_data_reads++;
                buildMailboxErrorResponse(static_cast<uint8_t*>(out), mbx_cnt,
                                          err_code, err_detail);
                return true;
            }

            std::memset(out, 0, len);
            return true;
        });

        master.setApwrTestCallback([this](uint16_t adp, uint16_t ado,
                                          const void* data, uint16_t len, unsigned int ms) {
            (void)adp; (void)ado; (void)data; (void)len; (void)ms;
            write_happened = true;
            return true;
        });
    }

    void remove() {
        if (master_) {
            master_->setAprdTestCallback(nullptr);
            master_->setApwrTestCallback(nullptr);
        }
    }
};

// Test: Download receives mailbox error response (type 0) — should return false
// immediately, not loop 50 times on the empty buffer.
TEST(CoeSDO, Download_MailboxErrorResponse_ReturnsFalseQuickly) {
    EtherCAT::Master master;
    MailboxErrorMock mock;
    mock.mbx_cnt = 1;
    mock.err_code = 0x0001;
    mock.err_detail = 0x0000;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               MailboxErrorMock::MBX_WRITE_ADDR, MailboxErrorMock::MBX_LEN,
                               MailboxErrorMock::MBX_READ_ADDR, MailboxErrorMock::MBX_LEN,
                               0x1000, 0, data, 4,
                               false, 5, 200);

    EXPECT_FALSE(ok) << "Mailbox error response should cause download to fail";
    EXPECT_EQ(mock.mailbox_data_reads, 1) << "Should read mailbox data exactly once (error response), not loop";

    mock.remove();
}

// Test: Upload receives mailbox error response (type 0) — should return false
// immediately, not loop on the empty buffer.
TEST(CoeSDO, Upload_MailboxErrorResponse_ReturnsFalseQuickly) {
    EtherCAT::Master master;
    MailboxErrorMock mock;
    mock.mbx_cnt = 1;
    mock.err_code = 0x0002;
    mock.err_detail = 0x0001;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t outbuf[256] = {0};
    size_t out_len = 0;

    bool ok = coe_sdo_upload(master, 0x0000, &mbx_cnt,
                             MailboxErrorMock::MBX_WRITE_ADDR, MailboxErrorMock::MBX_LEN,
                             MailboxErrorMock::MBX_READ_ADDR, MailboxErrorMock::MBX_LEN,
                             0x1000, 0, outbuf, sizeof(outbuf), &out_len,
                             false, 5, 200);

    EXPECT_FALSE(ok) << "Mailbox error response should cause upload to fail";
    EXPECT_EQ(mock.mailbox_data_reads, 1) << "Should read mailbox data exactly once (error response), not loop";

    mock.remove();
}

// Test: Segmented download init receives mailbox error response — should return
// false immediately.
TEST(CoeSDO, Download_Segmented_MailboxErrorInInit_ReturnsFalseQuickly) {
    EtherCAT::Master master;
    MailboxErrorMock mock;
    mock.mbx_cnt = 0; // Segmented download starts with counter 0
    mock.err_code = 0x0001;
    mock.err_detail = 0x0000;
    mock.install(master);

    uint8_t mbx_cnt = 0;
    uint8_t data[5] = {0x01, 0x02, 0x03, 0x04, 0x05};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               MailboxErrorMock::MBX_WRITE_ADDR, MailboxErrorMock::MBX_LEN,
                               MailboxErrorMock::MBX_READ_ADDR, MailboxErrorMock::MBX_LEN,
                               0x1000, 0, data, sizeof(data),
                               false, 5, 200);

    EXPECT_FALSE(ok) << "Mailbox error in segmented download init should fail";
    EXPECT_EQ(mock.mailbox_data_reads, 1) << "Should read mailbox data exactly once (error response), not loop";

    mock.remove();
}

// ============================================================================
// Non-CoE response tests
// ============================================================================

// Helper: build a non-CoE mailbox response (e.g. EoE type 0x02)
static void buildNonCoeResponse(uint8_t* buf, uint8_t mbx_cnt, uint8_t mbx_type) {
    MbxHeader mbx{};
    mbx.length_le = host_to_le16(8);
    mbx.address_le = host_to_le16(0);
    mbx.priority = 0;
    mbx.mbxtype = mbx_type_with_cnt(mbx_type, mbx_cnt);
    std::memcpy(buf, &mbx, sizeof(mbx));
    // Fill remaining with non-zero data to distinguish from empty buffer
    for (size_t i = sizeof(mbx); i < 32; ++i) {
        buf[i] = 0xFF;
    }
}

// Test: Download receives non-CoE response (EoE type 0x02) — should abort, not loop
TEST(CoeSDO, Download_NonCoeResponse_AbortsCleanly) {
    EtherCAT::Master master;
    MailboxErrorMock mock;
    // Override the data read to return a non-CoE response
    mock.install(master);
    master.setAprdTestCallback([&mock](uint16_t adp, uint16_t ado,
                                       void* out, uint16_t len, unsigned int ms) {
        (void)adp; (void)ms;

        if (ado == MailboxErrorMock::SM0_STATUS_ADDR && len >= 1) {
            uint8_t val = 0x00;
            std::memcpy(out, &val, 1);
            return true;
        }

        if (ado == MailboxErrorMock::SM1_STATUS_ADDR && len >= 1) {
            mock.sm1_status_reads++;
            uint8_t val = mock.write_happened ? EC_SM_STATUS_MBXFULL : 0x00;
            std::memcpy(out, &val, 1);
            return true;
        }

        if (ado == MailboxErrorMock::MBX_READ_ADDR && len >= 16) {
            mock.mailbox_data_reads++;
            // Return EoE (type 0x02) response instead of CoE
            buildNonCoeResponse(static_cast<uint8_t*>(out), 1, 0x02);
            return true;
        }

        std::memset(out, 0, len);
        return true;
    });

    uint8_t mbx_cnt = 1;
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               MailboxErrorMock::MBX_WRITE_ADDR, MailboxErrorMock::MBX_LEN,
                               MailboxErrorMock::MBX_READ_ADDR, MailboxErrorMock::MBX_LEN,
                               0x1000, 0, data, 4,
                               false, 5, 200);

    EXPECT_FALSE(ok) << "Non-CoE response should cause download to fail (timeout after break)";
    EXPECT_EQ(mock.mailbox_data_reads, 1) << "Should read mailbox data exactly once, not loop on empty buffer";

    mock.remove();
}

// ============================================================================
// Status-gated read tests — verify master re-checks SM1 status before data reads
// ============================================================================

// Test: Download status-gated read — master should check SM1 status before
// each data read, not blindly read the data buffer.
TEST(CoeSDO, Download_StatusGatedRead_ChecksSM1BeforeData) {
    EtherCAT::Master master;
    StaleResponseMockExt mock;
    mock.final_cnt = 1;
    mock.final_index = 0x1000;
    mock.final_sub = 0;
    mock.final_is_download = true;
    mock.num_stale_responses = 0;
    mock.install(master);

    uint8_t mbx_cnt = 1;
    uint8_t data[4] = {0x01, 0x02, 0x03, 0x04};

    bool ok = coe_sdo_download(master, 0x0000, &mbx_cnt,
                               StaleResponseMockExt::MBX_WRITE_ADDR, StaleResponseMockExt::MBX_LEN,
                               StaleResponseMockExt::MBX_READ_ADDR, StaleResponseMockExt::MBX_LEN,
                               0x1000, 0, data, 4,
                               false, 5, 200);

    EXPECT_TRUE(ok) << "Download should succeed with status-gated reads";
    EXPECT_EQ(mock.mailbox_data_reads, 1) << "Should read mailbox data exactly once for a clean response";

    mock.remove();
}
