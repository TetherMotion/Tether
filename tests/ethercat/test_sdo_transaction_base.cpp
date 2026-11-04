/**
 * @file test_sdo_transaction_base.cpp
 * @brief Coverage for SDOTransactionBase — mailbox response polling,
 *        error decoding, stale-counter drain, counter adoption, and
 *        counter-space resync.
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <cstring>
#include <vector>

#include "tether/ethercat/Master.hpp"
#include "tether/ethercat/SDOTransactionBase.hpp"
#include "tether/ethercat/SDOUpload.hpp"
#include "tether/ethercat/SDODownload.hpp"
#include "tether/ethercat/SDOMailboxIO.hpp"
#include "tether/ethercat/SDODiagnostics.hpp"
#include "tether/ethercat/SDOErrorDecoder.hpp"
#include "raw/RawConstants.hpp"
#include "raw/RawWireFormat.hpp"

using namespace EtherCAT;
using namespace EtherCAT::Raw;
using ::testing::_;
using ::testing::Return;

namespace {

constexpr uint16_t kAdp       = 0;        // slave 0, positional
constexpr uint16_t kMbxWrAddr = 0x1000;
constexpr uint16_t kMbxRdAddr = 0x1080;
constexpr uint16_t kMbxLen    = 0x40;
constexpr uint16_t kIndex     = 0x6040;
constexpr uint8_t  kSub       = 0;

/// Build a mailbox frame: MbxHeader + body.
std::vector<uint8_t> makeMbx(uint8_t type, uint8_t cnt,
                             const std::vector<uint8_t>& body) {
    std::vector<uint8_t> buf(sizeof(MbxHeader) + body.size(), 0);
    auto* h = reinterpret_cast<MbxHeader*>(buf.data());
    h->length_le = static_cast<uint16_t>(body.size());
    h->mbxtype   = mbx_type_with_cnt(type, cnt);
    std::memcpy(buf.data() + sizeof(MbxHeader), body.data(), body.size());
    return buf;
}

/// CoE body echoing index:sub at the fixed init-response offset.
std::vector<uint8_t> coeEchoBody(uint8_t sdo_cmd, uint16_t index,
                               uint8_t sub) {
    std::vector<uint8_t> body(sizeof(CoeHeader) + 4, 0);
    auto* coe = reinterpret_cast<CoeHeader*>(body.data());
    coe->raw_le = coe_make_raw(EC_COES_SDORES, 0);
    body[sizeof(CoeHeader) + 0] = sdo_cmd;
    body[sizeof(CoeHeader) + 1] = static_cast<uint8_t>(index & 0xFF);
    body[sizeof(CoeHeader) + 2] = static_cast<uint8_t>(index >> 8);
    body[sizeof(CoeHeader) + 3] = sub;
    return body;
}

class SdoTxnBaseTest : public ::testing::Test {
protected:
    SDODiagnostics diag_;
    SDOErrorDecoder decoder_;
    SDOMailboxIO mio_{diag_};
    SDOUpload upload_{decoder_, mio_, diag_};
    SDODownload download_{decoder_, mio_, diag_};
    Master master_;

    void SetUp() override {
        master_.setPreopRetryConfig(1, 3, 1, 1);
        master_.initSlaves(1);
    }
};

}  // namespace

// ============================================================================
// isCounterMismatchError (static, pure)
// ============================================================================

TEST_F(SdoTxnBaseTest, CounterMismatchErrorDecoding) {
    // err=0x0001 detail=0x0004/0x0005 on an EC_MBXT_ERR frame → mismatch.
    auto mk = [&](uint16_t err, uint16_t detail) {
        std::vector<uint8_t> body(4);
        body[0] = static_cast<uint8_t>(err & 0xFF);
        body[1] = static_cast<uint8_t>(err >> 8);
        body[2] = static_cast<uint8_t>(detail & 0xFF);
        body[3] = static_cast<uint8_t>(detail >> 8);
        return makeMbx(EC_MBXT_ERR, 1, body);
    };

    MbxResponseHeader hdr{};
    hdr.type = EC_MBXT_ERR;
    hdr.len = 4;
    auto b = mk(0x0001, 0x0004);
    EXPECT_TRUE(SDOTransactionBase::isCounterMismatchError(b.data(), hdr));
    b = mk(0x0001, 0x0005);
    EXPECT_TRUE(SDOTransactionBase::isCounterMismatchError(b.data(), hdr));
    b = mk(0x0001, 0x0006);
    EXPECT_FALSE(SDOTransactionBase::isCounterMismatchError(b.data(), hdr));
    b = mk(0x0002, 0x0004);
    EXPECT_FALSE(SDOTransactionBase::isCounterMismatchError(b.data(), hdr));

    hdr.type = EC_MBXT_COE;                  // wrong frame type
    EXPECT_FALSE(SDOTransactionBase::isCounterMismatchError(b.data(), hdr));
    hdr.type = EC_MBXT_ERR;
    hdr.len = 2;                             // truncated
    EXPECT_FALSE(SDOTransactionBase::isCounterMismatchError(b.data(), hdr));
}

TEST_F(SdoTxnBaseTest, HandleMailboxErrorBothArms) {
    std::vector<uint8_t> body(4, 0);
    body[0] = 0x01; body[2] = 0x04;          // err=1 detail=4
    auto buf = makeMbx(EC_MBXT_ERR, 1, body);

    MbxResponseHeader hdr{};
    hdr.cnt = 3; hdr.len = 4;
    EXPECT_FALSE(upload_.handleMailboxError(buf.data(), hdr, kAdp,
                                          kIndex, kSub));
    hdr.len = 2;                             // truncated arm
    EXPECT_FALSE(upload_.handleMailboxError(buf.data(), hdr, kAdp,
                                          kIndex, kSub));
}

// ============================================================================
// checkStaleCounter — drain, throttle, always continue
// ============================================================================

TEST_F(SdoTxnBaseTest, CheckStaleCounterDrainsAndContinues) {
    uint8_t mbxbuf[kMbxLen] = {};
    MbxResponseHeader hdr{};
    hdr.cnt = 7;
    uint8_t mbx_cnt = 1, expected = 1;
    int stale = 0;

    // The log-throttle arms: counts ≤4 warn, then once per 32.
    for (int i = 1; i <= 33; ++i) {
        EXPECT_TRUE(upload_.checkStaleCounter(
            master_, kAdp, kMbxWrAddr, kMbxLen, kMbxRdAddr, kMbxLen,
            mbxbuf, 0, 100, hdr, &mbx_cnt, expected, stale,
            kIndex, kSub, "test"));
        EXPECT_EQ(stale, i);
    }
}

// ============================================================================
// adoptCounterOnEcho — echo validation + counter adoption
// ============================================================================

TEST_F(SdoTxnBaseTest, AdoptCounterOnEchoArms) {
    std::vector<uint8_t> req = makeMbx(EC_MBXT_COE, 2, coeEchoBody(0x23, kIndex, kSub));
    std::vector<uint8_t> rsp = makeMbx(EC_MBXT_COE, 5, coeEchoBody(0x60, kIndex, kSub));

    MbxResponseHeader hdr{};
    hdr.type = EC_MBXT_COE;
    hdr.cnt = 5;
    hdr.len = static_cast<uint16_t>(rsp.size() - sizeof(MbxHeader));

    uint8_t mbx_cnt = 2, cur = 0, expected = 2;
    EXPECT_TRUE(upload_.adoptCounterOnEcho(
        master_, kAdp, req.data(), rsp.data(), kMbxLen, hdr,
        &mbx_cnt, cur, expected, kIndex, kSub, "init"));
    EXPECT_EQ(expected, 5);
    EXPECT_EQ(cur, 6);                       // nextMbxCnt(5)
    EXPECT_EQ(mbx_cnt, 6);

    // Abort command (0x80) also echoes.
    rsp = makeMbx(EC_MBXT_COE, 3, coeEchoBody(EC_SDO_ABORT, kIndex, kSub));
    hdr.cnt = 3;
    expected = 1; cur = 0; mbx_cnt = 1;
    EXPECT_TRUE(upload_.adoptCounterOnEcho(
        master_, kAdp, req.data(), rsp.data(), kMbxLen, hdr,
        &mbx_cnt, cur, expected, kIndex, kSub, "init"));

    // Duplicate-counter guard: adopting cnt whose nextMbxCnt equals the
    // request's counter skips ahead once more.
    std::vector<uint8_t> req5 = makeMbx(EC_MBXT_COE, 5, coeEchoBody(0x23, kIndex, kSub));
    rsp = makeMbx(EC_MBXT_COE, 4, coeEchoBody(0x60, kIndex, kSub));
    hdr.cnt = 4;
    expected = 4; cur = 0; mbx_cnt = 0;
    EXPECT_TRUE(upload_.adoptCounterOnEcho(
        master_, kAdp, req5.data(), rsp.data(), kMbxLen, hdr,
        &mbx_cnt, cur, expected, kIndex, kSub, "init"));
    EXPECT_EQ(cur, 6);                       // 5 == last_req → skip to 6
}

TEST_F(SdoTxnBaseTest, AdoptCounterOnEchoRejects) {
    std::vector<uint8_t> req = makeMbx(EC_MBXT_COE, 2, coeEchoBody(0x23, kIndex, kSub));
    std::vector<uint8_t> rsp = makeMbx(EC_MBXT_COE, 5, coeEchoBody(0x60, kIndex, kSub));
    MbxResponseHeader hdr{};
    hdr.type = EC_MBXT_COE; hdr.cnt = 5;
    hdr.len = static_cast<uint16_t>(rsp.size() - sizeof(MbxHeader));
    uint8_t mbx_cnt = 0, cur = 0, expected = 0;

    // Wrong frame type.
    hdr.type = EC_MBXT_FOE;
    EXPECT_FALSE(upload_.adoptCounterOnEcho(
        master_, kAdp, req.data(), rsp.data(), kMbxLen, hdr,
        &mbx_cnt, cur, expected, kIndex, kSub, "x"));
    hdr.type = EC_MBXT_COE;

    // Non-echoing command byte.
    auto bad = makeMbx(EC_MBXT_COE, 5, coeEchoBody(0x01, kIndex, kSub));
    EXPECT_FALSE(upload_.adoptCounterOnEcho(
        master_, kAdp, req.data(), bad.data(), kMbxLen, hdr,
        &mbx_cnt, cur, expected, kIndex, kSub, "x"));

    // Echo mismatches the request's index/sub.
    bad = makeMbx(EC_MBXT_COE, 5, coeEchoBody(0x60, 0x9999, kSub));
    EXPECT_FALSE(upload_.adoptCounterOnEcho(
        master_, kAdp, req.data(), bad.data(), kMbxLen, hdr,
        &mbx_cnt, cur, expected, kIndex, kSub, "x"));

    // Truncated body.
    hdr.len = 2;
    EXPECT_FALSE(upload_.adoptCounterOnEcho(
        master_, kAdp, req.data(), rsp.data(), kMbxLen, hdr,
        &mbx_cnt, cur, expected, kIndex, kSub, "x"));
}

// ============================================================================
// pollSm1AndRead — scripted register reads
// ============================================================================

TEST_F(SdoTxnBaseTest, PollSm1AndReadOutcomes) {
    // Script: SM1 status says full, mailbox read returns a CoE frame.
    auto rsp = makeMbx(EC_MBXT_COE, 3, coeEchoBody(0x60, kIndex, kSub));
    master_.setAprdTestCallback(
        [&](uint16_t, uint16_t ado, void* out, uint16_t len,
            unsigned int) {
            if (ado == sm_status_address(0)) {
                *static_cast<uint8_t*>(out) = 0x00;   // SM0 not full
                return true;
            }
            if (ado == sm_status_address(1)) {
                *static_cast<uint8_t*>(out) = EC_SM_STATUS_MBXFULL;
                return true;
            }
            if (ado == kMbxRdAddr) {
                std::memcpy(out, rsp.data(),
                            std::min<size_t>(len, rsp.size()));
                return true;
            }
            *static_cast<uint8_t*>(out) = 0x00;
            return true;
        });

    uint8_t mbxbuf[kMbxLen] = {};
    MbxResponseHeader hdr{};
    EXPECT_EQ(upload_.pollSm1AndRead(master_, kAdp, kMbxRdAddr, kMbxLen,
                                     mbxbuf, 0, hdr),
              MbxPollOutcome::GotData);
    EXPECT_EQ(hdr.type, EC_MBXT_COE);
    EXPECT_EQ(hdr.cnt, 3);
    EXPECT_EQ(hdr.len, rsp.size() - sizeof(MbxHeader));
}

TEST_F(SdoTxnBaseTest, PollSm1AndReadFailureArms) {
    uint8_t mbxbuf[kMbxLen] = {};
    MbxResponseHeader hdr{};

    // SM1 status read fails → Sm1Empty (retry).
    master_.setAprdTestCallback(
        [](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
            return false;
        });
    EXPECT_EQ(upload_.pollSm1AndRead(master_, kAdp, kMbxRdAddr, kMbxLen,
                                     mbxbuf, 0, hdr),
              MbxPollOutcome::Sm1Empty);

    // SM1 not full → Sm1Empty.
    master_.setAprdTestCallback(
        [](uint16_t, uint16_t ado, void* out, uint16_t, unsigned int) {
            if (ado == sm_status_address(1)) {
                *static_cast<uint8_t*>(out) = 0x00;
                return true;
            }
            return false;
        });
    EXPECT_EQ(upload_.pollSm1AndRead(master_, kAdp, kMbxRdAddr, kMbxLen,
                                     mbxbuf, 0, hdr),
              MbxPollOutcome::Sm1Empty);

    // Full but data read fails → ReadFailed.
    master_.setAprdTestCallback(
        [](uint16_t, uint16_t ado, void* out, uint16_t, unsigned int) {
            if (ado == sm_status_address(1)) {
                *static_cast<uint8_t*>(out) = EC_SM_STATUS_MBXFULL;
                return true;
            }
            return false;
        });
    EXPECT_EQ(upload_.pollSm1AndRead(master_, kAdp, kMbxRdAddr, kMbxLen,
                                     mbxbuf, 0, hdr),
              MbxPollOutcome::ReadFailed);
}

TEST_F(SdoTxnBaseTest, PollSm1AndReadClampsOverclaimedLen) {
    // Slave claims a body longer than the SM buffer — hdr.len clamps.
    auto rsp = makeMbx(EC_MBXT_COE, 1, std::vector<uint8_t>(0xFF, 0));
    master_.setAprdTestCallback(
        [&](uint16_t, uint16_t ado, void* out, uint16_t len,
            unsigned int) {
            if (ado == sm_status_address(1)) {
                *static_cast<uint8_t*>(out) = EC_SM_STATUS_MBXFULL;
                return true;
            }
            std::memcpy(out, rsp.data(), std::min<size_t>(len, rsp.size()));
            return true;
        });

    uint8_t mbxbuf[32] = {};
    MbxResponseHeader hdr{};
    EXPECT_EQ(upload_.pollSm1AndRead(master_, kAdp, kMbxRdAddr,
                                     sizeof(mbxbuf), mbxbuf, 0, hdr),
              MbxPollOutcome::GotData);
    EXPECT_EQ(hdr.len, sizeof(mbxbuf) - sizeof(MbxHeader));
}

// ============================================================================
// sendAndWait — APWR + SM1-full wait
// ============================================================================

TEST_F(SdoTxnBaseTest, SendAndWaitSuccessAndFailure) {
    uint8_t mbxbuf[kMbxLen] = {};
    master_.setApwrTestCallback(
        [](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return true;
        });
    master_.setAprdTestCallback(
        [](uint16_t, uint16_t ado, void* out, uint16_t, unsigned int) {
            if (ado == sm_status_address(0)) {
                *static_cast<uint8_t*>(out) = 0x00;   // SM0 not full
                return true;
            }
            if (ado == sm_status_address(1)) {
                *static_cast<uint8_t*>(out) = EC_SM_STATUS_MBXFULL;
                return true;
            }
            *static_cast<uint8_t*>(out) = 0x00;
            return true;
        });
    EXPECT_TRUE(upload_.sendAndWait(master_, kAdp, kMbxWrAddr, kMbxLen,
                                    kMbxRdAddr, kMbxLen, mbxbuf,
                                    10, 0, 10, "test"));

    // APWR fails → false.
    master_.setApwrTestCallback(
        [](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return false;
        });
    EXPECT_FALSE(upload_.sendAndWait(master_, kAdp, kMbxWrAddr, kMbxLen,
                                     kMbxRdAddr, kMbxLen, mbxbuf,
                                     10, 0, 10, "test"));

    // APWR ok but SM1 never fills → false + diagnostics dump.
    master_.setApwrTestCallback(
        [](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return true;
        });
    master_.setAprdTestCallback(
        [](uint16_t, uint16_t ado, void* out, uint16_t, unsigned int) {
            if (ado == sm_status_address(0)) {
                *static_cast<uint8_t*>(out) = 0x00;   // SM0 not full
                return true;
            }
            if (ado == sm_status_address(1)) {
                *static_cast<uint8_t*>(out) = 0x00;   // SM1 never fills
                return true;
            }
            *static_cast<uint8_t*>(out) = 0x00;
            return true;
        });
    EXPECT_FALSE(upload_.sendAndWait(master_, kAdp, kMbxWrAddr, kMbxLen,
                                     kMbxRdAddr, kMbxLen, mbxbuf,
                                     10, 0, 2, "test"));
}

// ============================================================================
// resyncMailboxCounter — counter-space probe
// ============================================================================

TEST_F(SdoTxnBaseTest, ResyncRecoversOnEchoingResponse) {
    auto req = makeMbx(EC_MBXT_COE, 1, coeEchoBody(0x23, kIndex, kSub));
    auto rsp = makeMbx(EC_MBXT_COE, 4, coeEchoBody(0x60, kIndex, kSub));

    master_.setApwrTestCallback(
        [](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return true;
        });
    master_.setAprdTestCallback(
        [&](uint16_t, uint16_t ado, void* out, uint16_t len,
            unsigned int) {
            if (ado == sm_status_address(0)) {
                *static_cast<uint8_t*>(out) = 0x00;   // SM0 not full
                return true;
            }
            if (ado == sm_status_address(1)) {
                *static_cast<uint8_t*>(out) = EC_SM_STATUS_MBXFULL;
                return true;
            }
            if (ado == kMbxRdAddr) {
                std::memcpy(out, rsp.data(),
                            std::min<size_t>(len, rsp.size()));
                return true;
            }
            *static_cast<uint8_t*>(out) = 0x00;
            return true;
        });

    std::vector<uint8_t> reqbuf = req;
    uint8_t rspbuf[kMbxLen] = {};
    uint8_t mbx_cnt = 1, cur = 0, expected = 0;
    MbxResponseHeader out_hdr{};
    EXPECT_EQ(upload_.resyncMailboxCounter(
                  master_, kAdp, kMbxWrAddr, kMbxLen, kMbxRdAddr, kMbxLen,
                  reqbuf.data(), rspbuf, &mbx_cnt, cur, expected,
                  kIndex, kSub, /*verifyIndexEcho=*/true, 0, "init",
                  out_hdr),
              MbxResyncResult::Recovered);
    EXPECT_EQ(out_hdr.cnt, 4);
    EXPECT_EQ(expected, 4);
    EXPECT_EQ(cur, 5);                       // nextMbxCnt(4), != cand
    EXPECT_EQ(mbx_cnt, 5);
}

TEST_F(SdoTxnBaseTest, ResyncFailsWhenNothingAnswers) {
    auto req = makeMbx(EC_MBXT_COE, 1, coeEchoBody(0x23, kIndex, kSub));
    master_.setApwrTestCallback(
        [](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return true;
        });
    master_.setAprdTestCallback(
        [](uint16_t, uint16_t ado, void* out, uint16_t, unsigned int) {
            if (ado == sm_status_address(0)) {
                *static_cast<uint8_t*>(out) = 0x00;   // SM0 not full
                return true;
            }
            if (ado == sm_status_address(1)) {
                *static_cast<uint8_t*>(out) = 0x00;   // never full
                return true;
            }
            *static_cast<uint8_t*>(out) = 0x00;
            return true;
        });

    std::vector<uint8_t> reqbuf = req;
    uint8_t rspbuf[kMbxLen] = {};
    uint8_t mbx_cnt = 1, cur = 0, expected = 0;
    MbxResponseHeader out_hdr{};
    EXPECT_EQ(upload_.resyncMailboxCounter(
                  master_, kAdp, kMbxWrAddr, kMbxLen, kMbxRdAddr, kMbxLen,
                  reqbuf.data(), rspbuf, &mbx_cnt, cur, expected,
                  kIndex, kSub, true, 0, "init", out_hdr),
              MbxResyncResult::Failed);
}

TEST_F(SdoTxnBaseTest, ResyncFailsOnApwrError) {
    auto req = makeMbx(EC_MBXT_COE, 1, coeEchoBody(0x23, kIndex, kSub));
    master_.setApwrTestCallback(
        [](uint16_t, uint16_t, const void*, uint16_t, unsigned int) {
            return false;
        });
    master_.setAprdTestCallback(
        [](uint16_t, uint16_t, void*, uint16_t, unsigned int) {
            return false;
        });

    std::vector<uint8_t> reqbuf = req;
    uint8_t rspbuf[kMbxLen] = {};
    uint8_t mbx_cnt = 1, cur = 0, expected = 0;
    MbxResponseHeader out_hdr{};
    EXPECT_EQ(upload_.resyncMailboxCounter(
                  master_, kAdp, kMbxWrAddr, kMbxLen, kMbxRdAddr, kMbxLen,
                  reqbuf.data(), rspbuf, &mbx_cnt, cur, expected,
                  kIndex, kSub, true, 0, "init", out_hdr),
              MbxResyncResult::Failed);
}
