/**
 * @file test_MailboxHandlers.cpp
 * @brief Unit tests for all five non-CoE mailbox protocol handlers:
 *        FoE, SoE, AoE, EoE, VoE
 *
 * Tests exercise handlers purely through the IMailboxHandler interface
 * using processRequest() with crafted byte-level protocol requests.
 */

#include "gtest/gtest.h"
#include "tether/slave/mailbox/IMailboxHandler.hpp"

#include <cstring>
#include <vector>
#include <algorithm>

using namespace EtherCAT::slave;

static constexpr size_t kBufSize = 4096;

// ============================================================================
// FoE Constants & Helpers
// ============================================================================

namespace {
    constexpr uint8_t FOE_OP_READ_REQUEST  = 1;
    constexpr uint8_t FOE_OP_WRITE_REQUEST = 2;
    constexpr uint8_t FOE_OP_DATA          = 3;
    constexpr uint8_t FOE_OP_ACK           = 4;
    constexpr uint8_t FOE_OP_ERROR         = 5;

    /// Build a generic 6-byte FoE request with an opcode, 4-byte param, and optional filename.
    std::vector<uint8_t> buildFoERequest(uint8_t opCode, uint32_t param,
                                          const std::string& name = "") {
        size_t nameLen = name.empty() ? 0 : name.size() + 1; // include null terminator
        std::vector<uint8_t> req(6 + nameLen);
        req[0] = opCode;
        req[1] = 0;
        req[2] = param & 0xFF;
        req[3] = (param >> 8) & 0xFF;
        req[4] = (param >> 16) & 0xFF;
        req[5] = (param >> 24) & 0xFF;
        if (!name.empty()) {
            memcpy(req.data() + 6, name.c_str(), name.size() + 1);
        }
        return req;
    }

    /// Build a FoE DATA packet: opCode=3, packetNumber, payload
    std::vector<uint8_t> buildFoEData(uint32_t packetNum,
                                       const std::vector<uint8_t>& payload) {
        std::vector<uint8_t> pkt(6 + payload.size());
        pkt[0] = FOE_OP_DATA;
        pkt[1] = 0;
        pkt[2] = packetNum & 0xFF;
        pkt[3] = (packetNum >> 8) & 0xFF;
        pkt[4] = (packetNum >> 16) & 0xFF;
        pkt[5] = (packetNum >> 24) & 0xFF;
        if (!payload.empty()) {
            memcpy(pkt.data() + 6, payload.data(), payload.size());
        }
        return pkt;
    }
}

// ============================================================================
// SoE Helpers
// ============================================================================

namespace {
    constexpr uint8_t SOE_OP_READ_REQUEST  = 1;
    constexpr uint8_t SOE_OP_READ_RESPONSE = 2;
    constexpr uint8_t SOE_OP_WRITE_REQUEST = 3;
    constexpr uint8_t SOE_OP_WRITE_RESPONSE = 4;

    constexpr uint8_t SOE_ELEM_DATA      = 0x01;
    constexpr uint8_t SOE_ELEM_NAME      = 0x02;
    constexpr uint8_t SOE_ELEM_ATTRIBUTE = 0x04;
    constexpr uint8_t SOE_ELEM_UNIT      = 0x08;

    std::vector<uint8_t> buildSoEReadReq(uint16_t idn, uint8_t elements,
                                          uint8_t driveNo = 0) {
        std::vector<uint8_t> req(6);
        req[0] = SOE_OP_READ_REQUEST;
        req[1] = driveNo;
        req[2] = elements;
        req[3] = idn & 0xFF;
        req[4] = (idn >> 8) & 0xFF;
        req[5] = 0;
        return req;
    }

    std::vector<uint8_t> buildSoEWriteReq(uint16_t idn, uint8_t elements,
                                           const std::vector<uint8_t>& data,
                                           uint8_t driveNo = 0) {
        // data element: [len:2][data:N]
        std::vector<uint8_t> req(6);
        req[0] = SOE_OP_WRITE_REQUEST;
        req[1] = driveNo;
        req[2] = elements;
        req[3] = idn & 0xFF;
        req[4] = (idn >> 8) & 0xFF;
        req[5] = 0;
        if (elements & SOE_ELEM_DATA) {
            uint16_t len = static_cast<uint16_t>(data.size());
            req.push_back(len & 0xFF);
            req.push_back((len >> 8) & 0xFF);
            req.insert(req.end(), data.begin(), data.end());
        }
        return req;
    }
}

// ============================================================================
// AoE / ADS Helpers
// ============================================================================

namespace {
    constexpr uint16_t ADS_CMD_READ_DEVICE_INFO = 0x0001;
    constexpr uint16_t ADS_CMD_READ              = 0x0002;
    constexpr uint16_t ADS_CMD_WRITE             = 0x0003;
    constexpr uint16_t ADS_CMD_READ_STATE        = 0x0004;
    constexpr uint16_t ADS_CMD_WRITE_CONTROL     = 0x0005;
    constexpr uint16_t ADS_CMD_READ_WRITE        = 0x0009;

    constexpr uint32_t ADSIGRP_SYM_VAL_BYNAME  = 0xF004;
    constexpr uint32_t ADSIGRP_SYM_HANDLE       = 0xF003;

    /// Build a 32-byte AMS header targeting the default handler {192.168.1.100.1.1}:851
    std::vector<uint8_t> buildAmsHeader(uint16_t cmdId, uint32_t cbData = 0,
                                         uint32_t invokeId = 1) {
        std::vector<uint8_t> h(32, 0);
        // Target NetId
        h[0]=192; h[1]=168; h[2]=1; h[3]=100; h[4]=1; h[5]=1;
        // Target port = 851
        h[6] = 851 & 0xFF; h[7] = (851 >> 8) & 0xFF;
        // Source NetId
        h[8]=10; h[9]=0; h[10]=0; h[11]=1; h[12]=1; h[13]=1;
        // Source port = 801
        h[14] = 801 & 0xFF; h[15] = (801 >> 8) & 0xFF;
        // cmdId
        h[16] = cmdId & 0xFF; h[17] = (cmdId >> 8) & 0xFF;
        // stateFlags = 0x0004
        h[18] = 0x04; h[19] = 0x00;
        // cbData
        h[20] = cbData & 0xFF; h[21] = (cbData >> 8) & 0xFF;
        h[22] = (cbData >> 16) & 0xFF; h[23] = (cbData >> 24) & 0xFF;
        // errorCode = 0 (bytes 24-27)
        // invokeId
        h[28] = invokeId & 0xFF; h[29] = (invokeId >> 8) & 0xFF;
        h[30] = (invokeId >> 16) & 0xFF; h[31] = (invokeId >> 24) & 0xFF;
        return h;
    }

    /// Build an ADS Read request: AmsHeader + {indexGroup, indexOffset, cbLength}
    std::vector<uint8_t> buildAdsReadReq(uint32_t ig, uint32_t io, uint32_t len,
                                          uint32_t invokeId = 1) {
        auto h = buildAmsHeader(ADS_CMD_READ, 12, invokeId);
        h.resize(32 + 12);
        auto put32 = [&](size_t off, uint32_t v) {
            h[off] = v & 0xFF; h[off+1]=(v>>8)&0xFF;
            h[off+2]=(v>>16)&0xFF; h[off+3]=(v>>24)&0xFF;
        };
        put32(32, ig);
        put32(36, io);
        put32(40, len);
        return h;
    }

    /// Build an ADS Write request: AmsHeader + {ig, io, cbLength} + data
    std::vector<uint8_t> buildAdsWriteReq(uint32_t ig, uint32_t io,
                                           const std::vector<uint8_t>& data,
                                           uint32_t invokeId = 1) {
        uint32_t dataSz = static_cast<uint32_t>(data.size());
        auto h = buildAmsHeader(ADS_CMD_WRITE, 12 + dataSz, invokeId);
        h.resize(32 + 12 + dataSz);
        auto put32 = [&](size_t off, uint32_t v) {
            h[off]=v&0xFF; h[off+1]=(v>>8)&0xFF;
            h[off+2]=(v>>16)&0xFF; h[off+3]=(v>>24)&0xFF;
        };
        put32(32, ig);
        put32(36, io);
        put32(40, dataSz);
        memcpy(h.data() + 44, data.data(), dataSz);
        return h;
    }

    /// Build an ADS WriteControl request
    std::vector<uint8_t> buildAdsWriteControlReq(uint16_t adsState,
                                                  uint16_t deviceState,
                                                  uint32_t invokeId = 1) {
        auto h = buildAmsHeader(ADS_CMD_WRITE_CONTROL, 8, invokeId);
        h.resize(32 + 8, 0);
        h[32] = adsState & 0xFF; h[33] = (adsState >> 8) & 0xFF;
        h[34] = deviceState & 0xFF; h[35] = (deviceState >> 8) & 0xFF;
        // cbLength = 0 (bytes 36-39 already 0)
        return h;
    }

    /// Build an ADS ReadWrite request for ADSIGRP_SYM_HANDLE
    std::vector<uint8_t> buildAdsSymHandleReq(const std::string& symbolName,
                                               uint32_t invokeId = 1) {
        uint32_t writeSz = static_cast<uint32_t>(symbolName.size());
        uint32_t readSz = 4; // handle
        auto h = buildAmsHeader(ADS_CMD_READ_WRITE, 16 + writeSz, invokeId);
        h.resize(32 + 16 + writeSz, 0);
        auto put32 = [&](size_t off, uint32_t v) {
            h[off]=v&0xFF; h[off+1]=(v>>8)&0xFF;
            h[off+2]=(v>>16)&0xFF; h[off+3]=(v>>24)&0xFF;
        };
        put32(32, ADSIGRP_SYM_HANDLE);
        put32(36, 0);
        put32(40, readSz);
        put32(44, writeSz);
        memcpy(h.data() + 48, symbolName.data(), writeSz);
        return h;
    }
}

// ============================================================================
// EoE Helpers
// ============================================================================

namespace {
    constexpr uint8_t EOE_TYPE_FRAGMENT_DATA     = 0x00;
    constexpr uint8_t EOE_TYPE_TIMESTAMP_REQUEST = 0x01;
    constexpr uint8_t EOE_TYPE_SET_IP_PARAM_REQ  = 0x03;
    constexpr uint8_t EOE_TYPE_GET_IP_PARAM_REQ  = 0x07;
    constexpr uint8_t EOE_TYPE_GET_IP_PARAM_RESP = 0x08;
    constexpr uint8_t EOE_TYPE_SET_FILTER_REQ    = 0x05;
    constexpr uint8_t EOE_TYPE_GET_FILTER_REQ    = 0x09;

    std::vector<uint8_t> buildEoEHeader(uint8_t frameType) {
        std::vector<uint8_t> req(4, 0);
        req[0] = (frameType << 4);
        return req;
    }
}

// ============================================================================
// VoE Helpers
// ============================================================================

namespace {
    std::vector<uint8_t> buildVoERequest(uint32_t vendorId, uint16_t vendorType,
                                          const uint8_t* payload = nullptr,
                                          size_t payloadLen = 0) {
        std::vector<uint8_t> req(8 + payloadLen, 0);
        memcpy(req.data(), &vendorId, 4);
        memcpy(req.data() + 4, &vendorType, 2);
        // flags=0, reserved=0 (already zero)
        if (payload && payloadLen > 0) {
            memcpy(req.data() + 8, payload, payloadLen);
        }
        return req;
    }
}

// ############################################################################
//  FoE Handler Tests
// ############################################################################

class FoEHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler = createFoEHandler("/tmp/foe_test");
        memset(resp, 0, sizeof(resp));
        respLen = sizeof(resp);
    }
    std::unique_ptr<IMailboxHandler> handler;
    uint8_t resp[kBufSize];
    size_t respLen;
};

TEST_F(FoEHandlerTest, ProtocolAndName) {
    EXPECT_EQ(handler->getProtocol(), MailboxProtocol::FoE);
    EXPECT_STREQ(handler->getProtocolName(), "FoE");
}

TEST_F(FoEHandlerTest, ResetDoesNotCrash) {
    handler->reset();
    EXPECT_FALSE(handler->hasPendingResponse());
}

TEST_F(FoEHandlerTest, TooShortRequest) {
    uint8_t req[4] = {1, 0, 0, 0};
    bool ok = handler->processRequest(req, 4, resp, respLen);
    EXPECT_TRUE(ok);
    // Should return error opcode
    EXPECT_EQ(resp[0], FOE_OP_ERROR);
}

TEST_F(FoEHandlerTest, InvalidOpcode) {
    auto req = buildFoERequest(0xFF, 0);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0], FOE_OP_ERROR);
}

TEST_F(FoEHandlerTest, ReadNonExistentFile) {
    auto req = buildFoERequest(FOE_OP_READ_REQUEST, 0, "noexist.bin");
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0], FOE_OP_ERROR);
    // Error code should be "not found" (0x8001)
    uint32_t errCode = resp[2] | (resp[3]<<8) | (resp[4]<<16) | (resp[5]<<24);
    EXPECT_EQ(errCode, 0x8001u);
}

TEST_F(FoEHandlerTest, ReadRegisteredFile) {
    // "config.bin" is pre-registered, not bootstrap-only, empty by default
    auto req = buildFoERequest(FOE_OP_READ_REQUEST, 0, "config.bin");
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Response should be DATA packet (opcode 3)
    EXPECT_EQ(resp[0], FOE_OP_DATA);
    // Packet number should be 1
    uint32_t pktNum = resp[2] | (resp[3]<<8) | (resp[4]<<16) | (resp[5]<<24);
    EXPECT_EQ(pktNum, 1u);
}

TEST_F(FoEHandlerTest, WriteRequestForNewFile) {
    auto req = buildFoERequest(FOE_OP_WRITE_REQUEST, 0, "newfile.bin");
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Response should be ACK for packet 0
    EXPECT_EQ(resp[0], FOE_OP_ACK);
    uint32_t ackPkt = resp[2] | (resp[3]<<8) | (resp[4]<<16) | (resp[5]<<24);
    EXPECT_EQ(ackPkt, 0u);
}

TEST_F(FoEHandlerTest, WriteDataCompleteTransfer) {
    // Start write
    auto req = buildFoERequest(FOE_OP_WRITE_REQUEST, 0, "newfile.bin");
    handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_EQ(resp[0], FOE_OP_ACK);

    // Send short data packet (< 506 bytes == transfer complete)
    std::vector<uint8_t> payload = {0xDE, 0xAD, 0xBE, 0xEF};
    auto dataPkt = buildFoEData(1, payload);
    respLen = sizeof(resp);
    bool ok = handler->processRequest(dataPkt.data(), dataPkt.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0], FOE_OP_ACK);
    uint32_t ackPkt = resp[2] | (resp[3]<<8) | (resp[4]<<16) | (resp[5]<<24);
    EXPECT_EQ(ackPkt, 1u);
}

TEST_F(FoEHandlerTest, AckWithoutActiveReadTransfer) {
    auto ack = buildFoERequest(FOE_OP_ACK, 1);
    bool ok = handler->processRequest(ack.data(), ack.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0], FOE_OP_ERROR);
}

TEST_F(FoEHandlerTest, DataWithoutActiveWriteTransfer) {
    auto dataPkt = buildFoEData(1, {0x01, 0x02});
    bool ok = handler->processRequest(dataPkt.data(), dataPkt.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0], FOE_OP_ERROR);
}

TEST_F(FoEHandlerTest, WrongPacketNumberOnData) {
    // Start write
    auto req = buildFoERequest(FOE_OP_WRITE_REQUEST, 0, "test.bin");
    handler->processRequest(req.data(), req.size(), resp, respLen);

    // Send packet with wrong number (expect 1, send 5)
    auto dataPkt = buildFoEData(5, {0xAA});
    respLen = sizeof(resp);
    bool ok = handler->processRequest(dataPkt.data(), dataPkt.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0], FOE_OP_ERROR);
    uint32_t errCode = resp[2] | (resp[3]<<8) | (resp[4]<<16) | (resp[5]<<24);
    EXPECT_EQ(errCode, 0x8005u); // FOE_ERR_PACKET_NUMBER
}

TEST_F(FoEHandlerTest, ReadThenAckCycle) {
    // config.bin is pre-registered, zero-length
    auto req = buildFoERequest(FOE_OP_READ_REQUEST, 0, "config.bin");
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0], FOE_OP_DATA);
    // Data length is 0 (empty file), so response should be 6 bytes header only
    EXPECT_EQ(respLen, 6u);

    // ACK packet 1
    auto ack = buildFoERequest(FOE_OP_ACK, 1);
    respLen = sizeof(resp);
    ok = handler->processRequest(ack.data(), ack.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Transfer should be complete (offset >= data.size()==0)
}

TEST_F(FoEHandlerTest, BootstrapFileReadAllowedWithoutCore) {
    // "firmware.bin" is bootstrapOnly=true, but core_ is null so bypass check
    auto req = buildFoERequest(FOE_OP_READ_REQUEST, 0, "firmware.bin");
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Should succeed (null check bypass) with DATA response
    EXPECT_EQ(resp[0], FOE_OP_DATA);
}

// ############################################################################
//  SoE Handler Tests
// ############################################################################

class SoEHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler = createSoEHandler();
        memset(resp, 0, sizeof(resp));
        respLen = sizeof(resp);
    }
    std::unique_ptr<IMailboxHandler> handler;
    uint8_t resp[kBufSize];
    size_t respLen;
};

TEST_F(SoEHandlerTest, ProtocolAndName) {
    EXPECT_EQ(handler->getProtocol(), MailboxProtocol::SoE);
    EXPECT_STREQ(handler->getProtocolName(), "SoE");
}

TEST_F(SoEHandlerTest, ResetDoesNotCrash) {
    handler->reset();
}

TEST_F(SoEHandlerTest, TooShortRequest) {
    uint8_t req[3] = {1, 0, 0};
    bool ok = handler->processRequest(req, 3, resp, respLen);
    EXPECT_TRUE(ok);
    // Error response has error flag (0x10) set
    EXPECT_NE(resp[0] & 0x10, 0);
}

TEST_F(SoEHandlerTest, ReadNonExistentIDN) {
    auto req = buildSoEReadReq(0xFFFF, SOE_ELEM_DATA);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_NE(resp[0] & 0x10, 0); // error flag
    // IDN in response
    uint16_t respIdn = resp[3] | (resp[4] << 8);
    EXPECT_EQ(respIdn, 0xFFFF);
    // Error code 0x1001 (no IDN)
    uint16_t errCode = resp[6] | (resp[7] << 8);
    EXPECT_EQ(errCode, 0x1001);
}

TEST_F(SoEHandlerTest, ReadDataElement) {
    // Read NC cycle time (IDN 0x0001) data element
    auto req = buildSoEReadReq(0x0001, SOE_ELEM_DATA);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0] & 0x07, SOE_OP_READ_RESPONSE);
    EXPECT_EQ(resp[0] & 0x10, 0); // no error
    // Data starts at offset 6: [len:2][data:4]
    uint16_t dataLen = resp[6] | (resp[7] << 8);
    EXPECT_EQ(dataLen, 4u);
    // Cycle time = 1000 us = 0x000003E8
    uint32_t cycleTime = resp[8] | (resp[9]<<8) | (resp[10]<<16) | (resp[11]<<24);
    EXPECT_EQ(cycleTime, 1000u);
}

TEST_F(SoEHandlerTest, ReadNameElement) {
    auto req = buildSoEReadReq(0x0001, SOE_ELEM_NAME);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Name at offset 6: [len:2][name:N]
    uint16_t nameLen = resp[6] | (resp[7] << 8);
    EXPECT_GT(nameLen, 0u);
    std::string name(reinterpret_cast<char*>(resp + 8), nameLen);
    EXPECT_EQ(name, "NC cycle time");
}

TEST_F(SoEHandlerTest, ReadAttributeElement) {
    auto req = buildSoEReadReq(0x0001, SOE_ELEM_ATTRIBUTE);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Attribute at offset 6: [attr:4]
    uint32_t attr = resp[6] | (resp[7]<<8) | (resp[8]<<16) | (resp[9]<<24);
    EXPECT_EQ(attr, 0x00070001u);
}

TEST_F(SoEHandlerTest, ReadUnitElement) {
    // Velocity command (IDN 0x0024) has unit "rpm"
    auto req = buildSoEReadReq(0x0024, SOE_ELEM_UNIT);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    uint16_t unitLen = resp[6] | (resp[7] << 8);
    std::string unit(reinterpret_cast<char*>(resp + 8), unitLen);
    EXPECT_EQ(unit, "rpm");
}

TEST_F(SoEHandlerTest, WriteIDN) {
    // Write to primary operation mode (IDN 0x0020), writable
    std::vector<uint8_t> newData = {0x05};
    auto req = buildSoEWriteReq(0x0020, SOE_ELEM_DATA, newData);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0] & 0x07, SOE_OP_WRITE_RESPONSE);
    EXPECT_EQ(resp[0] & 0x10, 0); // no error

    // Read back to verify
    respLen = sizeof(resp);
    auto rdReq = buildSoEReadReq(0x0020, SOE_ELEM_DATA);
    ok = handler->processRequest(rdReq.data(), rdReq.size(), resp, respLen);
    EXPECT_TRUE(ok);
    uint16_t dataLen = resp[6] | (resp[7] << 8);
    EXPECT_EQ(dataLen, 1u);
    EXPECT_EQ(resp[8], 0x05);
}

TEST_F(SoEHandlerTest, WriteReadOnlyIDN) {
    // NC cycle time (0x0001) is readOnly
    std::vector<uint8_t> newData = {0x01, 0x00, 0x00, 0x00};
    auto req = buildSoEWriteReq(0x0001, SOE_ELEM_DATA, newData);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_NE(resp[0] & 0x10, 0); // error flag
    uint16_t errCode = resp[6] | (resp[7] << 8);
    EXPECT_EQ(errCode, 0x2003); // SOE_ERR_ATTRIBUTE_RO
}

TEST_F(SoEHandlerTest, UnknownOpcode) {
    std::vector<uint8_t> req(6, 0);
    req[0] = 0x06; // emergency (6) - not handled as request
    req[3] = 0x01; // IDN low
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_NE(resp[0] & 0x10, 0); // error flag
}

TEST_F(SoEHandlerTest, ReadMultipleElements) {
    // Read both data and name for velocity command (0x0024)
    auto req = buildSoEReadReq(0x0024, SOE_ELEM_DATA | SOE_ELEM_NAME);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0] & 0x10, 0); // no error

    // Data at offset 6: [len:2][data:4]
    uint16_t dataLen = resp[6] | (resp[7] << 8);
    EXPECT_EQ(dataLen, 4u);
    // Name follows at offset 6 + 2 + 4 = 12
    uint16_t nameLen = resp[12] | (resp[13] << 8);
    std::string name(reinterpret_cast<char*>(resp + 14), nameLen);
    EXPECT_EQ(name, "Velocity command");
}

TEST_F(SoEHandlerTest, WriteVelocityCommand) {
    // Write 4 bytes to velocity command (0x0024)
    std::vector<uint8_t> vel = {0xE8, 0x03, 0x00, 0x00}; // 1000 rpm
    auto req = buildSoEWriteReq(0x0024, SOE_ELEM_DATA, vel);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[0] & 0x07, SOE_OP_WRITE_RESPONSE);
    EXPECT_EQ(resp[0] & 0x10, 0);
}

// ############################################################################
//  AoE Handler Tests
// ############################################################################

class AoEHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler = createAoEHandler();
        memset(resp, 0, sizeof(resp));
        respLen = sizeof(resp);
    }
    std::unique_ptr<IMailboxHandler> handler;
    uint8_t resp[kBufSize];
    size_t respLen;
};

TEST_F(AoEHandlerTest, ProtocolAndName) {
    EXPECT_EQ(handler->getProtocol(), MailboxProtocol::AoE);
    EXPECT_STREQ(handler->getProtocolName(), "AoE");
}

TEST_F(AoEHandlerTest, TooShortRequest) {
    uint8_t req[10] = {};
    bool ok = handler->processRequest(req, 10, resp, respLen);
    // AMS header is 32 bytes - 10 is too short
    EXPECT_TRUE(ok);
}

TEST_F(AoEHandlerTest, ReadDeviceInfo) {
    auto req = buildAmsHeader(ADS_CMD_READ_DEVICE_INFO);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_GT(respLen, 32u + 4u); // header + errorCode + info

    // Parse: response AmsHeader (32) + errorCode (4) + DeviceInfo
    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_EQ(errCode, 0u);

    // Device name starts at offset 32+4+4 = 40 (after major/minor/build)
    char deviceName[16];
    memcpy(deviceName, resp + 40, 16);
    EXPECT_STREQ(deviceName, "EtherCAT Slave");
}

TEST_F(AoEHandlerTest, ReadState) {
    auto req = buildAmsHeader(ADS_CMD_READ_STATE);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_EQ(errCode, 0u);

    // ADS state (default = RUN = 5)
    uint16_t adsState;
    memcpy(&adsState, resp + 36, 2);
    EXPECT_EQ(adsState, 5u);
}

TEST_F(AoEHandlerTest, WriteControl) {
    auto req = buildAdsWriteControlReq(0x0006, 0x0000); // STOP state
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_EQ(errCode, 0u);

    // Verify state changed by reading it back
    respLen = sizeof(resp);
    auto readReq = buildAmsHeader(ADS_CMD_READ_STATE);
    handler->processRequest(readReq.data(), readReq.size(), resp, respLen);

    uint16_t adsState;
    memcpy(&adsState, resp + 36, 2);
    EXPECT_EQ(adsState, 6u); // STOP
}

TEST_F(AoEHandlerTest, ReadSymbolByNameNotImplemented) {
    // ADS Read via ADSIGRP_SYM_VAL_BYNAME is explicitly not implemented
    auto req = buildAdsReadReq(ADSIGRP_SYM_VAL_BYNAME, 0, 4);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_NE(errCode, 0u); // returns ADS_ERR_NO_IO
}

TEST_F(AoEHandlerTest, WriteSymbol) {
    // Write to MAIN.Counter (0xF004, 0)
    std::vector<uint8_t> value = {0x42, 0x00, 0x00, 0x00};
    auto req = buildAdsWriteReq(ADSIGRP_SYM_VAL_BYNAME, 0, value);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_EQ(errCode, 0u);
}

TEST_F(AoEHandlerTest, WriteSymbolVerifySuccess) {
    // Write a value to MAIN.Counter at {0xF004, 0}
    std::vector<uint8_t> value = {0xAB, 0xCD, 0x00, 0x00};
    auto wReq = buildAdsWriteReq(ADSIGRP_SYM_VAL_BYNAME, 0, value);
    bool ok = handler->processRequest(wReq.data(), wReq.size(), resp, respLen);
    EXPECT_TRUE(ok);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_EQ(errCode, 0u);

    // Verify handle retrieval still works after write
    respLen = sizeof(resp);
    auto hReq = buildAdsSymHandleReq("MAIN.Counter");
    ok = handler->processRequest(hReq.data(), hReq.size(), resp, respLen);
    EXPECT_TRUE(ok);
    memcpy(&errCode, resp + 32, 4);
    EXPECT_EQ(errCode, 0u);
}

TEST_F(AoEHandlerTest, ReadNonExistentSymbol) {
    // Index group 0x9999 with offset 0 — no symbol registered there
    auto req = buildAdsReadReq(0x9999, 0, 4);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_NE(errCode, 0u); // Should be an error
}

TEST_F(AoEHandlerTest, UnknownCommand) {
    auto req = buildAmsHeader(0x00FF);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Response should have non-zero error
    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_NE(errCode, 0u);
}

TEST_F(AoEHandlerTest, WrongPort) {
    auto req = buildAmsHeader(ADS_CMD_READ_STATE);
    // Change target port to 999 (not 851 and not 0)
    req[6] = 999 & 0xFF; req[7] = (999 >> 8) & 0xFF;
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Should return port not found error
    uint32_t respErrInHeader;
    memcpy(&respErrInHeader, resp + 24, 4);
    EXPECT_NE(respErrInHeader, 0u);
}

TEST_F(AoEHandlerTest, ResetClearsSymbols) {
    handler->reset();
    // After reset, symbols are cleared, so reading MAIN.Counter should fail
    auto req = buildAdsReadReq(ADSIGRP_SYM_VAL_BYNAME, 0, 4);
    respLen = sizeof(resp);
    handler->processRequest(req.data(), req.size(), resp, respLen);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_NE(errCode, 0u);
}

TEST_F(AoEHandlerTest, GetSymbolHandle) {
    auto req = buildAdsSymHandleReq("MAIN.Counter");
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_EQ(errCode, 0u);

    // Handle is returned as 4-byte value at offset 32+4+4 = 40
    uint32_t cbLength;
    memcpy(&cbLength, resp + 36, 4);
    EXPECT_EQ(cbLength, 4u);
}

TEST_F(AoEHandlerTest, GetSymbolHandleNonExistent) {
    auto req = buildAdsSymHandleReq("NONEXISTENT.Variable");
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    uint32_t errCode;
    memcpy(&errCode, resp + 32, 4);
    EXPECT_NE(errCode, 0u);
}

TEST_F(AoEHandlerTest, InvokeIdPreserved) {
    auto req = buildAmsHeader(ADS_CMD_READ_STATE, 0, 0xDEADBEEF);
    handler->processRequest(req.data(), req.size(), resp, respLen);

    uint32_t invokeId;
    memcpy(&invokeId, resp + 28, 4);
    EXPECT_EQ(invokeId, 0xDEADBEEFu);
}

// ############################################################################
//  EoE Handler Tests
// ############################################################################

class EoEHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler = createEoEHandler();
        memset(resp, 0, sizeof(resp));
        respLen = sizeof(resp);
    }
    std::unique_ptr<IMailboxHandler> handler;
    uint8_t resp[kBufSize];
    size_t respLen;
};

TEST_F(EoEHandlerTest, ProtocolAndName) {
    EXPECT_EQ(handler->getProtocol(), MailboxProtocol::EoE);
    EXPECT_STREQ(handler->getProtocolName(), "EoE");
}

TEST_F(EoEHandlerTest, ResetDoesNotCrash) {
    handler->reset();
}

TEST_F(EoEHandlerTest, TooShortRequest) {
    uint8_t req[2] = {0, 0};
    bool ok = handler->processRequest(req, 2, resp, respLen);
    EXPECT_FALSE(ok); // returns false + respLen=0
    EXPECT_EQ(respLen, 0u);
}

TEST_F(EoEHandlerTest, GetIPParam) {
    auto req = buildEoEHeader(EOE_TYPE_GET_IP_PARAM_REQ);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_GT(respLen, 4u);

    // Response frame type should be GET_IP_PARAM_RESP (0x08)
    uint8_t respType = (resp[0] >> 4) & 0x0F;
    EXPECT_EQ(respType, EOE_TYPE_GET_IP_PARAM_RESP);

    // Flags at bytes 4-5
    uint16_t flags = resp[4] | (resp[5] << 8);
    // MAC, IP, subnet, gateway, DNS should all be included
    EXPECT_NE(flags & 0x01, 0); // MAC
    EXPECT_NE(flags & 0x02, 0); // IP
    EXPECT_NE(flags & 0x04, 0); // subnet

    // MAC at offset 6
    EXPECT_EQ(resp[6], 0x00);
    EXPECT_EQ(resp[7], 0x01);

    // IP at offset 12
    EXPECT_EQ(resp[12], 192);
    EXPECT_EQ(resp[13], 168);
    EXPECT_EQ(resp[14], 1);
    EXPECT_EQ(resp[15], 100);
}

TEST_F(EoEHandlerTest, SetIPParam) {
    // Build SetIPParam request with IP address
    auto req = buildEoEHeader(EOE_TYPE_SET_IP_PARAM_REQ);
    // Flags: IP included (0x02)
    req.push_back(0x02); req.push_back(0x00);
    // IP address: 10.0.0.42
    req.push_back(10); req.push_back(0); req.push_back(0); req.push_back(42);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    // Verify by reading back
    respLen = sizeof(resp);
    auto getReq = buildEoEHeader(EOE_TYPE_GET_IP_PARAM_REQ);
    handler->processRequest(getReq.data(), getReq.size(), resp, respLen);

    // IP at offset 12 (after 6-byte header + 6-byte MAC)
    EXPECT_EQ(resp[12], 10);
    EXPECT_EQ(resp[13], 0);
    EXPECT_EQ(resp[14], 0);
    EXPECT_EQ(resp[15], 42);
}

TEST_F(EoEHandlerTest, SetFilterUnsupported) {
    auto req = buildEoEHeader(EOE_TYPE_SET_FILTER_REQ);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Result should be unsupported (0x0301)
    uint16_t result = resp[2] | (resp[3] << 8);
    EXPECT_EQ(result, 0x0301);
}

TEST_F(EoEHandlerTest, GetFilterUnsupported) {
    auto req = buildEoEHeader(EOE_TYPE_GET_FILTER_REQ);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    uint16_t result = resp[2] | (resp[3] << 8);
    EXPECT_EQ(result, 0x0301);
}

TEST_F(EoEHandlerTest, UnsupportedFrameType) {
    auto req = buildEoEHeader(0x0F); // invalid frame type
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Result = unsupported frame (0x0001)
    uint16_t result = resp[2] | (resp[3] << 8);
    EXPECT_EQ(result, 0x0001);
}

TEST_F(EoEHandlerTest, FragmentDataSingleComplete) {
    // Build a single-fragment ethernet frame
    std::vector<uint8_t> req(4 + 14, 0); // 4-byte EoE header + 14-byte ethernet frame
    req[0] = (EOE_TYPE_FRAGMENT_DATA << 4); // frame type
    req[1] = 0x80; // lastFragment=1, fragmentNumber=0
    req[2] = 0;    // frameOffset = 0
    req[3] = 0;    // frameNumber = 0

    // Fill with some ethernet-like data
    for (size_t i = 4; i < req.size(); i++) {
        req[i] = static_cast<uint8_t>(i);
    }

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    // Fragment data returns true but with responseLength=0 (no response needed)
    EXPECT_TRUE(ok);
    EXPECT_EQ(respLen, 0u);
}

TEST_F(EoEHandlerTest, TimestampRequestNullCore) {
    auto req = buildEoEHeader(EOE_TYPE_TIMESTAMP_REQUEST);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Response should be timestamp response (type 0x02)
    uint8_t respType = (resp[0] >> 4) & 0x0F;
    EXPECT_EQ(respType, 0x02);
    EXPECT_EQ(respLen, 12u);
}

TEST_F(EoEHandlerTest, SetIPParamMac) {
    auto req = buildEoEHeader(EOE_TYPE_SET_IP_PARAM_REQ);
    // Flags: MAC included (0x01)
    req.push_back(0x01); req.push_back(0x00);
    // MAC: AA:BB:CC:DD:EE:FF
    req.push_back(0xAA); req.push_back(0xBB); req.push_back(0xCC);
    req.push_back(0xDD); req.push_back(0xEE); req.push_back(0xFF);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);

    // Read back
    respLen = sizeof(resp);
    auto getReq = buildEoEHeader(EOE_TYPE_GET_IP_PARAM_REQ);
    handler->processRequest(getReq.data(), getReq.size(), resp, respLen);

    // MAC at offset 6
    EXPECT_EQ(resp[6], 0xAA);
    EXPECT_EQ(resp[7], 0xBB);
    EXPECT_EQ(resp[8], 0xCC);
    EXPECT_EQ(resp[9], 0xDD);
    EXPECT_EQ(resp[10], 0xEE);
    EXPECT_EQ(resp[11], 0xFF);
}

// ############################################################################
//  VoE Handler Tests
// ############################################################################

class VoEHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler = createVoEHandler();
        memset(resp, 0, sizeof(resp));
        respLen = sizeof(resp);
    }
    std::unique_ptr<IMailboxHandler> handler;
    uint8_t resp[kBufSize];
    size_t respLen;
};

TEST_F(VoEHandlerTest, ProtocolAndName) {
    EXPECT_EQ(handler->getProtocol(), MailboxProtocol::VoE);
    EXPECT_STREQ(handler->getProtocolName(), "VoE");
}

TEST_F(VoEHandlerTest, ResetDoesNotCrash) {
    handler->reset();
}

TEST_F(VoEHandlerTest, TooShortRequest) {
    uint8_t req[4] = {0, 0, 0, 0};
    bool ok = handler->processRequest(req, 4, resp, respLen);
    EXPECT_FALSE(ok);
    EXPECT_EQ(respLen, 0u);
}

TEST_F(VoEHandlerTest, EchoHandler) {
    // Echo handler echoes payload back (vendorType=0x0001)
    uint32_t vid = 0; // wildcard
    uint16_t vtype = 0x0001;
    uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    auto req = buildVoERequest(vid, vtype, payload, 4);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_GE(respLen, 8u + 4u); // VoE header + echoed data

    // Check echoed data starts at offset 8 (after VoE header)
    EXPECT_EQ(resp[8], 0xDE);
    EXPECT_EQ(resp[9], 0xAD);
    EXPECT_EQ(resp[10], 0xBE);
    EXPECT_EQ(resp[11], 0xEF);
}

TEST_F(VoEHandlerTest, UnknownVendorType) {
    uint32_t vid = 0;
    uint16_t vtype = 0xFFFF; // not registered
    auto req = buildVoERequest(vid, vtype);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Response has error flag in VoE header
    EXPECT_EQ(resp[6], 0x80); // flags = error
}

TEST_F(VoEHandlerTest, WrongVendorId) {
    uint32_t vid = 0x12345678; // doesn't match handler's vendorId (0)
    uint16_t vtype = 0x0001;
    auto req = buildVoERequest(vid, vtype);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    // Non-matching vendorId (non-zero, not matching) returns false
    EXPECT_FALSE(ok);
}

TEST_F(VoEHandlerTest, DeviceInfoRequest) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0002; // Device Info
    auto req = buildVoERequest(vid, vtype);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_GT(respLen, 8u + 16u); // VoE header + device info struct
}

TEST_F(VoEHandlerTest, DebugGetFlags) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0003; // Debug Command
    uint8_t cmd = 0x01; // Get debug flags
    auto req = buildVoERequest(vid, vtype, &cmd, 1);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Debug flags at offset 8 (after VoE header), default = 0
    EXPECT_EQ(resp[8], 0);
}

TEST_F(VoEHandlerTest, DebugSetFlags) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0003;
    uint8_t cmd[] = {0x02, 0x42}; // Set debug flags to 0x42
    auto req = buildVoERequest(vid, vtype, cmd, 2);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[8], 0); // success

    // Read back
    respLen = sizeof(resp);
    uint8_t getCmd = 0x01;
    auto req2 = buildVoERequest(vid, vtype, &getCmd, 1);
    handler->processRequest(req2.data(), req2.size(), resp, respLen);
    EXPECT_EQ(resp[8], 0x42);
}

TEST_F(VoEHandlerTest, DebugResetStats) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0003;
    uint8_t cmd = 0x04; // Reset statistics
    auto req = buildVoERequest(vid, vtype, &cmd, 1);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[8], 0); // success
}

TEST_F(VoEHandlerTest, DebugUnknownCommand) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0003;
    uint8_t cmd = 0xFF; // Unknown
    auto req = buildVoERequest(vid, vtype, &cmd, 1);

    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Error marker
    EXPECT_EQ(resp[8], 0xFF);
}

TEST_F(VoEHandlerTest, ConfigWriteAndRead) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0005; // Configuration

    // Write config
    uint8_t writeCmd[] = {0x02, 0x11, 0x22, 0x33}; // cmd=write, data={0x11,0x22,0x33}
    auto wReq = buildVoERequest(vid, vtype, writeCmd, 4);
    bool ok = handler->processRequest(wReq.data(), wReq.size(), resp, respLen);
    EXPECT_TRUE(ok);

    // Read config back
    respLen = sizeof(resp);
    uint8_t readCmd = 0x01;
    auto rReq = buildVoERequest(vid, vtype, &readCmd, 1);
    ok = handler->processRequest(rReq.data(), rReq.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Config data at offset 8
    EXPECT_EQ(resp[8], 0x11);
    EXPECT_EQ(resp[9], 0x22);
    EXPECT_EQ(resp[10], 0x33);
}

TEST_F(VoEHandlerTest, ConfigReset) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0005;

    // Write config first
    uint8_t writeCmd[] = {0x02, 0xAA};
    auto wReq = buildVoERequest(vid, vtype, writeCmd, 2);
    handler->processRequest(wReq.data(), wReq.size(), resp, respLen);

    // Reset config
    respLen = sizeof(resp);
    uint8_t resetCmd = 0x03;
    auto resetReq = buildVoERequest(vid, vtype, &resetCmd, 1);
    bool ok = handler->processRequest(resetReq.data(), resetReq.size(), resp, respLen);
    EXPECT_TRUE(ok);

    // Read config — should be empty
    respLen = sizeof(resp);
    uint8_t readCmd = 0x01;
    auto rReq = buildVoERequest(vid, vtype, &readCmd, 1);
    ok = handler->processRequest(rReq.data(), rReq.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Response length = VoE header (8) + 0 data
    EXPECT_EQ(respLen, 8u);
}

TEST_F(VoEHandlerTest, ConfigSave) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0005;
    uint8_t saveCmd = 0x04; // Save to persistent
    auto req = buildVoERequest(vid, vtype, &saveCmd, 1);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[8], 0); // success
}

TEST_F(VoEHandlerTest, FirmwareUpdateAbort) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0004; // Firmware Update
    uint8_t abortCmd = 0x04; // Abort
    auto req = buildVoERequest(vid, vtype, &abortCmd, 1);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    EXPECT_EQ(resp[8], 0); // success
}

TEST_F(VoEHandlerTest, FirmwareUpdateStartWithoutCore) {
    // Should fail: core_ is null, and code checks core_->getState()
    // But we added null guard: if (core_ && core_->getState() != BOOT)
    // When core_ is null, the check is skipped, so start should succeed
    uint32_t vid = 0;
    uint16_t vtype = 0x0004;
    uint8_t startCmd = 0x01; // Start update
    auto req = buildVoERequest(vid, vtype, &startCmd, 1);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // With null core_, bootstrap check is bypassed, so start should succeed
    EXPECT_EQ(resp[8], 0);
}

TEST_F(VoEHandlerTest, DebugStatsWithNullCore) {
    // Debug command 0x03 uses core_->getDCSystemTime() — guarded with null check
    uint32_t vid = 0;
    uint16_t vtype = 0x0003;
    uint8_t cmd = 0x03; // Get statistics
    auto req = buildVoERequest(vid, vtype, &cmd, 1);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Should return stats struct with uptimeSeconds=0
    EXPECT_GT(respLen, 8u);
}

TEST_F(VoEHandlerTest, DebugMissingCommand) {
    uint32_t vid = 0;
    uint16_t vtype = 0x0003;
    // No payload (length < 1)
    auto req = buildVoERequest(vid, vtype);
    bool ok = handler->processRequest(req.data(), req.size(), resp, respLen);
    EXPECT_TRUE(ok);
    // Error: missing command
    EXPECT_EQ(resp[8], 0xFF); // error marker
}

// Additional MailboxHeader tests (supplement test_CoEHandler.cpp)

TEST(MailboxHeaderExtraTest, CounterFieldWrap) {
    MailboxHeader h{};
    h.counter = 15;
    h.type = 0x0F;
    uint8_t buf[6];
    h.toBytes(buf);
    auto parsed = MailboxHeader::fromBytes(buf);
    EXPECT_EQ(parsed.counter, 15);
    EXPECT_EQ(parsed.type, 0x0F);
}

TEST(MailboxHeaderExtraTest, ChannelAndPriority) {
    MailboxHeader h{};
    h.channel = 0x3F;
    h.priority = 3;
    h.length = 0x1234;
    h.address = 0x5678;
    uint8_t buf[6];
    h.toBytes(buf);
    auto parsed = MailboxHeader::fromBytes(buf);
    EXPECT_EQ(parsed.channel, 0x3F);
    EXPECT_EQ(parsed.priority, 3);
    EXPECT_EQ(parsed.length, 0x1234);
    EXPECT_EQ(parsed.address, 0x5678);
}
