#include <gtest/gtest.h>
#include <cstring>

// Minimal subset of utilities ported from original tests
namespace {
struct TestFrame {
    static constexpr size_t MAX_SIZE = 1518;
    uint8_t data[MAX_SIZE];
    size_t length{0};
    void clear() { std::memset(data,0,MAX_SIZE); length=0; }
    uint8_t& operator[](size_t i){return data[i];}
    const uint8_t& operator[](size_t i) const { return data[i]; }
};

size_t buildEcatFrame(TestFrame& frame, uint8_t cmd, uint16_t adp, uint16_t ado,
                      const uint8_t* data, uint16_t dataLen, uint8_t idx = 0) {
    frame.clear();
    std::memset(&frame[0], 0xFF, 6);
    frame[6]=0x00; frame[7]=0x01; frame[8]=0x02;
    frame[9]=0x03; frame[10]=0x04; frame[11]=0x05;
    frame[12]=0x88; frame[13]=0xA4;
    uint16_t ecatLen = 10 + dataLen + 2;
    uint16_t ecatHdr = ecatLen | (0x01 << 12);
    frame[14] = ecatHdr & 0xFF;
    frame[15] = (ecatHdr >> 8) & 0xFF;
    frame[16]=cmd; frame[17]=idx;
    frame[18] = adp & 0xFF; frame[19] = (adp>>8)&0xFF;
    frame[20] = ado & 0xFF; frame[21] = (ado>>8)&0xFF;
    uint16_t lenField = dataLen & 0x07FF;
    frame[22] = lenField & 0xFF; frame[23] = (lenField>>8)&0xFF;
    frame[24]=0; frame[25]=0;
    if (data && dataLen>0) memcpy(&frame[26], data, dataLen);
    frame[26+dataLen]=0; frame[27+dataLen]=0;
    frame.length = 28 + dataLen;
    return frame.length;
}

struct ParsedDatagram { uint8_t cmd; uint8_t idx; uint16_t adp; uint16_t ado; uint16_t len; uint16_t wkc; const uint8_t* data; bool valid; };

ParsedDatagram parseEcatDatagram(const TestFrame& frame, size_t offset=16) {
    ParsedDatagram dg{};
    if (frame.length < offset + 12) { dg.valid=false; return dg; }
    dg.cmd = frame[offset];
    dg.idx = frame[offset+1];
    dg.adp = frame[offset+2] | (frame[offset+3]<<8);
    dg.ado = frame[offset+4] | (frame[offset+5]<<8);
    uint16_t lenField = frame[offset+6] | (frame[offset+7]<<8);
    dg.len = lenField & 0x07FF;
    dg.data = &frame[offset+10];
    dg.wkc = frame[offset+10+dg.len] | (frame[offset+10+dg.len+1]<<8);
    dg.valid = true;
    return dg;
}
}

TEST(EtherCATPackets, BuildAndParseHeader) {
    TestFrame frame;
    uint8_t data[] = {1,2,3,4};
    buildEcatFrame(frame, 0x04, 0x1001, 0x0300, data, 4);
    uint16_t ethertype = (frame[12]<<8) | frame[13];
    EXPECT_EQ(0x88A4, ethertype);
    auto dg = parseEcatDatagram(frame);
    ASSERT_TRUE(dg.valid);
    EXPECT_EQ(0x04u, dg.cmd);
    EXPECT_EQ(0x0300u, dg.ado);
}
