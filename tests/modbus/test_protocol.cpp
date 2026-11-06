// Protocol-layer tests: CRC, bit packing, codec round-trips, malformed
// inputs, serial config, scalar codec.
#include <gtest/gtest.h>

#include "tether/modbus/protocol/BitPacker.hpp"
#include "tether/modbus/protocol/Builder.hpp"
#include "tether/modbus/protocol/Codec.hpp"
#include "tether/modbus/protocol/Crc.hpp"
#include "tether/modbus/link/SerialConfig.hpp"
#include "tether/modbus/link/RtuTiming.hpp"
#include "tether/modbus/typed/ScalarCodec.hpp"

using namespace tether::modbus;

TEST(Crc, KnownVector) {
    // Reference: CRC-16/MODBUS of 11 03 00 6B 00 03 = 0x7687 (pymodbus)
    const uint8_t frame[] = {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};
    EXPECT_EQ(Crc16::of(frame), 0x8776);
}
TEST(Crc, CheckValidates) {
    std::vector<uint8_t> f = {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03};
    const uint16_t c = Crc16::of(f);
    f.push_back(uint8_t(c)); f.push_back(uint8_t(c >> 8));
    EXPECT_TRUE(Crc16::check(f));
    f[2] ^= 1;
    EXPECT_FALSE(Crc16::check(f));
}
TEST(Lrc, SumsToZero) {
    const uint8_t b[] = {0x11, 0x03, 0x00, 0x6B};
    const uint8_t l = lrc(b);
    uint8_t sum = 0;
    for (auto x : b) sum = uint8_t(sum + x);
    EXPECT_EQ(uint8_t(sum + l), 0);
}

TEST(BitPacker, RoundTrip) {
    const std::vector<bool> bits = {1,0,1,1,0,0,0,1, 1,1};
    const auto packed = packBits(bits);
    ASSERT_EQ(packed.size(), 2u);
    EXPECT_EQ(packed[0], 0b10001101);
    EXPECT_EQ(packed[1], 0b00000011);
    const auto out = unpackBits(packed, 10);
    EXPECT_EQ(out, bits);
}

TEST(Codec, ReadRequestRoundTrip) {
    auto pdu = build::readRange(FunctionCode::ReadHoldingRegisters, 0x6B, 3);
    ASSERT_TRUE(pdu.has_value());
    ASSERT_EQ(pdu->body().size(), 4u);
    auto req = decodeRequest(FunctionCode::ReadHoldingRegisters,
                             pdu->body());
    ASSERT_TRUE(req.has_value());
    auto& r = std::get<ReadRangeRequest>(*req);
    EXPECT_EQ(r.addr, 0x6B); EXPECT_EQ(r.qty, 3);
}

TEST(Codec, ReadRegsResponseRoundTrip) {
    RegistersResponse rr{{0x02, 0x2B, 0x0000}};
    ReadRangeRequest ctx{0, 3};
    auto body = encodeResponse(FunctionCode::ReadHoldingRegisters,
                               Response{rr}, Request{ctx});
    ASSERT_TRUE(body.has_value());
    ASSERT_EQ(body->size(), 7u);
    EXPECT_EQ((*body)[0], 6);  // byte count
    auto dec = decodeResponse(FunctionCode::ReadHoldingRegisters,
                              std::span(*body));
    ASSERT_TRUE(dec.has_value());
    EXPECT_EQ(std::get<RegistersResponse>(*dec).regs,
              (std::vector<uint16_t>{2, 43, 0}));
}

TEST(Codec, BitsResponseByteCount) {
    BitsResponse br{{1,0,1,1,0,0,0,1,1,1}};
    ReadRangeRequest ctx{0, 10};
    auto body = encodeResponse(FunctionCode::ReadCoils, Response{br},
                               Request{ctx});
    ASSERT_TRUE(body.has_value());
    EXPECT_EQ((*body)[0], 2);
    auto dec = decodeResponse(FunctionCode::ReadCoils, std::span(*body));
    ASSERT_TRUE(dec.has_value());
    // decoder returns byteCount*8 lanes (10 data + 6 padding zeros)
    auto bits = std::get<BitsResponse>(*dec).bits;
    ASSERT_EQ(bits.size(), 16u);
    bits.resize(10);
    EXPECT_EQ(bits, br.bits);
}

TEST(Codec, WriteMultiRegsEncode) {
    auto pdu = build::writeMultiRegs(0x01, std::vector<uint16_t>{0x000A,0x0102});
    ASSERT_TRUE(pdu.has_value());
    const auto b = pdu->body();
    ASSERT_EQ(b.size(), 9u);
    EXPECT_EQ(b[4], 4);  // byte count
    auto req = decodeRequest(FunctionCode::WriteMultipleRegisters, b);
    ASSERT_TRUE(req.has_value());
    auto& r = std::get<WriteMultiRegsRequest>(*req);
    EXPECT_EQ(r.qty, 2);
    EXPECT_EQ(r.regs[1], 0x0102);
}

TEST(Codec, TruncatedInputsRejected) {
    for (size_t n = 0; n < 4; ++n) {
        const std::vector<uint8_t> b(n, 0);
        EXPECT_FALSE(decodeRequest(FunctionCode::ReadHoldingRegisters, b));
    }
    EXPECT_FALSE(decodeResponse(FunctionCode::ReadHoldingRegisters,
                                std::span<const uint8_t>{}));
}

TEST(Codec, OverrangeQtyRejected) {
    EXPECT_FALSE(build::readRange(FunctionCode::ReadHoldingRegisters,
                                  0, kMaxReadRegsQty + 1).has_value());
    EXPECT_FALSE(build::readRange(FunctionCode::ReadCoils, 0,
                                  kMaxReadBitsQty + 1).has_value());
}

TEST(Codec, EchoVerify) {
    WriteSingleRegRequest w{5, 0xABCD};
    auto body = encodeRequest(FunctionCode::WriteSingleRegister,
                              Request{w});
    ASSERT_TRUE(body.has_value());
    EXPECT_TRUE(verifyEcho(FunctionCode::WriteSingleRegister,
                           *body, *body));
    auto bad = *body; bad[3] ^= 1;
    EXPECT_FALSE(verifyEcho(FunctionCode::WriteSingleRegister, *body, bad));
}

TEST(SerialConfig, Parse) {
    auto c = SerialConfig::parse("/dev/ttyUSB0:19200-8E1");
    ASSERT_TRUE(c.has_value());
    EXPECT_EQ(c->device, "/dev/ttyUSB0");
    EXPECT_EQ(c->baud, 19200u);
    EXPECT_EQ(c->parity, Parity::Even);
    EXPECT_EQ(c->stopBits, StopBits::One);
    EXPECT_EQ(c->bitsPerChar(), 11);
    auto c2 = SerialConfig::parse("/dev/ttyS1:9600-8N2");
    ASSERT_TRUE(c2.has_value());
    EXPECT_EQ(c2->parity, Parity::None);
    EXPECT_EQ(c2->stopBits, StopBits::Two);
    EXPECT_EQ(c2->bitsPerChar(), 11);
    EXPECT_FALSE(SerialConfig::parse("/dev/x:notabaud").has_value());
}

TEST(RtuTiming, BaudFloors) {
    const auto hi = RtuTiming::forParams(115200, 11);
    EXPECT_EQ(hi.t35.count(), 1750);   // fixed floor above 19200
    const auto lo = RtuTiming::forParams(9600, 11);
    EXPECT_GT(lo.t35.count(), 1750);   // ~4.0ms computed
}

TEST(ScalarCodec, F32ByteOrders) {
    const float v = 1234.5f;
    ScalarSpec s{ValueType::F32, WordOrder::BigEndian, 1, 0};
    auto regs = encodeScalar(v, s);
    ASSERT_TRUE(regs.has_value());
    EXPECT_NEAR(decodeScalar(*regs, s), v, 0.01);
    s.order = WordOrder::WordSwap;
    regs = encodeScalar(v, s);
    ASSERT_TRUE(regs.has_value());
    EXPECT_NEAR(decodeScalar(*regs, s), v, 0.01);
    // order actually changed the register order
    ScalarSpec s2{ValueType::F32, WordOrder::BigEndian, 1, 0};
    EXPECT_NE(decodeScalar(*regs, s2), v);
}

TEST(ScalarCodec, ScaleOffset) {
    ScalarSpec s{ValueType::I16, WordOrder::BigEndian, 0.1, -40.0};
    auto regs = encodeScalar(23.5, s);
    ASSERT_TRUE(regs.has_value());
    EXPECT_NEAR(decodeScalar(*regs, s), 23.5, 0.05);
}
