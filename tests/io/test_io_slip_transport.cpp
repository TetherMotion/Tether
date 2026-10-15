/**
 * @file test_io_slip_transport.cpp
 * @brief Tests for SlipFramedTransport — the SLIP client-side framing
 *        adapter — including a full in-proc Session/Client round trip.
 */
#include <gtest/gtest.h>
#include "tether/io/SlipTransport.hpp"
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/SimulatedCiA402Fleet.hpp"
#include "PipeTransport.hpp"
#include "SLIPStream/Buffer.hpp"

#include <thread>

using namespace tether::io;
using namespace tether::io::testing;

namespace {

void noopLog(const char*, const char*, ...) {}

// Raw-level frame decode/encode through the pipe.
TEST(IOSlipTransport, DecodesServerFrameAndEscapesPayload) {
    auto [clientEnd, serverEnd] = PipeTransport::create();
    SlipFramedTransport slip(std::move(clientEnd));

    // Server pushes a SLIP frame whose payload contains END and ESC bytes.
    const uint8_t payload[] = {0x07, 0xC0, 0xDB, 0x42, 0xC0};
    const size_t encLen = SLIPStream::encoded_length(payload, sizeof(payload));
    std::vector<uint8_t> frame(encLen);
    ASSERT_EQ(SLIPStream::encode_packet(payload, sizeof(payload),
                                        frame.data(), frame.size()), encLen);
    ASSERT_TRUE(serverEnd->send(frame.data(), frame.size()));

    std::vector<uint8_t> msg;
    ASSERT_TRUE(slip.receiveMessage(msg, 1000));
    ASSERT_EQ(msg.size(), sizeof(payload));
    EXPECT_EQ(std::memcmp(msg.data(), payload, sizeof(payload)), 0);

    // Client-to-server direction: send payload with END/ESC, expect the
    // same frame shape back on the pipe.
    ASSERT_TRUE(slip.send(payload, sizeof(payload)));
    std::vector<uint8_t> echoed(encLen);
    const size_t got = serverEnd->receive(echoed.data(), echoed.size(), 1000);
    ASSERT_GT(got, 0u);
    EXPECT_EQ(std::memcmp(echoed.data(), frame.data(), got), 0);
}

TEST(IOSlipTransport, ResyncsAfterGarbageAndDropsOversized) {
    auto [clientEnd, serverEnd] = PipeTransport::create();
    SlipFramedTransport slip(std::move(clientEnd), /*maxEncoded*/64,
                             /*maxDecoded*/64);

    // Garbage bytes before the first END accumulate into the next packet —
    // matching Session's deframing semantics (resync only kicks in after
    // oversized or undecodable frames).
    const uint8_t garbage[] = {0x01, 0x02, 0x03};
    ASSERT_TRUE(serverEnd->send(garbage, sizeof(garbage)));
    const uint8_t good[] = {0x0D, 0x2A};
    const size_t encLen = SLIPStream::encoded_length(good, sizeof(good));
    std::vector<uint8_t> frame(encLen);
    ASSERT_EQ(SLIPStream::encode_packet(good, sizeof(good),
                                        frame.data(), frame.size()), encLen);
    ASSERT_TRUE(serverEnd->send(frame.data(), frame.size()));
    std::vector<uint8_t> msg;
    ASSERT_TRUE(slip.receiveMessage(msg, 1000));
    EXPECT_EQ(msg.size(), 5u);   // garbage + good = one decoded packet

    // A frame larger than maxEncoded is dropped through its next END; the
    // following frame still arrives.
    std::vector<uint8_t> big(200, 0x11);
    const size_t bigEnc = SLIPStream::encoded_length(big.data(), big.size());
    std::vector<uint8_t> bigFrame(bigEnc);
    ASSERT_EQ(SLIPStream::encode_packet(big.data(), big.size(),
                                        bigFrame.data(), bigFrame.size()),
              bigEnc);
    ASSERT_TRUE(serverEnd->send(bigFrame.data(), bigFrame.size()));
    ASSERT_TRUE(serverEnd->send(frame.data(), frame.size()));
    ASSERT_TRUE(slip.receiveMessage(msg, 1000));
    EXPECT_EQ(msg.size(), 2u);
    EXPECT_GE(slip.droppedFrames(), 1u);
}

// Full in-proc round trip: TetherIOClient over SlipFramedTransport talking
// to a real Session with Framing::Slip over PipeTransport byte-streams.
class SlipClientIntegrationTest : public ::testing::Test {
protected:
    Registry registry_;
    SchemaGraph graph_;
    SchemaCatalog catalog_;
    cia402::SimulatedCiA402Fleet fleet_;
    std::unique_ptr<Session> session_;
    std::unique_ptr<TetherIOClient> client_;
    std::thread sessionThread_;

    void SetUp() override {
        graph_ = cia402::machineProfileSchemaGraph();
        ASSERT_TRUE(cia402::SimulatedCiA402Fleet::installSchemas(graph_, catalog_));
        ASSERT_TRUE(fleet_.registerSignals(registry_, catalog_));
        auto [clientEnd, serverEnd] = PipeTransport::create();
        session_ = std::make_unique<Session>(
            std::move(serverEnd), registry_,
            [] { return cia402::simulated_fleet_detail::monotonicTimestampUs(); },
            noopLog, nullptr, nullptr, nullptr, nullptr, nullptr,
            Framing::Slip, nullptr, &catalog_);
        sessionThread_ = std::thread([this] { session_->run(); });
        client_ = std::make_unique<TetherIOClient>(
            std::make_unique<SlipFramedTransport>(std::move(clientEnd)), 5000);
        ASSERT_TRUE(client_->ping(0xC0)) << "client failed over SLIP";
    }

    void TearDown() override {
        if (client_) client_->close();
        if (session_) session_->requestStop();
        if (sessionThread_.joinable()) sessionThread_.join();
    }
};

TEST_F(SlipClientIntegrationTest, FullProtocolRoundTrip) {
    // Ping nonce containing a 0xC0 byte exercises escaping on both paths.
    auto nonce = client_->ping(0xC0C0C0C0);
    ASSERT_TRUE(nonce);
    EXPECT_EQ(*nonce, 0xC0C0C0C0u);

    auto signals = client_->listSignals();
    ASSERT_TRUE(signals) << signals.error().message;
    ASSERT_FALSE(signals->empty());

    auto raw = client_->getSignal(signals->front().id);
    ASSERT_TRUE(raw) << raw.error().message;
    auto v = client_->decodeValue(signals->front().schemaSlot,
                                  raw->data(), raw->size());
    ASSERT_TRUE(v) << v.error().message;

    auto snap = client_->snapshotSignals({signals->front().id});
    ASSERT_TRUE(snap) << snap.error().message;
    EXPECT_EQ(snap->second.size(), 1u);
}

} // namespace
