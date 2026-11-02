/**
 * @file test_io_input_stream.cpp
 * @brief Input-stream regression tests (status byte convention: 0 = ok).
 */

#include <gtest/gtest.h>
#include "tether/io/TetherIOClient.hpp"
#include "tether/io/Session.hpp"
#include "tether/io/SchemaCatalog.hpp"
#include "tether/io/SchemaDigest.hpp"
#include "tether/io/SchemaValueBuilder.hpp"
#include "PipeTransport.hpp"
#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

using namespace tether::io;
using namespace tether::io::testing;

static void noopLogFn(const char*, const char*, ...) {}

class IoInputStreamTest : public ::testing::Test {
protected:
    Registry registry_;
    SchemaGraph graph_;
    SchemaCatalog catalog_;
    std::unique_ptr<Session> session_;
    std::unique_ptr<TetherIOClient> client_;
    std::thread sessionThread_;
    std::atomic<uint32_t> valuesReceived_{0};

    static SchemaKey key(uint8_t tag) {
        SchemaKey k{};
        k[0] = 0xDE;
        k[1] = tag;
        return k;
    }

    void SetUp() override {
        SchemaNode u64n;
        u64n.key = key(1);
        u64n.kind = SchemaKind::Scalar;
        u64n.scalarType = ValueType::U64;
        u64n.name = "demo.U64";
        graph_.nodes = {u64n};
        ASSERT_TRUE(resolveSchemaDigests(graph_));
        ASSERT_TRUE(catalog_.install(graph_, buildSchemaManifest(graph_)));

        auto [clientEnd, serverEnd] = MessagePipeTransport::create();
        session_ = std::make_unique<Session>(
            std::move(serverEnd), registry_,
            [] {
                return static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now()
                            .time_since_epoch())
                        .count());
            },
            noopLogFn, nullptr,
            // inputStreamCreateFn
            [](uint32_t, const SchemaRef&, SchemaEpoch, SchemaSlot,
               uint32_t, uint32_t) { return true; },
            // inputStreamDataFn
            [this](uint32_t,
                   const std::vector<std::vector<uint8_t>>& values) {
                valuesReceived_ += static_cast<uint32_t>(values.size());
            },
            nullptr, nullptr, Framing::None, nullptr, &catalog_);
        sessionThread_ = std::thread([this] { session_->run(); });
        client_ = std::make_unique<TetherIOClient>(std::move(clientEnd), 5000);
        for (int i = 0; i < 50; ++i) {
            if (client_->ping(1)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void TearDown() override {
        client_.reset();
        if (session_) session_->requestStop();
        if (sessionThread_.joinable()) sessionThread_.join();
    }
};

TEST_F(IoInputStreamTest, CreateReportsSuccess) {
    // Regression: the response status byte is 0=success — a client reading
    // != 0 reported every accepted stream as failed.
    const auto slot = catalog_.slotForKey(key(1));
    ASSERT_TRUE(slot.has_value());
    const auto created = client_->createInputStream(*slot, 8, 4);
    ASSERT_TRUE(created.has_value());
    EXPECT_TRUE(created->success);
    EXPECT_NE(created->streamId, 0u);
}

TEST_F(IoInputStreamTest, DataAndCloseRoundTrip) {
    const auto slot = catalog_.slotForKey(key(1));
    ASSERT_TRUE(slot.has_value());
    const auto created = client_->createInputStream(*slot, 8, 4);
    ASSERT_TRUE(created && created->success);

    const SchemaNode& u64n = *graph_.find(key(1));
    std::vector<std::vector<uint8_t>> batch;
    for (uint64_t i = 0; i < 2; ++i) {
        auto v = encodeNative(graph_, u64n, nv::u64(i));
        ASSERT_TRUE(v.has_value());
        batch.push_back(std::move(*v));
    }
    EXPECT_TRUE(client_->inputStreamData(created->streamId, batch).has_value());

    const auto closed = client_->closeInputStream(created->streamId);
    ASSERT_TRUE(closed.has_value());
    EXPECT_TRUE(closed->second);

    for (int i = 0; i < 50 && valuesReceived_ < 2; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    EXPECT_EQ(valuesReceived_.load(), 2u);
}
