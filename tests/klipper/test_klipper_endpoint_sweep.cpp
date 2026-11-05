/**
 * @file test_klipper_endpoint_sweep.cpp
 * @brief Error-path sweep: calls every registered endpoint with empty
 *        params (exercising the missing-parameter branches the happy-path
 *        tests skip) and drives the stateful endpoints through their
 *        not-found / invalid-argument variants.
 */

#include <gtest/gtest.h>

#include <string>
#include <unistd.h>

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/JsonValue.hpp"

namespace {

using namespace tether::klipper::klippy;

static std::string uniqueSock() {
    return "/tmp/tether_test_epsweep_" + std::to_string(getpid()) + ".sock";
}

JsonValue params(std::map<std::string, JsonValue> m) {
    return JsonValue(std::move(m));
}
JsonValue str(const char* s) { return JsonValue(std::string(s)); }
JsonValue num(double v) { return JsonValue(v); }
JsonValue flag(bool v) { return JsonValue(v); }
JsonValue empty() { return params({}); }

class EndpointSweepTest : public ::testing::Test {
protected:
    void SetUp() override {
        UdsServerConfig cfg;
        cfg.socketPath = sockPath_;
        server_ = std::make_unique<KlippyServer>(cfg);
    }
    void TearDown() override {
        server_.reset();
        ::unlink(sockPath_.c_str());
    }

    // Every response must be an object; most error paths answer
    // {"error": ...} — the point is exercising the handler code.
    JsonValue call(const std::string& method,
                   JsonValue p = JsonValue(std::map<std::string, JsonValue>{})) {
        auto r = server_->callEndpoint(method, p);
        EXPECT_TRUE(r.isObject()) << method;
        return r;
    }

    std::string sockPath_ = uniqueSock();
    std::unique_ptr<KlippyServer> server_;
};

// Call every registered endpoint with empty params — covers the
// "missing parameter" and empty-store branches in one pass.
TEST_F(EndpointSweepTest, AllEndpointsWithEmptyParams) {
    auto eps = server_->listEndpoints();
    ASSERT_GT(eps.size(), 50u);
    for (const auto& ep : eps) {
        call(ep, empty());
    }
}

// Stateful handlers: not-found and invalid-argument variants.
TEST_F(EndpointSweepTest, WebcamVariants) {
    call("server/webcams/update", params({{"name", str("ghost")}}));
    call("server/webcams/get", params({{"name", str("ghost")}}));
    call("server/webcams/delete", params({{"name", str("ghost")}}));
    call("server/webcams/test", params({{"name", str("ghost")}}));

    // Create then exercise all update fields.
    call("server/webcams/update",
         params({{"name", str("cam")}, {"location", str("bed")},
                 {"stream_url", str("s")}, {"snapshot_url", str("sn")},
                 {"service", str("svc")}, {"enabled", flag(false)},
                 {"rotation", num(90)}}));
    call("server/webcams/get", params({{"name", str("cam")}}));
    call("server/webcams/test", params({{"name", str("cam")}}));
    call("server/webcams/delete", params({{"name", str("cam")}}));
    call("server/webcams/delete", params({{"name", str("cam")}}));  // again: not found
}

TEST_F(EndpointSweepTest, DevicePowerVariants) {
    call("machine/device_power/state", params({{"device", str("x")}}));
    call("machine/device_power/on", params({{"device", str("x")}}));
    call("machine/device_power/off", params({{"device", str("x")}}));
    call("machine/device_power/toggle", params({{"device", str("x")}}));
}

TEST_F(EndpointSweepTest, JobQueueAndHistoryVariants) {
    call("job_queue/jump_to", params({{"job_id", num(99)}}));
    call("job_queue/jump_to", params({{"job_id", str("x")}}));
    call("job_queue/delete_job", params({{"job_id", num(0)}}));
    call("job_queue/delete_job", params({{"job_id", str("x")}}));

    // Post a job then jump/delete validly.
    call("job_queue/post_job",
         params({{"filenames", JsonValue(std::vector<JsonValue>{str("a.gcode")})}}));
    call("job_queue/jump_to", params({{"job_id", num(0)}}));
    call("job_queue/delete_job", params({{"job_id", num(0)}}));
    call("job_queue/start");
    call("job_queue/pause");

    // job_history/delete by id, unknown id, all flag and missing param.
    call("job_history/delete", params({{"job_id", num(1)}}));
    call("job_history/delete", params({{"job_id", num(77)}}));
    call("job_history/delete", params({{"all", flag(true)}}));
    call("job_history/delete", params({{"all", flag(false)}}));
    call("job_history/get", params({{"uid", str("none")}}));
}

TEST_F(EndpointSweepTest, BotDeviceNotepadVariants) {
    call("bot/get", params({{"name", str("none")}}));
    call("bot/update", params({{"name", str("none")}, {"status", str("x")}}));
    call("bot/delete", params({{"name", str("none")}}));
    call("bot/create", params({{"name", str("b")}, {"status", str("s")}}));
    call("bot/get", params({{"name", str("b")}}));
    call("bot/update", params({{"name", str("b")}, {"status", str("off")}}));
    call("bot/delete", params({{"name", str("b")}}));

    call("devices/get", params({{"name", str("none")}}));
    call("devices/delete", params({{"name", str("none")}}));
    call("devices/update", params({{"name", str("none")}}));

    call("notepad/get", params({{"key", str("none")}}));
    call("notepad/delete", params({{"key", str("none")}}));
    call("notepad/put", params({{"key", str("k")}, {"value", num(1)}}));
    call("notepad/get", params({{"key", str("k")}}));
    call("notepad/delete", params({{"key", str("k")}}));
}

TEST_F(EndpointSweepTest, DatabaseVariants) {
    call("database/get", params({{"namespace", str("ns")}, {"key", str("none")}}));
    call("database/delete", params({{"namespace", str("ns")}, {"key", str("none")}}));
    call("server/database/get_item", params({{"namespace", str("ns")}}));
    call("server/database/delete", params({{"namespace", str("ns")}, {"key", str("k")}}));
    call("server/database/post_item",
         params({{"namespace", str("ns")}, {"key", str("k")}, {"value", num(5)}}));
    call("server/database/get_item",
         params({{"namespace", str("ns")}, {"key", str("k")}}));
}

TEST_F(EndpointSweepTest, AccessAndAnnouncementsVariants) {
    call("access/login", params({{"username", str("u")}}));
    call("access/login", params({{"username", str("u")}, {"password", str("p")}}));
    call("access/refresh_jwt", params({{"refresh_token", str("bad")}}));
    call("announcements/feed", params({{"feed", str("other_feed")}}));
    call("announcements/dismiss", params({{"entry_id", str("none")}}));
}

TEST_F(EndpointSweepTest, ServicesUpdateAndMiscVariants) {
    call("machine/services/start", params({{"service", str("nope")}}));
    call("machine/services/stop", params({{"service", str("nope")}}));
    call("machine/services/restart", params({{"service", str("nope")}}));

    call("spoolman/spool_id", params({{"spool_id", num(1)}}));
    call("spoolman/proxy", params({{"path", str("/v1/spool")}}));
    call("spoolman/proxy", empty());

    call("gcode/script", params({{"script", str("M115")}}));
    call("printer/gcode/script", empty());
    call("printer/objects/query", params({{"objects", empty()}}));
    call("printer/objects/subscribe", params({{"objects", empty()}}));

    call("server/files/metadata", params({{"filename", str("none.gcode")}}));
    call("server/files/metascan", params({{"filename", str("none.gcode")}}));
    call("server/files/thumbnails", params({{"filename", str("none.gcode")}}));
    call("server/files/directory", params({{"path", str("nowhere")}}));
    call("server/files/create_dir", params({{"path", str("gcodes/d")}}));
    call("server/files/delete", params({{"path", str("gcodes/d")}}));
}

}  // namespace
