/**
 * @file test_klipper_file_endpoints.cpp
 * @brief Coverage battery for the server/info, machine/system_info,
 *        machine/procstats, file-management and C1 file/log endpoint
 *        handlers (KlippyServerFileEndpoints.cpp +
 *        KlippyServerC1Endpoints.cpp) through callEndpoint() dispatch.
 */

#include <gtest/gtest.h>

#include <string>
#include <unistd.h>

#include "tether/klipper/klippy/JsonValue.hpp"
#include "tether/klipper/klippy/KlippyServer.hpp"

namespace {

using namespace tether::klipper::klippy;

static std::string uniqueSock() {
    return "/tmp/tether_test_fileep_" + std::to_string(getpid()) + ".sock";
}

JsonValue params(std::map<std::string, JsonValue> m) {
    return JsonValue(std::move(m));
}
JsonValue str(const char* s) { return JsonValue(std::string(s)); }
JsonValue empty() { return params({}); }

class FileEndpointsTest : public ::testing::Test {
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

    JsonValue call(const std::string& method,
                   JsonValue p = JsonValue(std::map<std::string, JsonValue>{})) {
        auto r = server_->callEndpoint(method, p);
        EXPECT_TRUE(r.isObject()) << method;
        return r;
    }

    std::string sockPath_ = uniqueSock();
    std::unique_ptr<KlippyServer> server_;
};

// ---------------------------------------------------------------------------
// server/info, machine/system_info, machine/procstats
// ---------------------------------------------------------------------------

TEST_F(FileEndpointsTest, ServerInfoReportsVersionAndState) {
    auto r = call("server/info");
    ASSERT_TRUE(r.has("result"));
    const auto& info = *r.find("result");
    EXPECT_TRUE(info.has("version"));
    EXPECT_TRUE(info.has("klippy_state"));
    EXPECT_TRUE(info.has("klippy_connected"));
    EXPECT_TRUE(info.has("components"));
    EXPECT_TRUE(info.has("api_version"));
}

TEST_F(FileEndpointsTest, MachineSystemInfo) {
    call("machine/system_info");
}

TEST_F(FileEndpointsTest, MachineProcstats) {
    call("machine/procstats");
}

// ---------------------------------------------------------------------------
// Stores
// ---------------------------------------------------------------------------

TEST_F(FileEndpointsTest, TemperatureStore) {
    call("server/temperature_store");
    call("server/temperature_store",
         params({{"include_monitors", JsonValue(true)}}));
}

TEST_F(FileEndpointsTest, GcodeStore) {
    call("server/gcode_store");
    call("server/gcode_store", params({{"count", JsonValue(static_cast<int64_t>(10))}}));
}

// ---------------------------------------------------------------------------
// server/files/* — listing and metadata
// ---------------------------------------------------------------------------

TEST_F(FileEndpointsTest, FilesListDefaultAndRoots) {
    call("server/files/list");
    call("server/files/list", params({{"root", str("config")}}));
    call("server/files/list", params({{"root", str("logs")}}));
    call("server/files/roots");
}

TEST_F(FileEndpointsTest, FilesDirectoryAndMetadata) {
    call("server/files/directory", params({{"path", str("nonexistent-dir")}}));
    call("server/files/get_directory",
         params({{"path", str("nonexistent-dir")}}));
    call("server/files/metadata",
         params({{"filename", str("does-not-exist.gcode")}}));
}

TEST_F(FileEndpointsTest, FilesMetascanAndThumbnails) {
    call("server/files/metascan");
    call("server/files/thumbnails");
}

// ---------------------------------------------------------------------------
// server/files/* — mutating ops on nonexistent paths
// ---------------------------------------------------------------------------

TEST_F(FileEndpointsTest, FilesMoveCopyDeleteMissingPaths) {
    call("server/files/move",
         params({{"source", str("no-such-a.gcode")},
                 {"dest", str("no-such-b.gcode")}}));
    call("server/files/copy",
         params({{"source", str("no-such-a.gcode")},
                 {"dest", str("no-such-b.gcode")}}));
    call("server/files/delete",
         params({{"path", str("no-such-a.gcode")}}));
    call("server/files/delete_file",
         params({{"path", str("no-such-a.gcode")}}));
}

TEST_F(FileEndpointsTest, FilesUploadMissingParams) {
    call("server/files/upload");
    call("server/files/upload", empty());
}

TEST_F(FileEndpointsTest, FilesCreateDirWithoutPath) {
    call("server/files/create_dir");
    call("server/files/create_dir", params({{"path", str("")}}));
}

// ---------------------------------------------------------------------------
// Logs and config
// ---------------------------------------------------------------------------

TEST_F(FileEndpointsTest, ServerConfig) {
    call("server/config");
}

TEST_F(FileEndpointsTest, LogEndpoints) {
    call("server/logs/list");
    call("server/klippy_log");
    call("server/moonraker_log");
    call("server/logs/rollover");
}

}  // namespace
