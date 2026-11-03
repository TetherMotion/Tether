/**
 * @file test_klipper_e1_endpoints.cpp
 * @brief Coverage battery for the E1 endpoint handlers
 *        (KlippyServerE1Endpoints.cpp): access/auth, announcements,
 *        bots, notepad, database, devices, device_power, job_queue,
 *        job_history, machine services/update/peripherals, webcams,
 *        and the Moonraker-compat printer/* aliases — all through the
 *        public KlippyServer::callEndpoint() dispatch.
 */

#include <gtest/gtest.h>

#include <string>
#include <unistd.h>

#include "tether/klipper/klippy/KlippyServer.hpp"
#include "tether/klipper/klippy/JsonValue.hpp"

namespace {

using namespace tether::klipper::klippy;

static std::string uniqueSock() {
    return "/tmp/tether_test_e1ep_" + std::to_string(getpid()) + ".sock";
}

JsonValue params(std::map<std::string, JsonValue> m) {
    return JsonValue(std::move(m));
}
JsonValue str(const char* s) { return JsonValue(std::string(s)); }
JsonValue num(double v) { return JsonValue(v); }
JsonValue empty() { return params({}); }

class E1EndpointsTest : public ::testing::Test {
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
// access/* — auth lifecycle
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, AccessEndpoints) {
    call("access/api_key");
    call("access/login", params({{"username", str("admin")},
                                 {"password", str("pw")}}));
    call("access/user");
    call("access/oneshot_token");
    call("access/refresh_jwt", params({{"refresh_token", str("tok")}}));
    call("access/logout");
}

// ---------------------------------------------------------------------------
// announcements/*
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, AnnouncementsEndpoints) {
    call("announcements/list");
    call("announcements/feed", params({{"feed", str("moonraker")}}));
    call("announcements/update");
    call("announcements/dismiss", params({{"entry_id", str("e1")}}));
}

// ---------------------------------------------------------------------------
// bot/* and devices/* and notepad/*
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, BotDeviceNotepadEndpoints) {
    call("bot/list");
    call("bot/get", params({{"name", str("b1")}}));
    call("bot/update", params({{"name", str("b1")}, {"status", str("on")}}));
    call("bot/delete", params({{"name", str("b1")}}));

    call("devices/list");
    call("devices/get", params({{"name", str("d1")}}));
    call("devices/create", params({{"name", str("d1")}, {"type", str("gpio")}}));
    call("devices/delete", params({{"name", str("d1")}}));

    call("notepad/list");
    call("notepad/put", params({{"key", str("k1")}, {"value", str("v1")}}));
    call("notepad/get", params({{"key", str("k1")}}));
    call("notepad/delete", params({{"key", str("k1")}}));
}

// ---------------------------------------------------------------------------
// database/* and server/database/*
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, DatabaseEndpoints) {
    call("database/list");
    call("database/put", params({{"namespace", str("ns")},
                                {"key", str("k")}, {"value", num(7)}}));
    call("database/get", params({{"namespace", str("ns")}, {"key", str("k")}}));
    call("database/ns");
    call("database/delete", params({{"namespace", str("ns")},
                                   {"key", str("k")}}));

    call("server/database/list");
    call("server/database/post_item", params({{"namespace", str("ns")},
                                              {"key", str("k")}, {"value", num(1)}}));
    call("server/database/get_item", params({{"namespace", str("ns")},
                                             {"key", str("k")}}));
    call("server/database/get", params({{"namespace", str("ns")},
                                        {"key", str("k")}}));
    call("server/database/put", params({{"namespace", str("ns")},
                                        {"key", str("k")}, {"value", num(2)}}));
    call("server/database/delete", params({{"namespace", str("ns")},
                                           {"key", str("k")}}));
}

// ---------------------------------------------------------------------------
// job_queue/* and job_history/*
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, JobQueueAndHistoryEndpoints) {
    call("job_queue/status");
    call("job_queue/post_job",
         params({{"filenames", JsonValue(std::vector<JsonValue>{str("a.gcode")})}}));
    call("job_queue/start");
    call("job_queue/pause");
    call("job_queue/jump_to", params({{"job_id", num(0)}}));
    call("job_queue/delete_job", params({{"job_id", num(0)}}));

    call("job_history/list");
    call("job_history/get", params({{"uid", str("u1")}}));
    call("job_history/delete", params({{"uid", str("u1")}}));
}

// ---------------------------------------------------------------------------
// machine/* — power, services, update, peripherals
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, MachinePowerEndpoints) {
    call("machine/device_power/devices");
    call("machine/device_power/state", params({{"device", str("d1")}}));
    call("machine/device_power/on", params({{"device", str("d1")}}));
    call("machine/device_power/off", params({{"device", str("d1")}}));
    call("machine/device_power/toggle", params({{"device", str("d1")}}));
}

TEST_F(E1EndpointsTest, MachineServicesAndUpdateEndpoints) {
    call("machine/services/list");
    call("machine/services/start", params({{"service", str("svc")}}));
    call("machine/services/stop", params({{"service", str("svc")}}));
    call("machine/services/restart", params({{"service", str("svc")}}));

    call("machine/update/list");
    call("machine/update/status");
    call("machine/update/refresh");
    call("machine/update/update");
    call("machine/update/recover");
    call("machine/update/rollback");
    call("machine/update/client");
    call("machine/system_perms");
    call("machine/peripherals/serial");
    call("machine/peripherals/usb");
    call("machine/proc_stats");
    call("machine/procstats");
}

// ---------------------------------------------------------------------------
// webcams + server/webcams (both registered spellings)
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, WebcamEndpoints) {
    for (const char* prefix : {"webcams", "server/webcams"}) {
        call(std::string(prefix) + "/list");
        call(std::string(prefix) + "/update",
             params({{"name", str("cam1")}, {"location", str("bed")}}));
        call(std::string(prefix) + "/get", params({{"name", str("cam1")}}));
        call(std::string(prefix) + "/test", params({{"name", str("cam1")}}));
        call(std::string(prefix) + "/delete", params({{"name", str("cam1")}}));
    }
}

// ---------------------------------------------------------------------------
// printer/* Moonraker-alias paths + restart family
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, PrinterAliasAndRestartEndpoints) {
    call("printer/info");
    call("printer/objects/list");
    call("printer/objects/query", params({{"objects", empty()}}));
    call("printer/objects/subscribe", params({{"objects", empty()}}));
    call("printer/query_endstops/status");
    call("printer/gcode/help");
    call("printer/gcode/script", params({{"script", str("M114")}}));
    call("printer/gcode/subscribe_output");
    call("printer/start");
    call("printer/pause");
    call("printer/resume");
    call("printer/cancel");
    call("printer/print/start");
    call("printer/print/pause");
    call("printer/print/resume");
    call("printer/print/cancel");
    call("printer/emergency_stop");
    call("printer/restart");
    call("printer/firmware_restart");
    call("gcode/restart");
    call("gcode/firmware_restart");
    call("server/restart");
}

// ---------------------------------------------------------------------------
// server/files/* stragglers + logs
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, FilesAndLogEndpoints) {
    call("server/files/list");
    call("server/files/roots");
    call("server/files/directory", params({{"path", str("gcodes")}}));
    call("server/files/get_directory");
    call("server/files/metadata", params({{"filename", str("a.gcode")}}));
    call("server/files/metascan", params({{"filename", str("a.gcode")}}));
    call("server/files/thumbnails", params({{"filename", str("a.gcode")}}));
    call("server/files/create_dir", params({{"path", str("gcodes/newdir")}}));
    call("server/logs/list");
    call("server/logs/rollover");
    call("server/klippy_log");
    call("server/moonraker_log");
    call("server/config");
    call("server/gcode_store");
    call("server/temperature_store");
    call("server/info");
}

// ---------------------------------------------------------------------------
// spoolman + misc core endpoints left unexercised
// ---------------------------------------------------------------------------

TEST_F(E1EndpointsTest, SpoolmanAndCoreEndpoints) {
    call("spoolman/info");
    call("spoolman/spool_id", params({{"spool_id", num(3)}}));
    call("spoolman/proxy", params({{"path", str("/spool")}}));
    call("gcode/help");
    call("objects/list");
    call("objects/query", params({{"objects", empty()}}));
    call("objects/subscribe", params({{"objects", empty()}}));
    call("machine/system_info");
    call("server/files/list", params({{"root", str("gcodes")}}));
}

}  // namespace
