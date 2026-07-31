/**
 * @file klipper_moonraker_api.cpp
 * @brief Example: Moonraker-compatible API endpoints via KlippyUdsServer.
 *
 * @details
 * This example demonstrates the full Moonraker API surface exposed by
 * Tether's KlippyUdsServer. It covers:
 *
 *   1. Server info and configuration
 *   2. File operations (list, metadata, roots)
 *   3. Temperature store
 *   4. G-code store
 *   5. Database operations (put/get/delete/list namespaces)
 *   6. Job queue (add/status/pause/start/jump/delete)
 *   7. Job history (add/list/get/delete)
 *   8. Announcements (add/list/feed/dismiss)
 *   9. Webcams (register/list/get/test/update/delete)
 *  10. Power devices (register/list/on/off/toggle)
 *  11. Machine services (register/list/restart/stop/start)
 *  12. Machine update (status/list/refresh)
 *  13. System info and procstats
 *  14. System permissions
 *  15. Access control (users/login/api_key/oneshot_token)
 *  16. Bot management (register/list/get/update/delete)
 *  17. Notepad (put/get/list/delete)
 *  18. Spoolman integration (info/spool_id/proxy)
 *  19. Log files (list/rollover)
 *  20. Device CRUD (create/delete)
 *
 * This example shows how to programmatically interact with the UDS server
 * without a socket connection, using the public C++ API directly.
 */

#include "tether/klipper/klippy/KlippyUdsServer.hpp"

#include <cstdio>
#include <string>

using namespace tether::klipper::klippy;

static void printSection(const char* title) {
    std::printf("\n--- %s ---\n\n", title);
}

static void printEndpoints(KlippyUdsServer& server, const std::string& prefix) {
    auto endpoints = server.listEndpoints();
    int count = 0;
    for (const auto& ep : endpoints) {
        if (ep.substr(0, prefix.size()) == prefix) {
            std::printf("  %s\n", ep.c_str());
            ++count;
        }
    }
    std::printf("  (%d endpoints)\n", count);
}

int main() {
    UdsServerConfig cfg;
    cfg.socketPath = "/tmp/tether_moonraker_api_uds";
    cfg.logFile = "/tmp/tether_moonraker_api.log";
    KlippyUdsServer server(cfg);

    std::printf("Tether Klipper Moonraker API Reference\n");
    std::printf("========================================\n");
    std::printf("Socket: %s\n", cfg.socketPath.c_str());

    // ── 1. Server Info ─────────────────────────────────────────────────
    printSection("1. Server Info & Configuration");
    std::printf("Registered endpoints (total: %zu)\n", server.listEndpoints().size());
    printEndpoints(server, "server/");

    // ── 2. File Operations ─────────────────────────────────────────────
    printSection("2. File Operations");
    server.registerFileRoot("gcodes", "/tmp/gcodes", true);
    server.registerFileRoot("config", "/tmp/config", true);
    server.registerFileRoot("logs", "/tmp/logs", false);
    std::printf("Registered file roots: gcodes, config, logs\n");

    // ── 3. Temperature Store ───────────────────────────────────────────
    printSection("3. Temperature Store");
    server.recordTemperature("extruder", 25.0, 200.0);
    server.recordTemperature("extruder", 30.0, 200.0);
    server.recordTemperature("heater_bed", 60.0, 60.0);
    std::printf("Recorded temperatures for extruder and heater_bed\n");

    // ── 4. G-code Store ────────────────────────────────────────────────
    printSection("4. G-code Store");
    server.emitGcodeResponse("// Starting print...");
    server.emitGcodeResponse("// Layer 1 of 100");
    server.emitGcodeResponse("// Layer 50 of 100");
    std::printf("Emitted 3 G-code responses\n");

    // ── 5. Database Operations ─────────────────────────────────────────
    printSection("5. Database Operations");
    server.databasePut("moonraker", "temp_setting", JsonValue(42));
    server.databasePut("moonraker", "string_setting", JsonValue("hello"));
    server.databasePut("tether", "custom_key", JsonValue(true));

    auto val = server.databaseGet("moonraker", "temp_setting");
    std::printf("Get 'moonraker/temp_setting': %s\n",
                val.has_value() ? std::to_string(val->asInt()).c_str() : "not found");

    val = server.databaseGet("moonraker", "string_setting");
    std::printf("Get 'moonraker/string_setting': %s\n",
                val.has_value() ? val->asString().c_str() : "not found");

    server.databaseDelete("moonraker", "temp_setting");
    val = server.databaseGet("moonraker", "temp_setting");
    std::printf("After delete, Get 'moonraker/temp_setting': %s\n",
                val.has_value() ? "still exists" : "deleted");
    std::printf("Database namespaces: moonraker, tether\n");

    // ── 6. Job Queue ───────────────────────────────────────────────────
    printSection("6. Job Queue");
    server.jobQueueAdd("print_part1.gcode");
    server.jobQueueAdd("print_part2.gcode");
    server.jobQueueAdd("print_part3.gcode");
    std::printf("Added 3 jobs to queue\n");
    printEndpoints(server, "job_queue/");

    // ── 7. Job History ─────────────────────────────────────────────────
    printSection("7. Job History");
    int64_t job1 = server.jobHistoryAdd("test_print.gcode", "completed");
    int64_t job2 = server.jobHistoryAdd("failed_print.gcode", "error");
    int64_t job3 = server.jobHistoryAdd("cancelled_print.gcode", "cancelled");
    std::printf("Added 3 job history entries (IDs: %ld, %ld, %ld)\n",
                (long)job1, (long)job2, (long)job3);
    printEndpoints(server, "job_history/");

    // ── 8. Announcements ───────────────────────────────────────────────
    printSection("8. Announcements");
    server.announcementAdd("update-001", "Update Available",
                            "New firmware version 2.0", "info");
    server.announcementAdd("warn-001", "High Temperature",
                            "Extruder temperature above 250C", "warning");
    std::printf("Added 2 announcements\n");
    printEndpoints(server, "announcements/");

    // ── 9. Webcams ─────────────────────────────────────────────────────
    printSection("9. Webcams");
    server.registerWebcam("webcam1", "http://localhost:8080/?action=stream", "mjpegstreamer");
    server.registerWebcam("webcam2", "http://localhost:8081/?action=stream", "mjpegstreamer");
    server.registerWebcam("webcam3", "rtsp://localhost:8554/stream", "ipstreamer");
    std::printf("Registered 3 webcams\n");
    printEndpoints(server, "webcams/");

    // ── 10. Power Devices ──────────────────────────────────────────────
    printSection("10. Power Devices");
    server.registerPowerDevice("psu", "on");
    server.registerPowerDevice("light", "off");
    server.registerPowerDevice("enclosure_fan", "on");
    std::printf("Registered 3 power devices\n");
    printEndpoints(server, "machine/device_power/");

    // ── 11. Machine Services ───────────────────────────────────────────
    printSection("11. Machine Services");
    server.registerService("klipper", "active", "running");
    server.registerService("moonraker", "active", "running");
    server.registerService("webcamd", "active", "running");
    server.registerService("klipper-mcu", "active", "running");
    std::printf("Registered 4 services\n");
    printEndpoints(server, "machine/services/");

    // ── 12. Machine Update ─────────────────────────────────────────────
    printSection("12. Machine Update");
    server.setUpdateStatus("klipper", "current");
    server.setUpdateStatus("moonraker", "current");
    server.setUpdateStatus("system", "update_available");
    std::printf("Set update status for 3 components\n");
    printEndpoints(server, "machine/update/");

    // ── 13. System Info ────────────────────────────────────────────────
    printSection("13. System Info");
    printEndpoints(server, "machine/");

    // ── 14. System Permissions ─────────────────────────────────────────
    printSection("14. System Permissions");
    server.setSystemPerms("read", {"read"});
    server.setSystemPerms("write", {"write"});
    server.setSystemPerms("admin", {"read", "write", "admin"});
    std::printf("Set permissions for read, write, admin roles\n");

    // ── 15. Access Control ─────────────────────────────────────────────
    printSection("15. Access Control");
    server.registerUser("admin", "password123", {"read", "write", "admin"});
    server.registerUser("operator", "op456", {"read", "write"});
    server.registerUser("viewer", "view789", {"read"});
    std::printf("Registered 3 users: admin, operator, viewer\n");
    printEndpoints(server, "access/");

    // ── 16. Bot Management ─────────────────────────────────────────────
    printSection("16. Bot Management");
    server.registerBot("telegram_bot", "telegram", "bot_token_123", true);
    server.registerBot("discord_bot", "discord", "discord_token_456", false);
    std::printf("Registered 2 bots: telegram_bot, discord_bot\n");
    printEndpoints(server, "bot/");

    // ── 17. Notepad ────────────────────────────────────────────────────
    printSection("17. Notepad");
    server.notepadPut("print_notes", "First layer at 0.2mm, 60C bed");
    server.notepadPut("filament_log", "PLA Black, 1.75mm, 220C");
    server.notepadPut("maintenance", "Clean bed after every 5 prints");

    auto note = server.notepadGet("filament_log");
    std::printf("Notepad 'filament_log': %s\n", note.has_value() ? note->c_str() : "not found");
    printEndpoints(server, "notepad/");

    // ── 18. Spoolman ───────────────────────────────────────────────────
    printSection("18. Spoolman Integration");
    server.setSpoolmanConnected(true, "http://localhost:8000");
    server.setSpoolId(42);
    std::printf("Spoolman connected, current spool ID: 42\n");
    printEndpoints(server, "spoolman/");

    // ── 19. Log Files ──────────────────────────────────────────────────
    printSection("19. Log Files");
    server.addLogFile("klippy.log", "/tmp/klippy.log");
    server.addLogFile("moonraker.log", "/tmp/moonraker.log");
    std::printf("Added 2 log files\n");
    printEndpoints(server, "server/logs/");

    // ── 20. Device CRUD ────────────────────────────────────────────────
    printSection("20. Device CRUD");
    printEndpoints(server, "devices/");

    // ── Summary ────────────────────────────────────────────────────────
    printSection("Summary");
    std::printf("Total registered endpoints: %zu\n", server.listEndpoints().size());
    std::printf("\nEndpoint groups:\n");
    std::printf("  server/*        — info, config, files, logs, temperature, gcode\n");
    std::printf("  printer/*       — info, objects, gcode, print control\n");
    std::printf("  machine/*       — system, services, update, device_power, reboot\n");
    std::printf("  database/*      — list, get, put, delete, ns\n");
    std::printf("  job_queue/*     — status, post, delete, pause, start, jump_to\n");
    std::printf("  job_history/*   — list, get, delete\n");
    std::printf("  announcements/* — list, update, dismiss, feed\n");
    std::printf("  webcams/*       — list, get, test, update, delete\n");
    std::printf("  devices/*       — list, get, create, delete\n");
    std::printf("  access/*        — login, logout, user, refresh_jwt, api_key, oneshot\n");
    std::printf("  bot/*           — list, get, update, delete\n");
    std::printf("  notepad/*       — list, get, put, delete\n");
    std::printf("  spoolman/*      — info, spool_id, proxy\n");

    std::printf("\nDone\n");
    return 0;
}
