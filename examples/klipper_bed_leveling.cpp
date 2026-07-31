/**
 * @file klipper_bed_leveling.cpp
 * @brief Example: bed leveling workflows (bed mesh, screws tilt, Z tilt, QGL).
 *
 * @details
 * This example demonstrates the complete bed leveling workflow using
 * Tether's Klipper compatibility layer:
 *
 *   1. Set up a printer with probe and heaters
 *   2. BED_MESH_CALIBRATE — probe the bed and build a mesh
 *   3. BED_MESH_PROFILE — save and load mesh profiles
 *   4. BED_MESH_OFFSET — apply XY offsets to the mesh
 *   5. BED_MESH_MAP — output the mesh as a map
 *   6. BED_MESH_CLEAR — clear the mesh
 *   7. SCREWS_TILT_ADJUST — adjust bed screws using probe
 *   8. Z_TILT_ADJUST — adjust Z tilt on multi-Z-axis printers
 *   9. QUAD_GANTRY_LEVEL — level a gantry at 4 points
 *  10. BED_SCREWS_ADJUST — manually adjust bed screws
 *  11. PROBE_ACCURACY — test probe repeatability
 *  12. PROBE_CALIBRATE — calibrate the probe Z offset
 *  13. DELTA_CALIBRATE — calibrate delta printer geometry
 *  14. SET_GCODE_OFFSET — apply a temporary Z offset
 *  15. Save and restore state for pause/resume
 *
 * This example is particularly useful for understanding how the bed
 * leveling commands interact and how to build a complete leveling workflow.
 */

#include "tether/klipper/klippy/KlippyInstance.hpp"
#include "tether/klipper/objects/Thermal.hpp"
#include "tether/klipper/objects/Peripherals.hpp"
#include "tether/klipper/objects/Homing.hpp"

#include <cstdio>
#include <memory>

using namespace tether::klipper;
using namespace tether::klipper::klippy;
using namespace tether::klipper::objects;

static void execAndPrint(KlippyInstance& inst, const std::string& gcode) {
    std::printf("  > %s\n", gcode.c_str());
    inst.executeGcode(gcode);
}

static void printSection(const char* title) {
    std::printf("\n--- %s ---\n\n", title);
}

int main() {
    KlippyInstanceConfig cfg;
    cfg.udsConfig.socketPath = "/tmp/tether_bed_leveling_uds";
    cfg.sdcardDir = "/tmp/tether_bed_leveling_sd";
    cfg.firmwareVersion = "tether-bed-level-1.0";
    KlippyInstance inst(cfg);

    // Wire up heaters
    Thermistor::Params thermParams;
    auto bedSensor = std::make_shared<Thermistor>(1, thermParams, []() { return 4095.0 * 0.5; });
    auto bedHeater = std::make_shared<Heater>(1, [](double) {}, [&]() { return bedSensor->read(); });
    inst.setHeaterBed(bedHeater);

    // Wire up probe
    auto probe = std::make_shared<Probe>(3, []() { return false; });
    inst.setProbe(probe);

    // Wire up endstops
    inst.setEndstop("x", std::make_shared<Endstop>(4, []() { return false; }));
    inst.setEndstop("y", std::make_shared<Endstop>(5, []() { return false; }));
    inst.setEndstop("z", std::make_shared<Endstop>(6, []() { return false; }));

    std::printf("Tether Klipper Bed Leveling Workflow\n");
    std::printf("======================================\n");

    // ── 1. Home the printer ────────────────────────────────────────────
    printSection("1. Homing");
    execAndPrint(inst, "G28");                    // Home all axes
    execAndPrint(inst, "M114");                   // Check position

    // ── 2. Heat the bed for accurate probing ───────────────────────────
    printSection("2. Heat Bed");
    execAndPrint(inst, "M140 S60");               // Set bed to 60C
    execAndPrint(inst, "M190 S60");               // Wait for bed temp
    std::printf("  Bed target: %.1f C\n", bedHeater->target());

    // ── 3. Probe accuracy test ─────────────────────────────────────────
    printSection("3. Probe Accuracy Test");
    execAndPrint(inst, "PROBE_ACCURACY SAMPLES=10 PROBE_SPEED=5.0");
    execAndPrint(inst, "QUERY_PROBE");            // Check probe state

    // ── 4. Bed mesh calibration ────────────────────────────────────────
    printSection("4. Bed Mesh Calibration");
    execAndPrint(inst, "BED_MESH_CALIBRATE");     // Probe and build mesh
    execAndPrint(inst, "BED_MESH_OUTPUT");        // Print mesh data
    execAndPrint(inst, "BED_MESH_MAP");           // Output mesh map

    // ── 5. Save and load mesh profiles ─────────────────────────────────
    printSection("5. Mesh Profile Management");
    execAndPrint(inst, "BED_MESH_PROFILE SAVE=pla_60c");   // Save profile
    execAndPrint(inst, "BED_MESH_CLEAR");                  // Clear current mesh
    execAndPrint(inst, "BED_MESH_PROFILE LOAD=pla_60c");   // Load saved profile

    // ── 6. Apply mesh offset ───────────────────────────────────────────
    printSection("6. Mesh Offset");
    execAndPrint(inst, "BED_MESH_OFFSET X=2.5 Y=3.0");    // Apply offset
    execAndPrint(inst, "BED_MESH_OFFSET X=-1.0 Y=0.0");   // Adjust offset

    // ── 7. Screws tilt adjust ──────────────────────────────────────────
    printSection("7. Screws Tilt Adjust");
    execAndPrint(inst, "SCREWS_TILT_ADJUST");     // Auto-adjust screws

    // ── 8. Z tilt adjust ───────────────────────────────────────────────
    printSection("8. Z Tilt Adjust");
    execAndPrint(inst, "Z_TILT_ADJUST");          // Level Z axis

    // ── 9. Quad gantry level ───────────────────────────────────────────
    printSection("9. Quad Gantry Level");
    execAndPrint(inst, "QUAD_GANTRY_LEVEL");      // Level gantry

    // ── 10. Bed screws manual adjust ───────────────────────────────────
    printSection("10. Bed Screws Adjust");
    execAndPrint(inst, "BED_SCREWS_ADJUST");      // Manual screw adjustment

    // ── 11. Probe calibration ──────────────────────────────────────────
    printSection("11. Probe Calibration");
    execAndPrint(inst, "PROBE_CALIBRATE");        // Calibrate probe Z offset
    execAndPrint(inst, "Z_OFFSET_APPLY_PROBE");   // Apply the offset
    execAndPrint(inst, "Z_OFFSET_APPLY_ENDSTOP"); // Apply to endstop

    // ── 12. Delta calibration ──────────────────────────────────────────
    printSection("12. Delta Calibration");
    execAndPrint(inst, "DELTA_CALIBRATE");        // Calibrate delta geometry
    execAndPrint(inst, "DELTA_ANALYZE CALIBRATE_RADIUS=100"); // Analyze delta

    // ── 13. G-code offset for first layer ──────────────────────────────
    printSection("13. G-code Offset (First Layer Adjustment)");
    execAndPrint(inst, "SET_GCODE_OFFSET Z=0.05 ADJUST=1");  // Babyskin +0.05mm
    execAndPrint(inst, "SET_GCODE_OFFSET Z=-0.02 ADJUST=1"); // Adjust down

    // ── 14. Save state for pause/resume ────────────────────────────────
    printSection("14. State Save/Restore for Pause/Resume");
    execAndPrint(inst, "SAVE_GCODE_STATE NAME=printing_state");
    execAndPrint(inst, "SET_GCODE_OFFSET Z=0.1 ADJUST=1");  // Z hop for pause
    execAndPrint(inst, "G1 Z10 F3000");                     // Lift Z
    execAndPrint(inst, "G1 X0 Y0 F6000");                   // Park toolhead
    // ... pause activity would happen here ...
    execAndPrint(inst, "RESTORE_GCODE_STATE NAME=printing_state"); // Resume

    // ── 15. Save config ────────────────────────────────────────────────
    printSection("15. Save Configuration");
    execAndPrint(inst, "SAVE_CONFIG");            // Save all calibration

    // ── Summary ────────────────────────────────────────────────────────
    printSection("Summary");
    std::printf("Bed leveling workflow completed.\n");
    std::printf("Commands demonstrated:\n");
    std::printf("  - BED_MESH_CALIBRATE/OUTPUT/MAP/CLEAR/PROFILE/OFFSET\n");
    std::printf("  - SCREWS_TILT_ADJUST, Z_TILT_ADJUST, QUAD_GANTRY_LEVEL\n");
    std::printf("  - BED_SCREWS_ADJUST, DELTA_CALIBRATE, DELTA_ANALYZE\n");
    std::printf("  - PROBE, PROBE_ACCURACY, PROBE_CALIBRATE\n");
    std::printf("  - Z_OFFSET_APPLY_PROBE/ENDSTOP\n");
    std::printf("  - SET_GCODE_OFFSET, SAVE/RESTORE_GCODE_STATE\n");
    std::printf("  - SAVE_CONFIG\n");

    std::printf("\nDone\n");
    return 0;
}
