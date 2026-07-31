/**
 * @file klipper_delta_kinematics.cpp
 * @brief Example: delta printer kinematics and geometry configuration.
 *
 * @details
 * Demonstrates the DeltaPrinter kinematics class:
 *   1. Create a DeltaPrinter with custom geometry.
 *   2. Convert Cartesian (X, Y, Z) to tower positions (A, B, C).
 *   3. Convert tower positions back to Cartesian (inverse kinematics).
 *   4. Set geometry via M665 G-code parameters.
 *   5. Set endstop adjustments via M666 G-code parameters.
 */

#include "tether/klipper/klippy/AdvancedObjects.hpp"

#include <cstdio>
#include <cmath>

using namespace tether::klipper::klippy;

int main() {
    // 1. Create a DeltaPrinter with custom geometry
    DeltaPrinter delta;
    DeltaGeometry geo;
    geo.armLength = 270.0;       // 270mm arms
    geo.deltaRadius = 130.0;     // 130mm delta radius
    geo.towerAngleA = 0.0;
    geo.towerAngleB = 0.0;
    geo.towerAngleC = 0.0;
    delta.setGeometry(geo);
    std::printf("Delta geometry: arm=%.1fmm radius=%.1fmm\n",
                delta.geometry().armLength, delta.geometry().deltaRadius);

    // 2. Cartesian to tower conversion (forward kinematics)
    double x = 0.0, y = 0.0, z = 100.0;
    auto towers = delta.cartesianToTower(x, y, z);
    std::printf("\nForward: (%.1f, %.1f, %.1f) -> towers A=%.3f B=%.3f C=%.3f\n",
                x, y, z, towers[0], towers[1], towers[2]);

    // Try an off-center position
    x = 30.0; y = 20.0; z = 150.0;
    towers = delta.cartesianToTower(x, y, z);
    std::printf("Forward: (%.1f, %.1f, %.1f) -> towers A=%.3f B=%.3f C=%.3f\n",
                x, y, z, towers[0], towers[1], towers[2]);

    // 3. Tower to Cartesian (inverse kinematics) — round-trip check
    auto cart = delta.towerToCartesian(towers[0], towers[1], towers[2]);
    std::printf("Inverse: towers A=%.3f B=%.3f C=%.3f -> (%.4f, %.4f, %.4f)\n",
                towers[0], towers[1], towers[2], cart[0], cart[1], cart[2]);
    std::printf("Round-trip error: dx=%.6f dy=%.6f dz=%.6f\n",
                cart[0] - x, cart[1] - y, cart[2] - z);

    // 4. Set geometry via M665-like parameters (simulated)
    std::printf("\n--- M665: Update geometry ---\n");
    DeltaGeometry newGeo = delta.geometry();
    newGeo.armLength = 250.0;
    newGeo.deltaRadius = 125.0;
    newGeo.towerAngleA = 1.0;  // Small tower angle corrections
    newGeo.towerAngleB = -0.5;
    newGeo.towerAngleC = 0.3;
    delta.setGeometry(newGeo);
    std::printf("Updated: arm=%.1fmm radius=%.1fmm angles A=%.1f B=%.1f C=%.1f\n",
                delta.geometry().armLength, delta.geometry().deltaRadius,
                delta.geometry().towerAngleA, delta.geometry().towerAngleB,
                delta.geometry().towerAngleC);

    // Recompute towers with new geometry
    towers = delta.cartesianToTower(0.0, 0.0, 100.0);
    std::printf("New forward (0,0,100): A=%.3f B=%.3f C=%.3f\n",
                towers[0], towers[1], towers[2]);

    // 5. Set endstop adjustments via M666-like parameters (simulated)
    std::printf("\n--- M666: Endstop adjustments ---\n");
    DeltaEndstopAdjust adj;
    adj.adjX = 0.05;   // 0.05mm X tower endstop adjustment
    adj.adjY = -0.03;  // -0.03mm Y tower endstop adjustment
    adj.adjZ = 0.02;   // 0.02mm Z tower endstop adjustment
    delta.setEndstopAdjust(adj);
    std::printf("Endstop adjust: X=%.3f Y=%.3f Z=%.3f\n",
                delta.endstopAdjust().adjX,
                delta.endstopAdjust().adjY,
                delta.endstopAdjust().adjZ);

    // Show effect of endstop adjustment on tower positions
    auto towersAdj = delta.cartesianToTower(0.0, 0.0, 100.0);
    std::printf("With adjustment (0,0,100): A=%.3f B=%.3f C=%.3f\n",
                towersAdj[0], towersAdj[1], towersAdj[2]);

    std::printf("\nDone\n");
    return 0;
}
