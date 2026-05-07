#include <iostream>
#include <vector>
#include <optional>
#include <tether/motion_planner/SCurveProfile.hpp>
#include <tether/motion_planner/MotionSegment.hpp>
#include <tether/motion_planner/GCodeAdapter.hpp>

using namespace MotionPlanner;

#if defined(DEBUG_SCURVE_STANDALONE)
int main() {
    SCurveConstraintsD constraints;
    constraints.maxVelocity = 100.0;
    constraints.maxAcceleration = 500.0;
    constraints.maxJerk = 5000.0;

    SCurveProfileD profile;
    bool ok = profile.compute(1.0, 0.0, 0.0, constraints);

    // Recompute some internals using standalone functions (copied from SCurveProfile)
    auto computeAccelDistance = [](double v0, double v1, double aMax, double jMax) {
        if (v1 <= v0) return 0.0;
        double tJerk = aMax / jMax;
        double deltaV = v1 - v0;
        double deltaVJerk = 0.5 * jMax * tJerk * tJerk;
        if (deltaV <= 2 * deltaVJerk) {
            double t = std::sqrt(deltaV / jMax);
            return v0 * 2 * t + (2.0/3.0) * jMax * t * t * t;
        }
        double tConst = (deltaV - 2 * deltaVJerk) / aMax;
        double d1 = v0 * tJerk + (1.0/6.0) * jMax * tJerk * tJerk * tJerk;
        double v1_temp = v0 + deltaVJerk;
        double d2 = v1_temp * tConst + 0.5 * aMax * tConst * tConst;
        double v2_temp = v1_temp + aMax * tConst;
        double d3 = v2_temp * tJerk + 0.5 * aMax * tJerk * tJerk - (1.0/6.0) * jMax * tJerk * tJerk * tJerk;
        return d1 + d2 + d3;
    };

    auto computeCruiseVelocity = [&](double distance, double v0, double vf, double vMax, double aMax, double jMax){
        double tJerk = aMax / jMax;
        double accelDist = computeAccelDistance(v0, vMax, aMax, jMax);
        double decelDist = computeAccelDistance(vf, vMax, aMax, jMax);
        double minDist = accelDist + decelDist;
        if (distance >= minDist) return vMax;
        double vLow = std::max(v0, vf);
        double vHigh = vMax;
        for (int iter = 0; iter < 50; ++iter) {
            double vMid = 0.5 * (vLow + vHigh);
            double accel = computeAccelDistance(v0, vMid, aMax, jMax);
            double decel = computeAccelDistance(vf, vMid, aMax, jMax);
            double needed = accel + decel;
            std::cout << "iter " << iter << " vLow=" << vLow << " vHigh=" << vHigh << " vMid=" << vMid << " needed=" << needed << "\n";
            if (std::abs(needed - distance) < 1e-9) return vMid;
            if (needed < distance) vLow = vMid; else vHigh = vMid;
        }
        return 0.5 * (vLow + vHigh);
    };

    double vCruise = computeCruiseVelocity(1.0, 0.0, 0.0, constraints.maxVelocity, constraints.maxAcceleration, constraints.maxJerk);
    std::cout << "vCruise: " << vCruise << "\n";

    std::cout << "compute ok: " << ok << "\n";
    std::cout << "totalDistance: " << profile.totalDistance() << "\n";
    std::cout << "totalDuration: " << profile.totalDuration() << "\n";
    for (size_t i = 0; i < 7; ++i) {
        auto p = profile.phase(i);
        std::cout << "phase " << i << ": duration=" << p.duration
                  << " startVel=" << p.startVelocity << " startAcc=" << p.startAccel
                  << " jerk=" << p.jerk << " endPos=" << p.endPosition() << "\n";
    }

    auto finalPos = profile.phase(6).endPosition();
    std::cout << "finalPos: " << finalPos << " error: " << std::abs(finalPos - profile.totalDistance()) << "\n";
    // Test dwell behavior
    std::array<double, 9> pos{};
    pos[0] = 1.0; pos[1] = 2.0;
    auto seg = MotionPlanner::MotionSegment::dwell(pos, 2.5);
    std::cout << "seg.type=" << static_cast<int>(seg.type) << " dwellTime=" << seg.dwellTime << " dwellDuration=" << seg.dwellDuration << "\n";

    // Build the full integration test scenario
    auto file = std::make_shared<MotionPlanner::SourceFile>("trace_test.gcode");
    std::vector<MotionPlanner::ParsedGCodeCommand> commands;

    MotionPlanner::ParsedGCodeCommand cmd1;
    cmd1.lineNumber = 1;
    cmd1.gCode = 0;
    cmd1.coordinates[0] = 10.0;
    commands.push_back(cmd1);

    MotionPlanner::ParsedGCodeCommand cmd2;
    cmd2.lineNumber = 2;
    cmd2.gCode = 1;
    cmd2.coordinates[1] = 10.0;
    cmd2.feedRate = 100.0;
    commands.push_back(cmd2);

    MotionPlanner::ParsedGCodeCommand cmd3;
    cmd3.lineNumber = 3;
    cmd3.gCode = 1;
    cmd3.coordinates[0] = 0.0;
    cmd3.coordinates[1] = 0.0;
    commands.push_back(cmd3);

    // Build equivalent segments manually
    MotionPlanner::MotionSegmentList segs;
    std::array<double, 9> p0{}, p1{}, p2{}, p3{};
    p1[0] = 10.0; p2[1] = 10.0; p3[0] = 0.0; p3[1] = 0.0;

    segs.append(MotionPlanner::MotionSegment::rapid(p0, p1));
    segs.append(MotionPlanner::MotionSegment::linear(p1, p2, 100.0));
    segs.append(MotionPlanner::MotionSegment::linear(p2, p3, 100.0));

    MotionPlanner::MotionPlanBuilder2D builder2d;
    auto plan2 = builder2d.build(segs, 100.0);

    std::cout << "path start: " << plan2.path().startPoint()[0] << ", " << plan2.path().startPoint()[1] << "\n";

    auto state0 = plan2.evaluateAt(0.0);
    std::cout << "state0.position[0]=" << state0.position[0] << " position[1]=" << state0.position[1] << "\n";

    auto stateEnd = plan2.evaluateAt(plan2.totalDuration());
    std::cout << "end.position[0]=" << stateEnd.position[0] << " position[1]=" << stateEnd.position[1] << "\n";

    return 0;
}
#endif
