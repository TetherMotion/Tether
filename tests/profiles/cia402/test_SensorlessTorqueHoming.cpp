/**
 * @file test_SensorlessTorqueHoming.cpp
 * @brief Tests for EtherCAT::SensorlessTorqueHomingController
 *
 * The simulated plant reproduces the characteristics measured on a real
 * ANCTL AS715N drive (EtherCAT, 1 kHz cycle, CSV mode, 2026-09-15 traces):
 *
 *  - position_actual is a clean incremental counter (quantization only,
 *    plus optional small dither at standstill)
 *  - speed_feedback (0x606C) is very noisy: sigma ~1400 counts/s while
 *    moving at ~2000 counts/s and sigma ~1000 counts/s at standstill;
 *    ~2% of samples read |speed| < 100 while cruising at 2000 counts/s
 *  - torque_actual sits at ~30 permille while cruising freely and rises
 *    to the torque limit (~90 permille for a 100 permille limit) when the
 *    axis presses against a mechanical stop
 *  - in CSV mode the drive-internal velocity loop converges to the
 *    streamed target_velocity within a few milliseconds
 */

#include <gtest/gtest.h>

#include "tether/ethercat/Master.hpp"
#include "tether/profiles/cia402/CiA402Drive.hpp"
#include "tether/profiles/cia402/SensorlessTorqueHomingController.hpp"
#include "tether/drives/AS715N/AS715NPDO.hpp"
#include "tether/profiles/cia301/CiA402Defs.hpp"

#include <cmath>
#include <cstdint>
#include <random>

using namespace EtherCAT;
using namespace EtherCAT::Drives::AS715N_pdo;

namespace {

using HomingCtrl =
    SensorlessTorqueHomingController<AS715N_RxPDO_1702, AS715N_TxPDO_1B04>;

constexpr int8_t kOpModeCSV = CiA402::OperatingMode::CyclicSyncVelocity;
constexpr uint16_t kSwOpEnabled =
    static_cast<uint16_t>(CiA402::StatuswordBits::OperationEnabled) | 0x1630u;

/// First-order plant with a mechanical stop, reproducing the measured
/// AS715N behaviour described above.
struct HomingPlant {
    double pos = 100000.0;   ///< true position (counts)
    double vel = 0.0;        ///< true velocity (counts/s)
    int32_t stop = 200000;   ///< mechanical stop (positive direction)
    int16_t torque_limit = 100;   ///< permille the drive applies at the stop
    int16_t cruise_torque = 30;   ///< measured friction torque while moving
    int16_t stall_torque = 90;    ///< measured torque while pressing the stop
    int stop_dither = 0;          ///< +/- counts of position noise at rest
    std::mt19937 rng{42};
    std::normal_distribution<double> noise{0.0, 1200.0};
    std::uniform_int_distribution<int> dither{-2, 2};

    void step(int32_t target_velocity, double dt)
    {
        // Drive-internal velocity loop: ~8 ms first-order convergence.
        const double tau = 0.008;
        vel += (static_cast<double>(target_velocity) - vel) *
               std::min(1.0, dt / tau);
        pos += vel * dt;

        const bool pressing = vel > 0.0 && pos >= stop;
        if (pressing) {
            pos = static_cast<double>(stop);
            vel = 0.0;
        }
        torque = pressing ? stall_torque
                          : (vel > 0.0 ? cruise_torque : -cruise_torque);
    }

    int32_t positionActual()
    {
        const double p = pos + (stop_dither > 0 && vel == 0.0
                                    ? dither(rng) * stop_dither / 2
                                    : 0.0);
        return static_cast<int32_t>(std::lround(p));
    }

    int32_t speedFeedback()
    {
        return static_cast<int32_t>(std::lround(vel + noise(rng)));
    }

    int16_t torque = 0;
};

struct Fixture {
    EtherCAT::Master master{EtherCAT::Master::Config{}};
    CiA402Drive drive{master, 0};
    HomingPlant plant;

    Fixture()
    {
        drive.setPDOBufferSizes(RxPDO_1702.index, TxPDO_1B04.index,
                                RxPDO_1702.size, TxPDO_1B04.size);
        // Route setOperatingMode() into the RxPDO buffer so no SDO/backend
        // is needed.  0x1702 carries modes_of_operation at offset 12.
        drive.setOpmodePDOOffset(opmodeOffsetFor(RxPDO_1702.index));
    }

    void writeInputs()
    {
        auto* tx = const_cast<AS715N_TxPDO_1B04*>(
            drive.txPDO<AS715N_TxPDO_1B04>());
        tx->statusword = kSwOpEnabled;
        tx->modes_of_operation_display = kOpModeCSV;
        tx->position_actual = plant.positionActual();
        tx->speed_feedback = plant.speedFeedback();
        tx->torque_actual = plant.torque;
    }

    /// Run the controller against the plant for up to max_seconds.
    /// Returns the simulated time elapsed.
    double run(HomingCtrl& ctrl, double max_seconds)
    {
        constexpr double dt = 0.001;
        const int steps = static_cast<int>(max_seconds / dt);
        writeInputs();
        for (int i = 0; i < steps; ++i) {
            if (!ctrl.update(drive, dt)) {
                break;
            }
            const auto* rx = drive.rxPDO<AS715N_RxPDO_1702>();
            plant.step(rx->target_velocity, dt);
            writeInputs();
            if (ctrl.isHomed() || ctrl.hasFailed()) {
                // Let the controller observe the final state once more.
                ctrl.update(drive, dt);
                break;
            }
        }
        return max_seconds;
    }

    static HomingCtrl::Config baseConfig()
    {
        HomingCtrl::Config cfg;
        cfg.use_csv_mode = true;
        cfg.target_velocity = 10000.0;   // 5x the original 2000 counts/s
        cfg.direction = 1;
        cfg.max_torque_percent = 10.0;   // -> 100 permille limit
        cfg.stall_time = 0.5;
        cfg.phase_timeout = 0.0;         // disable watchdog; tests bound time
        return cfg;
    }
};

// ---------------------------------------------------------------------------
// Position-based stall detection (the default)
// ---------------------------------------------------------------------------

TEST(SensorlessTorqueHoming, PositionDetectionHomesAtStop)
{
    Fixture f;
    auto cfg = Fixture::baseConfig();
    cfg.stall_detection = HomingCtrl::StallDetection::Position;
    cfg.stall_window = 0.1;
    cfg.stall_position_counts = 20.0;
    HomingCtrl ctrl(cfg);

    ASSERT_TRUE(ctrl.start(f.drive));
    f.run(ctrl, 15.0);

    ASSERT_TRUE(ctrl.isHomed());
    ASSERT_FALSE(ctrl.hasFailed()) << ctrl.failureMessage();
    ASSERT_TRUE(ctrl.hasHomePosition());
    // At the stop the position is stationary; the 250-sample average must
    // land within a couple of counts of the true stop position.
    EXPECT_NEAR(ctrl.homePosition(), f.plant.stop, 3);
}

TEST(SensorlessTorqueHoming, PositionDetectionNotFooledDuringCruise)
{
    // While cruising, the windowed position delta must stay well above the
    // stall threshold — measured: 185 +/- 63 counts per 100 ms at
    // 2000 counts/s; at 10000 counts/s it is ~1000 counts.
    Fixture f;
    auto cfg = Fixture::baseConfig();
    cfg.stall_detection = HomingCtrl::StallDetection::Position;
    cfg.stall_position_counts = 20.0;
    cfg.phase_timeout = 5.0;  // fails if a stall is never detected
    f.plant.stop = 10'000'000;  // effectively unreachable
    HomingCtrl ctrl(cfg);

    ASSERT_TRUE(ctrl.start(f.drive));
    f.run(ctrl, 10.0);

    EXPECT_FALSE(ctrl.isHomed());
    EXPECT_TRUE(ctrl.hasFailed());
    EXPECT_STREQ(ctrl.failureMessage(), "Homing phase timed out — no progress");
}

// ---------------------------------------------------------------------------
// Speed-based stall detection fails with this drive's noisy 0x606C
// ---------------------------------------------------------------------------

TEST(SensorlessTorqueHoming, SpeedDetectionCannotSeeTheStop)
{
    // Measured: at standstill speed_feedback still jitters ~+/-1000 counts/s,
    // so |speed| <= 100 (stall_velocity) essentially never holds for the
    // required stall_time.  Position detection succeeds in the same setup.
    Fixture f;
    auto cfg = Fixture::baseConfig();
    cfg.stall_detection = HomingCtrl::StallDetection::Speed;
    cfg.stall_velocity = 100.0;
    cfg.stall_time = 0.5;
    HomingCtrl ctrl(cfg);

    ASSERT_TRUE(ctrl.start(f.drive));
    f.run(ctrl, 15.0);  // ample time at the stop

    EXPECT_FALSE(ctrl.isHomed());
    EXPECT_FALSE(ctrl.hasFailed());  // it just waits forever — silent hang
}

// ---------------------------------------------------------------------------
// Torque-based stall detection
// ---------------------------------------------------------------------------

TEST(SensorlessTorqueHoming, TorqueDetectionHomesAtStop)
{
    Fixture f;
    auto cfg = Fixture::baseConfig();
    cfg.stall_detection = HomingCtrl::StallDetection::Torque;
    // threshold defaults to 80% of max_permille (100) = 80;
    // plant presses with 90 permille at the stop.
    HomingCtrl ctrl(cfg);

    ASSERT_TRUE(ctrl.start(f.drive));
    f.run(ctrl, 15.0);

    ASSERT_TRUE(ctrl.isHomed());
    EXPECT_NEAR(ctrl.homePosition(), f.plant.stop, 3);
}

// ---------------------------------------------------------------------------
// Home position averaging
// ---------------------------------------------------------------------------

TEST(SensorlessTorqueHoming, HomePositionAveragingReducesDitherError)
{
    // With +/-2 counts of position dither at standstill, the averaged
    // estimate must be tighter than a single sample can guarantee.
    for (uint32_t avg_samples : {1u, 250u}) {
        Fixture f;
        auto cfg = Fixture::baseConfig();
        cfg.stall_detection = HomingCtrl::StallDetection::Position;
        cfg.home_position_avg_samples = avg_samples;
        f.plant.stop_dither = 2;
        HomingCtrl ctrl(cfg);

        ASSERT_TRUE(ctrl.start(f.drive));
        f.run(ctrl, 15.0);
        ASSERT_TRUE(ctrl.isHomed()) << "avg_samples=" << avg_samples;
        if (avg_samples >= 250) {
            EXPECT_NEAR(ctrl.homePosition(), f.plant.stop, 1);
        } else {
            EXPECT_NEAR(ctrl.homePosition(), f.plant.stop, 3);
        }
    }
}

// ---------------------------------------------------------------------------
// Multi-pass fine homing
// ---------------------------------------------------------------------------

TEST(SensorlessTorqueHoming, MultiPassHomingBacksOffAndReapproaches)
{
    Fixture f;
    auto cfg = Fixture::baseConfig();
    cfg.stall_detection = HomingCtrl::StallDetection::Position;
    cfg.fine_homing_passes = 2;
    cfg.backoff_distance = 5000.0;
    cfg.fine_velocity = 5000.0;
    HomingCtrl ctrl(cfg);

    ASSERT_TRUE(ctrl.start(f.drive));
    f.run(ctrl, 30.0);

    ASSERT_TRUE(ctrl.isHomed());
    // Both passes record the same stop; mean of the two averages ~= stop.
    EXPECT_NEAR(ctrl.homePosition(), f.plant.stop, 3);
}

} // namespace
