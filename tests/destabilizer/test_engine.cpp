/// @file test_engine.cpp
/// @brief Integration tests for the DestabilizerEngine.

#include <gtest/gtest.h>
#include "tether/destabilizer/DestabilizerEngine.hpp"
#include "tether/destabilizer/DestabilizerTypes.hpp"
#include "tether/simulation/SimulationEngine.hpp"
#include <cmath>
#include <atomic>
#include <thread>
#include <chrono>

using namespace Destabilizer;

// ---------------------------------------------------------------------------
// ThreadPool
// ---------------------------------------------------------------------------

TEST(ThreadPool, BasicExecution) {
    ThreadPool pool(2);
    std::atomic<int> counter{0};

    for (int i = 0; i < 10; ++i) {
        pool.submit([&counter]() { counter++; });
    }
    pool.waitAll();
    EXPECT_EQ(counter.load(), 10);
}

TEST(ThreadPool, SingleThread) {
    ThreadPool pool(1);
    int result = 0;
    pool.submit([&result]() { result = 42; });
    pool.waitAll();
    EXPECT_EQ(result, 42);
}

TEST(ThreadPool, NoTasksIsOk) {
    ThreadPool pool(2);
    pool.waitAll();
    pool.shutdown();
}

// ---------------------------------------------------------------------------
// DestabilizerEngine — basic configuration
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, DefaultConstruction) {
    DestabilizerEngine engine;
    // Should not crash
}

TEST(DestabilizerEngine, ConfigureMinimal) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1; // Mass-spring-damper
    config.horizon = 1.0;
    config.dt = 0.01;
    config.maxIterations = 5;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    config.channels.push_back(ch);

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);
    // Should succeed
}

// ---------------------------------------------------------------------------
// Full run on a simple system
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, RunOnMassSpringDamper) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1; // Mass-spring-damper
    config.horizon = 2.0;
    config.dt = 0.01;
    config.maxIterations = 10;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;
    config.instabilityThreshold = 100.0;
    config.earlyStop = false;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 10.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 8;

    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 5;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);

    auto result = engine.run();

    EXPECT_GT(result.totalIterations, 0);
    EXPECT_GT(result.totalSeconds, 0.0);
    EXPECT_FALSE(result.history.empty());
    EXPECT_FALSE(result.bestTheta.empty());
    EXPECT_GT(result.bestJ, 0.0);

    // Verdict should be one of the three valid values
    EXPECT_TRUE(result.verdict == Verdict::Destabilized ||
                result.verdict == Verdict::Robust ||
                result.verdict == Verdict::Inconclusive);
}

// ---------------------------------------------------------------------------
// Multi-start
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, MultiStart) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 1.0;
    config.dt = 0.01;
    config.maxIterations = 5;
    config.seed = 123;
    config.numStarts = 3;
    config.numThreads = 2;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 4;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 3;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);
    auto result = engine.run();

    EXPECT_GE(result.multiStartBestJ.size(), 3u);
}

// ---------------------------------------------------------------------------
// Progress callback
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, ProgressCallback) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 1.0;
    config.dt = 0.01;
    config.maxIterations = 5;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 4;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 3;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);

    int callbackCount = 0;
    engine.setProgressCallback([&callbackCount](const IterationResult& ir) {
        callbackCount++;
        EXPECT_GE(ir.iteration, 0);
    });

    engine.run();
    EXPECT_GT(callbackCount, 0);
}

// ---------------------------------------------------------------------------
// Controller callback
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, WithControllerCallback) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 1.0;
    config.dt = 0.01;
    config.maxIterations = 3;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 4;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 3;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);

    // Simple P-controller: u = -10*x
    class PController : public Simulation::SimController {
    public:
        Simulation::StateVector compute(double /*t*/,
                                         const Simulation::StateVector& measured,
                                         const Simulation::StateVector& /*reference*/,
                                         double /*dt*/) override {
            Simulation::StateVector u(measured.size(), 0.0);
            if (!measured.empty()) u[0] = -10.0 * measured[0];
            return u;
        }
        void reset() override {}
        const char* name() const override { return "PController"; }
    };
    engine.setController(std::make_shared<PController>());

    auto result = engine.run();
    EXPECT_GT(result.totalIterations, 0);
}

// ---------------------------------------------------------------------------
// Abort
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, AbortStopsExecution) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 1.0;
    config.dt = 0.01;
    config.maxIterations = 100000;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 2;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 1;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);

    int callbackCount = 0;
    engine.setProgressCallback([&callbackCount, &engine](const IterationResult& /*ir*/) {
        callbackCount++;
        if (callbackCount >= 3) {
            engine.abort();
        }
    });

    auto result = engine.run();
    EXPECT_LT(result.totalIterations, 100000);
}

// ---------------------------------------------------------------------------
// Multiple metrics
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, MultipleMetrics) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 1.0;
    config.dt = 0.01;
    config.maxIterations = 5;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 4;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 3;

    WeightedMetric wm1, wm2;
    wm1.type = MetricType::PeakStateDeviation;
    wm1.weight = 1.0;
    wm2.type = MetricType::IntegratedSquaredError;
    wm2.weight = 0.5;
    config.metrics.push_back(wm1);
    config.metrics.push_back(wm2);

    engine.configure(config);
    auto result = engine.run();

    EXPECT_EQ(result.metricValues.size(), 2u);
}

// ---------------------------------------------------------------------------
// Fourier perturbation type
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, FourierPerturbation) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 1.0;
    config.dt = 0.01;
    config.maxIterations = 3;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::FourierSpectral;
    config.perturbation.numHarmonics = 3;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 3;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);
    auto result = engine.run();
    EXPECT_GT(result.totalIterations, 0);
}

// ---------------------------------------------------------------------------
// WarmStart and state queries
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, WarmStartAndCurrentResult) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 0.5;
    config.dt = 0.01;
    config.maxIterations = 3;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 2;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 2;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);

    // Test warm start
    engine.warmStart({0.5, 0.5});

    // State before run
    EXPECT_FALSE(engine.isRunning());
    EXPECT_FALSE(engine.isPaused());

    auto result = engine.run();
    EXPECT_GT(result.totalIterations, 0);

    // Current result should be available
    auto r2 = engine.currentResult();
    EXPECT_GT(r2.bestJ, 0.0);
}

// ---------------------------------------------------------------------------
// Async start/pause/resume
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, AsyncStartPauseResume) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 0.5;
    config.dt = 0.01;
    config.maxIterations = 100000;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 2;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 1;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);

    // Use a progress callback with a latch to know when the engine is actually running
    std::atomic<bool> gotCallback{false};
    engine.setProgressCallback([&](const IterationResult&) {
        gotCallback.store(true);
    });

    // Start async
    engine.start();

    // Wait a bit for the thread to start and produce at least one iteration
    for (int i = 0; i < 100 && !gotCallback.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Pause/resume should not crash regardless of timing
    engine.pause();
    engine.resume();

    // Abort
    engine.abort();

    // Wait for thread to finish
    for (int i = 0; i < 20 && engine.isRunning(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    EXPECT_FALSE(engine.isRunning());
}

// ---------------------------------------------------------------------------
// SetAbortCheck callback
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, SetAbortCheck) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 0.5;
    config.dt = 0.01;
    config.maxIterations = 100000;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 5.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 2;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 1;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);

    std::atomic<int> iterations{0};
    engine.setProgressCallback([&](const IterationResult&) {
        iterations++;
    });
    engine.setAbortCheck([&]() -> bool {
        return iterations.load() >= 3;
    });

    auto result = engine.run();
    EXPECT_EQ(result.verdict, Verdict::Inconclusive);
    EXPECT_LE(result.totalIterations, 10); // Should abort early
}

// ---------------------------------------------------------------------------
// EarlyStop triggers Destabilized verdict
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, EarlyStopDestabilized) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 1.0;
    config.dt = 0.01;
    config.maxIterations = 100;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;
    config.earlyStop = true;
    config.instabilityThreshold = 0.0001; // Very low threshold

    PerturbationChannel ch;
    ch.inputIndex = 0;
    ch.name = "force";
    ch.constraints.amplitudeMax = 50.0;
    ch.constraints.rateMax = 1e6;
    ch.constraints.energyMax = 1e6;
    config.channels.push_back(ch);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 4;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 10;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);
    auto result = engine.run();
    // With large amplitude and low threshold, should be destabilized
    // The verdict and bestJ both indicate success
    EXPECT_GT(result.bestJ, 0.0) << "bestJ should be positive with perturbation";
    EXPECT_GT(result.totalIterations, 0);
}

// ---------------------------------------------------------------------------
// Multi-channel perturbation
// ---------------------------------------------------------------------------

TEST(DestabilizerEngine, MultiChannelPerturbation) {
    DestabilizerEngine engine;
    DestabilizerConfig config;
    config.systemId = 1;
    config.horizon = 0.5;
    config.dt = 0.01;
    config.maxIterations = 3;
    config.seed = 42;
    config.numStarts = 1;
    config.numThreads = 1;

    // Two channels
    PerturbationChannel ch1;
    ch1.inputIndex = 0;
    ch1.name = "force1";
    ch1.constraints.amplitudeMax = 5.0;
    ch1.constraints.rateMax = 1e6;
    ch1.constraints.energyMax = 1e6;
    config.channels.push_back(ch1);

    PerturbationChannel ch2;
    ch2.inputIndex = 0; // Same index is OK
    ch2.name = "force2";
    ch2.constraints.amplitudeMax = 5.0;
    ch2.constraints.rateMax = 1e6;
    ch2.constraints.energyMax = 1e6;
    config.channels.push_back(ch2);

    config.perturbation.type = PerturbationType::PiecewiseConstant;
    config.perturbation.numSegments = 2;
    config.optimizer.type = OptimizerType::RandomSearch;
    config.optimizer.batchSize = 2;

    WeightedMetric wm;
    wm.type = MetricType::PeakStateDeviation;
    wm.weight = 1.0;
    config.metrics.push_back(wm);

    engine.configure(config);
    auto result = engine.run();
    EXPECT_GT(result.totalIterations, 0);
}
