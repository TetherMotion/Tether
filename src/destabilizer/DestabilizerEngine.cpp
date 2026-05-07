/// @file DestabilizerEngine.cpp
/// @brief Implementation of ThreadPool and DestabilizerEngine.

#include "tether/destabilizer/DestabilizerEngine.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <cassert>

namespace Destabilizer {

// ===========================================================================
// ThreadPool
// ===========================================================================

ThreadPool::ThreadPool(int numThreads) {
    if (numThreads <= 0)
        numThreads = static_cast<int>(std::thread::hardware_concurrency());
    if (numThreads <= 0) numThreads = 1;

    for (int i = 0; i < numThreads; ++i) {
        workers_.emplace_back([this] {
            while (true) {
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(mutex_);
                    cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
                    if (stop_ && tasks_.empty()) return;
                    task = std::move(tasks_.front());
                    tasks_.pop();
                }
                task();
            }
        });
    }
}

ThreadPool::~ThreadPool() { shutdown(); }

void ThreadPool::waitAll() {
    std::unique_lock<std::mutex> lock(mutex_);
    allDone_.wait(lock, [this] { return activeTasks_ == 0 && tasks_.empty(); });
}

void ThreadPool::shutdown() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& w : workers_) {
        if (w.joinable()) w.join();
    }
    workers_.clear();
}

// ===========================================================================
// DestabilizerEngine
// ===========================================================================

DestabilizerEngine::DestabilizerEngine() = default;

DestabilizerEngine::~DestabilizerEngine() {
    abort();
    if (runThread_.joinable()) runThread_.join();
}

void DestabilizerEngine::configure(const DestabilizerConfig& config) {
    config_ = config;
}

void DestabilizerEngine::setController(std::shared_ptr<Simulation::SimController> controller) {
    controller_ = std::move(controller);
}

void DestabilizerEngine::setProgressCallback(ProgressCallback cb) {
    progressCallback_ = std::move(cb);
}

void DestabilizerEngine::setAbortCheck(AbortCheck cb) {
    abortCheck_ = std::move(cb);
}

void DestabilizerEngine::warmStart(const std::vector<double>& theta) {
    std::lock_guard<std::mutex> lock(resultMutex_);
    currentResult_.bestTheta = theta;
}

bool DestabilizerEngine::isRunning() const { return running_; }
bool DestabilizerEngine::isPaused() const { return paused_; }

DestabilizerResult DestabilizerEngine::currentResult() const {
    std::lock_guard<std::mutex> lock(resultMutex_);
    return currentResult_;
}

// ---------------------------------------------------------------------------
// Helper: compute total parameter dimension
// ---------------------------------------------------------------------------

int DestabilizerEngine::totalParameterCount(
    const std::vector<std::unique_ptr<PerturbationSignal>>& signals)
{
    int total = 0;
    for (auto& s : signals) total += s->parameterCount();
    return total;
}

// ---------------------------------------------------------------------------
// Simulate one rollout
// ---------------------------------------------------------------------------

RolloutTrajectory DestabilizerEngine::simulateRollout(
    const std::vector<double>& theta,
    const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals) const
{
    // Create a fresh simulation engine for this rollout
    Simulation::SimulationEngine simEngine;

    // Create the system
    auto system = Simulation::createSystem(config_.systemId);
    if (!system) {
        return {};
    }
    simEngine.setSystem(std::move(system));

    // Set simulation config
    Simulation::SimConfig simCfg;
    simCfg.dt = config_.dt;
    simCfg.totalTime = config_.horizon;
    simEngine.setConfig(simCfg);

    // Set controller if provided
    if (controller_) {
        simEngine.setController(controller_);
    }

    // Set reference
    simEngine.setReference(config_.referenceState);

    // Build external force from perturbation signals
    simEngine.setExternalForce([&theta, &pertSignals, this](double t, const Simulation::StateVector& state)
        -> Simulation::StateVector
    {
        Simulation::StateVector force(state.size(), 0.0);

        int thetaOffset = 0;
        for (size_t c = 0; c < pertSignals.size(); ++c) {
            int channelIdx = (c < config_.channels.size()) ?
                             config_.channels[c].inputIndex : static_cast<int>(c);

            int pCount = pertSignals[c]->parameterCount();
            std::vector<double> channelTheta(theta.begin() + thetaOffset,
                                               theta.begin() + thetaOffset + pCount);
            thetaOffset += pCount;

            double pertValue = pertSignals[c]->evaluate(t, channelTheta, state);

            if (channelIdx >= 0 && channelIdx < static_cast<int>(force.size())) {
                force[channelIdx] += pertValue;
            }
        }
        return force;
    });

    // Run simulation
    Simulation::SimulationRecord record = simEngine.run();

    // Convert to RolloutTrajectory
    RolloutTrajectory traj;
    traj.times = record.times;
    traj.states = record.states;
    traj.outputs = record.outputs;
    traj.controlInputs = record.controlInputs;
    traj.referenceState = config_.referenceState;
    traj.dt = config_.dt;
    traj.stateDim = record.states.empty() ? 0 :
                    static_cast<int>(record.states[0].size());
    traj.outputDim = record.outputs.empty() ? 0 :
                     static_cast<int>(record.outputs[0].size());
    traj.inputDim = record.controlInputs.empty() ? 0 :
                    static_cast<int>(record.controlInputs[0].size());

    return traj;
}

// ---------------------------------------------------------------------------
// Evaluate objective J(θ)
// ---------------------------------------------------------------------------

double DestabilizerEngine::evaluateObjective(
    const std::vector<double>& theta,
    const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals,
    const CombinedMetricEvaluator& evaluator) const
{
    auto traj = simulateRollout(theta, pertSignals);
    if (traj.times.empty()) return -1e20;
    return evaluator.evaluate(traj);
}

// ---------------------------------------------------------------------------
// Record worst-case trajectory
// ---------------------------------------------------------------------------

void DestabilizerEngine::recordWorstCase(
    DestabilizerResult& result,
    const RolloutTrajectory& traj,
    const std::vector<double>& theta,
    const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals) const
{
    result.bestTheta = theta;
    result.trajectoryTimes = traj.times;
    result.trajectoryStates = traj.states;
    result.trajectoryOutputs = traj.outputs;
    result.trajectoryControlInputs = traj.controlInputs;

    // Record perturbation signal values per channel
    result.worstCaseTimes = traj.times;
    result.worstCasePerturbation.clear();
    // Build per-channel signal arrays
    size_t nChannels = pertSignals.size();
    result.worstCasePerturbation.resize(nChannels);
    for (size_t c = 0; c < nChannels; ++c) {
        result.worstCasePerturbation[c].resize(traj.times.size());
    }
    for (size_t ti = 0; ti < traj.times.size(); ++ti) {
        double t = traj.times[ti];
        int thetaOff = 0;
        for (size_t c = 0; c < nChannels; ++c) {
            int pCount = pertSignals[c]->parameterCount();
            std::vector<double> channelTheta(theta.begin() + thetaOff,
                                               theta.begin() + thetaOff + pCount);
            thetaOff += pCount;
            std::vector<double> dummyState;
            result.worstCasePerturbation[c][ti] =
                pertSignals[c]->evaluate(t, channelTheta, dummyState);
        }
    }
}

// ---------------------------------------------------------------------------
// Run a single start
// ---------------------------------------------------------------------------

DestabilizerResult DestabilizerEngine::runSingleStart(
    const std::vector<double>& initialTheta, uint64_t seed)
{
    DestabilizerResult result;
    result.seed = seed;
    auto startTime = std::chrono::steady_clock::now();

    // Create perturbation signals for each channel
    std::vector<std::unique_ptr<PerturbationSignal>> pertSignals;
    for (size_t i = 0; i < config_.channels.size(); ++i) {
        auto signal = PerturbationSignal::create(config_.perturbation.type,
                                                  config_.perturbation,
                                                  config_.horizon);
        pertSignals.push_back(std::move(signal));
    }

    // Compute total dimension
    int totalDim = totalParameterCount(pertSignals);
    if (totalDim == 0) {
        result.verdict = Verdict::Robust;
        return result;
    }

    // Initialize optimizer
    auto optimizer = Optimizer::create(config_.optimizer.type);
    optimizer->initialize(totalDim, config_.optimizer, seed);

    // Create per-channel constraint projectors
    std::vector<ConstraintProjector> projectors;
    for (const auto& ch : config_.channels) {
        ConstraintProjector proj;
        proj.configure(ch.constraints, config_.perturbation, config_.horizon, config_.dt);
        projectors.push_back(std::move(proj));
    }

    // Create metric evaluator
    CombinedMetricEvaluator evaluator;
    evaluator.configure(config_.metrics, config_.safeSet,
                        config_.controlSaturationMin, config_.controlSaturationMax);

    // Initialize theta
    auto theta = initialTheta;
    if (static_cast<int>(theta.size()) != totalDim) {
        theta.assign(totalDim, 0.0);
    }

    // Project initial theta per channel
    {
        int offset = 0;
        for (size_t c = 0; c < projectors.size(); ++c) {
            int pCount = pertSignals[c]->parameterCount();
            std::vector<double> channelTheta(theta.begin() + offset,
                                               theta.begin() + offset + pCount);
            projectors[c].project(channelTheta);
            std::copy(channelTheta.begin(), channelTheta.end(), theta.begin() + offset);
            offset += pCount;
        }
    }

    double bestJ = -1e30;
    int totalFuncEvals = 0;

    // Main optimization loop
    for (int iter = 0; iter < config_.maxIterations; ++iter) {
        // Check abort
        if (aborted_) {
            result.verdict = Verdict::Inconclusive;
            break;
        }
        if (abortCheck_ && abortCheck_()) {
            result.verdict = Verdict::Inconclusive;
            break;
        }

        // Pause support
        while (paused_ && !aborted_) {
            std::this_thread::yield();
        }

        // Wall-clock limit
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - startTime).count();
        if (elapsed > config_.maxWallclockSeconds) break;

        // Create the objective function
        ObjectiveFunction obj = [this, &pertSignals, &evaluator](const std::vector<double>& th) -> double {
            return evaluateObjective(th, pertSignals, evaluator);
        };

        // Optimizer step
        double J = optimizer->step(theta, obj);
        // Function evaluations tracking based on optimizer type
        totalFuncEvals += 2;  // minimum per step

        // Project theta per channel
        {
            int offset = 0;
            for (size_t c = 0; c < projectors.size(); ++c) {
                int pCount = pertSignals[c]->parameterCount();
                std::vector<double> channelTheta(theta.begin() + offset,
                                                   theta.begin() + offset + pCount);
                auto active = projectors[c].project(channelTheta);
                std::copy(channelTheta.begin(), channelTheta.end(), theta.begin() + offset);
                offset += pCount;
            }
        }

        // Record convergence history
        IterationResult iterResult;
        iterResult.iteration = iter;
        iterResult.bestJ = std::max(bestJ, J);
        iterResult.currentJ = J;
        iterResult.gradientNorm = optimizer->gradientNorm();
        iterResult.stepSize = optimizer->currentStepSize();
        iterResult.elapsedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime).count();
        result.history.push_back(iterResult);

        // Update best
        if (J > bestJ) {
            bestJ = J;
            result.bestJ = bestJ;
            result.destabilizedAtIteration = iter;

            // Record worst-case trajectory
            auto traj = simulateRollout(theta, pertSignals);
            recordWorstCase(result, traj, theta, pertSignals);

            // Compute metric breakdown
            result.metricValues = evaluator.evaluateBreakdown(traj);
        }

        // Progress callback
        if (progressCallback_) {
            progressCallback_(iterResult);
        }

        // Early stop
        if (config_.earlyStop && bestJ > config_.instabilityThreshold) {
            result.destabilizedAtIteration = iter;
            break;
        }

        // Function evaluation limit
        if (totalFuncEvals >= config_.maxFunctionEvaluations) break;
    }

    // Determine verdict
    if (result.verdict != Verdict::Inconclusive) {
        if (bestJ > config_.instabilityThreshold) {
            result.verdict = Verdict::Destabilized;
        } else {
            result.verdict = Verdict::Robust;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    result.totalSeconds = std::chrono::duration<double>(endTime - startTime).count();
    result.totalFunctionEvaluations = totalFuncEvals;
    result.totalIterations = static_cast<int>(result.history.size());

    return result;
}

// ---------------------------------------------------------------------------
// Main run() — multi-start
// ---------------------------------------------------------------------------

DestabilizerResult DestabilizerEngine::run() {
    running_ = true;
    aborted_ = false;
    paused_ = false;

    auto startTime = std::chrono::steady_clock::now();

    // Create thread pool
    int nThreads = config_.numThreads;
    if (nThreads <= 0) nThreads = static_cast<int>(std::thread::hardware_concurrency());
    pool_ = std::make_unique<ThreadPool>(nThreads);

    // Check if we have a warm-start theta
    std::vector<double> warmStartTheta;
    {
        std::lock_guard<std::mutex> lock(resultMutex_);
        warmStartTheta = currentResult_.bestTheta;
    }

    int nStarts = std::max(1, config_.numStarts);
    std::vector<std::future<DestabilizerResult>> futures;

    // Launch multi-start runs
    std::mt19937_64 seedGen(config_.seed);
    for (int s = 0; s < nStarts; ++s) {
        uint64_t startSeed = seedGen();
        std::vector<double> initTheta;

        if (s == 0 && !warmStartTheta.empty()) {
            initTheta = warmStartTheta;
        } else {
            // Random initialization
            std::mt19937_64 rng(startSeed);
            std::normal_distribution<double> dist(0.0, 0.1);

            std::vector<std::unique_ptr<PerturbationSignal>> tempSignals;
            for (size_t i = 0; i < config_.channels.size(); ++i) {
                auto sig = PerturbationSignal::create(config_.perturbation.type,
                    config_.perturbation, config_.horizon);
                tempSignals.push_back(std::move(sig));
            }
            int totalDim = totalParameterCount(tempSignals);
            initTheta.resize(totalDim);
            for (auto& v : initTheta) v = dist(rng);
        }

        futures.push_back(pool_->submit([this, initTheta, startSeed]() {
            return runSingleStart(initTheta, startSeed);
        }));
    }

    // Collect results, keep the best
    DestabilizerResult bestResult;
    bestResult.verdict = Verdict::Robust;
    double bestOverall = -1e30;

    for (auto& fut : futures) {
        auto result = fut.get();

        double thisJ = result.bestJ;
        bestResult.multiStartBestJ.push_back(thisJ);

        if (thisJ > bestOverall) {
            bestOverall = thisJ;
            auto savedMultiStartBestJ = std::move(bestResult.multiStartBestJ);
            bestResult = std::move(result);
            bestResult.multiStartBestJ = std::move(savedMultiStartBestJ);
        }
    }

    pool_->shutdown();
    pool_.reset();

    auto endTime = std::chrono::steady_clock::now();
    bestResult.totalSeconds = std::chrono::duration<double>(endTime - startTime).count();
    bestResult.seed = config_.seed;

    // Store result
    {
        std::lock_guard<std::mutex> lock(resultMutex_);
        currentResult_ = bestResult;
    }

    running_ = false;
    return bestResult;
}

// ---------------------------------------------------------------------------
// Async control
// ---------------------------------------------------------------------------

void DestabilizerEngine::start() {
    if (running_) return;
    if (runThread_.joinable()) runThread_.join();

    runThread_ = std::thread([this]() { run(); });
}

void DestabilizerEngine::pause() { paused_ = true; }
void DestabilizerEngine::resume() { paused_ = false; }

void DestabilizerEngine::abort() {
    aborted_ = true;
    paused_ = false;
    if (runThread_.joinable()) runThread_.join();
}

} // namespace Destabilizer
