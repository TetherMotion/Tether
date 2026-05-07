#pragma once
/// @file DestabilizerEngine.hpp
/// @brief Main Destabilizer engine: orchestrates adversarial search.

#include "DestabilizerTypes.hpp"
#include "Perturbation.hpp"
#include "ConstraintProjector.hpp"
#include "InstabilityMetrics.hpp"
#include "Optimizers.hpp"
#include "DefaultLimits.hpp"
#include "tether/simulation/SimulationEngine.hpp"
#include "tether/simulation/AllSystems.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <atomic>
#include <chrono>
#include <future>

namespace Destabilizer {

// ---------------------------------------------------------------------------
// Thread Pool
// ---------------------------------------------------------------------------

/// Simple work-stealing thread pool for parallel rollout evaluation.
class ThreadPool {
public:
    explicit ThreadPool(int numThreads = 0);
    ~ThreadPool();

    /// Submit a task and get a future for its result.
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))>;

    /// Get number of threads.
    int size() const { return static_cast<int>(workers_.size()); }

    /// Wait for all pending tasks to complete.
    void waitAll();

    /// Shutdown the pool.
    void shutdown();

private:
    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> stop_{false};
    std::atomic<int> activeTasks_{0};
    std::condition_variable allDone_;
};

// Template implementation (must be in header)
template<typename F, typename... Args>
auto ThreadPool::submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))> {
    using ReturnType = decltype(f(args...));
    auto task = std::make_shared<std::packaged_task<ReturnType()>>(
        std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );
    auto future = task->get_future();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_) throw std::runtime_error("ThreadPool is stopped");
        activeTasks_++;
        tasks_.emplace([task, this]() {
            (*task)();
            activeTasks_--;
            allDone_.notify_all();
        });
    }
    cv_.notify_one();
    return future;
}

// ---------------------------------------------------------------------------
// Destabilizer Engine
// ---------------------------------------------------------------------------

/// Main Destabilizer engine.
///
/// Usage:
/// 1. Configure with DestabilizerConfig.
/// 2. Optionally set a progress callback.
/// 3. Call run() — blocking, returns DestabilizerResult.
/// 4. Or call start()/pause()/resume()/abort() for async control.
class DestabilizerEngine {
public:
    DestabilizerEngine();
    ~DestabilizerEngine();

    /// Set the configuration for the run.
    void configure(const DestabilizerConfig& config);

    /// Set a custom controller (otherwise open-loop).
    void setController(std::shared_ptr<Simulation::SimController> controller);

    /// Set progress callback (called after each iteration).
    void setProgressCallback(ProgressCallback cb);

    /// Set abort check callback (called before each iteration).
    void setAbortCheck(AbortCheck cb);

    /// Run the destabilizer search. Blocking call.
    DestabilizerResult run();

    /// Warm-start from a previous result's best θ.
    void warmStart(const std::vector<double>& theta);

    /// Non-blocking control.
    void start();
    void pause();
    void resume();
    void abort();
    bool isRunning() const;
    bool isPaused() const;

    /// Get the current result (thread-safe snapshot).
    DestabilizerResult currentResult() const;

    /// Get config (const).
    const DestabilizerConfig& config() const { return config_; }

private:
    DestabilizerConfig config_;
    std::shared_ptr<Simulation::SimController> controller_;
    ProgressCallback progressCallback_;
    AbortCheck abortCheck_;

    // Thread pool for parallel rollouts
    std::unique_ptr<ThreadPool> pool_;

    // Run state
    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> aborted_{false};
    mutable std::mutex resultMutex_;
    DestabilizerResult currentResult_;

    // Async thread
    std::thread runThread_;

    /// Execute a single optimization run from an initial θ.
    DestabilizerResult runSingleStart(const std::vector<double>& initialTheta,
                                      uint64_t seed);

    /// Simulate one rollout with the given perturbation parameters.
    RolloutTrajectory simulateRollout(const std::vector<double>& theta,
                                      const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals) const;

    /// Evaluate J(θ) by running a simulation rollout.
    double evaluateObjective(const std::vector<double>& theta,
                             const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals,
                             const CombinedMetricEvaluator& evaluator) const;

    /// Build the combined parameter vector from per-channel θ.
    static int totalParameterCount(const std::vector<std::unique_ptr<PerturbationSignal>>& signals);

    /// Record worst-case trajectory into the result.
    void recordWorstCase(DestabilizerResult& result,
                         const RolloutTrajectory& traj,
                         const std::vector<double>& theta,
                         const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals) const;
};

} // namespace Destabilizer
