#pragma once
/**
 * @file DestabilizerEngine.hpp
 * @brief Main Destabilizer engine: orchestrates adversarial search.
 */

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

/**Simple work-stealing thread pool for parallel rollout evaluation. */
class ThreadPool {
public:
    explicit ThreadPool(int numThreads = 0);
    ~ThreadPool();

    /**
     * @brief Submit a task and get a future for its result.
     *
     * The task is queued and picked up by the next free worker;
     * activeTasks_ counts it immediately so waitAll() observes it even
     * before a worker dequeues it.
     *
     * @param f      Callable to execute.
     * @param args   Arguments forwarded to @p f (bound at submit time).
     * @return std::future delivering the callable's return value; throws
     *         std::runtime_error if the pool has been shut down.
     */
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args) -> std::future<decltype(f(args...))>;

    /**
     * @brief Get number of threads.
     *
     * @return Worker-thread count — the number of spawned workers, not
     *         the numThreads constructor hint.
     */
    int size() const { return static_cast<int>(workers_.size()); }

    /**Wait for all pending tasks to complete. */
    void waitAll();

    /**Shutdown the pool. */
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

/**
 * Main Destabilizer engine.
 *
 * Usage:
 * 1. Configure with DestabilizerConfig.
 * 2. Optionally set a progress callback.
 * 3. Call run() — blocking, returns DestabilizerResult.
 * 4. Or call start()/pause()/resume()/abort() for async control.
 */
class DestabilizerEngine {
public:
    DestabilizerEngine();
    ~DestabilizerEngine();

    /**
     * @brief Set the configuration for the run.
     *
     * Must be called before run()/start(). Copied — the caller's struct
     * may be reused; a re-configure between runs resets the search to
     * the new parameter bounds, population, and iteration budget.
     *
     * @param config  DestabilizerConfig: parameter-space definition,
     *                search strategy, rollout/iteration limits, and
     *                success criteria.
     */
    void configure(const DestabilizerConfig& config);

    /**
     * @brief Set a custom controller (otherwise open-loop).
     *
     * The controller drives each simulated rollout; without one the
     * system under test runs open-loop, useful for intrinsic-stability
     * searches. Shared ownership — the engine keeps it alive for the
     * run's duration.
     *
     * @param controller  Controller used by rollouts, or nullptr to
     *                    revert to open-loop.
     */
    void setController(std::shared_ptr<Simulation::SimController> controller);

    /**
     * @brief Set progress callback (called after each iteration).
     *
     * Fires once per completed search iteration with the current best —
     * use it for UI updates or early termination decisions. Called on
     * the engine's worker thread, so keep it fast; heavy work should be
     * handed off to another thread.
     *
     * @param cb  Callback receiving the iteration's progress report.
     */
    void setProgressCallback(ProgressCallback cb);

    /**
     * @brief Set abort check callback (called before each iteration).
     *
     * Returning true aborts the search at the next iteration boundary
     * and yields a DestabilizerResult flagged aborted — the cooperative
     * way to stop a long run without losing the best-so-far candidate.
     *
     * @param cb  Callback queried before every iteration.
     */
    void setAbortCheck(AbortCheck cb);

    /**
     * @brief Run the destabilizer search. Blocking call.
     *
     * Executes the full configured search synchronously — rollouts are
     * still fanned out across the internal thread pool, only the
     * orchestration blocks the caller.
     *
     * @return Final DestabilizerResult: best-found θ, achieved
     *         destabilization metric, iteration log, and completion/
     *         abort status.
     */
    DestabilizerResult run();

    /**
     * @brief Warm-start from a previous result's best θ.
     *
     * Seeds the initial population around @p theta instead of sampling
     * the parameter space cold — the standard way to refine a solution
     * found by an earlier run or to continue an aborted search. Call
     * between configure() and run()/start().
     *
     * @param theta  Parameter vector in the order defined by the
     *               configuration; dimension must match the search
     *               space.
     */
    void warmStart(const std::vector<double>& theta);

    /**Non-blocking control. */
    void start();
    void pause();
    void resume();
    void abort();
    bool isRunning() const;
    bool isPaused() const;

    /**
     * @brief Get the current result (thread-safe snapshot).
     *
     * @return Copy of the best result so far — while running this is a
     *         mid-search snapshot reflecting the latest completed
     *         iteration; after run()/abort() it equals the final result.
     */
    DestabilizerResult currentResult() const;

    /**
     * @brief Get config (const).
     *
     * @return Const reference to the configuration installed by the last
     *         configure() call (or the defaults); valid until the next
     *         configure().
     */
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

    /**Execute a single optimization run from an initial θ. */
    DestabilizerResult runSingleStart(const std::vector<double>& initialTheta,
                                      uint64_t seed);

    /**Simulate one rollout with the given perturbation parameters. */
    RolloutTrajectory simulateRollout(const std::vector<double>& theta,
                                      const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals) const;

    /**Evaluate J(θ) by running a simulation rollout. */
    double evaluateObjective(const std::vector<double>& theta,
                             const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals,
                             const CombinedMetricEvaluator& evaluator) const;

    /**Build the combined parameter vector from per-channel θ. */
    static int totalParameterCount(const std::vector<std::unique_ptr<PerturbationSignal>>& signals);

    /**Record worst-case trajectory into the result. */
    void recordWorstCase(DestabilizerResult& result,
                         const RolloutTrajectory& traj,
                         const std::vector<double>& theta,
                         const std::vector<std::unique_ptr<PerturbationSignal>>& pertSignals) const;
};

} // namespace Destabilizer
