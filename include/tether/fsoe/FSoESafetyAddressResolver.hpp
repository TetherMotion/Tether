#pragma once

// ============================================================================
// FSoESafetyAddressResolver — reverse-calculate a slave's safety address
// ============================================================================
//
// When the master rejects a slave frame with a CRC error, one common cause
// is a mismatched connection ID / device safety address: the slave answers
// on its configured address, but the master validates the frame CRCs with a
// different connID, so every segment fails — typically with a constant
// "received" CRC because the slave's frame never changes.
//
// This utility recovers the slave's actual connID from the rejected frame:
// since the connID is folded into every segment CRC, a brute-force sweep of
// all 65536 candidate connIDs (times a small sequence-number window) finds
// the value(s) under which every embedded CRC validates.  The search runs
// on a dedicated worker thread so the cyclic/RT path only pays the cost of
// a frame copy; the result is delivered through a callback invoked on that
// worker thread (a "second log line" after the CRCError line).
//
// ============================================================================

#include "fsoe/FSoECRC.hpp"

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <span>
#include <thread>
#include <vector>

namespace FSoE {

class SafetyAddressResolver {
public:
    struct Job {
        std::vector<uint8_t> frame;      ///< Raw rejected slave frame
        uint16_t start_crc = 0;          ///< CRC inheritance base used for checking
        uint16_t seq_no = 0;             ///< Expected sequence number
        uint16_t seq_window = 4;         ///< Try seq_no ± this many candidates
    };

    struct Result {
        bool     found = false;
        uint16_t conn_id = 0;            ///< connID under which all CRCs validate
        uint16_t seq_no = 0;             ///< seq under which all CRCs validate
        uint16_t frame_conn_id = 0;      ///< connID bytes from the frame trailer
        uint8_t  command = 0;            ///< command byte of the rejected frame
        int      crc_segments = 0;       ///< number of embedded CRCs compared
        int      matches = 0;            ///< connID×seq combos that validated (16-bit CRC collisions are possible)
        uint64_t candidates_tested = 0;  ///< connID×seq combinations tried
    };

    /// Invoked on the resolver's worker thread with the search outcome.
    using ResultCallback = std::function<void(const Result&)>;

    SafetyAddressResolver() = default;
    ~SafetyAddressResolver();

    SafetyAddressResolver(const SafetyAddressResolver&) = delete;
    SafetyAddressResolver& operator=(const SafetyAddressResolver&) = delete;

    /// Queue a rejected frame for background resolution.  Non-blocking.
    /// @return false if the resolver is busy with a previous frame (the
    ///         caller should drop this frame) or the resolver is stopped.
    bool submit(Job job, ResultCallback cb);

    /// Stop the worker thread and discard pending jobs.  Safe to call
    /// multiple times; also invoked by the destructor.
    void stop();

    /// True while a search is queued or running.
    bool busy() const;

private:
    void workerMain();
    static Result solve(const Job& job);

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<std::pair<Job, ResultCallback>> queue_;
    std::thread worker_;
    bool stop_ = false;
    bool running_ = false;              ///< a search is currently executing
};

} // namespace FSoE
