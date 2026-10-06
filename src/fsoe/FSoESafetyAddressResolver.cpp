#include "fsoe/FSoESafetyAddressResolver.hpp"

namespace FSoE {

SafetyAddressResolver::~SafetyAddressResolver() {
    stop();
}

void SafetyAddressResolver::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_) return;
        stop_ = true;
        queue_.clear();
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

bool SafetyAddressResolver::busy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return (running_ || !queue_.empty()) && !stop_;
}

bool SafetyAddressResolver::submit(Job job, ResultCallback cb) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_ || running_ || !queue_.empty()) return false;   // one search at a time
        if (!worker_.joinable()) {
            worker_ = std::thread(&SafetyAddressResolver::workerMain, this);
        }
        queue_.emplace_back(std::move(job), std::move(cb));
    }
    cv_.notify_one();
    return true;
}

void SafetyAddressResolver::workerMain() {
    for (;;) {
        std::pair<Job, ResultCallback> work;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [&] { return stop_ || !queue_.empty(); });
            if (stop_ || queue_.empty()) return;
            work = std::move(queue_.front());
            queue_.pop_front();
            running_ = true;
        }
        Result result = solve(work.first);
        if (work.second) work.second(result);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
        }
        // Keep the worker alive for reuse; loop back and wait for the
        // next submission.
    }
}

SafetyAddressResolver::Result SafetyAddressResolver::solve(const Job& job) {
    Result r;
    const uint8_t* frame = job.frame.data();
    const size_t len = job.frame.size();

    // Extract the frame contents (no CRC verification — that is what we
    // are brute-forcing).  data_buf must hold MAX_PARSE_DATA_SIZE bytes.
    std::array<uint8_t, CRC::MAX_PARSE_DATA_SIZE> data_buf{};
    size_t num_data = 0;
    if (!CRC::extractFSoEFrame(frame, len, r.command, data_buf, num_data,
                               r.frame_conn_id)) {
        return r;   // unparseable — nothing to search
    }
    r.crc_segments = 0;

    // Collect the CRCs embedded in the frame, in segment order.
    // Layout (see FSoECRC.hpp): size<=6 → CRC0 at offset 2; size>6 →
    // CRC0 at offset 3, each further segment is (2 data + 2 CRC) so
    // segment i sits at offset 3 + 4*i.
    const int first_data = (len > 6) ? 2 : 1;
    const int num_crcs = 1 + (num_data > static_cast<size_t>(first_data)
        ? static_cast<int>((num_data - first_data + 1) / 2) : 0);
    std::array<uint16_t, 16> embedded{};
    {
        size_t off = (len <= 6) ? 2 : 3;
        for (int i = 0; i < num_crcs && off + 1 < len; ++i) {
            embedded[i] = static_cast<uint16_t>(frame[off]) |
                          (static_cast<uint16_t>(frame[off + 1]) << 8);
            off += 4;
        }
        r.crc_segments = num_crcs;
    }

    // Sequence-number candidates: the expected seq plus a window around
    // it (the slave's counter may be ahead/behind ours).  Use the
    // ETG.5100 increment/decrement helpers to respect the 1..65535 range.
    std::vector<uint16_t> seqs;
    seqs.push_back(job.seq_no);
    uint16_t seq_m = job.seq_no, seq_p = job.seq_no;
    for (uint16_t d = 0; d < job.seq_window; ++d) {
        seq_m = CRC::decrementSeqNo(seq_m);
        seq_p = CRC::incrementSeqNo(seq_p);
        seqs.push_back(seq_m);
        seqs.push_back(seq_p);
    }

    // Brute force: for every connID × seq candidate, recompute all
    // segment CRCs and compare against the CRCs embedded in the frame.
    // A full match on every segment identifies the slave's real connID.
    for (const uint16_t seq : seqs) {
        for (uint32_t cand = 0; cand <= 0xFFFF; ++cand) {
            const auto crcs = CRC::computeAllCrcs(
                job.start_crc, static_cast<uint16_t>(cand), seq,
                r.command, data_buf.data(), static_cast<int>(num_data),
                static_cast<int>(len));
            ++r.candidates_tested;

            bool match = true;
            for (int i = 0; i < num_crcs; ++i) {
                if (crcs[i] != embedded[i]) { match = false; break; }
            }
            if (match) {
                ++r.matches;
                if (!r.found) {
                    r.found = true;
                    r.conn_id = static_cast<uint16_t>(cand);
                    r.seq_no = seq;
                }
            }
        }
    }
    return r;
}

} // namespace FSoE
