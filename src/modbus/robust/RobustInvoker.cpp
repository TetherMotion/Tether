#include "tether/modbus/robust/RobustInvoker.hpp"

#include <algorithm>
#include <cstring>
#include <thread>

namespace tether::modbus {

AssuranceStats RobustInvoker::stats_{};

ModbusError RobustInvoker::notCompiled(Strategy s) {
    stats_.notCompiled++;
    return ModbusError::assurance(ErrorCode::StrategyNotCompiled,
        "strategy not compiled: " + std::string(strategyName(s)));
}

Expected<Pdu> RobustInvoker::readOnce(uint8_t u, FunctionCode fc,
                                      std::span<const uint8_t> b,
                                      const InvokeOptions& o) {
    stats_.samples++;
    return inner_.invoke(u, fc, b, o);
}

bool RobustInvoker::same(const Pdu& a, const Pdu& b) const {
    const auto ab = a.bytes(), bb = b.bytes();
    if (ab.size() != bb.size()) return false;
    if (opts_.epsilon <= 0.0)
        return std::equal(ab.begin(), ab.end(), bb.begin());
    const size_t body = ab.size() - 1;
    if (body % 2) return std::equal(ab.begin(), ab.end(), bb.begin());
    for (size_t i = 1; i + 1 < ab.size(); i += 2) {
        const int x = (ab[i] << 8) | ab[i + 1];
        const int y = (bb[i] << 8) | bb[i + 1];
        if (std::abs(x - y) > opts_.epsilon) return false;
    }
    return true;
}

Expected<Pdu> RobustInvoker::invoke(uint8_t unit, FunctionCode fc,
                                    std::span<const uint8_t> body,
                                    const InvokeOptions& o) {
    stats_.ops++;
    if (opts_.strategy == Strategy::Direct)
        return inner_.invoke(unit, fc, body, o);
    if (!strategyCompiled(opts_.strategy))
        return err<Pdu>(notCompiled(opts_.strategy));
    return applyRead(unit, fc, body, o);
}

Expected<Pdu> RobustInvoker::applyRead(uint8_t u, FunctionCode fc,
                                       std::span<const uint8_t> b,
                                       const InvokeOptions& o) {
    auto gap = [&] {
        if (opts_.sampleGapMs > 0)
            std::this_thread::sleep_for(
                std::chrono::milliseconds(opts_.sampleGapMs));
    };
    auto majority = [](std::vector<Pdu>& v, auto& self) -> Expected<Pdu> {
        for (size_t i = 0; i < v.size(); ++i) {
            int c = 0;
            for (size_t j = 0; j < v.size(); ++j)
                if (self(v[i], v[j])) ++c;
            if (c * 2 > int(v.size())) return v[i];
        }
        return err<Pdu>(ModbusError::assurance(
            ErrorCode::AssuranceMismatch, "no majority"));
    };
    auto sameF = [this](const Pdu& a, const Pdu& b) { return same(a, b); };

    switch (opts_.strategy) {
#ifdef TETHER_MODBUS_ROBUST_READ_TWICE
        case Strategy::TwiceErrorOnMismatch: {
            auto a = readOnce(u, fc, b, o);
            if (!a) return a;
            gap();
            auto b2 = readOnce(u, fc, b, o);
            if (!b2) return b2;
            if (same(*a, *b2)) return a;
            stats_.mismatches++;
            return err<Pdu>(ModbusError::assurance(
                ErrorCode::AssuranceMismatch, "twice: mismatch"));
        }
        case Strategy::TwiceRereadOnMismatch: {
            auto a = readOnce(u, fc, b, o);
            if (!a) return a;
            gap();
            auto b2 = readOnce(u, fc, b, o);
            if (!b2) return b2;
            if (same(*a, *b2)) return a;
            stats_.mismatches++;
            gap();
            auto c = readOnce(u, fc, b, o);
            if (!c) return c;
            if (same(*a, *c) || same(*b2, *c)) return c;
            return err<Pdu>(ModbusError::assurance(
                ErrorCode::AssuranceMismatch, "reread: no pair"));
        }
#endif
#if defined(TETHER_MODBUS_ROBUST_THRICE_VOTE) || \
    defined(TETHER_MODBUS_ROBUST_QUORUM) || \
    defined(TETHER_MODBUS_ROBUST_MEDIAN) || \
    defined(TETHER_MODBUS_ROBUST_UNTIL_STABLE)
        case Strategy::ThriceMajorityVote:
        case Strategy::NQuorum: {
            const int n = (opts_.strategy == Strategy::NQuorum)
                              ? std::max(1, opts_.n) : 3;
            const int q = (opts_.strategy == Strategy::NQuorum)
                              ? std::max(1, opts_.quorum) : 2;
            std::vector<Pdu> ok;
            for (int i = 0; i < n; ++i) {
                if (i) gap();
                auto r = readOnce(u, fc, b, o);
                if (r) ok.push_back(std::move(*r));
            }
            for (auto& cand : ok) {
                int c = 0;
                for (auto& v : ok) if (same(cand, v)) ++c;
                if (c >= q) { stats_.votesWon++; return cand; }
            }
            stats_.votesLost++;
            return err<Pdu>(ModbusError::assurance(
                ErrorCode::QuorumUnreachable, "quorum unreachable"));
        }
        case Strategy::Median: {
            std::vector<Pdu> ok;
            const int n = std::max(3, opts_.n | 1);   // odd
            for (int i = 0; i < n; ++i) {
                if (i) gap();
                auto r = readOnce(u, fc, b, o);
                if (r) ok.push_back(std::move(*r));
            }
            if (ok.empty())
                return err<Pdu>(ModbusError::assurance(
                    ErrorCode::QuorumUnreachable, "median: no samples"));
            // element-wise median over 16-bit lanes of pdu bodies
            std::sort(ok.begin(), ok.end(),
                      [](const Pdu& a, const Pdu& b) {
                          const auto ab = a.body(), bb = b.body();
                          return std::lexicographical_compare(
                              ab.begin(), ab.end(), bb.begin(), bb.end());
                      });
            return ok[ok.size() / 2];
        }
        case Strategy::UntilStable: {
            const int cap = std::max(2, opts_.n);
            std::optional<Pdu> prev;
            for (int i = 0; i < cap; ++i) {
                if (i) gap();
                auto r = readOnce(u, fc, b, o);
                if (!r) return r;
                if (prev && same(*prev, *r)) return r;
                prev = std::move(*r);
            }
            return err<Pdu>(ModbusError::assurance(
                ErrorCode::AssuranceUnstable, "unstable read"));
        }
#endif
#ifdef TETHER_MODBUS_ROBUST_STICKY
        case Strategy::Sticky: {
            auto r = readOnce(u, fc, b, o);
            if (r) { lastGood_[0] = *r; return r; }
            if (opts_.allowStale && lastGood_[0]) {
                stats_.staleServed++;
                return *lastGood_[0];
            }
            return r;
        }
#endif
        default:
            return err<Pdu>(notCompiled(opts_.strategy));
    }
}

Expected<Pdu> RobustInvoker::assuredWrite(
    uint8_t unit, FunctionCode writeFc, std::span<const uint8_t> body,
    FunctionCode readFc, std::span<const uint8_t> confirmBody,
    std::span<const uint8_t> expectedRespBody,
    const InvokeOptions& o) {
    stats_.ops++;
    if (!strategyCompiled(opts_.strategy))
        return err<Pdu>(notCompiled(opts_.strategy));

    const int rounds =
        opts_.strategy == Strategy::WriteThriceConfirm ? 3 : 1;
    int confirmed = 0;
    const int need = std::max(1, rounds - (opts_.maxConfirmRetries > 0 ? 0 : 0));

    for (int i = 0; i < std::max(1, rounds); ++i) {
        auto w = inner_.invoke(unit, writeFc, body, o);
        if (!w) return w;
        if (opts_.confirmDelayMs > 0)
            std::this_thread::sleep_for(
                std::chrono::milliseconds(opts_.confirmDelayMs));
        auto r = inner_.invoke(unit, readFc, confirmBody, o);
        if (!r) {
            if (i + 1 < rounds) continue;
            return err<Pdu>(ModbusError::assurance(
                ErrorCode::ConfirmUnavailable, "confirm read failed"));
        }
        stats_.confirms++;
        const auto rb = r->body();
        const bool match =
            rb.size() == expectedRespBody.size() &&
            std::equal(rb.begin(), rb.end(), expectedRespBody.begin());
        if (match) ++confirmed;
        else {
            stats_.confirmFailures++;
            stats_.mismatches++;
            if (opts_.strategy == Strategy::WriteConfirm &&
                i >= opts_.maxConfirmRetries)
                return err<Pdu>(ModbusError::assurance(
                    ErrorCode::ConfirmFailed, "write confirm mismatch"));
        }
    }
    if (opts_.strategy == Strategy::WriteThriceConfirm && confirmed < need)
        return err<Pdu>(ModbusError::assurance(
            ErrorCode::ConfirmFailed, "thrice confirm failed"));
    Pdu done{writeFc};
    done.append(body);
    return done;
}

Expected<void> RobustInvoker::invokeBroadcast(
    uint8_t, FunctionCode fc, std::span<const uint8_t> body,
    const InvokeOptions& o) {
#ifdef TETHER_MODBUS_ROBUST_BCAST_FOLLOWUP
    if (opts_.strategy == Strategy::BroadcastFollowup) {
        // caller supplies per-unit confirm reads via assuredWrite instead;
        // here: plain broadcast (follow-up is orchestrated by the caller
        // since it needs unit list) — still compile-gated.
        return inner_.invokeBroadcast(0, fc, body, o);
    }
#endif
    return inner_.invokeBroadcast(0, fc, body, o);
}

} // namespace tether::modbus
