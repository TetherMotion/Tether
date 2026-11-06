/**
 * @file Strategy.hpp
 * @brief Assurance strategy enum + per-strategy compile flags + opts.
 *
 * Each strategy is independently selectable at compile time:
 *   TETHER_MODBUS_ROBUST_READ_TWICE          TwiceErrorOnMismatch +
 *                                            TwiceRereadOnMismatch
 *   TETHER_MODBUS_ROBUST_THRICE_VOTE         ThriceMajorityVote
 *   TETHER_MODBUS_ROBUST_QUORUM              NQuorum
 *   TETHER_MODBUS_ROBUST_MEDIAN              Median
 *   TETHER_MODBUS_ROBUST_UNTIL_STABLE        UntilStable
 *   TETHER_MODBUS_ROBUST_STICKY              Sticky
 *   TETHER_MODBUS_ROBUST_WRITE_CONFIRM       WriteConfirm
 *   TETHER_MODBUS_ROBUST_WRITE_THRICE_CONFIRM WriteThriceConfirm
 *   TETHER_MODBUS_ROBUST_WRITE_WATCHDOG      WriteWatchdog
 *   TETHER_MODBUS_ROBUST_BCAST_FOLLOWUP      BroadcastFollowup
 *   TETHER_MODBUS_ROBUST_STATS               AssuranceStats counters
 *
 * A disabled-but-requested strategy fails loudly with
 * ErrorCode::StrategyNotCompiled — public API never changes shape.
 */
#pragma once

#include <cstdint>
#include <string_view>

namespace tether::modbus {

enum class Strategy : uint8_t {
    Direct = 0,                  ///< single shot (always compiled)

    // ---- read strategies ---------------------------------------------------
    TwiceErrorOnMismatch,        ///< 2 reads; mismatch → error
    TwiceRereadOnMismatch,       ///< 2 reads; on mismatch a 3rd decides
    ThriceMajorityVote,          ///< 3 reads; 2-of-3 majority wins
    NQuorum,                     ///< n samples, ≥q must agree
    Median,                      ///< n samples → median (analog)
    UntilStable,                 ///< until 2 consecutive equal (cap n)
    Sticky,                      ///< on transient failure → last good+stale

    // ---- write strategies ---------------------------------------------------
    WriteConfirm,                ///< write, read back, compare
    WriteThriceConfirm,          ///< write+confirm, repeat ≥2 agreeing
    WriteWatchdog,               ///< confirm after delay
    BroadcastFollowup,           ///< unit-0 write + per-unit confirm read
};

constexpr std::string_view strategyName(Strategy s) {
    switch (s) {
        case Strategy::Direct:               return "direct";
        case Strategy::TwiceErrorOnMismatch: return "twice_error";
        case Strategy::TwiceRereadOnMismatch:return "twice_reread";
        case Strategy::ThriceMajorityVote:   return "thrice_vote";
        case Strategy::NQuorum:              return "n_quorum";
        case Strategy::Median:               return "median";
        case Strategy::UntilStable:          return "until_stable";
        case Strategy::Sticky:               return "sticky";
        case Strategy::WriteConfirm:         return "write_confirm";
        case Strategy::WriteThriceConfirm:   return "write_thrice_confirm";
        case Strategy::WriteWatchdog:        return "write_watchdog";
        case Strategy::BroadcastFollowup:    return "bcast_followup";
    }
    return "?";
}

/// Compile-time probe: is strategy `s` compiled in?
constexpr bool strategyCompiled(Strategy s) {
    switch (s) {
        case Strategy::Direct: return true;
        case Strategy::TwiceErrorOnMismatch:
        case Strategy::TwiceRereadOnMismatch:
#ifdef TETHER_MODBUS_ROBUST_READ_TWICE
            return true;
#else
            return false;
#endif
        case Strategy::ThriceMajorityVote:
#ifdef TETHER_MODBUS_ROBUST_THRICE_VOTE
            return true;
#else
            return false;
#endif
        case Strategy::NQuorum:
#ifdef TETHER_MODBUS_ROBUST_QUORUM
            return true;
#else
            return false;
#endif
        case Strategy::Median:
#ifdef TETHER_MODBUS_ROBUST_MEDIAN
            return true;
#else
            return false;
#endif
        case Strategy::UntilStable:
#ifdef TETHER_MODBUS_ROBUST_UNTIL_STABLE
            return true;
#else
            return false;
#endif
        case Strategy::Sticky:
#ifdef TETHER_MODBUS_ROBUST_STICKY
            return true;
#else
            return false;
#endif
        case Strategy::WriteConfirm:
#ifdef TETHER_MODBUS_ROBUST_WRITE_CONFIRM
            return true;
#else
            return false;
#endif
        case Strategy::WriteThriceConfirm:
#ifdef TETHER_MODBUS_ROBUST_WRITE_THRICE_CONFIRM
            return true;
#else
            return false;
#endif
        case Strategy::WriteWatchdog:
#ifdef TETHER_MODBUS_ROBUST_WRITE_WATCHDOG
            return true;
#else
            return false;
#endif
        case Strategy::BroadcastFollowup:
#ifdef TETHER_MODBUS_ROBUST_BCAST_FOLLOWUP
            return true;
#else
            return false;
#endif
    }
    return false;
}

struct AssuranceOpts {
    Strategy strategy = Strategy::Direct;
    int      n = 3;              ///< samples for vote/median/quorum/stable
    int      quorum = 2;         ///< required agreeing samples (NQuorum)
    int      confirmDelayMs = 0; ///< watchdog / post-write settle delay
    int      sampleGapMs = 0;    ///< delay between samples
    int      maxConfirmRetries = 0;
    double   epsilon = 0.0;      ///< Median/quantized equality epsilon
    bool     allowStale = true;  ///< Sticky: return stale instead of error
};

} // namespace tether::modbus
