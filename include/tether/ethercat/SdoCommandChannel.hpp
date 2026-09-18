#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <span>
#include <stop_token>
#include <string>

#include "tether/ethercat/Slave.hpp"
#include "tether/utils/StateChangeLogger.hpp"

namespace EtherCAT {

/// Result of an SDO command/poll operation.
enum class SdoPollResult {
    Success,    ///< Matcher accepted a polled value
    Rejected,   ///< Device signalled failure (failure_response seen)
    Timeout,    ///< No match before the deadline
    SDOError,   ///< Command write or a required read failed
    Cancelled,  ///< stop_token requested while polling
};

const char* sdoPollResultToString(SdoPollResult r);

/// One SDO U32 register that must read `expected` before a deadline.
struct SdoExpectation {
    uint16_t    index;
    uint8_t     subindex = 0;
    uint32_t    expected = 0;
    const char* name = nullptr;   ///< optional label for log lines
};

/// Width-aware SDO read primitive: reads the object at (index, subindex)
/// declared `bytes` wide (1, 2 or 4) into `out`.  Lets the free polling
/// helpers run against any transport (a Slave, an emulator, a test
/// double) without needing a concrete Slave instance.
using SdoReadFn =
    std::function<SlaveError(uint16_t index, uint8_t subindex,
                             uint8_t bytes, uint32_t& out)>;

/// Adapt a Slave's sdoReadU8/U16/U32 into an SdoReadFn.
SdoReadFn makeSdoReadFn(Slave& slave);

/// Poll every expectation until all read `expected`, the timeout expires,
/// a read error occurs, or `stop` is requested.  Reads are retried each
/// interval; values are logged on change (or every 10th unchanged poll).
/// Returns true only when every expectation matched.
bool waitForSdoValues(const SdoReadFn& read,
                      std::span<const SdoExpectation> expectations,
                      std::chrono::milliseconds poll_interval,
                      std::chrono::milliseconds timeout,
                      std::stop_token stop = {},
                      const char* tag = "SdoCommandChannel");

/// Slave& convenience overload — equivalent to
/// waitForSdoValues(makeSdoReadFn(slave), ...).
bool waitForSdoValues(Slave& slave,
                      std::span<const SdoExpectation> expectations,
                      std::chrono::milliseconds poll_interval,
                      std::chrono::milliseconds timeout,
                      std::stop_token stop = {},
                      const char* tag = "SdoCommandChannel");

/// One object in an SDO read table (see pollSdoObjects).
struct SdoObjectEntry {
    uint16_t index;
    uint8_t  subindex;
    uint8_t  bytes;   ///< object width: 1, 2 or 4
};

/// Best-effort poll of a table of SDO objects.
///
/// `values`/`ok` must be at least objects.size() long.  Each entry is read
/// with sdoReadU8/U16/U32 according to its declared width; failures mark
/// ok[i]=false and polling continues.  When `shouldAbort` is set and
/// returns true, the poll stops between objects (entries not attempted
/// keep their previous values and ok flags).  `onError` (optional) is
/// invoked for each failed read.
///
/// Use `shouldAbort` to keep SDO reads from overlapping mailbox-sensitive
/// command sequences (e.g. pausing a status poll while a device command
/// object is in use).
///
/// @return number of objects read successfully this call.
size_t pollSdoObjects(
    const SdoReadFn& read,
    std::span<const SdoObjectEntry> objects,
    std::span<uint32_t> values,
    std::span<bool> ok,
    const std::function<bool()>& shouldAbort = {},
    const std::function<void(size_t, const SdoObjectEntry&, SlaveError)>&
        onError = {});

/// Slave& convenience overload — equivalent to
/// pollSdoObjects(makeSdoReadFn(slave), ...).
size_t pollSdoObjects(
    Slave& slave,
    std::span<const SdoObjectEntry> objects,
    std::span<uint32_t> values,
    std::span<bool> ok,
    const std::function<bool()>& shouldAbort = {},
    const std::function<void(size_t, const SdoObjectEntry&, SlaveError)>&
        onError = {});

/// Generic "command register + polled response register" SDO protocol
/// channel — the pattern device command objects follow (write a command
/// code, then poll a response register until it signals the outcome).
///
/// Configurable for protocol quirks: an optional reset value written to
/// the command register before (and optionally after) each command, a
/// response value meaning "command failed", and a pluggable matcher that
/// decides which response values count as success.
///
/// Also offers the two related polling helpers as methods:
///   pollUntil()      — poll an arbitrary U32 register until a predicate
///   waitUntilMatch() — poll a set of registers until each reads a value
class SdoCommandChannel {
public:
    using Result = SdoPollResult;

    struct Config {
        uint16_t command_index     = 0;   ///< command code register
        uint8_t  command_subindex  = 0;
        uint16_t response_index    = 0;   ///< polled response register
        uint8_t  response_subindex = 0;
        uint16_t error_index       = 0;   ///< error-code register (0 = none)
        uint8_t  error_subindex    = 0;

        std::chrono::milliseconds command_timeout{10000};
        std::chrono::milliseconds poll_interval{20};

        /// Write `reset_value` to the command register and wait
        /// `reset_delay` before every command.
        bool                        pre_command_reset  = false;
        uint32_t                    reset_value        = 0;
        std::chrono::milliseconds   reset_delay{0};
        /// Same reset applied after the command completes.
        bool                        post_command_reset = false;

        /// Response value meaning "command failed" — ends the poll
        /// immediately with Result::Rejected.  0xFFFFFFFF disables.
        uint32_t failure_response = 0xFFFFFFFF;
    };

    /// Decides whether a polled response value means success for `cmd`.
    using ResponseMatcher =
        std::function<bool(uint32_t cmd, uint32_t response, Slave& slave)>;

    /// Called after every response/poll read (logging, PDO correlation).
    using PollObserver = std::function<void()>;

    /// Renders a response value in log lines (e.g. an enum name).
    using ValueNamer = std::function<std::string(uint32_t)>;

    SdoCommandChannel(Slave& slave, Config config,
                      const char* tag = "SdoCommandChannel");

    // --- Command protocol ---

    /// Apply the configured pre-command reset (if enabled).  Called
    /// automatically by sendAndWait(); may also be called manually.
    bool resetResponse();

    /// Write a command code to the command register (no reset, no polling).
    bool sendCommand(uint32_t cmd);

    /// Read the response register once.  Returns false on SDO error.
    bool pollResponse(uint32_t& out);

    /// Poll the response register until `matcher` accepts, the configured
    /// failure response is seen, or the timeout expires.
    /// SDO read errors are retried (they do not abort the wait).
    Result waitForResponse(uint32_t cmd,
                           const ResponseMatcher& matcher,
                           std::chrono::milliseconds timeout = {});

    /// Full sequence: resetResponse() → sendCommand(cmd) →
    /// waitForResponse() → optional post-command reset.
    Result sendAndWait(uint32_t cmd,
                       const ResponseMatcher& matcher = {},
                       std::chrono::milliseconds timeout = {});

    /// Read the error-code register (0 when none configured / on error).
    uint32_t readErrorCode();

    // --- Generic polling ---

    /// Poll `index`:`subindex` until `match(value)` returns true.
    /// `onValue` (optional) receives every successfully-read value — use
    /// it for change-detected logging.  The registered PollObserver is
    /// invoked after each read, the stop token aborts the wait.
    Result pollUntil(uint16_t index, uint8_t subindex,
                     const std::function<bool(uint32_t)>& match,
                     std::chrono::milliseconds timeout,
                     const std::function<void(uint32_t)>& onValue = {});

    /// Poll a set of registers until each reads its expected value.
    /// Thin wrapper over waitForSdoValues() with this channel's
    /// poll interval, stop token and tag.
    bool waitUntilMatch(std::span<const SdoExpectation> expectations,
                        std::chrono::milliseconds timeout);

    // --- Accessors / hooks ---

    uint32_t lastResponse() const { return last_response_.load(); }
    void setStopToken(std::stop_token token) { stop_token_ = std::move(token); }
    void setPollObserver(PollObserver observer) { poll_observer_ = std::move(observer); }
    void setResponseNamer(ValueNamer namer) { response_namer_ = std::move(namer); }

    const Config& config() const { return config_; }

private:
    std::string responseName(uint32_t value) const;

    Slave& slave_;
    Config config_;
    const char* tag_;

    std::atomic<uint32_t> last_response_{0xFFFFFFFF};
    PollObserver poll_observer_;
    ValueNamer   response_namer_;
    std::stop_token stop_token_{};

    /// Suppresses repetitive poll logging — fires on response change or
    /// every 10th consecutive unchanged value.
    StateChangeLogger<uint32_t> poll_logger_;
};

} // namespace EtherCAT
