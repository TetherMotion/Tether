#pragma once

/**
 * @file MachineService.hpp
 * @brief Application-facing service boundary for the machine.cia402.v1 profile.
 *
 * Layers the fail-closed control primitives of MachineControl.hpp onto the
 * V6 catalog: typed authority leases (`machine.authority.*`), validated
 * command dispatch (`machine.command`), operation progress
 * (`machine.operation.*`), and an alarm lifecycle (`machine.alarms.*`).
 *
 * Real machines provide an `IMachineStateSource` and an `IMachineDispatcher`
 * backed by EtherCAT/DS402; `SimulatedMachineAdapter.hpp` provides the demo
 * implementation. All callbacks run outside the real-time loop.
 */

#include "tether/io/CiA402MachineProfile.hpp"
#include "tether/io/Datalogging.hpp"
#include "tether/io/EventJournal.hpp"
#include "tether/io/Function.hpp"
#include "tether/io/MachineControl.hpp"
#include "tether/io/Registry.hpp"
#include "tether/io/SchemaCatalog.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <format>
#include <functional>
#include <map>
#include <mutex>
#include <atomic>
#include <bit>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace tether::io::machine {

// ---------------------------------------------------------------------------
// Wire helpers for tagged schema values
// ---------------------------------------------------------------------------

namespace detail {

inline uint64_t nowUs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

inline void putVarint(std::vector<uint8_t>& bytes, uint64_t value) {
    while (value >= 0x80) {
        bytes.push_back(static_cast<uint8_t>(value) | 0x80U);
        value >>= 7;
    }
    bytes.push_back(static_cast<uint8_t>(value));
}

using FieldList = std::vector<std::pair<uint32_t, std::vector<uint8_t>>>;

template <typename T>
inline std::vector<uint8_t> fieldScalar(T value) {
    using Unsigned = std::make_unsigned_t<T>;
    const Unsigned bits = static_cast<Unsigned>(value);
    std::vector<uint8_t> bytes(sizeof(T));
    for (size_t index = 0; index < sizeof(T); ++index)
        bytes[index] = static_cast<uint8_t>(bits >> (index * 8U));
    return bytes;
}

inline std::vector<uint8_t> fieldScalar(bool value) {
    return {static_cast<uint8_t>(value ? 1 : 0)};
}

inline std::vector<uint8_t> fieldScalar(double value) {
    const uint64_t bits = std::bit_cast<uint64_t>(value);
    return fieldScalar(bits);
}

inline std::vector<uint8_t> fieldString(std::string_view value) {
    std::vector<uint8_t> bytes(value.begin(), value.end());
    bytes.push_back(0);
    return bytes;
}

inline std::vector<uint8_t> fieldBytes(const std::vector<uint8_t>& value) {
    std::vector<uint8_t> bytes;
    putVarint(bytes, value.size());
    bytes.insert(bytes.end(), value.begin(), value.end());
    return bytes;
}

/// Encode a tagged struct: field-count varint then [key varint][len varint][bytes].
inline std::vector<uint8_t> encodeTagged(const FieldList& fields) {
    std::vector<uint8_t> encoded;
    putVarint(encoded, fields.size());
    for (const auto& [key, value] : fields) {
        putVarint(encoded, key);
        putVarint(encoded, value.size());
        encoded.insert(encoded.end(), value.begin(), value.end());
    }
    return encoded;
}

/// Encode a dynamic array of already-encoded elements.
inline std::vector<uint8_t> encodeArray(const std::vector<std::vector<uint8_t>>& elements) {
    std::vector<uint8_t> encoded;
    putVarint(encoded, elements.size());
    for (const auto& element : elements) {
        putVarint(encoded, element.size());
        encoded.insert(encoded.end(), element.begin(), element.end());
    }
    return encoded;
}

/// Parse a tagged struct into key → bytes. Returns false on malformed input.
inline bool parseTagged(const std::vector<uint8_t>& bytes,
                        std::map<uint32_t, std::vector<uint8_t>>& fields) {
    size_t position = 0;
    const auto varint = [&bytes, &position](uint64_t& value) {
        value = 0;
        for (unsigned index = 0; index < 10; ++index) {
            if (position >= bytes.size()) return false;
            const uint8_t byte = bytes[position++];
            if (index == 9 && byte > 1) return false;
            value |= static_cast<uint64_t>(byte & 0x7F) << (index * 7U);
            if ((byte & 0x80) == 0) return true;
        }
        return false;
    };
    uint64_t count = 0;
    if (!varint(count) || count > 1024) return false;
    uint64_t previous = 0;
    for (uint64_t index = 0; index < count; ++index) {
        uint64_t key = 0, length = 0;
        if (!varint(key) || !varint(length) || key <= previous || key > UINT32_MAX ||
            length > bytes.size() - position) return false;
        previous = key;
        fields.emplace(static_cast<uint32_t>(key),
            std::vector<uint8_t>(bytes.begin() + static_cast<ptrdiff_t>(position),
                                 bytes.begin() + static_cast<ptrdiff_t>(position + length)));
        position += static_cast<size_t>(length);
    }
    return position == bytes.size();
}

template <typename T>
inline bool getScalar(const std::vector<uint8_t>& bytes, T& value) {
    if (bytes.size() != sizeof(T)) return false;
    using Unsigned = std::make_unsigned_t<T>;
    Unsigned bits = 0;
    for (size_t index = 0; index < sizeof(T); ++index)
        bits |= static_cast<Unsigned>(bytes[index]) << (index * 8U);
    value = static_cast<T>(bits);
    return true;
}

inline bool getBool(const std::vector<uint8_t>& bytes, bool& value) {
    if (bytes.size() != 1 || bytes[0] > 1) return false;
    value = bytes[0] != 0;
    return true;
}

inline bool getString(const std::vector<uint8_t>& bytes, std::string& value,
                      size_t maxLength = 512) {
    if (bytes.empty() || bytes.back() != 0) return false;
    const size_t length = bytes.size() - 1;
    if (length > maxLength) return false;
    value.assign(reinterpret_cast<const char*>(bytes.data()), length);
    return value.find('\0') == std::string::npos;
}

} // namespace detail

// ---------------------------------------------------------------------------
// Alarms
// ---------------------------------------------------------------------------

enum class AlarmState : uint8_t { Active = 0, Acknowledged = 1, Cleared = 2 };

struct AlarmRecordV1 {
    uint64_t alarmId = 0;
    uint64_t raisedUs = 0;
    uint64_t updatedUs = 0;
    AlarmState state = AlarmState::Active;
    EventSeverity severity = EventSeverity::Info;
    uint32_t code = 0;
    std::string sourceId;
    std::string description;
    std::string actor;
};

struct AlarmPageV1 {
    uint64_t oldestCursor = 0;
    uint64_t latestCursor = 0;
    uint64_t nextCursor = 0;
    bool gap = false;
    std::vector<AlarmRecordV1> alarms;
};

/// External notification event kinds (plan item 90).
enum class AlarmNotification : uint8_t { Raised, Escalated, Acknowledged, Cleared };

/// Notification policy: which severities notify, when an unacknowledged
/// alarm escalates, and how often escalation repeats.
struct AlarmNotifyPolicy {
    EventSeverity minSeverity = EventSeverity::Warning;
    uint64_t escalationDelayUs = 300'000'000;    ///< unacked for 5 min → escalate
    uint64_t escalationIntervalUs = 60'000'000;  ///< repeat every 1 min
    uint8_t maxEscalations = 3;
};

/// Pluggable sink for external alarm notification (pager, webhook, e-mail).
/// The sink is invoked outside the alarm lock; it must not re-enter the
/// AlarmService from the notification call itself.
class IAlarmNotifier {
public:
    virtual ~IAlarmNotifier() = default;
    virtual void notify(const AlarmRecordV1& alarm, AlarmNotification kind) = 0;
};

/// Bounded alarm lifecycle store. Raising the same source+code while an
/// alarm is active/acknowledged reuses the existing record (no duplicates)
/// and never re-notifies — deduplication is inherent to the store.
class AlarmService final {
public:
    explicit AlarmService(size_t capacity = 512) : capacity_(capacity) {
        if (capacity_ == 0) throw std::invalid_argument("alarm capacity must be nonzero");
    }

    /// Attach an external notifier. `audit` is an optional EventJournal that
    /// records every emitted notification as `alarm.notify.*` — the durable
    /// evidence for escalation and acknowledgement delivery attempts.
    void setNotifier(IAlarmNotifier* notifier, AlarmNotifyPolicy policy = {},
                     EventJournal* audit = nullptr) {
        std::lock_guard lock(mutex_);
        notifier_ = notifier;
        notifyPolicy_ = policy;
        auditJournal_ = audit;
    }

    /// Escalation scan: notifies `Escalated` for alarms that stayed Active
    /// past `escalationDelayUs`, repeating at `escalationIntervalUs` up to
    /// `maxEscalations`. Called opportunistically from `readAfter`; hosts may
    /// also invoke it on a timer.
    void tick(uint64_t nowUs = detail::nowUs()) {
        std::vector<std::pair<AlarmRecordV1, AlarmNotification>> pending;
        {
            std::lock_guard lock(mutex_);
            for (const auto& alarm : alarms_) {
                if (alarm.state != AlarmState::Active) continue;
                if (alarm.severity < notifyPolicy_.minSeverity) continue;
                auto& esc = escalations_[alarm.alarmId];
                if (esc.count >= notifyPolicy_.maxEscalations) continue;
                if (nowUs - alarm.raisedUs < notifyPolicy_.escalationDelayUs)
                    continue;
                if (esc.count > 0 &&
                    nowUs - esc.lastUs < notifyPolicy_.escalationIntervalUs)
                    continue;
                esc.lastUs = nowUs;
                ++esc.count;
                pending.emplace_back(alarm, AlarmNotification::Escalated);
            }
            for (auto it = escalations_.begin(); it != escalations_.end();) {
                if (!find(it->first)) it = escalations_.erase(it);
                else ++it;
            }
        }
        for (auto& [alarm, kind] : pending) emit(alarm, kind);
    }

    uint64_t raise(EventSeverity severity, uint32_t code, std::string sourceId,
                   std::string description, uint64_t nowUs = detail::nowUs()) {
        std::optional<AlarmRecordV1> raised;
        uint64_t id;
        {
            std::lock_guard lock(mutex_);
            for (const auto& alarm : alarms_) {
                if (alarm.sourceId == sourceId && alarm.code == code &&
                    alarm.state != AlarmState::Cleared) {
                    return alarm.alarmId;
                }
            }
            AlarmRecordV1 record;
            record.alarmId = nextId_++;
            record.raisedUs = record.updatedUs = nowUs;
            record.severity = severity;
            record.code = code;
            record.sourceId = std::move(sourceId);
            record.description = std::move(description);
            alarms_.push_back(std::move(record));
            if (alarms_.size() > capacity_) alarms_.pop_front();
            latestCursor_ = alarms_.back().alarmId;
            id = alarms_.back().alarmId;
            if (severity >= notifyPolicy_.minSeverity) raised = alarms_.back();
        }
        if (raised) emit(*raised, AlarmNotification::Raised);
        return id;
    }

    /// Returns 0 on success, 1 when absent/already terminal, 2 when denied.
    uint8_t acknowledge(uint64_t alarmId, const SessionIdentity& identity,
                        uint64_t nowUs = detail::nowUs()) {
        if (!identity.authenticated || identity.role == Role::Observer) return 2;
        std::optional<AlarmRecordV1> acked;
        {
            std::lock_guard lock(mutex_);
            auto* alarm = find(alarmId);
            if (!alarm || alarm->state == AlarmState::Cleared) return 1;
            alarm->state = AlarmState::Acknowledged;
            alarm->actor = identity.actor;
            alarm->updatedUs = nowUs;
            if (alarm->severity >= notifyPolicy_.minSeverity) acked = *alarm;
        }
        if (acked) emit(*acked, AlarmNotification::Acknowledged);
        return 0;
    }

    uint8_t clear(uint64_t alarmId, const SessionIdentity& identity,
                  uint64_t nowUs = detail::nowUs()) {
        if (!identity.authenticated || identity.role == Role::Observer) return 2;
        std::optional<AlarmRecordV1> cleared;
        {
            std::lock_guard lock(mutex_);
            auto* alarm = find(alarmId);
            if (!alarm) return 1;
            alarm->state = AlarmState::Cleared;
            alarm->actor = identity.actor;
            alarm->updatedUs = nowUs;
            if (alarm->severity >= notifyPolicy_.minSeverity) cleared = *alarm;
        }
        if (cleared) emit(*cleared, AlarmNotification::Cleared);
        return 0;
    }

    AlarmPageV1 readAfter(uint64_t cursor, size_t limit) {
        tick();
        std::lock_guard lock(mutex_);
        AlarmPageV1 page;
        if (alarms_.empty()) {
            page.latestCursor = latestCursor_;
            page.nextCursor = std::min(cursor, latestCursor_);
            return page;
        }
        page.oldestCursor = alarms_.front().alarmId;
        page.latestCursor = latestCursor_;
        page.gap = cursor < page.oldestCursor - 1;
        const uint64_t effective = page.gap ? page.oldestCursor - 1 : cursor;
        for (const auto& alarm : alarms_) {
            if (alarm.alarmId <= effective) continue;
            page.alarms.push_back(alarm);
            if (page.alarms.size() == limit) break;
        }
        page.nextCursor = page.alarms.empty() ? std::min(cursor, latestCursor_)
                                              : page.alarms.back().alarmId;
        return page;
    }

    size_t activeCount() const {
        std::lock_guard lock(mutex_);
        return static_cast<size_t>(std::count_if(alarms_.begin(), alarms_.end(),
            [](const AlarmRecordV1& alarm) { return alarm.state == AlarmState::Active; }));
    }

    uint64_t latestCursor() const {
        std::lock_guard lock(mutex_);
        return latestCursor_;
    }

private:
    AlarmRecordV1* find(uint64_t id) {
        const auto it = std::find_if(alarms_.begin(), alarms_.end(),
                                     [id](const AlarmRecordV1& a) { return a.alarmId == id; });
        return it == alarms_.end() ? nullptr : &*it;
    }

    /// Deliver a notification outside the alarm lock: external sink first,
    /// then the audit journal (`alarm.notify.<kind>`). Sinks must not throw.
    void emit(const AlarmRecordV1& alarm, AlarmNotification kind) {
        if (notifier_) {
            try { notifier_->notify(alarm, kind); } catch (...) {}
        }
        if (auditJournal_) {
            static const char* kinds[] = {"raised", "escalated", "acknowledged",
                                          "cleared"};
            const std::string description =
                alarm.description.empty()
                    ? "Alarm " + std::to_string(alarm.alarmId)
                    : alarm.description;
            try {
                auditJournal_->append(
                    {0, detail::nowUs(), 0,
                     std::string("alarm.notify.") + kinds[static_cast<uint8_t>(kind)],
                     alarm.sourceId, alarm.severity, alarm.code, description});
            } catch (...) {}
        }
    }

    const size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<AlarmRecordV1> alarms_;
    uint64_t nextId_ = 1;
    uint64_t latestCursor_ = 0;
    IAlarmNotifier* notifier_ = nullptr;
    AlarmNotifyPolicy notifyPolicy_{};
    EventJournal* auditJournal_ = nullptr;
    struct Escalation { uint8_t count = 0; uint64_t lastUs = 0; };
    std::unordered_map<uint64_t, Escalation> escalations_;
};

// ---------------------------------------------------------------------------
// Operations
// ---------------------------------------------------------------------------

enum class OperationState : uint8_t {
    Queued = 0, Running = 1, Completed = 2, Failed = 3, Cancelled = 4,
};

struct OperationRecord {
    std::string operationId;
    std::string requestUuid;
    std::string ownerSession;
    OperationState state = OperationState::Queued;
    uint8_t progress = 0;
    Action action = Action::Enable;
    std::string target;
    std::string message;
    uint64_t startedUs = 0;
    uint64_t finishedUs = 0;
    uint32_t resultCode = 0;
};

inline bool isTerminal(OperationState state) {
    return state == OperationState::Completed || state == OperationState::Failed ||
           state == OperationState::Cancelled;
}

/// Bounded operation ledger. Dispatchers create a record on acceptance and
/// drive it to a terminal state; clients read progress through
/// machine.operation.* functions.
class OperationTracker final {
public:
    explicit OperationTracker(size_t capacity = 256) : capacity_(capacity) {}

    std::string begin(std::string requestUuid, std::string ownerSession,
                      Action action, std::string target) {
        std::lock_guard lock(mutex_);
        OperationRecord record;
        record.operationId = "op-" + std::to_string(nextId_++);
        record.requestUuid = std::move(requestUuid);
        record.ownerSession = std::move(ownerSession);
        record.action = action;
        record.target = std::move(target);
        record.state = OperationState::Running;
        record.startedUs = detail::nowUs();
        const std::string id = record.operationId;
        order_.push_back(id);
        operations_[id] = std::move(record);
        if (order_.size() > capacity_) {
            operations_.erase(order_.front());
            order_.pop_front();
        }
        return id;
    }

    bool finish(const std::string& id, OperationState terminal,
                std::string message, uint32_t resultCode = 0) {
        if (!isTerminal(terminal)) return false;
        std::lock_guard lock(mutex_);
        const auto it = operations_.find(id);
        if (it == operations_.end() || isTerminal(it->second.state)) return false;
        it->second.state = terminal;
        it->second.progress = terminal == OperationState::Completed ? 100 : it->second.progress;
        it->second.message = std::move(message);
        it->second.resultCode = resultCode;
        it->second.finishedUs = detail::nowUs();
        return true;
    }

    bool update(const std::string& id, uint8_t progress, std::string message) {
        std::lock_guard lock(mutex_);
        const auto it = operations_.find(id);
        if (it == operations_.end() || isTerminal(it->second.state)) return false;
        it->second.progress = std::min<uint8_t>(progress, 99);
        it->second.message = std::move(message);
        return true;
    }

    /// Cancel a non-terminal operation. Dispatchers that cannot abort return
    /// false via `tryCancel`; this only marks the record.
    bool tryCancel(const std::string& id, const SessionIdentity& identity,
                   bool allowOwnerOrPrivileged = true) {
        std::lock_guard lock(mutex_);
        const auto it = operations_.find(id);
        if (it == operations_.end() || isTerminal(it->second.state)) return false;
        const bool owner = it->second.ownerSession == identity.sessionId;
        const bool privileged = identity.role == Role::Technician ||
                                identity.role == Role::ControlsEngineer ||
                                identity.role == Role::Administrator;
        if (!identity.authenticated || (allowOwnerOrPrivileged && !owner && !privileged))
            return false;
        it->second.state = OperationState::Cancelled;
        it->second.message = "Cancelled by " + identity.actor;
        it->second.finishedUs = detail::nowUs();
        return true;
    }

    /// Cancel every non-terminal operation owned by a disconnecting session.
    size_t cancelAllForSession(const std::string& sessionId, std::string_view reason) {
        std::lock_guard lock(mutex_);
        size_t count = 0;
        for (auto& [id, operation] : operations_) {
            if (operation.ownerSession == sessionId && !isTerminal(operation.state)) {
                operation.state = OperationState::Cancelled;
                operation.message = std::string(reason);
                operation.finishedUs = detail::nowUs();
                ++count;
            }
        }
        return count;
    }

    std::optional<OperationRecord> get(const std::string& id) const {
        std::lock_guard lock(mutex_);
        const auto it = operations_.find(id);
        return it == operations_.end() ? std::nullopt : std::optional{it->second};
    }

    std::vector<OperationRecord> list() const {
        std::lock_guard lock(mutex_);
        std::vector<OperationRecord> out;
        out.reserve(order_.size());
        for (const auto& id : order_) {
            if (const auto it = operations_.find(id); it != operations_.end())
                out.push_back(it->second);
        }
        return out;
    }

private:
    const size_t capacity_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, OperationRecord> operations_;
    std::deque<std::string> order_;
    uint64_t nextId_ = 1;
};

// ---------------------------------------------------------------------------
// Application adapters
// ---------------------------------------------------------------------------

/// Read-only machine state the command gate validates against.
class IMachineStateSource {
public:
    virtual ~IMachineStateSource() = default;
    /// Stable resource identifiers of all commandable axes.
    virtual std::vector<std::string> axisIds() const = 0;
    virtual std::optional<cia402::DriveSnapshotV1> driveSnapshot(
        std::string_view stableId) const = 0;
    /// Current command environment (generations, supported actions, blockers).
    virtual std::optional<CommandEnvironment> commandEnvironment(
        const CommandRequest& request) = 0;
    /// Machine-wide configuration revision checked by the gate.
    virtual uint64_t configurationRevision() const = 0;
};

/// Dispatches a preflight-validated command to native motion/EtherCAT code.
/// Implementations must create an operation in the tracker, enqueue bounded
/// work off the real-time loop, and return immediately.
class IMachineDispatcher {
public:
    virtual ~IMachineDispatcher() = default;
    virtual DispatchResult dispatch(const CommandRequest& request,
                                    const SessionIdentity& identity,
                                    OperationTracker& operations) = 0;
    /// Attempt to abort an in-flight operation. Returns false when the
    /// operation cannot be interrupted (the tracker still marks it cancelled).
    virtual bool cancelOperation(std::string_view operationId) = 0;
};

// ---------------------------------------------------------------------------
// Capture service — server-side recording control
// ---------------------------------------------------------------------------

/// Trigger rule for a capture: when `entryId`'s numeric value crosses
/// `level` (op 0 = above, 1 = below, 2 = |value| above), `preRecords` records
/// already in the ring are kept and `postRecords` more are collected before
/// the recorder stops. This is a diagnostic aid, not a protection function.
struct CaptureTrigger {
    uint64_t entryId = 0;
    uint8_t op = 0;
    double level = 0;
    uint32_t preRecords = 0;
    uint32_t postRecords = 100;
};

enum class CaptureTriggerState : uint8_t {
    None = 0, Armed = 1, PostTrigger = 2, Complete = 3,
};

/// One paginated export chunk from the retained record buffer.
struct CaptureExportChunk {
    uint64_t offset = 0;
    uint64_t totalRecords = 0;
    uint64_t droppedRecords = 0;
    uint32_t recordSize = 0;
    std::vector<uint8_t> payload;
};

/// Binds a DatalogRecorder to registry reads with a bounded in-memory sink.
/// The application owns both objects; this class only adapts them for the
/// machine.capture.* functions. Recorded bytes stay on the server — exports
/// are explicit, paginated, access-controlled, and audited.
class MachineCaptureService final {
public:
    static constexpr size_t kMaxExportChunkBytes = 65536;

    MachineCaptureService(Registry& registry, DatalogRecorder& recorder,
                          size_t retainedRecords = 2048)
        : registry_(registry), recorder_(recorder), retainedRecords_(retainedRecords) {
        if (retainedRecords_ == 0)
            throw std::invalid_argument("capture retention must be nonzero");
    }

    /// Configure the recorder; entryIds empty = all fixed-size entries.
    /// An optional trigger switches the sink into pre/post-trigger mode.
    bool configure(std::string logName, uint32_t sampleRateHz, bool enabled,
                   std::vector<uint64_t> entryIds,
                   std::optional<CaptureTrigger> trigger = std::nullopt) {
        if (logName.size() > 128 || sampleRateHz == 0 || sampleRateHz > 100'000 ||
            entryIds.size() > 512)
            return false;
        for (uint64_t id : entryIds) {
            EntryView entry = registry_.find(id);
            if (!entry || entry.isVariableLength()) return false;
        }
        if (trigger) {
            EntryView entry = registry_.find(trigger->entryId);
            if (!entry || entry.isVariableLength() || trigger->op > 2 ||
                trigger->preRecords > retainedRecords_ ||
                trigger->postRecords > retainedRecords_)
                return false;
        }
        {
            std::lock_guard lock(mutex_);
            records_.clear();
            droppedRecords_ = 0;
            trigger_ = trigger;
            triggerState_ =
                trigger ? CaptureTriggerState::Armed : CaptureTriggerState::None;
            postRemaining_ = 0;
        }
        DatalogConfig config;
        config.logName = std::move(logName);
        config.sampleRateHz = sampleRateHz;
        config.enabled = enabled;
        config.entryIds = std::move(entryIds);
        auto readFn = [this](uint64_t id, void* dest, size_t maxSize) {
            EntryView entry = registry_.find(id);
            if (!entry || entry.isVariableLength() || entry.valueSize() > maxSize)
                return false;
            entry.read(dest);
            return true;
        };
        auto sink = [this](const uint8_t* record, size_t size) {
            this->sinkRecord(record, size);
        };
        recorder_.configure(config, readFn, detail::nowUs, sink);
        if (enabled) recorder_.start(); else recorder_.stop();
        return true;
    }

    /// Stop recording and discard retained records.
    void cancel() {
        recorder_.stop();
        std::lock_guard lock(mutex_);
        records_.clear();
        droppedRecords_ = 0;
        trigger_.reset();
        triggerState_ = CaptureTriggerState::None;
    }

    DatalogStatus status() const { return recorder_.status(); }

    CaptureTriggerState triggerState() const {
        std::lock_guard lock(mutex_);
        return triggerState_;
    }

    /// Records currently retained and exportable.
    uint64_t recordsAvailable() const {
        std::lock_guard lock(mutex_);
        return records_.size();
    }

    uint64_t recordsDropped() const {
        std::lock_guard lock(mutex_);
        return droppedRecords_;
    }

    /// Records retained in memory (test/support-bundle access).
    std::deque<std::vector<uint8_t>> records() const {
        std::lock_guard lock(mutex_);
        return records_;
    }

    /// Concatenate up to `maxBytes` of records starting at `recordOffset`.
    /// Chunk size is hard-capped at kMaxExportChunkBytes.
    CaptureExportChunk exportChunk(uint64_t recordOffset, uint32_t maxBytes) const {
        CaptureExportChunk chunk;
        std::lock_guard lock(mutex_);
        chunk.offset = recordOffset;
        chunk.totalRecords = records_.size();
        chunk.droppedRecords = droppedRecords_;
        chunk.recordSize = records_.empty()
            ? 0u : static_cast<uint32_t>(records_.front().size());
        const size_t cap = std::min<size_t>(maxBytes, kMaxExportChunkBytes);
        for (uint64_t index = recordOffset; index < records_.size(); ++index) {
            const auto& record = records_[static_cast<size_t>(index)];
            if (chunk.payload.size() + record.size() > cap) break;
            chunk.payload.insert(chunk.payload.end(), record.begin(), record.end());
        }
        return chunk;
    }

private:
    void sinkRecord(const uint8_t* record, size_t size) {
        std::lock_guard lock(mutex_);
        if (trigger_ && triggerState_ == CaptureTriggerState::Complete) return;
        records_.emplace_back(record, record + size);
        if (!trigger_) {
            while (records_.size() > retainedRecords_) {
                records_.pop_front();
                ++droppedRecords_;
            }
            return;
        }
        if (triggerState_ == CaptureTriggerState::Armed) {
            while (records_.size() > trigger_->preRecords) {
                records_.pop_front();
                ++droppedRecords_;
            }
            if (evaluateTrigger()) {
                triggerState_ = trigger_->postRecords == 0
                    ? CaptureTriggerState::Complete
                    : CaptureTriggerState::PostTrigger;
                postRemaining_ = trigger_->postRecords;
            }
        } else if (triggerState_ == CaptureTriggerState::PostTrigger) {
            if (--postRemaining_ == 0) {
                triggerState_ = CaptureTriggerState::Complete;
                recorder_.stop();
            }
        }
    }

    /// Read the trigger entry and compare its numeric value to the level.
    /// Caller holds mutex_.
    bool evaluateTrigger() const {
        EntryView entry = registry_.find(trigger_->entryId);
        if (!entry || entry.isVariableLength() || entry.valueSize() > 8) return false;
        uint64_t raw = 0;
        entry.read(&raw);
        double value = 0;
        switch (entry.valueType()) {
            case ValueType::F64: { double f; std::memcpy(&f, &raw, 8); value = f; break; }
            case ValueType::F32: { float f = 0; std::memcpy(&f, &raw, 4); value = f; break; }
            case ValueType::I8:  value = static_cast<int8_t>(raw); break;
            case ValueType::I16: value = static_cast<int16_t>(raw); break;
            case ValueType::I32: value = static_cast<int32_t>(raw); break;
            case ValueType::I64: value = static_cast<double>(static_cast<int64_t>(raw)); break;
            case ValueType::U8:  value = static_cast<uint8_t>(raw); break;
            case ValueType::U16: value = static_cast<uint16_t>(raw); break;
            case ValueType::U32: value = static_cast<uint32_t>(raw); break;
            case ValueType::U64: value = static_cast<double>(raw); break;
            case ValueType::Bool: value = raw != 0 ? 1.0 : 0.0; break;
            default: return false;
        }
        switch (trigger_->op) {
            case 0: return value > trigger_->level;
            case 1: return value < trigger_->level;
            default: return std::abs(value) > trigger_->level;
        }
    }

    Registry& registry_;
    DatalogRecorder& recorder_;
    const size_t retainedRecords_;
    mutable std::mutex mutex_;
    std::deque<std::vector<uint8_t>> records_;
    uint64_t droppedRecords_ = 0;
    std::optional<CaptureTrigger> trigger_;
    CaptureTriggerState triggerState_ = CaptureTriggerState::None;
    uint32_t postRemaining_ = 0;
};

// ---------------------------------------------------------------------------
// Staged configuration service — stage/validate/commit/rollback transactions
// ---------------------------------------------------------------------------

/// One staged parameter write (ConfigEntryV1).
struct ConfigWrite {
    uint64_t entryId = 0;
    std::vector<uint8_t> value;
};

enum class ConfigTxnState : uint8_t {
    Idle = 0, Staged = 1, Validated = 2, Committed = 3, Failed = 4, RolledBack = 5,
};

struct ConfigTxnStatus {
    uint64_t transactionId = 0;
    ConfigTxnState state = ConfigTxnState::Idle;
    uint64_t revision = 0;
    std::string message;
    std::vector<ConfigWrite> staged;
};

/// Transactional, staged parameter writes. Stage performs structural checks
/// (entry exists, is a writable parameter, size constraints); an optional
/// application validator runs on validate(); commit applies the writes in
/// order and bumps the configuration revision; rollback discards the
/// transaction. Nothing is written before commit.
class StagedConfigService final {
public:
    /// Application cross-field validator: returns an error string or nullopt.
    using Validator = std::function<std::optional<std::string>(
        const std::vector<ConfigWrite>&)>;
    /// Revision provider/bump callbacks supplied by the state source.
    using RevisionFn = std::function<uint64_t()>;
    using BumpFn = std::function<void()>;

    explicit StagedConfigService(Registry& registry) : registry_(registry) {}

    void setValidator(Validator validator) { validator_ = std::move(validator); }
    void setRevisionHooks(RevisionFn current, BumpFn bump) {
        revisionFn_ = std::move(current);
        bumpFn_ = std::move(bump);
        std::lock_guard lock(mutex_);
        status_.revision = revisionFn_ ? revisionFn_() : 0;
    }

    ConfigTxnStatus stage(std::vector<ConfigWrite> writes, const SessionIdentity&) {
        std::lock_guard lock(mutex_);
        if (writes.empty() || writes.size() > 256)
            return fail("A staged transaction must contain 1-256 entries");
        for (const auto& write : writes) {
            EntryView entry = registry_.find(write.entryId);
            if (!entry || entry.kind() != EntryKind::Parameter)
                return fail("Entry is not a known parameter");
            if (!entry.writable())
                return fail("Parameter is read-only");
            if (!entry.isVariableLength() && write.value.size() != entry.valueSize())
                return fail("Parameter value size does not match its type");
            if (entry.isVariableLength() &&
                write.value.size() > entry.maxValueSize())
                return fail("Parameter value exceeds its maximum size");
            // Capability check: numeric parameters may declare range.min /
            // range.max metadata; out-of-range writes are rejected before
            // they ever reach commit.
            double boundMin = 0, boundMax = 0;
            bool hasMin = false, hasMax = false;
            entry.forEachMetadata([&](std::string_view key, std::string_view value) {
                if (key == "range.min") { hasMin = parseDouble(value, boundMin); }
                else if (key == "range.max") { hasMax = parseDouble(value, boundMax); }
            });
            if ((hasMin || hasMax) && !entry.isVariableLength()) {
                double numeric = 0;
                if (decodeNumericValue(entry.valueType(), write.value, numeric)) {
                    if (hasMin && numeric < boundMin)
                        return fail("Parameter value below its declared minimum");
                    if (hasMax && numeric > boundMax)
                        return fail("Parameter value above its declared maximum");
                }
            }
        }
        ++status_.transactionId;
        status_.staged = std::move(writes);
        status_.state = ConfigTxnState::Staged;
        status_.message.clear();
        return status_;
    }

    ConfigTxnStatus validate(const SessionIdentity&) {
        std::lock_guard lock(mutex_);
        if (status_.state != ConfigTxnState::Staged)
            return fail("No staged transaction to validate");
        if (validator_) {
            if (auto error = validator_(status_.staged))
                return fail(*error);
        }
        status_.state = ConfigTxnState::Validated;
        status_.message = "Transaction validated";
        return status_;
    }

    ConfigTxnStatus commit(const SessionIdentity& identity) {
        if (!identity.authenticated || identity.role < Role::Technician) {
            std::lock_guard lock(mutex_);
            return fail("Configuration commit requires a technician role");
        }
        std::lock_guard lock(mutex_);
        if (status_.state != ConfigTxnState::Validated)
            return fail("No validated transaction to commit");
        for (const auto& write : status_.staged) {
            EntryView entry = registry_.find(write.entryId);
            if (!entry || !entry.writable())
                return fail("A staged parameter became unwritable");
            if (entry.isVariableLength()) entry.writeVar(write.value.data(), write.value.size());
            else entry.write(write.value.data());
        }
        if (bumpFn_) bumpFn_();
        status_.revision = revisionFn_ ? revisionFn_() : status_.revision + 1;
        status_.state = ConfigTxnState::Committed;
        status_.message = "Configuration committed";
        status_.staged.clear();
        return status_;
    }

    ConfigTxnStatus rollback(const SessionIdentity&) {
        std::lock_guard lock(mutex_);
        if (status_.state == ConfigTxnState::Idle)
            return fail("No transaction to roll back");
        status_.staged.clear();
        status_.state = ConfigTxnState::RolledBack;
        status_.message = "Transaction rolled back";
        return status_;
    }

    ConfigTxnStatus status() const {
        std::lock_guard lock(mutex_);
        return status_;
    }

private:
    ConfigTxnStatus fail(std::string message) {
        status_.state = ConfigTxnState::Failed;
        status_.message = std::move(message);
        return status_;
    }

    static bool parseDouble(std::string_view text, double& out) {
        std::string copy(text);
        char* end = nullptr;
        out = std::strtod(copy.c_str(), &end);
        return end != copy.c_str() && *end == '\0';
    }

    /// Decode a fixed-size numeric parameter value to double; false when the
    /// type is non-numeric or the size does not match.
    static bool decodeNumericValue(ValueType type, const std::vector<uint8_t>& bytes,
                                   double& out) {
        if (bytes.size() != valueTypeSize(type)) return false;
        switch (type) {
            case ValueType::U8:  { uint8_t v;  std::memcpy(&v, bytes.data(), 1); out = v; return true; }
            case ValueType::U16: { uint16_t v; std::memcpy(&v, bytes.data(), 2); out = v; return true; }
            case ValueType::U32: { uint32_t v; std::memcpy(&v, bytes.data(), 4); out = v; return true; }
            case ValueType::U64: { uint64_t v; std::memcpy(&v, bytes.data(), 8); out = static_cast<double>(v); return true; }
            case ValueType::I8:  { int8_t v;   std::memcpy(&v, bytes.data(), 1); out = v; return true; }
            case ValueType::I16: { int16_t v;  std::memcpy(&v, bytes.data(), 2); out = v; return true; }
            case ValueType::I32: { int32_t v;  std::memcpy(&v, bytes.data(), 4); out = v; return true; }
            case ValueType::I64: { int64_t v;  std::memcpy(&v, bytes.data(), 8); out = static_cast<double>(v); return true; }
            case ValueType::F32: { float v;    std::memcpy(&v, bytes.data(), 4); out = v; return true; }
            case ValueType::F64: { std::memcpy(&out, bytes.data(), 8); return true; }
            case ValueType::Bool:{ out = bytes[0] ? 1.0 : 0.0; return true; }
            default: return false;
        }
    }

    Registry& registry_;
    Validator validator_;
    RevisionFn revisionFn_;
    BumpFn bumpFn_;
    mutable std::mutex mutex_;
    ConfigTxnStatus status_;
};

// ---------------------------------------------------------------------------
// SDO inspection — pluggable backend interface + role/audit policy
// ---------------------------------------------------------------------------

/// One known object-dictionary entry as reported by machine.sdo.list.
struct SdoObjectInfoV1 {
    uint16_t index = 0;
    uint8_t subindex = 0;
    uint16_t dataType = 0;          ///< CiA 301 data type code
    uint8_t access = 3;             ///< 1 read-only, 3 read/write
    std::string name;
    std::string description;        ///< ESI/device-profile description (may be empty)
    double minValue = std::numeric_limits<double>::quiet_NaN();
    double maxValue = std::numeric_limits<double>::quiet_NaN();
};

struct SdoTransferResult {
    bool ok = false;
    uint32_t abortCode = 0;         ///< CoE SDO abort code when the slave aborts
    std::vector<uint8_t> data;      ///< Read payload (reads only)
    std::vector<uint8_t> readback;  ///< Post-write verification payload (writes only)
    std::string error;              ///< Transport/adapter failure text
};

/// CiA 301 / ETG.1000-6 SDO abort-code names for operator-facing display.
inline const char* sdoAbortName(uint32_t code) {
    switch (code) {
        case 0x00000000: return "";
        case 0x05030000: return "Toggle bit not alternated";
        case 0x05040000: return "SDO protocol timed out";
        case 0x05040001: return "Invalid or unknown command specifier";
        case 0x05040005: return "Out of memory";
        case 0x06010000: return "Unsupported access to an object";
        case 0x06010001: return "Attempt to read a write-only object";
        case 0x06010002: return "Attempt to write a read-only object";
        case 0x06020000: return "Object does not exist in the object dictionary";
        case 0x06040041: return "Object cannot be mapped to the PDO";
        case 0x06040042: return "Mapped objects exceed PDO length";
        case 0x06040043: return "General parameter incompatibility";
        case 0x06060000: return "Access failed due to a hardware error";
        case 0x06070010: return "Data type or parameter length mismatch";
        case 0x06090011: return "Subindex does not exist";
        case 0x06090030: return "Parameter value out of range";
        case 0x06090031: return "Parameter value too high";
        case 0x06090032: return "Parameter value too low";
        case 0x08000000: return "General error";
        case 0x08000020: return "Data cannot be transferred or stored to the application";
        case 0x08000021: return "Access restricted by local control";
        case 0x08000022: return "Access restricted by the present device state";
        case 0x08000023: return "Object dictionary generation failed";
        default: return "Unknown abort code";
    }
}

/// Pluggable backend for the SDO inspector. Implementations must document
/// their threading contract — EtherCatSdoAccess performs mailbox round-trips
/// on the calling IO session thread and must not be invoked while holding a
/// lock shared with the realtime loop.
class IMachineSdoAccess {
public:
    virtual ~IMachineSdoAccess() = default;

    /// Objects known for `slave`, typically from a static device profile.
    /// An empty list means no object table is available; explicit-index
    /// reads and writes remain possible.
    virtual std::vector<SdoObjectInfoV1> listObjects(uint16_t slave) = 0;

    virtual SdoTransferResult read(uint16_t slave, uint16_t index,
                                   uint8_t subindex, size_t maxBytes) = 0;
    virtual SdoTransferResult write(uint16_t slave, uint16_t index,
                                    uint8_t subindex, const std::vector<uint8_t>& data) = 0;
};

/// Role/audit policy around a pluggable IMachineSdoAccess.
class MachineSdoService final {
public:
    static constexpr size_t kMaxTransferBytes = 512;
    /// One write per 50 ms server-wide — SDO writes are commissioning
    /// operations; this bounds mailbox pressure and replayed bursts.
    static constexpr uint64_t kWriteMinIntervalUs = 50'000;

    explicit MachineSdoService(IMachineSdoAccess& access,
                               EventJournal* auditJournal = nullptr)
        : access_(access), auditJournal_(auditJournal) {}

    std::vector<uint8_t> encodeList(uint16_t slave) const {
        std::vector<std::vector<uint8_t>> entries;
        for (const auto& info : access_.listObjects(slave)) {
            entries.push_back(detail::encodeTagged({
                {1, detail::fieldScalar(info.index)},
                {2, detail::fieldScalar(info.subindex)},
                {3, detail::fieldScalar(info.dataType)},
                {4, detail::fieldScalar(info.access)},
                {5, detail::fieldString(info.name)},
                {6, detail::fieldString(info.description)},
                {7, detail::fieldScalar(info.minValue)},
                {8, detail::fieldScalar(info.maxValue)},
            }));
        }
        return detail::encodeArray(entries);
    }

    static std::vector<uint8_t> encodeResult(const SdoTransferResult& result) {
        return detail::encodeTagged({
            {1, detail::fieldScalar(result.ok)},
            {2, detail::fieldScalar(result.abortCode)},
            {3, detail::fieldBytes(result.data)},
            {4, detail::fieldString(result.error)},
            {5, detail::fieldString(sdoAbortName(result.abortCode))},
            {6, detail::fieldBytes(result.readback)},
        });
    }

    /// machine.sdo.list — any authenticated session; empty when the backend
    /// has no object table for the slave.
    FunctionCallResult list(uint16_t slave, const SessionIdentity& identity) const {
        FunctionCallResult result;
        if (!identity.authenticated) {
            result.errorMessage = "machine.sdo.list requires an authenticated session";
            return result;
        }
        result.returnValue = encodeList(slave);
        result.success = true;
        result.error = ErrorCode::None;
        return result;
    }

    /// machine.sdo.read — any authenticated session.
    FunctionCallResult read(const std::vector<uint8_t>& request,
                            const SessionIdentity& identity) const {
        FunctionCallResult result;
        if (!identity.authenticated) {
            result.errorMessage = "machine.sdo.read requires an authenticated session";
            return result;
        }
        std::map<uint32_t, std::vector<uint8_t>> fields;
        uint16_t slave = 0, index = 0, maxBytes = 0;
        uint8_t subindex = 0;
        if (!detail::parseTagged(request, fields) ||
            !detail::getScalar(fields[1], slave) ||
            !detail::getScalar(fields[2], index) ||
            !detail::getScalar(fields[3], subindex) ||
            !detail::getScalar(fields[4], maxBytes) || maxBytes == 0 ||
            maxBytes > kMaxTransferBytes) {
            result.errorMessage =
                "machine.sdo.read requires slave/index/subindex/max_bytes (<= 512)";
            return result;
        }
        result.returnValue = encodeResult(access_.read(slave, index, subindex, maxBytes));
        result.success = true;
        result.error = ErrorCode::None;
        return result;
    }

    /// machine.sdo.write — technician-or-above only; journaled, rate-limited,
    /// and followed by a verification read on success. Optional request field
    /// 5 (`verify_readback`, bool) disables the readback (default on).
    FunctionCallResult write(const std::vector<uint8_t>& request,
                             const SessionIdentity& identity) {
        FunctionCallResult result;
        if (!identity.authenticated || identity.role < Role::Technician) {
            result.errorMessage = "machine.sdo.write requires technician role or above";
            return result;
        }
        std::map<uint32_t, std::vector<uint8_t>> fields;
        uint16_t slave = 0, index = 0;
        uint8_t subindex = 0;
        if (!detail::parseTagged(request, fields) ||
            !detail::getScalar(fields[1], slave) ||
            !detail::getScalar(fields[2], index) ||
            !detail::getScalar(fields[3], subindex) ||
            fields[4].empty() || fields[4].size() > kMaxTransferBytes + 9) {
            result.errorMessage =
                "machine.sdo.write requires slave/index/subindex plus 1-512 data bytes";
            return result;
        }
        // Field 4 is a length-prefixed blob (varint length + payload).
        const auto& blob = fields[4];
        uint64_t payloadLen = 0;
        size_t varintBytes = 0;
        for (size_t i = 0; i < blob.size() && i < 10; ++i) {
            payloadLen |= static_cast<uint64_t>(blob[i] & 0x7F) << (i * 7U);
            ++varintBytes;
            if ((blob[i] & 0x80) == 0) break;
        }
        if (varintBytes == 0 || (blob[varintBytes - 1] & 0x80) != 0 ||
            payloadLen == 0 || payloadLen > kMaxTransferBytes ||
            payloadLen != blob.size() - varintBytes) {
            result.errorMessage = "machine.sdo.write data field must be a length-prefixed blob";
            return result;
        }
        const std::vector<uint8_t> payload(blob.begin() + static_cast<ptrdiff_t>(varintBytes),
                                           blob.end());
        const uint64_t now = detail::nowUs();
        const uint64_t last = lastWriteUs_.load(std::memory_order_relaxed);
        if (now < last + kWriteMinIntervalUs) {
            result.errorMessage =
                "machine.sdo.write rate limited: one write per 50 ms per server";
            return result;
        }
        lastWriteUs_.store(now, std::memory_order_relaxed);
        auto transfer = access_.write(slave, index, subindex, payload);
        bool verify = true;
        if (auto it = fields.find(5); it != fields.end())
            verify = it->second.empty() || it->second[0] != 0;
        if (transfer.ok && verify) {
            auto back = access_.read(slave, index, subindex, payload.size());
            transfer.readback = std::move(back.data);
            if (back.ok && transfer.readback != payload) {
                transfer.error =
                    "post-write readback differs from the written value (object may "
                    "be live-updated or clipped by the slave)";
            }
        }
        if (auditJournal_) {
            EventRecordV1 record;
            record.timestampUs = detail::nowUs();
            record.eventType = "audit.sdo.write";
            record.sourceId = std::to_string(slave);
            record.severity = transfer.ok ? EventSeverity::Info : EventSeverity::Warning;
            record.code = transfer.abortCode;
            record.description = identity.actor + " SDO write " +
                std::format("{:#06x}:{:#04x}", index, subindex) +
                (transfer.ok ? " ok" : " failed: " + transfer.error);
            auditJournal_->append(std::move(record));
        }
        result.returnValue = encodeResult(transfer);
        result.success = true;
        result.error = ErrorCode::None;
        return result;
    }

private:
    IMachineSdoAccess& access_;
    EventJournal* auditJournal_;
    std::atomic<uint64_t> lastWriteUs_{0};
};

// ---------------------------------------------------------------------------
// Machine diagnostics — pluggable cyclic-loop / DC / mailbox / link counters
// ---------------------------------------------------------------------------

/// Wire layout of machine.diagnostics (tagged MachineDiagnosticsV1). All
/// counters are cumulative since the source started; a source that cannot
/// observe a counter reports zero.
struct MachineDiagnosticsV1 {
    uint64_t timestampUs = 0;
    uint64_t cycleCount = 0;
    uint64_t missedDeadlines = 0;
    uint32_t maxCycleWorkUs = 0;
    uint32_t jitterMaxUs = 0;
    uint32_t jitterAvgUs = 0;
    uint64_t dcSyncCount = 0;
    uint64_t dcSyncErrors = 0;
    uint32_t dcJitterMaxUs = 0;
    uint64_t mbxSends = 0;
    uint64_t mbxSendErrors = 0;
    uint64_t mbxCollects = 0;
    uint64_t mbxCollectErrors = 0;
    uint32_t maxSendLatencyNs = 0;
    uint32_t txRetries = 0;
    uint32_t txFailures = 0;
    uint32_t rxFrames = 0;
    uint16_t slavesDetected = 0;
    uint16_t slavesOperational = 0;
    uint32_t lastWkc = 0;
};

/// Pluggable source for machine.diagnostics. EtherCatDiagnosticsAdapter
/// reads Master's real counters; the sim stack fabricates plausible ones.
/// Not safety-rated — these are observability counters, never interlocks.
class IMachineDiagnosticsSource {
public:
    virtual ~IMachineDiagnosticsSource() = default;
    virtual MachineDiagnosticsV1 read() = 0;
};

inline std::vector<uint8_t> encodeDiagnostics(const MachineDiagnosticsV1& d) {
    return detail::encodeTagged({
        {1, detail::fieldScalar(d.timestampUs)},
        {2, detail::fieldScalar(d.cycleCount)},
        {3, detail::fieldScalar(d.missedDeadlines)},
        {4, detail::fieldScalar(d.maxCycleWorkUs)},
        {5, detail::fieldScalar(d.jitterMaxUs)},
        {6, detail::fieldScalar(d.jitterAvgUs)},
        {7, detail::fieldScalar(d.dcSyncCount)},
        {8, detail::fieldScalar(d.dcSyncErrors)},
        {9, detail::fieldScalar(d.dcJitterMaxUs)},
        {10, detail::fieldScalar(d.mbxSends)},
        {11, detail::fieldScalar(d.mbxSendErrors)},
        {12, detail::fieldScalar(d.mbxCollects)},
        {13, detail::fieldScalar(d.mbxCollectErrors)},
        {14, detail::fieldScalar(d.maxSendLatencyNs)},
        {15, detail::fieldScalar(d.txRetries)},
        {16, detail::fieldScalar(d.txFailures)},
        {17, detail::fieldScalar(d.rxFrames)},
        {18, detail::fieldScalar(d.slavesDetected)},
        {19, detail::fieldScalar(d.slavesOperational)},
        {20, detail::fieldScalar(d.lastWkc)},
    });
}

// ---------------------------------------------------------------------------
// PDO map + slave-supervisor surfaces — pluggable backends
// ---------------------------------------------------------------------------

/// Logical placement of one enabled PDO mapping entry (machine.pdo.map).
struct PdoEntryV1 {
    uint16_t slaveIndex = 0;
    uint16_t pdoIndex = 0;
    uint8_t direction = 0;          ///< 0 = RxPDO (outputs), 1 = TxPDO (inputs)
    uint32_t offset = 0;            ///< byte offset in the logical image
    uint16_t length = 0;
    uint16_t entryIndex = 0;
};

/// Pluggable source for machine.pdo.map — e.g. EtherCatPdoAdapter over
/// LogicalAddressManager::describeEntries.

/// One evaluated commissioning-checklist item (ChecklistItemV1).
struct ChecklistItem {
    uint32_t itemId = 0;
    std::string label;
    bool required = true;
    bool passed = false;
    std::string evidence;
};

/// Pluggable checklist provider. The application evaluates requirements
/// against live state and returns the current verdict for every item.
class IMachineChecklistSource {
public:
    virtual ~IMachineChecklistSource() = default;
    virtual std::vector<ChecklistItem> evaluate() = 0;
};

/// Server-verified application profile document (AppProfileV1). The document
/// and its keyed-BLAKE3 MAC are served verbatim; integrity is verified when
/// the profile is attached (see ApplicationProfile.hpp).
struct MachineAppProfile {
    std::string name;
    std::string version;
    std::vector<uint8_t> document;
    std::array<uint8_t, 32> mac{};
};

/// Pluggable provider for the machine.app.profile signal. The surface is
/// absent entirely when no profile source is attached.
class IMachineAppProfileSource {
public:
    virtual ~IMachineAppProfileSource() = default;
    virtual const MachineAppProfile& profile() = 0;
};

/// One recipe descriptor (RecipeInfoV1).
struct RecipeInfo {
    std::string name;
    std::string description;
    uint32_t entryCount = 0;
};

/// Pluggable store of named parameter sets. `recipe` fills `writes` with the
/// ConfigWrite list to stage; returns false when the name is unknown.
class IMachineRecipeStore {
public:
    virtual ~IMachineRecipeStore() = default;
    virtual std::vector<RecipeInfo> list() = 0;
    virtual bool recipe(std::string_view name, std::vector<ConfigWrite>& writes) = 0;
};

class IMachinePdoSource {
public:
    virtual ~IMachinePdoSource() = default;
    virtual std::vector<PdoEntryV1> entries() = 0;
};

/// SlaveSupervisor recovery state for one slave (machine.supervisor.status).
struct SupervisorEntryV1 {
    uint16_t slaveIndex = 0;
    uint8_t state = 0;              ///< SlaveRecoveryState ordinals
    bool suspended = false;
    bool recovering = false;
    uint16_t attemptCount = 0;
};

/// Pluggable backend for machine.supervisor.* — e.g. EtherCatSupervisorAdapter.
class IMachineSupervisorAccess {
public:
    virtual ~IMachineSupervisorAccess() = default;
    virtual std::vector<SupervisorEntryV1> slaves() = 0;

    /// Controlled retry: reset a failed/critical slave's recovery state and
    /// trigger a bounded recovery pass. Returns false with `error` when the
    /// backend cannot retry (not supervised, recovery in progress).
    virtual bool retry(uint16_t slave, std::string& error) = 0;
};

/// Role/audit/rate-limit policy around a pluggable IMachineSupervisorAccess.
class MachineSupervisorService final {
public:
    /// Retries are commissioning actions: at most one per second server-wide.
    static constexpr uint64_t kRetryMinIntervalUs = 1'000'000;

    explicit MachineSupervisorService(IMachineSupervisorAccess& access,
                                      EventJournal* auditJournal = nullptr)
        : access_(access), auditJournal_(auditJournal) {}

    std::vector<uint8_t> encodeStatus() const {
        std::vector<std::vector<uint8_t>> entries;
        for (const auto& entry : access_.slaves()) {
            entries.push_back(detail::encodeTagged({
                {1, detail::fieldScalar(entry.slaveIndex)},
                {2, detail::fieldScalar(entry.state)},
                {3, detail::fieldScalar(entry.suspended)},
                {4, detail::fieldScalar(entry.recovering)},
                {5, detail::fieldScalar(entry.attemptCount)},
            }));
        }
        return detail::encodeArray(entries);
    }

    /// machine.supervisor.retry — technician-or-above, rate-limited, audited.
    FunctionCallResult retry(uint16_t slave, const SessionIdentity& identity) {
        FunctionCallResult result;
        if (!identity.authenticated || identity.role < Role::Technician) {
            result.errorMessage =
                "machine.supervisor.retry requires technician role or above";
            return result;
        }
        const uint64_t now = detail::nowUs();
        const uint64_t last = lastRetryUs_.load(std::memory_order_relaxed);
        if (now < last + kRetryMinIntervalUs) {
            result.errorMessage =
                "machine.supervisor.retry rate limited: one retry per second";
            return result;
        }
        lastRetryUs_.store(now, std::memory_order_relaxed);
        std::string error;
        const bool ok = access_.retry(slave, error);
        if (auditJournal_) {
            EventRecordV1 record;
            record.timestampUs = now;
            record.eventType = "audit.supervisor.retry";
            record.sourceId = std::to_string(slave);
            record.severity = ok ? EventSeverity::Info : EventSeverity::Warning;
            record.description = identity.actor + " supervisor retry slave " +
                std::to_string(slave) + (ok ? " accepted" : " rejected: " + error);
            auditJournal_->append(std::move(record));
        }
        result.returnValue = detail::encodeTagged({
            {1, detail::fieldScalar(ok)},
            {2, detail::fieldScalar(uint8_t{0})},
            {3, detail::fieldString(error)},
        });
        result.success = true;
        result.error = ErrorCode::None;
        return result;
    }

private:
    IMachineSupervisorAccess& access_;
    EventJournal* auditJournal_;
    std::atomic<uint64_t> lastRetryUs_{0};
};

// ---------------------------------------------------------------------------
// MachineService — installs the typed machine.cia402.v1 surface
// ---------------------------------------------------------------------------

class MachineService final {
public:
    static constexpr uint64_t kAuthoritySnapshotSignalId = 0x4349410300000001ULL;
    static constexpr uint64_t kAlarmCursorSignalId = 0x4349410400000001ULL;
    static constexpr uint64_t kDiagnosticsSignalId = 0x4349410700000001ULL;
    static constexpr uint64_t kAuthorityAcquireFnId = 0x4349410300000002ULL;
    static constexpr uint64_t kAuthorityRenewFnId = 0x4349410300000003ULL;
    static constexpr uint64_t kAuthorityReleaseFnId = 0x4349410300000004ULL;
    static constexpr uint64_t kAuthorityTakeoverFnId = 0x4349410300000009ULL;
    static constexpr uint64_t kCommandFnId = 0x4349410300000005ULL;
    static constexpr uint64_t kCommandCancelFnId = 0x4349410300000006ULL;
    static constexpr uint64_t kOperationReadFnId = 0x4349410300000007ULL;
    static constexpr uint64_t kOperationListFnId = 0x4349410300000008ULL;
    static constexpr uint64_t kAlarmReadFnId = 0x4349410400000002ULL;
    static constexpr uint64_t kAlarmAckFnId = 0x4349410400000003ULL;
    static constexpr uint64_t kAlarmClearFnId = 0x4349410400000004ULL;
    static constexpr uint64_t kCaptureConfigureFnId = 0x4349410500000001ULL;
    static constexpr uint64_t kCaptureStatusFnId = 0x4349410500000002ULL;
    static constexpr uint64_t kCaptureCancelFnId = 0x4349410500000008ULL;
    static constexpr uint64_t kCaptureExportFnId = 0x4349410500000009ULL;
    static constexpr uint64_t kConfigStageFnId = 0x4349410500000003ULL;
    static constexpr uint64_t kConfigValidateFnId = 0x4349410500000004ULL;
    static constexpr uint64_t kConfigCommitFnId = 0x4349410500000005ULL;
    static constexpr uint64_t kConfigRollbackFnId = 0x4349410500000006ULL;
    static constexpr uint64_t kConfigStatusFnId = 0x4349410500000007ULL;
    static constexpr uint64_t kSdoListFnId = 0x4349410600000001ULL;
    static constexpr uint64_t kSdoReadFnId = 0x4349410600000002ULL;
    static constexpr uint64_t kSdoWriteFnId = 0x4349410600000003ULL;
    static constexpr uint64_t kConfigExportFnId = 0x434941050000000AULL;
    static constexpr uint64_t kConfigDiffFnId = 0x434941050000000BULL;
    static constexpr uint64_t kConfigImportFnId = 0x434941050000000CULL;
    static constexpr uint64_t kChecklistListFnId = 0x4349410900000001ULL;
    static constexpr uint64_t kChecklistReportFnId = 0x4349410900000002ULL;
    static constexpr uint64_t kAppProfileSignalId = 0x4349410A00000001ULL;
    static constexpr uint64_t kRecipeListFnId = 0x4349410A00000002ULL;
    static constexpr uint64_t kRecipeApplyFnId = 0x4349410A00000003ULL;
    static constexpr uint64_t kMetricsFnId = 0x4349410A00000004ULL;
    static constexpr uint64_t kPdoMapFnId = 0x4349410800000001ULL;
    static constexpr uint64_t kSupervisorStatusFnId = 0x4349410800000002ULL;
    static constexpr uint64_t kSupervisorRetryFnId = 0x4349410800000003ULL;
    static constexpr uint16_t kMaximumEncodedValue = 16384;

    MachineService(IMachineStateSource& stateSource, IMachineDispatcher& dispatcher,
                   ControlAuthority& authority, MachineCommandGate& gate,
                   AlarmService& alarms, OperationTracker& operations,
                   EventJournal* auditJournal = nullptr,
                   MachineCaptureService* capture = nullptr,
                   StagedConfigService* config = nullptr)
        : stateSource_(stateSource), dispatcher_(dispatcher), authority_(authority),
          gate_(gate), alarms_(alarms), operations_(operations),
          auditJournal_(auditJournal), capture_(capture), config_(config) {}

    /// Attach optional services before install() when they depend on a
    /// registry that does not exist at construction time.
    void setAuxiliaryServices(MachineCaptureService* capture,
                              StagedConfigService* config) {
        capture_ = capture;
        config_ = config;
    }

    /// Attach an optional pluggable SDO backend. When unset, machine.sdo.*
    /// functions are not registered at all — the surface is absent, not
    /// merely failing.
    void setSdoService(MachineSdoService* sdo) { sdo_ = sdo; }

    /// Attach a pluggable diagnostics source before install(). Without one
    /// the machine.diagnostics signal is not registered.
    void setDiagnosticsSource(IMachineDiagnosticsSource* source) {
        diagnostics_ = source;
    }

    /// Attach a PDO-map source / supervisor backend before install(); the
    /// corresponding machine.pdo.map / machine.supervisor.* functions are
    /// only registered when set.
    void setPdoSource(IMachinePdoSource* source) { pdo_ = source; }
    /// Attach a commissioning checklist provider before install(); without
    /// one the machine.checklist.* functions are not registered.
    void setChecklistSource(IMachineChecklistSource* source) {
        checklist_ = source;
    }
    void setSupervisor(MachineSupervisorService* supervisor) {
        supervisor_ = supervisor;
    }
    /// Attach an application profile source before install(); without one
    /// the machine.app.profile signal is not registered.
    void setAppProfileSource(IMachineAppProfileSource* source) {
        appProfile_ = source;
    }
    /// Attach a recipe store before install(). Recipes are staged through the
    /// staged-config service, so the machine.recipe.* functions require
    /// setAuxiliaryServices() as well and are absent without it.
    void setRecipeStore(IMachineRecipeStore* store) { recipes_ = store; }

    /// Register every typed machine function/signal. Requires the full
    /// machine-profile schema graph to be installed in `catalog`.
    bool install(Registry& registry, const SchemaCatalog& catalog) {
        const auto* graph = catalog.graph();
        if (!graph) return false;
        const auto slotFor = [&catalog, graph](cia402::MachineSchemaId id,
                                               SchemaRef& ref) -> std::optional<SchemaSlot> {
            const auto* node = graph->find(cia402::schemaKey(id));
            if (!node) return std::nullopt;
            ref = SchemaRef{node->key, computeSchemaDigest(*node)};
            return catalog.slotFor(ref);
        };
        SchemaRef authoritySnapshotRef, authorityResultRef, commandRequestRef,
            commandReceiptRef, operationRef, operationArrayRef, alarmPageRef;
        const auto authoritySnapshotSlot = slotFor(cia402::MachineSchemaId::AuthoritySnapshot,
                                                   authoritySnapshotRef);
        const auto authorityResultSlot = slotFor(cia402::MachineSchemaId::AuthorityResult,
                                                 authorityResultRef);
        const auto commandRequestSlot = slotFor(cia402::MachineSchemaId::CommandRequest,
                                                commandRequestRef);
        const auto commandReceiptSlot = slotFor(cia402::MachineSchemaId::CommandReceipt,
                                                commandReceiptRef);
        const auto operationSlot = slotFor(cia402::MachineSchemaId::OperationSnapshot,
                                           operationRef);
        const auto operationArraySlot = slotFor(cia402::MachineSchemaId::OperationArray,
                                                operationArrayRef);
        const auto alarmPageSlot = slotFor(cia402::MachineSchemaId::AlarmPage, alarmPageRef);
        if (!authoritySnapshotSlot || !authorityResultSlot || !commandRequestSlot ||
            !commandReceiptSlot || !operationSlot || !operationArraySlot || !alarmPageSlot)
            return false;

        // machine.authority.snapshot — every active lease
        {
            SignalEntry signal{};
            signal.id = kAuthoritySnapshotSignalId;
            signal.name = "machine.authority.snapshot";
            signal.description = "Active server-owned control leases";
            signal.group = "machine.cia402";
            signal.valueType = ValueType::Struct;
            signal.schema = authoritySnapshotRef;
            signal.schemaSlot = *authoritySnapshotSlot;
            signal.maxValueSize = kMaximumEncodedValue;
            signal.metadata = {{"access.role", "observer"},
                               {"schema.name", "tether.machine.cia402.AuthoritySnapshotV1"}};
            signal.varReadFn = [this](void* destination, size_t capacity) {
                const auto bytes = encodeAuthoritySnapshot();
                if (bytes.size() > capacity) return size_t{0};
                std::memcpy(destination, bytes.data(), bytes.size());
                return bytes.size();
            };
            if (!registry.addSignal(std::move(signal))) return false;
        }
        // machine.alarms.cursor — latest retained alarm id
        {
            SignalEntry signal{};
            signal.id = kAlarmCursorSignalId;
            signal.name = "machine.alarms.cursor";
            signal.description = "Latest retained alarm cursor";
            signal.group = "machine.cia402";
            signal.valueType = ValueType::U64;
            signal.metadata = {{"access.role", "observer"}};
            signal.readFn = [this](void* destination) {
                const uint64_t cursor = alarms_.latestCursor();
                std::memcpy(destination, &cursor, sizeof(cursor));
            };
            if (!registry.addSignal(std::move(signal))) return false;
        }

        if (!addAuthorityFunction(registry, kAuthorityAcquireFnId,
                                  "machine.authority.acquire",
                                  "Acquire the server-owned control lease for a scope",
                                  AuthorityVerb::Acquire, *authorityResultSlot, authorityResultRef) ||
            !addAuthorityFunction(registry, kAuthorityRenewFnId,
                                  "machine.authority.renew",
                                  "Renew an held control lease",
                                  AuthorityVerb::Renew, *authorityResultSlot, authorityResultRef) ||
            !addAuthorityFunction(registry, kAuthorityReleaseFnId,
                                  "machine.authority.release",
                                  "Release a held control lease",
                                  AuthorityVerb::Release, *authorityResultSlot, authorityResultRef) ||
            !addAuthorityFunction(registry, kAuthorityTakeoverFnId,
                                  "machine.authority.takeover",
                                  "Force-take a scope lease from its current owner (controls engineer or administrator)",
                                  AuthorityVerb::Takeover, *authorityResultSlot, authorityResultRef))
            return false;

        // machine.command — validated dispatch through MachineCommandGate
        {
            FunctionEntry function;
            function.id = kCommandFnId;
            function.name = "machine.command";
            function.description =
                "Validated, idempotent machine command. The server re-checks "
                "authority, role, state generation, interlocks, bounds, and "
                "deadline immediately before dispatch.";
            function.group = "machine.cia402";
            FunctionParameter request;
            request.key = 1;
            request.name = "request";
            request.description = "CommandRequestV1 tagged struct";
            request.type = ValueType::Struct;
            request.schema = commandRequestRef;
            request.schemaSlot = *commandRequestSlot;
            request.maxValueSize = 4096;
            function.parameters.push_back(std::move(request));
            function.returnValue.present = true;
            function.returnValue.name = "receipt";
            function.returnValue.description = "CommandReceiptV1 tagged struct";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = commandReceiptRef;
            function.returnValue.schemaSlot = *commandReceiptSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "operator"},
                                 {"access.danger", "machine-motion"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    return handleCommand(arguments, context);
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.command.cancel
        {
            FunctionEntry function;
            function.id = kCommandCancelFnId;
            function.name = "machine.command.cancel";
            function.description = "Cancel a running operation owned by or visible to this session";
            function.group = "machine.cia402";
            FunctionParameter operationId;
            operationId.key = 1;
            operationId.name = "operation_id";
            operationId.description = "Server operation identifier from CommandReceiptV1";
            operationId.type = ValueType::String;
            operationId.maxValueSize = 128;
            function.parameters.push_back(std::move(operationId));
            function.returnValue.present = true;
            function.returnValue.name = "operation";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = operationRef;
            function.returnValue.schemaSlot = *operationSlot;
            function.returnValue.maxValueSize = 4096;
            function.metadata = {{"access.role", "operator"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    std::string operationId;
                    if (arguments.size() != 1 ||
                        !detail::getString(arguments[0].value, operationId, 127)) {
                        result.errorMessage = "machine.command.cancel requires an operation id";
                        return result;
                    }
                    if (auto operation = cancelOperation(operationId, context.identity)) {
                        result.returnValue = encodeOperation(*operation);
                        result.success = true;
                        result.error = ErrorCode::None;
                    } else {
                        result.errorMessage =
                            "Operation is absent, already terminal, or owned by another session";
                    }
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.operation.read / machine.operations.list
        if (!addOperationFunction(registry, kOperationReadFnId, "machine.operation.read",
                                  operationRef, *operationSlot, false) ||
            !addOperationFunction(registry, kOperationListFnId, "machine.operations.list",
                                  operationArrayRef, *operationArraySlot, true))
            return false;

        // machine.alarms.read / acknowledge / clear
        if (!addAlarmFunction(registry, alarmPageRef, *alarmPageSlot)) return false;
        if (capture_ && !addCaptureFunctions(registry, slotFor)) return false;
        if (config_ && !addConfigFunctions(registry, slotFor)) return false;
        if (sdo_ && !addSdoFunctions(registry, slotFor)) return false;
        if (diagnostics_ && !addDiagnosticsSignal(registry, slotFor)) return false;
        if (pdo_ && !addPdoFunctions(registry, slotFor)) return false;
        if (supervisor_ && !addSupervisorFunctions(registry, slotFor)) return false;
        if (checklist_ && !addChecklistFunctions(registry, slotFor)) return false;
        if (appProfile_ && !addAppProfileSignal(registry, slotFor)) return false;
        if (recipes_ && config_ && !addRecipeFunctions(registry, slotFor)) return false;
        if (!addMetricsFunction(registry, slotFor)) return false;
        return true;
    }

    /// Called when a session disconnects: force-release its leases and
    /// cancel its in-flight operations (disconnect/expiry stop semantics).
    void sessionEnded(const std::string& sessionId) {
        authority_.releaseAll(sessionId);
        operations_.cancelAllForSession(sessionId, "Owner session disconnected");
    }

    std::optional<OperationRecord> cancelOperation(const std::string& operationId,
                                                   const SessionIdentity& identity) {
        (void)dispatcher_.cancelOperation(operationId);
        if (!operations_.tryCancel(operationId, identity)) return std::nullopt;
        return operations_.get(operationId);
    }

private:
    enum class AuthorityVerb : uint8_t { Acquire, Renew, Release, Takeover };

    std::vector<uint8_t> encodeAuthoritySnapshot() const {
        std::vector<std::vector<uint8_t>> leases;
        const auto now = SteadyClock::now();
        for (const auto& lease : authority_.leases(now)) {
            const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                lease.expiresAt - now).count();
            leases.push_back(detail::encodeTagged({
                {1, detail::fieldString(lease.scope)},
                {2, detail::fieldString(lease.ownerActor)},
                {3, detail::fieldScalar(lease.token)},
                {4, detail::fieldScalar(static_cast<uint32_t>(std::max<int64_t>(remaining, 0)))},
            }));
        }
        return detail::encodeTagged({
            {1, detail::fieldScalar(detail::nowUs())},
            {2, detail::encodeArray(leases)},
        });
    }

    std::vector<uint8_t> encodeAuthorityResult(const std::string& scope, bool granted,
                                             uint64_t token, uint32_t remainingMs,
                                             std::string_view owner,
                                             std::string_view blocker) const {
        return detail::encodeTagged({
            {1, detail::fieldScalar(granted)},
            {2, detail::fieldString(scope)},
            {3, detail::fieldScalar(token)},
            {4, detail::fieldScalar(remainingMs)},
            {5, detail::fieldString(owner)},
            {6, detail::fieldString(blocker)},
        });
    }

    static std::vector<uint8_t> encodeOperation(const OperationRecord& operation) {
        return detail::encodeTagged({
            {1, detail::fieldString(operation.operationId)},
            {2, detail::fieldString(operation.requestUuid)},
            {3, detail::fieldScalar(static_cast<uint8_t>(operation.state))},
            {4, detail::fieldScalar(operation.progress)},
            {5, detail::fieldScalar(static_cast<uint8_t>(operation.action))},
            {6, detail::fieldString(operation.target)},
            {7, detail::fieldString(operation.message)},
            {8, detail::fieldScalar(operation.startedUs)},
            {9, detail::fieldScalar(operation.finishedUs)},
            {10, detail::fieldScalar(operation.resultCode)},
        });
    }

    std::vector<uint8_t> encodeAlarm(const AlarmRecordV1& alarm) const {
        return detail::encodeTagged({
            {1, detail::fieldScalar(alarm.alarmId)},
            {2, detail::fieldScalar(alarm.raisedUs)},
            {3, detail::fieldScalar(alarm.updatedUs)},
            {4, detail::fieldScalar(static_cast<uint8_t>(alarm.state))},
            {5, detail::fieldScalar(static_cast<uint8_t>(alarm.severity))},
            {6, detail::fieldScalar(alarm.code)},
            {7, detail::fieldString(alarm.sourceId)},
            {8, detail::fieldString(alarm.description)},
            {9, detail::fieldString(alarm.actor)},
        });
    }

    std::vector<uint8_t> encodeAlarmPage(const AlarmPageV1& page) const {
        std::vector<std::vector<uint8_t>> alarms;
        alarms.reserve(page.alarms.size());
        for (const auto& alarm : page.alarms) alarms.push_back(encodeAlarm(alarm));
        return detail::encodeTagged({
            {1, detail::fieldScalar(page.oldestCursor)},
            {2, detail::fieldScalar(page.latestCursor)},
            {3, detail::fieldScalar(page.nextCursor)},
            {4, detail::fieldScalar(page.gap)},
            {5, detail::encodeArray(alarms)},
        });
    }

    bool addAuthorityFunction(Registry& registry, uint64_t id, std::string name,
                              std::string description, AuthorityVerb verb,
                              SchemaSlot resultSlot, const SchemaRef& resultRef) {
        FunctionEntry function;
        function.id = id;
        function.name = std::move(name);
        function.description = std::move(description);
        function.group = "machine.cia402";
        const bool acquireLike =
            verb == AuthorityVerb::Acquire || verb == AuthorityVerb::Takeover;
        if (!acquireLike) {
            FunctionParameter token;
            token.key = 1;
            token.name = "token";
            token.description = "Lease token returned by acquire";
            token.type = ValueType::U64;
            function.parameters.push_back(std::move(token));
        }
        FunctionParameter scope;
        scope.key = acquireLike ? 1 : 2;
        scope.name = "scope";
        scope.description = "Machine or motion-group scope";
        scope.type = ValueType::String;
        scope.maxValueSize = 128;
        function.parameters.push_back(std::move(scope));
        if (verb != AuthorityVerb::Release) {
            FunctionParameter leaseMs;
            leaseMs.key = acquireLike ? 2 : 3;
            leaseMs.name = "lease_ms";
            leaseMs.description = "Requested lease duration in milliseconds";
            leaseMs.type = ValueType::U32;
            leaseMs.metadata["range.min"] = "1";
            function.parameters.push_back(std::move(leaseMs));
        }
        function.returnValue.present = true;
        function.returnValue.name = "result";
        function.returnValue.type = ValueType::Struct;
        function.returnValue.schema = resultRef;
        function.returnValue.schemaSlot = resultSlot;
        function.returnValue.maxValueSize = 4096;
        function.metadata = {{"access.role",
                              verb == AuthorityVerb::Takeover ? "controls-engineer" : "operator"}};
        function.contextualCallback =
            [this, verb, acquireLike](const std::vector<FunctionArgument>& arguments,
                                      const FunctionInvokeContext& context) {
                FunctionCallResult result;
                const auto encode = [this, &result](const std::string& scope,
                                                  const AuthorityResult& outcome) {
                    uint32_t remaining = 0;
                    if (outcome.lease) {
                        remaining = static_cast<uint32_t>(std::max<int64_t>(
                            std::chrono::duration_cast<std::chrono::milliseconds>(
                                outcome.lease->expiresAt - SteadyClock::now()).count(), 0));
                    }
                    const auto current = authority_.current(scope);
                    const auto owner = outcome.lease ? outcome.lease->ownerActor
                        : current ? current->ownerActor : std::string{};
                    result.returnValue = encodeAuthorityResult(
                        scope, outcome.lease.has_value(),
                        outcome.lease ? outcome.lease->token : 0, remaining, owner,
                        outcome.blocker);
                    result.success = true;
                    result.error = ErrorCode::None;
                };
                const auto identity = context.identity;
                if (acquireLike) {
                    std::string scope;
                    uint32_t leaseMs = 0;
                    if (arguments.size() != 2 ||
                        !detail::getString(arguments[0].value, scope, 127) ||
                        !detail::getScalar(arguments[1].value, leaseMs)) {
                        result.errorMessage = "acquire requires scope and lease_ms";
                        return result;
                    }
                    const auto outcome = verb == AuthorityVerb::Takeover
                        ? authority_.takeover(scope, identity, std::chrono::milliseconds(leaseMs))
                        : authority_.acquire(scope, identity, std::chrono::milliseconds(leaseMs));
                    encode(scope, outcome);
                    auditAuthority(identity,
                                   verb == AuthorityVerb::Takeover ? "takeover" : "acquire", scope);
                } else if (verb == AuthorityVerb::Renew) {
                    uint64_t token = 0;
                    std::string scope;
                    uint32_t leaseMs = 0;
                    if (arguments.size() != 3 ||
                        !detail::getScalar(arguments[0].value, token) ||
                        !detail::getString(arguments[1].value, scope, 127) ||
                        !detail::getScalar(arguments[2].value, leaseMs)) {
                        result.errorMessage = "renew requires token, scope, and lease_ms";
                        return result;
                    }
                    encode(scope, authority_.renew(token, scope, identity,
                                                 std::chrono::milliseconds(leaseMs)));
                } else {
                    uint64_t token = 0;
                    std::string scope;
                    if (arguments.size() != 2 ||
                        !detail::getScalar(arguments[0].value, token) ||
                        !detail::getString(arguments[1].value, scope, 127)) {
                        result.errorMessage = "release requires token and scope";
                        return result;
                    }
                    const bool released = authority_.release(token, scope, identity);
                    result.returnValue = encodeAuthorityResult(
                        scope, released, 0, 0, "",
                        released ? "" : "Authority lease could not be released by this session");
                    result.success = true;
                    result.error = ErrorCode::None;
                    auditAuthority(identity, "release", scope);
                }
                return result;
            };
        return registry.addFunction(std::move(function));
    }

    bool addOperationFunction(Registry& registry, uint64_t id, std::string name,
                              const SchemaRef& ref, SchemaSlot slot, bool listAll) {
        FunctionEntry function;
        function.id = id;
        function.name = std::move(name);
        function.group = "machine.cia402";
        function.metadata = {{"access.role", "observer"}};
        if (listAll) {
            function.description = "List all tracked operations, oldest first";
            FunctionParameter limit;
            limit.key = 1;
            limit.name = "limit";
            limit.description = "Maximum operations to return (1-64)";
            limit.type = ValueType::U32;
            limit.metadata["range.min"] = "1";
            limit.metadata["range.max"] = "64";
            function.parameters.push_back(std::move(limit));
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    uint32_t limit = 0;
                    if (arguments.size() != 1 ||
                        !detail::getScalar(arguments[0].value, limit) ||
                        limit == 0 || limit > 64) {
                        result.errorMessage = "operations.list requires a limit between 1 and 64";
                        return result;
                    }
                    auto all = operations_.list();
                    std::vector<std::vector<uint8_t>> encoded;
                    const size_t start = all.size() > limit ? all.size() - limit : 0;
                    for (size_t index = start; index < all.size(); ++index)
                        encoded.push_back(encodeOperation(all[index]));
                    result.returnValue = detail::encodeArray(encoded);
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
        } else {
            function.description = "Read the progress and outcome of one operation";
            FunctionParameter operationId;
            operationId.key = 1;
            operationId.name = "operation_id";
            operationId.type = ValueType::String;
            operationId.maxValueSize = 128;
            function.parameters.push_back(std::move(operationId));
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    std::string operationId;
                    if (arguments.size() != 1 ||
                        !detail::getString(arguments[0].value, operationId, 127)) {
                        result.errorMessage = "operation.read requires an operation id";
                        return result;
                    }
                    const auto operation = operations_.get(operationId);
                    if (!operation) {
                        result.errorMessage = "Unknown operation id";
                        return result;
                    }
                    result.returnValue = encodeOperation(*operation);
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
        }
        function.returnValue.present = true;
        function.returnValue.name = listAll ? "operations" : "operation";
        function.returnValue.type = listAll ? ValueType::Array : ValueType::Struct;
        function.returnValue.schema = ref;
        function.returnValue.schemaSlot = slot;
        function.returnValue.maxValueSize = kMaximumEncodedValue;
        return registry.addFunction(std::move(function));
    }

    bool addAlarmFunction(Registry& registry, const SchemaRef& pageRef, SchemaSlot pageSlot) {
        {
            FunctionEntry function;
            function.id = kAlarmReadFnId;
            function.name = "machine.alarms.read";
            function.description = "Read a bounded page from the retained alarm journal";
            function.group = "machine.cia402";
            function.parameters = {
                FunctionParameter{"after_cursor", "Exclusive alarm cursor", ValueType::U64},
                FunctionParameter{"limit", "Maximum alarms to return (1-100)", ValueType::U32},
            };
            function.parameters[0].key = 1;
            function.parameters[1].key = 2;
            function.parameters[1].metadata["range.min"] = "1";
            function.parameters[1].metadata["range.max"] = "100";
            function.returnValue.present = true;
            function.returnValue.name = "page";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = pageRef;
            function.returnValue.schemaSlot = pageSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "observer"}};
            function.callback = [this](const std::vector<FunctionArgument>& arguments) {
                FunctionCallResult result;
                uint64_t cursor = 0;
                uint32_t limit = 0;
                if (arguments.size() != 2 ||
                    !detail::getScalar(arguments[0].value, cursor) ||
                    !detail::getScalar(arguments[1].value, limit) ||
                    limit == 0 || limit > 100) {
                    result.errorMessage = "alarms.read requires cursor and a limit of 1-100";
                    return result;
                }
                result.returnValue = encodeAlarmPage(alarms_.readAfter(cursor, limit));
                result.success = true;
                result.error = ErrorCode::None;
                return result;
            };
            if (!registry.addFunction(std::move(function))) return false;
        }
        const auto ackOrClear = [&](uint64_t id, std::string name, std::string description,
                                    bool clear) {
            FunctionEntry function;
            function.id = id;
            function.name = std::move(name);
            function.description = std::move(description);
            function.group = "machine.cia402";
            FunctionParameter alarmId;
            alarmId.key = 1;
            alarmId.name = "alarm_id";
            alarmId.type = ValueType::U64;
            function.parameters.push_back(std::move(alarmId));
            function.returnValue.present = true;
            function.returnValue.name = "status";
            function.returnValue.type = ValueType::U8;
            function.metadata = {{"access.role", "operator"}};
            function.contextualCallback =
                [this, clear](const std::vector<FunctionArgument>& arguments,
                              const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    uint64_t alarmId = 0;
                    if (arguments.size() != 1 ||
                        !detail::getScalar(arguments[0].value, alarmId)) {
                        result.errorMessage = "alarm id argument required";
                        return result;
                    }
                    const uint8_t status = clear
                        ? alarms_.clear(alarmId, context.identity)
                        : alarms_.acknowledge(alarmId, context.identity);
                    if (status == 2) {
                        result.errorMessage =
                            "Alarm acknowledgement requires an authenticated operator role";
                        return result;
                    }
                    result.returnValue = detail::fieldScalar(status);
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            return registry.addFunction(std::move(function));
        };
        if (!ackOrClear(kAlarmAckFnId, "machine.alarms.acknowledge",
                        "Acknowledge an active alarm", false) ||
            !ackOrClear(kAlarmClearFnId, "machine.alarms.clear",
                        "Clear an acknowledged or resolved alarm", true))
            return false;
        return true;
    }

    // ---- machine.capture.* ---------------------------------------------------

    std::vector<uint8_t> encodeCaptureStatus() const {
        const DatalogStatus status = capture_->status();
        return detail::encodeTagged({
            {1, detail::fieldScalar(detail::nowUs())},
            {2, detail::fieldScalar(static_cast<uint8_t>(status.state))},
            {3, detail::fieldScalar(status.recordsWritten)},
            {4, detail::fieldScalar(status.bytesWritten)},
            {5, detail::fieldString(status.metadata.logName)},
            {6, detail::fieldScalar(status.metadata.sampleRateHz)},
            {7, detail::fieldScalar(static_cast<uint32_t>(status.metadata.fields.size()))},
            {8, detail::fieldScalar(status.state != DatalogState::Idle)},
            {9, detail::fieldScalar(static_cast<uint8_t>(capture_->triggerState()))},
            {10, detail::fieldScalar(capture_->recordsAvailable())},
            {11, detail::fieldScalar(capture_->recordsDropped())},
            {12, detail::encodeArray([&] {
                 std::vector<std::vector<uint8_t>> fields;
                 fields.reserve(status.metadata.fields.size());
                 for (const auto& field : status.metadata.fields) {
                     fields.push_back(detail::encodeTagged({
                         {1, detail::fieldScalar(field.entryId)},
                         {2, detail::fieldScalar(static_cast<uint32_t>(field.offset))},
                         {3, detail::fieldScalar(static_cast<uint32_t>(field.size))},
                     }));
                 }
                 return fields;
             }())},
        });
    }

    template <typename SlotFor>
    bool addCaptureFunctions(Registry& registry, const SlotFor& slotFor) {
        SchemaRef statusRef;
        const auto statusSlot = slotFor(cia402::MachineSchemaId::CaptureStatus, statusRef);
        SchemaRef idsRef;
        const auto idsSlot = slotFor(cia402::MachineSchemaId::GenerationArray, idsRef);
        SchemaRef exportRef;
        const auto exportSlot = slotFor(cia402::MachineSchemaId::CaptureExport, exportRef);
        if (!statusSlot || !idsSlot || !exportSlot) return false;

        // machine.capture.configure — name, rate, enabled, entry ids
        {
            FunctionEntry function;
            function.id = kCaptureConfigureFnId;
            function.name = "machine.capture.configure";
            function.description =
                "Configure server-side recording. Recorded bytes stay on the "
                "server; clients receive status and layout metadata only.";
            function.group = "machine.cia402";
            function.parameters = {
                FunctionParameter{"log_name", "Capture identifier", ValueType::String},
                FunctionParameter{"sample_rate_hz", "Nominal sample rate", ValueType::U32},
                FunctionParameter{"enabled", "Start/stop recording", ValueType::Bool},
                FunctionParameter{"entry_ids", "Entries to record (empty = all)", ValueType::Array},
            };
            function.parameters[0].key = 1;
            function.parameters[0].maxValueSize = 128;
            function.parameters[1].key = 2;
            function.parameters[1].metadata["range.min"] = "1";
            function.parameters[2].key = 3;
            function.parameters[3].key = 4;
            function.parameters[3].schema = idsRef;
            function.parameters[3].schemaSlot = *idsSlot;
            // Optional trigger profile (keys 5-9). A trigger_id of 0 disables
            // triggering; otherwise the capture runs in pre/post-trigger mode.
            const auto optParam = [](uint32_t key, const char* name,
                                     const char* description, ValueType type,
                                     std::vector<uint8_t> defaultValue) {
                FunctionParameter parameter;
                parameter.key = key;
                parameter.name = name;
                parameter.description = description;
                parameter.type = type;
                parameter.optional = true;
                parameter.hasDefault = true;
                parameter.defaultValue = std::move(defaultValue);
                return parameter;
            };
            function.parameters.push_back(optParam(5, "trigger_entry_id",
                "Signal evaluated per record (0 = no trigger)", ValueType::U64,
                detail::fieldScalar<uint64_t>(0)));
            function.parameters.push_back(optParam(6, "trigger_op",
                "0 above, 1 below, 2 |value| above", ValueType::U8,
                detail::fieldScalar<uint8_t>(0)));
            function.parameters.push_back(optParam(7, "trigger_level",
                "Trigger threshold", ValueType::F64, std::vector<uint8_t>(8, 0)));
            function.parameters.push_back(optParam(8, "pre_records",
                "Ring depth kept before the trigger", ValueType::U32,
                detail::fieldScalar<uint32_t>(0)));
            function.parameters.push_back(optParam(9, "post_records",
                "Records collected after the trigger", ValueType::U32,
                detail::fieldScalar<uint32_t>(0)));
            // Optional template preset (key 10): 1 = "service" — the server
            // picks every fixed-size signal when entry_ids is empty.
            function.parameters.push_back(optParam(10, "template",
                "0 custom, 1 service preset (all fixed-size signals)",
                ValueType::U8, detail::fieldScalar<uint8_t>(0)));
            function.returnValue.present = true;
            function.returnValue.name = "status";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = statusRef;
            function.returnValue.schemaSlot = *statusSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "technician"}};
            function.contextualCallback =
                [this, &registry](const std::vector<FunctionArgument>& arguments,
                                  const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    if (arguments.size() < 4) {
                        result.errorMessage = "capture.configure requires name, rate, enabled, ids";
                        return result;
                    }
                    std::string name;
                    uint32_t rate = 0;
                    bool enabled = false;
                    uint8_t enabledByte = 0;
                    if (!detail::getString(arguments[0].value, name, 127) ||
                        !detail::getScalar(arguments[1].value, rate) ||
                        !detail::getScalar(arguments[2].value, enabledByte)) {
                        result.errorMessage = "capture.configure arguments are malformed";
                        return result;
                    }
                    enabled = enabledByte != 0;
                    // Empty array encodes as a single zero count varint.
                    std::vector<uint64_t> empty;
                    const auto ids = arguments[3].value == std::vector<uint8_t>{0}
                        ? std::optional<std::vector<uint64_t>>(std::move(empty))
                        : decodeU64Array(arguments[3].value);
                    if (!ids) {
                        result.errorMessage = "capture.configure entry_ids is malformed";
                        return result;
                    }
                    std::vector<uint64_t> selected = *ids;
                    if (arguments.size() >= 10) {
                        uint8_t preset = 0;
                        if (!detail::getScalar(arguments[9].value, preset)) {
                            result.errorMessage = "capture.configure template is malformed";
                            return result;
                        }
                        if (preset > 1) {
                            result.errorMessage = "capture.configure: unknown template";
                            return result;
                        }
                        // Service preset: every fixed-size signal in the catalog.
                        if (preset == 1 && selected.empty()) {
                            const uint32_t total = registry.signalCount();
                            for (uint32_t offset = 0; offset < total; offset += 256) {
                                for (const EntryView& entry :
                                     registry.signalPage(offset, 256)) {
                                    if (!entry.isVariableLength() &&
                                        entry.valueType() != ValueType::Stream)
                                        selected.push_back(entry.id());
                                }
                            }
                        }
                    }
                    std::optional<CaptureTrigger> trigger;
                    if (arguments.size() >= 9) {
                        uint64_t triggerId = 0;
                        uint8_t op = 0;
                        uint64_t levelBits = 0;
                        double level = 0;
                        uint32_t pre = 0, post = 0;
                        if (!detail::getScalar(arguments[4].value, triggerId) ||
                            !detail::getScalar(arguments[5].value, op) ||
                            !detail::getScalar(arguments[6].value, levelBits) ||
                            !detail::getScalar(arguments[7].value, pre) ||
                            !detail::getScalar(arguments[8].value, post)) {
                            result.errorMessage = "capture.configure trigger fields are malformed";
                            return result;
                        }
                        std::memcpy(&level, &levelBits, sizeof(level));
                        if (triggerId != 0)
                            trigger = CaptureTrigger{triggerId, op, level, pre, post};
                    }
                    if (!capture_->configure(std::move(name), rate, enabled,
                                             selected, trigger)) {
                        result.errorMessage =
                            "capture.configure rejected: unknown, variable-length, or "
                            "too many entries, an invalid trigger, or an out-of-range rate/name";
                        return result;
                    }
                    result.returnValue = encodeCaptureStatus();
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.capture.status — no arguments
        {
            FunctionEntry function;
            function.id = kCaptureStatusFnId;
            function.name = "machine.capture.status";
            function.description = "Recording state, counters, and record layout metadata";
            function.group = "machine.cia402";
            function.returnValue.present = true;
            function.returnValue.name = "status";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = statusRef;
            function.returnValue.schemaSlot = *statusSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "observer"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>&,
                       const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    result.returnValue = encodeCaptureStatus();
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.capture.cancel — stop and discard retained records
        {
            FunctionEntry function;
            function.id = kCaptureCancelFnId;
            function.name = "machine.capture.cancel";
            function.description =
                "Stop the recording and discard all retained records";
            function.group = "machine.cia402";
            function.returnValue.present = true;
            function.returnValue.name = "status";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = statusRef;
            function.returnValue.schemaSlot = *statusSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "technician"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>&,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    capture_->cancel();
                    auditAuthority(context.identity, "capture.cancel", "capture");
                    result.returnValue = encodeCaptureStatus();
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.capture.export — paginated, bounded, audited record download
        {
            FunctionEntry function;
            function.id = kCaptureExportFnId;
            function.name = "machine.capture.export";
            function.description =
                "Export a bounded chunk of retained records. Paginate via "
                "offset; chunks are hard-capped at 64 KiB. Audited per call.";
            function.group = "machine.cia402";
            function.parameters = {
                FunctionParameter{"offset", "First record index to export", ValueType::U64},
                FunctionParameter{"max_bytes", "Payload byte cap (max 64 KiB)", ValueType::U32},
            };
            function.parameters[0].key = 1;
            function.parameters[1].key = 2;
            function.returnValue.present = true;
            function.returnValue.name = "chunk";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = exportRef;
            function.returnValue.schemaSlot = *exportSlot;
            function.returnValue.maxValueSize = 70'000;
            function.metadata = {{"access.role", "technician"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    uint64_t offset = 0;
                    uint32_t maxBytes = 0;
                    if (arguments.size() != 2 ||
                        !detail::getScalar(arguments[0].value, offset) ||
                        !detail::getScalar(arguments[1].value, maxBytes)) {
                        result.errorMessage = "capture.export requires offset and max_bytes";
                        return result;
                    }
                    const auto chunk = capture_->exportChunk(offset, maxBytes);
                    result.returnValue = detail::encodeTagged({
                        {1, detail::fieldScalar(chunk.offset)},
                        {2, detail::fieldScalar(chunk.totalRecords)},
                        {3, detail::fieldScalar(chunk.droppedRecords)},
                        {4, detail::fieldScalar(chunk.recordSize)},
                        {5, detail::fieldBytes(chunk.payload)},
                    });
                    result.success = true;
                    result.error = ErrorCode::None;
                    auditAuthority(context.identity, "capture.export", "capture");
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        return true;
    }

    // ---- machine.config.* -------------------------------------------------

    std::vector<uint8_t> encodeConfigStatus() const {
        const ConfigTxnStatus status = config_->status();
        std::vector<std::vector<uint8_t>> staged;
        staged.reserve(status.staged.size());
        for (const auto& write : status.staged) {
            staged.push_back(detail::encodeTagged({
                {1, detail::fieldScalar(write.entryId)},
                {2, detail::fieldBytes(write.value)},
            }));
        }
        return detail::encodeTagged({
            {1, detail::fieldScalar(status.transactionId)},
            {2, detail::fieldScalar(static_cast<uint8_t>(status.state))},
            {3, detail::fieldScalar(status.revision)},
            {4, detail::fieldScalar(static_cast<uint32_t>(status.staged.size()))},
            {5, detail::fieldString(status.message)},
            {6, detail::encodeArray(staged)},
        });
    }

    // Transaction failures are reported inside ConfigStatusV1.state so the
    // client still receives the full status struct (a failed InvokeEx would
    // drop the return value entirely).
    static FunctionCallResult configResult(const std::vector<uint8_t>& encoded,
                                           ConfigTxnState) {
        FunctionCallResult result;
        result.returnValue = encoded;
        result.success = true;
        result.error = ErrorCode::None;
        return result;
    }

    template <typename SlotFor>
    bool addConfigFunctions(Registry& registry, const SlotFor& slotFor) {
        SchemaRef statusRef, entryArrayRef;
        const auto statusSlot = slotFor(cia402::MachineSchemaId::ConfigStatus, statusRef);
        const auto entryArraySlot =
            slotFor(cia402::MachineSchemaId::ConfigEntryArray, entryArrayRef);
        if (!statusSlot || !entryArraySlot) return false;
        SchemaRef exportRef, diffRef;
        const auto exportSlot = slotFor(cia402::MachineSchemaId::ConfigExport, exportRef);
        const auto diffSlot = slotFor(cia402::MachineSchemaId::ConfigDiff, diffRef);
        if (!exportSlot || !diffSlot) return false;

        const auto decodeEntryArray = [](const std::vector<uint8_t>& bytes,
                                         std::vector<ConfigWrite>& out,
                                         std::string& error) -> bool {
            const auto elements = decodeElementArray(bytes);
            if (!elements) { error = "entries array is malformed"; return false; }
            out.reserve(elements->size());
            for (const auto& element : *elements) {
                std::map<uint32_t, std::vector<uint8_t>> fields;
                if (!detail::parseTagged(element, fields)) {
                    error = "entry is not a tagged struct"; return false;
                }
                ConfigWrite write;
                const auto idField = fields.find(1);
                const auto valueField = fields.find(2);
                if (idField == fields.end() || valueField == fields.end() ||
                    !detail::getScalar(idField->second, write.entryId)) {
                    error = "entry requires entry_id and value"; return false;
                }
                const auto decoded = decodeLengthPrefixed(valueField->second);
                if (!decoded) { error = "entry value is malformed"; return false; }
                write.value = *decoded;
                out.push_back(std::move(write));
            }
            return true;
        };

        const auto makeConfigFunction = [&](uint64_t id, std::string name,
                                            std::string description, std::string role) {
            FunctionEntry function;
            function.id = id;
            function.name = std::move(name);
            function.description = std::move(description);
            function.group = "machine.cia402";
            function.returnValue.present = true;
            function.returnValue.name = "status";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = statusRef;
            function.returnValue.schemaSlot = *statusSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", std::move(role)}};
            return function;
        };

        // machine.config.export — live values of every writable parameter
        {
            FunctionEntry function;
            function.id = kConfigExportFnId;
            function.name = "machine.config.export";
            function.description =
                "Export the live value of every writable parameter as a "
                "ConfigEntryV1 baseline (schema-validated import target)";
            function.group = "machine.cia402";
            function.returnValue.present = true;
            function.returnValue.name = "export";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = exportRef;
            function.returnValue.schemaSlot = *exportSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "observer"}};
            function.contextualCallback =
                [this, &registry](const std::vector<FunctionArgument>&,
                                  const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    const uint32_t total = registry.paramCount();
                    std::vector<std::vector<uint8_t>> entries;
                    for (uint32_t offset = 0; offset < total; offset += 256) {
                        for (const EntryView& entry : registry.paramPage(offset, 256)) {
                            if (entry.kind() != EntryKind::Parameter ||
                                !entry.writable()) continue;
                            std::vector<uint8_t> value;
                            if (entry.isVariableLength()) {
                                value.resize(entry.maxValueSize());
                                const size_t n =
                                    entry.readVar(value.data(), value.size());
                                value.resize(n);
                            } else {
                                value.resize(entry.valueSize());
                                entry.read(value.data());
                            }
                            entries.push_back(detail::encodeTagged({
                                {1, detail::fieldScalar(entry.id())},
                                {2, detail::fieldBytes(value)},
                            }));
                        }
                    }
                    const auto status = config_->status();
                    result.returnValue = detail::encodeTagged({
                        {1, detail::fieldScalar(status.revision)},
                        {2, detail::encodeArray(entries)},
                    });
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.config.diff — compare a submitted baseline against live values
        {
            FunctionEntry function;
            function.id = kConfigDiffFnId;
            function.name = "machine.config.diff";
            function.description =
                "Diff a ConfigEntryV1 baseline (e.g. from config.export or a "
                "recipe) against live parameter values; returns mismatches only";
            function.group = "machine.cia402";
            FunctionParameter baseline;
            baseline.key = 1;
            baseline.name = "baseline";
            baseline.description = "ConfigEntryV1 array to compare against";
            baseline.type = ValueType::Array;
            baseline.schema = entryArrayRef;
            baseline.schemaSlot = *entryArraySlot;
            baseline.maxValueSize = kMaximumEncodedValue;
            function.parameters.push_back(std::move(baseline));
            function.returnValue.present = true;
            function.returnValue.name = "diff";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = diffRef;
            function.returnValue.schemaSlot = *diffSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "observer"}};
            function.contextualCallback =
                [this, &registry, decodeEntryArray](
                    const std::vector<FunctionArgument>& arguments,
                    const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    if (arguments.size() != 1) {
                        result.errorMessage = "config.diff requires a baseline array";
                        return result;
                    }
                    std::vector<ConfigWrite> baseline;
                    std::string error;
                    if (!decodeEntryArray(arguments[0].value, baseline, error)) {
                        result.errorMessage = "config.diff: " + error;
                        return result;
                    }
                    std::vector<std::vector<uint8_t>> diffs;
                    for (const auto& want : baseline) {
                        const EntryView entry = registry.find(want.entryId);
                        std::vector<uint8_t> actual;
                        if (entry && entry.kind() == EntryKind::Parameter) {
                            if (entry.isVariableLength()) {
                                actual.resize(entry.maxValueSize());
                                actual.resize(
                                    entry.readVar(actual.data(), actual.size()));
                            } else {
                                actual.resize(entry.valueSize());
                                entry.read(actual.data());
                            }
                        }
                        if (actual != want.value) {
                            diffs.push_back(detail::encodeTagged({
                                {1, detail::fieldScalar(want.entryId)},
                                {2, detail::fieldBytes(want.value)},
                                {3, detail::fieldBytes(actual)},
                            }));
                        }
                    }
                    const auto status = config_->status();
                    result.returnValue = detail::encodeTagged({
                        {1, detail::fieldScalar(status.revision)},
                        {2, detail::fieldScalar(static_cast<uint32_t>(diffs.size()))},
                        {3, detail::encodeArray(diffs)},
                    });
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.config.import — audited import that stages a validated
        // transaction; commit still requires the normal validate+commit flow.
        {
            FunctionEntry function = makeConfigFunction(
                kConfigImportFnId, "machine.config.import",
                "Import a ConfigEntryV1 baseline as a staged transaction "
                "(validated + audited; commit still required)", "technician");
            FunctionParameter entries;
            entries.key = 1;
            entries.name = "entries";
            entries.description = "ConfigEntryV1 array to stage";
            entries.type = ValueType::Array;
            entries.schema = entryArrayRef;
            entries.schemaSlot = *entryArraySlot;
            entries.maxValueSize = kMaximumEncodedValue;
            function.parameters.push_back(std::move(entries));
            function.contextualCallback =
                [this, decodeEntryArray](
                    const std::vector<FunctionArgument>& arguments,
                    const FunctionInvokeContext& context) {
                    if (arguments.size() != 1) {
                        FunctionCallResult result;
                        result.errorMessage = "config.import requires an entries array";
                        return result;
                    }
                    std::vector<ConfigWrite> writes;
                    std::string error;
                    if (!decodeEntryArray(arguments[0].value, writes, error)) {
                        FunctionCallResult result;
                        result.errorMessage = "config.import: " + error;
                        return result;
                    }
                    if (!context.identity.authenticated ||
                        context.identity.role < Role::Technician) {
                        FunctionCallResult result;
                        result.errorMessage =
                            "config.import requires a technician role";
                        return result;
                    }
                    audit(context.identity, "audit.config.import",
                  std::string("import staged ") +
                      std::to_string(writes.size()) + " parameter write(s)");
                    const auto status = config_->stage(std::move(writes), context.identity);
                    return configResult(encodeConfigStatus(), status.state);
                };
            if (!registry.addFunction(std::move(function))) return false;
        }

        // machine.config.stage — ConfigEntryArray of {entry_id, value}
        {
            FunctionEntry function = makeConfigFunction(
                kConfigStageFnId, "machine.config.stage",
                "Stage a bounded set of parameter writes; nothing is applied "
                "until validate + commit", "technician");
            FunctionParameter entries;
            entries.key = 1;
            entries.name = "entries";
            entries.description = "ConfigEntryV1 array of parameter writes";
            entries.type = ValueType::Array;
            entries.schema = entryArrayRef;
            entries.schemaSlot = *entryArraySlot;
            entries.maxValueSize = kMaximumEncodedValue;
            function.parameters.push_back(std::move(entries));
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    if (arguments.size() != 1) {
                        FunctionCallResult result;
                        result.errorMessage = "config.stage requires an entries array";
                        return result;
                    }
                    const auto elements = decodeElementArray(arguments[0].value);
                    if (!elements) {
                        FunctionCallResult result;
                        result.errorMessage = "config.stage entries are malformed";
                        return result;
                    }
                    std::vector<ConfigWrite> writes;
                    writes.reserve(elements->size());
                    for (const auto& element : *elements) {
                        std::map<uint32_t, std::vector<uint8_t>> fields;
                        if (!detail::parseTagged(element, fields)) {
                            FunctionCallResult result;
                            result.errorMessage = "config.stage entry is not a tagged struct";
                            return result;
                        }
                        ConfigWrite write;
                        const auto idField = fields.find(1);
                        const auto valueField = fields.find(2);
                        if (idField == fields.end() || valueField == fields.end() ||
                            !detail::getScalar(idField->second, write.entryId)) {
                            FunctionCallResult result;
                            result.errorMessage = "config.stage entry requires entry_id and value";
                            return result;
                        }
                        // value is a Bytes field: varint length + payload.
                        const auto decoded = decodeLengthPrefixed(valueField->second);
                        if (!decoded) {
                            FunctionCallResult result;
                            result.errorMessage = "config.stage entry value is malformed";
                            return result;
                        }
                        write.value = *decoded;
                        writes.push_back(std::move(write));
                    }
                    const auto status = config_->stage(std::move(writes), context.identity);
                    return configResult(encodeConfigStatus(), status.state);
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // validate / commit / rollback / status — no arguments
        const auto noArg = [&](uint64_t id, std::string name, std::string description,
                               std::string role,
                               ConfigTxnStatus (StagedConfigService::*verb)(
                                   const SessionIdentity&)) {
            FunctionEntry function = makeConfigFunction(id, std::move(name),
                                                        std::move(description), std::move(role));
            function.contextualCallback =
                [this, verb](const std::vector<FunctionArgument>&,
                             const FunctionInvokeContext& context) {
                    const auto status = (config_->*verb)(context.identity);
                    if (verb == &StagedConfigService::commit &&
                        status.state == ConfigTxnState::Committed)
                        ++metrics_.configCommits;
                    return configResult(encodeConfigStatus(), status.state);
                };
            return registry.addFunction(std::move(function));
        };
        if (!noArg(kConfigValidateFnId, "machine.config.validate",
                   "Validate the staged transaction against application rules",
                   "technician", &StagedConfigService::validate) ||
            !noArg(kConfigCommitFnId, "machine.config.commit",
                   "Atomically apply the validated transaction and bump the "
                   "configuration revision", "technician",
                   &StagedConfigService::commit) ||
            !noArg(kConfigRollbackFnId, "machine.config.rollback",
                   "Discard the staged transaction", "technician",
                   &StagedConfigService::rollback))
            return false;
        {
            FunctionEntry function = makeConfigFunction(
                kConfigStatusFnId, "machine.config.status",
                "Current transaction state and staged entries", "observer");
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>&,
                       const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    result.returnValue = encodeConfigStatus();
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        return true;
    }

    // ---- machine.checklist.* (optional; requires setChecklistSource) ------

    template <typename SlotFor>
    bool addChecklistFunctions(Registry& registry, const SlotFor& slotFor) {
        SchemaRef arrayRef, reportRef;
        const auto arraySlot =
            slotFor(cia402::MachineSchemaId::ChecklistItemArray, arrayRef);
        const auto reportSlot =
            slotFor(cia402::MachineSchemaId::ChecklistReport, reportRef);
        if (!arraySlot || !reportSlot) return false;

        const auto encodeItems = [](const std::vector<ChecklistItem>& items) {
            std::vector<std::vector<uint8_t>> encoded;
            encoded.reserve(items.size());
            for (const auto& item : items) {
                encoded.push_back(detail::encodeTagged({
                    {1, detail::fieldScalar(item.itemId)},
                    {2, detail::fieldString(item.label)},
                    {3, detail::fieldScalar(static_cast<uint8_t>(item.required ? 1 : 0))},
                    {4, detail::fieldScalar(static_cast<uint8_t>(item.passed ? 1 : 0))},
                    {5, detail::fieldString(item.evidence)},
                }));
            }
            return detail::encodeArray(encoded);
        };

        {
            FunctionEntry function;
            function.id = kChecklistListFnId;
            function.name = "machine.checklist.list";
            function.description =
                "Evaluate the commissioning checklist against live state and "
                "return every item with its verdict and evidence";
            function.group = "machine.cia402";
            function.returnValue.present = true;
            function.returnValue.name = "items";
            function.returnValue.type = ValueType::Array;
            function.returnValue.schema = arrayRef;
            function.returnValue.schemaSlot = *arraySlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "observer"}};
            function.contextualCallback =
                [this, encodeItems](const std::vector<FunctionArgument>&,
                                    const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    result.returnValue = encodeItems(checklist_->evaluate());
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        {
            FunctionEntry function;
            function.id = kChecklistReportFnId;
            function.name = "machine.checklist.report";
            function.description =
                "Generate a recorded acceptance report; every required item "
                "must currently pass. The report is audited.";
            function.group = "machine.cia402";
            function.returnValue.present = true;
            function.returnValue.name = "report";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = reportRef;
            function.returnValue.schemaSlot = *reportSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "technician"}};
            function.contextualCallback =
                [this, encodeItems](const std::vector<FunctionArgument>&,
                                    const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    const auto items = checklist_->evaluate();
                    const bool allPassed = std::ranges::all_of(
                        items, [](const ChecklistItem& item) {
                            return item.passed || !item.required;
                        });
                    if (!allPassed) {
                        result.errorMessage =
                            "Acceptance report refused: required checklist items are failing";
                        return result;
                    }
                    const uint64_t revision =
                        config_ ? config_->status().revision : 0;
                    result.returnValue = detail::encodeTagged({
                        {1, detail::fieldScalar(detail::nowUs())},
                        {2, detail::fieldScalar(revision)},
                        {3, detail::fieldScalar(static_cast<uint8_t>(1))},
                        {4, encodeItems(items)},
                    });
                    result.success = true;
                    result.error = ErrorCode::None;
                    audit(context.identity, "audit.checklist.report",
                  "acceptance report generated, revision " +
                      std::to_string(revision));
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        return true;
    }

    // ---- machine.app.profile (optional; requires setAppProfileSource) ------

    template <typename SlotFor>
    bool addAppProfileSignal(Registry& registry, const SlotFor& slotFor) {
        SchemaRef profileRef;
        const auto profileSlot = slotFor(cia402::MachineSchemaId::AppProfile,
                                         profileRef);
        if (!profileSlot) return false;
        SignalEntry signal{};
        signal.id = kAppProfileSignalId;
        signal.name = "machine.app.profile";
        signal.description =
            "Signed application profile document (integrity verified "
            "server-side at attach time)";
        signal.group = "machine.cia402";
        signal.valueType = ValueType::Struct;
        signal.schema = profileRef;
        signal.schemaSlot = *profileSlot;
        signal.maxValueSize = kMaximumEncodedValue;
        signal.metadata = {{"access.role", "observer"},
                           {"schema.name", "tether.machine.cia402.AppProfileV1"}};
        signal.varReadFn = [this](void* destination, size_t capacity) {
            const MachineAppProfile& profile = appProfile_->profile();
            const auto bytes = detail::encodeTagged({
                {1, detail::fieldString(profile.name)},
                {2, detail::fieldString(profile.version)},
                {3, detail::fieldBytes(profile.document)},
                {4, detail::fieldBytes(std::vector<uint8_t>(
                        profile.mac.begin(), profile.mac.end()))},
            });
            if (bytes.size() > capacity) return size_t{0};
            std::memcpy(destination, bytes.data(), bytes.size());
            return bytes.size();
        };
        return registry.addSignal(std::move(signal));
    }

    // ---- machine.recipe.* (optional; requires setRecipeStore + config) -----

    template <typename SlotFor>
    bool addRecipeFunctions(Registry& registry, const SlotFor& slotFor) {
        SchemaRef arrayRef, statusRef;
        const auto arraySlot =
            slotFor(cia402::MachineSchemaId::RecipeInfoArray, arrayRef);
        const auto statusSlot =
            slotFor(cia402::MachineSchemaId::ConfigStatus, statusRef);
        if (!arraySlot || !statusSlot) return false;

        {
            FunctionEntry function;
            function.id = kRecipeListFnId;
            function.name = "machine.recipe.list";
            function.description =
                "List the named parameter sets offered by the recipe store";
            function.group = "machine.cia402";
            function.returnValue.present = true;
            function.returnValue.name = "recipes";
            function.returnValue.type = ValueType::Array;
            function.returnValue.schema = arrayRef;
            function.returnValue.schemaSlot = *arraySlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "observer"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>&,
                       const FunctionInvokeContext&) {
                    FunctionCallResult result;
                    std::vector<std::vector<uint8_t>> encoded;
                    for (const auto& recipe : recipes_->list()) {
                        encoded.push_back(detail::encodeTagged({
                            {1, detail::fieldString(recipe.name)},
                            {2, detail::fieldString(recipe.description)},
                            {3, detail::fieldScalar(recipe.entryCount)},
                        }));
                    }
                    result.returnValue = detail::encodeArray(encoded);
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        {
            FunctionEntry function;
            function.id = kRecipeApplyFnId;
            function.name = "machine.recipe.apply";
            function.description =
                "Stage a named recipe as a configuration transaction. "
                "Validate + commit are still required; nothing is applied "
                "by this call. The call is audited.";
            function.group = "machine.cia402";
            FunctionParameter name;
            name.key = 1;
            name.name = "recipe";
            name.description = "Recipe name from machine.recipe.list";
            name.type = ValueType::String;
            name.maxValueSize = 128;
            function.parameters.push_back(std::move(name));
            function.returnValue.present = true;
            function.returnValue.name = "status";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = statusRef;
            function.returnValue.schemaSlot = *statusSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "technician"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    if (!context.identity.authenticated ||
                        context.identity.role < Role::Technician) {
                        result.errorMessage =
                            "machine.recipe.apply requires the technician role";
                        return result;
                    }
                    std::string recipeName;
                    if (arguments.empty() ||
                        !detail::getString(arguments[0].value, recipeName)) {
                        result.errorMessage =
                            "machine.recipe.apply requires a recipe name";
                        return result;
                    }
                    std::vector<ConfigWrite> writes;
                    if (!recipes_->recipe(recipeName, writes)) {
                        result.errorMessage =
                            "Unknown recipe: " + recipeName;
                        return result;
                    }
                    const auto status =
                        config_->stage(std::move(writes), context.identity);
                    audit(context.identity, "audit.recipe.apply",
                          "recipe '" + recipeName + "' staged, txn " +
                              std::to_string(status.transactionId));
                    result.returnValue = encodeConfigStatus();
                    result.success = status.state == ConfigTxnState::Staged;
                    if (result.success) ++metrics_.recipesApplied;
                    result.error = result.success ? ErrorCode::None
                                                  : ErrorCode::InvalidMessage;
                    if (!result.success)
                        result.errorMessage = status.message;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        return true;
    }

    // ---- machine.metrics (core; informational counters) --------------------

    template <typename SlotFor>
    bool addMetricsFunction(Registry& registry, const SlotFor& slotFor) {
        SchemaRef metricsRef;
        const auto metricsSlot =
            slotFor(cia402::MachineSchemaId::Metrics, metricsRef);
        if (!metricsSlot) return false;
        FunctionEntry function;
        function.id = kMetricsFnId;
        function.name = "machine.metrics";
        function.description =
            "Process-local operational counters (MetricsV1); informational "
            "only, reset on restart, never safety-relevant";
        function.group = "machine.cia402";
        function.returnValue.present = true;
        function.returnValue.name = "metrics";
        function.returnValue.type = ValueType::Struct;
        function.returnValue.schema = metricsRef;
        function.returnValue.schemaSlot = *metricsSlot;
        function.returnValue.maxValueSize = 512;
        function.metadata = {{"access.role", "observer"}};
        function.contextualCallback =
            [this](const std::vector<FunctionArgument>&,
                   const FunctionInvokeContext&) {
                FunctionCallResult result;
                result.returnValue = detail::encodeTagged({
                    {1, detail::fieldScalar(metrics_.commandsReceived.load())},
                    {2, detail::fieldScalar(metrics_.commandsDenied.load())},
                    {3, detail::fieldScalar(metrics_.commandsDispatched.load())},
                    {4, detail::fieldScalar(metrics_.auditsWritten.load())},
                    {5, detail::fieldScalar(metrics_.configCommits.load())},
                    {6, detail::fieldScalar(metrics_.recipesApplied.load())},
                });
                result.success = true;
                result.error = ErrorCode::None;
                return result;
            };
        return registry.addFunction(std::move(function));
    }

    /// Decode a Bytes field payload: varint length + bytes, fully consumed.
    static std::optional<std::vector<uint8_t>> decodeLengthPrefixed(
        const std::vector<uint8_t>& bytes) {
        size_t position = 0;
        uint64_t length = 0;
        for (unsigned index = 0; index < 10 && position < bytes.size(); ++index) {
            const uint8_t byte = bytes[position++];
            if (index == 9 && byte > 1) return std::nullopt;
            length |= static_cast<uint64_t>(byte & 0x7F) << (index * 7U);
            if ((byte & 0x80) == 0) break;
        }
        if (position == 0 || position > bytes.size() ||
            length > bytes.size() - position || position + length != bytes.size())
            return std::nullopt;
        return std::vector<uint8_t>(
            bytes.begin() + static_cast<ptrdiff_t>(position),
            bytes.begin() + static_cast<ptrdiff_t>(position + length));
    }

    /// Decode a dynamic array into raw element payloads.
    static std::optional<std::vector<std::vector<uint8_t>>> decodeElementArray(
        const std::vector<uint8_t>& bytes) {
        size_t position = 0;
        const auto varint = [&]() -> std::optional<uint64_t> {
            uint64_t value = 0;
            for (unsigned index = 0; index < 10; ++index) {
                if (position >= bytes.size()) return std::nullopt;
                const uint8_t byte = bytes[position++];
                if (index == 9 && byte > 1) return std::nullopt;
                value |= static_cast<uint64_t>(byte & 0x7F) << (index * 7U);
                if ((byte & 0x80) == 0) return value;
            }
            return std::nullopt;
        };
        const auto count = varint();
        if (!count || *count > 512) return std::nullopt;
        std::vector<std::vector<uint8_t>> elements;
        for (uint64_t index = 0; index < *count; ++index) {
            const auto length = varint();
            if (!length || *length > bytes.size() - position) return std::nullopt;
            elements.emplace_back(
                bytes.begin() + static_cast<ptrdiff_t>(position),
                bytes.begin() + static_cast<ptrdiff_t>(position + *length));
            position += static_cast<size_t>(*length);
        }
        if (position != bytes.size()) return std::nullopt;
        return elements;
    }

    /// machine.diagnostics — registered only when a pluggable
    /// IMachineDiagnosticsSource has been attached via setDiagnosticsSource().
    template <typename SlotFor>
    bool addDiagnosticsSignal(Registry& registry, const SlotFor& slotFor) {
        SchemaRef diagnosticsRef;
        const auto slot = slotFor(cia402::MachineSchemaId::MachineDiagnostics, diagnosticsRef);
        if (!slot) return false;
        SignalEntry signal{};
        signal.id = kDiagnosticsSignalId;
        signal.name = "machine.diagnostics";
        signal.description =
            "Cyclic-loop, DC, mailbox, and link counters (diagnostic only — "
            "not a protective or functional-safety signal)";
        signal.group = "machine.cia402";
        signal.valueType = ValueType::Struct;
        signal.schema = diagnosticsRef;
        signal.schemaSlot = *slot;
        signal.maxValueSize = kMaximumEncodedValue;
        signal.metadata = {{"access.role", "observer"},
                           {"schema.name", "tether.machine.cia402.MachineDiagnosticsV1"}};
        signal.varReadFn = [this](void* destination, size_t capacity) {
            const auto bytes = encodeDiagnostics(diagnostics_->read());
            if (bytes.size() > capacity) return size_t{0};
            std::memcpy(destination, bytes.data(), bytes.size());
            return bytes.size();
        };
        return registry.addSignal(std::move(signal));
    }

    /// machine.sdo.list / read / write — registered only when a pluggable
    /// IMachineSdoAccess backend has been attached via setSdoService().
    template <typename SlotFor>
    bool addSdoFunctions(Registry& registry, const SlotFor& slotFor) {
        SchemaRef listRef, requestRef, writeRequestRef, resultRef;
        const auto listSlot = slotFor(cia402::MachineSchemaId::SdoEntryArray, listRef);
        const auto requestSlot = slotFor(cia402::MachineSchemaId::SdoRequest, requestRef);
        const auto writeSlot = slotFor(cia402::MachineSchemaId::SdoWriteRequest, writeRequestRef);
        const auto resultSlot = slotFor(cia402::MachineSchemaId::SdoResult, resultRef);
        if (!listSlot || !requestSlot || !writeSlot || !resultSlot) return false;

        // machine.sdo.list — known object-dictionary entries for a slave
        {
            FunctionEntry function;
            function.id = kSdoListFnId;
            function.name = "machine.sdo.list";
            function.description =
                "List known object-dictionary entries for a slave. Entries come "
                "from the backend's device profile; an empty list means no "
                "profile is installed, not that the slave is absent.";
            function.group = "machine.cia402";
            FunctionParameter slave;
            slave.key = 1;
            slave.name = "slave";
            slave.description = "EtherCAT slave position";
            slave.type = ValueType::U16;
            function.parameters.push_back(std::move(slave));
            function.returnValue.present = true;
            function.returnValue.name = "entries";
            function.returnValue.type = ValueType::Array;
            function.returnValue.schema = listRef;
            function.returnValue.schemaSlot = *listSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue * 4;
            function.metadata = {{"access.role", "observer"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    uint16_t slave = 0;
                    if (arguments.size() != 1 ||
                        !detail::getScalar(arguments[0].value, slave)) {
                        result.errorMessage = "machine.sdo.list requires a slave index";
                        return result;
                    }
                    return sdo_->list(slave, context.identity);
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.sdo.read — explicit-index upload
        {
            FunctionEntry function;
            function.id = kSdoReadFnId;
            function.name = "machine.sdo.read";
            function.description =
                "CoE SDO upload by explicit index. The transfer runs on this "
                "session's thread; it is not a realtime-cyclic operation.";
            function.group = "machine.cia402";
            FunctionParameter request;
            request.key = 1;
            request.name = "request";
            request.description = "SdoRequestV1 tagged struct";
            request.type = ValueType::Struct;
            request.schema = requestRef;
            request.schemaSlot = *requestSlot;
            request.maxValueSize = 256;
            function.parameters.push_back(std::move(request));
            function.returnValue.present = true;
            function.returnValue.name = "result";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = resultRef;
            function.returnValue.schemaSlot = *resultSlot;
            function.returnValue.maxValueSize = 1024;
            function.metadata = {{"access.role", "observer"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    if (arguments.size() != 1) {
                        result.errorMessage = "machine.sdo.read requires a request struct";
                        return result;
                    }
                    return sdo_->read(arguments[0].value, context.identity);
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.sdo.write — explicit-index download, technician+, audited
        {
            FunctionEntry function;
            function.id = kSdoWriteFnId;
            function.name = "machine.sdo.write";
            function.description =
                "CoE SDO download by explicit index. Writes are journaled and "
                "require a technician role; the browser is not a safety "
                "channel — drive-side interlocks still apply.";
            function.group = "machine.cia402";
            FunctionParameter request;
            request.key = 1;
            request.name = "request";
            request.description = "SdoWriteRequestV1 tagged struct";
            request.type = ValueType::Struct;
            request.schema = writeRequestRef;
            request.schemaSlot = *writeSlot;
            request.maxValueSize = 1024;
            function.parameters.push_back(std::move(request));
            function.returnValue.present = true;
            function.returnValue.name = "result";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = resultRef;
            function.returnValue.schemaSlot = *resultSlot;
            function.returnValue.maxValueSize = 1024;
            function.metadata = {{"access.role", "technician"},
                                 {"access.danger", "parameter-write"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    if (arguments.size() != 1) {
                        result.errorMessage = "machine.sdo.write requires a request struct";
                        return result;
                    }
                    return sdo_->write(arguments[0].value, context.identity);
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        return true;
    }

    FunctionCallResult handleCommand(const std::vector<FunctionArgument>& arguments,
                                     const FunctionInvokeContext& context) {
        FunctionCallResult result;
        if (arguments.size() != 1) {
            result.errorMessage = "machine.command requires a CommandRequestV1 struct";
            return result;
        }
        std::map<uint32_t, std::vector<uint8_t>> fields;
        if (!detail::parseTagged(arguments[0].value, fields)) {
            result.errorMessage = "CommandRequestV1 payload is malformed";
            return result;
        }
        CommandRequest request;
        uint8_t action = 0;
        uint64_t deadlineUs = 0, revision = 0;
        std::vector<uint8_t> targetsBytes, generationsBytes;
        const auto required = [&fields](uint32_t key) -> const std::vector<uint8_t>* {
            const auto it = fields.find(key);
            return it == fields.end() ? nullptr : &it->second;
        };
        const auto fail = [&result](std::string message) {
            result.errorMessage = std::move(message);
            return result;
        };
        if (const auto* value = required(1); !value ||
            !detail::getString(*value, request.requestUuid, 127))
            return fail("request_uuid is missing or invalid");
        if (const auto* value = required(2); !value ||
            !detail::getString(*value, request.scope, 127))
            return fail("scope is missing or invalid");
        if (const auto* value = required(3); !value ||
            !detail::getScalar(*value, request.authorityToken))
            return fail("authority_token is missing or invalid");
        if (const auto* value = required(4); !value ||
            !detail::getScalar(*value, action))
            return fail("action is missing or invalid");
        if (const auto* value = required(5)) targetsBytes = *value; else
            return fail("targets are missing");
        if (const auto* value = required(6)) generationsBytes = *value;
        if (const auto* value = required(7); !value ||
            !detail::getScalar(*value, deadlineUs))
            return fail("deadline_us is missing or invalid");
        if (const auto* value = required(8)) {
            if (!detail::getScalar(*value, revision))
                return fail("configuration_revision is invalid");
        }
        if (const auto* value = required(9)) {
            // Bytes fields are length-prefixed (varint length + payload).
            size_t position = 0;
            uint64_t length = 0;
            bool ok = false;
            for (unsigned index = 0; index < 10 && position < value->size(); ++index) {
                const uint8_t byte = (*value)[position++];
                if (index == 9 && byte > 1) break;
                length |= static_cast<uint64_t>(byte & 0x7F) << (index * 7U);
                if ((byte & 0x80) == 0) { ok = true; break; }
            }
            if (!ok || length > value->size() - position)
                return fail("parameters are malformed");
            request.parameters.assign(value->begin() + static_cast<ptrdiff_t>(position),
                                      value->begin() + static_cast<ptrdiff_t>(position + length));
            if (position + length != value->size())
                return fail("parameters have trailing bytes");
        }

        const auto strings = decodeStringArray(targetsBytes);
        const auto generations = decodeU64Array(generationsBytes);
        if (!strings || strings->empty() || !generations ||
            strings->size() != generations->size())
            return fail("targets and expected_generations must be non-empty parallel arrays");
        request.targets = *strings;
        for (size_t index = 0; index < request.targets.size(); ++index)
            request.expectedGenerations[request.targets[index]] = (*generations)[index];
        if (revision != 0) request.configurationRevision = revision;
        // deadline_us is a relative validity budget in microseconds —
        // clients cannot express the server's steady clock. Zero or an
        // already-past instant is rejected by the gate.
        const auto now = SteadyClock::now();
        request.deadline = deadlineUs > 0
            ? now + std::chrono::microseconds(deadlineUs) : now;
        request.action = static_cast<Action>(action);

        ++metrics_.commandsReceived;
        const CommandReceipt receipt = gate_.submit(request, context.identity, now);
        if (receipt.state == CommandState::Rejected ||
            receipt.state == CommandState::Failed)
            ++metrics_.commandsDenied;
        else
            ++metrics_.commandsDispatched;
        if (!receipt.operationId.empty()) {
            // The dispatcher created the operation; the gate's dispatch ran
            // before the tracker entry for legacy dispatchers — nothing to do
            // here besides surfacing the receipt.
        }
        result.returnValue = detail::encodeTagged({
            {1, detail::fieldString(receipt.requestUuid)},
            {2, detail::fieldScalar(static_cast<uint8_t>(receipt.state))},
            {3, detail::fieldString(receipt.message)},
            {4, detail::fieldString(receipt.operationId)},
            {5, detail::fieldString(receipt.auditId)},
            {6, detail::encodeArray([&] {
                std::vector<std::vector<uint8_t>> blockers;
                blockers.reserve(receipt.blockers.size());
                for (const auto& blocker : receipt.blockers)
                    blockers.push_back(detail::fieldString(blocker));
                return blockers;
            }())},
        });
        result.success = true;
        result.error = ErrorCode::None;
        return result;
    }

    static std::optional<std::vector<std::string>> decodeStringArray(
        const std::vector<uint8_t>& bytes) {
        size_t position = 0;
        const auto varint = [&]() -> std::optional<uint64_t> {
            uint64_t value = 0;
            for (unsigned index = 0; index < 10; ++index) {
                if (position >= bytes.size()) return std::nullopt;
                const uint8_t byte = bytes[position++];
                if (index == 9 && byte > 1) return std::nullopt;
                value |= static_cast<uint64_t>(byte & 0x7F) << (index * 7U);
                if ((byte & 0x80) == 0) return value;
            }
            return std::nullopt;
        };
        const auto count = varint();
        if (!count || *count > 128) return std::nullopt;
        std::vector<std::string> out;
        for (uint64_t index = 0; index < *count; ++index) {
            const auto length = varint();
            if (!length || *length > bytes.size() - position) return std::nullopt;
            std::string value;
            const std::vector<uint8_t> element(
                bytes.begin() + static_cast<ptrdiff_t>(position),
                bytes.begin() + static_cast<ptrdiff_t>(position + *length));
            position += static_cast<size_t>(*length);
            if (!detail::getString(element, value, 127)) return std::nullopt;
            out.push_back(std::move(value));
        }
        if (position != bytes.size()) return std::nullopt;
        return out;
    }

    static std::optional<std::vector<uint64_t>> decodeU64Array(
        const std::vector<uint8_t>& bytes) {
        size_t position = 0;
        const auto varint = [&]() -> std::optional<uint64_t> {
            uint64_t value = 0;
            for (unsigned index = 0; index < 10; ++index) {
                if (position >= bytes.size()) return std::nullopt;
                const uint8_t byte = bytes[position++];
                if (index == 9 && byte > 1) return std::nullopt;
                value |= static_cast<uint64_t>(byte & 0x7F) << (index * 7U);
                if ((byte & 0x80) == 0) return value;
            }
            return std::nullopt;
        };
        const auto count = varint();
        if (!count || *count > 128) return std::nullopt;
        std::vector<uint64_t> out;
        for (uint64_t index = 0; index < *count; ++index) {
            const auto length = varint();
            if (!length || *length != 8 || *length > bytes.size() - position)
                return std::nullopt;
            uint64_t value = 0;
            if (!detail::getScalar(std::vector<uint8_t>(
                    bytes.begin() + static_cast<ptrdiff_t>(position),
                    bytes.begin() + static_cast<ptrdiff_t>(position + 8)), value))
                return std::nullopt;
            position += 8;
            out.push_back(value);
        }
        if (position != bytes.size()) return std::nullopt;
        return out;
    }

    /// Append a typed audit event (`audit.<surface>`) for non-authority
    /// actions like config import or checklist reports.
    void audit(const SessionIdentity& identity, std::string_view eventType,
               std::string_view description) {
        if (!auditJournal_) return;
        auditJournal_->append({0, detail::nowUs(), 0, std::string(eventType),
            std::string(identity.actor), EventSeverity::Info, 0,
            std::string(description)});
        ++metrics_.auditsWritten;
    }

    void auditAuthority(const SessionIdentity& identity, std::string_view verb,
                        std::string_view scope) {
        if (!auditJournal_) return;
        auditJournal_->append({0, detail::nowUs(), 0, "audit.authority",
            std::string(scope), EventSeverity::Info, 0,
            std::string(identity.actor) + " " + std::string(verb) +
                " authority on " + std::string(scope)});
        ++metrics_.auditsWritten;
    }

    /// machine.pdo.map — registered only when an IMachinePdoSource is
    /// attached via setPdoSource().
    template <typename SlotFor>
    bool addPdoFunctions(Registry& registry, const SlotFor& slotFor) {
        SchemaRef entriesRef;
        const auto entriesSlot = slotFor(cia402::MachineSchemaId::PdoEntryArray, entriesRef);
        if (!entriesSlot) return false;
        FunctionEntry function;
        function.id = kPdoMapFnId;
        function.name = "machine.pdo.map";
        function.description =
            "Logical-address placement of every enabled PDO mapping entry "
            "(slave, PDO index, direction, offset, length). Read-only view of "
            "the process-image layout.";
        function.group = "machine.cia402";
        function.returnValue.present = true;
        function.returnValue.name = "entries";
        function.returnValue.type = ValueType::Array;
        function.returnValue.schema = entriesRef;
        function.returnValue.schemaSlot = *entriesSlot;
        function.returnValue.maxValueSize = kMaximumEncodedValue * 4;
        function.metadata = {{"access.role", "observer"}};
        function.contextualCallback =
            [this](const std::vector<FunctionArgument>& arguments,
                   const FunctionInvokeContext& context) {
                FunctionCallResult result;
                if (!arguments.empty() || !context.identity.authenticated) {
                    result.errorMessage =
                        "machine.pdo.map takes no arguments and requires an authenticated session";
                    return result;
                }
                std::vector<std::vector<uint8_t>> entries;
                for (const auto& entry : pdo_->entries()) {
                    entries.push_back(detail::encodeTagged({
                        {1, detail::fieldScalar(entry.slaveIndex)},
                        {2, detail::fieldScalar(entry.pdoIndex)},
                        {3, detail::fieldScalar(entry.direction)},
                        {4, detail::fieldScalar(entry.offset)},
                        {5, detail::fieldScalar(entry.length)},
                        {6, detail::fieldScalar(entry.entryIndex)},
                    }));
                }
                result.returnValue = detail::encodeArray(entries);
                result.success = true;
                result.error = ErrorCode::None;
                return result;
            };
        return registry.addFunction(std::move(function));
    }

    /// machine.supervisor.status / retry — registered only when a
    /// MachineSupervisorService is attached via setSupervisor().
    template <typename SlotFor>
    bool addSupervisorFunctions(Registry& registry, const SlotFor& slotFor) {
        SchemaRef statusRef, resultRef;
        const auto statusSlot =
            slotFor(cia402::MachineSchemaId::SupervisorEntryArray, statusRef);
        const auto resultSlot =
            slotFor(cia402::MachineSchemaId::SupervisorResult, resultRef);
        if (!statusSlot || !resultSlot) return false;

        // machine.supervisor.status — per-slave recovery states
        {
            FunctionEntry function;
            function.id = kSupervisorStatusFnId;
            function.name = "machine.supervisor.status";
            function.description =
                "SlaveSupervisor recovery state per slave: state, suspended, "
                "recovering, attempt count. Diagnostic only — not a "
                "protective channel.";
            function.group = "machine.cia402";
            function.returnValue.present = true;
            function.returnValue.name = "entries";
            function.returnValue.type = ValueType::Array;
            function.returnValue.schema = statusRef;
            function.returnValue.schemaSlot = *statusSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "observer"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    if (!arguments.empty() || !context.identity.authenticated) {
                        result.errorMessage =
                            "machine.supervisor.status takes no arguments and requires an authenticated session";
                        return result;
                    }
                    result.returnValue = supervisor_->encodeStatus();
                    result.success = true;
                    result.error = ErrorCode::None;
                    return result;
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        // machine.supervisor.retry — bounded, audited recovery retry
        {
            FunctionEntry function;
            function.id = kSupervisorRetryFnId;
            function.name = "machine.supervisor.retry";
            function.description =
                "Controlled recovery retry for one slave: clears a failed "
                "recovery state and triggers a bounded recovery pass. "
                "Technician role, one retry per second server-wide, audited. "
                "Hardware safety interlocks are unaffected.";
            function.group = "machine.cia402";
            FunctionParameter slave;
            slave.key = 1;
            slave.name = "slave";
            slave.description = "EtherCAT slave position";
            slave.type = ValueType::U16;
            function.parameters.push_back(std::move(slave));
            function.returnValue.present = true;
            function.returnValue.name = "result";
            function.returnValue.type = ValueType::Struct;
            function.returnValue.schema = resultRef;
            function.returnValue.schemaSlot = *resultSlot;
            function.returnValue.maxValueSize = kMaximumEncodedValue;
            function.metadata = {{"access.role", "technician"}};
            function.contextualCallback =
                [this](const std::vector<FunctionArgument>& arguments,
                       const FunctionInvokeContext& context) {
                    FunctionCallResult result;
                    uint16_t slave = 0;
                    if (arguments.size() != 1 ||
                        !detail::getScalar(arguments[0].value, slave)) {
                        result.errorMessage =
                            "machine.supervisor.retry requires a slave index";
                        return result;
                    }
                    return supervisor_->retry(slave, context.identity);
                };
            if (!registry.addFunction(std::move(function))) return false;
        }
        return true;
    }

    IMachineStateSource& stateSource_;
    IMachineDispatcher& dispatcher_;
    ControlAuthority& authority_;
    MachineCommandGate& gate_;
    AlarmService& alarms_;
    OperationTracker& operations_;
    EventJournal* auditJournal_;
    MachineCaptureService* capture_ = nullptr;
    StagedConfigService* config_ = nullptr;
    MachineSdoService* sdo_ = nullptr;
    IMachineDiagnosticsSource* diagnostics_ = nullptr;
    IMachinePdoSource* pdo_ = nullptr;
    IMachineChecklistSource* checklist_ = nullptr;
    MachineSupervisorService* supervisor_ = nullptr;
    IMachineAppProfileSource* appProfile_ = nullptr;
    IMachineRecipeStore* recipes_ = nullptr;
    struct {
        std::atomic<uint64_t> commandsReceived{0};
        std::atomic<uint64_t> commandsDenied{0};
        std::atomic<uint64_t> commandsDispatched{0};
        std::atomic<uint64_t> auditsWritten{0};
        std::atomic<uint64_t> configCommits{0};
        std::atomic<uint64_t> recipesApplied{0};
    } metrics_;
};

} // namespace tether::io::machine
