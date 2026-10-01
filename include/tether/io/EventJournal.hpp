#pragma once

/**
 * @file EventJournal.hpp
 * @brief Bounded, cursor-paginated read-only event history.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace tether::io {

enum class EventSeverity : uint8_t {
    Info = 0,
    Warning = 1,
    Fault = 2,
    Critical = 3,
};

struct EventRecordV1 {
    uint64_t eventId = 0;
    uint64_t timestampUs = 0;
    uint64_t stateGeneration = 0;
    std::string eventType;
    std::string sourceId;
    EventSeverity severity = EventSeverity::Info;
    uint32_t code = 0;
    std::string description;
};

struct EventPageV1 {
    uint64_t oldestCursor = 0;
    uint64_t latestCursor = 0;
    uint64_t nextCursor = 0;
    bool gap = false;
    std::vector<EventRecordV1> events;
};

/**
 * Thread-safe bounded event history. IDs are strictly increasing and never
 * reused during the journal lifetime. If retention has evicted records after
 * a client's cursor, the next page sets `gap` and begins at the oldest retained
 * event. This service is observational only; it has no acknowledgement or
 * command semantics.
 */
class EventJournal {
public:
    explicit EventJournal(size_t capacity = 1024) : capacity_(capacity) {
        if (capacity_ == 0) throw std::invalid_argument("event journal capacity must be nonzero");
    }

    uint64_t append(EventRecordV1 event) {
        validate(event);
        std::lock_guard lock(mutex_);
        if (nextEventId_ == 0) throw std::overflow_error("event journal cursor exhausted");
        event.eventId = nextEventId_++;
        latestCursor_ = event.eventId;
        events_.push_back(std::move(event));
        if (events_.size() > capacity_) events_.pop_front();
        return events_.back().eventId;
    }

    EventPageV1 readAfter(uint64_t cursor, size_t limit) const {
        if (limit == 0) throw std::invalid_argument("event page limit must be nonzero");
        std::lock_guard lock(mutex_);
        EventPageV1 page;
        if (events_.empty()) {
            page.nextCursor = std::min(cursor, latestCursor_);
            page.latestCursor = latestCursor_;
            return page;
        }

        page.oldestCursor = events_.front().eventId;
        page.latestCursor = latestCursor_;
        page.gap = cursor < page.oldestCursor - 1;
        const uint64_t effectiveCursor = page.gap ? page.oldestCursor - 1 : cursor;
        for (const auto& event : events_) {
            if (event.eventId <= effectiveCursor) continue;
            page.events.push_back(event);
            if (page.events.size() == limit) break;
        }
        page.nextCursor = page.events.empty() ? std::min(cursor, latestCursor_)
                                               : page.events.back().eventId;
        return page;
    }

    uint64_t latestCursor() const {
        std::lock_guard lock(mutex_);
        return latestCursor_;
    }

    size_t retainedCount() const {
        std::lock_guard lock(mutex_);
        return events_.size();
    }

    size_t capacity() const noexcept { return capacity_; }

private:
    static void validate(const EventRecordV1& event) {
        const auto validText = [](std::string_view value, size_t maxLength) {
            return !value.empty() && value.size() <= maxLength &&
                   value.find('\0') == std::string_view::npos;
        };
        if (!validText(event.eventType, 127) || !validText(event.sourceId, 127) ||
            !validText(event.description, 512) ||
            static_cast<uint8_t>(event.severity) > static_cast<uint8_t>(EventSeverity::Critical)) {
            throw std::invalid_argument("event record is outside profile bounds");
        }
    }

    const size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<EventRecordV1> events_;
    uint64_t nextEventId_ = 1;
    uint64_t latestCursor_ = 0;
};

} // namespace tether::io
