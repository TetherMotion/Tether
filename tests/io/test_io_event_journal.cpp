#include <tether/io/EventJournal.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace tether::io;

namespace {

EventRecordV1 event(std::string type, uint64_t generation = 0) {
    EventRecordV1 record;
    record.timestampUs = 1000 + generation;
    record.stateGeneration = generation;
    record.eventType = std::move(type);
    record.sourceId = "sim-axis-x";
    record.severity = EventSeverity::Info;
    record.description = "simulated event";
    return record;
}

TEST(EventJournalTest, AssignsIncreasingCursorsAndPaginates) {
    EventJournal journal(8);
    EXPECT_EQ(journal.append(event("Started", 1)), 1u);
    EXPECT_EQ(journal.append(event("StateChanged", 2)), 2u);
    EXPECT_EQ(journal.append(event("Stopped", 3)), 3u);

    const auto first = journal.readAfter(0, 2);
    EXPECT_FALSE(first.gap);
    EXPECT_EQ(first.oldestCursor, 1u);
    EXPECT_EQ(first.latestCursor, 3u);
    EXPECT_EQ(first.nextCursor, 2u);
    ASSERT_EQ(first.events.size(), 2u);
    EXPECT_EQ(first.events[0].eventType, "Started");
    EXPECT_EQ(first.events[1].eventId, 2u);

    const auto second = journal.readAfter(first.nextCursor, 2);
    ASSERT_EQ(second.events.size(), 1u);
    EXPECT_EQ(second.events.front().eventId, 3u);
    EXPECT_EQ(second.nextCursor, 3u);
}

TEST(EventJournalTest, ReportsRetentionGapAndStartsAtOldestRetainedEvent) {
    EventJournal journal(3);
    for (uint64_t generation = 1; generation <= 5; ++generation)
        journal.append(event("StateChanged", generation));

    EXPECT_EQ(journal.retainedCount(), 3u);
    const auto page = journal.readAfter(0, 2);
    EXPECT_TRUE(page.gap);
    EXPECT_EQ(page.oldestCursor, 3u);
    EXPECT_EQ(page.latestCursor, 5u);
    ASSERT_EQ(page.events.size(), 2u);
    EXPECT_EQ(page.events[0].eventId, 3u);
    EXPECT_EQ(page.events[1].eventId, 4u);
    EXPECT_EQ(page.nextCursor, 4u);

    const auto resumed = journal.readAfter(page.nextCursor, 4);
    EXPECT_FALSE(resumed.gap);
    ASSERT_EQ(resumed.events.size(), 1u);
    EXPECT_EQ(resumed.events.front().eventId, 5u);
}

TEST(EventJournalTest, EmptyAndFutureCursorReadsAreStable) {
    EventJournal journal;
    EXPECT_EQ(journal.readAfter(0, 10).nextCursor, 0u);
    journal.append(event("Started"));

    const auto page = journal.readAfter(100, 10);
    EXPECT_FALSE(page.gap);
    EXPECT_TRUE(page.events.empty());
    EXPECT_EQ(page.latestCursor, 1u);
    EXPECT_EQ(page.nextCursor, 1u);
}

TEST(EventJournalTest, RejectsInvalidCapacityLimitsAndRecords) {
    EXPECT_THROW(EventJournal(0), std::invalid_argument);
    EventJournal journal(2);
    EXPECT_THROW(journal.readAfter(0, 0), std::invalid_argument);
    EXPECT_THROW(journal.append(event("")), std::invalid_argument);

    auto invalid = event("Started");
    invalid.description.assign(513, 'x');
    EXPECT_THROW(journal.append(std::move(invalid)), std::invalid_argument);
}

TEST(EventJournalTest, ConcurrentAppendsRetainUniqueMonotonicIds) {
    constexpr size_t threadCount = 4;
    constexpr size_t eventsPerThread = 100;
    EventJournal journal(threadCount * eventsPerThread);
    std::mutex idsMutex;
    std::vector<uint64_t> ids;
    std::vector<std::thread> threads;
    for (size_t thread = 0; thread < threadCount; ++thread) {
        threads.emplace_back([&, thread] {
            std::vector<uint64_t> local;
            for (size_t index = 0; index < eventsPerThread; ++index)
                local.push_back(journal.append(event("Concurrent", thread * eventsPerThread + index)));
            std::lock_guard lock(idsMutex);
            ids.insert(ids.end(), local.begin(), local.end());
        });
    }
    for (auto& worker : threads) worker.join();

    std::sort(ids.begin(), ids.end());
    ASSERT_EQ(ids.size(), threadCount * eventsPerThread);
    for (size_t index = 0; index < ids.size(); ++index) EXPECT_EQ(ids[index], index + 1);
    EXPECT_EQ(journal.latestCursor(), ids.size());
    EXPECT_EQ(journal.readAfter(0, ids.size()).events.size(), ids.size());
}

} // namespace
