/**
 * @file test_cyclic_task_scheduler.cpp
 * @brief Coverage for CyclicTaskScheduler phase/priority ordering and
 *        lifecycle (src/ethercat/CyclicTaskScheduler.cpp).
 */

#include <gtest/gtest.h>

#include <vector>

#include "tether/ethercat/CyclicTaskScheduler.hpp"

using namespace EtherCAT;

namespace {

struct FakeTask : ICyclicTask {
    std::vector<int>* order = nullptr;
    int id = 0;
    bool fail = false;
    FakeTask(std::vector<int>* o, int i, bool f) : order(o), id(i), fail(f) {}
    bool update(DS402Master&, double) override {
        order->push_back(id);
        return !fail;
    }
};

TEST(CyclicTaskSchedulerTest, EmptyExecuteSucceeds) {
    CyclicTaskScheduler s;
    EXPECT_TRUE(s.executeAll(*reinterpret_cast<DS402Master*>(0), 0.001));
    EXPECT_EQ(s.taskCount(), 0u);
}

TEST(CyclicTaskSchedulerTest, ExecutesInPhaseThenPriorityOrder) {
    CyclicTaskScheduler s;
    std::vector<int> order;
    FakeTask a{&order, 1, false}, b{&order, 2, false},
            c{&order, 3, false}, d{&order, 4, false};
    // Insert out of order: MotionControl(30) first, then PreExchange(0).
    s.addTask(&a, TaskPhase::MotionControl, 0);
    s.addTask(&b, TaskPhase::PreExchange, 200);
    s.addTask(&c, TaskPhase::PreExchange, 10);
    s.addTask(&d, TaskPhase::Diagnostics, 0);
    EXPECT_EQ(s.taskCount(), 4u);
    EXPECT_TRUE(s.executeAll(*reinterpret_cast<DS402Master*>(0), 0.001));
    ASSERT_EQ(order.size(), 4u);
    EXPECT_EQ(order[0], 3);  // PreExchange prio 10
    EXPECT_EQ(order[1], 2);  // PreExchange prio 200
    EXPECT_EQ(order[2], 1);  // MotionControl
    EXPECT_EQ(order[3], 4);  // Diagnostics
}

TEST(CyclicTaskSchedulerTest, RemoveTaskStopsExecution) {
    CyclicTaskScheduler s;
    std::vector<int> order;
    FakeTask a{&order, 1, false}, b{&order, 2, false};
    auto ha = s.addTask(&a, TaskPhase::Exchange, 0);
    s.addTask(&b, TaskPhase::Exchange, 0);
    EXPECT_TRUE(s.removeTask(ha));
    EXPECT_EQ(s.taskCount(), 1u);
    EXPECT_TRUE(s.executeAll(*reinterpret_cast<DS402Master*>(0), 0.001));
    ASSERT_EQ(order.size(), 1u);
    EXPECT_EQ(order[0], 2);
}

TEST(CyclicTaskSchedulerTest, RemoveInvalidOrUnknownReturnsFalse) {
    CyclicTaskScheduler s;
    EXPECT_FALSE(s.removeTask(TaskHandle{0}));
    EXPECT_FALSE(s.removeTask(TaskHandle{999}));
}

TEST(CyclicTaskSchedulerTest, ClearEmptiesSchedule) {
    CyclicTaskScheduler s;
    std::vector<int> order;
    FakeTask a{&order, 1, false};
    s.addTask(&a, TaskPhase::PreExchange, 0);
    s.clear();
    EXPECT_EQ(s.taskCount(), 0u);
    EXPECT_TRUE(s.executeAll(*reinterpret_cast<DS402Master*>(0), 0.001));
    EXPECT_TRUE(order.empty());
}

TEST(CyclicTaskSchedulerTest, FailingTaskAbortsSequence) {
    CyclicTaskScheduler s;
    std::vector<int> order;
    FakeTask bad{&order, 1, true}, after{&order, 2, false};
    s.addTask(&bad, TaskPhase::PreExchange, 0);
    s.addTask(&after, TaskPhase::PreExchange, 1);
    EXPECT_FALSE(s.executeAll(*reinterpret_cast<DS402Master*>(0), 0.001));
    EXPECT_EQ(order.size(), 1u);  // second task never ran
}

TEST(CyclicTaskSchedulerTest, TaskHandleValidity) {
    TaskHandle h;
    EXPECT_FALSE(h.valid());
    CyclicTaskScheduler s;
    FakeTask a{nullptr, 0, false};
    h = s.addTask(&a, TaskPhase::PreExchange, 0);
    EXPECT_TRUE(h.valid());
}

} // namespace
