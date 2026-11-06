#include "tether/modbus/client/Poller.hpp"

namespace tether::modbus {

PollItem& Poller::add(PollItem it) {
    std::lock_guard g(m_);
    it.nextRun = std::chrono::steady_clock::now();
    items_.push_back(std::move(it));
    cv_.notify_all();
    return items_.back();
}

void Poller::remove(const std::string& name) {
    std::lock_guard g(m_);
    std::erase_if(items_, [&](const PollItem& i) {
        return i.name == name;
    });
}

Expected<void> Poller::start() {
    if (running_) return {};
    stop_ = false;
    running_ = true;
    th_ = std::thread([this] { loop(); });
    return {};
}

void Poller::stop() {
    {
        std::lock_guard g(m_);
        stop_ = true;
    }
    cv_.notify_all();
    if (th_.joinable()) th_.join();
    running_ = false;
}

void Poller::kick(const std::string& name) {
    std::lock_guard g(m_);
    for (auto& i : items_)
        if (name.empty() || i.name == name)
            i.nextRun = std::chrono::steady_clock::now();
    cv_.notify_all();
}

void Poller::runOnce() {
    std::vector<PollItem*> due;
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard g(m_);
        for (auto& i : items_)
            if (i.enabled && i.nextRun <= now) due.push_back(&i);
        std::sort(due.begin(), due.end(), [](auto* a, auto* b) {
            return a->priority > b->priority;
        });
    }
    for (auto* it : due) {
        std::vector<uint16_t> vals;
        const ModbusError* err = nullptr;
        ModbusError e;
        switch (it->fc) {
            case FunctionCode::ReadCoils:
            case FunctionCode::ReadDiscreteInputs: {
                auto r = it->fc == FunctionCode::ReadCoils
                    ? client_.readCoils(it->addr, it->qty)
                    : client_.readDiscreteInputs(it->addr, it->qty);
                if (!r) { e = r.error(); err = &e; }
                else for (bool b : *r) vals.push_back(b ? 1 : 0);
                break;
            }
            default: {
                auto r = it->fc == FunctionCode::ReadInputRegisters
                    ? client_.readInput(it->addr, it->qty)
                    : client_.readHolding(it->addr, it->qty);
                if (!r) { e = r.error(); err = &e; }
                else vals = std::move(*r);
                break;
            }
        }
        stats.polls++;
        if (err) stats.errors++;
        bool differs;
        if (!it->haveLast) differs = true;
        else if (it->epsilon > 0) {
            differs = vals.size() != it->last.size();
            if (!differs)
                for (size_t i = 0; i < vals.size(); ++i)
                    if (std::abs(int(vals[i]) - int(it->last[i])) >
                        it->epsilon) { differs = true; break; }
        } else {
            differs = it->last != vals;
        }
        // `last` is the last *notified* value: streak grows while the
        // polled value keeps differing from it (debounce), resets when
        // the callback fires and `last` catches up.
        if (err) it->changeStreak = 0;
        else if (differs) ++it->changeStreak;
        else it->changeStreak = 0;
        // First sample is a baseline, not a change — never debounce it.
        const bool fire = !it->changedOnly || err || !it->haveLast ||
                          it->changeStreak > it->debounceN;
        {
            std::lock_guard g(m_);
            it->nextRun = now + std::chrono::milliseconds(
                std::max(1, it->intervalMs));
            if (fire && !err) {
                it->last = vals; it->haveLast = true;
                it->changeStreak = 0;
            }
        }
        if (it->onResult && fire) {
            if (differs && !err) stats.changes++;
            it->onResult(*it, vals, err);
        }
    }
}

void Poller::loop() {
    while (true) {
        {
            std::lock_guard g(m_);
            if (stop_) return;
        }
        runOnce();
        std::unique_lock lk(m_);
        // sleep till next due item (or 10ms poll granularity)
        auto next = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(10);
        for (const auto& i : items_)
            if (i.enabled && i.nextRun < next) next = i.nextRun;
        cv_.wait_until(lk, next, [this] { return stop_; });
    }
}

} // namespace tether::modbus
