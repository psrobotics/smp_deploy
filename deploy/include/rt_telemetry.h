#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <string>

namespace unitree::rt
{
inline bool env_flag(const char* key, bool default_value = false)
{
    if (const char* v = std::getenv(key)) {
        return std::atoi(v) != 0;
    }
    return default_value;
}

inline uint32_t env_u32(const char* key, uint32_t default_value)
{
    if (const char* v = std::getenv(key)) {
        int val = std::atoi(v);
        return val > 0 ? static_cast<uint32_t>(val) : default_value;
    }
    return default_value;
}

class LoopStats
{
public:
    LoopStats(std::string name, int64_t target_period_ns, bool enabled, uint32_t log_every, bool track_wake_lag)
        : name_(std::move(name)),
          target_period_ns_(target_period_ns),
          enabled_(enabled),
          log_every_(std::max<uint32_t>(1, log_every)),
          track_wake_lag_(track_wake_lag)
    {}

    inline void record(int64_t exec_ns, int64_t wake_lag_ns)
    {
        if (!enabled_) return;

        ++count_;
        sum_exec_ns_ += static_cast<double>(exec_ns);
        min_exec_ns_ = std::min(min_exec_ns_, exec_ns);
        max_exec_ns_ = std::max(max_exec_ns_, exec_ns);

        if (exec_ns > target_period_ns_) {
            ++exec_overrun_count_;
        }

        if (track_wake_lag_) {
            sum_wake_lag_ns_ += static_cast<double>(wake_lag_ns);
            min_wake_lag_ns_ = std::min(min_wake_lag_ns_, wake_lag_ns);
            max_wake_lag_ns_ = std::max(max_wake_lag_ns_, wake_lag_ns);
            if (wake_lag_ns > 0) {
                ++wake_miss_count_;
            }
        }
    }

    inline bool should_log() const
    {
        return enabled_ && (count_ % log_every_ == 0);
    }

    inline std::string summary() const
    {
        if (!enabled_ || count_ == 0) {
            return name_ + ": telemetry disabled";
        }

        const double n = static_cast<double>(count_);
        const double avg_exec_us = (sum_exec_ns_ / n) / 1000.0;
        const double min_exec_us = static_cast<double>(min_exec_ns_) / 1000.0;
        const double max_exec_us = static_cast<double>(max_exec_ns_) / 1000.0;
        const double exec_overrun_pct = (100.0 * static_cast<double>(exec_overrun_count_)) / n;

        std::string out =
            name_ + " rt: n=" + std::to_string(count_) +
            " exec_us(avg/min/max)=" + std::to_string(avg_exec_us) + "/" + std::to_string(min_exec_us) + "/" + std::to_string(max_exec_us) +
            " exec_overrun=" + std::to_string(exec_overrun_count_) + "(" + std::to_string(exec_overrun_pct) + "%)";

        if (track_wake_lag_) {
            const double avg_wake_us = (sum_wake_lag_ns_ / n) / 1000.0;
            const double min_wake_us = static_cast<double>(min_wake_lag_ns_) / 1000.0;
            const double max_wake_us = static_cast<double>(max_wake_lag_ns_) / 1000.0;
            const double wake_miss_pct = (100.0 * static_cast<double>(wake_miss_count_)) / n;
            out +=
                " wake_lag_us(avg/min/max)=" + std::to_string(avg_wake_us) + "/" + std::to_string(min_wake_us) + "/" + std::to_string(max_wake_us) +
                " wake_miss=" + std::to_string(wake_miss_count_) + "(" + std::to_string(wake_miss_pct) + "%)";
        }
        return out;
    }

private:
    std::string name_;
    int64_t target_period_ns_;
    bool enabled_{false};
    uint32_t log_every_{1000};
    bool track_wake_lag_{false};

    uint64_t count_{0};

    double sum_exec_ns_{0.0};
    int64_t min_exec_ns_{std::numeric_limits<int64_t>::max()};
    int64_t max_exec_ns_{std::numeric_limits<int64_t>::min()};
    uint64_t exec_overrun_count_{0};

    double sum_wake_lag_ns_{0.0};
    int64_t min_wake_lag_ns_{std::numeric_limits<int64_t>::max()};
    int64_t max_wake_lag_ns_{std::numeric_limits<int64_t>::min()};
    uint64_t wake_miss_count_{0};
};
} // namespace unitree::rt
