#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>

namespace unitree::rt
{
class Dashboard
{
public:
    static Dashboard& instance()
    {
        static Dashboard d;
        return d;
    }

    void start_if_enabled()
    {
        const bool want_dashboard = env_flag("UNITREE_RT_DASHBOARD", false);
        if (!want_dashboard || started_.exchange(true, std::memory_order_acq_rel)) {
            return;
        }

        if (!::isatty(STDOUT_FILENO)) {
            enabled_.store(false, std::memory_order_release);
            return;
        }

        enabled_.store(true, std::memory_order_release);
        refresh_ms_ = env_u32("UNITREE_RT_DASHBOARD_MS", 250);
        running_.store(true, std::memory_order_release);
        render_thread_ = std::thread([this] { render_loop(); });
    }

    void stop()
    {
        if (!started_.load(std::memory_order_acquire)) return;
        running_.store(false, std::memory_order_release);
        if (render_thread_.joinable()) render_thread_.join();
        enabled_.store(false, std::memory_order_release);
    }

    bool enabled() const
    {
        return enabled_.load(std::memory_order_relaxed);
    }

    void set_fsm_state(const std::string& state)
    {
        if (!enabled()) return;
        std::lock_guard<std::mutex> lock(state_mtx_);
        fsm_state_ = state;
    }

    void set_fsm_target_hz(double hz)
    {
        if (!enabled()) return;
        fsm_target_hz_.store(hz, std::memory_order_relaxed);
    }

    void set_policy_target_hz(double hz)
    {
        if (!enabled()) return;
        policy_target_hz_.store(hz, std::memory_order_relaxed);
    }

    void on_fsm_tick(int64_t exec_ns)
    {
        if (!enabled()) return;
        fsm_count_.fetch_add(1, std::memory_order_relaxed);
        fsm_exec_sum_ns_.fetch_add(exec_ns, std::memory_order_relaxed);
        fsm_exec_latest_ns_.store(exec_ns, std::memory_order_relaxed);
        fetch_max(fsm_exec_max_ns_, exec_ns);
    }

    void on_policy_tick(int64_t exec_ns, int64_t wake_lag_ns)
    {
        if (!enabled()) return;
        policy_count_.fetch_add(1, std::memory_order_relaxed);
        policy_exec_sum_ns_.fetch_add(exec_ns, std::memory_order_relaxed);
        policy_wake_sum_ns_.fetch_add(wake_lag_ns, std::memory_order_relaxed);
        policy_exec_latest_ns_.store(exec_ns, std::memory_order_relaxed);
        policy_wake_latest_ns_.store(wake_lag_ns, std::memory_order_relaxed);
        fetch_max(policy_exec_max_ns_, exec_ns);
        fetch_max(policy_wake_max_ns_, wake_lag_ns);
    }

    void set_obs_dim(uint32_t obs_dim)
    {
        if (!enabled()) return;
        obs_dim_.store(obs_dim, std::memory_order_relaxed);
    }

    void set_action_dim(uint32_t action_dim)
    {
        if (!enabled()) return;
        action_dim_.store(action_dim, std::memory_order_relaxed);
    }

    void set_action_l2(float action_l2)
    {
        if (!enabled()) return;
        action_l2_.store(action_l2, std::memory_order_relaxed);
    }

private:
    Dashboard() = default;
    ~Dashboard()
    {
        stop();
    }

    static bool env_flag(const char* key, bool default_value)
    {
        if (const char* v = std::getenv(key)) return std::atoi(v) != 0;
        return default_value;
    }

    static uint32_t env_u32(const char* key, uint32_t default_value)
    {
        if (const char* v = std::getenv(key)) {
            int val = std::atoi(v);
            return val > 0 ? static_cast<uint32_t>(val) : default_value;
        }
        return default_value;
    }

    static void fetch_max(std::atomic<int64_t>& target, int64_t value)
    {
        int64_t prev = target.load(std::memory_order_relaxed);
        while (value > prev && !target.compare_exchange_weak(prev, value, std::memory_order_relaxed)) {
        }
    }

    static double ns_to_us(int64_t ns)
    {
        return static_cast<double>(ns) / 1000.0;
    }

    void render_loop()
    {
        auto last_fsm_count = uint64_t{0};
        auto last_policy_count = uint64_t{0};
        auto last_ts = std::chrono::steady_clock::now();

        while (running_.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(refresh_ms_));
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last_ts).count();
            if (dt <= 0.0) continue;

            const uint64_t fsm_count = fsm_count_.load(std::memory_order_relaxed);
            const uint64_t policy_count = policy_count_.load(std::memory_order_relaxed);
            const double fsm_hz = static_cast<double>(fsm_count - last_fsm_count) / dt;
            const double policy_hz = static_cast<double>(policy_count - last_policy_count) / dt;
            last_fsm_count = fsm_count;
            last_policy_count = policy_count;
            last_ts = now;

            const int64_t fsm_exec_latest = fsm_exec_latest_ns_.load(std::memory_order_relaxed);
            const int64_t fsm_exec_max = fsm_exec_max_ns_.load(std::memory_order_relaxed);
            const int64_t fsm_exec_sum = fsm_exec_sum_ns_.load(std::memory_order_relaxed);
            const double fsm_exec_avg = (fsm_count > 0) ? ns_to_us(fsm_exec_sum / static_cast<int64_t>(fsm_count)) : 0.0;

            const int64_t policy_exec_latest = policy_exec_latest_ns_.load(std::memory_order_relaxed);
            const int64_t policy_exec_max = policy_exec_max_ns_.load(std::memory_order_relaxed);
            const int64_t policy_exec_sum = policy_exec_sum_ns_.load(std::memory_order_relaxed);
            const double policy_exec_avg = (policy_count > 0) ? ns_to_us(policy_exec_sum / static_cast<int64_t>(policy_count)) : 0.0;

            const int64_t policy_wake_latest = policy_wake_latest_ns_.load(std::memory_order_relaxed);
            const int64_t policy_wake_max = policy_wake_max_ns_.load(std::memory_order_relaxed);
            const int64_t policy_wake_sum = policy_wake_sum_ns_.load(std::memory_order_relaxed);
            const double policy_wake_avg = (policy_count > 0) ? ns_to_us(policy_wake_sum / static_cast<int64_t>(policy_count)) : 0.0;

            std::string fsm_state;
            {
                std::lock_guard<std::mutex> lock(state_mtx_);
                fsm_state = fsm_state_;
            }

            const double fsm_target_hz = fsm_target_hz_.load(std::memory_order_relaxed);
            const double policy_target_hz = policy_target_hz_.load(std::memory_order_relaxed);
            const uint32_t obs_dim = obs_dim_.load(std::memory_order_relaxed);
            const uint32_t action_dim = action_dim_.load(std::memory_order_relaxed);
            const float action_l2 = action_l2_.load(std::memory_order_relaxed);

            std::cout << "\033[2J\033[H";
            std::cout << "================ UNITREE RT DASHBOARD ================\n";
            std::cout << "FSM\n";
            std::cout << "  state      : " << (fsm_state.empty() ? "N/A" : fsm_state) << "\n";
            std::cout << "  rate hz    : " << fsm_hz << " / target " << fsm_target_hz << "\n";
            std::cout << "  exec us    : latest " << ns_to_us(fsm_exec_latest) << ", avg " << fsm_exec_avg
                      << ", max " << ns_to_us(fsm_exec_max) << "\n";
            std::cout << "CONTROL/POLICY\n";
            std::cout << "  rate hz    : " << policy_hz << " / target " << policy_target_hz << "\n";
            std::cout << "  exec us    : latest " << ns_to_us(policy_exec_latest) << ", avg " << policy_exec_avg
                      << ", max " << ns_to_us(policy_exec_max) << "\n";
            std::cout << "LATENCY\n";
            std::cout << "  wake lag us: latest " << ns_to_us(policy_wake_latest) << ", avg " << policy_wake_avg
                      << ", max " << ns_to_us(policy_wake_max) << "\n";
            std::cout << "OBS/ACTION\n";
            std::cout << "  obs dim    : " << obs_dim << "\n";
            std::cout << "  action dim : " << action_dim << "\n";
            std::cout << "  action l2  : " << action_l2 << "\n";
            std::cout << "=======================================================\n";
            std::cout.flush();
        }
    }

    std::atomic<bool> started_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<bool> running_{false};
    uint32_t refresh_ms_{250};
    std::thread render_thread_;

    std::mutex state_mtx_;
    std::string fsm_state_;

    std::atomic<double> fsm_target_hz_{0.0};
    std::atomic<double> policy_target_hz_{0.0};

    std::atomic<uint64_t> fsm_count_{0};
    std::atomic<int64_t> fsm_exec_latest_ns_{0};
    std::atomic<int64_t> fsm_exec_max_ns_{0};
    std::atomic<int64_t> fsm_exec_sum_ns_{0};

    std::atomic<uint64_t> policy_count_{0};
    std::atomic<int64_t> policy_exec_latest_ns_{0};
    std::atomic<int64_t> policy_exec_max_ns_{0};
    std::atomic<int64_t> policy_exec_sum_ns_{0};
    std::atomic<int64_t> policy_wake_latest_ns_{0};
    std::atomic<int64_t> policy_wake_max_ns_{0};
    std::atomic<int64_t> policy_wake_sum_ns_{0};

    std::atomic<uint32_t> obs_dim_{0};
    std::atomic<uint32_t> action_dim_{0};
    std::atomic<float> action_l2_{0.0f};
};
} // namespace unitree::rt
