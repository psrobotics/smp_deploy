#include "rt_telemetry.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>

int main(int argc, char** argv)
{
    int cycles = 3000;
    int dt_us = 1000;
    int work_us = 120;

    if (argc > 1) cycles = std::max(100, std::atoi(argv[1]));
    if (argc > 2) dt_us = std::max(100, std::atoi(argv[2]));
    if (argc > 3) work_us = std::max(0, std::atoi(argv[3]));

    using clock = std::chrono::steady_clock;
    using ns = std::chrono::nanoseconds;
    const int64_t target_period_ns = static_cast<int64_t>(dt_us) * 1000;

    // 1) Benchmark telemetry update overhead itself.
    {
        constexpr int kN = 2'000'000;
        unitree::rt::LoopStats stats("BenchOverhead", target_period_ns, true, static_cast<uint32_t>(kN + 1), true);

        auto t0 = clock::now();
        for (int i = 0; i < kN; ++i) {
            const int64_t exec_ns = 70'000 + (i % 17);
            const int64_t wake_lag_ns = (i % 7) - 3;
            stats.record(exec_ns, wake_lag_ns);
        }
        auto t1 = clock::now();

        const double elapsed_ns = static_cast<double>(std::chrono::duration_cast<ns>(t1 - t0).count());
        const double ns_per_record = elapsed_ns / static_cast<double>(kN);
        std::cout << "[rt_bench] telemetry_overhead_ns_per_record=" << ns_per_record << "\n";
    }

    // 2) Benchmark scheduler behavior for a fixed-rate loop.
    unitree::rt::LoopStats loop_stats("BenchLoop", target_period_ns, true, static_cast<uint32_t>(cycles + 1), true);

    auto loop_start = clock::now();
    auto sleep_till = loop_start + std::chrono::microseconds(dt_us);
    int64_t wake_lag_ns = 0;

    for (int i = 0; i < cycles; ++i) {
        const auto step_start = clock::now();

        // Simulated deterministic work.
        const auto work_deadline = step_start + std::chrono::microseconds(work_us);
        while (clock::now() < work_deadline) {
        }

        const auto step_end = clock::now();
        const int64_t exec_ns = std::chrono::duration_cast<ns>(step_end - step_start).count();
        loop_stats.record(exec_ns, wake_lag_ns);

        std::this_thread::sleep_until(sleep_till);
        const auto wake = clock::now();
        wake_lag_ns = std::chrono::duration_cast<ns>(wake - sleep_till).count();
        sleep_till += std::chrono::microseconds(dt_us);
    }

    auto loop_end = clock::now();
    const double elapsed_s = std::chrono::duration<double>(loop_end - loop_start).count();
    const double effective_hz = static_cast<double>(cycles) / elapsed_s;
    const double target_hz = 1e6 / static_cast<double>(dt_us);

    std::cout << "[rt_bench] " << loop_stats.summary() << "\n";
    std::cout << "[rt_bench] effective_hz=" << effective_hz << " target_hz=" << target_hz
              << " cycles=" << cycles << " dt_us=" << dt_us << " work_us=" << work_us << "\n";

    return 0;
}
