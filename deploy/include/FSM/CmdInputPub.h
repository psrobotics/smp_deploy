// RT-safe LCM publisher for cmd_t from joystick state.
//
// Same pattern as StatePub: policy thread calls push() with try_lock
// (never blocks), background thread publishes.  Reuses the joystick
// values already available in ArticulationData so the value_eval_lcm
// node receives CMD_INPUT without a separate joystick LCM node.

#pragma once

#include "smp_hw_lcm/cmd_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <string>

namespace unitree::fsm {

class CmdInputPub {
public:
    explicit CmdInputPub(std::string channel = "CMD_INPUT")
        : channel_(std::move(channel)) {}

    void set_channel(std::string channel) { channel_ = std::move(channel); }

    // RT hot path — never blocks (try_lock).
    void push(float vx, float vy, float wz)
    {
        if (!mtx_.try_lock()) return;
        msg_.utime = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg_.vx = vx;
        msg_.vy = vy;
        msg_.wz = wz;
        fresh_ = true;
        mtx_.unlock();
        cv_.notify_one();
    }

    void start()
    {
        running_ = true;
        thread_ = std::thread([this] {
            std::unique_lock<std::mutex> lk(mtx_);
            while (running_) {
                cv_.wait(lk, [this] { return fresh_ || !running_; });
                if (!running_) break;
                fresh_ = false;
                auto m = msg_;
                lk.unlock();
                lcm_.publish(channel_.c_str(), &m);
                lk.lock();
            }
        });
    }

    void stop()
    {
        { std::lock_guard<std::mutex> lk(mtx_); running_ = false; }
        cv_.notify_one();
        if (thread_.joinable()) thread_.join();
    }

private:
    std::string                channel_;
    lcm::LCM                  lcm_;
    smp_hw_lcm::cmd_t        msg_{};
    std::mutex                 mtx_;
    std::condition_variable    cv_;
    bool                       fresh_{false};
    bool                       running_{false};
    std::thread                thread_;
};

} // namespace unitree::fsm
