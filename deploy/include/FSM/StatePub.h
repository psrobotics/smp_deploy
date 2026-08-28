// New here, not upstream (which has no LCM). The Unitree copyright header this
// file was created with was a copy-paste artifact and was removed; re-confirm
// when UPSTREAM's commit gets pinned.
//
// RT-safe background LCM publisher for proprioceptive state.

#pragma once

#include "isaaclab/assets/articulation/articulation.h"
#include "smp_hw_lcm/robot_state_t.hpp"
#include <lcm/lcm-cpp.hpp>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdint>

namespace unitree::fsm {

// ── Observation field descriptors ────────────────────────────────────────────
// Describes which ArticulationData fields are packed into robot_state_t::state.
// Entries are written in order; offsets are computed once at construction.

enum class ObsSource : uint8_t {
    ProjectedGravityB,  // d.projected_gravity_b            (natural size = 3)
    RootAngVelB,        // d.root_ang_vel_b                 (natural size = 3)
    JointPosDiff,       // d.joint_pos - d.default_joint_pos (natural size = N_JOINTS)
    JointVel,           // d.joint_vel                       (natural size = N_JOINTS)
    JointPos,           // d.joint_pos (raw)                 (natural size = N_JOINTS)
};

struct ObsEntry {
    ObsSource source;
    int       count;    // number of floats to write
};

using ObsLayout = std::vector<ObsEntry>;

// Default layout: [ projected_gravity(3) | ang_vel(3) | joint_pos_diff(N) | joint_vel(N) ]
inline ObsLayout default_obs_layout(int n_joints = 29)
{
    return {
        { ObsSource::ProjectedGravityB, 3        },
        { ObsSource::RootAngVelB,       3        },
        { ObsSource::JointPosDiff,      n_joints },
        { ObsSource::JointVel,          n_joints },
    };
}

// ── StatePub ─────────────────────────────────────────────────────────────────
// RT-safe background LCM publisher for robot proprioceptive state.
//
// Policy thread calls push() after each env->step(); it uses try_lock so it
// never blocks — at worst one frame is silently dropped.
//
// Configure before calling start():
//   pub.set_channel("MY_CHANNEL");
//   pub.set_layout({ { ObsSource::JointVel, 29 }, ... });
class StatePub {
public:
    explicit StatePub(ObsLayout layout = default_obs_layout(),
                      std::string channel = "ROBOT_STATE")
        : channel_(std::move(channel))
    {
        apply_layout(std::move(layout));
    }

    // Call before start() to override the LCM channel.
    void set_channel(std::string channel) { channel_ = std::move(channel); }

    // Call before start() to change which obs fields are published.
    // Computes per-entry offsets once so push() is allocation-free.
    void set_layout(ObsLayout layout)
    {
        apply_layout(std::move(layout));
    }

    // RT hot path — called from the policy thread.
    // Uses try_lock; silently drops the frame if the pub thread is busy.
    void push(const isaaclab::ArticulationData& d)
    {
        if (!mtx_.try_lock()) return;

        msg_.utime = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg_.frame_index = frame_index_++;

        for (const auto& e : entries_) {
            float* dst = msg_.state.data() + e.offset;
            switch (e.source) {
                case ObsSource::ProjectedGravityB:
                    for (int i = 0; i < e.count; i++) dst[i] = d.projected_gravity_b[i];
                    break;
                case ObsSource::RootAngVelB:
                    for (int i = 0; i < e.count; i++) dst[i] = d.root_ang_vel_b[i];
                    break;
                case ObsSource::JointPosDiff: {
                    const int nj = std::min(e.count, (int)d.joint_pos.size());
                    for (int i = 0; i < nj; i++) dst[i] = d.joint_pos[i] - d.default_joint_pos[i];
                    break;
                }
                case ObsSource::JointVel: {
                    const int nj = std::min(e.count, (int)d.joint_vel.size());
                    for (int i = 0; i < nj; i++) dst[i] = d.joint_vel[i];
                    break;
                }
                case ObsSource::JointPos: {
                    const int nj = std::min(e.count, (int)d.joint_pos.size());
                    for (int i = 0; i < nj; i++) dst[i] = d.joint_pos[i];
                    break;
                }
            }
        }

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
                auto m = msg_;          // copy while holding lock (~state_dim_ * 4 B)
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
    struct Entry {
        ObsSource source;
        int       count;
        int       offset;   // pre-computed index into msg_.state
    };

    void apply_layout(ObsLayout layout)
    {
        entries_.clear();
        int offset = 0;
        for (const auto& e : layout) {
            entries_.push_back({ e.source, e.count, offset });
            offset += e.count;
        }
        state_dim_ = offset;
        msg_.state.assign(state_dim_, 0.0f);
        msg_.state_dim = state_dim_;
    }

    std::string   channel_;
    int           state_dim_{0};
    int32_t       frame_index_{0};

    std::vector<Entry>              entries_;
    lcm::LCM                        lcm_;
    smp_hw_lcm::robot_state_t      msg_;
    std::mutex                      mtx_;
    std::condition_variable         cv_;
    bool                            fresh_{false};
    bool                            running_{false};
    std::thread                     thread_;
};

} // namespace unitree::fsm
