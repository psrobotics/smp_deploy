// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "FSM/StatePub.h"
#include "FSM/CmdInputPub.h"
#include "rt_dashboard.h"
#include "rt_telemetry.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"
#include <atomic>
#include <chrono>

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);
    
    void enter()
    {
        // set gain
        for (int i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
        {
            lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
            lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
            lowcmd->msg_.motor_cmd()[i].dq() = 0;
            lowcmd->msg_.motor_cmd()[i].tau() = 0;
        }

        env->robot->update();
        _state_pub.start();
        _cmd_input_pub.start();
        // Start policy thread
        policy_thread_running.store(true, std::memory_order_release);
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);
            const auto dt_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count();

            const bool policy_rt_telemetry =
                unitree::rt::env_flag("UNITREE_POLICY_RT_TELEMETRY", false) ||
                unitree::rt::env_flag("UNITREE_RT_TELEMETRY", false);
            const uint32_t rt_log_every = unitree::rt::env_u32("UNITREE_RT_LOG_EVERY", 500);
            unitree::rt::LoopStats policy_stats("Policy", dt_ns, policy_rt_telemetry, rt_log_every, true);
            auto& dashboard = unitree::rt::Dashboard::instance();
            const bool dashboard_enabled = dashboard.enabled();
            if (dashboard_enabled) {
                dashboard.set_policy_target_hz(1.0 / env->step_dt);
            }
            if (policy_rt_telemetry) {
                spdlog::info("Policy: RT telemetry enabled, log_every={}, step_dt={}s", rt_log_every, env->step_dt);
            }

            // Initialize timing
            auto sleepTill = clock::now() + dt;
            env->reset();
            int64_t wake_lag_ns = 0;

            while (policy_thread_running.load(std::memory_order_acquire))
            {
                const auto step_start = clock::now();
                env->step();
                _state_pub.push(env->robot->data);
                // Publish the pad as CMD_INPUT, scaled the same way
                // velocity_commands scales it, so a subscriber sees velocity
                // targets rather than raw stick deflection.
                {
                    const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];
                    const float lo_vx = cfg["lin_vel_x"][0].as<float>(), hi_vx = cfg["lin_vel_x"][1].as<float>();
                    const float lo_vy = cfg["lin_vel_y"][0].as<float>(), hi_vy = cfg["lin_vel_y"][1].as<float>();
                    const float lo_wz = cfg["ang_vel_z"][0].as<float>(), hi_wz = cfg["ang_vel_z"][1].as<float>();
                    auto scale_joy = [](float joy, float lo, float hi) -> float {
                        return (joy >= 0.0f) ? joy * hi : -joy * lo;
                    };
                    auto& joy = env->robot->data.joystick;
                    _cmd_input_pub.push(
                        std::clamp(scale_joy( joy->ly(), lo_vx, hi_vx), lo_vx, hi_vx),
                        std::clamp(scale_joy(-joy->lx(), lo_vy, hi_vy), lo_vy, hi_vy),
                        std::clamp(scale_joy(-joy->rx(), lo_wz, hi_wz), lo_wz, hi_wz));
                }
                const auto step_end = clock::now();

                const int64_t exec_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(step_end - step_start).count();
                policy_stats.record(exec_ns, wake_lag_ns);
                if (dashboard_enabled) {
                    dashboard.on_policy_tick(exec_ns, wake_lag_ns);
                }
                if (policy_stats.should_log()) {
                    spdlog::info("{}", policy_stats.summary());
                }

                // Sleep
                std::this_thread::sleep_until(sleepTill);
                const auto wake = clock::now();
                wake_lag_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(wake - sleepTill).count();
                sleepTill += dt;
            }
        });
    }

    void run();
    
    void exit()
    {
        policy_thread_running.store(false, std::memory_order_release);
        if (policy_thread.joinable()) {
            policy_thread.join();
        }
        _state_pub.stop();
        _cmd_input_pub.stop();
    }

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    std::thread policy_thread;
    std::atomic<bool> policy_thread_running{false};

    unitree::fsm::StatePub                    _state_pub;
    unitree::fsm::CmdInputPub                 _cmd_input_pub;
};

REGISTER_FSM(State_RLBase)
