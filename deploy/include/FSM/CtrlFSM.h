// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <unitree/common/thread/recurrent_thread.hpp>
#include "BaseState.h"
#include "rt_dashboard.h"
#include "rt_telemetry.h"
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <chrono>

class CtrlFSM
{
public:
    CtrlFSM(std::shared_ptr<BaseState> initstate)
    {
        // Initialize FSM states
        states.push_back(std::move(initstate));

    }

    CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"]; // enabled FSMs

        // register FSM string map; used for state transition
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        // Initialize FSM states
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            std::string fsm_name = it->first.as<std::string>();
            int id = it->second["id"].as<int>();
            std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end()) {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            auto state_instance = fsm_class->second(id, fsm_name);
            add(state_instance);
        }
    }

    void start() 
    {
        // Start From State_Passive
        currentState = states[0];
        currentState->enter();
        auto& dashboard = unitree::rt::Dashboard::instance();
        dashboard.start_if_enabled();
        dashboard_enabled_ = dashboard.enabled();
        dashboard.set_fsm_target_hz(1.0 / dt);
        dashboard.set_fsm_state(currentState->getStateString());

        // Optional env overrides:
        //   UNITREE_FSM_CPU_ID: pin control thread to a CPU core (default: -1, no pinning)
        //   UNITREE_FSM_RT_PRIO: SCHED_FIFO priority for control thread (default: 88, <=0 disables RT priority)
        int32_t cpu_id = UT_CPU_ID_NONE;
        int32_t rt_prio = 88;
        bool rt_prio_enabled = true;
        if (const char* env_cpu = std::getenv("UNITREE_FSM_CPU_ID")) {
            cpu_id = std::atoi(env_cpu);
        }
        if (const char* env_prio = std::getenv("UNITREE_FSM_RT_PRIO")) {
            rt_prio = std::atoi(env_prio);
        }
        if (rt_prio <= 0) {
            rt_prio_enabled = false;
        } else {
            if (rt_prio < 1) rt_prio = 1;
            if (rt_prio > 99) rt_prio = 99;
        }

        const bool fsm_rt_telemetry =
            unitree::rt::env_flag("UNITREE_FSM_RT_TELEMETRY", false) ||
            unitree::rt::env_flag("UNITREE_RT_TELEMETRY", false);
        const uint32_t rt_log_every = unitree::rt::env_u32("UNITREE_RT_LOG_EVERY", 1000);
        fsm_stats_ = std::make_unique<unitree::rt::LoopStats>(
            "FSM", static_cast<int64_t>(dt * 1e9), fsm_rt_telemetry, rt_log_every, false);

        fsm_thread_ = std::make_shared<unitree::common::RecurrentThread>(
            "FSM", cpu_id, this->dt * 1e6, &CtrlFSM::run_, this);
        if (rt_prio_enabled) {
            try {
                fsm_thread_->SetPriority(rt_prio);
                spdlog::info("FSM: RT settings cpu_id={}, rt_prio={}", cpu_id, rt_prio);
            } catch (const std::exception& e) {
                spdlog::warn(
                    "FSM: failed to set SCHED_FIFO priority {} ({}). "
                    "Continuing with default scheduler policy.",
                    rt_prio, e.what());
            }
        } else {
            spdlog::info("FSM: RT priority disabled (UNITREE_FSM_RT_PRIO<=0), cpu_id={}", cpu_id);
        }
        if (fsm_rt_telemetry) {
            spdlog::info("FSM: RT telemetry enabled, log_every={}", rt_log_every);
        }
        spdlog::info("FSM: Start {}", currentState->getStateString());
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for(auto & s : states)
        {
            if(s->isState(state->getState()))
            {
                spdlog::error("FSM: State_{} already exists", state->getStateString());
                std::exit(0);
            }
        }

        states.push_back(std::move(state));
    }
    
    ~CtrlFSM()
    {
        unitree::rt::Dashboard::instance().stop();
        states.clear();
    }

    std::vector<std::shared_ptr<BaseState>> states;
private:
    const double dt = 0.001;

    void run_()
    {
        const auto start = std::chrono::steady_clock::now();

        currentState->pre_run();
        currentState->run();
        currentState->post_run();
        
        // Check if need to change state
        int nextStateMode = 0;
        for(int i(0); i<currentState->registered_checks.size(); i++)
        {
            if(currentState->registered_checks[i].first())
            {
                nextStateMode = currentState->registered_checks[i].second;
                break;
            }
        }

        if(nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for(auto & state : states)
            {
                if(state->isState(nextStateMode))
                {
                    spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    if (dashboard_enabled_) {
                        unitree::rt::Dashboard::instance().set_fsm_state(currentState->getStateString());
                    }
                    break;
                }
            }
        }

        if (fsm_stats_) {
            const auto end = std::chrono::steady_clock::now();
            const int64_t exec_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
            fsm_stats_->record(exec_ns, 0);
            if (dashboard_enabled_) {
                unitree::rt::Dashboard::instance().on_fsm_tick(exec_ns);
            }
            if (fsm_stats_->should_log()) {
                spdlog::info("{}", fsm_stats_->summary());
            }
        }
    }

    std::shared_ptr<BaseState> currentState;
    unitree::common::RecurrentThreadPtr fsm_thread_;
    std::unique_ptr<unitree::rt::LoopStats> fsm_stats_;
    bool dashboard_enabled_{false};
};
