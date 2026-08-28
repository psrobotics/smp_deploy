#include "FSM/State_RLBase.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include <unordered_map>

namespace isaaclab
{
// keyboard velocity commands example
// change "velocity_commands" observation name in policy deploy.yaml to "keyboard_velocity_commands"
REGISTER_OBSERVATION(keyboard_velocity_commands)
{
    std::string key = FSMState::keyboard->key();
    static auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    static std::unordered_map<std::string, std::vector<float>> key_commands = {
        {"w", {1.0f, 0.0f, 0.0f}},
        {"s", {-1.0f, 0.0f, 0.0f}},
        {"a", {0.0f, 1.0f, 0.0f}},
        {"d", {0.0f, -1.0f, 0.0f}},
        {"q", {0.0f, 0.0f, 1.0f}},
        {"e", {0.0f, 0.0f, -1.0f}}
    };
    std::vector<float> cmd = {0.0f, 0.0f, 0.0f};
    if (key_commands.find(key) != key_commands.end())
    {
        cmd = key_commands[key];
    }
    return cmd;
}


REGISTER_OBSERVATION(gait_phase_my)
{
    float period = params["period"].as<float>();
    float delta_phase = env->step_dt * (1.0f / period);

    env->global_phase += delta_phase;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    auto cmd = isaaclab::mdp::velocity_commands(env, params);
    float cmd_norm = std::sqrt(
        cmd[0] * cmd[0] +
        cmd[1] * cmd[1] +
        cmd[2] * cmd[2]
    );

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2 * M_PI);
    obs[1] = std::cos(env->global_phase * 2 * M_PI);

    if (cmd_norm < 0.1f)
    {
        obs[0] = 0.0f;
        obs[1] = 0.0f;
    }

    return obs;
}


}

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    if (cfg["lcm_channel_robot_state"])
        _state_pub.set_channel(cfg["lcm_channel_robot_state"].as<std::string>());
    if (cfg["lcm_channel_cmd_input"])
        _cmd_input_pub.set_channel(cfg["lcm_channel_cmd_input"].as<std::string>());
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    // Per-state policy files (config-driven). Defaults preserve previous behavior.
    const std::string env_cfg_file = cfg["env_cfg_file"]
        ? cfg["env_cfg_file"].as<std::string>() : "deploy_mjlab.yaml";
    const std::string policy_file = cfg["policy_file"]
        ? cfg["policy_file"].as<std::string>()
        : "2026-06-01_23-03-16_smp_joystick_deploy_g1.onnx";
    spdlog::info("State_{}: env_cfg='{}', policy='{}'", state_string, env_cfg_file, policy_file);
    

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / env_cfg_file),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    // Any ONNX graph with an `obs` input and an `actions` output drops in here.
    // What it expects is described by env_cfg_file, not by code.
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / policy_file);


    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    static thread_local std::vector<float> action;
    static thread_local uint32_t sample_counter = 0;
    static thread_local bool dashboard_enabled = unitree::rt::Dashboard::instance().enabled();
    env->action_manager->copy_processed_actions(action);
    for(int i(0); i < env->robot->data.joint_ids_map.size() && i < action.size(); i++) {
        lowcmd->msg_.motor_cmd()[env->robot->data.joint_ids_map[i]].q() = action[i];
    }

    if (dashboard_enabled && (++sample_counter % 32u == 0u)) {
        float action_l2 = 0.0f;
        for (float a : action) action_l2 += a * a;
        unitree::rt::Dashboard::instance().set_action_l2(std::sqrt(action_l2));
    }
}
