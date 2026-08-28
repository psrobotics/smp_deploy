# deploy/ — the G1 controller

A carve-out of [`unitree_rl_mjlab`](https://github.com/unitreerobotics/unitree_rl_mjlab)'s
`deploy/` tree, kept at the same paths so the diff against upstream stays one
page. What changed is enumerated in the repository's `NOTICE`.

## Build

```bash
../scripts/fetch_onnxruntime.sh           # once
cmake -S robots/g1 -B robots/g1/build -DCMAKE_BUILD_TYPE=Release
cmake --build robots/g1/build -j
```

Needs `unitree_sdk2`, CycloneDDS (`ddsc`/`ddscxx`), `lcm` (with `lcm-gen` on
PATH), `yaml-cpp`, `spdlog`/`fmt`, Boost.program_options and Eigen 3.

Two binaries: `g1_ctrl`, and `rt_bench`, which measures what the scheduler
actually gives you before you blame the policy for a missed deadline.

## Shape

```
main.cpp                  wait for the robot, then run the FSM
config/config.yaml        every state and every transition, as config
include/FSM/              the states
include/isaaclab/         Unitree's C++ mini-runtime: observation and action
                          managers, articulation, the ONNX runner
```

`config.yaml` is the file to read first. Each state names a policy, an env
config, and its outgoing transitions as gamepad chords — `Velocity: RT +
A.on_pressed` — parsed by `include/unitree_joystick_dsl.hpp`. Adding a state
that runs your own policy is an entry in that file, not C++.

## The four states

```
         LT + up            RT + X           RT + X
Passive ─────────> FixStand ───────> RlStand ───────> Velocity
   ^                  │  RT + A         │  RT + A        │
   └──────────────────┴────── B ────────┴────────────────┘
```

**Passive** is limp — the robot falls. **FixStand** interpolates to a standing
pose over 2 s and cannot recover from prone. **RlStand** runs the standing
policy; it is the state to check a robot, a set of gains or a new scene in.
**Velocity** runs the SMP joystick gait.

## Running your own policy

Export to ONNX with an `obs` input and an `actions` output, drop it in
`config/policy/<name>/v0/exported/`, describe it in a `params/*.yaml`, and
point a state at both. The runtime builds the observation vector from that YAML
— it has no compiled-in knowledge of any particular policy.

Then check the two agree:

```bash
uv run ../scripts/check_contract.py <policy>.onnx <params>.yaml
```

## Timing

The policy runs on its own thread at `step_dt` (50 Hz for the shipped
policies), separate from the FSM's control loop. Both report deadline
statistics; `UNITREE_RT_TELEMETRY=1` and `UNITREE_RT_DASHBOARD=1` are on by
default, `UNITREE_RT_LOG_EVERY` sets the log interval.

## Driving it from your own planner

`ArticulationData::use_vel_cmd_override` is the integration point: set it, write
`[vx, vy, wz]` before each step, and the `velocity_commands` observation reads
from there instead of the pad. Values are clamped to the env config's command
ranges either way. Nothing in this repository sets it.
