# scripts/

| | |
| --- | --- |
| `setup_ubuntu.sh` | Check every build dependency; `--install` to get the missing ones. |
| `fetch_onnxruntime.sh` | Pull the CPU ONNX Runtime for this host. Run once. |
| `check_contract.py` | Assert a deploy YAML agrees with the ONNX it runs. |
| `check_upstream_diff.sh` | Diff a vendored tree against the commit `UPSTREAM` pins. |
| `sim2sim.sh` | tmux bring-up. `--scripted` for chords over LCM. |
| `sim_fsm.py` | Publish the chords that walk the FSM to a state. |
| `joy_bridge.py` | A pad onto the `JOYSTICK` channel; only for `--scripted` runs. |

`check_contract.py` is the one that earns its place. It found a real
disagreement the first time it ran: the hand-written YAML had rounded damping
up to 3% off the trained value, in a run with no gain randomisation to cover it.
