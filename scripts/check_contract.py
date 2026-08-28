#!/usr/bin/env python3
"""Assert a deploy YAML agrees with the ONNX policy it runs.

mjlab writes the policy's interface into the ONNX metadata at export: joint
order, gains, default pose, action scale, observation layout. The deploy YAML
restates all of it for the C++ runtime, which cannot read that metadata. A
permuted joint order or a stale action scale gives you a robot that is
confidently wrong, with no error anywhere.

    uv run scripts/check_contract.py <policy>.onnx <params>.yaml

Exit 0 if it holds, 1 if it does not.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import onnx
import yaml

# Same tensors, different vocabulary: mjlab names them after the manager term,
# the C++ runtime after its registered observation function.
OBS_ALIASES = {
    "joint_pos": "joint_pos_rel",
    "joint_vel": "joint_vel_rel",
    "actions": "last_action",
    "command": "velocity_commands",
}

# The YAML is read by humans, so allow rounding. Anything past this is real.
RTOL = 0.01
ATOL = 5e-3


class Report:
    def __init__(self) -> None:
        self.failures: list[str] = []
        self.notes: list[str] = []

    def check(self, ok: bool, label: str, detail: str = "") -> None:
        if ok:
            print(f"  ok    {label}")
        else:
            print(f"  FAIL  {label}")
            if detail:
                for line in detail.splitlines():
                    print(f"          {line}")
            self.failures.append(label)

    def check_close(self, label: str, onnx_vals: list[float],
                    yaml_vals: list[float]) -> None:
        ok, detail = close(onnx_vals, yaml_vals)
        self.check(ok, label, detail)

    def note(self, label: str, detail: str) -> None:
        print(f"  note  {label}")
        for line in detail.splitlines():
            print(f"          {line}")
        self.notes.append(label)


def close(a: list[float], b: list[float]) -> tuple[bool, str]:
    if len(a) != len(b):
        return False, f"length {len(a)} vs {len(b)}"
    bad = [
        f"[{i}] onnx={x:.6g} yaml={y:.6g}"
        for i, (x, y) in enumerate(zip(a, b))
        if abs(x - y) > ATOL + RTOL * abs(x)
    ]
    return (not bad), "\n".join(bad[:8] + (["..."] if len(bad) > 8 else []))


def onnx_contract(path: Path) -> tuple[dict[str, str], int, int]:
    model = onnx.load(str(path))
    meta = {kv.key: kv.value for kv in model.metadata_props}
    if not meta:
        raise SystemExit(
            f"{path.name} carries no metadata — it was not exported by mjlab's "
            "runner, so there is no contract to check against."
        )

    def dim(proto) -> int:
        shape = proto.type.tensor_type.shape.dim
        return shape[-1].dim_value

    return meta, dim(model.graph.input[0]), dim(model.graph.output[0])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("onnx", type=Path)
    ap.add_argument("yaml", type=Path)
    args = ap.parse_args()

    meta, obs_dim, act_dim = onnx_contract(args.onnx)
    cfg = yaml.safe_load(args.yaml.read_text())
    r = Report()

    floats = lambda key: [float(x) for x in meta[key].split(",")]
    names = lambda key: meta[key].split(",")

    joints = names("joint_names")
    n = len(joints)
    print(f"\n{args.onnx.name}")
    print(f"  policy   obs[{obs_dim}] -> actions[{act_dim}], {n} joints, "
          f"run '{meta.get('run_path', '?')}'")
    print(f"  against  {args.yaml.name}\n")

    # ── actuation ────────────────────────────────────────────────────────────
    r.check(act_dim == n, f"action width matches joint count ({act_dim} == {n})")
    r.check_close("joint stiffness", floats("joint_stiffness"), cfg["stiffness"])
    r.check_close("joint damping", floats("joint_damping"), cfg["damping"])
    r.check_close("default joint pose", floats("default_joint_pos"),
                  cfg["default_joint_pos"])

    action = cfg["actions"]["JointPositionAction"]
    r.check_close("action scale", floats("action_scale"), action["scale"])

    # q_target = raw_action * scale + offset, and offset is the default pose.
    # Drift here and the robot stands wrong from the first step.
    r.check_close("action offset == default joint pose",
                  floats("default_joint_pos"), action["offset"])

    ids = cfg["joint_ids_map"]
    r.check(sorted(ids) == list(range(n)),
            f"joint_ids_map is a permutation of 0..{n - 1}",
            f"got {ids}")

    # ── observations ─────────────────────────────────────────────────────────
    expected = [OBS_ALIASES.get(t, t) for t in names("observation_names")]
    actual = list(cfg["observations"].keys())
    r.check(expected == actual, "observation terms match, in order",
            f"onnx: {expected}\nyaml: {actual}")

    width = sum(len(t["scale"]) * max(1, t.get("history_length", 1) or 1)
                for t in cfg["observations"].values())
    r.check(width == obs_dim,
            f"observation terms sum to the model's input width ({width} == {obs_dim})",
            "\n".join(f"{k}: {len(v['scale'])} x {v.get('history_length', 1)}"
                      for k, v in cfg["observations"].items()))

    # ── commands ─────────────────────────────────────────────────────────────
    # Not an equality check. Narrower than training is a deliberate margin;
    # wider is extrapolation, where the policy owes you nothing.
    ranges = cfg["commands"]["base_velocity"]["ranges"]
    r.note("command ranges are deploy-side, not from the policy",
           "\n".join(f"{k}: {v}" for k, v in ranges.items() if v is not None)
           + "\ncompare against the training ranges in the run's config.yaml;"
           + "\nnarrower is intentional, wider is extrapolation.")

    print()
    if r.failures:
        print(f"CONTRACT BROKEN — {len(r.failures)} check(s) failed: "
              f"{', '.join(r.failures)}")
        return 1
    print("contract holds.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
