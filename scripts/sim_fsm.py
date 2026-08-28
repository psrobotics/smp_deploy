#!/usr/bin/env python3
"""Drive the G1 controller's FSM by publishing gamepad chords over LCM.

Needs the sim in ``--joystick lcm`` mode. The controller only changes state on
chords, and an unattended run has nobody to press them.

    uv run scripts/sim_fsm.py stand       # Passive -> FixStand
    uv run scripts/sim_fsm.py rl-stand    # ...     -> RlStand
    uv run scripts/sim_fsm.py velocity    # ...     -> Velocity
    uv run scripts/sim_fsm.py passive     # drop to Passive; the robot goes LIMP
    uv run scripts/sim_fsm.py chord LT up # anything else in config.yaml

Chords have to be held, not tapped. The SDK low-passes the trigger axes
(smooth 0.03) before thresholding at 0.5, which is ~25 updates from rest, and
the button half is an edge, so the trigger must already be down when it fires.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import lcm

ROOT = Path(__file__).resolve().parent.parent
LCM_DEF = ROOT / "lcm_types" / "smp_hw_lcm.lcm"
JOYSTICK = "JOYSTICK"


def _import_lcmtypes():
    """Generate the Python bindings on demand; never commit generated code.

    A committed binding that drifts from its definition is a silent wire-format
    mismatch, and it shows up as garbage floats rather than an error.
    """
    cache = Path(tempfile.gettempdir()) / "smp_hw_lcm_py"
    stamp = cache / ".stamp"
    if not stamp.exists() or stamp.stat().st_mtime < LCM_DEF.stat().st_mtime:
        cache.mkdir(parents=True, exist_ok=True)
        subprocess.run(["lcm-gen", "-p", "--ppath", str(cache), str(LCM_DEF)],
                       check=True)
        stamp.touch()
    sys.path.insert(0, str(cache))
    from smp_hw_lcm import joy_t  # noqa: E402
    return joy_t


joy_t = _import_lcmtypes()

# Linux xpad layout — what a real pad reports and what the simulator's LCM
# joystick reads.
_AXES = 8
_BUTTONS = 11
_AXIS = {"LX": 0, "LY": 1, "LT": 2, "RX": 3, "RY": 4, "RT": 5, "DX": 6, "DY": 7}
_BUTTON = {"A": 0, "B": 1, "X": 2, "Y": 3, "LB": 4, "RB": 5,
           "back": 6, "start": 7, "LS": 9, "RS": 10}
# The D-pad is a hat axis; up is negative on Linux.
_HAT = {"up": ("DY", -1.0), "down": ("DY", 1.0),
        "left": ("DX", -1.0), "right": ("DX", 1.0)}


class Pad:
    def __init__(self, lc, channel: str, rate_hz: float = 100.0):
        self._lc = lc
        self._channel = channel
        self._period = 1.0 / rate_hz
        self.reset()

    def reset(self) -> None:
        self.axes = [0.0] * _AXES
        self.buttons = [0] * _BUTTONS

    def set(self, name: str, on: bool) -> None:
        if name in _BUTTON:
            self.buttons[_BUTTON[name]] = 1 if on else 0
        elif name in _HAT:
            axis, value = _HAT[name]
            self.axes[_AXIS[axis]] = value if on else 0.0
        elif name in _AXIS:
            self.axes[_AXIS[name]] = 1.0 if on else 0.0
        else:
            raise SystemExit(f"unknown control: {name}")

    def hold(self, seconds: float) -> None:
        """Publish the current state for `seconds`. Holding matters — see the
        module docstring on trigger smoothing."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            msg = joy_t()
            msg.utime = int(time.time() * 1e6)
            msg.frame_id = "sim_fsm"
            msg.num_axes = _AXES
            msg.axes = list(self.axes)
            msg.num_buttons = _BUTTONS
            msg.buttons = list(self.buttons)
            self._lc.publish(self._channel, msg.encode())
            time.sleep(self._period)


def chord(pad: Pad, trigger: str, button: str) -> None:
    """Press `trigger + button.on_pressed`, the shape every transition uses."""
    print(f"  {trigger} + {button}")
    pad.reset()
    pad.set(trigger, True)
    pad.hold(0.5)          # let the trigger's low-pass cross threshold
    pad.set(button, True)
    pad.hold(0.3)          # the .on_pressed edge
    pad.reset()
    pad.hold(0.3)          # release, so the next edge is a fresh one


def main() -> None:
    ap = argparse.ArgumentParser(description="Drive the G1 FSM over LCM")
    ap.add_argument("action",
                    choices=("stand", "rl-stand", "velocity", "passive", "chord"))
    ap.add_argument("rest", nargs="*", help="for 'chord': <trigger> <button>")
    ap.add_argument("--channel", default=JOYSTICK)
    ap.add_argument("--lcm-url", default="")
    ap.add_argument("--settle", type=float, default=2.0,
                    help="seconds between transitions; FixStand runs a 2 s "
                         "stand-up trajectory, so do not rush it")
    args = ap.parse_args()

    lc = lcm.LCM(args.lcm_url) if args.lcm_url else lcm.LCM()
    pad = Pad(lc, args.channel)

    if args.action == "chord":
        if len(args.rest) != 2:
            raise SystemExit("chord needs <trigger> <button>")
        chord(pad, args.rest[0], args.rest[1])
        return

    if args.action == "passive":
        print("-> Passive  (the robot goes limp)")
        pad.reset(); pad.set("B", True); pad.hold(0.3); pad.reset(); pad.hold(0.2)
        return

    # Each step is a separate chord with a settle in between, because the
    # controller is running a real trajectory between them.
    print("Passive -> FixStand")
    chord(pad, "LT", "up")
    time.sleep(args.settle)
    if args.action == "stand":
        return

    # Route through RlStand even when Velocity is the destination. FixStand has
    # a direct RT + A edge, but taking it puts the hopping gait on the robot
    # before anything has checked it.
    print("FixStand -> RlStand  (standing policy)")
    chord(pad, "RT", "X")
    time.sleep(args.settle)
    if args.action == "rl-stand":
        return

    print("RlStand -> Velocity  (SMP joystick gait)")
    chord(pad, "RT", "X")
    time.sleep(0.5)


if __name__ == "__main__":
    main()
