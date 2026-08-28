#!/usr/bin/env python3
"""Publish a Linux gamepad on the JOYSTICK channel as ``joy_t``.

You probably do not need this. In its default mode the sim reads ``/dev/input``
itself and the pad reaches the controller over DDS with nothing in between.
This is only for hand-driving a ``--scripted`` run, where the sim is taking its
pad off LCM and yours is on another machine or the far end of an ssh session.

    uv run scripts/joy_bridge.py                # /dev/input/js0
    uv run scripts/joy_bridge.py --device js1

Take turns with sim_fsm.py: two publishers on one channel interleave, and
whether a chord lands becomes a race.
"""

from __future__ import annotations

import argparse
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import lcm

ROOT = Path(__file__).resolve().parent.parent
LCM_DEF = ROOT / "lcm_types" / "smp_hw_lcm.lcm"

# struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; }
_EVENT = struct.Struct("IhBB")
_BUTTON, _AXIS, _INIT = 0x01, 0x02, 0x80

_N_AXES = 8
_N_BUTTONS = 11


def _import_lcmtypes():
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


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="js0", help="under /dev/input/")
    ap.add_argument("--channel", default="JOYSTICK")
    ap.add_argument("--rate", type=float, default=100.0,
                    help="publish rate; the pad is polled faster than this")
    ap.add_argument("--lcm-url", default="")
    args = ap.parse_args()

    path = Path("/dev/input") / args.device
    if not path.exists():
        raise SystemExit(f"{path} not found — is a pad plugged in? (ls /dev/input/js*)")

    lc = lcm.LCM(args.lcm_url) if args.lcm_url else lcm.LCM()
    axes = [0.0] * _N_AXES
    buttons = [0] * _N_BUTTONS
    period = 1.0 / args.rate

    print(f"{path} -> {args.channel} at {args.rate:g} Hz.  Ctrl-C to stop.")
    with open(path, "rb") as dev:
        import os
        os.set_blocking(dev.fileno(), False)
        next_pub = time.monotonic()
        while True:
            # Drain every event that has arrived, then publish one snapshot.
            # Publishing per event would put the pad's report rate on the wire.
            while True:
                raw = dev.read(_EVENT.size)
                if not raw:
                    break
                _, value, etype, number = _EVENT.unpack(raw)
                etype &= ~_INIT      # synthetic initial-state events count too
                if etype == _AXIS and number < _N_AXES:
                    axes[number] = value / 32767.0
                elif etype == _BUTTON and number < _N_BUTTONS:
                    buttons[number] = 1 if value else 0

            now = time.monotonic()
            if now >= next_pub:
                msg = joy_t()
                msg.utime = int(time.time() * 1e6)
                msg.frame_id = args.device
                msg.num_axes = _N_AXES
                msg.axes = list(axes)
                msg.num_buttons = _N_BUTTONS
                msg.buttons = list(buttons)
                lc.publish(args.channel, msg.encode())
                next_pub = now + period
            time.sleep(0.001)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
