#!/usr/bin/env bash
# Bring up the sim2sim stack in a tmux session.
#
#   scripts/sim2sim.sh                     # pad in hand, you press Enter
#   scripts/sim2sim.sh --scripted          # chords over LCM, no pad needed
#   scripts/sim2sim.sh --auto              # implies --scripted, drives it too
#   scripts/sim2sim.sh --scene scene_obstacles.xml
#   scripts/sim2sim.sh --tiled             # even grid instead of left/right
#   scripts/sim2sim.sh --reset             # clobber an old session
#
# Default: the sim opens /dev/input itself and the pad arrives over DDS.
# --scripted: the sim takes joy_t off LCM so sim_fsm.py can publish chords.
#
# Commands are pre-typed, not run. Press Enter when each pane's dependencies
# are actually up; the pane order is the dependency chain.
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION=sim2sim
RESET=0
AUTO=0
ATTACH=1
SCENE=""
SCRIPTED=0
FSM_SEQ=rl-stand
LAYOUT=main-vertical
MAIN_WIDTH=55%

while [[ $# -gt 0 ]]; do
  case "$1" in
    --session)     SESSION="$2"; shift 2 ;;
    --reset)       RESET=1; shift ;;
    --auto)        AUTO=1; SCRIPTED=1; shift ;;
    --scripted)    SCRIPTED=1; shift ;;
    --no-attach)   ATTACH=0; shift ;;
    --scene)       SCENE="$2"; shift 2 ;;
    --layout)      LAYOUT="$2"; shift 2 ;;
    --main-width)  MAIN_WIDTH="$2"; shift 2 ;;
    --tiled)       LAYOUT=tiled; MAIN_WIDTH=""; shift ;;
    --fsm)         FSM_SEQ="$2"; shift 2 ;;
    -h|--help)     sed -n '2,14p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

command -v tmux >/dev/null || { echo "tmux is not installed" >&2; exit 1; }

SIM_DIR="${ROOT}/sim/simulate"
G1_DIR="${ROOT}/deploy/robots/g1"

if [[ ! -f "${SIM_DIR}/CMakeLists.txt" ]]; then
  echo "error: sim/ is missing its sources — this repository vendors the" >&2
  echo "       simulator, so a clone should already have them. See UPSTREAM." >&2
  exit 1
fi

# The deploy YAML restates what the ONNX already knows. Nothing else catches a
# disagreement, and the robot is confidently wrong when there is one.
POLICY_DIR="${G1_DIR}/config/policy/velocity/v0"
for onnx in "${POLICY_DIR}"/exported/*.onnx; do
  uv run "${ROOT}/scripts/check_contract.py" "${onnx}" \
     "${POLICY_DIR}/params/deploy_mjlab.yaml" >/dev/null || {
    echo "!! ${onnx##*/} disagrees with deploy_mjlab.yaml — run check_contract.py" >&2
    exit 1
  }
done
echo "contract ok for $(ls "${POLICY_DIR}"/exported/*.onnx | wc -l) polic(y/ies)"

# A stale binary reads the current config with the old code's expectations, and
# that failure is always oblique. Incremental build is a no-op when nothing
# changed, so just always do it.
build_fresh() {
  local src="$1" build="$1/build"
  local log; log="$(mktemp)"
  if [[ ! -d "${build}" ]]; then
    if ! cmake -S "${src}" -B "${build}" -DCMAKE_BUILD_TYPE=Release >"${log}" 2>&1; then
      echo "error: cmake configure failed for ${src}:" >&2; tail -25 "${log}" >&2
      rm -f "${log}"; exit 1
    fi
  fi
  if ! cmake --build "${build}" -j"$(nproc)" >"${log}" 2>&1; then
    echo "error: build failed in ${build}:" >&2; tail -25 "${log}" >&2; rm -f "${log}"; exit 1
  fi
  rm -f "${log}"
}
build_fresh "${G1_DIR}"
build_fresh "${SIM_DIR}"

# Both ends need the same CycloneDDS config to find each other on loopback.
DDS="export CYCLONEDDS_URI=file://${SIM_DIR}/cyclonedds.xml;"

# Separate from the URI above, and it wins: unitree_sdk2's Init(domain, iface)
# configures CycloneDDS programmatically when iface is non-empty, overriding
# CYCLONEDDS_URI. The sim always passes one, so keep this equal to its
# config.yaml `interface` or the two never discover each other and the
# controller waits on rt/lowstate forever.
G1_IFACE=lo

SIM_ARGS=""
[[ -n "${SCENE}" ]] && SIM_ARGS="--scene ${SCENE}"

if [[ "${SCRIPTED}" == 1 ]]; then
  SIM_ARGS="${SIM_ARGS} --joystick lcm"
else
  SIM_ARGS="${SIM_ARGS} --joystick xbox"
  # Fail here, not at the first chord that silently does nothing.
  if [[ ! -r /dev/input/js0 ]]; then
    echo "error: no readable /dev/input/js0 — plug a pad in, or use --scripted" >&2
    echo "       (--scripted drives the FSM over LCM and needs no pad at all)" >&2
    exit 1
  fi
fi

if [[ "${RESET}" == 1 ]]; then tmux kill-session -t "${SESSION}" 2>/dev/null || true; fi
if tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "session '${SESSION}' already exists — attach, or re-run with --reset" >&2
  exit 1
fi

# "title|command|bg:fg". g1_ctrl is red: it is the one that moves the robot.
NODES=(
  "sim|${DDS} cd ${SIM_DIR} && ./build/unitree_mujoco ${SIM_ARGS}|33:231"
  "g1_ctrl|${DDS} cd ${G1_DIR} && ./build/g1_ctrl -n ${G1_IFACE}|124:231"
)
[[ "${SCRIPTED}" == 1 ]] && NODES+=("fsm|uv run scripts/sim_fsm.py ${FSM_SEQ}|100:231")

tmux new-session -d -s "${SESSION}" -n stack -c "${ROOT}"

# Percentages for main-pane-width need tmux 3.4. Older ones error out, which
# under `set -e` kills us before any pane exists, so fall back to cells. Ask
# the terminal for its width: the detached session reports tmux's default 80.
if [[ "${LAYOUT}" == main-* && -n "${MAIN_WIDTH}" ]]; then
  width="${MAIN_WIDTH}"
  if [[ "${width}" == *% ]] &&
     ! tmux set-option -t "${SESSION}" -g main-pane-width "${width}" 2>/dev/null; then
    cols="$(tput cols 2>/dev/null || true)"
    [[ "${cols}" =~ ^[0-9]+$ && "${cols}" -gt 0 ]] || cols=80
    width=$(( cols * ${width%\%} / 100 ))
    [[ "${width}" -ge 20 ]] || width=20
  fi
  tmux set-option -t "${SESSION}" -g main-pane-width "${width}"
fi

# The title carries its own colour escape, so keep the format bare.
tmux set-option -t "${SESSION}" -g pane-border-status top
tmux set-option -t "${SESSION}" -g pane-border-format '#{pane_title}'
tmux set-option -t "${SESSION}" -g pane-border-style 'fg=colour240'
tmux set-option -t "${SESSION}" -g pane-active-border-style 'fg=colour196,bold'
tmux set-option -t "${SESSION}" -g mouse on        # click a pane to switch to it

for i in "${!NODES[@]}"; do
  entry="${NODES[$i]}"
  title="${entry%%|*}"
  rest="${entry#*|}"
  cmd="${rest%|*}"
  colors="${rest##*|}"
  strip="$(printf '#[bg=colour%s,fg=colour%s,bold] [%d] %s #[default]' \
             "${colors%%:*}" "${colors##*:}" "$i" "${title}")"
  # Re-tile after each split, or the last pane ends up unreadably narrow.
  [[ $i -gt 0 ]] && tmux split-window -t "${SESSION}:stack" -c "${ROOT}"
  tmux select-layout -t "${SESSION}:stack" "${LAYOUT}" >/dev/null
  tmux select-pane   -t "${SESSION}:stack" -T "${strip}"
  tmux send-keys     -t "${SESSION}:stack" "${cmd}"     # typed, NOT sent
done
tmux select-layout -t "${SESSION}:stack" "${LAYOUT}" >/dev/null
tmux select-pane -t "${SESSION}:stack.0"

if [[ "${SCRIPTED}" == 1 ]]; then
  JOY_DESC="gamepad over LCM"
  FSM_PANE_NOTE="    2  fsm       publishes the chords: Passive -> FixStand -> ${FSM_SEQ}, then exits
"
else
  JOY_DESC="reading /dev/input/js0 directly"
  FSM_PANE_NOTE="
  Then drive it yourself on the pad:  L2 + Up for FixStand, R2 + X for
  RlStand, R2 + X again for Velocity. B drops to Passive, which is LIMP."
fi

cat <<MSG
─────────────────────────────────────────────────────────────────────────
  tmux session '${SESSION}'.   Attach:  tmux attach -t ${SESSION}
─────────────────────────────────────────────────────────────────────────

  Each pane has its command PRE-TYPED. Press Enter in this order:

    0  sim       MuJoCo + the DDS bridge  (${JOY_DESC})
    1  g1_ctrl   the controller — wait for the sim first
${FSM_PANE_NOTE}
  The elastic band holds the G1 up through the FSM transitions and releases
  at sim/simulate/config.yaml's band_release_s (30 s) — reach a standing state
  before it lets go. Release it too early and the robot falls while still limp
  in Passive; FixStand cannot get up from prone, so everything after that runs
  on a fallen robot and looks like a broken policy.

  Left stick is forward/strafe, right stick is yaw.

  Layout is ${LAYOUT}${MAIN_WIDTH:+ (pane 0 gets ${MAIN_WIDTH})};
  --tiled for an even grid, --main-width 40% to rebalance.

  Switch pane:  click it (mouse is on)  —  or Ctrl-b o / arrows
  Zoom a pane:  Ctrl-b z                   (toggles full screen)
  Detach:       Ctrl-b d                   (keeps running over SSH)
  Re-attach:    tmux attach -t ${SESSION}
  Tear down:    tmux kill-session -t ${SESSION}
MSG

if [[ "${AUTO}" == 1 ]]; then
  # 6 s gaps + ~5 s of chords reaches RlStand around 20 s, inside the 30 s band.
  echo "  --auto: running panes in order..."
  for i in 0 1 2; do tmux send-keys -t "${SESSION}.${i}" C-m; sleep 6; done
fi

# Not `[[ ]] && exec`: as the last command that makes --no-attach exit 1.
if [[ "${ATTACH}" == 1 ]]; then
  exec tmux attach-session -t "${SESSION}"
fi
