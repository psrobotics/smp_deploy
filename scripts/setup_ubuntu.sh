#!/usr/bin/env bash
# Check for everything this repo needs to build, and optionally install it.
#
#   scripts/setup_ubuntu.sh              # report what is missing, exit 1 if any
#   scripts/setup_ubuntu.sh --install    # install the missing pieces
#   scripts/setup_ubuntu.sh --install -y # ...without prompting
#
# Tested on Ubuntu 22.04 and 24.04, x86_64 and aarch64. Everything installs to
# /usr/local or your home directory; nothing is written inside this repo except
# deploy/thirdparty/onnxruntime-*.
set -uo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
MUJOCO_VERSION="${MUJOCO_VERSION:-3.3.6}"
MUJOCO_DIR="${HOME}/.mujoco/mujoco-${MUJOCO_VERSION}"
SDK_SRC="${SDK_SRC:-${HOME}/.cache/smp_hw/unitree_sdk2}"
PREFIX=/usr/local

INSTALL=0
ASSUME_YES=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install)   INSTALL=1; shift ;;
    -y|--yes)    ASSUME_YES=1; shift ;;
    -h|--help)   sed -n '2,11p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

case "$(uname -m)" in
  x86_64)  ARCH=x86_64; ORT_ARCH=x64 ;;
  aarch64) ARCH=aarch64; ORT_ARCH=aarch64 ;;
  *) echo "unsupported architecture: $(uname -m)" >&2; exit 1 ;;
esac

MISSING=()
ok()   { printf '  \033[32mok\033[0m    %s\n' "$1"; }
miss() { printf '  \033[31mmiss\033[0m  %s\n' "$1"; MISSING+=("$2"); }
step() { printf '\n\033[1m%s\033[0m\n' "$1"; }

run() {  # echo then run, with sudo when not already root
  local pre=""; [[ $EUID -ne 0 ]] && pre="sudo "
  echo "    + ${pre}$*"
  ${pre:+sudo} "$@"
}

# ── apt ──────────────────────────────────────────────────────────────────────
# liblcm-bin is separate from liblcm-dev and easy to miss: it is what provides
# lcm-gen, which both CMakeLists call at build time.
APT_PKGS=(
  build-essential cmake pkg-config git git-lfs tmux curl
  libeigen3-dev libyaml-cpp-dev libspdlog-dev libfmt-dev
  libboost-program-options-dev liblcm-dev liblcm-bin
  libglfw3-dev libgl1-mesa-dev
)

step "apt packages"
APT_MISSING=()
for pkg in "${APT_PKGS[@]}"; do
  if dpkg -s "$pkg" >/dev/null 2>&1; then ok "$pkg"; else
    printf '  \033[31mmiss\033[0m  %s\n' "$pkg"; APT_MISSING+=("$pkg")
  fi
done
[[ ${#APT_MISSING[@]} -gt 0 ]] && MISSING+=("apt")

# ── uv ───────────────────────────────────────────────────────────────────────
step "python tooling"
command -v uv >/dev/null && ok "uv ($(uv --version 2>/dev/null))" || miss "uv" uv

# ── unitree_sdk2 + CycloneDDS ────────────────────────────────────────────────
#
# The SDK's own install() covers its headers and static lib only. CycloneDDS
# lives in its thirdparty/ and has to be copied to the prefix by hand, which is
# why a plain `make install` still leaves you without ddsc/ddscxx.
step "unitree_sdk2 + CycloneDDS"
[[ -f "${PREFIX}/lib/cmake/unitree_sdk2/unitree_sdk2Config.cmake" ]] \
  && ok "unitree_sdk2" || miss "unitree_sdk2" sdk
[[ -d "${PREFIX}/include/ddscxx" && -e "${PREFIX}/lib/libddsc.so" ]] \
  && ok "cyclonedds (ddsc, ddscxx)" || miss "cyclonedds (ddsc, ddscxx)" sdk

# ── MuJoCo ───────────────────────────────────────────────────────────────────
#
# The release tarball, not the pip package: the simulator compiles MuJoCo's own
# simulate/ sources, which pip does not ship.
step "MuJoCo ${MUJOCO_VERSION}"
found_mj=""
for c in "${MUJOCO_DIR}" "${ROOT}/sim/simulate/mujoco" ${MUJOCO_DIR%/*}/mujoco-*; do
  [[ -f "${c}/simulate/simulate.cc" ]] && { found_mj="$c"; break; }
done
[[ -n "${found_mj}" ]] && ok "MuJoCo at ${found_mj}" || miss "MuJoCo release tarball" mujoco

# ── ONNX Runtime ─────────────────────────────────────────────────────────────
step "ONNX Runtime"
ORT="${ROOT}/deploy/thirdparty/onnxruntime-linux-${ORT_ARCH}-1.22.0"
[[ -f "${ORT}/lib/libonnxruntime.so.1.22.0" ]] && ok "onnxruntime 1.22.0" \
  || miss "onnxruntime 1.22.0" ort

# ── git-lfs payload ──────────────────────────────────────────────────────────
#
# Cloning without git-lfs leaves the policies as ~130-byte pointer files. The
# controller then dies inside ONNX Runtime with nothing that names the cause.
step "policy files"
POLICY="${ROOT}/deploy/robots/g1/config/policy/velocity/v0/exported"
lfs_bad=0
for f in "${POLICY}"/*.onnx; do
  [[ -e "$f" ]] || continue
  if head -c 40 "$f" | grep -q "git-lfs.github.com"; then
    printf '  \033[31mmiss\033[0m  %s is an LFS pointer, not the model\n' "$(basename "$f")"
    lfs_bad=1
  else
    ok "$(basename "$f") ($(du -h "$f" | cut -f1))"
  fi
done
[[ $lfs_bad -eq 1 ]] && MISSING+=(lfs)

# ── report ───────────────────────────────────────────────────────────────────
if [[ ${#MISSING[@]} -eq 0 ]]; then
  step "all present"
  echo "  Build and run:  scripts/sim2sim.sh"
  exit 0
fi

if [[ ${INSTALL} -eq 0 ]]; then
  step "missing: ${MISSING[*]}"
  echo "  Install them with:  $0 --install"
  exit 1
fi

if [[ ${ASSUME_YES} -eq 0 ]]; then
  step "about to install: ${MISSING[*]}"
  read -rp "  Continue? [y/N] " reply
  [[ "${reply}" =~ ^[Yy] ]] || { echo "  aborted"; exit 1; }
fi

for what in "${MISSING[@]}"; do
  case "${what}" in
    apt)
      step "installing apt packages"
      run apt-get update
      run apt-get install -y "${APT_MISSING[@]}"
      ;;
    uv)
      step "installing uv"
      curl -LsSf https://astral.sh/uv/install.sh | sh
      export PATH="${HOME}/.local/bin:${PATH}"
      echo "  note: uv is in ~/.local/bin — open a new shell, or"
      echo "        export PATH=\"\${HOME}/.local/bin:\${PATH}\""
      ;;
    sdk)
      step "installing unitree_sdk2 + CycloneDDS"
      if [[ ! -d "${SDK_SRC}/.git" ]]; then
        mkdir -p "$(dirname "${SDK_SRC}")"
        git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2 "${SDK_SRC}"
      fi
      cmake -S "${SDK_SRC}" -B "${SDK_SRC}/build" -DCMAKE_INSTALL_PREFIX="${PREFIX}"
      run cmake --install "${SDK_SRC}/build"
      # The part the SDK's install() leaves out.
      run cp -a "${SDK_SRC}/thirdparty/include/." "${PREFIX}/include/"
      run cp -a "${SDK_SRC}/thirdparty/lib/${ARCH}/." "${PREFIX}/lib/"
      run ldconfig
      ;;
    mujoco)
      step "installing MuJoCo ${MUJOCO_VERSION}"
      mj_arch=$([[ "${ARCH}" == x86_64 ]] && echo x86_64 || echo aarch64)
      url="https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${mj_arch}.tar.gz"
      mkdir -p "${HOME}/.mujoco"
      tmp="$(mktemp -d)"
      echo "    + curl ${url}"
      curl -fL --progress-bar -o "${tmp}/mujoco.tgz" "${url}"
      tar -xzf "${tmp}/mujoco.tgz" -C "${HOME}/.mujoco"
      rm -rf "${tmp}"
      test -f "${MUJOCO_DIR}/simulate/simulate.cc" \
        || echo "  !! ${MUJOCO_DIR} has no simulate/ sources — wrong tarball?" >&2
      ;;
    ort)
      step "installing ONNX Runtime"
      "${ROOT}/scripts/fetch_onnxruntime.sh"
      ;;
    lfs)
      step "fetching the policy files"
      git -C "${ROOT}" lfs install --local
      git -C "${ROOT}" lfs pull
      ;;
  esac
done

step "done — re-run without --install to verify"
