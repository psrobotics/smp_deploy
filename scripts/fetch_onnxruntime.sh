#!/usr/bin/env bash
# Fetch the official ONNX Runtime release for this host into deploy/thirdparty/.
#
# Not committed: 20-40 MB of prebuilt binaries per arch would cost every clone.
# CPU build only — the policies are 854 KB and infer well inside the 20 ms
# control period, so the CUDA provider buys nothing and costs a cuDNN dep.
set -euo pipefail

VERSION="${ORT_VERSION:-1.22.0}"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${ROOT}/deploy/thirdparty"

case "$(uname -m)" in
  x86_64)  ARCH=x64 ;;
  aarch64) ARCH=aarch64 ;;
  *) echo "error: no ONNX Runtime release mapped for $(uname -m)" >&2; exit 1 ;;
esac

NAME="onnxruntime-linux-${ARCH}-${VERSION}"
URL="https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/${NAME}.tgz"

if [[ -f "${DEST}/${NAME}/lib/libonnxruntime.so.${VERSION}" ]]; then
  echo "already present: ${DEST}/${NAME}"
  exit 0
fi

mkdir -p "${DEST}"
echo "fetching ${URL}"
tmp="$(mktemp -d)"
trap 'rm -rf "${tmp}"' EXIT
curl -fL --progress-bar -o "${tmp}/ort.tgz" "${URL}"
tar -xzf "${tmp}/ort.tgz" -C "${DEST}"

test -f "${DEST}/${NAME}/lib/libonnxruntime.so.${VERSION}"
echo "installed: ${DEST}/${NAME}"
