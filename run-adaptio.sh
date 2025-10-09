#!/usr/bin/env bash
set -euo pipefail

# Run the Adaptio application end-to-end.
# - Defaults to simulation mode with local data/logs dirs
# - Auto-selects Nix if available, otherwise uses cmake via ./adaptio.sh
# - Accepts extra args after -- to pass to the Adaptio binary

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${SCRIPT_DIR}"

usage() {
  cat <<'EOF'
Usage: ./run-adaptio.sh [OPTIONS] [-- <args to adaptio>]

Options:
  -h, --help         Show this help
  --sim              Use simulation for controller and image provider (default)
  --hw               Use hardware settings (pn_driver + basler)
  --nix              Force Nix build/run
  --cmake            Force cmake build/run via ./adaptio.sh
  --release          Build/run release variant (cmake path only)
  --no-build         Skip the build step if binary already exists
  --data-dir <path>  Application data directory (default: ./.adaptio/data)
  --logs-dir <path>  Application logs directory (default: ./.adaptio/logs)
  --config <path>    Path to configuration.yaml to use (auto-generated if unset)

Examples:
  ./run-adaptio.sh --sim -- --debug
  ./run-adaptio.sh --cmake --release -- --info
  ./run-adaptio.sh --hw --data-dir ./local/data --logs-dir ./local/logs
EOF
}

# ---- Argument parsing ----
MODE="sim"           # sim | hw
BUILD_SYSTEM="auto"  # auto | nix | cmake
RELEASE=0
NO_BUILD=0
DATA_DIR="${REPO_ROOT}/.adaptio/data"
LOGS_DIR="${REPO_ROOT}/.adaptio/logs"
CONFIG_DIR="${REPO_ROOT}/.adaptio/config"
CONFIG_FILE="${CONFIG_DIR}/configuration.yaml"
EXTRA_ARGS=()

if [[ $# -eq 0 ]]; then
  usage
  exit 1
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --sim) MODE="sim" ;;
    --hw) MODE="hw" ;;
    --nix) BUILD_SYSTEM="nix" ;;
    --cmake) BUILD_SYSTEM="cmake" ;;
    --release) RELEASE=1 ;;
    --no-build) NO_BUILD=1 ;;
    --data-dir) [[ $# -ge 2 ]] || { echo "--data-dir requires a value"; exit 1; }; DATA_DIR="$2"; shift ;;
    --logs-dir) [[ $# -ge 2 ]] || { echo "--logs-dir requires a value"; exit 1; }; LOGS_DIR="$2"; shift ;;
    --config) [[ $# -ge 2 ]] || { echo "--config requires a value"; exit 1; }; CONFIG_FILE="$2"; shift ;;
    --) shift; EXTRA_ARGS+=("$@"); break ;;
    *) EXTRA_ARGS+=("$1") ;;
  esac
  shift
done

# ---- Sanity checks ----
if [[ ! -f "${REPO_ROOT}/CMakeLists.txt" ]]; then
  echo "ERROR: Please run from the repository root (CMakeLists.txt not found)." >&2
  exit 1
fi

# ---- Prepare directories ----
mkdir -p "${DATA_DIR}" "${LOGS_DIR}" "$(dirname "${CONFIG_FILE}")"

# ---- Copy required calibration assets into data dir if missing ----
copy_if_missing() {
  local src="$1"; local dst_dir="$2"; local base
  if [[ -f "${src}" ]]; then
    base="$(basename "${src}")"
    if [[ ! -f "${dst_dir}/${base}" ]]; then
      cp "${src}" "${dst_dir}/${base}"
    fi
  fi
}
copy_if_missing "${REPO_ROOT}/assets/configuration/circular_weld_object_calibration.yaml" "${DATA_DIR}"
copy_if_missing "${REPO_ROOT}/assets/configuration/laser_torch_calibration.yaml" "${DATA_DIR}"

# Ensure a placeholder Profinet REMA file exists if referenced
if [[ ! -f "${DATA_DIR}/rema.xml" ]]; then
  printf '%s\n' '<rema/>' > "${DATA_DIR}/rema.xml"
fi

# ---- Generate configuration (if none provided / missing) ----
if [[ ! -f "${CONFIG_FILE}" ]]; then
  BASE_CFG="${REPO_ROOT}/assets/configuration/configuration.yaml"
  if [[ ! -f "${BASE_CFG}" ]]; then
    echo "ERROR: Base configuration not found at ${BASE_CFG}" >&2
    exit 1
  fi

  tmp_cfg="$(mktemp)"
  # Adjust types for simulation mode using an AWK section-aware rewrite
  awk -v mode="${MODE}" '
    function is_toplevel(line) { return match(line, /^[A-Za-z0-9_]+:[[:space:]]*$/) }
    function section_name(line) { if (match(line, /^([A-Za-z0-9_]+):[[:space:]]*$/, arr)) return arr[1]; return "" }
    {
      if (is_toplevel($0)) { sec=section_name($0) }
      if (mode=="sim") {
        if (sec=="controller" && match($0, /^[[:space:]]+type:[[:space:]]/)) { print "  type: simulation"; next }
        if (sec=="image_provider" && match($0, /^[[:space:]]+type:[[:space:]]/)) { print "  type: simulation"; next }
      }
      print $0
    }
  ' "${BASE_CFG}" > "${tmp_cfg}"

  # Rewrite absolute data paths to the chosen data directory
  data_esc="$(printf '%s' "${DATA_DIR}" | sed 's/[\/:&]/\\&/g')"
  sed "s#\/var\/lib\/adaptio#${data_esc}#g" "${tmp_cfg}" > "${CONFIG_FILE}"
  rm -f "${tmp_cfg}"
fi

# ---- Choose build system ----
if [[ "${BUILD_SYSTEM}" == "auto" ]]; then
  if command -v nix >/dev/null 2>&1; then BUILD_SYSTEM="nix"; else BUILD_SYSTEM="cmake"; fi
fi

# ---- Build (unless skipped) ----
if [[ "${NO_BUILD}" -eq 0 ]]; then
  if [[ "${BUILD_SYSTEM}" == "nix" ]]; then
    echo "Building with Nix..."
    nix build .#
  else
    echo "Building with CMake via ./adaptio.sh..."
    if [[ "${REPO_ROOT}/adaptio.sh" ]]; then
      if [[ "${RELEASE}" -eq 1 ]]; then
        "${REPO_ROOT}/adaptio.sh" --build --release
      else
        "${REPO_ROOT}/adaptio.sh" --build
      fi
    else
      # Fallback direct cmake (out-of-tree)
      mkdir -p "${REPO_ROOT}/build/${RELEASE:+release}" || true
      BUILD_DIR="${REPO_ROOT}/build/${RELEASE:+release}"
      CMAKE_BUILD_TYPE=$([[ "${RELEASE}" -eq 1 ]] && echo Release || echo Debug)
      cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
      cmake --build "${BUILD_DIR}" -j "$(($(nproc)+1))" --target adaptio
    fi
  fi
else
  echo "Skipping build as requested (--no-build)."
fi

# ---- Run ----
if [[ "${BUILD_SYSTEM}" == "nix" ]]; then
  echo "Running Adaptio (Nix result)..."
  ADAPTIO_DEV_CONF="${CONFIG_FILE}" "${REPO_ROOT}/result/bin/adaptio" \
    --data-dir "${DATA_DIR}" \
    --logs-dir "${LOGS_DIR}" \
    --config-file "${CONFIG_FILE}" \
    "${EXTRA_ARGS[@]}"
else
  echo "Running Adaptio (cmake binary via ./adaptio.sh)..."
  if [[ "${RELEASE}" -eq 1 ]]; then
    ADAPTIO_DEV_CONF="${CONFIG_FILE}" "${REPO_ROOT}/adaptio.sh" --run --release -- \
      --data-dir "${DATA_DIR}" \
      --logs-dir "${LOGS_DIR}" \
      --config-file "${CONFIG_FILE}" \
      "${EXTRA_ARGS[@]}"
  else
    ADAPTIO_DEV_CONF="${CONFIG_FILE}" "${REPO_ROOT}/adaptio.sh" --run -- \
      --data-dir "${DATA_DIR}" \
      --logs-dir "${LOGS_DIR}" \
      --config-file "${CONFIG_FILE}" \
      "${EXTRA_ARGS[@]}"
  fi
fi
