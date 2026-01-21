#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Build script for my_online_ik + RTOSIM + dependencies
# Author: Mathis D’Haene
# Version: 5 — GitHub-friendly, reproducible, no global find(~)
# ============================================================

# ----------------------------
# Project layout
# ----------------------------
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTOSIM_DIR="${ROOT_DIR}/RTOSIM"
BUILD_SUPPORT_DIR="${ROOT_DIR}/build_support"

# ----------------------------
# Compilers (override via env)
# ----------------------------
CC="${CC:-gcc-11}"
CXX="${CXX:-g++-11}"

# ----------------------------
# Install prefixes (override via env)
# ----------------------------
OPENSIM_PREFIX="${OPENSIM_PREFIX:-$HOME/opensim-core-install}"
OPENSIM_DEPS_PREFIX="${OPENSIM_DEPS_PREFIX:-$HOME/opensim_dependencies_install}"
SIMBODY_PREFIX="${SIMBODY_PREFIX:-$OPENSIM_DEPS_PREFIX/simbody}"

CONCURRENCY_INSTALL="${CONCURRENCY_INSTALL:-$HOME/concurrency-install}"
FILTER_INSTALL="${FILTER_INSTALL:-$HOME/filter-install}"
SPDLOG_DIR="${SPDLOG_DIR:-$HOME/spdlog-1.5.0-install/lib/cmake/spdlog}"

# ----------------------------
# Optional: header-only spdlog include path
# (kept minimal; can be removed if you always use spdlog CMake target)
# ----------------------------
export CPLUS_INCLUDE_PATH="$HOME/spdlog-1.5.0-install/include:${CPLUS_INCLUDE_PATH:-}"

# ----------------------------
# Helpers
# ----------------------------
nproc_safe() { command -v nproc >/dev/null 2>&1 && nproc || echo 4; }
abspath() { python3 - <<'PY' "$1"
import os,sys
print(os.path.abspath(os.path.expanduser(sys.argv[1])))
PY
}
need_file() {
  local f="$1"
  local hint="$2"
  if [ ! -f "$f" ]; then
    echo "[error] Missing: $f"
    echo "[hint]  $hint"
    exit 1
  fi
}

# Normalize prefixes (expand ~)
OPENSIM_PREFIX="$(abspath "$OPENSIM_PREFIX")"
OPENSIM_DEPS_PREFIX="$(abspath "$OPENSIM_DEPS_PREFIX")"
SIMBODY_PREFIX="$(abspath "$SIMBODY_PREFIX")"
CONCURRENCY_INSTALL="$(abspath "$CONCURRENCY_INSTALL")"
FILTER_INSTALL="$(abspath "$FILTER_INSTALL")"

# ----------------------------
# Banner
# ----------------------------
echo "=============================================="
echo "     🧠 Building my_online_ik pipeline"
echo "=============================================="
echo
echo "[info] ROOT_DIR           = ${ROOT_DIR}"
echo "[info] RTOSIM_DIR         = ${RTOSIM_DIR}"
echo "[info] BUILD_SUPPORT_DIR  = ${BUILD_SUPPORT_DIR}"
echo "[info] CC / CXX           = ${CC} / ${CXX}"
echo
echo "[info] OPENSIM_PREFIX      = ${OPENSIM_PREFIX}"
echo "[info] OPENSIM_DEPS_PREFIX = ${OPENSIM_DEPS_PREFIX}"
echo "[info] SIMBODY_PREFIX      = ${SIMBODY_PREFIX}"
echo "[info] CONCURRENCY_INSTALL = ${CONCURRENCY_INSTALL}"
echo "[info] FILTER_INSTALL      = ${FILTER_INSTALL}"
echo "[info] SPDLOG_DIR          = ${SPDLOG_DIR}"
echo

# ============================================================
# 0) Sanity checks (no auto-find, reproducible)
# ============================================================
echo "[info] Checking required dependencies..."

need_file \
  "${OPENSIM_PREFIX}/lib/cmake/OpenSim/OpenSimConfig.cmake" \
  "Install OpenSim, then set OPENSIM_PREFIX (e.g. export OPENSIM_PREFIX=\$HOME/opensim-core-install)."

need_file \
  "${SIMBODY_PREFIX}/lib/cmake/simbody/SimbodyConfig.cmake" \
  "Simbody comes from OpenSim deps superbuild. Set OPENSIM_DEPS_PREFIX or SIMBODY_PREFIX (e.g. export OPENSIM_DEPS_PREFIX=\$HOME/opensim_dependencies_install)."

if [ ! -d "${RTOSIM_DIR}" ]; then
  echo "[error] RTOSIM directory not found at: ${RTOSIM_DIR}"
  echo "[hint]  If RTOSIM is a submodule: git submodule update --init --recursive"
  exit 1
fi

echo "[ok] Dependencies look present."
echo

# ============================================================
# 1) Build Concurrency
# ============================================================
echo "🧱 Building Concurrency..."
mkdir -p "${BUILD_SUPPORT_DIR}/Concurrency"
cd "${BUILD_SUPPORT_DIR}/Concurrency"

cmake "${RTOSIM_DIR}/Dependencies/Concurrency" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CONCURRENCY_INSTALL}"

make -j"$(nproc_safe)"
make install
echo "[ok] Concurrency installed at ${CONCURRENCY_INSTALL}"
echo

# ============================================================
# 2) Build Filter
# ============================================================
echo "🧩 Building Filter..."
mkdir -p "${BUILD_SUPPORT_DIR}/Filter"
cd "${BUILD_SUPPORT_DIR}/Filter"

cmake "${RTOSIM_DIR}/Dependencies/Filter" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${FILTER_INSTALL}" \
  -DCMAKE_PREFIX_PATH="${CONCURRENCY_INSTALL}"

make -j"$(nproc_safe)"
make install
echo "[ok] Filter installed at ${FILTER_INSTALL}"
echo

# ============================================================
# 3) Build RTOSIM
# ============================================================
echo "🚀 Building RTOSIM..."
cd "${RTOSIM_DIR}"
rm -rf build install
mkdir -p build && cd build

# ---- Compile-time hints (kept, but avoid LD_LIBRARY_PATH pollution) ----
INCLUDE_DIRS="${SIMBODY_PREFIX}/include:${SIMBODY_PREFIX}/include/simbody:${OPENSIM_PREFIX}/include"
LIB_DIRS="${SIMBODY_PREFIX}/lib:${OPENSIM_PREFIX}/lib"
export CPLUS_INCLUDE_PATH="${INCLUDE_DIRS}:${CPLUS_INCLUDE_PATH:-}"
export LIBRARY_PATH="${LIB_DIRS}:${LIBRARY_PATH:-}"

# ---- CMake prefix path (main discovery mechanism) ----
CMAKE_PREFIX_PATH_LOCAL="${FILTER_INSTALL}:${CONCURRENCY_INSTALL}:${OPENSIM_PREFIX}:${OPENSIM_DEPS_PREFIX}:${SIMBODY_PREFIX}"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH_LOCAL}:${CMAKE_PREFIX_PATH:-}"

CC="${CC}" CXX="${CXX}" cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenSim_DIR="${OPENSIM_PREFIX}/lib/cmake/OpenSim" \
  -DSimbody_DIR="${SIMBODY_PREFIX}/lib/cmake/simbody" \
  -DCMAKE_INSTALL_PREFIX="${RTOSIM_DIR}/install" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_BUILD_RPATH="${SIMBODY_PREFIX}/lib;${OPENSIM_PREFIX}/lib" \
  -DCMAKE_INSTALL_RPATH="${SIMBODY_PREFIX}/lib;${OPENSIM_PREFIX}/lib" \
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON

make -j"$(nproc_safe)"
make install
echo "[ok] RTOSIM installed at ${RTOSIM_DIR}/install"
echo

# ============================================================
# 4) Build my_online_ik
# ============================================================
echo "🎯 Building my_online_ik..."
cd "${ROOT_DIR}"
rm -rf build
mkdir -p build && cd build

CC="${CC}" CXX="${CXX}" cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${RTOSIM_DIR}/install;${CONCURRENCY_INSTALL};${FILTER_INSTALL};${OPENSIM_PREFIX};${OPENSIM_DEPS_PREFIX};${SIMBODY_PREFIX}" \
  -DOpenSim_DIR="${OPENSIM_PREFIX}/lib/cmake/OpenSim" \
  -DSimbody_DIR="${SIMBODY_PREFIX}/lib/cmake/simbody" \
  -DRTOSIM_DIR="${RTOSIM_DIR}/install/lib/cmake/RTOSIM" \
  -Dspdlog_DIR="${SPDLOG_DIR}" \
  -DCMAKE_BUILD_RPATH="${SIMBODY_PREFIX}/lib;${OPENSIM_PREFIX}/lib;${RTOSIM_DIR}/install/lib" \
  -DCMAKE_INSTALL_RPATH="${SIMBODY_PREFIX}/lib;${OPENSIM_PREFIX}/lib;${RTOSIM_DIR}/install/lib" \
  -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON

make -j"$(nproc_safe)"

echo
echo "✅ Build complete — my_online_ik ready 🎉"
echo
echo "[tip] Runtime environment (recommended):"
echo "  export LD_LIBRARY_PATH=\"${SIMBODY_PREFIX}/lib:${OPENSIM_PREFIX}/lib:\$LD_LIBRARY_PATH\""
echo "  ./build/online_ik_test --no-viz data/upperlimb-biorob.osim"
echo

