#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Build script for my_online_ik + RTOSIM + dependencies
# Author: Mathis D’Haene
# Version: 4 — OpenSim superbuild-compatible (OpenSim deps + Simbody)
# ============================================================

# ----------------------------
# Project layout
# ----------------------------
ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
RTOSIM_DIR="${ROOT_DIR}/RTOSIM"
BUILD_SUPPORT_DIR="${ROOT_DIR}/build_support"

# ----------------------------
# Compilers
# ----------------------------
CC=${CC:-gcc-11}
CXX=${CXX:-g++-11}

# ----------------------------
# Optional: spdlog include path (header-only usage sometimes)
# ----------------------------
export CPLUS_INCLUDE_PATH="$HOME/spdlog-1.5.0-install/include:${CPLUS_INCLUDE_PATH:-}"

# ----------------------------
# Default expected install locations (OpenSim superbuild)
# Adjust here if you use different folders.
# ----------------------------
OPENSIM_PREFIX="${OPENSIM_PREFIX:-$HOME/opensim-core-install}"
OPENSIM_DEPS_PREFIX="${OPENSIM_DEPS_PREFIX:-$HOME/opensim_dependencies_install}"
SIMBODY_PREFIX="${SIMBODY_PREFIX:-$OPENSIM_DEPS_PREFIX/simbody}"

CONCURRENCY_INSTALL="${CONCURRENCY_INSTALL:-$HOME/concurrency-install}"
FILTER_INSTALL="${FILTER_INSTALL:-$HOME/filter-install}"
SPDLOG_DIR="${SPDLOG_DIR:-$HOME/spdlog-1.5.0-install/lib/cmake/spdlog}"

# ----------------------------
# Pretty banner
# ----------------------------
echo "=============================================="
echo "     🧠 Building my_online_ik pipeline"
echo "=============================================="
echo
echo "[info] ROOT_DIR            = ${ROOT_DIR}"
echo "[info] RTOSIM_DIR          = ${RTOSIM_DIR}"
echo "[info] BUILD_SUPPORT_DIR   = ${BUILD_SUPPORT_DIR}"
echo "[info] CC / CXX            = ${CC} / ${CXX}"
echo

# ============================================================
# 0. Locate OpenSim & Simbody automatically if needed
# ============================================================
echo "[info] Checking required dependencies..."

# ---- OpenSim ----
if [ ! -f "${OPENSIM_PREFIX}/lib/cmake/OpenSim/OpenSimConfig.cmake" ]; then
  echo "[warn] OpenSim not found in ${OPENSIM_PREFIX}, searching..."
  OPENSIM_PATH="$(find ~ -type f -name "OpenSimConfig.cmake" 2>/dev/null | head -n 1 || true)"
  if [ -n "${OPENSIM_PATH}" ]; then
    OPENSIM_PREFIX="$(dirname "${OPENSIM_PATH}")/../.."
    echo "[ok] Found OpenSim at ${OPENSIM_PREFIX}"
  else
    echo "[error] Could not locate OpenSimConfig.cmake"
    exit 1
  fi
else
  echo "[ok] OpenSim found at ${OPENSIM_PREFIX}"
fi

# ---- Simbody (prefer superbuild deps, but search if not found) ----
if [ ! -f "${SIMBODY_PREFIX}/lib/cmake/simbody/SimbodyConfig.cmake" ]; then
  echo "[warn] Simbody not found in ${SIMBODY_PREFIX}, searching..."
  SIMBODY_PATH="$(find ~ -type f -name "SimbodyConfig.cmake" 2>/dev/null | head -n 1 || true)"
  if [ -n "${SIMBODY_PATH}" ]; then
    SIMBODY_PREFIX="$(dirname "${SIMBODY_PATH}")/../.."
    echo "[ok] Found Simbody at ${SIMBODY_PREFIX}"
  else
    echo "[error] Could not locate SimbodyConfig.cmake"
    exit 1
  fi
else
  echo "[ok] Simbody found at ${SIMBODY_PREFIX}"
fi

# If deps prefix seems wrong but Simbody was found, try to infer deps root
if [ ! -d "${OPENSIM_DEPS_PREFIX}" ]; then
  # Best-effort inference: .../opensim_dependencies_install from .../opensim_dependencies_install/simbody
  if [[ "${SIMBODY_PREFIX}" == *"/simbody" ]]; then
    OPENSIM_DEPS_PREFIX="${SIMBODY_PREFIX%/simbody}"
    echo "[info] Inferred OPENSIM_DEPS_PREFIX = ${OPENSIM_DEPS_PREFIX}"
  fi
fi

echo
echo "[info] Using:"
echo "  OPENSIM_PREFIX       = ${OPENSIM_PREFIX}"
echo "  OPENSIM_DEPS_PREFIX  = ${OPENSIM_DEPS_PREFIX}"
echo "  SIMBODY_PREFIX       = ${SIMBODY_PREFIX}"
echo "  CONCURRENCY_INSTALL  = ${CONCURRENCY_INSTALL}"
echo "  FILTER_INSTALL       = ${FILTER_INSTALL}"
echo "  SPDLOG_DIR           = ${SPDLOG_DIR}"
echo

# ============================================================
# Helpers
# ============================================================
nproc_safe() {
  if command -v nproc >/dev/null 2>&1; then nproc; else echo 4; fi
}

# ============================================================
# 1. Build Concurrency
# ============================================================
echo "🧱 Building Concurrency..."
mkdir -p "${BUILD_SUPPORT_DIR}/Concurrency"
cd "${BUILD_SUPPORT_DIR}/Concurrency"

cmake "${RTOSIM_DIR}/Dependencies/Concurrency" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CONCURRENCY_INSTALL}"

make -j"$(nproc_safe)" && make install
echo "[ok] Concurrency installed at ${CONCURRENCY_INSTALL}"

# ============================================================
# 2. Build Filter
# ============================================================
echo
echo "🧩 Building Filter..."
mkdir -p "${BUILD_SUPPORT_DIR}/Filter"
cd "${BUILD_SUPPORT_DIR}/Filter"

cmake "${RTOSIM_DIR}/Dependencies/Filter" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${FILTER_INSTALL}" \
  -DCMAKE_PREFIX_PATH="${CONCURRENCY_INSTALL}"

make -j"$(nproc_safe)" && make install
echo "[ok] Filter installed at ${FILTER_INSTALL}"

# ============================================================
# 3. Build RTOSIM
# ============================================================
echo
echo "🚀 Building RTOSIM..."
cd "${RTOSIM_DIR}"
rm -rf build install
mkdir -p build && cd build

# ---- Explicit include + lib paths (compile-time) ----
INCLUDE_DIRS="${SIMBODY_PREFIX}/include:${SIMBODY_PREFIX}/include/simbody:${OPENSIM_PREFIX}/include"
LIB_DIRS="${SIMBODY_PREFIX}/lib:${OPENSIM_PREFIX}/lib"

export CPLUS_INCLUDE_PATH="${INCLUDE_DIRS}:${CPLUS_INCLUDE_PATH:-}"
export LIBRARY_PATH="${LIB_DIRS}:${LIBRARY_PATH:-}"

# ---- Runtime libs (important for running binaries that link OpenSim/Simbody) ----
export LD_LIBRARY_PATH="${OPENSIM_PREFIX}/lib:${SIMBODY_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

echo "[info] Using include paths:"
echo "  ${INCLUDE_DIRS}"
echo "[info] Using library paths:"
echo "  ${LIB_DIRS}"
echo "[info] Using runtime LD_LIBRARY_PATH additions:"
echo "  ${OPENSIM_PREFIX}/lib"
echo "  ${SIMBODY_PREFIX}/lib"
echo

# ---- CMake prefix path ----
export CMAKE_PREFIX_PATH="${FILTER_INSTALL}:${CONCURRENCY_INSTALL}:${OPENSIM_PREFIX}:${OPENSIM_DEPS_PREFIX}:${SIMBODY_PREFIX}:${CMAKE_PREFIX_PATH:-}"

CC="${CC}" CXX="${CXX}" cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenSim_DIR="${OPENSIM_PREFIX}/lib/cmake/OpenSim" \
  -DSimbody_DIR="${SIMBODY_PREFIX}/lib/cmake/simbody" \
  -DCMAKE_INSTALL_PREFIX="${RTOSIM_DIR}/install" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

make -j"$(nproc_safe)" && make install
echo "[ok] RTOSIM installed at ${RTOSIM_DIR}/install"

# ============================================================
# 4. Build my_online_ik
# ============================================================
echo
echo "🎯 Building my_online_ik..."
cd "${ROOT_DIR}"
rm -rf build
mkdir build && cd build

# Make sure runtime env still has OpenSim/Simbody for post-build runs
export LD_LIBRARY_PATH="${OPENSIM_PREFIX}/lib:${SIMBODY_PREFIX}/lib:${LD_LIBRARY_PATH:-}"

CC="${CC}" CXX="${CXX}" cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${RTOSIM_DIR}/install;${CONCURRENCY_INSTALL};${FILTER_INSTALL};${OPENSIM_PREFIX};${OPENSIM_DEPS_PREFIX};${SIMBODY_PREFIX}" \
  -DOpenSim_DIR="${OPENSIM_PREFIX}/lib/cmake/OpenSim" \
  -DSimbody_DIR="${SIMBODY_PREFIX}/lib/cmake/simbody" \
  -DRTOSIM_DIR="${RTOSIM_DIR}/install/lib/cmake/RTOSIM" \
  -Dspdlog_DIR="${SPDLOG_DIR}"

make -j"$(nproc_safe)"

echo
echo "✅ Build complete — my_online_ik ready 🎉"
echo
echo "[tip] For runtime (if needed):"
echo "  export LD_LIBRARY_PATH=\"${OPENSIM_PREFIX}/lib:${SIMBODY_PREFIX}/lib:\$LD_LIBRARY_PATH\""
echo

