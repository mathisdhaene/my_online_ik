#!/usr/bin/env bash
set -e

# ============================================================
# Build script for my_online_ik + RTOSIM + dependencies
# Author: Mathis D’Haene
# Version: 3 — robust path + include fix edition
# ============================================================

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
RTOSIM_DIR="${ROOT_DIR}/RTOSIM"
BUILD_SUPPORT_DIR="${ROOT_DIR}/build_support"

CC=gcc-11
CXX=g++-11
export CPLUS_INCLUDE_PATH="$HOME/spdlog-1.5.0-install/include:$CPLUS_INCLUDE_PATH"


# --- Default expected install locations ---
OPENSIM_PREFIX="${HOME}/opensim-core-4.3-install"
SIMBODY_PREFIX="${HOME}/soft_builds/simbody-3.7-install"
CONCURRENCY_INSTALL="${HOME}/concurrency-install"
FILTER_INSTALL="${HOME}/filter-install"
SPDLOG_DIR="${HOME}/spdlog-1.5.0-install/lib/cmake/spdlog"

echo "=============================================="
echo "     🧠 Building my_online_ik pipeline"
echo "=============================================="
echo

# ============================================================
# 0. Locate OpenSim & Simbody automatically if needed
# ============================================================

echo "[info] Checking required dependencies..."

if [ ! -f "${OPENSIM_PREFIX}/lib/cmake/OpenSim/OpenSimConfig.cmake" ]; then
    echo "[warn] OpenSim not found in ${OPENSIM_PREFIX}, searching..."
    OPENSIM_PATH=$(find ~ -type f -name "OpenSimConfig.cmake" 2>/dev/null | head -n 1)
    if [ -n "$OPENSIM_PATH" ]; then
        OPENSIM_PREFIX=$(dirname "$OPENSIM_PATH")/../..
        echo "[ok] Found OpenSim at ${OPENSIM_PREFIX}"
    else
        echo "[error] Could not locate OpenSimConfig.cmake"
        exit 1
    fi
else
    echo "[ok] OpenSim found at ${OPENSIM_PREFIX}"
fi

if [ ! -f "${SIMBODY_PREFIX}/lib/cmake/simbody/SimbodyConfig.cmake" ]; then
    echo "[warn] Simbody not found in ${SIMBODY_PREFIX}, searching..."
    SIMBODY_PATH=$(find ~ -type f -name "SimbodyConfig.cmake" 2>/dev/null | head -n 1)
    if [ -n "$SIMBODY_PATH" ]; then
        SIMBODY_PREFIX=$(dirname "$SIMBODY_PATH")/../..
        echo "[ok] Found Simbody at ${SIMBODY_PREFIX}"
    else
        echo "[error] Could not locate SimbodyConfig.cmake"
        exit 1
    fi
else
    echo "[ok] Simbody found at ${SIMBODY_PREFIX}"
fi

# ============================================================
# 1. Build Concurrency
# ============================================================
echo
echo "🧱 Building Concurrency..."
mkdir -p "${BUILD_SUPPORT_DIR}/Concurrency"
cd "${BUILD_SUPPORT_DIR}/Concurrency"

cmake "${RTOSIM_DIR}/Dependencies/Concurrency" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${CONCURRENCY_INSTALL}"

make -j"$(nproc)" && make install
echo "[ok] Concurrency installed at ${CONCURRENCY_INSTALL}"

# ============================================================
# 2. Create fake Filter (stub)
# ============================================================
echo
echo "🧩 Creating fake Filter package..."
mkdir -p "${FILTER_INSTALL}/lib/Filter" "${FILTER_INSTALL}/include/rtb/Filter"
cat > "${FILTER_INSTALL}/lib/Filter/FilterConfig.cmake" <<EOF
# Minimal fake FilterConfig for RTOSIM linking
set(Filter_FOUND TRUE)
add_library(rtb::Filter INTERFACE IMPORTED)
EOF
echo "[ok] Fake Filter ready at ${FILTER_INSTALL}"

# ============================================================
# 3. Build RTOSIM
# ============================================================
echo
echo "🚀 Building RTOSIM..."
cd "${RTOSIM_DIR}"
rm -rf build install
mkdir -p build && cd build

# ---- Explicit include + lib paths ----
INCLUDE_DIRS="${SIMBODY_PREFIX}/include:${SIMBODY_PREFIX}/include/simbody:${OPENSIM_PREFIX}/include:${OPENSIM_PREFIX}/include"
LIB_DIRS="${SIMBODY_PREFIX}/lib:${OPENSIM_PREFIX}/lib"

export CPLUS_INCLUDE_PATH="${INCLUDE_DIRS}:${CPLUS_INCLUDE_PATH}"
export LIBRARY_PATH="${LIB_DIRS}:${LIBRARY_PATH}"

echo "[info] Using include paths:"
echo "  ${INCLUDE_DIRS}"
echo "[info] Using library paths:"
echo "  ${LIB_DIRS}"

CC=${CC} CXX=${CXX} cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenSim_DIR="${OPENSIM_PREFIX}/lib/cmake/OpenSim" \
  -DSimbody_DIR="${SIMBODY_PREFIX}/lib/cmake/simbody" \
  -DConcurrency_DIR="${CONCURRENCY_INSTALL}/lib/Concurrency" \
  -DFilter_DIR="${FILTER_INSTALL}/lib/Filter" \
  -DCMAKE_INSTALL_PREFIX="${RTOSIM_DIR}/install" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

make -j"$(nproc)" && make install
echo "[ok] RTOSIM installed."

# ============================================================
# 4. Build my_online_ik
# ============================================================
echo
echo "🎯 Building my_online_ik..."
cd "${ROOT_DIR}"
rm -rf build
mkdir build && cd build

CC=${CC} CXX=${CXX} cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenSim_DIR="${OPENSIM_PREFIX}/lib/cmake/OpenSim" \
  -DSimbody_DIR="${SIMBODY_PREFIX}/lib/cmake/simbody" \
  -DRTOSIM_DIR="${RTOSIM_DIR}/install/lib/cmake/RTOSIM" \
  -DConcurrency_DIR="${CONCURRENCY_INSTALL}/lib/Concurrency" \
  -DFilter_DIR="${FILTER_INSTALL}/lib/Filter" \
  -Dspdlog_DIR="${HOME}/spdlog-1.5.0-install/lib/cmake/spdlog"

make -j"$(nproc)"
echo
echo "✅ Build complete — my_online_ik ready 🎉"
