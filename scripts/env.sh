#!/usr/bin/env bash
# Usage:
#   source scripts/env.sh
#   source scripts/env.sh --print
#   OPENSIM_PREFIX=... OPENSIM_DEPS_PREFIX=... source scripts/env.sh
#
# This file only sets environment variables for the CURRENT shell.
# It is meant to be sourced, not executed.

__env_old_opts="$(set +o)"
set +e
set +u
set +o pipefail

# ----------------------------
# User-configurable prefixes
# ----------------------------
export OPENSIM_PREFIX="${OPENSIM_PREFIX:-$HOME/opensim-core-install}"
export OPENSIM_DEPS_PREFIX="${OPENSIM_DEPS_PREFIX:-$HOME/opensim_dependencies_install}"
export SIMBODY_PREFIX="${SIMBODY_PREFIX:-$OPENSIM_DEPS_PREFIX/simbody}"

# Optional (if you installed SWIG locally)
export SWIG_HOME="${SWIG_HOME:-$HOME/swig}"

# Optional (if you installed spdlog locally)
export SPDLOG_PREFIX="${SPDLOG_PREFIX:-$HOME/spdlog-1.5.0-install}"

# ----------------------------
# Sanity checks (soft)
# ----------------------------
warn() { echo "[env.sh][warn] $*" >&2; }

if [ ! -d "$OPENSIM_PREFIX" ]; then
  warn "OPENSIM_PREFIX not found: $OPENSIM_PREFIX"
fi
if [ ! -d "$SIMBODY_PREFIX" ]; then
  warn "SIMBODY_PREFIX not found: $SIMBODY_PREFIX"
fi

# ----------------------------
# Core variables
# ----------------------------
export OPENSIM_HOME="$OPENSIM_PREFIX"
export OPENSIM_DEPS="$OPENSIM_DEPS_PREFIX"
export SIMBODY_HOME="$SIMBODY_PREFIX"

# CMake discovery helpers
# (useful for manual cmake configure)
export OpenSim_DIR="${OpenSim_DIR:-$OPENSIM_PREFIX/lib/cmake/OpenSim}"
export Simbody_DIR="${Simbody_DIR:-$SIMBODY_PREFIX/lib/cmake/simbody}"

# ----------------------------
# Runtime: shared libraries
# ----------------------------
# Put OpenSim FIRST so its libSimTK*.so are preferred when present there.
# Put Simbody next so simbody-visualizer can find libSimTKcommon, etc.
export LD_LIBRARY_PATH="$OPENSIM_PREFIX/lib:$SIMBODY_PREFIX/lib:${LD_LIBRARY_PATH:-}"

# ----------------------------
# Runtime: executables
# ----------------------------
export PATH="$OPENSIM_PREFIX/bin:${PATH:-}"

# ----------------------------
# OpenSim plugins (usually not needed, but safe)
# ----------------------------
# If you ever use custom OpenSim plugins, they are often discovered via OPENSIM_PLUGIN_PATH
export OPENSIM_PLUGIN_PATH="${OPENSIM_PLUGIN_PATH:-}"

# ----------------------------
# Optional: Python bindings convenience
# ----------------------------
# If you copied Python bindings into:
#   $OPENSIM_PREFIX/sdk/Python
# then this enables `python3 -c "import opensim"`
if [ -d "$OPENSIM_PREFIX/sdk/Python" ]; then
  export PYTHONPATH="$OPENSIM_PREFIX/sdk/Python:${PYTHONPATH:-}"
fi

# ----------------------------
# Optional: spdlog CMake package (if needed by consumers)
# ----------------------------
if [ -d "$SPDLOG_PREFIX" ]; then
  export CMAKE_PREFIX_PATH="$SPDLOG_PREFIX:${CMAKE_PREFIX_PATH:-}"
fi

# ----------------------------
# Print mode
# ----------------------------
if [[ "${1:-}" == "--print" ]]; then
  cat <<EOF
OPENSIM_PREFIX      = $OPENSIM_PREFIX
OPENSIM_DEPS_PREFIX = $OPENSIM_DEPS_PREFIX
SIMBODY_PREFIX      = $SIMBODY_PREFIX
OpenSim_DIR         = $OpenSim_DIR
Simbody_DIR         = $Simbody_DIR
PATH (head)         = $(echo "$PATH" | tr ':' '\n' | head -n 5 | tr '\n' ':' )
LD_LIBRARY_PATH     = $LD_LIBRARY_PATH
PYTHONPATH          = ${PYTHONPATH:-}
EOF
fi
# ---- Restore caller shell options ----
eval "$__ENV_OLD_OPTS"
unset __ENV_OLD_OPTS

