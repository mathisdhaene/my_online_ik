cmake_minimum_required(VERSION 3.10)
project(my_online_ik CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------
# 1. Paths to dependencies
# ---------------------------------------------------------------
set(OpenSim_DIR "$ENV{HOME}/opensim-core-install/sdk/cmake")
set(Simbody_DIR "$ENV{HOME}/simbody-install/share/cmake/Simbody")
set(Concurrency_DIR "$ENV{HOME}/concurrency-install/lib/Concurrency")

# Internal RTOSIM (subfolder)
set(RTOSIM_SOURCE_DIR "${CMAKE_SOURCE_DIR}/RTOSIM")
set(RTOSIM_BUILD_DIR "${RTOSIM_SOURCE_DIR}/build")
set(RTOSIM_INSTALL_DIR "${RTOSIM_SOURCE_DIR}/install")
set(RTOSIM_DIR "${RTOSIM_INSTALL_DIR}/lib/cmake/RTOSIM")

# ---------------------------------------------------------------
# 2. Include directories
# ---------------------------------------------------------------
include_directories(
    ${CMAKE_SOURCE_DIR}/RTOSIM/lib/Common
    ${CMAKE_SOURCE_DIR}/RTOSIM/lib/InverseKinematics
    ${CMAKE_SOURCE_DIR}/RTOSIM/lib/Utilities
    ${CMAKE_SOURCE_DIR}/RTOSIM/Dependencies/Concurrency/include
    $ENV{HOME}/concurrency-install/include
    $ENV{HOME}/opensim-core-install/include
    $ENV{HOME}/opensim-core-install/include/simbody
)

link_directories(
    ${CMAKE_SOURCE_DIR}/RTOSIM/build/lib
    $ENV{HOME}/opensim-core-install/lib
    $ENV{HOME}/RTOSIM/build/lib
    $ENV{HOME}/concurrency-install/lib
)

# ---------------------------------------------------------------
# 3. Build executable
# ---------------------------------------------------------------
add_executable(online_ik_test main.cpp)

# --- Find spdlog (header-only or compiled) ---
if(NOT DEFINED spdlog_DIR AND EXISTS "$ENV{HOME}/spdlog-1.5.0-install/lib/cmake/spdlog")
    set(spdlog_DIR "$ENV{HOME}/spdlog-1.5.0-install/lib/cmake/spdlog")
endif()

find_package(spdlog QUIET)

if(spdlog_FOUND)
    message(STATUS "Found spdlog via CMake package: ${spdlog_DIR}")
else()
    # Fallback: look for a static/shared library manually
    find_library(SPDLOG_LIB spdlog
        PATHS
            ${OPENSIM_PREFIX}/lib
            ${CMAKE_PREFIX_PATH}/../lib
            $ENV{HOME}/opensim-core-4.3-install/lib
            $ENV{HOME}/spdlog-1.5.0-install/lib
        NO_DEFAULT_PATH
    )
    if(SPDLOG_LIB)
        message(STATUS "Found spdlog library: ${SPDLOG_LIB}")
    else()
        message(FATAL_ERROR "❌ spdlog not found — please set spdlog_DIR or OPENSIM_PREFIX")
    endif()
endif()



# --- Link everything together ---
target_link_libraries(online_ik_test
    Concurrency
    InverseKinematics
    Common
    Utilities

    osimTools
    osimAnalyses
    osimActuators
    osimSimulation
    osimCommon

    SimTKsimbody
    SimTKmath
    SimTKcommon

    pthread
    ${SPDLOG_LIB}     # <-- ensure it is the last one!
)
