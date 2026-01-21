cmake_minimum_required(VERSION 3.18)
project(my_online_ik LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenSim 4.3 REQUIRED)
find_package(Simbody REQUIRED)
find_package(Concurrency CONFIG REQUIRED)
find_package(Filter REQUIRED)
find_package(RTOSIM REQUIRED)   # ← IMPORTANT

add_executable(online_ik_test main.cpp)

target_link_libraries(online_ik_test
    PRIVATE
        rtb::Common
        rtb::Utilities
        rtb::InverseKinematics
        rtb::InverseDynamics
        rtb::SignalProcessing
        rtb::InputDevice
        rtb::OutputDevice

        rtb::Concurrency

        osimSimulation
        osimCommon
        osimTools
        osimAnalyses
        osimActuators

        SimTKsimbody
        SimTKmath
        SimTKcommon

        pthread
)
