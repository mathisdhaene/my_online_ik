# For an example of how to make your own CMake project that uses OpenSim,
# see SampleCMakeLists.txt.
#
# This config file defines the following variables:
#   OpenSim_FOUND
#       True if search for libraries and headers succeeds.
#   OpenSim_ROOT_DIR
#       The installation directory.
#   OpenSim_USE_FILE
#       A file containing CMake macros and functions that may be useful for 
#       client projects. 
#   OpenSim_BIN_DIR
#       The directory containing executables (and DLLs on Windows).
#   OpenSim_LIB_DIR
#       The directory containing libraries. Suitable for link_directories().
#   OpenSim_INCLUDE_DIRS
#       Suitable for include_directories().
#   OpenSim_LIBRARIES
#       Suitable for target_link_libraries().
#   OpenSim_JAVA_WRAPPING
#       Boolean indicating whether or not OpenSim was built to include Java
#       wrapping.
#   OpenSim_JAR_FILE
#       Path to Java wrapping's jar file.
#   OpenSim_JAVA_FILES_DIR
#       Directory containing the Java source files.
#   OpenSim_PYTHON_WRAPPING
#       Boolean indicating whether or not OpenSim was built to include Python
#       wrapping.
#   OpenSim_PYTHON_PACKAGE_DIR
#       Directory containing the OpenSim Python package.
#                                            
# This file will also find Simbody; you do not
# need to use `find_package(Simbody)` in your own project. If OpenSim's
# installation does *not* contain Simbody (and ezc3d or BTK), you might need to set
# CMAKE_PREFIX_PATH to the directory containing Simbody, and make sure that ezc3d or BTK
# libraries are available during runtime.
#
# Adapted from SimbodyConfig.cmake
#

# To make the OpenSim installation relocatable:

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was OpenSimConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

set_and_check(OpenSim_ROOT_DIR "${PACKAGE_PREFIX_DIR}")
        
set_and_check(OpenSim_USE_FILE
    "${PACKAGE_PREFIX_DIR}/lib/cmake/OpenSim/UseOpenSim.cmake")

# BIN_DIR
# -------
set_and_check(OpenSim_BIN_DIR "${PACKAGE_PREFIX_DIR}/bin")

# LIB_DIR
# -------
set_and_check(OpenSim_LIB_DIR "${PACKAGE_PREFIX_DIR}/lib")

# INCLUDE_DIRS
# ------------
set_and_check(OpenSim_INCLUDE_DIRS
    "${PACKAGE_PREFIX_DIR}/include")


# LIBRARIES
# ---------
include("${PACKAGE_PREFIX_DIR}/lib/cmake/OpenSim/OpenSimTargets.cmake")

# The osimTools target uses all the other targets.
# This makes a distinction between release and debug libraries.
list(APPEND OpenSim_LIBRARIES osimTools osimExampleComponents osimMoco)

check_required_components(OpenSim)

# Python/Java Wrapping
#---------------------
set(OpenSim_PYTHON_WRAPPING OFF)
set(OpenSim_PYTHON_PACKAGE_DIR
    ${PACKAGE_PREFIX_DIR}/lib/pythonVERSION/site-packages)

set(OpenSim_JAVA_WRAPPING OFF)
set(OpenSim_JAVA_FILES_DIR 
    ${PACKAGE_PREFIX_DIR}/share/OpenSim/java/org)
set(OpenSim_JAR_FILE 
    ${PACKAGE_PREFIX_DIR}/share/java/)

# Dependencies
# ------------
if (ON) # OPENSIM_COPY_DEPENDENCIES
    # Find the copies of Simbody and spdlog within the OpenSim installation.
    # We define _SIMBODY_PATH and _SPDLOG_PATH before find_package(Simbody)
    # because finding Simbody redefines the PACKAGE_PREFIX_DIR variable
    # that appears in this file after configuring, thereby incorrectly
    # causing CMake to look for spdlog within Simbody's installation.
    set(_SIMBODY_PATH "${PACKAGE_PREFIX_DIR}/.")
    set(_SPDLOG_PATH "${PACKAGE_PREFIX_DIR}/.")
    find_package(Simbody 3.7 REQUIRED
        PATHS "${_SIMBODY_PATH}" NO_MODULE NO_DEFAULT_PATH)
    find_package(spdlog REQUIRED
        PATHS "${_SPDLOG_PATH}" NO_MODULE NO_DEFAULT_PATH)
else()
    # Find the correct version anywhere on the machine.
    include(CMakeFindDependencyMacro)
    find_dependency(Simbody 3.7)
    find_dependency(spdlog)
endif()


