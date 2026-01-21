#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "osimLepton" for configuration "Release"
set_property(TARGET osimLepton APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimLepton PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosimLepton.so"
  IMPORTED_SONAME_RELEASE "libosimLepton.so"
  )

list(APPEND _cmake_import_check_targets osimLepton )
list(APPEND _cmake_import_check_files_for_osimLepton "${_IMPORT_PREFIX}/lib/libosimLepton.so" )

# Import target "osimCommon" for configuration "Release"
set_property(TARGET osimCommon APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimCommon PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosimCommon.so"
  IMPORTED_SONAME_RELEASE "libosimCommon.so"
  )

list(APPEND _cmake_import_check_targets osimCommon )
list(APPEND _cmake_import_check_files_for_osimCommon "${_IMPORT_PREFIX}/lib/libosimCommon.so" )

# Import target "osimSimulation" for configuration "Release"
set_property(TARGET osimSimulation APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimSimulation PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosimSimulation.so"
  IMPORTED_SONAME_RELEASE "libosimSimulation.so"
  )

list(APPEND _cmake_import_check_targets osimSimulation )
list(APPEND _cmake_import_check_files_for_osimSimulation "${_IMPORT_PREFIX}/lib/libosimSimulation.so" )

# Import target "osimActuators" for configuration "Release"
set_property(TARGET osimActuators APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimActuators PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosimActuators.so"
  IMPORTED_SONAME_RELEASE "libosimActuators.so"
  )

list(APPEND _cmake_import_check_targets osimActuators )
list(APPEND _cmake_import_check_files_for_osimActuators "${_IMPORT_PREFIX}/lib/libosimActuators.so" )

# Import target "osimAnalyses" for configuration "Release"
set_property(TARGET osimAnalyses APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimAnalyses PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosimAnalyses.so"
  IMPORTED_SONAME_RELEASE "libosimAnalyses.so"
  )

list(APPEND _cmake_import_check_targets osimAnalyses )
list(APPEND _cmake_import_check_files_for_osimAnalyses "${_IMPORT_PREFIX}/lib/libosimAnalyses.so" )

# Import target "osimExampleComponents" for configuration "Release"
set_property(TARGET osimExampleComponents APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimExampleComponents PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosimExampleComponents.so"
  IMPORTED_SONAME_RELEASE "libosimExampleComponents.so"
  )

list(APPEND _cmake_import_check_targets osimExampleComponents )
list(APPEND _cmake_import_check_files_for_osimExampleComponents "${_IMPORT_PREFIX}/lib/libosimExampleComponents.so" )

# Import target "osimTools" for configuration "Release"
set_property(TARGET osimTools APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(osimTools PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libosimTools.so"
  IMPORTED_SONAME_RELEASE "libosimTools.so"
  )

list(APPEND _cmake_import_check_targets osimTools )
list(APPEND _cmake_import_check_files_for_osimTools "${_IMPORT_PREFIX}/lib/libosimTools.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
