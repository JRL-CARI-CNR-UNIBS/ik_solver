#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ik_solver::ik_solver" for configuration "Debug"
set_property(TARGET ik_solver::ik_solver APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(ik_solver::ik_solver PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libik_solver.so"
  IMPORTED_SONAME_DEBUG "libik_solver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ik_solver::ik_solver )
list(APPEND _IMPORT_CHECK_FILES_FOR_ik_solver::ik_solver "${_IMPORT_PREFIX}/lib/libik_solver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
