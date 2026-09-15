#----------------------------------------------------------------
# Generated CMake target import file for configuration "RELEASE".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ndt_omp::ndt_omp" for configuration "RELEASE"
set_property(TARGET ndt_omp::ndt_omp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ndt_omp::ndt_omp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libndt_omp.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libndt_omp.dylib"
  )

list(APPEND _cmake_import_check_targets ndt_omp::ndt_omp )
list(APPEND _cmake_import_check_files_for_ndt_omp::ndt_omp "${_IMPORT_PREFIX}/lib/libndt_omp.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
