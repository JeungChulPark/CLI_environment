#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "hdl_graph_slam::hdl_graph_slam__rosidl_generator_c" for configuration "Release"
set_property(TARGET hdl_graph_slam::hdl_graph_slam__rosidl_generator_c APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(hdl_graph_slam::hdl_graph_slam__rosidl_generator_c PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libhdl_graph_slam__rosidl_generator_c.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libhdl_graph_slam__rosidl_generator_c.dylib"
  )

list(APPEND _cmake_import_check_targets hdl_graph_slam::hdl_graph_slam__rosidl_generator_c )
list(APPEND _cmake_import_check_files_for_hdl_graph_slam::hdl_graph_slam__rosidl_generator_c "${_IMPORT_PREFIX}/lib/libhdl_graph_slam__rosidl_generator_c.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
