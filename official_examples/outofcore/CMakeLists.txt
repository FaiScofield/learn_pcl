cmake_minimum_required(VERSION 2.8)

# Find VTK
if(NOT VTK_FOUND)
  set(DEFAULT FALSE)
  set(REASON "VTK was not found.")
else()
  set(DEFAULT TRUE)
  set(REASON)
  include(${VTK_USE_FILE})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/official_examples)

add_executable (pcl_example_outofcore_with_lod example_outofcore_with_lod.cpp)
add_executable (pcl_example_outofcore example_outofcore.cpp)
target_link_libraries (pcl_example_outofcore_with_lod ${PCL_LIBRARIES})
target_link_libraries (pcl_example_outofcore ${PCL_LIBRARIES})
