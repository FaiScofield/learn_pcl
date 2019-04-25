cmake_minimum_required(VERSION 2.8)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/official_examples)

add_executable(pcl_example_half_edge_mesh example_half_edge_mesh.cpp)
target_link_libraries (pcl_example_half_edge_mesh ${PCL_LIBRARIES})
