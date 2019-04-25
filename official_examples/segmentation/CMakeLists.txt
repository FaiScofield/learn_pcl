cmake_minimum_required(VERSION 2.8)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/official_examples)

add_executable(pcl_example_extract_clusters_normals example_extract_clusters_normals.cpp)
add_executable(pcl_example_region_growing example_region_growing.cpp)

target_link_libraries (pcl_example_extract_clusters_normals ${PCL_LIBRARIES})
target_link_libraries (pcl_example_region_growing ${PCL_LIBRARIES})

## Find VTK
if(VTK_FOUND)
  include(${VTK_USE_FILE})
  add_executable(pcl_example_supervoxels example_supervoxels.cpp)
  add_executable(pcl_example_lccp_segmentation example_lccp_segmentation.cpp)
  add_executable(pcl_example_cpc_segmentation example_cpc_segmentation.cpp)

  target_link_libraries (pcl_example_supervoxels ${PCL_LIBRARIES})
  target_link_libraries (pcl_example_lccp_segmentation ${PCL_LIBRARIES})
  target_link_libraries (pcl_example_cpc_segmentation ${PCL_LIBRARIES})
endif()
