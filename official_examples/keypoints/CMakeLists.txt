cmake_minimum_required(VERSION 2.8)

## Find VTK
if(NOT VTK_FOUND)
    set(DEFAULT FALSE)
    set(REASON "VTK was not found.")
else()
    set(DEFAULT TRUE)
    set(REASON)
    include(${VTK_USE_FILE})
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/official_examples)

add_executable(pcl_example_sift_keypoint_estimation example_sift_keypoint_estimation.cpp)
add_executable(pcl_example_sift_normal_keypoint_estimation example_sift_normal_keypoint_estimation.cpp)
add_executable(pcl_example_sift_z_keypoint_estimation example_sift_z_keypoint_estimation.cpp)
add_executable(pcl_example_get_keypoints_indices example_get_keypoints_indices.cpp)

target_link_libraries (pcl_example_sift_keypoint_estimation ${PCL_LIBRARIES})
target_link_libraries (pcl_example_sift_normal_keypoint_estimation ${PCL_LIBRARIES})
target_link_libraries (pcl_example_sift_z_keypoint_estimation ${PCL_LIBRARIES})
target_link_libraries (pcl_example_get_keypoints_indices ${PCL_LIBRARIES})
