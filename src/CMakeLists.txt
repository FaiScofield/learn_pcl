cmake_minimum_required(VERSION 2.8)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/src)

add_executable(bin2pcd bin2pcd.cpp)
target_link_libraries (bin2pcd ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(PFH PFH.cpp)
target_link_libraries(PFH ${PCL_LIBRARIES})



