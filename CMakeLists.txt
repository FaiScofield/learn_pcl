cmake_minimum_required(VERSION 2.8)

project(learn_pcl)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(PCL  REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED )


include_directories(
#    ${PROJECT_SOURCE_DIR}/include
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
)

subdirs(src)
subdirs(official_examples)
subdirs(official_tutorials)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

#add_executable(bin2pcd src/bin2pcd.cpp)
#target_link_libraries (bin2pcd ${PCL_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(PFH src/PFH.cpp)
#target_link_libraries(PFH ${PCL_LIBRARIES})

#add_executable(visualizer src/visualization/visualizer.cpp)
#target_link_libraries(visualizer ${PCL_LIBRARIES})
