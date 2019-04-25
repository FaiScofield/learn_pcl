cmake_minimum_required(VERSION 2.8)

project(learn_pcl)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(PCL_DIR "/home/softwares/pcl-1.9.1/install")
find_package(PCL REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED )

#message("====================================================================================")
#message("PCL Headers: ${PCL_INCLUDE_DIRS}")
#message("PCL LIBS DIR: ${PCL_LIBRARY_DIRS}")
#message("PCL LIBS: ${PCL_LIBRARIES}")

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
#subdirs(official_examples)
subdirs(official_tutorials)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(pcd2csv src/pcd2csv.cpp)
target_link_libraries(pcd2csv ${PCL_LIBRARY_DIRS})
