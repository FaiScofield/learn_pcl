cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

#project(narf_feature_extraction)

#find_package(PCL 1.3 REQUIRED)

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable (example-narf_feature_extraction narf_feature_extraction.cpp)
target_link_libraries (example-narf_feature_extraction ${PCL_LIBRARIES})
