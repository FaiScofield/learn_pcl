cmake_minimum_required(VERSION 2.8)

include_directories(${PCL_INCLUDE_DIRS})

## This looks for all examples/XXX/CMakeLists.txt
#file (GLOB examples_sbudirs */CMakeLists.txt)
## Extract only relevant XXX and append it to PCL_EXAMPLES_SUBDIRS
#foreach(subdir ${examples_sbudirs})
#  # PATH of get_filename_component has problem dealing with windows shared folders "//xxx.xxx.xxx.xxx/folder"
#  # Check here for the discussion: http://www.pcl-developers.org/Small-hack-to-get-filename-component-td5706691.html
#  # Modified by Yangyan Li(yangyan.lee@gmail.com)
#  # get_filename_component (dir ${subdir} PATH)
#  get_filename_component (absolute ${subdir} ABSOLUTE)
#  get_filename_component (name ${subdir} NAME)
#  STRING(REGEX REPLACE ${name} "" dir ${absolute})
#  list(APPEND PCL_EXAMPLES_SUBDIRS ${dir})
#endforeach()


foreach(subdir
  common
  features
  filters
  geometry
  keypoints
  outofcore
  segmentation
  stereo
#  surface
  )
  # Add the subdirectories
  add_subdirectory(${subdir})
endforeach()


