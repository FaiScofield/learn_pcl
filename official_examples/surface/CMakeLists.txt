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

#add_executable(pcl_test_nurbs_fitting_surface
#                test_nurbs_fitting_surface.cpp)

#add_executable(pcl_example_nurbs_fitting_surface
#                example_nurbs_fitting_surface.cpp)

#add_executable(pcl_example_nurbs_viewer_surface
#                example_nurbs_viewer_surface.cpp )

#add_executable(pcl_example_nurbs_fitting_closed_curve
#                example_nurbs_fitting_closed_curve.cpp)

#add_executable(pcl_example_nurbs_fitting_curve2d
#                example_nurbs_fitting_curve2d.cpp)

#add_executable(pcl_example_nurbs_fitting_closed_curve3d
#                example_nurbs_fitting_closed_curve3d.cpp)

#target_link_libraries (pcl_test_nurbs_fitting_surface ${PCL_LIBRARIES})
#target_link_libraries (pcl_example_nurbs_fitting_surface ${PCL_LIBRARIES})
#target_link_libraries (pcl_example_nurbs_viewer_surface ${PCL_LIBRARIES})
#target_link_libraries (pcl_example_nurbs_fitting_closed_curve ${PCL_LIBRARIES})
#target_link_libraries (pcl_example_nurbs_fitting_curve2d ${PCL_LIBRARIES})
#target_link_libraries (pcl_example_nurbs_fitting_closed_curve3d ${PCL_LIBRARIES})
