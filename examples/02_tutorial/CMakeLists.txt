cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

#project      (pcl-visualizer)
find_package (Qt  REQUIRED)
find_package (VTK REQUIRED)


set(project_SOURCES main.cpp pclviewer.cpp)
set(project_HEADERS pclviewer.h)
set(project_FORMS   pclviewer.ui)
set(VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

#QT_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
#QT_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

#INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE(example-pcl_visualizer
    ${project_SOURCES}
#    ${project_FORMS_HEADERS}
    ${project_FORMS}
    ${project_HEADERS}
)

TARGET_LINK_LIBRARIES(example-pcl_visualizer
    ${QT_LIBRARIES}
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
)

