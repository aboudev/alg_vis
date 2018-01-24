set(PRJ_NAME alg_vis)

project(${PRJ_NAME})

cmake_minimum_required(VERSION 3.1)
if(NOT POLICY CMP0070 AND POLICY CMP0053)
  # Only set CMP0053 to OLD with CMake<3.10, otherwise there is a warning.
  cmake_policy(SET CMP0053 OLD)
endif()
if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# Find CGAL and CGAL Qt5
find_package(CGAL COMPONENTS Qt5)
include( ${CGAL_USE_FILE} )

# Find Qt5 itself
find_package(Qt5 QUIET COMPONENTS Xml Script OpenGL)

# Find OpenGL
find_package(OpenGL)

# Find QGLViewer
if(Qt5_FOUND)
  find_package(QGLViewer)
endif(Qt5_FOUND)

# Find Eigen for eidge detection
find_package(Eigen3 3 REQUIRED)
if (EIGEN3_FOUND)
  include( ${EIGEN3_USE_FILE} )
endif()

if(CGAL_Qt5_FOUND AND Qt5_FOUND AND OPENGL_FOUND AND QGLVIEWER_FOUND)

  # Include this package's headers first
  include_directories(BEFORE ../../include ./)

  include_directories (${QGLVIEWER_INCLUDE_DIR})

  # ui files, created with Qt Designer
  qt5_wrap_ui( UI_FILES Mainwindow.ui Settings_dialog.ui)

  # qrc files (resources files, that contain icons, at least)
  qt5_add_resources (CGAL_Qt5_RESOURCE_FILES ${PRJ_NAME}.qrc)

  # cpp files
  add_executable(${PRJ_NAME}
    Color_256.cpp
    Shape_detection.cpp
    Ridge_detection.cpp
    Surface_simplification.cpp
    Scene.cpp
    Viewer.cpp
    Settings_dialog.cpp
    Mainwindow.cpp
    ${PRJ_NAME}.cpp
    ${UI_FILES} ${CGAL_Qt5_RESOURCE_FILES} ${CGAL_Qt5_MOC_FILES})

  qt5_use_modules(${PRJ_NAME} Xml Script OpenGL)

  # Link with Qt libraries
  target_link_libraries(${PRJ_NAME} ${QT_LIBRARIES})

  # Link with CGAL
  target_link_libraries(${PRJ_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

  # Link with libQGLViewer, OpenGL
  target_link_libraries(${PRJ_NAME} ${QGLVIEWER_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})

  add_to_cached_list(CGAL_EXECUTABLE_TARGETS ${PRJ_NAME})

else (CGAL_Qt5_FOUND AND Qt5_FOUND AND OPENGL_FOUND AND QGLVIEWER_FOUND)

  set(DEMO_MISSING_DEPS "")

  if(NOT CGAL_Qt5_FOUND)
    set(DEMO_MISSING_DEPS "the CGAL Qt5 library, ${DEMO_MISSING_DEPS}")
  endif()

  if(NOT Qt5_FOUND)
    set(DEMO_MISSING_DEPS "Qt5, ${DEMO_MISSING_DEPS}")
  endif()

  if(NOT OPENGL_FOUND)
    set(DEMO_MISSING_DEPS "OpenGL, ${DEMO_MISSING_DEPS}")
  endif()

  if(NOT QGLVIEWER_FOUND)
    set(DEMO_MISSING_DEPS "QGLViewer, ${DEMO_MISSING_DEPS}")
  endif()

  message(STATUS "NOTICE: This demo requires ${DEMO_MISSING_DEPS} and will not be compiled.")

endif (CGAL_Qt5_FOUND AND Qt5_FOUND AND OPENGL_FOUND AND QGLVIEWER_FOUND)
