cmake_minimum_required(VERSION 2.8.3)
project(srrg2_solver_gui)

#ds help IDEs - uncomment on your machine if needed
#set(CMAKE_VERBOSE_MAKEFILE ON)

#ia find catkin packages
find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  srrg2_solver
  srrg2_qgl_viewport
  )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia find basic packages
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)
find_package(QGLViewer REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${CHOLMOD_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${QGLViewer_INCLUDE_DIR}
  ${SRRG_QT_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
)

#ds help the catkin tool on 16.04 (cmake seems unable to find single libraries, although catkin claims the link_directories call is not required)
#ds in order to avoid linking against the catkin_LIBRARIES bulk everytime enable this so one can select single libraries
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(src)
