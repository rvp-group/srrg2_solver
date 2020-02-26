cmake_minimum_required(VERSION 2.8.3)
project(srrg2_solver_extras)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  srrg2_core
  srrg2_solver
  srrg_cmake_modules
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia find system stuff
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)

## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS src
 LIBRARIES srrg2_solver_extras_types2d_library srrg2_solver_extras_types3d_library
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${CHOLMOD_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(src)
add_subdirectory(tests)
