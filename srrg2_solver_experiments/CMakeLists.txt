cmake_minimum_required(VERSION 2.8.3)
project(srrg2_solver_experiments)

find_package(catkin REQUIRED COMPONENTS
  srrg2_core
  srrg2_solver
  srrg2_solver_extras
  srrg_cmake_modules
)

find_package(PCL REQUIRED)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia find system stuff
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)

catkin_package(
 INCLUDE_DIRS
 src

 CATKIN_DEPENDS
 srrg2_core
 srrg2_solver
 srrg2_solver_extras
 srrg_cmake_modules
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

add_subdirectory(src/apps)

