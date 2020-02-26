cmake_minimum_required(VERSION 2.8.3)
project(srrg2_solver)

#ds help IDEs - uncomment on your machine if needed
#set(CMAKE_VERBOSE_MAKEFILE ON)

#ia catkin package
find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia find system stuff
find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)

#ia create a package
catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_solver_core_library
    srrg2_solver_types2d_library
    srrg2_solver_types3d_library
    srrg2_solver_calib_library
    srrg2_solver_projective_library
    srrg2_solver_factor_graph_utils_library
    srrg2_solver_linear_solvers_library
    srrg2_solver_sparse_block_matrix_library
)

#ia include directories
include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${CHOLMOD_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DSRRG2_SOLVER_EXAMPLE_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/examples")
message("${PROJECT_NAME}|example dir = ${CMAKE_CURRENT_SOURCE_DIR}/examples")
add_definitions(-DSRRG2_SOLVER_DATA_FOLDER="${CMAKE_CURRENT_SOURCE_DIR}/examples/data")
message("${PROJECT_NAME}|data dir    = ${CMAKE_CURRENT_SOURCE_DIR}/examples/data")

#ds help the catkin tool on 16.04 (cmake seems unable to find single libraries, although catkin claims the link_directories call is not required)
#ds in order to avoid linking against the catkin_LIBRARIES bulk everytime enable this so one can select single libraries
link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(examples)
add_subdirectory(app)
