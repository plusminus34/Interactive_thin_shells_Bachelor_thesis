cmake_minimum_required(VERSION 2.8)

# set name of the project
project(SCP)

# directory 'cmake' contains macros to find thirdparty libraries
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

# set SCP working dir
# TODO: find a better place than data ...
add_definitions(-DSCP_WORKING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

# dependencies project-wide
find_package(Eigen3 3.3.4 REQUIRED)

# enable parallel build for Visual Studio
if (WIN32)
	add_compile_options(/MP)
endif()

# add directories (in each subdirectory there needs to be a 'CMakeLists.txt')
add_subdirectory(Utils)
add_subdirectory(MathLib)
add_subdirectory(GUILib)
add_subdirectory(OptimizationLib)
add_subdirectory(RBSimLib)
add_subdirectory(ControlLib)
# add_subdirectory(FEMSimLib)
add_subdirectory(BulletCollision)
add_subdirectory(RobotDesignerLib)
add_subdirectory(Apps)

set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "_CMakeTargets")
