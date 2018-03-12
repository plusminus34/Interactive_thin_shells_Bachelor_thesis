cmake_minimum_required(VERSION 3.5)

# set name of the project
project(SCP)

# directory 'cmake' contains macros to find thirdparty libraries
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

# set SCP working dir
# TODO: find a better place than data ...
add_definitions(-DSCP_WORKING_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

set(SCP_GUI_TWO_WINDOWS FALSE CACHE BOOL "Use two window mode to circumvent OpenGL compatibility")
if (SCP_GUI_TWO_WINDOWS)
    add_definitions(-DGUI_TWO_WINDOWS)
endif()

set(SCP_EIGEN_NO_DEBUG FALSE CACHE BOOL "Disable Eigen asserts")
if (SCP_EIGEN_NO_DEBUG)
    add_definitions(-DEIGEN_NO_DEBUG)
endif()

# dependencies project-wide
find_package(Eigen3 3.3.4 REQUIRED)

# use openmp
if (FALSE)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
endif()

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
add_subdirectory(FEMSimLib)
add_subdirectory(BulletCollision)
add_subdirectory(RobotDesignerLib)
add_subdirectory(PlushHelpers)
add_subdirectory(Apps)
add_subdirectory(YuMiLib)

set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "_CMakeTargets")
