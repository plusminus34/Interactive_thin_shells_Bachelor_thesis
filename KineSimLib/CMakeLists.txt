cmake_minimum_required(VERSION 3.5)

project(KineSimLib)

# add source files
file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
	)

add_library(${PROJECT_NAME} STATIC ${sources})

# ODE
find_package(ODE REQUIRED)
include_directories(${ODE_INCLUDE_DIRS})

# include_directories("${EIGEN3_INCLUDE_DIR}")
# include_directories("${CMAKE_SOURCE_DIR}/thirdparty/ft2.5.5")
include_directories("${CMAKE_SOURCE_DIR}/thirdparty")

find_package(OpenGL REQUIRED)

target_link_libraries(
	${PROJECT_NAME}
	PUBLIC
		MathLib
		OptimizationLib
		GUILib
		BulletCollision
		${ODE_LIBRARIES}
		${OPENGL_LIBRARIES}
	PRIVATE
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
		${ODE_INCLUDE_DIRS}
)
