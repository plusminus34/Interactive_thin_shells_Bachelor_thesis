cmake_minimum_required(VERSION 2.8)

project(FEMSimLib)

# add source files
file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
		
		${CMAKE_SOURCE_DIR}/thirdparty/tetgen/*.h
		${CMAKE_SOURCE_DIR}/thirdparty/tetgen/*.cxx
	)

add_library(${PROJECT_NAME} STATIC ${sources})

include_directories("${EIGEN3_INCLUDE_DIR}")
include_directories("${CMAKE_SOURCE_DIR}/thirdparty")

target_link_libraries(
	${PROJECT_NAME}
	PUBLIC
		MathLib
		OptimizationLib
		GUILib
	PRIVATE
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
)
