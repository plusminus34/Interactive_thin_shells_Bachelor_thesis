cmake_minimum_required(VERSION 2.8)

project(MathLib)

# add source files
file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
	)

add_library(${PROJECT_NAME} STATIC ${sources})

include_directories("${EIGEN3_INCLUDE_DIR}")

target_link_libraries(
	${PROJECT_NAME}
	Utils
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
		${EIGEN3_INCLUDE_DIR}
		# $<INSTALL_INTERFACE:include/${PROJECT_NAME}> # TODO
)
