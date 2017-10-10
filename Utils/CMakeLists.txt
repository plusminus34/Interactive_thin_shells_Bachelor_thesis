cmake_minimum_required(VERSION 2.8)

project(Utils)

# add source files
file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
	)

add_library(${PROJECT_NAME} STATIC ${sources})

target_link_libraries(
	${PROJECT_NAME}
	PUBLIC
		MathLib
	PRIVATE
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
		# MathLib
		# $<INSTALL_INTERFACE:include/${PROJECT_NAME}> # TODO
)
