cmake_minimum_required(VERSION 3.5)

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

# Tests
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

file(GLOB test_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.h
    )
add_executable(${PROJECT_NAME}_test ${test_sources})
target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}
    ${GTEST_BOTH_LIBRARIES}
    -lpthread) # need pthread for google test. TODO: does this link on Windows?
add_test(${PROJECT_NAME}_test ${PROJECT_NAME}_test)
