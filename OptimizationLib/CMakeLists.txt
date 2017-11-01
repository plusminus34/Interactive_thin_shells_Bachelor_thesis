cmake_minimum_required(VERSION 3.5)

project(OptimizationLib)

# add source files
file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/cmaes/*.c
		${CMAKE_CURRENT_SOURCE_DIR}/cmaes/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.hpp
	)

add_library(${PROJECT_NAME} STATIC ${sources})

# OOQP
find_package(OOQP REQUIRED)
include_directories(${OOQP_INCLUDE_DIRS})

include_directories("${CMAKE_SOURCE_DIR}/thirdparty")

target_link_libraries(
	${PROJECT_NAME}
	    MathLib
	    ${OOQP_LIBRARIES}
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
	    ${CMAKE_SOURCE_DIR}/thirdparty
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
