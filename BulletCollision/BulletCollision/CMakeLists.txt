cmake_minimum_required(VERSION 2.8)

project(BulletCollision)

# add source files
file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/*.h
		
		${CMAKE_CURRENT_SOURCE_DIR}/BroadphaseCollision/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/BroadphaseCollision/*.cpp

		${CMAKE_CURRENT_SOURCE_DIR}/CollisionDispatch/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/CollisionDispatch/*.cpp
		
		${CMAKE_CURRENT_SOURCE_DIR}/CollisionShapes/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/CollisionShapes/*.cpp
		
		${CMAKE_CURRENT_SOURCE_DIR}/Gimpact/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/Gimpact/*.cpp
		
		${CMAKE_CURRENT_SOURCE_DIR}/LinearMath/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/LinearMath/*.cpp
		
		${CMAKE_CURRENT_SOURCE_DIR}/NarrowPhaseCollision/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/NarrowPhaseCollision/*.cpp
	)

add_library(${PROJECT_NAME} STATIC ${sources})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/..")
# include_directories("${EIGEN3_INCLUDE_DIR}")
# include_directories("${CMAKE_SOURCE_DIR}/thirdparty/ft2.5.5")
# include_directories("${CMAKE_SOURCE_DIR}/thirdparty")

target_link_libraries(
	${PROJECT_NAME}
	PUBLIC
	PRIVATE
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/..
		# MathLib
		# $<INSTALL_INTERFACE:include/${PROJECT_NAME}> # TODO
)
