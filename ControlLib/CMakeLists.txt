cmake_minimum_required(VERSION 2.8)

project(ControlLib)

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
		GUILib
		OptimizationLib
		RBSimLib
	PRIVATE
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
)

# Create filters:
file(GLOB IK_headers
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/IK_*.h
	)
source_group("Header Files\\IK" 
	FILES
		${IK_headers}
)

file(GLOB IK_sources
		${CMAKE_CURRENT_SOURCE_DIR}/src/IK_*.cpp
	)
source_group("Source Files\\IK" 
	FILES
		${IK_sources}
)

source_group("Header Files\\QPControl" 
	FILES 
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/MOPTQPTrackingController.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/QPControlConstraints.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/QPControlEnergyFunction.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/QPControlEngine.h
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/QPControlPlan.h
)

source_group("Source Files\\QPControl"
	FILES
		${CMAKE_CURRENT_SOURCE_DIR}/src/MOPTQPTrackingController.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/QPControlConstraints.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/QPControlEnergyFunction.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/QPControlEngine.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/QPControlPlan.cpp
)
