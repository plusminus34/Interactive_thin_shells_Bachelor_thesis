cmake_minimum_required(VERSION 3.5)

project(RobotDesignerApp)

file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
	)
add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME}
	MathLib
	GUILib
	OptimizationLib
	RBSimLib
	ControlLib
	RobotDesignerLib
)
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Apps")

if(WIN32)
	
	# ignore linker warnings
	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/ignore:4006 /ignore:4088")

endif()