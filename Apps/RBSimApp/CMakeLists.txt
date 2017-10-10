cmake_minimum_required(VERSION 2.8)

project(RBSimApp)

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
)
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Apps")

if(WIN32)
	add_custom_command(
	        TARGET ${PROJECT_NAME} POST_BUILD
	        COMMAND ${CMAKE_COMMAND} -E copy
	                ${FREETYPE_DLLS}
	                ${CMAKE_CURRENT_BINARY_DIR}
	)
endif()