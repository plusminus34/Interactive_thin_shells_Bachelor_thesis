cmake_minimum_required(VERSION 3.5)

project(OptimizationDemoApp)

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
)
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Apps")

if(WIN32)
	# copy freetype dlls to binary dir
	add_custom_command(
	        TARGET ${PROJECT_NAME} POST_BUILD
	        COMMAND ${CMAKE_COMMAND} -E copy
	                ${FREETYPE_DLLS}
	                ${CMAKE_CURRENT_BINARY_DIR}
	)
	
	# ignore linker warnings
	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/ignore:4006 /ignore:4088")

endif()