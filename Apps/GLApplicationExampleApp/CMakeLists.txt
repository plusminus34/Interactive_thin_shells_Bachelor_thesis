cmake_minimum_required(VERSION 2.8)

project(GLApplicationExampleApp)

file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/*.h
	)
add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME}
	MathLib
	GUILib
#        OptimizationLib
#        RBSimLib
)
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Apps")
