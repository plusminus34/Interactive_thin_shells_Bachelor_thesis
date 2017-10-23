cmake_minimum_required(VERSION 3.5)

project(NanoGuiTestApp)

file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/nanogui_test_app.cpp
	)
add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME}
	MathLib
	GUILib
	OptimizationLib
	RBSimLib
)
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Apps")
