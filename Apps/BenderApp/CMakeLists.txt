cmake_minimum_required(VERSION 3.5)

project(BenderApp)

file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/*.h
		${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
		
		#add stuff from the LazyFEMLib, without creating it as a seperate library
		${CMAKE_CURRENT_SOURCE_DIR}/LazyFEMSimLib/src/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/LazyFEMSimLib/include/${PROJECT_NAME}/*.h
		${CMAKE_SOURCE_DIR}/thirdparty/tetgen/*.h
		${CMAKE_SOURCE_DIR}/thirdparty/tetgen/*.cxx
	)
add_executable(${PROJECT_NAME} ${sources})

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/LazyFEMSimLib/include")

#include_directories("${EIGEN3_INCLUDE_DIR}")
#include_directories("${CMAKE_SOURCE_DIR}/thirdparty")

target_link_libraries(${PROJECT_NAME}
	MathLib
	GUILib
	OptimizationLib
#	LazyFEMSimLib
	RBSimLib
	ControlLib
	YuMiLib
)
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Apps")

if(WIN32)
	# ignore linker warnings
	set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/ignore:4006 /ignore:4088")

endif()

#add_subdirectory(LazyFEMSimLib)