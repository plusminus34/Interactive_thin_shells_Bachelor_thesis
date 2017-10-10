cmake_minimum_required(VERSION 2.8)

project(GUILib)

# fix for Visual Studio: copy dll's to bin directory 
# (otherwise VS doesn't find the dll ...)
# in this case for nanogui.dll
# If this doesn't work, one can always copy the dll's by hand.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# add source files
file(GLOB sources
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h
	)

# Adding nanogui:
# Disable building extras we won't need (pure C++ project)
set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
# add nanogui subirectory
add_subdirectory(ext/nanogui)
# For reliability of parallel build, make the NanoGUI targets dependencies
set_property(TARGET glfw glfw_objects nanogui nanogui-obj PROPERTY FOLDER "dependencies")
add_definitions(${NANOGUI_EXTRA_DEFS})
include_directories(ext/nanogui/include)
include_directories(${NANOGUI_EXTRA_INCS})

# Freetype
if(NOT DEFINED ENV{FREETYPE_DIR})
	set(ENV{FREETYPE_DIR} ${CMAKE_SOURCE_DIR}/../libs/thirdPartyCode/freetype)
	message(STATUS "No environment variable 'FREETYPE_DIR' found. Using '" $ENV{FREETYPE_DIR} "'.")
endif()
find_package(Freetype REQUIRED)
include_directories(${FREETYPE_INCLUDE_DIRS})

# OpenGL
find_package(OpenGL REQUIRED)

# Make this a static library
add_library(${PROJECT_NAME} STATIC ${sources})

target_link_libraries(
	${PROJECT_NAME}
	PUBLIC
		MathLib
		nanogui
		${NANOGUI_EXTRA_LIBS}
		${FREETYPE_LIBRARIES}
		${OPENGL_LIBRARIES}
	PRIVATE
)

target_include_directories(
	${PROJECT_NAME} 
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/include
		ext/nanogui/include
		${NANOGUI_EXTRA_INCS}
)
