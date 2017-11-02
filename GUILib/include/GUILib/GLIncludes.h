#pragma once

#ifndef GL_INCLUDES_H
#define GL_INCLUDES_H

#ifdef WIN32
#include <Windows.h>
#undef min
#undef max
#endif // WIN32

#pragma warning( disable : 4996)
#pragma warning( disable : 4006)

// Make sure nanogui uses glad
// (actually, this define is added by the nanogui CMakelists.txt, but this doesn't seem to work...)
#ifndef NANOGUI_GLAD
	#define NANOGUI_GLAD
#endif // NANOUI_GLAD
#include <nanogui/opengl.h>

#include <GL/glu.h>

//#include <glad/glad.h>
//#define GLFW_INCLUDE_GLU
//#include <GLFW/glfw3.h>

#endif // !GL_INCLUDES_H
