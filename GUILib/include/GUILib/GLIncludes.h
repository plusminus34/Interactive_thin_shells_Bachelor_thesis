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

#include <nanogui/opengl.h>

#include <GL/glu.h>

//#include <glad/glad.h>
//#define GLFW_INCLUDE_GLU
//#include <GLFW/glfw3.h>

#endif // !GL_INCLUDES_H
