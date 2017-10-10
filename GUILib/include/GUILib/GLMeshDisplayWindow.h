#pragma once
#include "GLIncludes.h"
#include "GLWindow3D.h"
#include <Utils/Logger.h>
#include <Utils/Timer.h>
#include "GLMeshDisplayWindow.h"
#include "GLCamera.h"
#include <GUILib/GLMesh.h>

/**
* Used for any window that needs to be displayed in the OpenGL window
*/
class GLMeshDisplayWindow : public GLWindow3D {
protected:
public:
	GLMesh* mesh = NULL;

	// constructors
	GLMeshDisplayWindow(GLMesh* mesh, int x, int y, int w, int h);
	GLMeshDisplayWindow(GLMesh* mesh);
	GLMeshDisplayWindow();

	// destructor
	virtual ~GLMeshDisplayWindow() {};

	virtual void drawScene();
};

