#pragma once
#include "GLWindow3D.h"
class GLWindow3DWithMesh :
	public GLWindow3D
{
public:
	GLWindow3DWithMesh(int x, int y, int w, int h);
	~GLWindow3DWithMesh();
	virtual void drawScene();
	GLMesh *mesh;
};

