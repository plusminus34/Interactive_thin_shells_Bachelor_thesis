#include <GUILib/GLWindow3DWithMesh.h>

GLWindow3DWithMesh::GLWindow3DWithMesh(int x, int y, int w, int h) : GLWindow3D(x, y, w, h)
{
}


GLWindow3DWithMesh::~GLWindow3DWithMesh()
{
}

void GLWindow3DWithMesh::drawScene()
{
	mesh->drawMesh();
}
