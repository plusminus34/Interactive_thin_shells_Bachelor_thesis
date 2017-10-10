#include <GUILib/GLApplication.h>

#include <GUILib/GLMeshDisplayWindow.h>
#include <Utils/Utils.h>
#include <GUILib/GLUtils.h>

#include <GUILib/GLContentManager.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLCamera.h>
#include <GUILib/GLTrackingCamera.h>

/**
Default constructor
*/
GLMeshDisplayWindow::GLMeshDisplayWindow(GLMesh* mesh, int posX, int posY, int sizeX, int sizeY) : GLWindow3D(posX, posY, sizeX, sizeY) {
	this->mesh = mesh;
}

GLMeshDisplayWindow::GLMeshDisplayWindow(GLMesh* mesh) : GLWindow3D() {
	this->mesh = mesh;
}

GLMeshDisplayWindow::GLMeshDisplayWindow() : GLWindow3D() {
	this->mesh = NULL;
}

void GLMeshDisplayWindow::drawScene() {
	if (mesh)
		mesh->drawMesh();
}

