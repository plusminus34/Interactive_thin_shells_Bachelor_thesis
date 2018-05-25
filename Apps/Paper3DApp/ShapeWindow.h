#pragma once

#include <GUILib/GLWindow3D.h>
#include "Paper3DApp.h"

class BarycentricZeroLengthSpring;

class ShapeWindow: public GLWindow3D{
protected:
	Paper3DApp* paperApp;

	//helper variables for dragging
	bool dragging;
	double xDrag, yDrag;

	//helper variables for pinning
	bool first_point_set = false;
	double xPin, yPin;

	//Paper shape parameters
	int dim_x=11;// number of nodes in x
	int dim_y=7;// number of nodes in y
	double h=0.1;// distance between neighboring nodes

	int findNodeClosestTo(double x, double y);
	BarycentricZeroLengthSpring* createZeroLengthSpring(double x0, double y0, double x1, double y1);

public:
	ShapeWindow(int x, int y, int w, int h, Paper3DApp *glApp);
	~ShapeWindow();

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);

	//TODO pinning
	/*
	DONE select two points on the sheet to connect
	  -> have to find barycentric coordinates from screen coordinates
	  -> need access to the mesh to add pin
	alter existing pin by rotating/flipping/translating pin ends
	  -> should be totally doable if creation of the pins is possible
	*/

	//TODO cutting
	/*
	  -> need to know triangles adjacent to nodes
	  -> need ability to alter mesh elements ("triangle 15, you now connect nodes 42 and 55 instead of 12 and 13")
	  -> should have an idea of the boundary
	*/

	/*
	Looking at those two, it seems like a good idea to have a different representation of the paper sheet here
	*/
};

