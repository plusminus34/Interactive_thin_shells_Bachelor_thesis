#pragma once

#include <GUILib/GLWindow3D.h>
#include "Paper3DApp.h"
#include "PinHandle.h"

class BarycentricZeroLengthSpring;
class Pin;

class ShapeWindow: public GLWindow3D{
protected:
	Paper3DApp* paperApp;

	//helper variables for dragging
	double xDrag, yDrag;
	int selected_i;

	//helper variables for pinning
	bool first_point_set = false;
	double xPin, yPin, initialAngle;
	int next_pin_id = 0;
	DynamicArray<PinHandle*> pinHandles;

	//Paper shape parameters
	int dim_x;// number of nodes in x
	int dim_y;// number of nodes in y
	double h;// distance between neighboring nodes

	//helper variables for cutting
	DynamicArray<uint> cutPath;

	//helper variables for display
	bool display_bending = true;
	double e_bend_max = 0.002;

	int findNodeClosestTo(double x, double y);
	int findPinHandleClosestTo(double x, double y, double max_distance = 0.1);

	BarycentricZeroLengthSpring* createZeroLengthSpring(double x0, double y0, double x1, double y1);
	Pin* createPinFromHandles(uint h0, uint h1);

public:
	ShapeWindow(int x, int y, int w, int h, Paper3DApp *glApp);
	~ShapeWindow();

	virtual void getSaveData(int &grid_width, int &grid_height, double &cell_size, MatrixNxM &pins);
	virtual void applyLoadData(int &grid_width, int &grid_height, double &cell_size, MatrixNxM &pins);

	virtual void setGridDimensions(int l, int h, double d);

	virtual void drawScene();
	virtual void drawAuxiliarySceneInfo();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);

};

