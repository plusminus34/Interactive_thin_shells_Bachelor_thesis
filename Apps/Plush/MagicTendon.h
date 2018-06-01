#pragma once
// --
#include <GUILib/GLUtils.h>
#include <GUILib/GLTrackingCamera.h>
#include <MathLib/Ray.h>
#include <PlushHelpers/helpers_star.h>
#include "Handler.h"
#include "MagicPoint.h"

class PlateMesh; 

class MagicTendon : public Handler {

public:
	MagicTendon(GLTrackingCamera *, vector<MagicPoint *>);
	virtual void draw();
	GLTrackingCamera *camera;
	vector<MagicPoint *> points;
	MagicPoint *selected_point = nullptr;
 
public:
	virtual bool mouse_move_(double xPos, double yPos);
	virtual bool mouse_button_(int button, int action, int mods, double xPos, double yPos);

public:
	const double POINT_THRESH = .02;

};
