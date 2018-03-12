#pragma once
// --
#include <PlushHelpers/helpers_star.h>
// --
#include <GUILib/GLUtils.h>
#include "GUILib/InteractiveWidget.h"
// --
#include <MathLib/Ray.h>
#include <MathLib/Plane.h>
// --
#include "Handler.h"
#include "Node.h"
#include "SimulationMesh.h"
#include "SoftIKSolver.h"

class Poser : public Handler {

public:
	Poser(SimulationMesh *, SoftIKSolver *);
	virtual void draw();
	// --
	SimulationMesh *mesh;
	SoftIKSolver *ik;

public:
	Node *get_closest_node(double, double, const dVector &, vector<Node *>);
	Node *selected_node = nullptr;
	vector<Node *> active_nodes;
	// --
	void toggle_node(Node *, const P3D &);
	void move_node_target(Node *, const P3D &);

public:
	virtual bool mouse_move(double xPos, double yPos); 
	virtual bool mouse_button(int button, int action, int mods, double xPos, double yPos);

public:
	const double POINT_THRESH = .05;
	const double SEGMENT_THRESH = .05;

};
