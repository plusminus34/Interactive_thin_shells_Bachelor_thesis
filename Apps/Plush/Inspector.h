#pragma once
// --
#include <PlushHelpers/helpers_star.h>
// --
#include <GUILib\GLUtils.h>
// --
#include <MathLib\Ray.h>

#include "Handler.h"
#include "SimulationMesh.h"
#include "Node.h"
#include "Tendon.h"
// --
// #include <AntTweakBar.h>

class Inspector : public Handler {

public: 
	// static inline void TW_CALL dummyErrorHandler(const char *errorMessage) {}
	// static inline void TW_CALL defaultErrorHandler(const char *errorMessage) { cout << "[TWERROR] " << errorMessage << endl; } // FORNOW

public:
	Inspector(SimulationMesh *);
	virtual void draw();
	// --
	SimulationMesh *mesh;
	vector<Node *> nodes;
	vector<Tendon *> tendons;
	double TMP_ALPHAC = 0.;

public:
	bool PREFER_NODES = false;

public:
	int Z = 5;
	const P3D HOVER_COLOR    = color_shade(LAVISH);
	const P3D SELECTED_COLOR = LAVISH;

public:
	pair<int, double> get_closest_node_i(double, double);   // TODO: Port these over to a mesh class eventually
	pair<int, double> get_closest_tendon_i(double, double); // -- // ""
	void interrogate(double xPos, double yPos, int &node_i, double &node_d, int &tendon_i, double &tendon_d);
	// TODO: Version of this that exclusively uses pointers, with relavent nullptr guards for when stuff gets deleted?  Does that work?
	int hovered_node_i    = -1;
	int hovered_tendon_i  = -1;
	int selected_node_i   = -1;
	int selected_tendon_i = -1;

public:
	virtual bool mouse_move(double xPos, double yPos); 
	virtual bool mouse_button(int button, int action, int mods, double xPos, double yPos);
	virtual bool mouse_wheel(double xOffset, double yOffset);
	virtual bool key_event(int key, int action, int mods);

public:
	const double POINT_THRESH = .05;
	const double SEGMENT_THRESH = .05;

};
