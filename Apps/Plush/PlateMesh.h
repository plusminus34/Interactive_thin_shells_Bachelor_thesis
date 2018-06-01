#pragma once

// --
#include <PlushHelpers/tetgen.h> // --
// --
#include "Plate.h"
#include "MagicTendon.h"
#include "CSTSimulationMesh3D.h"
#include "SoftIKSolver.h"
#include "SmoothestStep.h"
#include "QueuePlot.h"

class PlateMesh : public Handler {

public:
	PlateMesh(GLCamera *, Plane *);
	inline ~PlateMesh() {}
	void draw();

public:
	GLCamera *camera = nullptr;
	Plane *drag_plane = nullptr;

public: 
	bool BAKE_TETS  = false;
	bool BAKE_TDNS  = false;
	bool BAKED_TETS = false;
	bool BAKED_TDNS = false;

public:
	bool DRAW_DESIGNER           = true;
	bool DRAW_BAKE               = false;
	bool DRAW_SIMULATION         = false;

public:
	void reset_simulation();
	void step_simulation();
	bool SIMULATION_STARTED = false;
	bool PLEASE_REBUILD_SIMULATION_MESH = true;
	CSTSimulationMesh3D *simMesh = nullptr;
	SoftIKSolver *ik = nullptr;
	const double nominalTimeStep = .01;
	// --
	double t_simulation = 0.;
	double t_motionplan = 0.;
	double SIMULATION_SPEED_ = 1.;
	double MOTIONPLAN_SPEED_ = .05;
	// --
	vector<QueuePlot *> queue_plots;
	vector<QueuePlot *> xy_plots;
	// --
	bool ACTUALLY_SOLVE = true;
	bool SOLVE_DYNAMICS = true;

public:
	bool DRAW_LOCAL_COORDINATES = false;
	bool DRAW_POINTS = false;
	bool DRAW_PLATES = true;

public: 
	vector<P3D>    points; 
	vector<Plate *> plates; 

public:
	vector<MagicTendon *> tendons;

public:
	double MAX_TET_VOLUME = .1;
 
public:
	void perform_tetrahedralization(tetgenio &, tetgenio &);
	void store_tetrahedralization(const tetgenio &, const tetgenio &);
	// -- 
	vector<P3D>         v_out;
	vector<vector<int>> tet_out;
	// --
	void post_process_tetrahedralization();

public:
	void bake_tendons();
	// --
	typedef pair<vector<int>, vector<double>> RawPoint;
	vector<vector<RawPoint>> tdn_out;

public:
	void save(const char *);
	// --
	void save_s(const char *);
	void save_poly(const char *);
	void save_plan(const char *); 
	// --
	bool SAVE = false;

public:
	void load(const char *);
	// --
	void load_s(const char *);
	void load_poly(const char *);
	void load_plan(const char *); 
	// --
	bool LOAD = true; 
 
// --

public:
	void dump(const char *);
	// --
	void dump_v(const char *);
	void dump_tet(const char *);
	void dump_tdn(const char *);

public:
	virtual bool mouse_move_(double xPos, double yPos);
	virtual bool mouse_button_(int button, int action, int mods, double xPos, double yPos);

};