#pragma once

#include "SimulationMesh.h"
#include "ZeroCubicQuadratic.h"
#include "Tendon.h"

class SimulationMesh;

typedef vector<dVector> Traj;

class SoftLocoSolver {

	friend class SimulationMesh; 

public:
	P3D COMp_FORNOW = P3D();

	vector<bool> SPEC_FREESTYLE_J;
	vector<bool> SPEC_COM_J;
	vector<P3D> COMpJ;

public:
	SoftLocoSolver(SimulationMesh *);
	inline ~SoftLocoSolver() {};
	void draw();

public:
	SimulationMesh *mesh;

public:

	int SELECTED_FRAME_i = 0;
 
public:
	double timeStep = .01;
	// --
	bool PROJECT = false;
	bool CHECK_GRADIENT = false;
	bool LINEAR_APPROX = true;
	bool VERBOSE = false;
	bool SOLVE_DYNAMICS = true;

public:
	int D();
	int N();
	int T();
	const int K = 1; // HORIZON
	// --
	bool check_x_size(const dVector &x);
	bool check_alphac_size(const dVector &alphac);

public:
	double     calculate_OJ(const Traj &alphac);
	double     calculate_QJ(const Traj &alphac);
	double     calculate_RJ(const Traj &alphac);
	double     calculate_O(const dVector &alphac);
	double     calculate_Q(const dVector &alphac);
	double     calculate_R(const dVector &alphac);
	// --
	double calculate_Q_formal(const dVector &alphac);
	double calculate_Q_approx(const dVector &alphac);
	double calculate_Q_of_x(const dVector &x, const P3D &COMp);


public:
	dVector x_0, v_0;
	Traj alphacJ_curr;
	Traj xJ_curr;
	// --
	dVector alphac_curr; dVector x_curr;

public:
	// MatrixNxM       dtaudalphac;
	MatrixNxM       dxdalphac;
	// TODO: How do we LINEAR_APPROX on a trajectory? Answer: Just store the relavent Jacobians yo.

public:
	Traj solve_trajectory(double dt, const dVector &x_0, const dVector &v_0, const Traj &alphacJ);
 
public:
	void step();
	int NUM_ITERS_PER_STEP = 5;

public:
	void iterate();
	void project(); 
	void projectJ(); 

	Traj alphacJ_next(const Traj &alphacJ, const Traj &xJ);
	double calculate_gammaJ(const Traj &alphacJ, const Traj &dOdalphacJ);
	Traj xJ_of_alphacJ(const Traj &alphacJ);

	dVector alphac_next(const dVector &alphac, const dVector &x); 
	double calculate_gamma(const dVector &alphac, const dVector &dOdalphac); 
	dVector x_of_alphac(const dVector &alphac);

	// -- //
	Traj calculate_dOdalphacJ(const Traj &alphacJ, const Traj &xJ);
	Traj calculate_dQdalphacJ(const Traj &alphacJ, const Traj &xJ);
	Traj calculate_dRdalphacJ(const Traj &alphacJ);

	dVector calculate_dOdalphac(const dVector &alphac, const dVector &x);
	dVector calculate_dQdalphac(const dVector &alphac, const dVector &x);
	dVector calculate_dQdx(const dVector &alphac, const dVector &x, const P3D &COMp);
	MatrixNxM calculate_dxdalphac(const dVector &alphac, const dVector &x);
	dVector calculate_dRdalphac(const dVector &alphac);

	// -- //

	SparseMatrix calculate_A(const dVector &x);
	SparseMatrix calculate_H(const dVector &x, const dVector &alphac);

public: 
	vector<ZeroCubicQuadratic *> alphac_barrierFuncs;
	void construct_alphac_barrierFuncs();

	// NOTE(*): For bringing back to alpha <- alphaz
	bool REGULARIZE_alphac = true;
	double c_alphac_ = .01;// 2.e-1;
	Quadratic *alphac_regFunc = new Quadratic(&c_alphac_);

};

