#pragma once

#include "SimulationMesh.h"
#include "ZeroCubicQuadratic.h"
#include "Tendon.h"
#include "SoftLocoObjectiveFunction.h"
#include <OptimizationLib/GradientDescentFunctionMinimizer.h>
#include <OptimizationLib/BFGSFunctionMinimizer.h>

class SimulationMesh;

typedef vector<dVector> Traj;
typedef vector<MatrixNxM> MTraj;
typedef Eigen::RowVectorXd dRowVector;

class SoftLocoSolver {

	friend class SimulationMesh; 

public:
	SoftLocoObjectiveFunction *objectiveFunction;

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
	bool PROJECT = false;
	bool CHECK_GRADIENT = false;
	bool LINEAR_APPROX = true;
	bool VERBOSE = false;
	bool SOLVE_DYNAMICS = true;
	bool TEST_Q_FD = false;
	bool TEST_R_FD = false;
	bool REPLAY = false;

public:
	int D();
	int N();
	int DN();
	int T();
	const int K = 1; // 48; // HORIZON
	// --
	bool check_x_size(const dVector &x);
	bool check_u_size(const dVector &u);

public:
	double     calculate_OJ(const Traj &u);
	double     calculate_QJ(const Traj &u);
	double     calculate_RJ(const Traj &u);
	double     calculate_R(const dVector &u);
	double calculate_Q_of_x(const dVector &x, const P3D &COMp);

	// double     calculate_O(const dVector &u);
	// double     calculate_Q(const dVector &u);
	// double calculate_Q_formal(const dVector &u);
	// double calculate_Q_approx(const dVector &u);


public:
	dVector xm1_curr, vm1_curr; // TODO: rename xm1, vm
	dVector um1_curr;
	Traj uJ_curr;
	Traj xJ_curr;
	// --
	// dVector u_curr; dVector x_curr;

public:
	// MatrixNxM       dtaudu;
	vector<vector<MatrixNxM>> dxiduj_SAVED;
	MatrixNxM dxdu_SAVED;
	// TODO: How do we LINEAR_APPROX on a trajectory? Answer: Just store the relavent Jacobians yo.

public:
	Traj solve_trajectory(double dt, const dVector &x_S, const dVector &v_S, const Traj &uJ);
 
public:
	void step();
	int NUM_ITERS_PER_STEP = 1;

public:
	void iterate();
	// void project(); 
	void projectJ(); 

	Traj uJ_next(const Traj &uJ, const Traj &xJ);
	double calculate_gammaJ(const Traj &uJ, const vector<dRowVector> &dOduJ);
	Traj xJ_of_uJ(const Traj &uJ);

	// dVector u_next(const dVector &u, const dVector &x); 
	// double calculate_gamma(const dVector &u, const dVector &dOdu); 
	// dVector x_of_u(const dVector &u);

	// -- //
	vector<dRowVector> calculate_dOduJ(const Traj &uJ, const Traj &xJ);
	vector<dRowVector> calculate_dQduJ(const Traj &uJ, const Traj &xJ);
	vector<dRowVector> calculate_dRduJ(const Traj &uJ);

	dRowVector calculate_dQdx(const dVector &u, const dVector &x, const P3D &COMp);
	MatrixNxM calculate_dxdu(const dVector &u, const dVector &x, const dVector &x_ctc=dVector());
	dRowVector calculate_dRdu(const dVector &u);

	// -- //

	SparseMatrix calculate_A(const dVector &x);
	SparseMatrix calculate_H(const dVector &x, const dVector &u, const dVector &x_ctc=dVector());

public: 
	vector<ZeroCubicQuadratic *> u_barrierFuncs;
	void construct_u_barrierFuncs();

	bool REGULARIZE_u = true;
	double r_u_ = .01;
	Quadratic *u_regFunc = new Quadratic(&r_u_);

	bool SUBSEQUENT_u = true;
	double s_u_ = .33;
	Quadratic *u_subFunc = new Quadratic(&s_u_);

public:
	void Traj_equality_check(const Traj &, const Traj &);
	void MTraj_equality_check(const MTraj &, const MTraj &);
	Traj unstack_Traj(const dVector &);


};
 