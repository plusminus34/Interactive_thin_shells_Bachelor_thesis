#pragma once

#include "SimulationMesh.h"
#include "ZeroCubicQuadratic.h"
#include "Tendon.h"

#include "CubicHermiteSpline.h"
#include "CubicHermiteSpline_v2.h"

#include <OptimizationLib/GradientDescentFunctionMinimizer.h>
#include <OptimizationLib/BFGSFunctionMinimizer.h>
#include <OptimizationLib/SQPFunctionMinimizer.h>
#include <OptimizationLib/ConstrainedObjectiveFunction.h>

#include "SoftLocoObjectiveFunction.h"
#include "SoftLocoConstraints.h"

class SimulationMesh;

typedef Eigen::SimplicialLDLT<SparseMatrix> SLSSolver;
 
class SoftLocoSolver {

	friend class SimulationMesh; 

public:
	SoftLocoObjectiveFunction *objectiveFunction;
	SoftLocoConstraints *constraints;
	ConstrainedObjectiveFunction *constrainedObjectiveFunction;

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

// (K, Z) = (32, 4)
// (K, Z) = (64, 8)

public:
	int D();
	int N();
	int DN();
	int T();
	const int K = 64 + 1;// 64 + 1;
	int ZS();
	// --
	bool check_x_size(const dVector &x);
	bool check_u_size(const dVector &u);
	bool check_y_size(const dVector &y);

public:
	CubicHermiteSpline *god_spline;
	SparseMatrix  dUdY_;

	CubicHermiteSpline_v2 *god_spline2;
	SparseMatrix  dUdY2_;
	SparseMatrix  dUdM2_;

	const int Z = 3;// ((K - 1) / 32) + 1;
	const dVector knot_times = vecDouble2dVector(linspace(8, 0., 1.)); 
	int k_of_z(const int &z) { return (K-1)/(Z-1)*z; }
	int S();

public:
	void FD_TEST_dOJdymJ(const Traj &ymJ, const Traj &xJ);
	double FD_TEST_STEPSIZE = 1.5e-5;

public:
	double     calculate_OJ(const Traj &ymJ);
	double     calculate_QJ(const Traj &uJ);
	double     calculate_RJ(const Traj &uJ);
	double     calculate_R(const dVector &u);
	double calculate_Q_of_x(const dVector &x, const P3D &COMp);

public:
	dVector xm1_curr, vm1_curr; // TODO: rename xm1, vm
	dVector um1_curr;

	Traj ymJ_curr; // TODO: Rename sJ_curr ([ y ... y , m ... m ])
	Traj xJ_curr;
	Traj uJ_curr() { return uJ_of_ymJ(ymJ_curr); }
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
	// void projectJ(); 

	// Traj uJ_next(const Traj &uJ, const Traj &xJ);
	// Traj yJ_next(const Traj &yJ, const Traj &xJ);
	// double calculate_gammaJ(const Traj &yJ, const vector<dRowVector> &dOdyJ);
	Traj xJ_of_ymJ(const Traj &ymJ);
	Traj xJ_of_uJ(const Traj &uJ);
	Traj uJ_of_ymJ(const Traj &ymJ);

	Traj xZ_from_xJ(const Traj &xJ); // keyframes

	// -- //

	vector<dRowVector> calculate_dOdymJ(const Traj &ymJ, const Traj &xJ);
	vector<dRowVector> calculate_dQduJ(const Traj &uJ, const Traj &xJ);
	vector<dRowVector> calculate_dRduJ(const Traj &uJ);

	dRowVector calculate_dQdx(const dVector &x, const P3D &COMp);
	SparseMatrix calculate_dxdu(const dVector &u, const dVector &x, SLSSolver *Hsolver, const dVector &x_ctc=dVector());
	// SparseMatrix calculate_dudz(const dVector &u, const dVector &z);
	dRowVector calculate_dRdu(const dVector &u);
	vector<dRowVector> dSTARduJ2dSTARdymJ(const vector<dRowVector> &);

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
	SparseMatrix solve_AX_EQUALS_B(const SparseMatrix &A, const SparseMatrix &B);
	// --
	void analyze_A(SLSSolver &solver, const SparseMatrix &A);
	SparseMatrix solve_AX_EQUALS_B_WITHOUT_ANALYSIS_OR_FACTORIZATION(SLSSolver *solver, const SparseMatrix &B);

public:
	void Traj_equality_check(const Traj &, const Traj &);
	void Traj_equality_check(const Traj &, const vector<dRowVector> &);
	void MTraj_equality_check(const MTraj &, const MTraj &);
	void MTraj_equality_check(const MTraj &T1, const vector<SparseMatrix> &T2);
	Traj unstack_Traj(const dVector &, int VEC_SIZE);
	
	template<typename T>
	vector<T> zipunzip(const vector<T> &in) {
		int in_LENGTH = in[0].size();
		int in_COUNT  = in.size();
		// --
		for (auto &dVec : in) { if (dVec.size() != in_LENGTH) { helpers_error("dVec's are different lengths."); } }
		vector<T> out;
		for (int i = 0; i < in_LENGTH; ++i) {
			T dVec; dVec.setZero(in_COUNT);
			for (int j = 0; j < in_COUNT; ++j) {
				dVec[j] = in[j][i];
			} 
			out.push_back(dVec);
		} 
		return out; 
	}

public:
	vec<double> magG_tmp_vec;
};
 
	// reindex

	// T() tendons
	// T() splines
	// U() sampled data points
	// Y() spline parameters

	// dQJdui is a list of vectors, where the i-th vector
	// tells us how the total objective QJ changes wrt
	// all contractions at timestep i.

	// We now want dQJdyj, which will be a list of vectors, where the j-th 
	// tells us how the total objective QJ changes wrt
	// all spline control points at timestep j.

	// We need duidyj, which will be a table of [T()xT()] matrices, where the [i][j]-th matrix
	// tells us how the contractions at timestep i change wrt
	// the knot positions at timestep j.

	// What we have for cubic hermite splines is dUdY, which is a [U()xY()] constant matrix
	// telling us for a given spline how spline positions change wrt
	// knot positions.

	// A given entry M(a, b) of M = duidyj[i][j]
	// asks how does the contraction of the a-th tendon @ time i changes wrt
	// the knot position the b-th spline @ time j 

	// If a != b, then M(a, b) = 0.
	// So we will just be populating the diagonal.
	// M(a, a) says for the a-th tendon/spline,
	// -- how does the contraction of ui change wrt yj
	// This is dUdY(i, j), which is a scalar constant (times the identity).

	// So in conclusion.
	// duidyj can be considered as a table of constant scalars, i.e. exactly dUdY.  