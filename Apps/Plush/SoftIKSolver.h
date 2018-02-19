#pragma once

#include "SimulationMesh.h"
#include "ZeroCubicQuadratic.h"

class SimulationMesh;

class SoftIKSolver { 
	friend class SimulationMesh;

public:
	SoftIKSolver(SimulationMesh *);
	inline ~SoftIKSolver() {};
	void draw();

public:
	SimulationMesh *mesh;
 
public:
	bool SPEC_COM = true;
	bool SPEC_FREESTYLE = true;

public:
	P3D COMp;

public:
	void toggle_Z(Node *);
	dVector Z(); 
	dVector Z_01_bool_spoof;
	double Z_ON = 1.;
	double Z_OFF = 0.;
	double Z_SPECIAL = 0. ;
 
public:
	double timeStep = .01;
	// --
	bool PROJECT = false;
	// TODO: bool CHECK_GRADIENT = false;
	bool SOLVE_DYNAMICS = false;
	bool LINEAR_APPROX = true;
	bool VERBOSE = false;

public:
	int D();
	int N();
	int T();

public:
	double     calculate_O(const dVector &alphac);
	double     calculate_Q(const dVector &x);
	double     calculate_R(const dVector &alphac, bool verbose);
	// --
	double calculate_Q_formal(const dVector &alphac, bool verbose);
	double calculate_Q_approx(const dVector &alphac, bool verbose);


public:
	dVector x_0, v_0;
	dVector alphac_curr; dVector x_curr;

public:
	MatrixNxM dtaudalphac;
	MatrixNxM dxdalphac;
 
public:
	void step();
	// --
	int NUM_ITERS_PER_STEP = 5;
	void iterate();
	void project();
 
	dVector alphac_next(const dVector &alphac, const dVector &x);

	dVector calculate_dOdalphac(const dVector &alphac, const dVector &x);
	double calculate_gamma(const dVector &alphac, const dVector &dOdalphac);

	dVector x_of_alphac(const dVector &alphac);

	dVector calculate_dQdx(const dVector &alphac, const dVector &x);
	MatrixNxM calculate_dxdalphac(const dVector &alphac, const dVector &x);
	dVector calculate_dRdalphac(const dVector &alphac, const dVector &x);

	SparseMatrix calculate_A(const dVector &x);
	SparseMatrix calculate_H(const dVector &x, const dVector &alphac);

public: 
	vector<ZeroCubicQuadratic *> alphac_barrierFuncs;
	void construct_alphac_barrierFuncs();

public:
	// NOTE(*): For bringing back to alpha <- alphaz
	bool REGULARIZE_alphac = true;
	double c_alphac_ = .01;// 2.e-1;
	Quadratic *alphac_regFunc = new Quadratic(&c_alphac_);

	// NOTE(*): Honey regularizer
	bool REGULARIZE_honey = false;
	dVector x_honey;
	double c_honey_ = 1.e2;
	Quadratic *honey_regFunc = new Quadratic(&c_honey_);
};