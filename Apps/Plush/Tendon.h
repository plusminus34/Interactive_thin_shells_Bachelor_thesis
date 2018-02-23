#pragma once
// --
#include <PlushHelpers/helpers_star.h>
// -- 
#include "Point.h"
#include "SimMeshElement.h"
#include "Node.h"
#include "ScalarFunction.h"
#include "ZeroCubicQuadratic.h"
#include "Quadratic.h"

#include <MathLib/mathLib.h>
#include <MathLib/Matrix.h>

// Forward declarations.
class SimulationMesh;

class Tendon : public SimMeshElement {

public:
	Tendon(SimulationMesh* plushy, vector<Point *> waypoints);
	~Tendon();
	virtual void draw(const dVector &x, const dVector &balpahc=dVector());
	// --
	P3D SPEC_COLOR = BLACK;

public:
	// vector<Node *> nodes; // FORNOW

// core suite
public:
	vector<Point *> waypoints;
	vector<int> IC; // nodalIndexConvention;
	// --
	int tendonIndex = -1;
	// --
	double get_alpha( const dVector &balphac);
	double get_alphaz();
	double get_alphac(const dVector &balphac);
	// --
	void DEPRECATED_set_alpha(const double &);
	void set_alphac(const double &);
	void set_alphac_over_alphaz(const double &);
	// --
	double get_L(const dVector &x);
	double get_Gamma(const dVector &x, const dVector &balphac);
	double get_tau(const dVector &x, const dVector &balphac);

	// -- // definitions suite
	// --
	static inline double alpha_of_Gamma_L(const double &Gamma, const double &L) {
		return L - Gamma; 
	}
	// --
	static inline double L_of_alpha_Gamma(const double &alpha, const double &Gamma) {
		return alpha + Gamma;
	}
	// --
	static inline double Gamma_of_alpha_L(const double &alpha, const double &L) {
		return L - alpha; 
	}
	// --
	static inline double alpha_of_alphac_alphaz(const double &alphac, const double &alphaz) {
		return alphaz - alphac;
	}
	// --
	static inline double alphac_of_alpha_alphaz(const double &alpha, const double &alphaz) {
		return alphaz - alpha;
	}
	// --
	static inline double alphaz_of_alpha_alphac(const double &alpha, const double &alphac) {
		return alpha + alphac;
	}
	// --

// special tactics maff
public:
	int D();
	MatrixNxM MatrixDxD();
	MatrixNxM outerDxD(const V3D &, const V3D &);
	MatrixNxM xxTDxD(const V3D &);

public:
	double c, eps;
	ScalarFunction *tendon_energy_model();
	ScalarFunction *quadratic_E;
	ScalarFunction *zcq_E;

	// // Energy computation.
	virtual double getEnergy(const dVector &x, const dVector &balphac);
	double get_dEdDelta(const dVector &x, const dVector &balphac);
	double get_d2Ed2Delta(const dVector &x, const dVector &balphac);
	void computeFirstDeltaPartials(const dVector &x, const dVector &balphac);
	void computeSecondDeltaPartials(const dVector &x, const dVector &balphac);
	void computeGradientComponents(const dVector &x, const dVector &balphac);
	void computeHessianComponents(const dVector &x, const dVector &balphac);
	// Parameters.
	double E_FACTOR, L_FRACTION;
	// Shell variables.
	// vector<V3D> D1; // Delta first partials
	// vector<vector<MatrixNxM>> D2; // Delta second partials // @
	std::map<int, V3D> D1; // Delta first partials
	std::map<std::pair<int, int>, MatrixNxM> D2; // Delta second partials // @
	// vector<V3D> dEdx;
	// vector<vector<MatrixNxM>> ddEdxdx; // @
	std::map<int, V3D> dEdx;
	std::map<std::pair<int, int>, MatrixNxM> ddEdxdx; // @
	// --
	void addToD1(int a,        const V3D       &v);
	void addToD2(int a, int b, const MatrixNxM &m);

	// Storage into grad and hesEntries.
	virtual void addEnergyGradientTo(const dVector& x, const dVector& balphac, dVector& grad);
    virtual void addEnergyHessianTo(const dVector& x, const dVector& balphac, std::vector<MTriplet>& hesEntries);
 
	// (Unused) virtual functions to override.
	inline double getMass() { return 0.0; };

};
