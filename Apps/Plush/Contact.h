#pragma once 
// --
#include <GUILib\GLUtils.h>
// --
#include "SimMeshElement.h"
#include "Node.h"
// --
#include "ZeroCubicQuadratic.h"
#include "Quadratic.h"
#include "SmoothestStep.h"
// --
#include <PlushHelpers\helpers_colors.h>

// Forward declarations.
class SimulationMesh; 

class Contact : public SimMeshElement {

public:
	Contact(Node *, SimulationMesh*);
	inline ~Contact() {};
	virtual void draw(const dVector &);
	int contactIndex = -1;

public:
	double SCALE_T_BY_N = true;

public:
	// Ex = b * Q(x; K_x)
	static double K_x;
	// double eps_x = .01;
	static Quadratic *QT;
	// Ey = ZCQ(-y; K_y, eps_y)
	static double K_y;
	static double eps_y;
	static ZeroCubicQuadratic *ZCQ1;

public:
	// b
	double _eps_b = .01;
	SmoothestStep *_SSb = new SmoothestStep(&this->_eps_b);
	double b_(const double &y);
	double b_prime_(const double &y);
	double b() { return b_(prevPosition[1]); }
	double b_prime() { return b_prime_(prevPosition[1]); }

public:
	P3D ACTIVE_COLOR = ORCHID;
	P3D INACTIVE_COLOR = GRAY;

public:
	Node *node = nullptr;
	P3D prevPosition = P3D();
	P3D getPosition(const dVector &);


public:
	bool ACTIVE = false;
	bool FORCE_INACTIVE_FOR_TESTING_DRAWING = false;
	void update(const dVector &);

public:
	bool NO_SLIP = false;

public:
	// -- // helpers
	int D();
	// --
	V3D get_Delta(const dVector &x);
	// --
	double get_E(const dVector &x);
	dVector get_dEdx(const dVector &x);
	MatrixNxM get_ddEdxdx(const dVector &x);
 
public:
	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	
// -- //

public:
	inline double getMass() { return 0.0; };
};
