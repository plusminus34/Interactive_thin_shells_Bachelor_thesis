#pragma once 
#include <PlushHelpers/helpers_star.h>

// A minimal cubic hermit spline class for reparameterizing trajectories.
// Notation: https://en.wikipedia.org/wiki/Cubic_Hermite_spline

class CubicHermiteSpline {

public:
	CubicHermiteSpline(const dVector &, const dVector &);
	virtual ~CubicHermiteSpline() {}
	void draw(const dVector &Y);

public:
	P3D SPEC_COLOR = ORCHID;

public: 
	// (x, y)
	dVector X;   // knot x positions
	// dVector Y // knot y positions
	dVector M;   // knot tangents
	// --
	int X_SIZE() {
		if (X.size() != M.size()) { error("SizeMismatchError"); }
		return X.size();
	}

public:
	// (s, u)
	dVector S;     // data x positons
	// dVector U   // data y positions
	vector<int> K; // data k indices (see Wiki)
	dVector T;     // data t values  (see Wiki)
	// --
	int S_SIZE() {
		if ((S.size() != K.size()) || (S.size() != T.size())) { error("SizeMismatchError"); }
		return S.size();
	}
	// --
	void populate_KT();

public:
	pair<int, double> calculate_kt(const double &s);
	dVector calculate_U(const dVector &Y);
	vector<SparseMatrix> dudyJ;
	// --
	SparseMatrix calculate_dUdY_();
	// --
	bool checkJacobian(const dVector &Y);

public:
	double h00(const double &t) { return  2.*pow(t, 3) - 3.*pow(t, 2)     + 1.; }
	double h10(const double &t) { return     pow(t, 3) - 2.*pow(t, 2) + t     ; }
	double h01(const double &t) { return -2.*pow(t, 3) + 3.*pow(t, 2)         ; }
	double h11(const double &t) { return     pow(t, 3) -    pow(t, 2)         ; }

public:
	dVector randY();
	void assign_CFD2M(dVector &Y);
	
};
