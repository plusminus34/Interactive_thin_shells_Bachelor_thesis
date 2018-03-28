#pragma once 
#include <PlushHelpers/helpers_star.h>

// https://en.wikipedia.org/wiki/Cubic_Hermite_spline

class CubicHermiteSpline {

public:
	CubicHermiteSpline();
	virtual ~CubicHermiteSpline() {}
	void draw();

public:
	P3D SPEC_COLOR = ORCHID;

public: 
	dVector X; // knot times  (x)
	dVector Y; // knot values (y)
	dVector M; // knot tangents (???)
	int Z() {
		if ((X.size() != Y.size()) || (X.size() != M.size())) { error("SizeMismatchError"); }
		return X.size();
	}

public:
	pair<int, double> kt_of_x(const double &x);
	double y_of_x(const double &x);
	double h00(const double &t) { return  2.*pow(t, 3) - 3.*pow(t, 2)     + 1.; }
	double h10(const double &t) { return     pow(t, 3) - 2.*pow(t, 2) + t     ; }
	double h01(const double &t) { return -2.*pow(t, 3) + 3.*pow(t, 2)         ; }
	double h11(const double &t) { return     pow(t, 3) -    pow(t, 2)         ; }
	
};
