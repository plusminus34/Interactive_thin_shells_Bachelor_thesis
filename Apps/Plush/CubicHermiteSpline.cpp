#include "CubicHermiteSpline.h"

CubicHermiteSpline::CubicHermiteSpline() {
	X = vecDouble2dVector(linspace(5, 0., 1.));
	Y.setZero(X.size());
	M.setZero(X.size());

	auto F = [&](const int &k) {
		return (Y[k] - Y[k - 1]) / (X[k] - X[k - 1]);
	};

	for (int k = 0; k < Z(); ++k) {
		Y[k] = .33*random_double();
	}

	for (int k = 0; k < Z(); ++k) {
		if (k == 0) {
			M[k] = F(k + 1);
		} else if (k == Z() - 1) {
			M[k] = F(k);
		} else {
			M[k] = .5*(F(k + 1) + F(k));
		}
	}
}
 
void CubicHermiteSpline::draw() {
	glMasterPush(); {
		glPointSize(5.);
		glBegin(GL_POINTS); {
			for (int i = 0; i < Z(); ++i) {
				glP3D(P3D(X[i], Y[i]));
			}
		} glEnd();

		vector<double> x_vec = linspace(100, X[0], X[X.size() - 1]);
		glLineWidth(2.);
		glBegin(GL_LINE_STRIP); {
			for (auto x : x_vec) {
				glP3D(P3D(x, y_of_x(x))); 
			} 
		} glEnd();
	} glMasterPop();
} 

pair<int, double> CubicHermiteSpline::kt_of_x(const double &x) {
	int k = -1;
	double t = 0.;
	{
		if ((x < X[0]) || (x > X[X.size() - 1])) { error("[t_of_x] : IOError"); }
		for (int i = 0; i < Z() - 1; ++i) {
			int ip1 = i + 1;
			// --
			double &xi = X[i];
			double &xip1 = X[ip1];
			if ((x >= xi) && (x <= xip1)) {
				k = i;
				t = interval_fraction(x, xi, xip1);
				return make_pair(k, t);
			}
		}
	}
	error("Failed to find kt.");
	return make_pair(k, t);
}

double CubicHermiteSpline::y_of_x(const double &x) { 
	auto kt = kt_of_x(x);
	int    &k = kt.first;
	double &t = kt.second;
	// --
	double dx = X[k + 1] - X[k];
	return h00(t)*Y[k] + h10(t)*dx*M[k] + h01(t)*Y[k + 1] + h11(t)*dx*M[k + 1]; 
}

