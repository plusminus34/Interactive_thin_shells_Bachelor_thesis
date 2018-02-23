#include "ZeroCubicQuadratic.h"
#include <PlushHelpers/helpers_star.h>
#include <PlushHelpers/error.h>

ZeroCubicQuadratic::ZeroCubicQuadratic(double *c, double *eps, V3D t, bool negate_x, bool negate_y) {
	// (0) zero for x <= 0; cubic for x \in (0, eps); quadratic for x > eps
	// (1) multiply by f (either no-op or flip over x axis)
	// (2) translate by <tx, ty>
	this->c_ptr = c;
	this->eps_ptr = eps;

	// tx is the boundary of flat and cubic
	// ty is value of y for the flat region
	// i.e. f(tx) = ty
	this->tx = t[0];
	this->ty = t[1];

	// flips can be thought of as applied _before_ applying translations
	this->fx = (negate_x) ? -1. : 1.;
	this->fy = (negate_y) ? -1. : 1.;
}

ZeroCubicQuadratic::ZeroCubicQuadratic(const double &c, const double &eps, V3D t, bool negate_x, bool negate_y) {
	this->c_dummy   = c;   this->c_ptr   = &c_dummy;
	this->eps_dummy = eps; this->eps_ptr = &eps_dummy;
	this->tx = t[0];
	this->ty = t[1];
	this->fx = (negate_x) ? -1. : 1.;
	this->fy = (negate_y) ? -1. : 1.;
}

ZeroCubicQuadratic::~ZeroCubicQuadratic() {
}

void ZeroCubicQuadratic::build_locals(double &eps, double &c, double &r1, double &r0, double &m3) {
	eps = *eps_ptr;
	c = *c_ptr;
	r1 = -c*eps;
	r0 = c*eps*eps/3.;
	m3 = c/(3.*eps);
}

double ZeroCubicQuadratic::computeValue(double x) {
	double eps, c, r1, r0, m3;
	build_locals(eps, c, r1, r0, m3);
	x = fx*(x - tx);
	
	double y;
	if (x <= 0.) {
		y = 0.;
	} else if (x <= eps) {
		y = m3*x*x*x;
	} else {
		y = c*x*x + r1*x + r0;
	}

	return (fy*y) + ty;
}

double ZeroCubicQuadratic::computeDerivative(double x) {
	double eps, c, r1, r0, m3;
	build_locals(eps, c, r1, r0, m3);
	x = fx*(x - tx);

	double yp;
	if (x <= 0.) {
		yp = 0.;
	} else if (x <= eps) {
		yp = m3*3*x*x;
	} else {
		yp = c*2*x + r1;
	}

	return fx*(fy*yp);
}

double ZeroCubicQuadratic::computeSecondDerivative(double x) {
	double eps, c, r1, r0, m3;
	build_locals(eps, c, r1, r0, m3);
	x = fx*(x - tx);

	double ypp;
	if (x <= 0.) {
		ypp = 0.;
	} else if (x <= eps) {
		ypp = m3*6*x;
	} else {
		ypp = c*2;
	}

	return fx*fx*(fy*ypp);
}


double ZeroCubicQuadratic::invertValue(double y) {
	double eps, c, r1, r0, m3;
	build_locals(eps, c, r1, r0, m3);
	y = fy*(y - ty);

	double y_thresh = computeValue((fx*eps) + tx);
	y_thresh = (y_thresh - ty) / fy;

	double x;
	if (y <= 0.) {
		if (abs(y) > 1e-5) { error("Inversion impossible, defaulting to 0."); }
		x = 0.;
	} else if (y <= y_thresh) {
		x = pow(3.*y*eps/c, 1./3.);
	} else {
		// NOTE: abs hack to prevent rooting -0.
		x = (3*c*eps + sqrt(3.*c*abs(12.*y-c*eps*eps))) / (6.*c);
	}

	return (fx*x) + tx;
}


void ZeroCubicQuadratic::report_parameters() {
	cout << "-------------------------------------" << endl;
	cout << "c:   "   << *c_ptr  << endl;
	cout << "eps: " << *eps_ptr  << endl;
	cout << "tx:  "  << tx       << endl;
	cout << "ty:  "  << ty       << endl;
	cout << "fx:  "  << fx       << endl;
	cout << "fy:  "  << fy       << endl;
}

void ZeroCubicQuadratic::plot_self() {
	return;
	/*
	// NOTE: We flip x in these lambdas.
	const auto vdb_plot = [](const vector<double> &X, const vector<double> &Y) {
		for (size_t i = 0; i < X.size() - 1; ++i) {
			int ip1 = i + 1;
			vdb_line(-float(X[i]), float(Y[i]), 0.f, -float(X[ip1]), float(Y[ip1]), 0.f);
		}
	};

	const auto vdb_2d_line = [](const double x0, const double y0, const double x1, const double y1) {
		vdb_line(-float(x0), float(y0), 0.f, -float(x1), float(y1), 0.f);
	};

	double eps = *eps_ptr;
	double c   = *c_ptr;
	double x_max = invertValue(f*.1); // FORNOW, remove 2 fac after debugging invert Value
	vector<double> X = linspace(100, tx, tx + 1.5*eps);
	// y, y', y''
	vector<double> Y, Yp, Ypp;
	for (auto &x : X) {
		Y.push_back(computeValue(x));
		Yp.push_back(computeDerivative(x));
		Ypp.push_back(computeSecondDerivative(x));
	}

	// [y]^{-1}
	vector<double> Y_inv;
	for (auto &y : Y) {
		Y_inv.push_back(invertValue(y));
	}
	// [y']^{-1}
	vector<double> Yp_inv;
	for (auto &yp : Yp) {
		Yp_inv.push_back(invertDerivative(yp));
	}

	vdb_frame();

	// // axes
	vdb_label("axes");
	vdb_color(.5, .5, .5);
	vdb_2d_line(X[0], 0., X[X.size() - 1], 0.); // x-axis
	vdb_2d_line(0., 0., 0., .1); // y-axis

	// tx, ty
	vdb_color(1., 0., 1.);
	vdb_2d_line(X[0], ty, X[X.size() - 1], ty); // x-axis
	vdb_2d_line(tx, 0., tx, .1); // y-axis

	// x = eps
	vdb_color(0., .5, 0.);
	vdb_2d_line(eps, 0., eps, -.1);
	vdb_color(0., 1., 0.);
	vdb_2d_line(tx + eps, 0., tx + eps, -.1);

	// y
	vdb_label("y");
	vdb_color(1., 0., 0.);
	vdb_plot(X, Y);

	// y'
	// vdb_label("y'");
	// vdb_color(1., .5, 0.);
	// vdb_plot(X, Yp);

	// y''
	// vdb_label("y''");
	// vdb_color(1., 1., 0.);
	// vdb_plot(X, Ypp);
	
	// [y]^{-1}
	for (size_t i = 0; i < X.size(); ++i) {
		double x = X[i];
		double y = Y[i];
		double y_inv = Y_inv[i];
		vdb_color(1., 1., 0.);
		vdb_2d_line(x, ty, x, y);
		vdb_color(1., 1., 1.);
		vdb_2d_line(tx, y, y_inv, y);
	}
	// [y']^{-1}
	// for (size_t i = 0; i < X.size(); ++i) {
		// double x = X[i];
		// double yp = Yp[i];
		// double yp_inv = Yp_inv[i];
		// vdb_color(0., 1., 1.);
		// vdb_2d_line(x, 0., x, yp);
		// vdb_color(1., 0., 1.);
		// vdb_2d_line(0, yp, yp_inv, yp);
	}
	*/
}

void ZeroCubicQuadratic::test_self() {

	
	cout << "-------------------------------------" << endl;
	cout << "-- BEGINNING SELF TEST --------------" << endl;
	cout << "-------------------------------------" << endl;

	double eps = *eps_ptr;
	double buffer = eps;
	vector<double> X = linspace(10000, tx -3*eps, tx + 3*eps );

	vector<double> Y, Yp, Ypp, Yinv;
	for (auto &x : X) {
		Y.push_back(computeValue(x));
		Yp.push_back(computeDerivative(x));
		Ypp.push_back(computeSecondDerivative(x));
		Yinv.push_back(computeValue(invertValue(computeValue(x))));
	}

	double dx = 1e-6;
	vector<double> Yp_fd, Ypp_fd;
	for (auto &x : X) {
		Yp_fd.push_back((computeValue(x + dx) - computeValue(x - dx)) / (2.*dx));
		Ypp_fd.push_back((computeValue(x + dx) - 2.*computeValue(x) + computeValue(x - dx)) / (dx*dx));
	}

	double thresh = 1e-3;
	bool Yp_check  = vector_equality_check(Yp, Yp_fd);
	bool Ypp_check = vector_equality_check(Ypp, Ypp_fd);
	bool Yinv_check = true;
	for (size_t i = 0; i < X.size(); ++i) {


		// if (abs(Yp[i] - Yp_fd[i]) > thresh) {
		// 	Yp_check = false;
		// 	cout << "Yp_" << i << " fails." << endl;
		// 	cout << "X_"        << i << "= " <<     X[i] << endl;
		// 	cout << "---Yp_"    << i << "= " <<    Yp[i] << endl;
		// 	cout << "---Yp_fd_" << i << "= " << Yp_fd[i] << endl;
		// }

		// if (abs(Ypp[i] - Ypp_fd[i]) > thresh) {
		// 	Ypp_check = false;
		// 	std::ios_base::fmtflags oldflags = std::cout.flags();
		// 	std::streamsize oldprecision = std::cout.precision();
		// 	{
		// 		std::cout << std::setprecision(5) << std::fixed;
		// 		// --
		// 		cout << "Ypp_" << i << " fails." << endl;
		// 		cout << "X_" << i << " =" << X[i] << endl;
		// 		cout << "---Ypp_" << i << " =" << Ypp[i] << endl;
		// 		cout << "---Ypp_fd_" << i << " =" << Ypp_fd[i] << endl;
		// 	}
		// 	std::cout.flags (oldflags);
		// 	std::cout.precision (oldprecision);
		// }

		if (abs(Yinv[i] - Y[i]) > thresh) {
			Yinv_check = false;
			cout << "Yinv_" << i << " fails." << endl;
			cout << "X_"         << i << " =" <<      X[i] << endl;
			cout << "---Yinv"   << i << " =" <<  Yinv[i] << endl;
			cout << "---Y"       << i << " =" <<      Y[i] << endl;
		}
	}

	cout << "-------------------------------------" << endl;
	if (Yp_check && Ypp_check && Yinv_check) {
		cout << "ZCQ _PASSED_ for given parameters." << endl;
	} else {
		cout << "ZCQ _FAILED_ for given parameters." << endl;
		cout << "Yp_check:" << ((Yp_check) ? "PASS" : "FAIL") << endl;
		cout << "Ypp_check:" << ((Ypp_check) ? "PASS" : "FAIL") << endl;
		cout << "Yinv_check:" << ((Yinv_check) ? "PASS" : "FAIL") << endl;
	}

	report_parameters();
}

/*
double ZeroCubicQuadratic::computeThirdDerivative(double x) {
	double eps, c, r1, r0, m3;
	build_locals(eps, c, r1, r0, m3);
	x = (x - tx) * fx;

	double yppp;
	if (x <= 0.) {
		yppp = 0.;
	} else if (x <= eps) {
		yppp = m3 * 6;
	} else {
		yppp = 0;
	}

	return fx*fx*fx*(fy*yppp);
}

double ZeroCubicQuadratic::invertDerivative(double yp) {
	double eps, c, r1, r0, m3;
	build_locals(eps, c, r1, r0, m3);
	yp = (yp / fy) / fx; // return fx*(fy*yp);

	double yp_thresh = computeDerivative(eps + tx);
	yp_thresh = (yp_thresh / fy) / fx;

	double x;
	if (yp <= 0.) {
		x = 0.;
	} else if (yp <= yp_thresh) {
		// NOTE: abs hack to prevent rooting -0.
		x = sqrt(abs(yp*eps/c));
	} else {
		x = (yp + c*eps) / (2.*c);
	} 

	return (fx*x) + tx;
}
*/