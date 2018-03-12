#include "Sigmoid.h"
#include <PlushHelpers/helpers_star.h>
#include <PlushHelpers/error.h>
#include "XYPlot.h"

Sigmoid::Sigmoid(const double &bot, const double &top, const double &yoz) {
	this->bot = bot;
	this->top = top;
	this->yoz = yoz;

	// These are the asymptotes of the sigmoid.
	if (top <= bot) { error("Are top / bot flipped?"); }
	this->sy = top - bot;
	this->ty = bot;
	
	// This is y(x = 0).
	if ((yoz <= bot) || (yoz >= top)) { error("Bad y(0)."); }
	if (!IS_EQUAL(yoz, bot) && !IS_EQUAL(yoz, top)) {
		// y(0) = yoz
		this->tx = -invertPrimal(unmap_y(yoz));
	} else {
		error("Sigmoid bottomed/topped out.");
		this->tx = -invertPrimal(unmap_y(.5*(bot + top)));
	}
}

Sigmoid::~Sigmoid() {
}

double Sigmoid::computePrimal(double X) {
	// Y \in [0, 1]
	return exp(X) / (1. + exp(X));
}

double Sigmoid::invertPrimal(double Y) {
	// https://en.wikipedia.org/wiki/Logit
	return log(Y / (1. - Y));
}

double Sigmoid::map_Y(double Y) {
	// y \in [bot, top]
	return sy*Y + ty;
}

double Sigmoid::map_Ypx(double Y) {
	// y \in [bot, top]
	return sy*Y;
}

double Sigmoid::unmap_y(double y) {
	// y \in [bot, top]
	return (y - ty) / sy;
}

double Sigmoid::map_X(double X) {
	return X + tx;
}

double Sigmoid::unmap_x(double x) {
	return x - tx;
}

////////////////////////////////////////////////////////////////////////////////

double Sigmoid::computeValue(double x) {
	return map_Y(computePrimal(unmap_x(x))); 
}

double Sigmoid::computeDerivative(double x) {
	double X = unmap_x(x);
	double primal = computePrimal(X);
	double primalDerivative = primal * (1. - primal);
	return map_Ypx(primalDerivative);
}

double Sigmoid::computeSecondDerivative(double x) {
	double X = unmap_x(x);
	double primal = computePrimal(X);
	double primalDerivative = primal * (1. - primal);
	double primalSecondDerivative = primalDerivative * (1. - 2*primal);
	return map_Ypx(primalSecondDerivative);
}
 
double Sigmoid::invertValue(double y) {
	return map_X(invertPrimal(unmap_y(y)));
}


void Sigmoid::report_parameters() {
	return;
}

void Sigmoid::plot_self() {
	double xL = -10.;
	double xR = 10.; vector<double> X = linspace(1000, xL, xR);
	vector<double> Y;
	for (auto &x : X) { Y.push_back(computeValue(x)); }
	XYPlot *sig_plot = new XYPlot(X, Y);
	XYPlot *bot_plot = new XYPlot({ xL, xR }, { bot, bot });
	XYPlot *top_plot = new XYPlot({ xL, xR }, { top, top });
	XYPlot *yoz_plot = new XYPlot({ 0., 0. }, { yoz, 0. });
	sig_plot->SPEC_COLOR = ORCHID;
	bot_plot->SPEC_COLOR = RATIONALITY;
	top_plot->SPEC_COLOR = RATIONALITY;
	yoz_plot->SPEC_COLOR = HENN1NK;
	auto plots = { sig_plot, bot_plot, top_plot, yoz_plot };
	XYPlot::uniformize_axes(plots);
	for (auto &plot : plots) { plot->draw(); }
}

void Sigmoid::test_self() {

	
	cout << "-------------------------------------" << endl;
	cout << "-- BEGINNING SELF TEST --------------" << endl;
	cout << "-------------------------------------" << endl;

	vector<double> X = linspace(10000, tx -100., tx + 100. );

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
