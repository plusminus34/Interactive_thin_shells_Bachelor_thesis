#include "SmoothestStep.h"
#include <PlushHelpers\helpers_star.h>
#include "XYPlot.h"
 
// we will use:                         t \in [-eps, eps]
// we obtain from standard formulation: s \in [   0,   1]
// ?: [-eps, eps] -> [0, 1]
// => .5*(t/eps + 1) = s

// from f(s), f'(s), f''(s) we blindly compute f(t), f'(t), f''(t) using mathematica :)

SmoothestStep::SmoothestStep(double *eps_ptr) {
	this->eps_ptr = eps_ptr;
}

SmoothestStep::SmoothestStep(double eps) {
	this->eps__ = eps;
	this->eps_ptr = &eps__;
}

double SmoothestStep::g(const double &t_) {
	double eps = *eps_ptr;
	double t = clamp(t_, -eps, eps);
	// --
	return
		  ( 1. / 2.)
		- ( 5. / 32.)*pow(t, 7)*pow(eps, -7)
		+ (21. / 32.)*pow(t, 5)*pow(eps, -5)
		- (35. / 32.)*pow(t, 3)*pow(eps, -3)
		+ (35. / 32.)*pow(t, 1)*pow(eps, -1);
}

double SmoothestStep::g1(const double &t_) {
	double eps = *eps_ptr;
	double t = clamp(t_, -eps, eps);
	// --
	return
		- (35.  / 32.)*pow(t, 6)*pow(eps, -7)
		+ (105. / 32.)*pow(t, 4)*pow(eps, -5)
		- (105. / 32.)*pow(t, 2)*pow(eps, -3)
		+ (35.  / 32.)          *pow(eps, -1);
}

double SmoothestStep::g2(const double &t_) {
	double eps = *eps_ptr;
	double t = clamp(t_, -eps, eps);
	// --
	return
		- (105. / 16.)*pow(t, 5)*pow(eps, -7)
		+ (105. /  8.)*pow(t, 3)*pow(eps, -5)
		- (105. / 16.)*pow(t, 1)*pow(eps, -3);
}

void SmoothestStep ::draw() {
	double eps = *eps_ptr;
	// --
	vector<double> T = linspace(100, -1.5*eps, 1.5*eps);
	vector<double> Y, Yp, Ypp;
	for (const auto &t : T) {
		Y.push_back(g(t));
		Yp.push_back(g1(t));
		Ypp.push_back(g2(t));
	}

	XYPlot Y_plot = XYPlot(T, Y);   Y_plot.SPEC_COLOR = ORCHID;
	XYPlot Yp_plot = XYPlot(T, Yp); Yp_plot.SPEC_COLOR = PUMPKIN;
	XYPlot Ypp_plot = XYPlot(T, Ypp); Ypp_plot.SPEC_COLOR = GOLDCLOVER;
	vector<XYPlot *> plots = { &Y_plot, &Yp_plot, &Ypp_plot };
	XYPlot::uniformize_axes(plots);
	for (auto &plot : plots) {
		plot->draw();
	} 
}
 