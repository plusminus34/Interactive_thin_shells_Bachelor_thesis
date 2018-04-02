#pragma once

#include <PlushHelpers/helpers_star.h>

class XYPlot {

public:
	XYPlot(vector<double>, vector<double>);
	~XYPlot();
	void draw(); int Z = 10;
	P3D *DRAW_SMOOTHED = false;
	P3D SPEC_COLOR = BLACK;

public:
	vector<double> X;
	vector<double> Y;
	vector<double> Y_smooth;
	int N;
	int T; // Window size for moving average (num points)
	P3D *origin    = new P3D(1., 0.);
	P3D *top_right = new P3D(2., .5);
	bool FILLED = false;
	bool DRAW_LINES = true;
	bool DRAW_POINTS = true;
	bool THIN_LINES = false;

public:
	vector<P3D> point_rep_mod(const vector<double> &);

public:
	vector<double> X_L_H = { 0., 1. };
	vector<double> Y_L_H = { 0, 1. };
	double PLOT_WIDTH();
	double PLOT_HALF_HEIGHT();
	double X_MIN;
	double Y_MIN;
	double X_MAX;
	double Y_MAX;
	double X_SCALE();
	double Y_SCALE();

public:
	static void uniformize_axes(vector<XYPlot *>);

};