#pragma once

#include <PlushHelpers/helpers_star.h>

class QueuePlot {

public:
	QueuePlot(int N, int T);
	~QueuePlot();
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

public:
	void add_new_data_point(const double &);

public:
	vector<P3D> point_rep(const vector<double> &);

public:
	vector<double> Y_L_H = { -1, 1. };
	double PLOT_WIDTH();
	double PLOT_HALF_HEIGHT();
	double ONE_OVER_Y_MAX = 1.;

};