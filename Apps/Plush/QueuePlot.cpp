#include "QueuePlot.h"
#include <PlushHelpers\error.h>

QueuePlot::QueuePlot(int N, int T) {
	for (int i = 0; i < N; ++i) { X.push_back(double(i) / (N - 1)); }
	for (int _ = 0; _ < N; ++_) { Y.push_back(0.); }
	for (int _ = 0; _ < N; ++_) { Y_smooth.push_back(0.); }
	this->N = N;
	this->T = T;
}

QueuePlot::~QueuePlot() { }

double QueuePlot::PLOT_WIDTH() {
	return top_right->x() - origin->x();
}

double QueuePlot::PLOT_HALF_HEIGHT() {
	return top_right->y() - origin->y(); 
}

void QueuePlot::draw() {
	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		glLineWidth(5.);
		glPointSize(5.);
		set_color(HENN1NK);
		glBegin(GL_POINTS); {
			glP3Dz(*top_right, Z + 1);
		} glEnd();
		// --
		glPushMatrix(); {
			glTranslated(origin->x(), origin->y(), 0);
			glScaled(PLOT_WIDTH(), PLOT_HALF_HEIGHT(), 1.);
			// --
			set_color(CLAY);
			glBegin(GL_LINES); {
				glP3Dz(P3D(X.front(), 0), Z - 1);
				glP3Dz(P3D(X.back(), 0), Z - 1);
				// --
				glP3Dz(P3D(0, Y_L_H[0]), Z - 1);
				glP3Dz(P3D(0, Y_L_H[1]), Z - 1);
			} glEnd();
			// --
			glScaled(1., ONE_OVER_Y_MAX, 1.);
			// --
			set_color((SPEC_COLOR == BLACK) ? RATIONALITY : SPEC_COLOR);
			glBegin(GL_LINE_STRIP); {
				glvecP3Dz(point_rep(Y), Z);
			} glEnd();
			// --
			glBegin(GL_POINTS); {
				glvecP3Dz(point_rep(Y), Z);
			} glEnd();

			// -- // NOTE: Experimental
			if (DRAW_SMOOTHED) {
				set_color(ORCHID);
				glBegin(GL_LINE_STRIP); {
					glvecP3Dz(point_rep(Y_smooth), Z);
				} glEnd();
				// --
				glBegin(GL_POINTS); {
					glvecP3Dz(point_rep(Y_smooth), Z);
				} glEnd();
			}

		} glPopMatrix();
	} glPopAttrib();
}

vector<P3D> QueuePlot::point_rep(const vector<double> &Y_twiddle) {
	if (X.size() != Y_twiddle.size()) { error("X, Y size mismatch."); }
	vector<P3D> XY0;
	for (size_t i = 0; i < X.size(); ++i) {
		XY0.push_back(P3D(X[i], Y_twiddle[i]));
	} 
	return XY0;
}

void QueuePlot::add_new_data_point(const double &c) {
	Y.pop_back(); 
	Y.insert(Y.begin(), c);
	// --
	Y_smooth.pop_back();
	{
		double avg = 0.;
		for (int j = 0; j < T; ++j) {
			avg += Y[j];
		}
		avg /= double(T);
		Y_smooth.insert(Y_smooth.begin(), avg);
	}
	// --
	if (!FILLED) {
		FILLED = true;
		// --
		for (int i = 0; i < N; ++i) {
			Y[i]        = c;
			Y_smooth[i] = c;
		}
	}
}
