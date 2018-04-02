#include "XYPlot.h"
#include <PlushHelpers\error.h>

XYPlot::XYPlot(vector<double> X, vector<double> Y) {
	this->X = X;
	this->Y = Y;
	// --
	X_MIN = min_element(X);
	Y_MIN = min_element(Y);
	// --
	X_MAX = max_element(X);
	Y_MAX = max_element(Y);
}

XYPlot::~XYPlot() { }

double XYPlot::X_SCALE() {
	return 1. / (X_MAX - X_MIN);
}

double XYPlot::Y_SCALE() {
	return 1. / (Y_MAX - Y_MIN);
} 

double XYPlot::PLOT_WIDTH() {
	return top_right->x() - origin->x();
}

double XYPlot::PLOT_HALF_HEIGHT() {
	return top_right->y() - origin->y(); 
}

void XYPlot::draw() {
	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		glLineWidth((!THIN_LINES) ? 5.f : 2.f);
		glPointSize(5.);
		set_color(HENN1NK);
		glBegin(GL_POINTS); {
			glP3Dz(*origin,    Z + 1);
			glP3Dz(*top_right, Z + 1);
		} glEnd();
		// --
		glPushMatrix(); {
			glTranslated(origin->x(), origin->y(), 0);
			glScaled(PLOT_WIDTH(), PLOT_HALF_HEIGHT(), 1.);
			// --
			set_color(CLAY);
			glBegin(GL_LINES); {
				glP3Dz(P3D(X_L_H[0], 0), Z - 1);
				glP3Dz(P3D(X_L_H[1], 0), Z - 1);
				// --
				glP3Dz(P3D(0, Y_L_H[0]), Z - 1);
				glP3Dz(P3D(0, Y_L_H[1]), Z - 1);
			} glEnd();
			// --
			glScaled(X_SCALE(), Y_SCALE(), 1.);
			// --
			set_color((SPEC_COLOR == BLACK) ? RATIONALITY : SPEC_COLOR);
			if (DRAW_LINES) {
				glBegin(GL_LINE_STRIP); {
					glvecP3Dz(point_rep_mod(Y), Z);
				} glEnd();
			}
			// --
			if (DRAW_POINTS) {
				glBegin(GL_POINTS); {
					glvecP3Dz(point_rep_mod(Y), Z);
				} glEnd();
			}
		} glPopMatrix();
	} glPopAttrib();
}

vector<P3D> XYPlot::point_rep_mod(const vector<double> &Y_twiddle) {
	if (X.size() != Y_twiddle.size()) { error("X, Y size mismatch."); }
	vector<P3D> XY0;
	for (size_t i = 0; i < X.size(); ++i) {
		XY0.push_back(P3D(X[i] - X_MIN, Y_twiddle[i] - Y_MIN));
	} 
	return XY0;
}

void XYPlot::uniformize_axes(vector <XYPlot *> plots) {
	vector<double> X_MIN_vec;
	vector<double> Y_MIN_vec;
	vector<double> X_MAX_vec;
	vector<double> Y_MAX_vec;
	for (auto &plot : plots) { 
		X_MIN_vec.push_back(plot->X_MIN);
		Y_MIN_vec.push_back(plot->Y_MIN);
		X_MAX_vec.push_back(plot->X_MAX);
		Y_MAX_vec.push_back(plot->Y_MAX);
	} 
	// --
	for (auto &plot : plots) { 
		plot->X_MIN = min_element(X_MIN_vec);
		plot->Y_MIN = min_element(Y_MIN_vec);
		plot->X_MAX = max_element(X_MAX_vec);
		plot->Y_MAX = max_element(Y_MAX_vec);
	} 
};

