#include "CubicHermiteSpline.h"

CubicHermiteSpline::CubicHermiteSpline(const dVector &X, const dVector &S) {
	if (X.size() > S.size()) { error("ArgumentOrderError"); }
	if ((!IS_EQUAL(X[0], S[0])) || (!IS_EQUAL(X[X.size() - 1], S[S.size() - 1]))) { error("Knot and data ranges don't meet at the ends."); }

	this->X = X;
	M.setZero(X.size());
	// --
	this->S = S;
	populate_KT(); 
}
 
void CubicHermiteSpline::draw(const dVector &Y) {
	glMasterPush(); {
		glTranslated(0., 0., .33);
		// --
		set_color(SPEC_COLOR);

		glPointSize(5.);
		glBegin(GL_POINTS); {
			glvecP3D(zip_vec_dVector2vecP3D({ X, Y }));
		} glEnd();
		// --
		glLineWidth(2.);
		glPointSize(2.);
		for (auto &GL_PRIMITIVE : { GL_LINE_STRIP, GL_POINTS }) {
			glBegin(GL_PRIMITIVE); {
				glvecP3D(zip_vec_dVector2vecP3D({ S, calculate_U(Y) }));
			} glEnd();
		}
	} glMasterPop();
} 

void CubicHermiteSpline::populate_KT() {
	K.clear();
	vector<double> T_vec;
	for (int i = 0; i < S.size(); ++i) {
		auto kt = calculate_kt(S[i]);
		int    &k = kt.first;
		double &t = kt.second;
		// --
		K.push_back(k);
		T_vec.push_back(t);;
	}
	T = vecDouble2dVector(T_vec);
}

pair<int, double> CubicHermiteSpline::calculate_kt(const double &s) {
	int k = -1;
	double t = 0.;
	{
		if ((s < X[0]) || (s > X[X.size() - 1])) { error("[t_of_x] : IOError"); }
		for (int i = 0; i < X_SIZE() - 1; ++i) {
			int ip1 = i + 1;
			// --
			double &xi = X[i];
			double &xip1 = X[ip1];
			if ((s >= xi) && (s <= xip1)) {
				k = i;
				t = interval_fraction(s, xi, xip1);
				return make_pair(k, t);
			}
		}
	}
	error("Failed to find kt.");
	return make_pair(k, t);
}

dVector CubicHermiteSpline::calculate_U(const dVector &Y) {
	dVector U; U.setZero(S_SIZE());
	for (int i = 0; i < U.size(); ++i) {
		int    &k = K[i];
		double &t = T[i];
		// --
		double dx = X[k + 1] - X[k];
		U[i] = h00(t)*Y[k] + h10(t)*dx*M[k] + h01(t)*Y[k + 1] + h11(t)*dx*M[k + 1];
	}
	return U;
}

SparseMatrix CubicHermiteSpline::calculate_dUdY_() {
	vector<MTriplet> triplets;
	SparseMatrix dUdY_; dUdY_.resize(S_SIZE(), X_SIZE());
	for (int i = 0; i < dUdY_.rows(); ++i) {     // U[i]
		int    &k = K[i];
		double &t = T[i];
		triplets.push_back(MTriplet(i, k, h00(t)));
		triplets.push_back(MTriplet(i, k + 1, h01(t)));
	}
	dUdY_.setFromTriplets(triplets.begin(), triplets.end());
	return dUdY_;
}

// TODO: calculate_dUDM
// TODO: calculate_dUDYM // NOTE: Should just stack

bool CubicHermiteSpline::checkJacobian(const dVector &Y) {
	auto &U_wrapper = [this](const dVector &Y) {return calculate_U(Y); };
	return matrix_equality_check(mat_FD(Y, U_wrapper), calculate_dUdY_().toDense());
}

dVector CubicHermiteSpline::randY() {
	dVector Y; Y.setZero(X_SIZE());
	for (int k = 0; k < Y.size(); ++k) {
		Y[k] = .33*random_double();
	}
	return Y;
}

void CubicHermiteSpline::assign_CFD2M(dVector &Y) {
	auto F = [&](const int &k) { return (Y[k] - Y[k - 1]) / (X[k] - X[k - 1]); };
	for (int k = 0; k < X_SIZE(); ++k) {
		if (k == 0) {
			M[k] = F(k + 1);
		} else if (k == X_SIZE() - 1) {
			M[k] = F(k);
		} else {
			M[k] = .5*(F(k + 1) + F(k));
		}
	} 
}
