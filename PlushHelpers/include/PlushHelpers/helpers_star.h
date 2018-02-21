#pragma once

#include <GUILib\GLIncludes.h>

#include <iostream>
#include <iomanip> 
#include <vector>
#include <ctime>
#include <algorithm>
#include <stack>
#include <memory>
#include <numeric> 

#include <MathLib/MathLib.h>
#include <MathLib/P3D.h>
#include <MathLib/Ray.h>
#include <MathLib/Quaternion.h>

#include <PlushHelpers/helpers_colors.h>
#include <PlushHelpers/helpers_colormaps.h>
#include <PlushHelpers/error.h>

////////////////////////////////////////////////////////////////////////////////
// namespace ///////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

using std::vector;
using std::map;
using std::cout;
using std::endl;
using std::max;
using std::min;
using std::clock_t;
using std::clock;
using std::pair;
using std::make_pair;
using std::string;
using std::shared_ptr;

const auto vector_equality_check = [] (const auto &fd, const auto &anal) {
	bool ret = true;

	if (fd.size() != anal.size()) {
		error("vec_check: sizes do _not_ match.");
		return false;
	}

	Logger::print("vec_check...\n");
	for (int i = 0; i < int(fd.size()); ++i) {
		double err = fd[i] - anal[i];
		if (fabs(err) > 0.0001 && 2 * fabs(err) / (fabs(fd[i]) + fabs(anal[i])) > 0.001) {
			ret = false; 
			// --
			Logger::logPrint("Mismatch element %d: Anal val: %lf, FD val: %lf. Error: %lf\n", i, anal[i], fd[i], err);
			Logger::print(   "Mismatch element %d: Anal val: %lf, FD val: %lf. Error: %lf\n", i, anal[i], fd[i], err);
		}
	}

	return ret;
};

const auto matrix_equality_check = [] (const MatrixNxM &fd, const MatrixNxM &anal) {
	bool ret = true;

	if (fd.rows() != anal.rows() || fd.cols() != anal.cols()) {
		error("mat_check: sizes do _not_ match");
		return false;
	}

	Logger::print("mat_check...\n");
	for (int i = 0; i < fd.rows(); ++i) {
		for (int j = 0; j < fd.cols(); ++j) {
			double err = fd(i, j) - anal(i, j);
			if (fabs(err) > 0.0001 && 2 * fabs(err) / (fabs(fd(i, j)) + fabs(anal(i, j))) > 0.001) {
				ret = false; 
				// --
				Logger::logPrint("Mismatch element (%d, %d): Anal val: %lf, FD val: %lf. Error: %lf\n", i, j, anal(i, j), fd(i, j), err);
				Logger::print(   "Mismatch element (%d, %d): Anal val: %lf, FD val: %lf. Error: %lf\n", i, j, anal(i, j), fd(i, j), err);
			}
		}
	}

	return ret;
};
 
const auto cout_path = []() {
	// https://stackoverflow.com/questions/143174/how-do-i-get-the-directory-that-a-program-is-running-from
	char result[MAX_PATH];
	cout << std::string(result, GetModuleFileName(NULL, result, MAX_PATH)) << endl;
}; 

////////////////////////////////////////////////////////////////////////////////
// internal helpers_error handling /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
const auto helpers_error = [](const string &msg) {
	cout << "[HELPERS ERROR] " << msg << endl;
	exit(EXIT_FAILURE); 
};

////////////////////////////////////////////////////////////////////////////////
// math ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

 const auto dfrac = [](int num, int den) {
	return double(num) / double(den);
};

// auto e_theta = [](double theta) {
// 	return V3D(1., 0.).rotate(theta, V3D(0., 0., 1.));
// };

const auto e_theta_2D = [](const double &theta) {
	return V3D(cos(theta), sin(theta));
};

const auto ncos01 = [](const double &theta) {
	return .5 - .5*cos(theta);
}; 

const auto angleWith_negPi2posPi = [](const V3D &u, const V3D &v) {
	return u.angleWith(v)*SGN((u.cross(v)).z());
};
 
const auto clamp = [](const double a, const double left, const double right) {
	if (a < left) {
		return left;
	} else if (a > right) {
		return right;
	} else {
		return a;
	}
};

const auto clamp01 = [](const double a) {
	return clamp(a, 0., 1.);
};

const auto mod01 = [](const double a_) {
	bool SUB_FROM_ONE = (a_ < 0);
	double a = abs(a_);
	double b = a - long(a);
	return (SUB_FROM_ONE) ? 1. - b : b;
};

const auto deg2rad = [](const double theta) {
	return theta * PI / 180.;
};

const auto rad2deg = [](const double theta) {
	return theta * 180. / PI; 
};

const auto interval_fraction = [](const double x, const double left, const double right) {
	return (x - left) / (right - left);
};

const auto interp1d = [](const double f, const double left, const double right) {
	return left + (right - left)*f;
};

const auto linspace = [](const int N, const double left, const double right) {
	// N samples total, including both left and right.
	vector<double> X;
	for (int i = 0; i < N; ++i) {
		double f = double(i) / double(N - 1);
		double x = interp1d(f, left, right);
		X.push_back(x); 
	}

	return X;
};

template<typename T>
inline std::vector<T> flatten(const std::vector<std::vector<T>> &orig) {   
	// https://stackoverflow.com/questions/38874605/generic-method-for-flattening-2d-vectors
    std::vector<T> ret;
    for(const auto &v: orig)
        ret.insert(ret.end(), v.begin(), v.end());                                                                                         
    return ret;
}   
 
const auto quadratic1d = [](const double x, const double y0, const double y1) {
	// y(x) = ax^2 + c
	// y0 = y(0); y1 = y(1)
	double a = y1 - y0;
	double c = y0;
	return a*x*x + c;
};

const auto root1d = [](const double x, const double y0, const double y1) {
	// y(x) = sqrt(x) + c
	// y0 = y(0); y1 = y(1)
	double a = y1 - y0;
	double c = y0;
	return a*sqrt(x) + c;
};

const auto min_element = [](const auto &v) {
	return *std::min_element(v.begin(), v.end());
};

const auto max_element = [](const auto &v) {
	return *std::max_element(v.begin(), v.end());
};

// FORNOW
const auto minmax = [](const auto &v, double &v_min, double &v_max) {
	v_min = *std::min_element(v.begin(), v.end());
	v_max = *std::max_element(v.begin(), v.end());
}; 

const auto find_max_i = [](const vector<double> &v) {
	double max_el = -INFINITY;
	int max_i = -1;
	for (size_t i = 0; i < v.size(); ++i) {
		double el = v[i];
		if (el > max_el) {
			max_el = el;
			max_i = i;
		}
	}
	return max_i;
};

const auto find_min_i = [](const vector<double> &v) {
	double min_el = INFINITY;
	int min_i = -1;
	for (size_t i = 0; i < v.size(); ++i) {
		double el = v[i];
		if (el < min_el) {
			min_el = el;
			min_i = i;
		}
	}
	return min_i;
};

////////////////////////////////////////////////////////////////////////////////
// eigen wrappers //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// construct w st v x w is 1.
const auto construct_perp2d = [](V3D v) -> V3D {
	if (!IS_ZERO(v[2])) { helpers_error("Calling 2d function on 3D vector."); }
	if (!IS_ZERO(v.norm() - 1.)) { helpers_error("Calling on unnormalized vector. TODO: Version that preserves length."); }
	// --
	return V3D(-v[1], v[0]);
};

const auto outer2x2 = [](const V3D &u, const V3D &v) {
	double u0v0 = u[0]*v[0];
	double u0v1 = u[0]*v[1];
	double u1v0 = u[1]*v[0];
	double u1v1 = u[1]*v[1];
	Matrix2x2 res;
	res << u0v0, u0v1, u1v0, u1v1;
	return res;
};

const auto xxT2x2 = [](const V3D &u) {
	return outer2x2(u, u);
};

const auto outer3x3 = [](const V3D &u, const V3D &v) {
	// -- // m0*
	double m00 = u[0]*v[0];
	double m01 = u[0]*v[1];
	double m02 = u[0]*v[2];
	// -- // m1*
	double m10 = u[1]*v[0];
	double m11 = u[1]*v[1];
	double m12 = u[1]*v[2];
	// -- // m2*
	double m20 = u[2]*v[0];
	double m21 = u[2]*v[1];
	double m22 = u[2]*v[2];
	// --
	Matrix3x3 M;
	M << m00, m01, m02, m10, m11, m12, m20, m21, m22;
	return M;
};

const auto xxT3x3 = [](const V3D &u) {
	return outer3x3(u, u);
};

const auto resize_zero = [](dVector &v, int n) {
	v.resize(n);
	v.setZero();
};

const auto resize_fill = [](dVector &v, int n, double val) {
	v.resize(n);
	v.setZero();
	v.fill(val);
};

const auto mat_resize_zero = [](MatrixNxM &M, int R, int C) {
	M.resize(R, C);
	M.setZero();
};

const auto mat_resize_fill = [](MatrixNxM &M, int R, int C, double val) {
	M.resize(R, C);
	M.setZero();
	M.fill(val);
};

const auto assert_unit = [](const V3D &e) {
	if (!IS_ZERO(e.norm() - 1.)) { error("AssertUnitError"); cout << endl << "--> " << e.transpose() << endl; } 
	// __debugbreak();
};

const auto pack_into_basis_matrix = [](const V3D &x, const V3D &y, const V3D &z) {
	Matrix3x3 B;
	B(0, 0) = x[0]; B(0, 1) = y[0]; B(0, 2) = z[0];
	B(1, 0) = x[1]; B(1, 1) = y[1]; B(1, 2) = z[1];
	B(2, 0) = x[2]; B(2, 1) = y[2]; B(2, 2) = z[2];
	return B;
};

const auto e3from12 = [](const V3D &e1, const V3D &e2) {
	assert_unit(e1);
	assert_unit(e2);
	// --
	return e1.cross(e2).normalized();
};

const auto e2from13 = [](const V3D &e1, const V3D &e3) {
	assert_unit(e1);
	assert_unit(e3);
	// --
	return e3.cross(e1).normalized();
};

const auto e1from23 = [](const V3D &e2, const V3D &e3) {
	assert_unit(e2);
	assert_unit(e3);
	// --
	return e2.cross(e3).normalized();
};

const auto Bfrom12 = [](const V3D &e1, const V3D &e2) {
	assert_unit(e1);
	assert_unit(e2);
	// -- 
	return pack_into_basis_matrix(e1, e2, e3from12(e1, e2));
};

const auto Bfrom13 = [](const V3D &e1, const V3D &e3) {
	assert_unit(e1);
	assert_unit(e3);
	// --
	return pack_into_basis_matrix(e1, e2from13(e1, e3), e3); 
};

const auto Bfrom23 = [](const V3D &e2, const V3D &e3) {
	assert_unit(e2);
	assert_unit(e3);
	// --
	return pack_into_basis_matrix(e1from23(e2, e3), e2, e3); 

};

const auto is_nan = [](const dVector& x) {
	for (int i = 0; i < x.size(); ++i) {
		if (isnan(x[i]) || isinf(x[i])) {
			return true;
		}
	}
	return false;
};

const auto mat_spy = [](const MatrixNxM &M) {
	MatrixNxM spy;
	mat_resize_zero(spy, M.rows(), M.cols());
	for (int i = 0; i < M.rows(); ++i) {
		for (int j = 0; j < M.cols(); ++j) {
			if (abs(M(i, j)) > 1e-20) {
				spy(i, j) = 1;
			}
		}
	}
	cout << spy << endl;
};

const auto vec_FD = [] (dVector s0, auto O_of_s) {
	// ~ dOds|s0
	int n = s0.size();
	dVector vec;
	resize_zero(vec, n);

	double d = 10e-4;
	for (int i = 0; i < n; ++i) {
		double s0i = s0[i];

		s0[i] -= d;
		double left = O_of_s(s0);
		s0[i] = s0i;

		s0[i] += d;
		double right = O_of_s(s0);
		s0[i] = s0i;

		vec[i] = (right - left) / (2. * d);
	}
	return vec;
};

const auto mat_FD = [] (dVector s0, auto f_of_s) {
	// ~ dfds|s0

	dVector f0 = f_of_s(s0);

	int r = s0.size();
	int c = f0.size();

	MatrixNxM mat;
	mat_resize_zero(mat, r, c);

	double d = 10e-4;
	for (int i = 0; i < r; ++i) {
		double s0i = s0[i];

		s0[i] -= d;
		dVector left = f_of_s(s0);
		s0[i] = s0i;

		s0[i] += d;
		dVector right = f_of_s(s0);
		s0[i] = s0i;

		dVector dfdsi = (right - left) / (2. * d);
		for (int j = 0; j < c; ++j) {
			mat(i, j) = dfdsi[j];
		}
	}
	
	return mat;
};

const auto vec_FD_SPEC = [] (dVector s0, auto O_of_s, vector<int> i_vec) {
	// ~ dOds|s0
	int n = s0.size();
	dVector vec;
	resize_zero(vec, n);

	double d = 10e-6;
	for (int &i : i_vec) {
		double s0i = s0[i];

		s0[i] -= d;
		double left = O_of_s(s0);
		s0[i] = s0i;

		s0[i] += d;
		double right = O_of_s(s0);
		s0[i] = s0i;

		vec[i] = (right - left) / (2. * d);
	}
	return vec;
};

const auto mat_FD_SPEC = [] (dVector s0, auto f_of_s, vector<int> i_vec) {
	// only looks at entries in i_vec x i_vec

	dVector f0 = f_of_s(s0);

	int r = s0.size();
	int c = f0.size();

	MatrixNxM mat;
	mat_resize_zero(mat, r, c);

	double d = 10e-6;
	for (int &i : i_vec) {
		double s0i = s0[i];

		s0[i] -= d;
		dVector left = f_of_s(s0);
		s0[i] = s0i;

		s0[i] += d;
		dVector right = f_of_s(s0);
		s0[i] = s0i;

		dVector dfdsi = (right - left) / (2. * d);
		for (int &j : i_vec) {
			if (j >= c) {
				break;
			}
			mat(i, j) = dfdsi[j];
		}
	}
	
	return mat;
};

const auto mat_FD_SPEC_sparse = [] (dVector s0, auto f_of_s, vector<int> i_vec) -> vector<MTriplet> {
	// returns vector<MTriplet>
	// NOTE: f_of_s is expected dense, i.e. to return a dVector of length 2*NUM_NODES

	dVector f0 = f_of_s(s0);

	int r = s0.size();
	int c = f0.size();

	vector<MTriplet> ret;

	double d = 10e-6;
	for (int &i : i_vec) {
		double s0i = s0[i];

		s0[i] -= d;
		dVector left = f_of_s(s0);
		s0[i] = s0i;

		s0[i] += d;
		dVector right = f_of_s(s0);
		s0[i] = s0i;

		dVector dfdsi = (right - left) / (2. * d);
		for (int &j : i_vec) {
			if (j >= c) {
				break;
			}
			double v = dfdsi[j];
			if (!IS_ZERO(v)) {
				ret.push_back(MTriplet(i, j, v));
			}
		}
	}
	
	return ret;
};

const auto dVectorEquality = [](const dVector &FDGradient, const dVector &analyticGradient) -> bool {
	if (FDGradient.size() != analyticGradient.size()) {
		helpers_error("dVector size mismatch.");
	}
	// --
	bool ret = true;
	for (int i = 0; i < FDGradient.size(); i++) {
		double err = FDGradient[i] - analyticGradient[i];
		if (fabs(err) > 0.0001 && 2 * fabs(err) / (fabs(FDGradient[i]) + fabs(analyticGradient[i])) > 0.001) {
			ret = false;
			Logger::print("--> Mismatch g_%d: Anal val: %lf, FD val: %lf. Error: %lf\n", i, analyticGradient[i], FDGradient[i], err);
		}
	}
	return ret;
};

const auto MatrixLowerTriangleEquality = [](const MatrixNxM &FDHessian, const auto &analyticHessian) -> bool {
	if ( (FDHessian.rows() != analyticHessian.rows()) || (FDHessian.cols() != analyticHessian.cols())) {
		helpers_error("Matrix size mismatch.");
	}
	// --
	bool ret = true;
	for (int i = 0; i < FDHessian.rows(); i++) {
		for (int j = 0; j <= i; j++) {
			double err = FDHessian(i, j) - analyticHessian(i, j);
			if (fabs(err) > 0.001) {
				ret = false;
				Logger::print("--> Mismatch H_{%d,%d}: Anal val: %lf, FD val: %lf. Error: %lf\n", i, j, analyticHessian(i, j), FDHessian(i, j), err);
			}
		}
	}
	return ret;
};

 const auto stack_vec_dVector = [](const vector<dVector> &in) -> dVector {

	int N = 0;
	for (auto &dvec : in) {
		N += dvec.size();
	}
	dVector out; resize_zero(out, N);

	int o = 0;
	for (auto &dvec : in) {
		out.segment(o, dvec.size()) = dvec;
		o += dvec.size();
	}

	return out;

 };



////////////////////////////////////////////////////////////////////////////////
// gl Wrappers /////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const auto set_color = [](const P3D &p) {
	glColor3d(p[0], p[1], p[2]);
};

const auto set_color_alpha = [](const P3D &p, const double &a) {
	glColor4d(p[0], p[1], p[2], a);
};

const auto glP3D = [](const P3D &p) {
	glVertex3d(p[0], p[1], p[2]);
};

const auto glP3D_ = [](const Vector3d &p) {
	glP3D((P3D) p);
};

const auto glP3Dz = [](const P3D &p_, const int &i) {
	// Spec a point on z layer i, where the 0-th layer is farthest from eye.
	V3D offset = V3D(0., 0., .001*double(i));
	P3D q = p_ + offset;
	glP3D(q);
};

const auto glvecP3D = [](const auto &p_vec) {
	for (auto &p : p_vec) {
		glP3D(p);
	}
};

const auto glMasterPush = []() {
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPushMatrix();
};

const auto glMasterPop = []() {
	glPopMatrix();
	glPopAttrib();
};

const auto glvecP3Dz = [](const auto &p_vec, const int &i) {
	for (auto &p : p_vec) {
		glP3Dz(p, i);
	}
};

const auto glTranslateP3D = [](const P3D &s) {
	glTranslated(s[0], s[1], s[2]);
};

const auto glTranslateV3D = [](const V3D &s) {
	glTranslated(s[0], s[1], s[2]);
};

const auto glRotateQuat = [](Quaternion q) {
	V3D axis; double angle;
	q.getAxisAngle(axis, angle);
	angle = rad2deg(angle);
	glRotated(angle, axis[0], axis[1], axis[2]); 
};

const auto glRotateV3D2V3D = [](V3D v1, V3D v2) {
	Quaternion q0 = Quaternion(); 
	if (v1.cross(v2).norm() < .1) {
		V3D a_perp, _;
		v1.getOrthogonalVectors(a_perp, _);
		q0 = Quaternion(.5*PI, a_perp);
		q0.toUnit();
	}
	v1 = q0.rotate(v1);

	V3D a = v1.cross(v2);
	double w = sqrt(v1.length2() * v2.length2()) + v1.dot(v2);
	Quaternion q12 = Quaternion(w, a);
	q12.toUnit();

	Quaternion q02 = q0 * q12;
	q02.toUnit(); 
	glRotateQuat(q02);
};

const auto glBeginBernCircles = [](P3D color) {
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPointSize(7);
	set_color(color);
	glBegin(GL_POINTS);
};

const auto glEndBernCircles = []() {
	glEnd();
	glPopAttrib();
};


////////////////////////////////////////////////////////////////////////////////
// colors //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const auto color_swirl = [](const double f_, const P3D colA, const P3D colB) -> P3D {
	double f = clamp01(f_);
	return colA + (colB - colA)*f;
};

const auto color_tint = [](const P3D col) -> P3D {
	return color_swirl(.5, WHITE, col);
};

const auto color_shade = [](const P3D col) -> P3D {
	return color_swirl(.5, BLACK, col);
};

const auto color_inverse = [](const P3D col) -> P3D {
	return (P3D) (P3D(1., 1., 1.) - col);
};

const auto map_swirl = [](const double f_, const double map[][3]) -> P3D {
	double f = clamp01(f_);

	int N = 20;

	double i_plus_frac = f * (N - 1);
	int i = (int) floor(i_plus_frac);
	double g = i_plus_frac - i;
 
	P3D left = P3D(map[i][0], map[i][1], map[i][2]);
	P3D right = P3D(map[i+1][0], map[i+1][1], map[i+1][2]);
	return color_swirl(g, left, right);
};

////////////////////////////////////////////////////////////////////////////////
// misc ////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


const auto tic = [](clock_t &t, auto string) {
	double dt = (clock() - t) / (double)CLOCKS_PER_SEC;
	cout << string << ": " << dt << endl;
	t = clock(); 
};

const auto contains = [](const auto &v, const auto &e) {
	return (std::find(v.begin(), v.end(), e) != v.end());
};

const auto find_index = [](const auto &v, const auto &e) { 
	if (!contains(v, e)) { helpers_error("v does not contain e"); }
	// --
	auto i_as_iterator = std::find(v.begin(), v.end(), e);
	return std::distance(v.begin(), i_as_iterator);
};

const auto all_true = [](const vector<bool> &v) {
	return (std::find(v.begin(), v.end(), false) == v.end());
};

const auto all_false = [](const vector<bool> &v) {
	return (std::find(v.begin(), v.end(), true) == v.end());
};
 
// TODO: pack_vecX3D_into_dVector2

const auto unique_push_back = [](auto &vec, auto el) {
	if (std::find(vec.begin(), vec.end(), el) == vec.end()) {
		vec.push_back(el);
	}
};
const auto push_back_if_exists_else_delete = [](auto &vec, auto el) {
	auto it = std::find(vec.begin(), vec.end(), el);
	if (it == vec.end()) {
		vec.push_back(el);
	} else {
		vec.erase(it);
	}
};

// TODO: dVec packer

const auto pack_dVector2_into_vecP3D = [](const dVector &x) {
	vector<P3D> packed;
	int N = x.size() / 2;
	for (int i = 0; i < N; ++i) {
		packed.push_back(P3D(x[2*i], x[2*i+1]));
	}
	return packed;
};
const auto pack_dVector2_into_vecV3D = [](const dVector &x) {
	vector<V3D> packed;
	int N = x.size() / 2;
	for (int i = 0; i < N; ++i) {
		packed.push_back(V3D(x[2*i], x[2*i+1]));
	}
	return packed;
};

const auto pack_dVector3_into_vecP3D = [](const dVector &x) {
	vector<P3D> packed;
	int N = x.size() / 3;
	for (int i = 0; i < N; ++i) {
		packed.push_back(P3D(x[3*i], x[3*i+1], x[3*i+2]));
	}
	return packed;
};
const auto pack_dVector3_into_vecV3D = [](const dVector &x) {
	vector<V3D> packed;
	int N = x.size() / 3;
	for (int i = 0; i < N; ++i) {
		packed.push_back(V3D(x[3*i], x[3*i+1], x[3*i+2]));
	}
	return packed;
};

const auto apply_suffix = [](const char *prefix, const char *suffix, char *dest) {
	strcpy(dest, prefix);
	strcat(dest, ".");
	strcat(dest, suffix);
};

const auto random_string = [](char *s, const int len) {
	// http://stackoverflow.com/questions/440133/how-do-i-create-a-random-alpha-numeric-string-in-c
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    s[len] = 0;
};

const auto xy0_from_ray = [] (const Ray &ray) {
	P3D o = ray.origin;
	V3D d = ray.direction;
	// (o + d*f).z = 0
	double f = -o[2] / d[2];
	return o + d*f;
};

const auto min_i_min_d = [] (auto points, auto distance, int &min_i, double &min_d) {
	min_d = INFINITY;
	min_i = -1;
	for (size_t i = 0; i < points.size(); ++i) {
		P3D p = points[i];
		double d = distance(p);
		bool better_candidate = (d < min_d);
		if (d < min_d) {
			min_d = d;
			min_i = i;
		}
	}
};

////////////////////////////////////////////////////////////////////////////////
// drawing /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

const auto draw_arrow2d = [](const P3D &p, const double w, const double h, const V3D F_hat) {


	//  --> v
	// |
	// |
	// V
	// u

	double theta = atan2(F_hat[1], F_hat[0]);
	V3D u = V3D(cos(theta-PI/2), sin(theta-PI/2))*w;
	V3D v = V3D(-sin(theta-PI/2), cos(theta-PI/2))*h;

	// (4wx3h)
	//
	//  wwww-
	//  -----
    //
	//    6   ||
	//  74 25 |h
	//        |h
	//   3p1  |h

	P3D p1, p2, p3, p4, p5, p6, p7;
	p1 = p + u;
	p2 = p + u   + v*2;
	p3 = p - u;
	p4 = p - u   + v*2;
	p5 = p + u*2 + v*2;
	p6 = p       + v*3;
	p7 = p - u*2 + v*2;

	glBegin(GL_TRIANGLES);
	// 1, 2, 3
	glP3Dz(p1, 9);
	glP3Dz(p2, 9);
	glP3Dz(p3, 9);
	// 3, 2, 4
	glP3Dz(p3, 9);
	glP3Dz(p2, 9);
	glP3Dz(p4, 9);
	// 5, 6, 7
	glP3Dz(p5, 9);
	glP3Dz(p6, 9);
	glP3Dz(p7, 9);
	glEnd();
};

const auto draw_arrow3d = [](const P3D &p, const double w, const double h,  const V3D v2) {
	// https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another

	bool PATCH = false; // TODO: case out quat initialization w/ zero qut as default
	V3D v1 = V3D(0., 0., 1.); // from gluCylinder
	if (v1.cross(v2).norm() < .1) { // When v1 exactly antiparallel to v2 the stackoverflow method fails.
		PATCH = true;
		v1 = V3D(0., 1., 0.);
	}
 
	V3D a = v1.cross(v2);
	double w_ = sqrt(v1.length2() * v2.length2()) + v1.dot(v2);
	Quaternion *q = new Quaternion(w_, a[0], a[1], a[2]); 
	q->toUnit();
 
	// --
	V3D axis; double angle;
	q->getAxisAngle(axis, angle);
	angle = rad2deg(angle);
	// --
	glPushAttrib(GL_LIGHTING_BIT); {
		glEnable(GL_LIGHTING);
		glPushMatrix(); {
			GLUquadric *dummy = gluNewQuadric(); {
				glTranslated(p[0], p[1], p[2]);
				glRotated(angle, axis[0], axis[1], axis[2]); if (PATCH) { glRotated(-90., 1., 0., 0.); }
				gluCylinder(dummy, w, w, 2 * h, 20, 5);
				gluDisk(dummy, 0., w, 20, 5);
				glTranslated(0., 0., 2 * h);
				gluCylinder(dummy, 2 * w, 0, h, 20, 5);
				gluDisk(dummy, 0., 2 * w, 20, 5);
			} gluDeleteQuadric(dummy);
		} glPopMatrix();
	} glPopAttrib(); 
};

const auto glBalledCylinder = [](const P3D &a, const P3D &b) {
	V3D ab = V3D(a, b);
	GLdouble gl_line_width; glGetDoublev(GL_LINE_WIDTH, &gl_line_width);
	double w = .003 * gl_line_width;
	double h = ab.norm();
	V3D v2 = ab.normalized();
	// --
	bool PATCH = false; // TODO: case out quat initialization w/ zero qut as default
	V3D v1 = V3D(0., 0., 1.); // from gluCylinder
	if (v1.cross(v2).norm() < .1) { // When v1 exactly antiparallel to v2 the stackoverflow method fails.
		PATCH = true;
		v1 = V3D(0., 1., 0.);
	}
 
	V3D a_ = v1.cross(v2);
	double w_ = sqrt(v1.length2() * v2.length2()) + v1.dot(v2);
	Quaternion *q = new Quaternion(w_, a_[0], a_[1], a_[2]); 
	q->toUnit();
 
	// --
	V3D axis; double angle;
	q->getAxisAngle(axis, angle);
	angle = rad2deg(angle);
	// --
	glPushAttrib(GL_LIGHTING_BIT); {
		glEnable(GL_LIGHTING);
		glPushMatrix(); {
			GLUquadric *dummy = gluNewQuadric(); {
				glTranslated(a[0], a[1], a[2]);
				glRotated(angle, axis[0], axis[1], axis[2]); if (PATCH) { glRotated(-90., 1., 0., 0.); }
				gluSphere(dummy, w, 20, 20);
				gluCylinder(dummy, w, w, h, 20, 5);
				glTranslated(0., 0., h);
				gluSphere(dummy, w, 20, 20);
			} gluDeleteQuadric(dummy);
		} glPopMatrix();
	} glPopAttrib(); 
};

const auto glBalledCylinderLoop = [](const vector<P3D> &s_vec) {
	for (size_t i = 0; i < s_vec.size(); ++i) {
		int j = (i + 1) % s_vec.size();
		// --
		glBalledCylinder(s_vec[i], s_vec[j]);
	}
}; 


const auto glSphere = [](P3D &s) {
	GLdouble gl_point_size; glGetDoublev(GL_POINT_SIZE, &gl_point_size);
	double r = .005 * gl_point_size;
	// --
	glPushAttrib(GL_LIGHTING_BIT); {
		glEnable(GL_LIGHTING);
		// --
		glPushMatrix(); {
			glTranslated(s[0], s[1], s[2]);
			GLUquadric *dummy = gluNewQuadric(); {
				gluSphere(dummy, r, 24, 24);
			} gluDeleteQuadric(dummy);
		} glPopMatrix();
	} glPopAttrib();
};

const auto glSphereVec = [](vector<P3D> &s_vec) {
	for (auto &s : s_vec) {
		glSphere(s);
	}
};

 
const auto quiver = [](const vector<P3D> &x_vec, const vector<V3D> &F_vec, int D) {

	double ZERO_W = 0.;
	double UNIT_W = .01; // unit force produces this width

	for (size_t i = 0; i < x_vec.size(); ++i) {
		P3D x = x_vec[i];
		V3D F = F_vec[i];
		if (F.norm() > 1.e-6) {
			double f = F.norm();
			V3D F_hat = F.normalized();
			double w = root1d(f, ZERO_W, UNIT_W);
			double h = 2 * w;
			// set_map_swirl(f, SUMMER);
			if (D == 2) {
				draw_arrow2d(x, w, h, F_hat);
			} else {
				draw_arrow3d(x, w, h, F_hat); 
			}
		}
	}
};

const auto big_bad_panic_rectangle = []() {
	glBegin(GL_QUADS);
	double R = 1000.;
	double Z = -100.;
	glP3D(P3D( R,  R, Z));
	glP3D(P3D(-R,  R, Z));
	glP3D(P3D(-R, -R, Z));
	glP3D(P3D( R, -R, Z));
	glEnd();
};

const auto draw_floor2d = []() {
	glMasterPush(); {
		glLineWidth(4.);
		set_color(ORCHID);
		// --
		glBegin(GL_LINES); {
			glP3D(P3D(-100., 0.));
			glP3D(P3D(100., 0.));
		} glEnd();
	} glMasterPop();
}; 

////////////////////////////////////////////////////////////////////////////////
// vector work /////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// FORNOW: Move to standard math
const auto random_int = [](int N) -> int {
	// [0, N)
	return rand() % N;
};

const auto random_double = []() -> double {
	// https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
	return (double) rand() / RAND_MAX;
};

const auto remove_at = [](auto &v, int i) {
	auto el = v[i];
	v.erase(v.begin() + i);
	return el;
};

const auto remove_first_instance = [](auto &v, auto &el) {
	if (!contains(v, el)) { error("el not in v."); }
	int i = find_index(v, el);
	remove_at(v, i);
};

// TODO: remove all

const auto random_pop = [](auto &v) {
	return remove_at(v, random_int(v.size()));
};

const auto vecDouble_sum = [](const auto &v) {
	double ret = 0.;
	for (auto &e : v) {
		ret += e;
	}
	return ret;
};

const auto vecP3D_sum = [](const auto &v) {
	P3D ret = P3D();
	for (auto &e : v) {
		ret += e;
	}
	return ret;
};

const auto approx_equal = [](const double &a, const double &b) {
	return (abs(a - b) < 1e-6);
};
 

// TODO: bern_assert

/*
	// https://en.wikipedia.org/wiki/Circumscribed_circle
	Matrix3x3 M_Sx;
	Matrix3x3 M_Sy;
	Matrix3x3 M_aa;
	M_Sx << a.squaredNorm(), a.y(), 1.,
			b.squaredNorm(), b.y(), 1.,
			c.squaredNorm(), c.y(), 1.;
	M_Sy << a.x(), a.squaredNorm(), 1.,
			b.x(), b.squaredNorm(), 1.,
			c.x(), c.squaredNorm(), 1.;
	M_aa << a.x(), a.y(), 1.,
			b.x(), b.y(), 1.,
			c.x(), c.y(), 1.;
	double Sx = .5*M_Sx.determinant();
	double Sy = .5*M_Sy.determinant();
	double aa = M_aa.determinant();
	P3D C = P3D(Sx, Sy)*(1. / aa);
	double R = (a - C).norm();
*/
