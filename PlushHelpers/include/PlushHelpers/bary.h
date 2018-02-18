// James M Bern
// Gruppe CoRoS
// ETH Z\"urich

#pragma once

#include <MathLib/MathLib.h>
#include <MathLib/P3D.h>

const auto get_bary_lambda_Xd = [](const std::vector<dVector> &simplex, const dVector &s) -> dVector {
	MatrixNxM A;
	int N = s.size();
	A.resize(N + 1, N + 1);
	for (int i = 0; i < N + 1; ++i) {
		A.block(N, 1, 0, N) = simplex[0];
	}
	A.block(1, N + 1, N, 0).fill(1.);
	// --
	dVector s_hom = s.homogeneous();
	dVector lambda = A.inverse() * s_hom;
	return lambda; 
};

const auto get_bary_lambda_2d_ = [](const std::vector<Vector2d> &tri, const Vector2d &s) -> P3D {
	// -- // https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Conversion_between_barycentric_and_Cartesian_coordinates
	Matrix3x3 A;
	A.block<2, 1>(0, 0) = tri[0];
	A.block<2, 1>(0, 1) = tri[1];
	A.block<2, 1>(0, 2) = tri[2];
	A.block<1, 3>(2, 0).fill(1.);
	// --
	Vector3d s_hom = s.homogeneous();
	Vector3d lambda = A.inverse() * s_hom;
	return P3D(lambda[0], lambda[1], lambda[2]); 
};

const auto get_bary_lambda_2d = [](const std::vector<P3D> &tri_as_vecP3D, const P3D &s_as_P3D) -> P3D {
	Vector2d t0 = tri_as_vecP3D[0].head<2>();
	Vector2d t1 = tri_as_vecP3D[1].head<2>();
	Vector2d t2 = tri_as_vecP3D[2].head<2>();
	Vector2d s = s_as_P3D.head<2>();
	return get_bary_lambda_2d_({ t0, t1, t2 }, s);
}; 

const auto lambda_inside_triangle = [](const P3D &lambda) -> bool {
	return (lambda[0] > -TINY && lambda[1] > -TINY && lambda[2] > -TINY);
};