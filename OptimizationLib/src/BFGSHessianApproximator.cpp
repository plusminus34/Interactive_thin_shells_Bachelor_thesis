#include <OptimizationLib/BFGSHessianApproximator.h>
#include <MathLib/MathLib.h>
#include <Utils/Logger.h>

#define TINY_NUMER_BFGS 1e-30

/*
	L-BFGS inverse Hessian multiplication. Given a direction v (typically the gradient), this method
	computes the transformation H^{-1}*v, where H is the approximated Hessian matrix. Note that the multiplication
	happens in place, v will be overwritten.
	Based on description from https://en.wikipedia.org/wiki/Limited-memory_BFGS
*/
void BFGSHessianApproximator::compute_Hinv_v_inplace(dVector& v) {
	int n = xDiffArray.size();
	if (n == 0) return;

	dVector tmpV = v;


	dVector alphas; resize(alphas, n);

//	Logger::consolePrint("v norm 1: %lf\n", v.norm());

	// Compute right product
	for (int i = n - 1; i >= 0; i--) {
		double d = dfdxDiffArray[i].dot(xDiffArray[i]);
		if (d < TINY_NUMER_BFGS) continue;

		double rho_i = 1. / d;

		double alpha_i = rho_i * xDiffArray[i].dot(v);
		alphas[i] = alpha_i;

		for (int j = 0; j < v.size(); j++)
			v[j] -= alpha_i * dfdxDiffArray[i][j];
	}

//	Logger::consolePrint("v norm 2: %lf\n", v.norm());

	// Center product
	double h_0 = 1;
	double denom = dfdxDiffArray[n - 1].dot(dfdxDiffArray[n - 1]);
	if (denom > TINY_NUMER_BFGS) {
		h_0 = dfdxDiffArray[n - 1].dot(xDiffArray[n - 1]) / denom;
	}
	if (h_0 < 1 / 100.0) h_0 = 1 / 100.0;
	v *= h_0;

//	Logger::consolePrint("v norm 3: %lf\n", v.norm());

	// Compute left product
	for (int i = 0; i < n; i++) {
		double d = dfdxDiffArray[i].dot(xDiffArray[i]);
		if (d < TINY_NUMER_BFGS) continue;

		double rho_i = 1. / d;

		double beta_i = rho_i * dfdxDiffArray[i].dot(v);

		for (int j = 0; j < v.size(); j++)
			v[j] += (alphas[i] - beta_i) * xDiffArray[i][j];
	}
//	Logger::consolePrint("v norm 4: %lf\n", v.norm());

//	Logger::consolePrint("dot product between gradient and transformed gradient: %lf\n", v.dot(tmpV));

	// Return v; it's already in the transformed result.
}

dVector BFGSHessianApproximator::compute_Hinv_v(const dVector& v) {
	dVector result = v;
	compute_Hinv_v_inplace(result);
	return result;
}
