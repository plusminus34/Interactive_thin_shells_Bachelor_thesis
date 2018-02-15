#include <OptimizationLib/LazyNewtonFunctionMinimizer.h>

//TODO: try out different linear solvers, with prefactorization and without. Have a way of specifying which one this newton solver should be using...
//TODO: get a version of the objective functions (and everything else) that works with dense matrices as well...

#include <iostream>
#define USE_PARDISO
// The search direction is given by -Hinv * g
void LazyNewtonFunctionMinimizer::computeSearchDirection(ObjectiveFunction *function, const dVector &p, dVector& dp) {


	// get the hessian entries (as vector of triplets)
	hessianEntries.resize(0);
	function->addHessianEntriesTo(hessianEntries, pi);

	// plug hessian entries into the eigen sparse matrix
	if (newHessianStructure) {
#ifdef USE_PARDISO
		std::vector<int> II, JJ;
		std::vector<double> SS;
		for (int i = 0; i < hessianEntries.size(); i++)
		{
			JJ.push_back(hessianEntries[i].row());
			II.push_back(hessianEntries[i].col());
			SS.push_back(hessianEntries[i].value());
		}
		pardiso->set_pattern(II, JJ, SS);
#else
		resize(H, p.size(), p.size());
		H.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
		// store location of each entry in the hessian
		hessianEntries_Hptr.resize(hessianEntries.size());
		for (size_t i = 0; i < hessianEntries.size(); ++i) {
			hessianEntries_Hptr[i] = &(H.coeffRef(hessianEntries[i].row(), hessianEntries[i].col()));
		}
#endif // USE_PARDISO
	}
	else {
#ifdef USE_PARDISO
		std::vector<double> SS;
		for (int i = 0; i < hessianEntries.size(); i++)
		{
			if (!isfinite(hessianEntries[i].value()))
				return;
			SS.push_back(hessianEntries[i].value());
		}
		pardiso->update_a(SS);
		try
		{
			
		}
		catch (std::runtime_error& err)
		{
			
		}
#else
		double * data = H.valuePtr();
		std::memset(static_cast<void*>(data), 0, H.nonZeros() * sizeof(*(H.valuePtr())));
		for(size_t i = 0; i < hessianEntries.size(); ++i) {
			*(hessianEntries_Hptr[i]) += hessianEntries[i].value();
		}
#endif // USE_PARDISO
	}

	// get the gradient
	resize(gradient, p.size());
	function->addGradientTo(gradient, pi);

	//TODO
/*
	if (svdAnalysis) {
		SVDFactorization svd;
		SparseMatrix U, S, V;
		svd.computeSVD(*function->getHessianAt(pi), U, S, V);
		for (int i = 0; i<min(S.getNumCols(), S.getNumRows()); i++)
		Logger::printStatic("%d th singluer value of hessian : %e \n", i + 1, S.getElementAt(i, i));
	}
*/

	timerN.restart();

	//dp = Hes^-1 * grad
	if(newHessianStructure) {
#ifdef USE_PARDISO
		pardiso->analyze_pattern();
#else
		solver.analyzePattern(H);
#endif
		newHessianStructure = false;
	}
#ifdef USE_PARDISO
	pardiso->factorize();
	for (int i = 0; i < dp.size(); i++)
		if (!isfinite(dp(i)))
			return;
	pardiso->solve(gradient, dp);
#else
	solver.factorize(H);
	dp = solver.solve(gradient);
#endif
	//dp = H.triangularView<Eigen::Lower>().solve(gradient);

	double dotProduct = dp.dot(gradient);
	if (dotProduct < 0 && hessCorrectionMethod != None) {
		// 		if (printOutput)
		Logger::logPrint("Search direction is not a descent direction (g.dp = %lf). Patching it up...\n", dotProduct);
		if (hessCorrectionMethod == DynamicRegularization) {
			double currStabValue = stabValue;
			int i = 0;
			for (; i < nMaxStabSteps; ++i) {
				// stabilize hessian
				for (int j = 0; j < p.size(); ++j) {
					H.coeffRef(j, j) += currStabValue;
				}
				currStabValue *= 10;
				solver.compute(H);
				dp = solver.solve(gradient);

				// check if stabilization worked
				dotProduct = dp.dot(gradient);
				if (dotProduct > 0)
					break;
			}
			if (dotProduct > 0)
				Logger::logPrint("Search direction fixed after %d stabilization steps (g.dp = %lf)\n", i + 1, dotProduct);
			else
				Logger::logPrint("Search direction NOT fixed after %d stabilization steps (g.dp = %lf)\n", i + 1, dotProduct);
		}
		else if (hessCorrectionMethod == Projection) {
			Eigen::SimplicialLDLT<MatrixNxM, Eigen::Lower> denseSolver;
			MatrixNxM Hd(H);
			Eigen::SelfAdjointEigenSolver<MatrixNxM> es(Hd);
			Eigen::VectorXd D = es.eigenvalues();
			Eigen::MatrixXd U = es.eigenvectors();
			D = D.unaryExpr([](double x) {return (x < 1e-4) ? 1e-4 : x; });
			dp = U * D.cwiseInverse().asDiagonal()*U.transpose()*gradient;
// 			dp = denseSolver.solve(gradient);

			// check if stabilization worked
			dotProduct = dp.dot(gradient);
			if (dotProduct < 0)
				Logger::logPrint("Search direction correction failed (Projection method)");
		}
	}

	if (printOutput)
		Logger::logPrint("Time to solve linear system: %lf\n", timerN.timeEllapsed());

	int checkLinearSystemSolve = 0;
	if (checkLinearSystemSolve) {
		print("../out/hes.m", H);
		print("../out/grad.m", gradient);
		print("../out/result.m", dp);

		dVector pn = dp;
		// check how well we solved the linear system
		dVector rhs((int)gradient.size());
//		H.triangularView<Eigen::StrictlyLower>().transpose().addTo(tmpH);
		rhs = H.triangularView<Eigen::Lower>() * dp + H.transpose().triangularView<Eigen::StrictlyUpper>() * dp;
		print("../out/hes_x_dp.m", rhs);
		rhs -= gradient;
		double residual = rhs.norm();
		if (printOutput)
			Logger::logPrint("Checking linear system. Residual: %10.10lf\n", residual);
	}
}
