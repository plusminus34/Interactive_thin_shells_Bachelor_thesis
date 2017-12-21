
#include <OptimizationLib/NewtonFunctionMinimizer.h>

//TODO: try out different linear solvers, with prefactorization and without. Have a way of specifying which one this newton solver should be using...
//TODO: get a version of the objective functions (and everything else) that works with dense matrices as well...

#include <iostream>

// The search direction is given by -Hinv * g
void NewtonFunctionMinimizer::computeSearchDirection(ObjectiveFunction *function, const dVector &p, dVector& dp) {
	timerN.restart();
	resize(H, p.size(), p.size());
	hessianEntries.clear();

	if (printOutput)
		Logger::logPrint("Time to zero out hessian: %lf\n", timerN.timeEllapsed());

	timerN.restart();
	function->addHessianEntriesTo(hessianEntries, pi);
	if (printOutput)
		Logger::logPrint("Time to compute hessian entries: %lf\n", timerN.timeEllapsed());

	timerN.restart();
	H.setFromTriplets(hessianEntries.begin(), hessianEntries.end());
	if (printOutput)
		Logger::logPrint("Time to write out hessian entries: %lf\n", timerN.timeEllapsed());

// 	print("..\\out\\hes.m", H);

	timerN.restart();
	resize(gradient, p.size());
	function->addGradientTo(gradient, pi);
	if (printOutput)
		Logger::logPrint("Time to compute gradient: %lf\n", timerN.timeEllapsed());

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
	Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
	//	Eigen::SparseLU<SparseMatrix> solver;
	solver.compute(H);
	dp = solver.solve(gradient);
	//dp = H.triangularView<Eigen::Lower>().solve(gradient);

	double dotProduct = dp.dot(gradient);
	if (dotProduct < 0 && useDynamicRegularization) {
// 		if (printOutput)
			Logger::logPrint("Search direction is not a descent direction (g.dp = %lf). Patching it up...\n", dotProduct);

		double currStabValue = stabValue;
		int i = 0;
		for (; i < nMaxStabSteps; ++i) {
			// stabilize hessian
			for (int j = 0; j < p.size(); ++j) {
				H.coeffRef(j,j) += currStabValue;
			}
			currStabValue *= 10;
			solver.compute(H);
			dp = solver.solve(gradient);

			// check if stabilization worked
			dotProduct = dp.dot(gradient);
			if (dotProduct > 0)
				break;
		}

// 		if(printOutput)
		{
			if(dotProduct > 0)
				Logger::logPrint("Search direction fixed after %d stabilization steps (g.dp = %lf)\n", i+1, dotProduct);
			else
				Logger::logPrint("Search direction NOT fixed after %d stabilization steps (g.dp = %lf)\n", i+1, dotProduct);
		}

		//invert the part that goes against the gradient while keeping everything else the same...
//		dVector dpProj = gradient * dotProduct / gradient.squaredNorm();
//		dp = -dpProj;
//		dp -= 2*dpProj;
//		dp *= -1;
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
