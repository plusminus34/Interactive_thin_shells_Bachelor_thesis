#include <RobotDesignerLib/MPO_VelocityL0Regularization.h>
#include <iostream>

MPO_VelocityL0Regularization::MPO_VelocityL0Regularization(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight, int startQIndex, int endQIndex, bool mode) {

	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
	this->startQIndex = startQIndex;
	this->endQIndex = endQIndex;
	this->mode = mode;
}

MPO_VelocityL0Regularization::~MPO_VelocityL0Regularization(void) {
}

double MPO_VelocityL0Regularization::computeValue(const dVector& s) {

	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return 0;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	double delta = theMotionPlan->jointL0Delta;

// 	double dt;
// 	if (theMotionPlan->wrapAroundBoundaryIndex >= 0)
// 		dt = theMotionPlan->motionPlanDuration / (nSamplePoints-1);
// 	else
// 		dt = theMotionPlan->motionPlanDuration / nSamplePoints;

	auto f = [delta](double t) {return t / (t + delta); };

	Eigen::Map<const Eigen::MatrixXd> Q(s.data()+theMotionPlan->robotStatesParamsStartIndex, theMotionPlan->robotStateTrajectory.qArray[0].size(), nSamplePoints);
	auto Qt = Q.bottomRows(endQIndex - startQIndex + 1);
	double retVal;
	
	if (mode) // f(v11^2)+f(v12^2)+...+f(v21^2)+f(v22^2)+...
		retVal = (Qt.rightCols(nSamplePoints - 1) - Qt.leftCols(nSamplePoints - 1)).cwiseAbs2().unaryExpr(f).sum();
	else     //  f(v11^2+v12^2+..)+f(v21^2+v22^2+...)+...
		retVal = (Qt.rightCols(nSamplePoints - 1) - Qt.leftCols(nSamplePoints - 1)).cwiseAbs2().rowwise().sum().unaryExpr(f).sum();


// 	for (int i = startQIndex; i <= endQIndex; i++) {
// 		for (int j = 0; j < nSamplePoints; j++) {
// 			int jm, jp;
// 
// 			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
// 			if (jm == -1 || jp == -1) continue;
// 
// 			double velocity = (Q(i,jp) - Q(i,jm)) / dt;
// 			retVal += f(velocity);
// 		}
// 	}

	return retVal * weight;
}

void MPO_VelocityL0Regularization::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	double delta = theMotionPlan->jointL0Delta;

	auto g = [delta](double t) {return delta / ((t + delta)*(t + delta)); };

	Eigen::Map<const Eigen::MatrixXd> Q(p.data() + theMotionPlan->robotStatesParamsStartIndex, theMotionPlan->robotStateTrajectory.qArray[0].size(), nSamplePoints);
	auto Qt = Q.bottomRows(endQIndex - startQIndex + 1);

	Eigen::Map<Eigen::MatrixXd> G(grad.data() + theMotionPlan->robotStatesParamsStartIndex, theMotionPlan->robotStateTrajectory.qArray[0].size(), nSamplePoints);
	auto Gt = G.bottomRows(endQIndex - startQIndex + 1);

	MatrixNxM dQ = (Qt.rightCols(nSamplePoints - 1) - Qt.leftCols(nSamplePoints - 1));
	if(mode)
	{
		MatrixNxM GdQ = dQ.cwiseAbs2().unaryExpr(g);
		for (int i = 0; i < nSamplePoints-1; i++)
		{
			Gt.col(i) -=     weight * 2 * dQ.col(i).cwiseProduct(GdQ.col(i));
			Gt.col(i + 1) += weight * 2 * dQ.col(i).cwiseProduct(GdQ.col(i));
		}
	}
	else
	{
		dVector GdQ = dQ.cwiseAbs2().rowwise().sum().unaryExpr(g);
		for (int i = 0; i < nSamplePoints - 1; i++)
		{
			Gt.col(i) -= weight * 2 * dQ.col(i)*GdQ(i);
			Gt.col(i + 1) += weight * 2 * dQ.col(i)*GdQ(i);
		}
	}
// 	for (int j=0; j<nSamplePoints; j++){
// 
// 		int jm, jp;
// 
// 		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
// 		if (jm == -1 || jp == -1) continue;
// 
// 		double dt = theMotionPlan->motionPlanDuration / nSamplePoints;
// 
// 		for (int i=startQIndex; i<=endQIndex; i++){
// 			double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;
// 			double dC = g(velocity);
// 				
// 
// 			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + i] -= weight * dC / dt;
// 			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i] += weight * dC / dt;
// 		}
// 	}
}

void MPO_VelocityL0Regularization::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	double delta = theMotionPlan->jointL0Delta;

	Eigen::Map<const Eigen::MatrixXd> Q(p.data() + theMotionPlan->robotStatesParamsStartIndex, theMotionPlan->robotStateTrajectory.qArray[0].size(), nSamplePoints);
	auto Qt = Q.bottomRows(endQIndex - startQIndex + 1);

	auto g = [delta](double t) {return delta / ((t + delta)*(t + delta)); };
	auto h = [delta](double t) {return -2*delta / ((t + delta)*(t + delta)*(t + delta)); };


	for (int j=0; j<nSamplePoints-1; j++){

		int jm, jp;
// 		auto ij2ind = [this](int i, int j){}
// 		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
// 		if (jm == -1 || jp == -1) continue;
		jp = j + 1;
		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = Q(i,jp) - Q(i,j);

			if (theMotionPlan->robotStatesParamsStartIndex >= 0){
				// grad = 2 velocity g(velocity.^2)
				// lower bound hessian
				// d_jm_jm
				double val = 4 * velocity * velocity *h(velocity*velocity) + 2 * g(velocity*velocity);
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + j  * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + j  * theMotionPlan->robotStateTrajectory.nStateDim + i,
							weight *val);
				// d_jp_jp
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							weight * val);
				// d_jm_jp
				addMTripletToList_reflectUpperElements(
							hessianEntries,
							theMotionPlan->robotStatesParamsStartIndex + j  * theMotionPlan->robotStateTrajectory.nStateDim + i,
							theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
							- weight * val);
			}
		}
	}
}



