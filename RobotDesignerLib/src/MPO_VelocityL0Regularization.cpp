#include <RobotDesignerLib/MPO_VelocityL0Regularization.h>

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

	double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

	double delta = theMotionPlan->jointL0Delta;

	auto f = [delta](double t) {return (t*t) / (t*t + delta); };

	double retVal = 0;

	int nSamplePoints = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	Eigen::Map<const Eigen::MatrixXd> qMap(s.data()+theMotionPlan->robotStatesParamsStartIndex, theMotionPlan->robotStateTrajectory.qArray[0].size(), nSamplePoints);
	for (int i = startQIndex; i <= endQIndex; i++) {
		for (int j = 0; j < nSamplePoints; j++) {
			int jm, jp;

			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			double velocity = (qMap(i,jp) - qMap(i,jm)) / dt;
			retVal += f(velocity);
		}
	}

	return retVal * weight;
}

void MPO_VelocityL0Regularization::addGradientTo(dVector& grad, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex < 0)
		return;
	

	double delta = theMotionPlan->jointL0Delta;

	auto g = [delta](double t) {return (2*t*delta) / ((t*t + delta)*(t*t + delta)); };


	int nSamplePoints = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	for (int j=0; j<nSamplePoints; j++){

		int jm, jp;

		theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;
			double dC = g(velocity);
				

			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jm + i] -= weight * dC / dt;
			grad[theMotionPlan->robotStatesParamsStartIndex + theMotionPlan->robotStateTrajectory.nStateDim * jp + i] += weight * dC / dt;
		}
	}
}

void MPO_VelocityL0Regularization::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {

	if (theMotionPlan->robotStatesParamsStartIndex >= 0){

		double delta = theMotionPlan->jointL0Delta;

		auto h = [delta](double t) {double dt2 = delta + t*t; return (2 * delta)*(delta - 3 * t*t) / (dt2*dt2*dt2); };

		int nSamplePoints = theMotionPlan->nSamplePoints;
		if (theMotionPlan->wrapAroundBoundaryIndex >= 0) nSamplePoints -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

		for (int j=0; j<nSamplePoints; j++){

			int jm, jp;

			theMotionPlan->getVelocityTimeIndicesFor(j, jm, jp);
			if (jm == -1 || jp == -1) continue;

			double dt = theMotionPlan->motionPlanDuration / theMotionPlan->nSamplePoints;

			for (int i=startQIndex; i<=endQIndex; i++){
				double velocity = (theMotionPlan->robotStateTrajectory.qArray[jp][i] - theMotionPlan->robotStateTrajectory.qArray[jm][i]) / dt;

				if (theMotionPlan->robotStatesParamsStartIndex >= 0){

					// lower bound hessian
					// d_jm_jm
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								weight * h(velocity) / (dt*dt));
					// d_jp_jp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								weight * h(velocity) / (dt*dt));
					// d_jm_jp
					addMTripletToList_reflectUpperElements(
								hessianEntries,
								theMotionPlan->robotStatesParamsStartIndex + jm * theMotionPlan->robotStateTrajectory.nStateDim + i,
								theMotionPlan->robotStatesParamsStartIndex + jp * theMotionPlan->robotStateTrajectory.nStateDim + i,
								- weight * h(velocity) / (dt*dt));
				}
			}
		}
	}
}


