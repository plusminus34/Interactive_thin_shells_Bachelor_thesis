#include <RobotDesignerLib/LocomotionEngineConstraints.h>

LocomotionEngine_Constraints::LocomotionEngine_Constraints(LocomotionEngineMotionPlan* mp){
	theMotionPlan = mp;
	dVector params;
	mp->writeMPParametersToList(params);
	getEqualityConstraintValues(params);
}

	//and bound constraints for the parameters min <= p <= max
const dVector& LocomotionEngine_Constraints::getBoundConstraintsMinValues() {
	theMotionPlan->getParameterMinValues(l);

	return l;
}

const dVector& LocomotionEngine_Constraints::getBoundConstraintsMaxValues() {
	theMotionPlan->getParameterMaxValues(u);
	return u;
}

LocomotionEngine_Constraints::~LocomotionEngine_Constraints(void){
}

int LocomotionEngine_Constraints::getEqualityConstraintCount() {
	int nEqConstraints = 0;
	weightsUnityConstraintsStartIndex = -1;
	periodicBoundaryConstraintsIndex = -1;
	footSlidingConstraintsStartIndex = -1;

	//for every sample, we want the sum of the weights associated with the stance feet to add up to 1...
	if (theMotionPlan->barycentricWeightsParamsStartIndex >= 0){
		weightsUnityConstraintsStartIndex = nEqConstraints;
		nEqConstraints += theMotionPlan->nSamplePoints;
	}

	//figure out the constraints required to not let the end effectors slide...
	footSlidingConstraintsStartIndex = nEqConstraints;
	for (int j = 0; j<theMotionPlan->nSamplePoints - 1; j++) {
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
			//this constraint is between the end effector positions at time j and j+1
			if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0)
				nEqConstraints += 2;
		}
	}

	//if we want a periodic motion, add appropriate constraints for all the joint angles...
	if (theMotionPlan->wrapAroundBoundaryIndex > -1){
		periodicBoundaryConstraintsIndex = nEqConstraints;
		//we will also constrain the pitch, roll, and body height to be periodic. This means that globally the robot will be able to move and turn only...
		nEqConstraints += theMotionPlan->robotRepresentation->getDimensionCount() - 6 + 3;
	}

	/*if (theMotionPlan->optimizeContactForces)
	{
		forceAccelConstrainsIndex = nEqConstraints;
		nEqConstraints += 3 * (theMotionPlan->nSamplePoints - 1);
	}*/

	return nEqConstraints;
}

int LocomotionEngine_Constraints::getInequalityConstraintCount() {
	//other than bound constraints, no inequality constraints...

	int nIneqConstraints = 0;
	frictionConeConstraintsStartIndex = -1;

	if (theMotionPlan->frictionCoeff >= 0 && theMotionPlan->contactForcesParamsStartIndex >= 0 && theMotionPlan->enforceGRFConstraints) {
		frictionConeConstraintsStartIndex = nIneqConstraints;
		for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
			for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++) 
			{
				if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0)
					nIneqConstraints += 4;
			}
		}
	}
	
	return nIneqConstraints;
}

const dVector& LocomotionEngine_Constraints::getInequalityConstraintValues(const dVector& p) {
	//first decide how many constraints there are

	theMotionPlan->setMPParametersFromList(p);

	int nIneqConstraints = getInequalityConstraintCount();

	resize(ineqConstraintVals, nIneqConstraints);
	resize(d, nIneqConstraints);
	resize(f, nIneqConstraints);

	addFrictionConeConstraints(frictionConeConstraintsStartIndex);

	return ineqConstraintVals;
}

void LocomotionEngine_Constraints::addFrictionConeConstraints(int constraintStartIndex){
	if (frictionConeConstraintsStartIndex < 0) return;

	int cIndex = constraintStartIndex;
	for (int j = 0; j < theMotionPlan->nSamplePoints; j++) {
		for (uint i = 0; i < theMotionPlan->endEffectorTrajectories.size(); i++)
		{
			if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0)
			{
				V3D force = theMotionPlan->endEffectorTrajectories[i].contactForce[j];
				double ub = force[1] * theMotionPlan->frictionCoeff;
				ineqConstraintVals[cIndex] = force[0] - ub; 
				d[cIndex] = -10000; f[cIndex++] = 0;
				ineqConstraintVals[cIndex] = force[0] + ub;
				d[cIndex] = 0; f[cIndex++] = 10000;

				ineqConstraintVals[cIndex] = force[2] - ub;
				d[cIndex] = -10000; f[cIndex++] = 0;
				ineqConstraintVals[cIndex] = force[2] + ub;
				d[cIndex] = 0; f[cIndex++] = 10000;
			}
		}
	}
}

void LocomotionEngine_Constraints::addWeightsAPartitionOfUnityConstraints(int constraintStartIndex){
	if (constraintStartIndex < 0) return;
	//for every time sample that we have, we want the weights of the end effectors that are in contact with the ground to sum up to 1. Ask this as hard constraints
	if (theMotionPlan->barycentricWeightsParamsStartIndex>=0)
		for (int i=0; i<theMotionPlan->nSamplePoints; i++){
			double val = 0;
			for (uint j=0;j<theMotionPlan->endEffectorTrajectories.size();j++){
				val += theMotionPlan->endEffectorTrajectories[j].EEWeights[i] * theMotionPlan->endEffectorTrajectories[j].contactFlag[i];
			}
			eqConstraintVals[i + constraintStartIndex] = val;
			b[i + constraintStartIndex] = 1.0;
		}
}

void LocomotionEngine_Constraints::addFootSlidingConstraints(int constraintStartIndex) {
	if (constraintStartIndex < 0) return;
	int cIndex = 0;
	for (int j = 0; j<theMotionPlan->nSamplePoints - 1; j++) {
		for (uint i = 0; i<theMotionPlan->endEffectorTrajectories.size(); i++) {
			//this constraint is between the end effector positions at time j and j+1
			if (theMotionPlan->endEffectorTrajectories[i].contactFlag[j] > 0){
				V3D eeOffset = V3D(theMotionPlan->endEffectorTrajectories[i].EEPos[j], theMotionPlan->endEffectorTrajectories[i].EEPos[j + 1]);
				eqConstraintVals[cIndex + constraintStartIndex] = eeOffset[0];
				b[cIndex + constraintStartIndex] = 0.0;
				cIndex++;

				eqConstraintVals[cIndex + constraintStartIndex] = eeOffset[2];
				b[cIndex + constraintStartIndex] = 0.0;
				cIndex++;
			}
		}
	}
}

void LocomotionEngine_Constraints::addPeriodicMotionConstraints(int constraintStartIndex) {
	if (constraintStartIndex < 0) return;
	//for every time sample that we have, we want the weights of the end effectors that are in contact with the ground to sum up to 1. Ask this as hard constraints
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) {

		//body height
		eqConstraintVals[constraintStartIndex] = theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->nSamplePoints - 1][1] - theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->wrapAroundBoundaryIndex][1];
		b[constraintStartIndex] = 0.0;
		constraintStartIndex++;
		//roll
		eqConstraintVals[constraintStartIndex] = theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->nSamplePoints - 1][4] - theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->wrapAroundBoundaryIndex][4];
		b[constraintStartIndex] = 0.0;
		constraintStartIndex++;
		//pitch
		eqConstraintVals[constraintStartIndex] = theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->nSamplePoints - 1][5] - theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->wrapAroundBoundaryIndex][5];
		b[constraintStartIndex] = 0.0;
		constraintStartIndex++;

		//and now all the joints...
		for (int i = 0; i < theMotionPlan->robotRepresentation->getDimensionCount() - 6; i++) {
			eqConstraintVals[i + constraintStartIndex] = theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->nSamplePoints - 1][i + 6] - theMotionPlan->robotStateTrajectory.qArray[theMotionPlan->wrapAroundBoundaryIndex][i + 6];
			b[i + constraintStartIndex] = 0.0;
		}
	}
}

//get the actual value of the equality constraints...
const dVector& LocomotionEngine_Constraints::getEqualityConstraintValues(const dVector& p){
	//first decide how many constraints there are

	theMotionPlan->setMPParametersFromList(p);

	int nEqConstraints = getEqualityConstraintCount();

	resize(eqConstraintVals, nEqConstraints);
	resize(b, nEqConstraints);
	
	addWeightsAPartitionOfUnityConstraints(weightsUnityConstraintsStartIndex);
	addFootSlidingConstraints(footSlidingConstraintsStartIndex);
	addPeriodicMotionConstraints(periodicBoundaryConstraintsIndex);

	return eqConstraintVals;
}

/*!
*  evaluates the constraint jacobian
*/
/*
void LocomotionEngine_Constraints::addEqualityConstraintsJacobianEntriesTo(DynamicArray<MTriplet>& jacobianEntries, const dVector& p) {
	//first decide how many constraints there are
	int nEqConstraints = getEqualityConstraintCount();

}
*/