#include <RobotDesignerLib/MPO_DynamicStabilityObjective.h>

MPO_DynamicStabilityObjective::MPO_DynamicStabilityObjective(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight){
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

MPO_DynamicStabilityObjective::~MPO_DynamicStabilityObjective(void){
}

//evaluates the error vector at sample j
V3D MPO_DynamicStabilityObjective::getErrorVector(int j){
	P3D targetValue;
	for (uint i=0;i<theMotionPlan->endEffectorTrajectories.size();i++){
		double w = theMotionPlan->endEffectorTrajectories[i].EEWeights[j];
		double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
		targetValue += theMotionPlan->endEffectorTrajectories[i].EEPos[j] * w * c;
	}

	int nIntervalCounts = theMotionPlan->nSamplePoints-1;

	P3D x = theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(j);

	double timeStep = theMotionPlan->motionPlanDuration / (nIntervalCounts);
	int jmm, jm, jp, jpp;
	theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
	if (jmm < 0 || jm < 0 || jp < 0 || jpp < 0)
		return V3D();
	
	V3D vp = (theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jpp) - theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jp))/timeStep;
	V3D vm = (theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jm) - theMotionPlan->bodyTrajectory.getCOMPositionAtTimeIndex(jmm))/timeStep;
	V3D xDotDot = (vp - vm)/timeStep;

//	P3D zmp = x - xDotDot * (x.y / (xDotDot.y + fabs(SimGlobals::g)));
	//assuming negligible accelerations in the vertical direction, and an everage desired height, this formula is approximately correct but much easier to solve...
	P3D zmp = x + xDotDot * (theMotionPlan->desCOMHeight / (fabs(Globals::g))) * -1;

	V3D err = V3D(zmp, targetValue);
//	err[1] = 0;

	return err;
}

double MPO_DynamicStabilityObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
//	theMotionPlan->setMPParametersFromList(p);

	double retVal = 0;
	
	//we want the ZMP to be as close as possible to the weighted position of the grounded feet (sum_i c_i*w_i*p_i=com_p)
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count... the last robot pose is already the same as the first one, which means that COM and feet locations are in correct locations relative to each other, so no need to ask for that again explicitely...

	end = end;
	for (int j=0;j<end;j++)
		retVal += 0.5 * getErrorVector(j).length2();
	
	return retVal * weight;
}

void MPO_DynamicStabilityObjective::addGradientTo(dVector& grad, const dVector& p){
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count...

	end = end;
	for (int j=0;j<end;j++){
		int nIntervalCounts = theMotionPlan->nSamplePoints-1;

		double timeStep = theMotionPlan->motionPlanDuration / (nIntervalCounts);
		int jmm, jm, jp, jpp;
		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm < 0 || jm < 0 || jp < 0 || jpp < 0)
			continue;

		V3D err = getErrorVector(j);
		for (int i=0;i<nLimbs;i++){
			double c = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			double w = theMotionPlan->endEffectorTrajectories[i].EEWeights[j];

			//compute the gradient with respect to the barycentric weights
			if (theMotionPlan->barycentricWeightsParamsStartIndex >= 0){
				V3D tmp = theMotionPlan->endEffectorTrajectories[i].EEPos[j] * c;
				grad[theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i] += tmp.dot(err) * weight;
			}

			if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
				for (int k = 0; k < 3; ++k)
					grad[theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k] += err[k] * w * c * weight;
			}
		}

		if (theMotionPlan->COMPositionsParamsStartIndex >= 0){
			for (int k=0;k<3;k++){
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k] -= err[k] * weight;
				double s = (theMotionPlan->desCOMHeight / (fabs(Globals::g))) / timeStep / timeStep;
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k] += err[k]*s * weight;
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k] -= err[k]*s * weight;
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k] -= err[k]*s * weight;
				grad[theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k] += err[k]*s * weight;
			}
		}
	}
}

void MPO_DynamicStabilityObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p){
	//	assume the parameters of the motion plan have been set already by the collection of objective functions class
	//	theMotionPlan->setMPParametersFromList(p);
	int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	int end = theMotionPlan->nSamplePoints;
	if (theMotionPlan->wrapAroundBoundaryIndex >= 0) end -= 1; //don't double count...

	for (int j=0;j<end;j++){
		int nIntervalCounts = theMotionPlan->nSamplePoints-1;

		double timeStep = theMotionPlan->motionPlanDuration / (nIntervalCounts);
		int jmm, jm, jp, jpp;
		theMotionPlan->getAccelerationTimeIndicesFor(j, jmm, jm, jp, jpp);
		if (jmm < 0 || jm < 0 || jp < 0 || jpp < 0)
			continue;
		double s = (theMotionPlan->desCOMHeight / (fabs(Globals::g))) / timeStep / timeStep;

		V3D err = getErrorVector(j);
		for (int i=0;i<nLimbs;i++){
			double ci = theMotionPlan->endEffectorTrajectories[i].contactFlag[j];
			double wi = theMotionPlan->endEffectorTrajectories[i].EEWeights[j];
			V3D footPosi(theMotionPlan->endEffectorTrajectories[i].EEPos[j]);

			if (theMotionPlan->feetPositionsParamsStartIndex >= 0 && theMotionPlan->COMPositionsParamsStartIndex >= 0){

				for (int k = 0; k < 3; ++k) {
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
									theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k,
									-wi*ci,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
									theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k,
									wi*ci*s,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
									theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k,
									-wi*ci*s,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
									theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k,
									-wi*ci*s,
									weight);
					ADD_HES_ELEMENT(hessianEntries,
									theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + k,
									theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k,
									wi*ci*s,
									weight);
				}
			}

			if (theMotionPlan->barycentricWeightsParamsStartIndex >= 0 && theMotionPlan->COMPositionsParamsStartIndex >= 0){
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 0, -footPosi[0]*ci, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 1, -footPosi[1]*ci, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + 2, -footPosi[2]*ci, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + 0, footPosi[0]*ci*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + 1, footPosi[1]*ci*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + 2, footPosi[2]*ci*s, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + 0, -footPosi[0]*ci*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + 1, -footPosi[1]*ci*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + 2, -footPosi[2]*ci*s, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + 0, -footPosi[0]*ci*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + 1, -footPosi[1]*ci*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + 2, -footPosi[2]*ci*s, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + 0, footPosi[0]*ci*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + 1, footPosi[1]*ci*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + 2, footPosi[2]*ci*s, weight);
			}

			for (int k=0;k<nLimbs;k++){
				double ck = theMotionPlan->endEffectorTrajectories[k].contactFlag[j];
				double wk = theMotionPlan->endEffectorTrajectories[k].EEWeights[j];
				V3D footPosk(theMotionPlan->endEffectorTrajectories[k].EEPos[j]);

				//compute the gradient with respect to the barycentric weights
				if (theMotionPlan->barycentricWeightsParamsStartIndex >= 0){
					if (i>=k)
						ADD_HES_ELEMENT(hessianEntries, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + i, theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + k, footPosi.dot(footPosk)*ci*ck, weight);
				}

				if (theMotionPlan->feetPositionsParamsStartIndex >= 0){
					if (i>=k){
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 0,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + k * 3 + 0,
										wi*wk*ci*ck,
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + k * 3 + 1,
										wi*wk*ci*ck,
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 2,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + k * 3 + 2,
										wi*wk*ci*ck,
										weight);
					}
				}

				if (theMotionPlan->feetPositionsParamsStartIndex >= 0 && theMotionPlan->barycentricWeightsParamsStartIndex >= 0){
					if (i == k){
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 0,
										theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + k,
										err[0]*ci+wi*ci*ck*footPosk[0],
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1,
										theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + k,
										err[1]*ci+wi*ci*ck*footPosk[1], weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 2,
										theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + k,
										err[2]*ci+wi*ci*ck*footPosk[2], weight);
					}
					else{
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 0,
										theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + k,
										wi*ci*ck*footPosk[0],
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 1,
										theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + k,
										wi*ci*ck*footPosk[1],
										weight);
						ADD_HES_ELEMENT(hessianEntries,
										theMotionPlan->feetPositionsParamsStartIndex + j * nLimbs * 3 + i * 3 + 2,
										theMotionPlan->barycentricWeightsParamsStartIndex + j * nLimbs + k,
										wi*ci*ck*footPosk[2],
										weight);
					}
				}
			}
		}

		if (theMotionPlan->COMPositionsParamsStartIndex >= 0){
			for (int k=0;k<3;k++){
				if (k == 1) continue;

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, 1, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, s*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, s*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, s*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, s*s, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, -s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, -s, weight);

				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, -s*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, -s*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jpp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, s*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, s*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, -s*s, weight);
				ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jmm + k, -s*s, weight);
				//since the hessian is symmetric, we don't need to be adding the values both below and above the diagonal. However, when the "off-diagonal" elements are actually on the diagonal, they need to be added twice
				if (jp == jm)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, s*s, weight);
				if (j == jm)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jm + k, s, weight);
				if (j == jp)
					ADD_HES_ELEMENT(hessianEntries, theMotionPlan->COMPositionsParamsStartIndex + 3 * j + k, theMotionPlan->COMPositionsParamsStartIndex + 3 * jp + k, s, weight);
			}
		}
	}
}



