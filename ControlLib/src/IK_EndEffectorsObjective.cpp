#include <ControlLib/IK_EndEffectorsObjective.h>

IK_EndEffectorsObjective::IK_EndEffectorsObjective(IK_Plan* mp, const std::string& objectiveDescription, double weight){
	IKPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

IK_EndEffectorsObjective::~IK_EndEffectorsObjective(void){
}

/*
double IK_EndEffectorsObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
//	IKPlan->setParametersFromList(s);

	double retVal = 0;
	int nEE = IKPlan->endEffectors.size();

	for (int i=0;i<nEE;i++){
		V3D err(IKPlan->endEffectors[i].targetEEPos, IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(IKPlan->endEffectors[i].endEffectorLocalCoords, IKPlan->endEffectors[i].endEffectorRB));

		for (int k = 0; k<3; k++)
			err[k] *= IKPlan->endEffectors[i].mask[k];

		retVal += 0.5 * err.length2();
	}

	return retVal * weight;
}
*/
 
double IK_EndEffectorsObjective::computeValue(const dVector& p){
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
//	IKPlan->setParametersFromList(s);

	auto valuePosError = [&](IK_EndEffector const & endEffector) -> double
	{
		double result = 0.0;
		P3D targetPos = endEffector.targetEEPos;
		P3D currentPos = IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(endEffector.endEffectorLocalCoords, 
																			   endEffector.endEffectorRB);
		V3D errPos = currentPos - targetPos;

		for(int k = 0; k < 3; ++k) {
			result += errPos[k]*errPos[k] * endEffector.positionMask[k];
		}
		result *= 0.5;
		return(result);
	};

	auto valueVec_i_Error = [&](IK_EndEffector const & endEffector, int i_vec) -> double
	{
		double result = 0.0;
		double s2 = std::pow(endEffector.lengthScaleOrientation, 2);
		V3D targetVec = endEffector.targetEEOrientation[i_vec];
		V3D currentVec = IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(endEffector.endEffectorLocalOrientation[i_vec],
																				endEffector.endEffectorRB);
		V3D errVec = targetVec - currentVec;
		result = 0.5 * endEffector.orientationMask[i_vec] * errVec.dot(errVec) * s2;
		return(result);
	};

	auto valueVecError = [&](IK_EndEffector const & endEffector) -> double
	{
		double result = 0.0;
		for(int i = 0; i < 3; ++i) {
			result += valueVec_i_Error(endEffector, i);
		}
		return(result);
	};


	double retVal = 0.0;

	for(IK_EndEffector & endEffector: IKPlan->endEffectors) {
		retVal += valuePosError(endEffector);
		if(endEffector.lengthScaleOrientation > 0.0) {
			retVal += valueVecError(endEffector);
		}
	}

	return retVal * weight;
}
/*
void IK_EndEffectorsObjective::addGradientTo(dVector& grad, const dVector& p) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//IKPlan->setParametersFromList(p);

	MatrixNxM dEndEffectordq;

	int nEE = IKPlan->endEffectors.size();

	for (int i=0;i<nEE;i++){
		V3D err(IKPlan->endEffectors[i].targetEEPos, IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(IKPlan->endEffectors[i].endEffectorLocalCoords, IKPlan->endEffectors[i].endEffectorRB));

		IKPlan->gcRobotRepresentation->compute_dpdq(IKPlan->endEffectors[i].endEffectorLocalCoords, IKPlan->endEffectors[i].endEffectorRB, dEndEffectordq);
	
		//dEdee * deedq = dEdq
		for (int k=0;k<3;k++)
			for (int l=0;l<IKPlan->gcRobotRepresentation->getDimensionCount();l++)
				if (IKPlan->optimizeRootConfiguration)
					grad[l] += dEndEffectordq(k, l) * err[k] * IKPlan->endEffectors[i].mask[k] * weight;
				else
					if (l>=6)
						grad[l-6] += dEndEffectordq(k, l) * err[k] * IKPlan->endEffectors[i].mask[k] * weight;
	}
}
*/
void IK_EndEffectorsObjective::addGradientTo(dVector& grad, const dVector& p) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//IKPlan->setParametersFromList(p);

	auto addGradPosError = [&](IK_EndEffector const & endEffector, int p_active_start_idx, dVector & grad)
	{
		int nP = IKPlan->gcRobotRepresentation->getDimensionCount();
		// position error
		double result = 0.0;
		P3D targetPos = endEffector.targetEEPos;
		P3D currentPos = IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(endEffector.endEffectorLocalCoords, 
																			   endEffector.endEffectorRB);
		V3D errPos = currentPos - targetPos;
		// first derivative
		MatrixNxM dPosEEdq;
		IKPlan->gcRobotRepresentation->compute_dpdq(endEffector.endEffectorLocalCoords, 
											endEffector.endEffectorRB,
											dPosEEdq);

		for(int k = 0; k < 3; ++k) {
			for(int l = p_active_start_idx; l < nP; ++l) {
				grad[l-p_active_start_idx] += endEffector.positionMask[k] * errPos[k] * dPosEEdq(k,l);
			}
		}
	};

	auto addGradVec_i_Error = [&](IK_EndEffector const & endEffector, int p_active_start_idx, dVector & grad, int i_vec)
	{
		int nP = IKPlan->gcRobotRepresentation->getDimensionCount();
		double s2 = std::pow(endEffector.lengthScaleOrientation, 2);
		// error
		V3D targetVec = endEffector.targetEEOrientation[i_vec];
		V3D currentVec = IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(endEffector.endEffectorLocalOrientation[i_vec],
																				endEffector.endEffectorRB);
		V3D errVec = targetVec - currentVec;
		// first derivative
		MatrixNxM dVecEEdq;
		IKPlan->gcRobotRepresentation->compute_dvdq(endEffector.endEffectorLocalOrientation[i_vec], 
											endEffector.endEffectorRB,
											dVecEEdq);

		for(int l = p_active_start_idx; l < nP; ++l) {
			grad[l-p_active_start_idx] += - endEffector.orientationMask[i_vec] * dVecEEdq.col(l).dot(errVec) * s2;
		}
	};
	
	auto addGradVecError = [&](IK_EndEffector const & endEffector, int p_active_start_idx, dVector & grad)
	{
		for(int i = 0; i < 3; ++i) {
			addGradVec_i_Error(endEffector, p_active_start_idx, grad, i);
		}
	};
	
	// if root configuration is not part of the optimization, ignore the first 6 parameters
	int p_active_start_idx = IKPlan->optimizeRootConfiguration ? 0 : 6;

	for(IK_EndEffector & endEffector: IKPlan->endEffectors) {
		addGradPosError(endEffector, p_active_start_idx, grad);
		if(endEffector.lengthScaleOrientation > 0.0) {
			addGradVecError(endEffector, p_active_start_idx, grad);
		}
	}
}

/*
void IK_EndEffectorsObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//IKPlan->setParametersFromList(p);

	MatrixNxM dEndEffectordq, ddEndEffectordq_dqi;
	int nEE = IKPlan->endEffectors.size();

	for (int i=0;i<nEE;i++){
		V3D err(IKPlan->endEffectors[i].targetEEPos, IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(IKPlan->endEffectors[i].endEffectorLocalCoords, IKPlan->endEffectors[i].endEffectorRB));

			//and now compute the gradient with respect to the robot q's
		IKPlan->gcRobotRepresentation->compute_dpdq(IKPlan->endEffectors[i].endEffectorLocalCoords, IKPlan->endEffectors[i].endEffectorRB, dEndEffectordq);

		for (int k=0;k<IKPlan->gcRobotRepresentation->getDimensionCount();k++){
			bool hasNonZeros = IKPlan->gcRobotRepresentation->compute_ddpdq_dqi(IKPlan->endEffectors[i].endEffectorLocalCoords, IKPlan->endEffectors[i].endEffectorRB, ddEndEffectordq_dqi, k);
			if (hasNonZeros == false) continue;
			for (int l=k;l<IKPlan->gcRobotRepresentation->getDimensionCount();l++)
				for (int m=0;m<3;m++){
					double val = ddEndEffectordq_dqi(m, l) * err[m] * IKPlan->endEffectors[i].mask[m];
					if (IKPlan->optimizeRootConfiguration)
						ADD_HES_ELEMENT(hessianEntries, k, l, val, weight);
					else
						if (k>=6)
							ADD_HES_ELEMENT(hessianEntries, k-6, l-6, val, weight);
				}
		}

		//now add the outer product of the jacobians...
		for (int k=0;k<IKPlan->gcRobotRepresentation->getDimensionCount();k++){
			for (int l=k;l<IKPlan->gcRobotRepresentation->getDimensionCount();l++){
				double val = 0;
				for (int m=0;m<3;m++)
					val += dEndEffectordq(m, k) * dEndEffectordq(m, l) * IKPlan->endEffectors[i].mask[m];
				if (IKPlan->optimizeRootConfiguration)
					ADD_HES_ELEMENT(hessianEntries, k, l, val, weight);
				else
					if (k>=6)
						ADD_HES_ELEMENT(hessianEntries, k - 6, l - 6, val, weight);
			}
		}
	}
}
*/


void IK_EndEffectorsObjective::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	//assume the parameters of the motion plan have been set already by the collection of objective functions class
	//IKPlan->setParametersFromList(p);

	auto addHessianPosError = [&](IK_EndEffector const & endEffector, int p_active_start_idx, DynamicArray<MTriplet>& hessianEntries)
	{
		int nP = IKPlan->gcRobotRepresentation->getDimensionCount();
		// position error
		double result = 0.0;
		P3D targetPos = endEffector.targetEEPos;
		P3D currentPos = IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(endEffector.endEffectorLocalCoords, 
																			   endEffector.endEffectorRB);
		V3D errPos = currentPos - targetPos;
		// first derivative
		MatrixNxM dPosEEdq;
		IKPlan->gcRobotRepresentation->compute_dpdq(endEffector.endEffectorLocalCoords, 
											endEffector.endEffectorRB,
											dPosEEdq);
		// second derivative
		MatrixNxM ddPosEEdq_dql;

		for(int l = p_active_start_idx; l < nP; ++l) {
			bool hasNonZeros = IKPlan->gcRobotRepresentation->compute_ddpdq_dqi(endEffector.endEffectorLocalCoords, 
																				endEffector.endEffectorRB,
																				ddPosEEdq_dql, l);
			if (hasNonZeros == false) continue;
			for(int m = l; m < nP; ++m) {
				double val = 0.0;
				for(int k = 0; k < 3; ++k) {
					val += dPosEEdq(k, m) * dPosEEdq(k,l) * endEffector.positionMask[k];
					val += errPos[k] * ddPosEEdq_dql(k, m) * endEffector.positionMask[k];
				}
				ADD_HES_ELEMENT(hessianEntries, l-p_active_start_idx, m-p_active_start_idx, val, weight);
			}
		}
	};

	auto addHessianVec_i_Error = [&](IK_EndEffector const & endEffector, int p_active_start_idx, DynamicArray<MTriplet>& hessianEntries, int i_vec)
	{
		int nP = IKPlan->gcRobotRepresentation->getDimensionCount();
		double s2 = std::pow(endEffector.lengthScaleOrientation, 2);
		// error
		V3D targetVec = endEffector.targetEEOrientation[i_vec];
		V3D currentVec = IKPlan->gcRobotRepresentation->getWorldCoordinatesFor(endEffector.endEffectorLocalOrientation[i_vec],
																				endEffector.endEffectorRB);
		V3D errVec = targetVec - currentVec;
		// first derivative
		MatrixNxM dVecEEdq;
		IKPlan->gcRobotRepresentation->compute_dvdq(endEffector.endEffectorLocalOrientation[i_vec], 
											endEffector.endEffectorRB,
											dVecEEdq);
		// second derivative
		MatrixNxM ddVecEEdq_dql;

		for(int l = p_active_start_idx; l < nP; ++l) {
			bool hasNonZeros = IKPlan->gcRobotRepresentation->compute_ddvdq_dqi(endEffector.endEffectorLocalCoords, 
																				endEffector.endEffectorRB,
																				ddVecEEdq_dql, l);
			if (hasNonZeros == false) continue;
			for(int m = l; m < nP; ++m) {
				double val = 0.0;
				val += dVecEEdq.col(l).dot(dVecEEdq.col(m)) * endEffector.orientationMask[i_vec];
				val += - ddVecEEdq_dql.col(m).dot(errVec) * endEffector.orientationMask[i_vec];
				ADD_HES_ELEMENT(hessianEntries, l-p_active_start_idx, m-p_active_start_idx, val, weight);
			}
		}
	};

	auto addHessianVecError = [&](IK_EndEffector const & endEffector, int p_active_start_idx, DynamicArray<MTriplet>& hessianEntries)
	{
		for(int i = 0; i < 3; ++i) {
			addHessianVec_i_Error(endEffector, p_active_start_idx, hessianEntries, i);
		}
	};


	// if root configuration is not part of the optimization, ignore the first 6 parameters
	int p_active_start_idx = IKPlan->optimizeRootConfiguration ? 0 : 6;

	for(IK_EndEffector & endEffector: IKPlan->endEffectors) {
		addHessianPosError(endEffector, p_active_start_idx, hessianEntries);
		if(endEffector.lengthScaleOrientation > 0.0) {
			addHessianVecError(endEffector, p_active_start_idx, hessianEntries);
		}
	}
	
}