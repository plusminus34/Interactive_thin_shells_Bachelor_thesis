#include <ControlLib/IK_EndEffectorsObjective.h>

IK_EndEffectorsObjective::IK_EndEffectorsObjective(IK_Plan* mp, const std::string& objectiveDescription, double weight){
	IKPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;
}

IK_EndEffectorsObjective::~IK_EndEffectorsObjective(void){
}

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



