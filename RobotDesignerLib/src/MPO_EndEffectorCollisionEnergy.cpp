#include <RobotDesignerLib/MPO_EndEffectorCollisionEnergy.h>

MPO_EndEffectorCollisionEnergy::MPO_EndEffectorCollisionEnergy(LocomotionEngineMotionPlan* mp, const std::string& objectiveDescription, double weight) {
	theMotionPlan = mp;
	this->description = objectiveDescription;
	this->weight = weight;

	boundFunction = std::make_unique<SoftUnilateralConstraint>(theMotionPlan->EEminDistance, 10, 0.02 * 0.02);
}

MPO_EndEffectorCollisionEnergy::~MPO_EndEffectorCollisionEnergy(void){
}

double MPO_EndEffectorCollisionEnergy::computeValue(const dVector& p) {
	if (theMotionPlan->feetPositionsParamsStartIndex < 0/* || theMotionPlan->wheelParamsStartIndex < 0*/)
		return 0;

	double retVal = 0;
	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();

	
	Eigen::Map<const Eigen::MatrixXd> EE(p.data() + theMotionPlan->feetPositionsParamsStartIndex, 3 * nLimbs, theMotionPlan->nSamplePoints);
	for (int i = 0; i < nLimbs; i++)
		for (int j = 0; j < i; j++)	{
			if (theMotionPlan->endEffectorTrajectories[i].endEffectorRB == theMotionPlan->endEffectorTrajectories[j].endEffectorRB)
				continue;
			boundFunction->limit = theMotionPlan->EEminDistance;
			if (theMotionPlan->endEffectorTrajectories[i].isWheel)
				boundFunction->limit += theMotionPlan->endEffectorTrajectories[i].wheelRadius;

			if (theMotionPlan->endEffectorTrajectories[j].isWheel)
				boundFunction->limit += theMotionPlan->endEffectorTrajectories[j].wheelRadius;

			boundFunction->limit *= boundFunction->limit;

			auto EExi = EE.row(3 * i);
			auto EEzi = EE.row(3 * i+2);
			auto EExj = EE.row(3 * j);
			auto EEzj = EE.row(3 * j+2);
			retVal += ((EExi - EExj).cwiseAbs2() + (EEzi - EEzj).cwiseAbs2()).unaryExpr([&](double val) {return boundFunction->computeValue(val); }).sum();
		}
	return retVal*weight;
}

void MPO_EndEffectorCollisionEnergy::addGradientTo(dVector& grad, const dVector& p) {
	if (theMotionPlan->feetPositionsParamsStartIndex < 0/* || theMotionPlan->wheelParamsStartIndex < 0*/)
		return;

	double retVal = 0;
	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();


	Eigen::Map<const Eigen::MatrixXd> EE(p.data() + theMotionPlan->feetPositionsParamsStartIndex, 3 * nLimbs, theMotionPlan->nSamplePoints);
	Eigen::Map<Eigen::MatrixXd> G(grad.data() + theMotionPlan->feetPositionsParamsStartIndex, 3 * nLimbs, theMotionPlan->nSamplePoints);
	for (int i = 0; i < nLimbs; i++)
		for (int j = 0; j < i; j++)	{
			if (theMotionPlan->endEffectorTrajectories[i].endEffectorRB == theMotionPlan->endEffectorTrajectories[j].endEffectorRB)
				continue;
			boundFunction->limit = theMotionPlan->EEminDistance;
			if (theMotionPlan->endEffectorTrajectories[i].isWheel)
				boundFunction->limit += theMotionPlan->endEffectorTrajectories[i].wheelRadius;

			if (theMotionPlan->endEffectorTrajectories[j].isWheel)
				boundFunction->limit += theMotionPlan->endEffectorTrajectories[j].wheelRadius;

			boundFunction->limit *= boundFunction->limit;

			auto EExi = EE.row(3 * i);
			auto EExj = EE.row(3 * j);
			auto EEzi = EE.row(3 * i + 2);
			auto EEzj = EE.row(3 * j + 2);
			dVector dx = EExi - EExj;
			dVector dz = EEzi - EEzj;
			dVector g = (dx.cwiseAbs2() + dz.cwiseAbs2()).unaryExpr([&](double val) {return boundFunction->computeDerivative(val); });
			G.row(3 * i) +=  weight * (2 * dx).cwiseProduct(g);
			G.row(3 * j) += -weight * (2 * dx).cwiseProduct(g);
			G.row(3 * i+2) += weight * (2 * dz).cwiseProduct(g);
			G.row(3 * j+2) += -weight * (2 * dz).cwiseProduct(g);
		}
}

void MPO_EndEffectorCollisionEnergy::addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p) {
	if (theMotionPlan->feetPositionsParamsStartIndex < 0 /*|| theMotionPlan->wheelParamsStartIndex < 0*/)
		return;


	const int nLimbs = theMotionPlan->endEffectorTrajectories.size();
	Eigen::Map<const Eigen::MatrixXd> Q(p.data() + theMotionPlan->feetPositionsParamsStartIndex, 3 * nLimbs, theMotionPlan->nSamplePoints);

	for (int i = 0; i < nLimbs; i++){
		for (int j = 0; j < i; j++){
			if (theMotionPlan->endEffectorTrajectories[i].endEffectorRB == theMotionPlan->endEffectorTrajectories[j].endEffectorRB)
				continue;
			boundFunction->limit = theMotionPlan->EEminDistance;
			if (theMotionPlan->endEffectorTrajectories[i].isWheel)
				boundFunction->limit += theMotionPlan->endEffectorTrajectories[i].wheelRadius;

			if (theMotionPlan->endEffectorTrajectories[j].isWheel)
				boundFunction->limit += theMotionPlan->endEffectorTrajectories[j].wheelRadius;

			boundFunction->limit *= boundFunction->limit;

			auto Qxi = Q.row(3 * i);
			auto Qxj = Q.row(3 * j);
			auto Qzi = Q.row(3 * i + 2);
			auto Qzj = Q.row(3 * j + 2);
			dVector dx = Qxi - Qxj;
			dVector dz = Qzi - Qzj;
			dVector dx2 = dx.cwiseAbs2();
			dVector dz2 = dz.cwiseAbs2();
			dVector dxdz = dx.cwiseProduct(dz);
			dVector err = dx2 + dz2;
			dVector g = err.unaryExpr([&](double val) {return boundFunction->computeDerivative(val); });
			dVector h = err.unaryExpr([&](double val) {return boundFunction->computeSecondDerivative(val); });
			for (int k = 0; k < theMotionPlan->nSamplePoints; k++)
			{
				int I = theMotionPlan->feetPositionsParamsStartIndex + 3 * nLimbs*k + 3 * i;
				int J = theMotionPlan->feetPositionsParamsStartIndex + 3 * nLimbs*k + 3 * j;
				ADD_HES_ELEMENT(hessianEntries, I, I, 2*g(k) + 4*dx2(k)*h(k), weight);
				ADD_HES_ELEMENT(hessianEntries, J, J, 2*g(k) + 4*dx2(k)*h(k), weight);
				ADD_HES_ELEMENT(hessianEntries, I, J,-2*g(k) - 4*dx2(k)*h(k), weight);

				ADD_HES_ELEMENT(hessianEntries, I+2, I+2, 2*g(k) + 4*dz2(k)*h(k), weight);
				ADD_HES_ELEMENT(hessianEntries, J+2, J+2, 2*g(k) + 4*dz2(k)*h(k), weight);
				ADD_HES_ELEMENT(hessianEntries, I+2, J+2,-2*g(k) - 4*dz2(k)*h(k), weight);

				ADD_HES_ELEMENT(hessianEntries, I, I+2, 4*dxdz(k)*h(k), weight);
				ADD_HES_ELEMENT(hessianEntries, J, J+2, 4*dxdz(k)*h(k), weight);
				ADD_HES_ELEMENT(hessianEntries, I, J+2, -4*dxdz(k)*h(k), weight);
				ADD_HES_ELEMENT(hessianEntries, I+2, J, -4*dxdz(k)*h(k), weight);
			}
		}
	}
}
