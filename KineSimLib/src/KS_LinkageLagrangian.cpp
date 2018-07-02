#include "KineSimLib/KS_LinkageLagrangian.h"
#include "KineSimLib/KS_Constraint.h"

int iter = 0;

KS_LinkageLagrangian::KS_LinkageLagrangian(KS_MechanicalAssembly* a, DynamicArray<double> &s, std::pair<Point3d, int> endEff, DynamicArray<Point3d> &c){
	assert(a != NULL);
	assert(a->getComponentCount() >= 1);
	assembly = a;
	pin1=NULL;
	pin2=NULL;
	actuator=NULL;

	ticks = s;
	endEffector = endEff;
	curve = c;

	epsilon = 0.00001;
	regularizer = 0.0;
	mu = 1.0;
	wCurve = 1.0;
	wArm = 0.01;
	wAngle = 0.001;

	constraints.clear();
	for (int i=0; i<assembly->getConnectionCount(); i++)
		assembly->getConnection(i)->addConstraintsToList(constraints);
	assemblyConstraintSize = 0;
	for (uint i=0;i<constraints.size();i++){
		constraints[i]->cleanSparseMatrixBlocks();
		constraints[i]->setConstraintStartIndex(assemblyConstraintSize);
		assemblyConstraintSize += constraints[i]->getConstraintCount();
	}
	constraintSize=assemblyConstraintSize*ticks.size();
	paramSize = 5;
	assemblyStateSize = KS_MechanicalComponent::getStateSize()*assembly->getComponentCount();
	stateSize = assemblyStateSize*ticks.size();

	dCds.resize(assemblyConstraintSize, assemblyStateSize, false);
}

KS_LinkageLagrangian::~KS_LinkageLagrangian(void){
}

void KS_LinkageLagrangian::initialize(KS_BindComponentsConnection* p1, KS_BindComponentsConnection* p2, KS_MotorConnection* motor){
	pin1 = p1;
	pin2 = p2;
	actuator = motor;

	dE_ds.resize(stateSize+paramSize+constraintSize); setValues(dE_ds, 0);
	ddE_dsds.resize(stateSize+paramSize+constraintSize, stateSize+paramSize+constraintSize, true);
	ddE_dsds.zero();
	Hessian.resize(stateSize+paramSize+constraintSize, stateSize+paramSize+constraintSize);
}

//regularizer looks like: r/2 * (p-p0)'*(p-p0). This function can update p0 if desired, given the current value of p.
void KS_LinkageLagrangian::updateRegularizingSolutionToCurrentIterationBest(const dVector &currentP){}

double KS_LinkageLagrangian::computeValue(const double* p){
	dVector pVec;
	for (uint i=0;i<pVec.size();i++)	pVec[i] = p[i];
	return computeValue(pVec);
}

double KS_LinkageLagrangian::computeValue(const dVector& p){
	double objectiveValue = 0;
	double constraintValue = 0;
	Point3d x1 (p[stateSize],p[stateSize+1],0);
	Point3d x2 (p[stateSize+2],p[stateSize+3],0);
	Point3d x3 (-0.5*p[stateSize+4],0,0);
	Point3d x4 (0.5*p[stateSize+4],0,0);
	pin1->setPinOnC1(x1);
	pin1->setPinOnC2(x3);
	pin2->setPinOnC1(x2);
	pin2->setPinOnC2(x4);
	for(uint i=0;i<ticks.size();i++){
		dVector si;
		si.insert(si.begin(),p.begin()+i*assemblyStateSize,p.begin()+(i+1)*assemblyStateSize);
		assembly->setAssemblyState(si);
		assembly->setTickerValue(ticks[i]);
		Point3d wp1 = pin1->getInput()->get_w(x1);
		Point3d wp2 = pin2->getInput()->get_w(x2);
		Point3d wac = actuator->getInput()->get_w(actuator->getPinOnC1());
		double dot = (wp2-wp1).dotProductWith(wac-wp1);
		objectiveValue+=0.5*wCurve*(assembly->getComponent(endEffector.second)->get_w(endEffector.first)-curve[i]).length2();
		objectiveValue+=wArm/(epsilon+(wp1-wac).length2()-dot*dot/(wp1-wp2).length2());
		dVector param;
		param.insert(param.begin(),p.begin()+stateSize, p.begin()+stateSize+5);
		objectiveValue+=wAngle*angleSensitivity(param);
		dVector C (assemblyConstraintSize,0);
		for(uint j=0;j<constraints.size();j++){
			constraints[j]->setConstraintValues(C);
		}
//		dVector lambda;
//		lambda.insert(lambda.begin(),p.begin()+stateSize+paramSize+i*assemblyConstraintSize,p.begin()+stateSize+paramSize+(i+1)*assemblyConstraintSize);
//		constraintValue+=dotprod(lambda,C);
		for(int j=0;j<assemblyConstraintSize;j++)
			constraintValue+=std::abs(C[j]);
	}
	return objectiveValue+mu*constraintValue;
}

double KS_LinkageLagrangian::getValue(const dVector& p){
	double totalConstraintEnergy = 0;
	Point3d x1 (p[stateSize],p[stateSize+1],0);
	Point3d x2 (p[stateSize+2],p[stateSize+3],0);
	Point3d x3 (-0.5*p[stateSize+4],0,0);
	Point3d x4 (0.5*p[stateSize+4],0,0);
	pin1->setPinOnC1(x1);
	pin1->setPinOnC2(x3);
	pin2->setPinOnC1(x2);
	pin2->setPinOnC2(x4);
	for(uint i=0;i<ticks.size();i++){
		dVector si;
		si.insert(si.begin(),p.begin()+i*assemblyStateSize,p.begin()+(i+1)*assemblyStateSize);
		assembly->setAssemblyState(si);
		assembly->setTickerValue(ticks[i]);
		Point3d wp1 = pin1->getInput()->get_w(x1);
		Point3d wp2 = pin2->getInput()->get_w(x2);
		Point3d wac = actuator->getInput()->get_w(actuator->getPinOnC1());
		double dot = (wp2-wp1).dotProductWith(wac-wp1);
		totalConstraintEnergy+=0.5*wCurve*(assembly->getComponent(endEffector.second)->get_w(endEffector.first)-curve[i]).length2();
		totalConstraintEnergy+=wArm/(epsilon+(wp1-wac).length2()-dot*dot/(wp1-wp2).length2());
		dVector param;
		param.insert(param.begin(),p.begin()+stateSize, p.begin()+stateSize+5);
		totalConstraintEnergy+=wAngle*angleSensitivity(param);
		dVector C (assemblyConstraintSize,0);
		for(uint j=0;j<constraints.size();j++){
			constraints[j]->setConstraintValues(C);
		}
		dVector lambda;
		lambda.insert(lambda.begin(),p.begin()+stateSize+paramSize+i*assemblyConstraintSize,p.begin()+stateSize+paramSize+(i+1)*assemblyConstraintSize);
		totalConstraintEnergy+=dotprod(lambda,C);
	}
	return totalConstraintEnergy;
}

double KS_LinkageLagrangian::angleSensitivity(const dVector &p){
	Point3d wp1 = pin1->getInput()->get_w(Point3d(p[0],p[1],0));
	Point3d wp2 = pin2->getInput()->get_w(Point3d(p[2],p[3],0));
	Point3d wac = actuator->getInput()->get_w(actuator->getPinOnC1());
	double linkLength2 = (wp2-wp1).length2();
	double dist2 = (wac-wp1).length2();
	double linkLength = sqrt(linkLength2);
	double dist = sqrt(dist2);
	double dot = (wp2-wp1).dotProductWith(wac-wp1);
	Matrix3x3 r1,r2;
	pin1->getInput()->get_dw_dp(&r1);
	pin2->getInput()->get_dw_dp(&r2);
	dVector ddot_dp (4, 0);
	ddot_dp[0] = r1.column(0).dotProductWith((wp1-wac+wp1-wp2)/(dist*linkLength)-((wp1-wp2)*dist/linkLength+(wp1-wac)*linkLength/dist)*dot/(linkLength2*dist2));
	ddot_dp[1] = r1.column(1).dotProductWith((wp1-wac+wp1-wp2)/(dist*linkLength)-((wp1-wp2)*dist/linkLength+(wp1-wac)*linkLength/dist)*dot/(linkLength2*dist2));
	ddot_dp[2] = r2.column(0).dotProductWith((wac-wp1)/(dist*linkLength)+(wp1-wp2)*dot/(linkLength2*linkLength*dist));
	ddot_dp[3] = r2.column(1).dotProductWith((wac-wp1)/(dist*linkLength)+(wp1-wp2)*dot/(linkLength2*linkLength*dist));
	return dotprod(ddot_dp,ddot_dp)/(1.0-dot*dot/(linkLength2*dist2));
}

void KS_LinkageLagrangian::momentArmStateGradient(dVector &dEds, const dVector &p){
	Point3d x1 (p[0],p[1],0);
	Point3d x2 (p[2],p[3],0);
	Matrix dw_ds (3,KS_MechanicalComponent::getStateSize());
	Point3d wp1 = pin1->getInput()->get_w(x1);
	Point3d wp2 = pin2->getInput()->get_w(x2);
	Point3d wac = actuator->getInput()->get_w(actuator->getPinOnC1());
	double linkLength2 = (wp1-wp2).length2();
	double dist2 = (wp1-wac).length2();
	double dot = (wp2-wp1).dotProductWith(wac-wp1);
	double denom = epsilon+(wp1-wac).length2()-dot*dot/(wp1-wp2).length2();
	denom *= denom;
	dEds.resize(assemblyStateSize);
	dVector dM_ds (KS_MechanicalComponent::getStateSize(),0);
	pin1->getInput()->get_dw_ds(x1,dw_ds);
	addPreMultiply(dw_ds,wp1-wac,-2.0,dM_ds);
	addPreMultiply(dw_ds,wp1-wac+wp1-wp2,2.0*dot/linkLength2,dM_ds);
	addPreMultiply(dw_ds,wp1-wp2,-2.0*dot*dot/(linkLength2*linkLength2),dM_ds);
	for(int i=0;i<KS_MechanicalComponent::getStateSize();i++)
		dEds[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+i]+=dM_ds[i]/denom;
	actuator->getInput()->get_dw_ds(actuator->getPinOnC1(),dw_ds);
	dM_ds.clear();
	dM_ds.resize(KS_MechanicalComponent::getStateSize(),0);
	addPreMultiply(dw_ds,wac-wp1,-2.0,dM_ds);
	addPreMultiply(dw_ds,wp2-wp1,2.0*dot/linkLength2,dM_ds);
	for(int i=0;i<KS_MechanicalComponent::getStateSize();i++)
		dEds[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+i]+=dM_ds[i]/denom;
	pin2->getInput()->get_dw_ds(x2,dw_ds);
	dM_ds.clear();
	dM_ds.resize(KS_MechanicalComponent::getStateSize(),0);
	addPreMultiply(dw_ds,wac-wp1,2.0*dot/linkLength2,dM_ds);
	addPreMultiply(dw_ds,wp2-wp1,-2.0*dot*dot/(linkLength2*linkLength2),dM_ds);
	for(int i=0;i<KS_MechanicalComponent::getStateSize();i++)
		dEds[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+i]+=dM_ds[i]/denom;
}

void KS_LinkageLagrangian::angleSensitivityStateGradient(dVector &dE_ds, const dVector &s, const dVector &p){
	dE_ds.resize(assemblyStateSize);
	double ds=0.0001;
	double gradP, gradM;
	Vector3d angles;
	Point3d center;
	angles[0] = s[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
	angles[1] = s[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
	angles[2] = s[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
	for(uint j=0;j<3;j++){
		double temp=angles[j];
		angles[j]=temp+ds;
		pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
		gradP = angleSensitivity(p);
		angles[j]=temp-ds;
		pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
		gradM = angleSensitivity(p);
		dE_ds[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]+=(gradP-gradM)/(2.0*ds);
		angles[j]=temp;
	}
	pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
	center[0] = s[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
	center[1] = s[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
	center[2] = s[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
	for(uint j=0;j<3;j++){
		double temp=center[j];
		center[j]=temp+ds;
		pin1->getInput()->setWorldCenterPosition(center);
		gradP = angleSensitivity(p);
		center[j]=temp-ds;
		pin1->getInput()->setWorldCenterPosition(center);
		gradM = angleSensitivity(p);
		dE_ds[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=(gradP-gradM)/(2.0*ds);
		center[j]=temp;
	}
	pin1->getInput()->setWorldCenterPosition(center);

	if(pin2->getInput()!=pin1->getInput()){
		angles[0] = s[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
		angles[1] = s[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
		angles[2] = s[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
		for(uint j=0;j<3;j++){
			double temp=angles[j];
			angles[j]=temp+ds;
			pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
			gradP = angleSensitivity(p);
			angles[j]=temp-ds;
			pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
			gradM = angleSensitivity(p);
			dE_ds[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]+=(gradP-gradM)/(2.0*ds);
			angles[j]=temp;
		}
		pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
		center[0] = s[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
		center[1] = s[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
		center[2] = s[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
		for(uint j=0;j<3;j++){
			double temp=center[j];
			center[j]=temp+ds;
			pin2->getInput()->setWorldCenterPosition(center);
			gradP = angleSensitivity(p);
			center[j]=temp-ds;
			pin2->getInput()->setWorldCenterPosition(center);
			gradM = angleSensitivity(p);
			dE_ds[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=(gradP-gradM)/(2.0*ds);
			center[j]=temp;
		}
		pin2->getInput()->setWorldCenterPosition(center);
	}

	if((actuator->getInput()!=pin1->getInput())&&(actuator->getInput()!=pin2->getInput())){
		angles[0] = s[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
		angles[1] = s[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
		angles[2] = s[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
		for(uint j=0;j<3;j++){
			dVector dAdsP, dAdsM;
			double temp=angles[j];
			angles[j]=temp+ds;
			actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
			gradP = angleSensitivity(p);
			angles[j]=temp-ds;
			actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
			gradM = angleSensitivity(p);
			dE_ds[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]+=(gradP-gradM)/(2.0*ds);
			angles[j]=temp;
		}
		actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
		center[0] = s[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
		center[1] = s[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
		center[2] = s[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
		for(uint j=0;j<3;j++){
			double temp=center[j];
			center[j]=temp+ds;
			actuator->getInput()->setWorldCenterPosition(center);
			gradP = angleSensitivity(p);
			center[j]=temp-ds;
			actuator->getInput()->setWorldCenterPosition(center);
			gradM = angleSensitivity(p);
			dE_ds[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=(gradP-gradM)/(2.0*ds);
			center[j]=temp;
		}
		actuator->getInput()->setWorldCenterPosition(center);
	}
}

void KS_LinkageLagrangian::computeGradient(dVector& gradient, const dVector &p){
	gradient.resize(stateSize+paramSize+constraintSize);
	setValues(gradient, 0);
	Point3d x1 (p[stateSize],p[stateSize+1],0);
	Point3d x2 (p[stateSize+2],p[stateSize+3],0);
	Point3d x3 (-0.5*p[stateSize+4],0,0);
	Point3d x4 (0.5*p[stateSize+4],0,0);
	pin1->setPinOnC1(x1);
	pin1->setPinOnC2(x3);
	pin2->setPinOnC1(x2);
	pin2->setPinOnC2(x4);
	for(uint i=0;i<ticks.size();i++){
		dVector si;
		si.insert(si.begin(),p.begin()+i*assemblyStateSize,p.begin()+(i+1)*assemblyStateSize);
		assembly->setAssemblyState(si);
		assembly->setTickerValue(ticks[i]);
		Matrix3x3 r1;
		pin1->getInput()->get_dw_dp(&r1);
		Matrix3x3 r2;
		pin2->getInput()->get_dw_dp(&r2);
		Matrix3x3 r3;
		pin1->getOutput()->get_dw_dp(&r3);
		Point3d wp1 = pin1->getInput()->get_w(x1);
		Point3d wp2 = pin2->getInput()->get_w(x2);
		Point3d wac = actuator->getInput()->get_w(actuator->getPinOnC1());
		double linkLength2 = (wp1-wp2).length2();
		double dist2 = (wp1-wac).length2();
		double linkLength = sqrt(linkLength2);
		double dist = sqrt(dist2);
		double dot = (wp2-wp1).dotProductWith(wac-wp1);
		double denom = epsilon+(wp1-wac).length2()-dot*dot/(wp1-wp2).length2();
		denom *= denom;
		//end effector related terms
		Matrix dw_ds (3,KS_MechanicalComponent::getStateSize());
		assembly->getComponent(endEffector.second)->get_dw_ds(endEffector.first,dw_ds);
		Vector3d delta = assembly->getComponent(endEffector.second)->get_w(endEffector.first)-curve[i];
		int endEffectorOffset = i*assemblyStateSize+endEffector.second*KS_MechanicalComponent::getStateSize();
		for(int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			gradient[endEffectorOffset+j]+=wCurve*(dw_ds(0,j)*delta.x+dw_ds(1,j)*delta.y+dw_ds(2,j)*delta.z);

		//pins and actuator states derivatives
		dVector dM_ds;
		dVector param;
		param.insert(param.begin(),p.begin()+stateSize, p.begin()+stateSize+5);

		//dM_ds
		momentArmStateGradient(dM_ds,param);
		for(int j=0;j<assemblyStateSize;j++)
			gradient[i*assemblyStateSize+j]+=wArm*dM_ds[j];

		//dA_ds
		double ds=0.0001;
		double gradP, gradM;
		Vector3d angles;
		Point3d center;
		angles[0] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
		angles[1] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
		angles[2] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
		for(uint j=0;j<3;j++){
			dVector dAdsP, dAdsM, dMdsP, dMdsM;
			double temp=angles[j];
			angles[j]=temp+ds;
			pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
			gradP = angleSensitivity(param);
			angles[j]=temp-ds;
			pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
			gradM = angleSensitivity(param);
			gradient[i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]+=wAngle*(gradP-gradM)/(2.0*ds);
			angles[j]=temp;
		}
		pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
		center[0] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
		center[1] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
		center[2] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
		for(uint j=0;j<3;j++){
			dVector dAdsP, dAdsM, dMdsP, dMdsM;
			double temp=center[j];
			center[j]=temp+ds;
			pin1->getInput()->setWorldCenterPosition(center);
			gradP = angleSensitivity(param);
			center[j]=temp-ds;
			pin1->getInput()->setWorldCenterPosition(center);
			gradM = angleSensitivity(param);
			gradient[i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=wAngle*(gradP-gradM)/(2.0*ds);
			center[j]=temp;
		}
		pin1->getInput()->setWorldCenterPosition(center);

		if(pin2->getInput()!=pin1->getInput()){
			angles[0] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
			angles[1] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
			angles[2] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
			for(uint j=0;j<3;j++){
				dVector dAdsP, dAdsM, dMdsP, dMdsM;
				double temp=angles[j];
				angles[j]=temp+ds;
				pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
				gradP = angleSensitivity(param);
				angles[j]=temp-ds;
				pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
				gradM = angleSensitivity(param);
				gradient[i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]+=wAngle*(gradP-gradM)/(2.0*ds);
				angles[j]=temp;
			}
			pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
			center[0] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
			center[1] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
			center[2] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
			for(uint j=0;j<3;j++){
				dVector dAdsP, dAdsM, dMdsP, dMdsM;
				double temp=center[j];
				center[j]=temp+ds;
				pin2->getInput()->setWorldCenterPosition(center);
				gradP = angleSensitivity(param);
				center[j]=temp-ds;
				pin2->getInput()->setWorldCenterPosition(center);
				gradM = angleSensitivity(param);
				gradient[i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=wAngle*(gradP-gradM)/(2.0*ds);
				center[j]=temp;
			}
			pin2->getInput()->setWorldCenterPosition(center);
		}

		if((actuator->getInput()!=pin1->getInput())&&(actuator->getInput()!=pin2->getInput())){
			angles[0] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
			angles[1] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
			angles[2] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
			for(uint j=0;j<3;j++){
				dVector dAdsP, dAdsM, dMdsP, dMdsM;
				double temp=angles[j];
				angles[j]=temp+ds;
				actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
				gradP = angleSensitivity(param);
				angles[j]=temp-ds;
				actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
				gradM = angleSensitivity(param);
				gradient[i*assemblyStateSize+actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]+=wAngle*(gradP-gradM)/(2.0*ds);
				angles[j]=temp;
			}
			actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
			center[0] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
			center[1] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
			center[2] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
			for(uint j=0;j<3;j++){
				dVector dAdsP, dAdsM, dMdsP, dMdsM;
				double temp=center[j];
				center[j]=temp+ds;
				actuator->getInput()->setWorldCenterPosition(center);
				gradP = angleSensitivity(param);
				center[j]=temp-ds;
				actuator->getInput()->setWorldCenterPosition(center);
				gradM = angleSensitivity(param);
				gradient[i*assemblyStateSize+actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=wAngle*(gradP-gradM)/(2.0*ds);
				center[j]=temp;
			}
			actuator->getInput()->setWorldCenterPosition(center);
		}

		//parameters related terms
		//dM_dp
		dVector dE_dp (stateSize,0);
		dE_dp[0]=r1.column(0).dotProductWith((wac-wp1)*2+(wp1-wac+wp1-wp2)*2*dot/linkLength2+(wp2-wp1)*2*dot*dot/(linkLength2*linkLength2))/denom;
		dE_dp[1]=r1.column(1).dotProductWith((wac-wp1)*2+(wp1-wac+wp1-wp2)*2*dot/linkLength2+(wp2-wp1)*2*dot*dot/(linkLength2*linkLength2))/denom;
		dE_dp[2]=r2.column(0).dotProductWith((wac-wp1)*2*dot/linkLength2-(wp2-wp1)*2*dot*dot/(linkLength2*linkLength2))/denom;
		dE_dp[3]=r2.column(1).dotProductWith((wac-wp1)*2*dot/linkLength2-(wp2-wp1)*2*dot*dot/(linkLength2*linkLength2))/denom;
		gradient[stateSize]+=wArm*dE_dp[0];
		gradient[stateSize+1]+=wArm*dE_dp[1];
		gradient[stateSize+2]+=wArm*dE_dp[2];
		gradient[stateSize+3]+=wArm*dE_dp[3];

		//dA_dp
		dVector paramP;
		paramP.insert(paramP.begin(),p.begin()+stateSize, p.begin()+stateSize+5);
		dVector paramM;
		paramM.insert(paramM.begin(),p.begin()+stateSize, p.begin()+stateSize+5);
		for(uint j=0;j<5;j++){
			paramP[j]+=ds;
			paramM[j]-=ds;
			gradient[stateSize+j]+=wAngle*(angleSensitivity(paramP)-angleSensitivity(paramM))/(2.0*ds);
			paramP[j]-=ds;
			paramM[j]+=ds;
		}

		//dC_cp
		int pin1Offset = stateSize+paramSize+i*assemblyConstraintSize+pin1->getPinConstraintIndex();
		int pin2Offset = stateSize+paramSize+i*assemblyConstraintSize+pin2->getPinConstraintIndex();
		gradient[stateSize]+=r1(0,0)*p[pin1Offset]+r1(1,0)*p[pin1Offset+1]+r1(2,0)*p[pin1Offset+2];
		gradient[stateSize+1]+=r1(0,1)*p[pin1Offset]+r1(1,1)*p[pin1Offset+1]+r1(2,1)*p[pin1Offset+2];
		gradient[stateSize+2]+=r2(0,0)*p[pin2Offset]+r2(1,0)*p[pin2Offset+1]+r2(2,0)*p[pin2Offset+2];
		gradient[stateSize+3]+=r2(0,1)*p[pin2Offset]+r2(1,1)*p[pin2Offset+1]+r2(2,1)*p[pin2Offset+2];
		gradient[stateSize+4]+=0.5*(r3(0,0)*(p[pin1Offset]-p[pin2Offset])+r3(1,0)*(p[pin1Offset+1]-p[pin2Offset+1])+r3(2,0)*(p[pin1Offset+2]-p[pin2Offset+2]));

		//constraint related terms
		dVector C (assemblyConstraintSize,0);
		//SparseMatrix dCds (assemblyConstraintSize, assemblyStateSize, false);
		for(uint j=0;j<constraints.size();j++){
			constraints[j]->computeEnergyGradientAndHessian(si);
			constraints[j]->setConstraintValues(C);
			constraints[j]->setConstraintJacobianValues(dCds);
		}
		for(int j=0;j<assemblyConstraintSize;j++)
			gradient[stateSize+paramSize+i*assemblyConstraintSize+j]+=C[j];
		for(int j=0;j<assemblyConstraintSize;j++){
			for(int k=0;k<assemblyStateSize;k++){
				gradient[i*assemblyStateSize+k]+=dCds.getElementAt(j,k)*p[stateSize+paramSize+i*assemblyConstraintSize+j];
			}
		}
	}
	/*
	double ds = 0.0001;
	dVector pi;
	copy(p,pi);
	for(uint i=0;i<p.size();i++){
		double valP, valM;
		double temp = pi[i];
		pi[i]=temp+ds;
		valP = computeValue(pi);
		pi[i]=temp-ds;
		valM = computeValue(pi);
		double error = fabs(gradient[i]-(valP-valM)/(2*ds));
		if(error>10e-6)
			error=0;
		pi[i]=temp;
	}
	*/
}

void KS_LinkageLagrangian::precomputeDerivativeInformationAt(const dVector &p){
	DynamicArray<Eigen::Triplet<double> > tripletList;
	setValues(dE_ds, 0);
	ddE_dsds.resize(stateSize+paramSize+constraintSize, stateSize+paramSize+constraintSize, true);
	ddE_dsds.zero();
	Hessian.setZero();
	regularizer=0.02;
	Point3d x1 (p[stateSize],p[stateSize+1],0);
	Point3d x2 (p[stateSize+2],p[stateSize+3],0);
	Point3d x3 (-0.5*p[stateSize+4],0,0);
	Point3d x4 (0.5*p[stateSize+4],0,0);
	pin1->setPinOnC1(x1);
	pin1->setPinOnC2(x3);
	pin2->setPinOnC1(x2);
	pin2->setPinOnC2(x4);
	for(uint i=0;i<ticks.size();i++){
		dVector si;
		si.insert(si.begin(),p.begin()+i*assemblyStateSize,p.begin()+(i+1)*assemblyStateSize);
		assembly->setAssemblyState(si);
		assembly->setTickerValue(ticks[i]);
		Matrix3x3 r1;
		pin1->getInput()->get_dw_dp(&r1);
		Matrix3x3 r2;
		pin2->getInput()->get_dw_dp(&r2);
		Matrix3x3 r3;
		pin1->getOutput()->get_dw_dp(&r3);
		Point3d wp1 = pin1->getInput()->get_w(x1);
		Point3d wp2 = pin2->getInput()->get_w(x2);
		Point3d wac = actuator->getInput()->get_w(actuator->getPinOnC1());
		double linkLength2 = (wp1-wp2).length2();
		double dist2 = (wp1-wac).length2();
		double linkLength = sqrt(linkLength2);
		double dist = sqrt(dist2);
		double dot = (wp2-wp1).dotProductWith(wac-wp1);
		double denom = epsilon+(wp1-wac).length2()-dot*dot/(wp1-wp2).length2();
		denom *= denom;
		//end effector related terms
		Matrix dw_ds (3,KS_MechanicalComponent::getStateSize());
		assembly->getComponent(endEffector.second)->get_dw_ds(endEffector.first,dw_ds);
		Vector3d delta = assembly->getComponent(endEffector.second)->get_w(endEffector.first)-curve[i];
		int endEffectorOffset = i*assemblyStateSize+endEffector.second*KS_MechanicalComponent::getStateSize();
		for(int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			dE_ds[endEffectorOffset+j]+=wCurve*(dw_ds(0,j)*delta.x+dw_ds(1,j)*delta.y+dw_ds(2,j)*delta.z);
		
		Matrix dw_dsTdw_ds (KS_MechanicalComponent::getStateSize(),KS_MechanicalComponent::getStateSize());
		dw_dsTdw_ds.setToATransposedB(dw_ds,dw_ds);
		for(int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			for(int k=j;k<KS_MechanicalComponent::getStateSize();k++)
				ddE_dsds.addToElementAt(endEffectorOffset+j,endEffectorOffset+k,wCurve*dw_dsTdw_ds(j,k));

		for(int j=0;j<KS_MechanicalComponent::getStateSize();j++)
			for(int k=0;k<KS_MechanicalComponent::getStateSize();k++)
				tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset+j,endEffectorOffset+k,wCurve*dw_dsTdw_ds(j,k)));

		Matrix ddw_dsds (3,3);
		assembly->getComponent(endEffector.second)->get_ddw_dgds(endEffector.first,ddw_dsds);
		ddE_dsds.addToElementAt(endEffectorOffset,endEffectorOffset,wCurve*(ddw_dsds(0,0)*delta.x+ddw_dsds(1,0)*delta.y+ddw_dsds(2,0)*delta.z));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset,endEffectorOffset,wCurve*(ddw_dsds(0,0)*delta.x+ddw_dsds(1,0)*delta.y+ddw_dsds(2,0)*delta.z)));
		ddE_dsds.addToElementAt(endEffectorOffset,endEffectorOffset+1,wCurve*(ddw_dsds(0,1)*delta.x+ddw_dsds(1,1)*delta.y+ddw_dsds(2,1)*delta.z));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset,endEffectorOffset+1,wCurve*(ddw_dsds(0,1)*delta.x+ddw_dsds(1,1)*delta.y+ddw_dsds(2,1)*delta.z)));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset+1,endEffectorOffset,wCurve*(ddw_dsds(0,1)*delta.x+ddw_dsds(1,1)*delta.y+ddw_dsds(2,1)*delta.z)));
		ddE_dsds.addToElementAt(endEffectorOffset,endEffectorOffset+2,wCurve*(ddw_dsds(0,2)*delta.x+ddw_dsds(1,2)*delta.y+ddw_dsds(2,2)*delta.z));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset,endEffectorOffset+2,wCurve*(ddw_dsds(0,2)*delta.x+ddw_dsds(1,2)*delta.y+ddw_dsds(2,2)*delta.z)));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset+2,endEffectorOffset,wCurve*(ddw_dsds(0,2)*delta.x+ddw_dsds(1,2)*delta.y+ddw_dsds(2,2)*delta.z)));
		assembly->getComponent(endEffector.second)->get_ddw_dbds(endEffector.first,ddw_dsds);
		ddE_dsds.addToElementAt(endEffectorOffset+1,endEffectorOffset+1,wCurve*(ddw_dsds(0,1)*delta.x+ddw_dsds(1,1)*delta.y+ddw_dsds(2,1)*delta.z));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset+1,endEffectorOffset+1,wCurve*(ddw_dsds(0,1)*delta.x+ddw_dsds(1,1)*delta.y+ddw_dsds(2,1)*delta.z)));
		ddE_dsds.addToElementAt(endEffectorOffset+1,endEffectorOffset+2,wCurve*(ddw_dsds(0,2)*delta.x+ddw_dsds(1,2)*delta.y+ddw_dsds(2,2)*delta.z));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset+1,endEffectorOffset+2,wCurve*(ddw_dsds(0,2)*delta.x+ddw_dsds(1,2)*delta.y+ddw_dsds(2,2)*delta.z)));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset+2,endEffectorOffset+1,wCurve*(ddw_dsds(0,2)*delta.x+ddw_dsds(1,2)*delta.y+ddw_dsds(2,2)*delta.z)));
		assembly->getComponent(endEffector.second)->get_ddw_dads(endEffector.first,ddw_dsds);
		ddE_dsds.addToElementAt(endEffectorOffset+2,endEffectorOffset+2,wCurve*(ddw_dsds(0,2)*delta.x+ddw_dsds(1,2)*delta.y+ddw_dsds(2,2)*delta.z));
		tripletList.push_back(Eigen::Triplet<double>(endEffectorOffset+2,endEffectorOffset+2,wCurve*(ddw_dsds(0,2)*delta.x+ddw_dsds(1,2)*delta.y+ddw_dsds(2,2)*delta.z)));

		//pins and actuator states derivatives
		dVector dM_ds;
		dVector param;
		param.insert(param.begin(),p.begin()+stateSize, p.begin()+stateSize+5);

		//dM_ds
		momentArmStateGradient(dM_ds,param);
		for(int j=0;j<assemblyStateSize;j++)
			dE_ds[i*assemblyStateSize+j]+=wArm*dM_ds[j];

		//dA_ds, ddA_dsds and ddM_dsds
		int pin1Offset = pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize();
		int pin2Offset = pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize();
		int actuatorOffset = actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize();
		double ds=0.0001;
		double gradP, gradM;
		Vector3d angles;
		Point3d center;
		angles[0] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
		angles[1] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
		angles[2] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
		for(uint j=0;j<3;j++){
			dVector dAdsP, dAdsM, dMdsP, dMdsM;
			double temp=angles[j];
			angles[j]=temp+ds;
			si[pin1Offset+j]=temp+ds;
			pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
			gradP = angleSensitivity(param);
			momentArmStateGradient(dMdsP,param);
			angleSensitivityStateGradient(dAdsP,si,param);
			angles[j]=temp-ds;
			si[pin1Offset+j]=temp-ds;
			pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
			gradM = angleSensitivity(param);
			momentArmStateGradient(dMdsM,param);
			angleSensitivityStateGradient(dAdsM,si,param);
			dE_ds[i*assemblyStateSize+pin1Offset+j]+=wAngle*(gradP-gradM)/(2.0*ds);
			for(int k=pin1Offset+j;k<pin1Offset+KS_MechanicalComponent::getStateSize();k++)
				ddE_dsds.addToElementAt(i*assemblyStateSize+pin1Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
			for(int k=pin1Offset;k<pin1Offset+KS_MechanicalComponent::getStateSize();k++)
				tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
			if(pin1->getInput()!=pin2->getInput())
				for(int k=pin2Offset;k<pin2Offset+KS_MechanicalComponent::getStateSize();k++){
					ddE_dsds.addToElementAt(i*assemblyStateSize+pin1Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+k,i*assemblyStateSize+pin1Offset+j,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				}
			if((pin1->getInput()!=actuator->getInput())&&(pin2->getInput()!=actuator->getInput()))
				for(int k=actuatorOffset;k<actuatorOffset+KS_MechanicalComponent::getStateSize();k++){
					ddE_dsds.addToElementAt(i*assemblyStateSize+pin1Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+k,i*assemblyStateSize+pin1Offset+j,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				}
			angles[j]=temp;
			si[pin1Offset+j]=temp;
		}
		pin1->getInput()->setAngles(angles[0],angles[1],angles[2]);
		center[0] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
		center[1] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
		center[2] = si[pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
		for(uint j=0;j<3;j++){
			dVector dAdsP, dAdsM, dMdsP, dMdsM;
			double temp=center[j];
			center[j]=temp+ds;
			si[pin1Offset+3+j]=temp+ds;
			pin1->getInput()->setWorldCenterPosition(center);
			gradP = angleSensitivity(param);
			momentArmStateGradient(dMdsP,param);
			angleSensitivityStateGradient(dAdsP,si,param);
			center[j]=temp-ds;
			si[pin1Offset+3+j]=temp-ds;
			pin1->getInput()->setWorldCenterPosition(center);
			gradM = angleSensitivity(param);
			momentArmStateGradient(dMdsM,param);
			angleSensitivityStateGradient(dAdsM,si,param);
			dE_ds[i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=wAngle*(gradP-gradM)/(2.0*ds);
			for(int k=pin1Offset+3+j;k<pin1Offset+KS_MechanicalComponent::getStateSize();k++)
				ddE_dsds.addToElementAt(i*assemblyStateSize+pin1Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
			for(int k=pin1Offset;k<pin1Offset+KS_MechanicalComponent::getStateSize();k++)
				tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
			if(pin1->getInput()!=pin2->getInput())
				for(int k=pin2Offset;k<pin2Offset+KS_MechanicalComponent::getStateSize();k++){
					ddE_dsds.addToElementAt(i*assemblyStateSize+pin1Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+k,i*assemblyStateSize+pin1Offset+3+j,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				}
			if((pin1->getInput()!=actuator->getInput())&&(pin2->getInput()!=actuator->getInput()))
				for(int k=actuatorOffset;k<actuatorOffset+KS_MechanicalComponent::getStateSize();k++){
					ddE_dsds.addToElementAt(i*assemblyStateSize+pin1Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+k,i*assemblyStateSize+pin1Offset+3+j,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				}
			center[j]=temp;
			si[pin1Offset+3+j]=temp;
		}
		pin1->getInput()->setWorldCenterPosition(center);

		if(pin2->getInput()!=pin1->getInput()){
			angles[0] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
			angles[1] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
			angles[2] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
			for(uint j=0;j<3;j++){
				dVector dAdsP, dAdsM, dMdsP, dMdsM;
				double temp=angles[j];
				angles[j]=temp+ds;
				si[pin2Offset+j]=temp+ds;
				pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
				gradP = angleSensitivity(param);
				momentArmStateGradient(dMdsP,param);
				angleSensitivityStateGradient(dAdsP,si,param);
				angles[j]=temp-ds;
				si[pin2Offset+j]=temp-ds;
				pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
				gradM = angleSensitivity(param);
				momentArmStateGradient(dMdsM,param);
				angleSensitivityStateGradient(dAdsM,si,param);
				dE_ds[i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]+=wAngle*(gradP-gradM)/(2.0*ds);
				for(int k=pin2Offset+j;k<pin2Offset+KS_MechanicalComponent::getStateSize();k++)
					ddE_dsds.addToElementAt(i*assemblyStateSize+pin2Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
				for(int k=pin2Offset;k<pin2Offset+KS_MechanicalComponent::getStateSize();k++)
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin2Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				if((pin2->getInput()!=actuator->getInput())&&(actuator->getInput()!=pin1->getInput()))
					for(int k=actuatorOffset;k<actuatorOffset+KS_MechanicalComponent::getStateSize();k++){
						ddE_dsds.addToElementAt(i*assemblyStateSize+pin2Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
						tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin2Offset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
						tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+k,i*assemblyStateSize+pin2Offset+j,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
					}
				angles[j]=temp;
				si[pin2Offset+j]=temp;
			}
			pin2->getInput()->setAngles(angles[0],angles[1],angles[2]);
			center[0] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
			center[1] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
			center[2] = si[pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
			for(uint j=0;j<3;j++){
				dVector dAdsP, dAdsM, dMdsP, dMdsM;
				double temp=center[j];
				center[j]=temp+ds;
				si[pin2Offset+3+j]=temp+ds;
				pin2->getInput()->setWorldCenterPosition(center);
				gradP = angleSensitivity(param);
				momentArmStateGradient(dMdsP,param);
				angleSensitivityStateGradient(dAdsP,si,param);
				center[j]=temp-ds;
				si[pin2Offset+3+j]=temp-ds;
				pin2->getInput()->setWorldCenterPosition(center);
				gradM = angleSensitivity(param);
				momentArmStateGradient(dMdsM,param);
				angleSensitivityStateGradient(dAdsM,si,param);
				dE_ds[i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=wAngle*(gradP-gradM)/(2.0*ds);
				for(int k=pin2Offset+3+j;k<pin2Offset+KS_MechanicalComponent::getStateSize();k++)
					ddE_dsds.addToElementAt(i*assemblyStateSize+pin2Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
				for(int k=pin2Offset;k<pin2Offset+KS_MechanicalComponent::getStateSize();k++)
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin2Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				if((pin2->getInput()!=actuator->getInput())&&(actuator->getInput()!=pin1->getInput()))
					for(int k=actuatorOffset;k<actuatorOffset+KS_MechanicalComponent::getStateSize();k++){
						ddE_dsds.addToElementAt(i*assemblyStateSize+pin2Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
						tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin2Offset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
						tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+k,i*assemblyStateSize+pin2Offset+3+j,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
					}
				center[j]=temp;
				si[pin2Offset+3+j]=temp;
			}
			pin2->getInput()->setWorldCenterPosition(center);
		}

		if((actuator->getInput()!=pin1->getInput())&&(actuator->getInput()!=pin2->getInput())){
			angles[0] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()];
			angles[1] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+1];
			angles[2] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+2];
			for(uint j=0;j<3;j++){
				dVector dAdsP, dAdsM, dMdsP, dMdsM;
				double temp=angles[j];
				angles[j]=temp+ds;
				si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]=temp+ds;
				actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
				gradP = angleSensitivity(param);
				momentArmStateGradient(dMdsP,param);
				angleSensitivityStateGradient(dAdsP,si,param);
				angles[j]=temp-ds;
				si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]=temp-ds;
				actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
				gradM = angleSensitivity(param);
				momentArmStateGradient(dMdsM,param);
				angleSensitivityStateGradient(dAdsM,si,param);
				dE_ds[i*assemblyStateSize+actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j]+=wAngle*(gradP-gradM)/(2.0*ds);
				for(int k=actuatorOffset+j;k<actuatorOffset+KS_MechanicalComponent::getStateSize();k++)
					ddE_dsds.addToElementAt(i*assemblyStateSize+actuatorOffset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
				for(int k=actuatorOffset;k<actuatorOffset+KS_MechanicalComponent::getStateSize();k++)
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+actuatorOffset+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				angles[j]=temp;
				si[actuatorOffset+j]=temp;
			}
			actuator->getInput()->setAngles(angles[0],angles[1],angles[2]);
			center[0] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3];
			center[1] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+4];
			center[2] = si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+5];
			for(uint j=0;j<3;j++){
				dVector dAdsP, dAdsM, dMdsP, dMdsM;
				double temp=center[j];
				center[j]=temp+ds;
				si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]=temp+ds;
				actuator->getInput()->setWorldCenterPosition(center);
				gradP = angleSensitivity(param);
				momentArmStateGradient(dMdsP,param);
				angleSensitivityStateGradient(dAdsP,si,param);
				center[j]=temp-ds;
				si[actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]=temp-ds;
				actuator->getInput()->setWorldCenterPosition(center);
				gradM = angleSensitivity(param);
				momentArmStateGradient(dMdsM,param);
				angleSensitivityStateGradient(dAdsM,si,param);
				dE_ds[i*assemblyStateSize+actuator->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+3+j]+=wAngle*(gradP-gradM)/(2.0*ds);
				for(int k=actuatorOffset+3+j;k<actuatorOffset+KS_MechanicalComponent::getStateSize();k++)
					ddE_dsds.addToElementAt(i*assemblyStateSize+actuatorOffset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
				for(int k=actuatorOffset;k<actuatorOffset+KS_MechanicalComponent::getStateSize();k++)
					tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+actuatorOffset+3+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				center[j]=temp;
				si[actuatorOffset+3+j]=temp;
			}
			actuator->getInput()->setWorldCenterPosition(center);
		}

		// ddM_dsdp, ddA_dsdp and ddA_dpdp
		for(int j=0;j<4;j++){
			dVector dMdsP, dMdsM, dAdsM, dAdsP, dAdpP, dAdpM;
			dAdpP.resize(4,0);
			dAdpM.resize(4,0);
			double temp=param[j];
			param[j]=temp+ds;
			momentArmStateGradient(dMdsP,param);
			angleSensitivityStateGradient(dAdsP,si,param);
			for(int k=0;k<4;k++){
				double temp2=param[k];
				param[k]=temp2+ds;
				dAdpP[k]+=angleSensitivity(param);
				param[k]=temp2-ds;
				dAdpP[k]-=angleSensitivity(param);
				dAdpP[k]/=2*ds;
				param[k]=temp2;
			}
			param[j]=temp-ds;
			momentArmStateGradient(dMdsM,param);
			angleSensitivityStateGradient(dAdsM,si,param);
			for(int k=0;k<4;k++){
				double temp2=param[k];
				param[k]=temp2+ds;
				dAdpM[k]+=angleSensitivity(param);
				param[k]=temp2-ds;
				dAdpM[k]-=angleSensitivity(param);
				dAdpM[k]/=2*ds;
				param[k]=temp2;
			}
			for(int k=0;k<assemblyStateSize;k++){
				ddE_dsds.addToElementAt(i*assemblyStateSize+k,stateSize+j,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds));
				tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+k,stateSize+j,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
				tripletList.push_back(Eigen::Triplet<double>(stateSize+j,i*assemblyStateSize+k,(wAngle*(dAdsP[k]-dAdsM[k])+wArm*(dMdsP[k]-dMdsM[k]))/(2*ds)));
			}
			for(int k=j;k<4;k++)
				ddE_dsds.addToElementAt(stateSize+j,stateSize+k,wAngle*(dAdpP[k]-dAdpM[k])/(2*ds));
			for(int k=0;k<4;k++)
				tripletList.push_back(Eigen::Triplet<double>(stateSize+j,stateSize+k,wAngle*(dAdpP[k]-dAdpM[k])/(2*ds)));
			param[j]=temp;
		}

		//parameters related terms
		//dM_dp
		dVector dE_dp (stateSize,0);
		dE_dp[0]=r1.column(0).dotProductWith((wac-wp1)*2+(wp1-wac+wp1-wp2)*2*dot/linkLength2+(wp2-wp1)*2*dot*dot/(linkLength2*linkLength2))/denom;
		dE_dp[1]=r1.column(1).dotProductWith((wac-wp1)*2+(wp1-wac+wp1-wp2)*2*dot/linkLength2+(wp2-wp1)*2*dot*dot/(linkLength2*linkLength2))/denom;
		dE_dp[2]=r2.column(0).dotProductWith((wac-wp1)*2*dot/linkLength2-(wp2-wp1)*2*dot*dot/(linkLength2*linkLength2))/denom;
		dE_dp[3]=r2.column(1).dotProductWith((wac-wp1)*2*dot/linkLength2-(wp2-wp1)*2*dot*dot/(linkLength2*linkLength2))/denom;
		dE_ds[stateSize]+=wArm*dE_dp[0];
		dE_ds[stateSize+1]+=wArm*dE_dp[1];
		dE_ds[stateSize+2]+=wArm*dE_dp[2];
		dE_ds[stateSize+3]+=wArm*dE_dp[3];

		//dA_dp
		dVector paramP;
		paramP.insert(paramP.begin(),p.begin()+stateSize, p.begin()+stateSize+5);
		dVector paramM;
		paramM.insert(paramM.begin(),p.begin()+stateSize, p.begin()+stateSize+5);
		for(uint j=0;j<5;j++){
			paramP[j]+=ds;
			paramM[j]-=ds;
			dE_ds[stateSize+j]+=wAngle*(angleSensitivity(paramP)-angleSensitivity(paramM))/(2.0*ds);
			paramP[j]-=ds;
			paramM[j]+=ds;
		}

		//dC_cp and ddC_dsdp
		pin1Offset = stateSize+paramSize+i*assemblyConstraintSize+pin1->getPinConstraintIndex();
		pin2Offset = stateSize+paramSize+i*assemblyConstraintSize+pin2->getPinConstraintIndex();
		dE_ds[stateSize]+=r1(0,0)*p[pin1Offset]+r1(1,0)*p[pin1Offset+1]+r1(2,0)*p[pin1Offset+2];
		dE_ds[stateSize+1]+=r1(0,1)*p[pin1Offset]+r1(1,1)*p[pin1Offset+1]+r1(2,1)*p[pin1Offset+2];
		for(int j=0;j<2;j++){
			for(int k=0;k<3;k++){
				ddE_dsds.addToElementAt(stateSize+j,pin1Offset+k,r1(k,j));
				tripletList.push_back(Eigen::Triplet<double>(stateSize+j,pin1Offset+k,r1(k,j)));
				tripletList.push_back(Eigen::Triplet<double>(pin1Offset+k,stateSize+j,r1(k,j)));
			}
		}
		dE_ds[stateSize+2]+=r2(0,0)*p[pin2Offset]+r2(1,0)*p[pin2Offset+1]+r2(2,0)*p[pin2Offset+2];
		dE_ds[stateSize+3]+=r2(0,1)*p[pin2Offset]+r2(1,1)*p[pin2Offset+1]+r2(2,1)*p[pin2Offset+2];
		for(int j=0;j<2;j++){
			for(int k=0;k<3;k++){
				ddE_dsds.addToElementAt(stateSize+2+j,pin2Offset+k,r2(k,j));
				tripletList.push_back(Eigen::Triplet<double>(stateSize+2+j,pin2Offset+k,r2(k,j)));
				tripletList.push_back(Eigen::Triplet<double>(pin2Offset+k,stateSize+2+j,r2(k,j)));
			}
		}
		dE_ds[stateSize+4]+=0.5*(r3(0,0)*(p[pin1Offset]-p[pin2Offset])+r3(1,0)*(p[pin1Offset+1]-p[pin2Offset+1])+r3(2,0)*(p[pin1Offset+2]-p[pin2Offset+2]));
		for(int k=0;k<3;k++){
			ddE_dsds.addToElementAt(stateSize+4,pin1Offset+k,0.5*r3(k,0));
			tripletList.push_back(Eigen::Triplet<double>(stateSize+4,pin1Offset+k,0.5*r3(k,0)));
			tripletList.push_back(Eigen::Triplet<double>(pin1Offset+k,stateSize+4,0.5*r3(k,0)));
			ddE_dsds.addToElementAt(stateSize+4,pin2Offset+k,-0.5*r3(k,0));
			tripletList.push_back(Eigen::Triplet<double>(stateSize+4,pin2Offset+k,-0.5*r3(k,0)));
			tripletList.push_back(Eigen::Triplet<double>(pin2Offset+k,stateSize+4,-0.5*r3(k,0)));
		}
		pin1->getInput()->get_dw_ds(Point3d(1,0,0),dw_ds);
		for(int j=0;j<3;j++){
			ddE_dsds.addToElementAt(i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize,p[pin1Offset]*dw_ds(0,j)+p[pin1Offset+1]*dw_ds(1,j)+p[pin1Offset+2]*dw_ds(2,j));
			tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize,p[pin1Offset]*dw_ds(0,j)+p[pin1Offset+1]*dw_ds(1,j)+p[pin1Offset+2]*dw_ds(2,j)));
			tripletList.push_back(Eigen::Triplet<double>(stateSize,i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,p[pin1Offset]*dw_ds(0,j)+p[pin1Offset+1]*dw_ds(1,j)+p[pin1Offset+2]*dw_ds(2,j)));
		}
		pin1->getInput()->get_dw_ds(Point3d(0,1,0),dw_ds);
		for(int j=0;j<3;j++){
			ddE_dsds.addToElementAt(i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize+1,p[pin1Offset]*dw_ds(0,j)+p[pin1Offset+1]*dw_ds(1,j)+p[pin1Offset+2]*dw_ds(2,j));
			tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize+1,p[pin1Offset]*dw_ds(0,j)+p[pin1Offset+1]*dw_ds(1,j)+p[pin1Offset+2]*dw_ds(2,j)));
			tripletList.push_back(Eigen::Triplet<double>(stateSize+1,i*assemblyStateSize+pin1->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,p[pin1Offset]*dw_ds(0,j)+p[pin1Offset+1]*dw_ds(1,j)+p[pin1Offset+2]*dw_ds(2,j)));
		}
		pin2->getInput()->get_dw_ds(Point3d(1,0,0),dw_ds);
		for(int j=0;j<3;j++){
			ddE_dsds.addToElementAt(i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize+2,p[pin2Offset]*dw_ds(0,j)+p[pin2Offset+1]*dw_ds(1,j)+p[pin2Offset+2]*dw_ds(2,j));
			tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize+2,p[pin2Offset]*dw_ds(0,j)+p[pin2Offset+1]*dw_ds(1,j)+p[pin2Offset+2]*dw_ds(2,j)));
			tripletList.push_back(Eigen::Triplet<double>(stateSize+2,i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,p[pin2Offset]*dw_ds(0,j)+p[pin2Offset+1]*dw_ds(1,j)+p[pin2Offset+2]*dw_ds(2,j)));
		}
		pin2->getInput()->get_dw_ds(Point3d(0,1,0),dw_ds);
		for(int j=0;j<3;j++){
			ddE_dsds.addToElementAt(i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize+3,p[pin2Offset]*dw_ds(0,j)+p[pin2Offset+1]*dw_ds(1,j)+p[pin2Offset+2]*dw_ds(2,j));
			tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize+3,p[pin2Offset]*dw_ds(0,j)+p[pin2Offset+1]*dw_ds(1,j)+p[pin2Offset+2]*dw_ds(2,j)));
			tripletList.push_back(Eigen::Triplet<double>(stateSize+3,i*assemblyStateSize+pin2->getInput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,p[pin2Offset]*dw_ds(0,j)+p[pin2Offset+1]*dw_ds(1,j)+p[pin2Offset+2]*dw_ds(2,j)));
		}
		pin1->getOutput()->get_dw_ds(Point3d(1,0,0),dw_ds);
		for(int j=0;j<3;j++){
			ddE_dsds.addToElementAt(i*assemblyStateSize+pin1->getOutput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize+4,0.5*((p[pin1Offset]-p[pin2Offset])*dw_ds(0,j)+(p[pin1Offset+1]-p[pin2Offset+1])*dw_ds(1,j)+(p[pin1Offset+2]-p[pin2Offset+2])*dw_ds(2,j)));
			tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+pin1->getOutput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,stateSize+4,0.5*((p[pin1Offset]-p[pin2Offset])*dw_ds(0,j)+(p[pin1Offset+1]-p[pin2Offset+1])*dw_ds(1,j)+(p[pin1Offset+2]-p[pin2Offset+2])*dw_ds(2,j))));
			tripletList.push_back(Eigen::Triplet<double>(stateSize+4,i*assemblyStateSize+pin1->getOutput()->getComponentIndex()*KS_MechanicalComponent::getStateSize()+j,0.5*((p[pin1Offset]-p[pin2Offset])*dw_ds(0,j)+(p[pin1Offset+1]-p[pin2Offset+1])*dw_ds(1,j)+(p[pin1Offset+2]-p[pin2Offset+2])*dw_ds(2,j))));
		}
		//ddM_dpdp
		Matrix3x3 ddE_dpdp, temp;
		ddE_dpdp.setToATransposedB(r1,r1);
		ddE_dpdp *= -2*(1-dot/linkLength2)*(1-dot/linkLength2);
		temp.setToOuterProduct(r1.transpose()*(wp1-wac+wp1-wp2+(wp2-wp1)*2*dot/linkLength2),r1.transpose()*(wp1-wac+wp1-wp2+(wp2-wp1)*2*dot/linkLength2));
		temp *= 2.0/linkLength2;
		ddE_dpdp += temp;
		ddE_dsds.addToElementAt(stateSize,stateSize,wArm*ddE_dpdp(0,0)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize,stateSize,wArm*ddE_dpdp(0,0)/denom));
		ddE_dsds.addToElementAt(stateSize,stateSize+1,wArm*ddE_dpdp(0,1)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize,stateSize+1,wArm*ddE_dpdp(0,1)/denom));
		tripletList.push_back(Eigen::Triplet<double>(stateSize+1,stateSize,wArm*ddE_dpdp(0,1)/denom));
		ddE_dsds.addToElementAt(stateSize+1,stateSize+1,wArm*ddE_dpdp(1,1)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize+1,stateSize+1,wArm*ddE_dpdp(1,1)/denom));
		ddE_dpdp.setToZeros(); temp.setToZeros();
		ddE_dpdp.setToOuterProduct(r1.transpose()*(wp1-wac+wp1-wp2+(wp2-wp1)*2*dot/linkLength2),r2.transpose()*(wac-wp1+(wp1-wp2)*2*dot/linkLength2));
		ddE_dpdp *= 2/linkLength2;
		temp.setToATransposedB(r1,r2);
		temp *= 2.0*(dot/linkLength2-1)*dot/linkLength2;
		ddE_dpdp += temp;
		ddE_dsds.addToElementAt(stateSize,stateSize+2,wArm*ddE_dpdp(0,0)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize,stateSize+2,wArm*ddE_dpdp(0,0)/denom));
		tripletList.push_back(Eigen::Triplet<double>(stateSize+2,stateSize,wArm*ddE_dpdp(0,0)/denom));
		ddE_dsds.addToElementAt(stateSize,stateSize+3,wArm*ddE_dpdp(0,1)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize,stateSize+3,wArm*ddE_dpdp(0,1)/denom));
		tripletList.push_back(Eigen::Triplet<double>(stateSize+3,stateSize,wArm*ddE_dpdp(0,1)/denom));
		ddE_dsds.addToElementAt(stateSize+1,stateSize+2,wArm*ddE_dpdp(1,0)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize+1,stateSize+2,wArm*ddE_dpdp(1,0)/denom));
		tripletList.push_back(Eigen::Triplet<double>(stateSize+2,stateSize+1,wArm*ddE_dpdp(1,0)/denom));
		ddE_dsds.addToElementAt(stateSize+1,stateSize+3,wArm*ddE_dpdp(1,1)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize+1,stateSize+3,wArm*ddE_dpdp(1,1)/denom));
		tripletList.push_back(Eigen::Triplet<double>(stateSize+3,stateSize+1,wArm*ddE_dpdp(1,1)/denom));
		ddE_dpdp.setToZeros(); temp.setToZeros();
		ddE_dpdp.setToOuterProduct(r2.transpose()*(wac-wp1-(wp2-wp1)*2*dot/linkLength2),r2.transpose()*(wac-wp1-(wp2-wp1)*2*dot/linkLength2));
		ddE_dpdp *= 2.0/linkLength2;
		temp.setToATransposedB(r2,r2);
		temp *= -2.0*dot*dot/(linkLength2*linkLength2);
		ddE_dpdp += temp;
		ddE_dsds.addToElementAt(stateSize+2,stateSize+2,wArm*ddE_dpdp(0,0)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize+2,stateSize+2,wArm*ddE_dpdp(0,0)/denom));
		ddE_dsds.addToElementAt(stateSize+2,stateSize+3,wArm*ddE_dpdp(0,1)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize+2,stateSize+3,wArm*ddE_dpdp(0,1)/denom));
		tripletList.push_back(Eigen::Triplet<double>(stateSize+3,stateSize+2,wArm*ddE_dpdp(0,1)/denom));
		ddE_dsds.addToElementAt(stateSize+3,stateSize+3,wArm*ddE_dpdp(1,1)/denom);
		tripletList.push_back(Eigen::Triplet<double>(stateSize+3,stateSize+3,wArm*ddE_dpdp(1,1)/denom));
		for(int j=0;j<paramSize-1;j++)
			for(int k=j;k<paramSize-1;k++)
				ddE_dsds.addToElementAt(stateSize+j,stateSize+k,wArm*dE_dp[j]*dE_dp[k]*2*(epsilon+(wp1-wac).length2()-dot*dot/(wp1-wp2).length2()));
		for(int j=0;j<paramSize-1;j++)
			for(int k=0;k<paramSize-1;k++)
				tripletList.push_back(Eigen::Triplet<double>(stateSize+j,stateSize+k,wArm*dE_dp[j]*dE_dp[k]*2*(epsilon+(wp1-wac).length2()-dot*dot/(wp1-wp2).length2())));

		//constraint related terms
		dVector C (assemblyConstraintSize,0);
		//SparseMatrix dCds (assemblyConstraintSize, assemblyStateSize, false);
		for(uint j=0;j<constraints.size();j++){
			constraints[j]->computeEnergyGradientAndHessian(si);
			constraints[j]->setConstraintValues(C);
			constraints[j]->setConstraintJacobianValues(dCds);
		}
		for(int j=0;j<assemblyConstraintSize;j++)
			dE_ds[stateSize+paramSize+i*assemblyConstraintSize+j]+=C[j];
		for(int j=0;j<assemblyConstraintSize;j++){
			for(int k=0;k<assemblyStateSize;k++){
				dE_ds[i*assemblyStateSize+k]+=dCds.getElementAt(j,k)*p[stateSize+paramSize+i*assemblyConstraintSize+j];
				ddE_dsds.addToElementAt(i*assemblyStateSize+k,stateSize+paramSize+i*assemblyConstraintSize+j,dCds.getElementAt(j,k));
				tripletList.push_back(Eigen::Triplet<double>(i*assemblyStateSize+k,stateSize+paramSize+i*assemblyConstraintSize+j,dCds.getElementAt(j,k)));
				tripletList.push_back(Eigen::Triplet<double>(stateSize+paramSize+i*assemblyConstraintSize+j,i*assemblyStateSize+k,dCds.getElementAt(j,k)));
			}
		}
		int currentConstraintCount = 0;
		for(uint j=0;j<constraints.size();j++){
			int n = constraints[j]->getNumberOfAffectedComponents();
			DynamicArray<Matrix*> dC_dsidsj;
			if(n==1){
				pin1Offset=i*assemblyStateSize+constraints[j]->getIthAffectedComponent(0)->getComponentIndex()*KS_MechanicalComponent::getStateSize();
				constraints[j]->getConstraintHessian(0,0,dC_dsidsj);
				for(uint k=0;k<dC_dsidsj.size();k++){
					SparseMatrixBlockHandle block;
					block.createBlock(&ddE_dsds,pin1Offset,pin1Offset,KS_MechanicalComponent::getStateSize(),KS_MechanicalComponent::getStateSize());
					block.addBlockValues(&ddE_dsds,dC_dsidsj[k]->getData(),p[stateSize+paramSize+i*assemblyConstraintSize+currentConstraintCount+k]);
					for(int l=0;l<KS_MechanicalComponent::getStateSize();l++){
						for(int m=0;m<KS_MechanicalComponent::getStateSize();m++){
							tripletList.push_back(Eigen::Triplet<double>(pin1Offset+l,pin1Offset+m,(*dC_dsidsj[k])(l,m)*p[stateSize+paramSize+i*assemblyConstraintSize+currentConstraintCount+k]));
						}
					}
				}			
			}else if(n==2){
				int c1,c2;
				if(constraints[j]->getIthAffectedComponent(0)->getComponentIndex()<constraints[j]->getIthAffectedComponent(1)->getComponentIndex()){
					c1=1;
					c2=0;
				}else{
					c1=0;
					c2=1;
				}
				pin1Offset=i*assemblyStateSize+constraints[j]->getIthAffectedComponent(c1)->getComponentIndex()*KS_MechanicalComponent::getStateSize();
				pin2Offset=i*assemblyStateSize+constraints[j]->getIthAffectedComponent(c2)->getComponentIndex()*KS_MechanicalComponent::getStateSize();
				constraints[j]->getConstraintHessian(c1,c1,dC_dsidsj);
				for(uint k=0;k<dC_dsidsj.size();k++){
					SparseMatrixBlockHandle block;
					block.createBlock(&ddE_dsds,pin1Offset,pin1Offset,KS_MechanicalComponent::getStateSize(),KS_MechanicalComponent::getStateSize());
					block.addBlockValues(&ddE_dsds,dC_dsidsj[k]->getData(),p[stateSize+paramSize+i*assemblyConstraintSize+currentConstraintCount+k]);
					for(int l=0;l<KS_MechanicalComponent::getStateSize();l++){
						for(int m=0;m<KS_MechanicalComponent::getStateSize();m++){
							tripletList.push_back(Eigen::Triplet<double>(pin1Offset+l,pin1Offset+m,(*dC_dsidsj[k])(l,m)*p[stateSize+paramSize+i*assemblyConstraintSize+currentConstraintCount+k]));
						}
					}
				}
				dC_dsidsj.clear();
				constraints[j]->getConstraintHessian(c2,c2,dC_dsidsj);
				for(uint k=0;k<dC_dsidsj.size();k++){
					SparseMatrixBlockHandle block;
					block.createBlock(&ddE_dsds,pin2Offset,pin2Offset,KS_MechanicalComponent::getStateSize(),KS_MechanicalComponent::getStateSize());
					block.addBlockValues(&ddE_dsds,dC_dsidsj[k]->getData(),p[stateSize+paramSize+i*assemblyConstraintSize+currentConstraintCount+k]);
					for(int l=0;l<KS_MechanicalComponent::getStateSize();l++){
						for(int m=0;m<KS_MechanicalComponent::getStateSize();m++){
							tripletList.push_back(Eigen::Triplet<double>(pin2Offset+l,pin2Offset+m,(*dC_dsidsj[k])(l,m)*p[stateSize+paramSize+i*assemblyConstraintSize+currentConstraintCount+k]));
						}
					}
				}
				dC_dsidsj.clear();
				constraints[j]->getConstraintHessian(c1,c2,dC_dsidsj);
				for(uint k=0;k<dC_dsidsj.size();k++){
					SparseMatrixBlockHandle block;
					block.createBlock(&ddE_dsds,pin1Offset,pin2Offset,KS_MechanicalComponent::getStateSize(),KS_MechanicalComponent::getStateSize());
					block.addBlockValues(&ddE_dsds,dC_dsidsj[k]->getData(),p[stateSize+paramSize+i*assemblyConstraintSize+currentConstraintCount+k]);
					for(int l=0;l<KS_MechanicalComponent::getStateSize();l++){
						for(int m=0;m<KS_MechanicalComponent::getStateSize();m++){
							tripletList.push_back(Eigen::Triplet<double>(pin1Offset+l,pin2Offset+m,(*dC_dsidsj[k])(l,m)*p[stateSize+paramSize+i*assemblyConstraintSize+currentConstraintCount+k]));
						}
					}
				}
			}
			currentConstraintCount+=constraints[j]->getConstraintCount();
		}
	}
	if(regularizer!=0){
		for(int i=stateSize+paramSize;i<stateSize+paramSize+constraintSize;i++)
			tripletList.push_back(Eigen::Triplet<double>(i,i,regularizer));
		for(int i=stateSize+paramSize;i<stateSize+paramSize+constraintSize;i++)
			dE_ds[i]+=2*regularizer*p[i];
	}
	Hessian.setFromTriplets(tripletList.begin(), tripletList.end());
	/*
	iter++;
	if(iter==2){
		double ds = 0.0001;
		dVector pi;
		copy(p,pi);
		for(uint i=0;i<p.size();i++){
			double valP, valM;
			double temp = pi[i];
			pi[i]=temp+ds;
			valP = getValue(pi);
			pi[i]=temp-ds;
			valM = getValue(pi);
			double a = dE_ds[i];
			double b = (valP-valM)/(2*ds);
			double error;
			if((b>1.0)||(b<-1.0))
				error = fabs(a/b-1);
			else
				error = fabs(a-b);
			if(error>10e-6)
				error=0;
			pi[i]=temp;
		}
		for(uint i=0;i<p.size();i++){
			double temp = pi[i];
			dVector gradP, gradM;
			pi[i]=temp+ds;
			computeGradient(gradP,pi);
			pi[i]=temp-ds;
			computeGradient(gradM,pi);
			for(uint j=i;j<p.size();j++){
				double a = Hessian.coeffRef(i,j);
				double b = (gradP[j]-gradM[j])/(2*ds);
				double error;
				if((b>1.0)||(b<-1.0))
					error = fabs(a/b-1);
				else
					error = fabs(a-b);
				if(error>10e-6){
					error=0;
				}
			}
			pi[i]=temp;
		}
	}
	*/
}

void KS_LinkageLagrangian::setRegularizer(double val){
	if(val!=regularizer){
		for(int i=0;i<stateSize+paramSize;i++)
			Hessian.coeffRef(i,i)+=(val-regularizer)*ticks.size();
		regularizer=val;
	}
}

SparseMatrix* KS_LinkageLagrangian::getCurrentHessian(){
	return &ddE_dsds;
}

Eigen::SparseMatrix<double>* KS_LinkageLagrangian::getCurrentHessianEigen(){
	return &Hessian;
}

dVector* KS_LinkageLagrangian::getCurrentGradient(){
	return &dE_ds;
}

void KS_LinkageLagrangian::setState(int s, const dVector &p){
	if((s>-1)&&(s<(int)ticks.size())){
		dVector si;
		si.insert(si.begin(),p.begin()+s*assemblyStateSize,p.begin()+(s+1)*assemblyStateSize);
		assembly->setAssemblyState(si);
	}
}