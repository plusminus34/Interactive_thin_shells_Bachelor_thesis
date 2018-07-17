#include "KineSimLib/KS_rMotorConnection.h"
#include "KineSimLib/KS_MechanicalComponent.h"
#include "KineSimLib/KS_LoaderUtils.h"

KS_rMotorConnection::KS_rMotorConnection(void){
	rMotorAngleConstraint = NULL;
	pt2ptConstraint = NULL;
	v2vConstraint = NULL;
	m_offset=0.0;
	isActuated=true;
	
	nOnC1 = V3D(0,0,1);
	nOnC2 = V3D(0,0,1);
	vOnC1 = V3D(1, 0, 0);
	vOnC2 = V3D(1, 0, 0);

	ddAE_ds_dp1.resize(1, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp1.setZero();
	ddAE_ds_dp2.resize(1, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp2.setZero();
	ddAE_ds_dp1Temp.resize(3, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp1Temp.setZero();
	ddAE_ds_dp2Temp.resize(3, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp2Temp.setZero();
	dw2_ds2_p.resize(3, KS_MechanicalComponent::getStateSize()); dw2_ds2_p.setZero();
	ddAE_ds_dpTemp2.setZero();
}

KS_rMotorConnection::~KS_rMotorConnection(void){
	delete rMotorAngleConstraint;
	delete pt2ptConstraint;
	delete v2vConstraint;
}

void KS_rMotorConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;
	m_compOut->addCylinderMesh(20, 0.12, 0.12, pOnC2, nOnC2);
	m_pin=m_compOut->getNumOfMeshes()-1;

    //lets add points to the list
	m_compIn->addCubeAroundApoint(pOnC1,0.05);
	m_compOut->addCubeAroundApoint(pOnC2, 0.05);

	
	
	rMotorAngleConstraint = new KS_V2VConstraint(vOnC1, m_compIn, vOnC2, m_compOut);
	pt2ptConstraint = new KS_P2PConstraint(pOnC1, m_compIn, pOnC2, m_compOut); 
	v2vConstraint = new KS_V2VConstraint(nOnC1, m_compIn, nOnC2, m_compOut);
}

void KS_rMotorConnection::addConstraintsToList(std::vector<KS_Constraint*>& constraints){
	
	constraints.push_back(rMotorAngleConstraint);
	constraints.push_back(pt2ptConstraint);
	constraints.push_back(v2vConstraint);
}

void KS_rMotorConnection::updateConnection()
{
	nOnC1.normalize();
	vOnC1 = V3D(1, 0, 0);
	vOnC1 = getRotationQuaternion(m_offset, nOnC1)*vOnC1;
	rMotorAngleConstraint->movePinOnC1(vOnC1);
	//Logger::print("roated vec %lf %lf %lf\n", vOnC1[0], vOnC1[1], vOnC1[2]);
}



bool KS_rMotorConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		Logger::print("KS_PhaseDriver: Cannot load input file.\n");
		return false;
	}
	//have a temporary buffer used to read the file line by line...
	char buffer[1000];
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, sizeof(buffer));
		char *line = lTrim(buffer);

		if (KS_Connection::processInputLine(line, ma))
			continue;

		int lineType = getKSLineType(line);
		switch (lineType) {
			case KS_PIN_ON_COMP_IN:
				if (sscanf(line, "%lf %lf %lf", &pOnC1[0], &pOnC1[1], &pOnC1[2]) != 3) assert(false);
				break;
			case KS_PIN_ON_COMP_OUT:
				if (sscanf(line, "%lf %lf %lf", &pOnC2[0], &pOnC2[1], &pOnC2[2]) != 3) assert(false);
				break;
			case KS_VEC_ON_COMP_IN:
				if (sscanf(line, "%lf %lf %lf", &nOnC1[0], &nOnC1[1], &nOnC1[2]) != 3) assert(false);
				nOnC1.normalize();
				break;
			case KS_VEC_ON_COMP_OUT:
				if (sscanf(line, "%lf %lf %lf", &nOnC2[0], &nOnC2[1], &nOnC2[2]) != 3) assert(false);
				nOnC2.normalize();
				break;
			case KS_VEC2_ON_COMP_IN:
				if (sscanf(line, "%lf %lf %lf", &vOnC1[0], &vOnC1[1], &vOnC1[2]) != 3) assert(false);
				vOnC1.normalize();
				break;
			case KS_VEC2_ON_COMP_OUT:
				if (sscanf(line, "%lf %lf %lf", &vOnC2[0], &vOnC2[1], &vOnC2[2]) != 3) assert(false);
				vOnC1.normalize();
				break;
			case KS_END:
				connect(this->m_compIn, this->m_compOut);
				return true;
				break;
			case KS_COMMENT:
				break;
			default:
				Logger::print("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	Logger::print("Phase Driver: Warning - end of file met before END primitive\n");
	return false;
}


bool KS_rMotorConnection::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_R_MOTOR_CON);
	fprintf(f, "%s\n", str);

	writeBaseConnectionToFile(f);

	str = getKSString(KS_PIN_ON_COMP_IN);
	fprintf(f, "\t%s %lf %lf %lf\n", str, pOnC1[0], pOnC1[1], pOnC1[2]);

	str = getKSString(KS_PIN_ON_COMP_OUT);
	fprintf(f, "\t%s %lf %lf %lf\n", str, pOnC2[0], pOnC2[1], pOnC2[2]);

	str = getKSString(KS_VEC_ON_COMP_IN);
	fprintf(f, "\t%s %lf %lf %lf\n", str, nOnC1[0], nOnC1[1], nOnC1[2]);

	str = getKSString(KS_VEC_ON_COMP_OUT);
	fprintf(f, "\t%s %lf %lf %lf\n", str, nOnC2[0], nOnC2[1], nOnC2[2]);

	str = getKSString(KS_VEC2_ON_COMP_IN);
	fprintf(f, "\t%s %lf %lf %lf\n", str, vOnC1[0], vOnC1[1], vOnC1[2]);

	str = getKSString(KS_VEC2_ON_COMP_OUT);
	fprintf(f, "\t%s %lf %lf %lf\n", str, vOnC2[0], vOnC2[1], vOnC2[2]);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_rMotorConnection* KS_rMotorConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const {
	KS_rMotorConnection* pin = new KS_rMotorConnection(*this);
	pin->m_compIn = pCompIn;
	pin->m_compOut = pCompOut;
	pin->rMotorAngleConstraint = rMotorAngleConstraint->clone(pCompIn,pCompOut);
	pin->pt2ptConstraint = pt2ptConstraint->clone(pCompIn,pCompOut);
	pin->v2vConstraint = v2vConstraint->clone(pCompIn,pCompOut);
	return pin;
}


void KS_rMotorConnection::computeddAE_ds_dp1()
{
	ddAE_ds_dp1.resize(1, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp1.setZero();
	ddAE_ds_dp2.resize(1, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp2.setZero();
	ddAE_ds_dp1Temp.resize(3, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp1Temp.setZero();
	ddAE_ds_dp2Temp.resize(3, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp2Temp.setZero();
	dw2_ds2_p.resize(3, KS_MechanicalComponent::getStateSize()); dw2_ds2_p.setZero();
	ddAE_ds_dpTemp2.setZero();
	//(c1->get_w(x1) - c2->get_w(x2))*(c1->get_dw_ds(x1, dw1_ds1));
	//dE_ds2 = -1 * tmpV.transpose()*dw2_ds2;
	//(c1->get_w(x1) - c2->get_w(x2))*(c1->get_dw_ds(x2, dw1_ds2));
	//V3D dw_dalpha = [const V3D x] { return m_compIn->R_gamma * (m_compIn->R_beta*(m_compIn->n_alpha.cross(m_compIn->R_alpha*x))); }
	//V3D dw_dbeta(const V3D& x) { return  m_compIn->R_gamma * (m_compIn->n_beta.cross(m_compIn->R_beta*(m_compIn->R_alpha*x))); }
	//V3D dw_dgamma(const V3D& x) { return m_compIn->n_gamma.cross(m_compIn->R_gamma*(m_compIn->R_beta*(m_compIn->R_alpha*x))); }
	//dE_ds1 = tmpV.transpose()*dw1_ds1;
	Matrix3x3 tmp; tmp.setIdentity();
	//auto glambda = [](auto a, auto&& b) { return a < b; };
	auto dw_dalpha = [this](const V3D x) { return m_compIn->R_gamma * (m_compIn->R_beta*(m_compIn->n_alpha.cross(m_compIn->R_alpha*x))); };
	auto dw_dbeta = [this](const V3D x) {return  m_compIn->R_gamma * (m_compIn->n_beta.cross(m_compIn->R_beta*(m_compIn->R_alpha*x)));  };
	auto dw_dgamma = [this](const V3D x) {return m_compIn->n_gamma.cross(m_compIn->R_gamma*(m_compIn->R_beta*(m_compIn->R_alpha*x)));  };

	/*//m_compIn->get_w(V3D(tmp.col(0)));
	//dw_dgamma(vOnC1);
		/*- m_compOut->get_w(vOnC2)).transpose()*dw_dgamma(vOnC1) +
		(m_compIn->get_w(vOnC1) - m_compOut->get_w(vOnC2)).transpose()*dw_dgamma(V3D(tmp.col(0))));
	MatrixNxM temp=((m_compIn->get_w(V3D(tmp.col(0))) - m_compOut->get_w(vOnC2)).transpose()*dw_dgamma(vOnC1) +
		(m_compIn->get_w(vOnC1) - m_compOut->get_w(vOnC2)).transpose()*dw_dgamma(V3D(tmp.col(0))));
	Logger::print("temp %d %d %lf\n",temp.rows(),temp.cols(),temp(0,0) );*/

	for (int i = 0; i < 3; i++) {
		ddAE_ds_dp1Temp(i, 0) += (m_compIn->get_w(V3D(tmp.col(i))) - m_compOut->get_w(vOnC2)).transpose()*dw_dgamma(vOnC1);
		ddAE_ds_dp1Temp(i, 0) += (m_compIn->get_w(vOnC1) - m_compOut->get_w(vOnC2)).transpose()*dw_dgamma(V3D(tmp.col(i)));
	}
	for (int i = 0; i < 3; i++) {
		ddAE_ds_dp1Temp(i, 1) += (m_compIn->get_w(V3D(tmp.col(i))) - m_compOut->get_w(vOnC2)).transpose()*dw_dbeta(vOnC1);
		ddAE_ds_dp1Temp(i, 1) +=	(m_compIn->get_w(vOnC1) - m_compOut->get_w(vOnC2)).transpose()*dw_dbeta(V3D(tmp.col(i)));
	}
	for (int i = 0; i < 3; i++) {
		ddAE_ds_dp1Temp(i, 2) += (m_compIn->get_w(V3D(tmp.col(i))) - m_compOut->get_w(vOnC2)).transpose()*dw_dalpha(vOnC1);
		ddAE_ds_dp1Temp(i, 2) += (m_compIn->get_w(vOnC1) - m_compOut->get_w(vOnC2)).transpose()*dw_dalpha(V3D(tmp.col(i))); 
	}
	m_compOut->get_dw_ds(vOnC2, dw2_ds2_p);
	for (int i = 0; i < 3; i++) {
		ddAE_ds_dp2Temp.row(i) = (m_compIn->get_w(V3D(tmp.col(i))) - m_compOut->get_w(vOnC2)).transpose()*dw2_ds2_p;
	}
	V3D vOnC1tmp = V3D(1, 0, 0);
	ddAE_ds_dpTemp2=nOnC1.cross(getRotationQuaternion(m_offset, nOnC1)*vOnC1tmp);
	ddAE_ds_dp1 = ddAE_ds_dpTemp2.transpose()*ddAE_ds_dp1Temp;
	ddAE_ds_dp2 = ddAE_ds_dpTemp2.transpose()*ddAE_ds_dp2Temp;

}