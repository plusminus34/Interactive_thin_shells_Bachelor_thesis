#include "KineSimLib/KS_rMotorConnection.h"
#include "KineSimLib/KS_MechanicalComponent.h"
#include "KineSimLib/KS_LoaderUtils.h"

KS_rMotorConnection::KS_rMotorConnection(void){
	rMotorAngleConstraint = NULL;
	pt2ptConstraint = NULL;
	v2vConstraint = NULL;
	m_offset=0.0;
	isActivated=true;
	
	nOnC1 = V3D(0,0,1);
	nOnC2 = V3D(0,0,1);
	vOnC1 = V3D(1, 0, 0);
	vOnC2 = V3D(1, 0, 0);
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
	m_compOut->addCylinderMesh(20, 0.06, 0.08, pOnC2, nOnC2);
	m_pin=m_compOut->getNumOfMeshes()-1;

	DynamicArray<P3D> tmpPointsList = m_compIn->getPoints_list();
	tmpPointsList.push_back(pOnC1);
	tmpPointsList.push_back(pOnC1 + P3D(0, .1, 0));
	tmpPointsList.push_back(pOnC1 + P3D(0, 0, .1));
	tmpPointsList.push_back(pOnC1 / 2 + P3D(0, .1, 0));
	tmpPointsList.push_back(pOnC1 / 2 + P3D(0, 0, .1));


	m_compIn->setPoints_list(tmpPointsList);
	tmpPointsList = m_compOut->getPoints_list();
	tmpPointsList.push_back(pOnC2);
	tmpPointsList.push_back(pOnC2 + P3D(0, .1, 0));
	tmpPointsList.push_back(pOnC2 + P3D(0, 0, .1));
	tmpPointsList.push_back(pOnC2 / 2 + P3D(0, .1, 0));
	tmpPointsList.push_back(pOnC2 / 2 + P3D(0, 0, .1));

	m_compOut->setPoints_list(tmpPointsList);
	
	if(isActivated)
		rMotorAngleConstraint = new KS_V2VConstraint(vOnC1, m_compIn, vOnC2, m_compOut);
	pt2ptConstraint = new KS_P2PConstraint(pOnC1, m_compIn, pOnC2, m_compOut); 
	v2vConstraint = new KS_V2VConstraint(nOnC1, m_compIn, nOnC2, m_compOut);
}

void KS_rMotorConnection::addConstraintsToList(std::vector<KS_Constraint*>& constraints){
	if(isActivated)
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
			case KS_IS_ACTIVATED:
				if (sscanf(line, "%d", &isActivated) != 1) assert(false);
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

	str = getKSString(KS_IS_ACTIVATED);
	fprintf(f, "\t%s %d \n", str, isActivated);

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