#include "KineSimLib/KS_PlanarComponentConnection.h"

KS_PlanarComponentConnection::KS_PlanarComponentConnection(void){
	lcCon = NULL;
}

KS_PlanarComponentConnection::~KS_PlanarComponentConnection(void){
	delete lcCon;
}

void KS_PlanarComponentConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;
	m_compOut->addCylinderMesh(20, 0.015, 0.08, Point3d(), Vector3d(0,0,1));
	m_pin=m_compOut->getNumOfMeshes()-1;

	assert(m_compIn == NULL);

	lcCon = new KS_LockedComponentConstraint(m_compOut, m_compOut->getAlpha(), false, m_compOut->getBeta(), true, m_compOut->getGamma(), true, m_compOut->getWorldCenterPosition().x, false, m_compOut->getWorldCenterPosition().y, false, m_compOut->getWorldCenterPosition().z, true); 
}

bool KS_PlanarComponentConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		logPrint("KS_PlanarComponentConnection: Cannot load input file.\n");
		return false;
	}
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, sizeof(buffer));
		char *line = lTrim(buffer);

		if (KS_Connection::processInputLine(line, ma))
			continue;

		int lineType = getKSLineType(line);
		switch (lineType) {
			case KS_COMMENT:
				break;
			case KS_END:
				if (m_compIn != NULL){
					logPrint("KS_PlanarComponentConnection: Warning! Component IN should be NULL! Ignoring...");
					m_compIn = NULL;
				}
				connect(this->m_compIn, this->m_compOut);
				return true;
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_PlanarComponentConnection: Warning - end of file met before END primitive\n");
	return false;
}

bool KS_PlanarComponentConnection::writeToFile(FILE* f){

	char* str;

	str = getKSString(KS_PLANAR_COMP_CON);
	fprintf(f, "%s\n", str);

	writeBaseConnectionToFile(f);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_PlanarComponentConnection* KS_PlanarComponentConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const {
	KS_PlanarComponentConnection* pin = new KS_PlanarComponentConnection(*this);
	pin->m_compIn = pCompIn;
	pin->m_compOut = pCompOut;
	pin->lcCon = lcCon->clone(pCompIn,pCompOut);
	return pin;	
}

