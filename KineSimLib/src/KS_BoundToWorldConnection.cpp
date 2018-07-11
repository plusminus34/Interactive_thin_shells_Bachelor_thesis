#include "KineSimLib/KS_BoundToWorldConnection.h"

KS_BoundToWorldConnection::KS_BoundToWorldConnection(void){
	lcCon = NULL;
}

KS_BoundToWorldConnection::~KS_BoundToWorldConnection(void){
	delete lcCon;
}

void KS_BoundToWorldConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;

	assert(m_compIn == NULL);
	lcCon = new KS_LockedComponentConstraint(m_compOut, m_compOut->getAlpha(), true, m_compOut->getBeta(), true, m_compOut->getGamma(), true, m_compOut->getWorldCenterPosition()[0], true, m_compOut->getWorldCenterPosition()[1], true, m_compOut->getWorldCenterPosition()[2], true); 
}

bool KS_BoundToWorldConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		Logger::print("KS_BoundToWorldConnection: Cannot load input file.\n");
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
					Logger::print("KS_BoundToWorldConnection: Warning! Component IN should be NULL! Ignoring...");
					m_compIn = NULL;
				}
				connect(this->m_compIn, this->m_compOut);
				return true;
				break;
			default:
				Logger::print("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	Logger::print("KS_BoundToWorldConnection: Warning - end of file met before END primitive\n");
	return false;
}

bool KS_BoundToWorldConnection::writeToFile(FILE* f){

	char* str;

	str = getKSString(KS_BOUND_TO_WORLD_CON);
	fprintf(f, "%s\n", str);

	writeBaseConnectionToFile(f);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_BoundToWorldConnection* KS_BoundToWorldConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const {
	KS_BoundToWorldConnection* pin = new KS_BoundToWorldConnection(*this);
	pin->m_compIn = pCompIn;
	pin->m_compOut = pCompOut;
	pin->lcCon = lcCon->clone(pCompIn,pCompOut);
	return pin;	
}