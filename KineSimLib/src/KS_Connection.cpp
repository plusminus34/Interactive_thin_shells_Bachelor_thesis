#include "KineSimLib/KS_Connection.h"
#include "KineSimLib/KS_LoaderUtils.h"
#include "KineSimLib/KS_MechanicalAssembly.h"

KS_Connection::KS_Connection(void){
	m_compIn = m_compOut = NULL;
	m_pin = NULL;
	isNew=false;
}

KS_Connection::~KS_Connection(void){

}

void KS_Connection::assignConnectedComponents(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	if (pCompIn)
		strcpy(compInName, pCompIn->getName());
	if (pCompOut)
		strcpy(compOutName, pCompOut->getName());
}

void KS_Connection::writeBaseConnectionToFile(FILE* f){
	char* str;

	if (m_compIn){
		str = getKSString(KS_COMPONENT_IN);
		fprintf(f, "\t%s %s\n", str, compInName);
	}

	if (m_compOut){
		str = getKSString(KS_COMPONENT_OUT);
		fprintf(f, "\t%s %s\n", str, compOutName);
	}
}


//returns true if the input line was processed, false otherwise
bool KS_Connection::processInputLine(char* line, KS_MechanicalAssembly* ma){
	int lineType = getKSLineType(line);
	switch (lineType) {
		case KS_COMPONENT_IN:
			strcpy(compInName, trim(line));
			m_compIn = ma->getComponentByName(compInName);
			assert(m_compIn != NULL);
			return true;
			break;
		case KS_COMPONENT_OUT:
			strcpy(compOutName, trim(line));
			m_compOut = ma->getComponentByName(compOutName);
			assert(m_compOut != NULL);
			return true;
			break;
	}
	return false;
}

