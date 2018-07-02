#include "KineSimLib/KS_Gear2GearConnection.h"
#include "KineSimLib/KS_Gear.h"
#include "KineSimLib/KS_LoaderUtils.h"

/**
	Very important!! The gear2gear connection assumes the two gears connected to each other do not move relative to each other!
*/

KS_Gear2GearConnection::KS_Gear2GearConnection(void){
	m_pGearIn = m_pGearOut = NULL;
}

KS_Gear2GearConnection::~KS_Gear2GearConnection(void){
//	delete p2pConstraint;
}

void KS_Gear2GearConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	KS_Phase2PhaseConnection::connect(pCompIn,pCompOut);
	m_pGearIn = dynamic_cast<KS_Gear*>(pCompIn);
	m_pGearOut = dynamic_cast<KS_Gear*>(pCompOut);
	if (m_pGearIn && m_pGearOut){
		setRatio((double)m_pGearOut->getNumTeeth()/m_pGearIn->getNumTeeth());
		setSign(false);
	}else{
		assert(false);
	}
	createPhaseToPhaseConstraint();
}


bool KS_Gear2GearConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		logPrint("KS_Gear2GearConnection: Cannot load input file.\n");
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
			case KS_GEAR_2_GEAR_RELATIVE_PHASE_SHIFT:
				{
				double offset = 0;
				if(sscanf(line, "%lf", &offset) != 1) assert(false);
				setOffset(offset);
				}
				break;
			case KS_END:
				connect(this->m_compIn, this->m_compOut);
				return true;				
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_Gear2GearConnection: Warning - end of file met before END primitive\n");
	return false;
}

bool KS_Gear2GearConnection::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_GEAR_2_GEAR_CON);
	fprintf(f, "%s\n", str);
	
	writeBaseConnectionToFile(f);

	str = getKSString(KS_GEAR_2_GEAR_RELATIVE_PHASE_SHIFT);
	fprintf(f, "\t%s %lf\n", str, m_offset);
	

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_Gear2GearConnection* KS_Gear2GearConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const {
	KS_Gear2GearConnection* pin = new KS_Gear2GearConnection(*this);
	pin->m_pGearIn = dynamic_cast<KS_Gear*>(pCompIn);
	pin->m_pGearOut = dynamic_cast<KS_Gear*>(pCompOut);
	pin->p2pConstraint = p2pConstraint->clone(pCompIn,pCompOut); 
	return pin;
}