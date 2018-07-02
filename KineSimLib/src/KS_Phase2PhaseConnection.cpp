#include "KineSimLib/KS_Phase2PhaseConnection.h"
#include "KineSimLib/KS_MechanicalComponent.h"
#include "KineSimLib/KS_LoaderUtils.h"

KS_Phase2PhaseConnection::KS_Phase2PhaseConnection(void){
	setSign(true);
	setRatio(1.0);
	setOffset(0);
}

KS_Phase2PhaseConnection::~KS_Phase2PhaseConnection(void){
	delete p2pConstraint;
}

void KS_Phase2PhaseConnection::createPhaseToPhaseConstraint(){
	p2pConstraint = new KS_PhaseToPhaseConstraint(m_compIn, m_compOut, m_ratio*m_sign, m_offset);
}

void KS_Phase2PhaseConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;
	createPhaseToPhaseConstraint();
}

bool KS_Phase2PhaseConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	//not used!!
	assert(false);
	return false;
}


bool KS_Phase2PhaseConnection::writeToFile(FILE* f){
	//not used!!
	assert(false);
	return false;
}

KS_Phase2PhaseConnection* KS_Phase2PhaseConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const {
	KS_Phase2PhaseConnection* pin = new KS_Phase2PhaseConnection(*this);
	pin->connect(pCompIn,pCompOut);
	return pin;
}