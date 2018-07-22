#include "KineSimlib/KS_xMobileBaseToWorldConnection.h"


KS_xMobileBaseToWorldConnection::KS_xMobileBaseToWorldConnection(void){
	lcCon = NULL;
}

KS_xMobileBaseToWorldConnection::~KS_xMobileBaseToWorldConnection(void){
	delete lcCon;
}

void KS_xMobileBaseToWorldConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;

	assert(m_compIn == NULL);
	lcCon = new KS_LockedComponentConstraint(m_compOut, m_compOut->getAlpha(), true, m_compOut->getBeta(), true, m_compOut->getGamma(), true, m_compOut->getWorldCenterPosition()[0], true, m_compOut->getWorldCenterPosition()[1], true, m_compOut->getWorldCenterPosition()[2], true);
	lcCon->PositionWeight = P3D(1, 100, 100);
}

void KS_xMobileBaseToWorldConnection::updateConnection()
{
	lcCon->pxD = m_offset;
}

void KS_xMobileBaseToWorldConnection::computeddAE_ds_dp()
{
	ddAE_ds_dp2.resize(1, KS_MechanicalComponent::getStateSize()); ddAE_ds_dp2.setZero();
	
	ddAE_ds_dp2(0, 3) = -(lcCon->PositionWeight[0]);
	/*dE_ds[0] = (c->getGamma() - gammaD) * anglesWeight;
	dE_ds[1] = (c->getBe() - betaD) * anglesWeight;
	dE_ds[2] = freezeAlpha * (c->getAlpha() - alphaD) * anglesWeight;
	dE_ds[3] = freezePx * (c->getWorldCenterPosition()[0] - pxD)*positionWeight;
	dE_ds[4] = freezePy * (c->getWorldCenterPosition()[1] - pyD)*positionWeight;
	dE_ds[5] = freezePz * (c->getWorldCenterPosition()[2] - pzD)*positionWeight;*/


}

bool KS_xMobileBaseToWorldConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		Logger::print("KS_xMobileBaseToWorldConnection: Cannot load input file.\n");
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
					Logger::print("KS_xMobileBaseToWorldConnection: Warning! Component IN should be NULL! Ignoring...");
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

	Logger::print("KS_xMobileBaseToWorldConnection: Warning - end of file met before END primitive\n");
	return false;
}

bool KS_xMobileBaseToWorldConnection::writeToFile(FILE* f){

	char* str;

	str = getKSString(KS_XMBASE_CONNECTION);
	fprintf(f, "%s\n", str);

	writeBaseConnectionToFile(f);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_xMobileBaseToWorldConnection* KS_xMobileBaseToWorldConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const {
	KS_xMobileBaseToWorldConnection* pin = new KS_xMobileBaseToWorldConnection(*this);
	pin->m_compIn = pCompIn;
	pin->m_compOut = pCompOut;
	pin->lcCon = lcCon->clone(pCompIn,pCompOut);
	return pin;	
}