#include "KineSimLib/KS_NonCircularGearsConnection.h"
#include "KineSimLib/KS_Gear.h"
#include "KineSimLib/KS_LoaderUtils.h"


/**
	Very important!! The gear2gear connection assumes the two gears connected to each other do not move relative to each other!
*/
KS_NonCircularGearsConnection::KS_NonCircularGearsConnection(void){
	m_pGearIn = m_pGearOut = NULL;
	s1 = 0.2;
	s2 = 0.4;
	s3 = 0.6;
	s4 = 0.8;
}

KS_NonCircularGearsConnection::~KS_NonCircularGearsConnection(void){
	delete nonLinearPhaseConstraint;
	delete v2vConstraint;
}


void KS_NonCircularGearsConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	nonConstantSpeedProfile.setRBF1DFunctionData(s1, s2, s3, s4);

	m_pGearIn = dynamic_cast<KS_NonCircularGear*>(pCompIn);
	m_pGearOut = dynamic_cast<KS_NonCircularGear*>(pCompOut);

	assert (m_pGearIn != NULL);
	assert (m_pGearOut != NULL);

	assert(m_pGearIn->getNumTeeth() == m_pGearOut->getNumTeeth());

	//now we need to create the geometry for these two gears - start with the profiles...
	Trajectory3D gear1Profile, gear2Profile;
	//assume that the two gears are already position correctly in the world!!
	Point3d gear1Pos = m_pGearIn->getWorldCenterPosition();
	Point3d gear2Pos = m_pGearOut->getWorldCenterPosition();
	Vector3d dir = (gear1Pos - gear2Pos);
	double dist = dir.length();
	dir /= dist;	

	int sampleCount = 5000;

	for (int i=0; i < sampleCount; i++){
		double t = ((double)i / sampleCount);

		//we assume that the default rate of speed is 1 to 1 (i.e. v == 0 implies r1 = r2), and we know that r1 + r2 = a always. Let p = w1/w2 = v = r1/r2
		double v = nonConstantSpeedProfile.evaluateDerivative(t);
		//no going backwards...
		if (v < 0) v = 0;
		double r2 = dist / (v+1);
		boundToRange(&r2, dist*0.1, dist-0.1*dist);
		double r1 = dist - r2;

//		logPrint3("%lf %lf\n", r1, r2);

		Vector3d v1 = rotate(-dir, 2 * PI * -t, Vector3d(0,0,1)) * r1;

		Vector3d v2 = rotate(-dir, 2 * PI * nonConstantSpeedProfile.evaluate(t) + PI, Vector3d(0,0,1)) * r2;

		gear1Profile.addKnot(1-t, v1);
		gear2Profile.addKnot(1-t, v2);
	}

	double lenGear1=0, lenGear2=0;
	for (int i=0;i<gear1Profile.getKnotCount()-1; i++)
		lenGear1 += (gear1Profile.getKnotValue(i) - gear1Profile.getKnotValue(i+1)).length();
	lenGear1 += (gear1Profile.getKnotValue(0) - gear1Profile.getKnotValue(gear1Profile.getKnotCount()-1)).length();
	for (int i=0;i<gear2Profile.getKnotCount()-1; i++)
		lenGear2 += (gear2Profile.getKnotValue(i) - gear2Profile.getKnotValue(i+1)).length();	
	lenGear2 += (gear2Profile.getKnotValue(0) - gear2Profile.getKnotValue(gear2Profile.getKnotCount()-1)).length();

//	logPrint3("-----------------\n");
//	logPrint3("%lf %lf\n", lenGear1, lenGear2);

	m_pGearIn->createGearGeometry(&gear1Profile, lenGear1, 0, true);
	m_pGearOut->createGearGeometry(&gear2Profile, lenGear2, 0.5, false);

	//these gears should always lie in the same plane
	v2vConstraint = new KS_V2VConstraint(Vector3d(0,0,1), m_pGearIn, Vector3d(0,0,1), m_pGearOut); 
	nonLinearPhaseConstraint = new KS_NonLinearPhaseConstraint(m_pGearIn, m_pGearOut, s1, s2, s3, s4);
}

bool KS_NonCircularGearsConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		logPrint("Gear2GearConnection: Cannot load input file.\n");
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
				connect(this->m_compIn, this->m_compOut);
				return true;
				break;
			case KS_PHASE_PROFILE:
				if (sscanf(line, "%lf %lf %lf %lf", &s1, &s2, &s3, &s4) != 4) assert(false);
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("Gear2GearConnection: Warning - end of file met before END primitive\n");
	return false;
}

bool KS_NonCircularGearsConnection::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_NON_CIRCULAR_GEARS_CON);
	fprintf(f, "%s\n", str);

	writeBaseConnectionToFile(f);

	str = getKSString(KS_PHASE_PROFILE);
	fprintf(f,"\t%s %lf %lf %lf %lf\n", str, s1, s2, s3, s4);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_NonCircularGearsConnection* KS_NonCircularGearsConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut, KS_Ticker* clock) const {
	KS_NonCircularGearsConnection* pin = new KS_NonCircularGearsConnection(*this);
	pin->m_pGearIn = dynamic_cast<KS_NonCircularGear*>(pCompIn);
	pin->m_pGearOut = dynamic_cast<KS_NonCircularGear*>(pCompOut);
	pin->nonLinearPhaseConstraint = nonLinearPhaseConstraint->clone(pCompIn,pCompOut);
	pin->v2vConstraint = v2vConstraint->clone(pCompIn,pCompOut);
	return pin;
}