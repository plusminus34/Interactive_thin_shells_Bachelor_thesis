#include "KineSimLib/KS_PointOnLineConnection.h"
#include "KineSimLib/KS_LoaderUtils.h"
#include "KineSimLib/KS_MechanicalAssembly.h"

KS_PointOnLineConnection::KS_PointOnLineConnection(void){
	pOnlConstraint = NULL;
	v2vConstraint = NULL;
	xOnC1 = P3D(0,0,0);
	pOnC2 = P3D(0,0,0);
	lOnC2 = V3D(1,0,0);
	nOnC1 = nOnC2 = V3D(0,0,1);
	constrainedNormal = true;
}

KS_PointOnLineConnection::~KS_PointOnLineConnection(void){
	delete pOnlConstraint;
	delete v2vConstraint;
}

void KS_PointOnLineConnection::connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut){
	assignConnectedComponents(pCompIn, pCompOut);
	m_compIn = pCompIn;
	m_compOut = pCompOut;

	m_compIn->addCylinderMesh(20, 0.1, 0.1, xOnC1, nOnC1);

	pOnlConstraint = new KS_PointOnLineConstraint(xOnC1, m_compIn, pOnC2, lOnC2, m_compOut); 

	if (constrainedNormal)
		v2vConstraint = new KS_V2VConstraint(nOnC1, m_compIn, nOnC2, m_compOut); 
}

bool KS_PointOnLineConnection::loadFromFile(FILE* f, KS_MechanicalAssembly* ma){
	if (f == NULL){
		Logger::print("BindComponentsConnection: Cannot load input file.\n");
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
			case KS_PIN_ON_COMP_IN:
				if (sscanf(line, "%lf %lf %lf", &xOnC1[0], &xOnC1[1], &xOnC1[2]) != 3) assert(false);
				break;
			case KS_LINE_ON_COMP_OUT:
				if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &pOnC2[0], &pOnC2[1], &pOnC2[2], &lOnC2[0], &lOnC2[1], &lOnC2[2]) != 6) assert(false);
				lOnC2.normalize();
				break;
			case KS_VEC_ON_COMP_IN:
				if (sscanf(line, "%lf %lf %lf", &nOnC1[0], &nOnC1[1], &nOnC1[2]) != 3) assert(false);
				nOnC1.normalize();
				break;
			case KS_VEC_ON_COMP_OUT:
				if (sscanf(line, "%lf %lf %lf", &nOnC2[0], &nOnC2[1], &nOnC2[2]) != 3) assert(false);
				nOnC2.normalize();
				break;
			case KS_ALLOW_ARBITRARY_ROTATION:
				constrainedNormal = false;
				break;
			default:
				Logger::print("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	Logger::print("BindComponentsConnection: Warning - end of file met before END primitive\n");
	return false;
}

bool KS_PointOnLineConnection::writeToFile(FILE* f){

	char* str;

	str = getKSString(KS_POINT_ON_LINE_CON);
	fprintf(f, "%s\n", str);

	writeBaseConnectionToFile(f);

	str = getKSString(KS_PIN_ON_COMP_IN);
	fprintf(f, "\t%s %lf %lf %lf\n", str, xOnC1[0], xOnC1[1], xOnC1[2]);

	str = getKSString(KS_LINE_ON_COMP_OUT);
	fprintf(f, "\t%s %lf %lf %lf %lf %lf %lf\n", str, pOnC2[0], pOnC2[1], pOnC2[2], lOnC2[0], lOnC2[1], lOnC2[2]);

	str = getKSString(KS_VEC_ON_COMP_IN);
	fprintf(f, "\t%s %lf %lf %lf\n", str, nOnC1[0], nOnC1[1], nOnC1[2]);

	str = getKSString(KS_VEC_ON_COMP_OUT);
	fprintf(f, "\t%s %lf %lf %lf\n", str, nOnC2[0], nOnC2[1], nOnC2[2]);

	if (constrainedNormal == false){
		str = getKSString(KS_ALLOW_ARBITRARY_ROTATION);
		fprintf(f, "\t%s\n", str);
	}

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}

KS_PointOnLineConnection* KS_PointOnLineConnection::clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut)const{
	KS_PointOnLineConnection* pin = new KS_PointOnLineConnection(*this);
	pin->connect(pCompIn,pCompOut);
	return pin;
}