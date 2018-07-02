#include "KineSimLib/KS_Shaft.h"
#include "KineSimLib/KS_LoaderUtils.h"

KS_Shaft::KS_Shaft(char* sName) : KS_MechanicalComponent(sName){
	radius = 0.2;
	length = 1.0;
}

KS_Shaft* KS_Shaft::clone() const {
	KS_Shaft* comp = new KS_Shaft(*this);
	for(uint i=0;i<meshes.size();i++)
		comp->meshes[i]=this->meshes[i]->clone();
	for(uint i=0;i<this->tracerParticles.size();i++)
		comp->tracerParticles[i].mc=dynamic_cast<KS_MechanicalComponent*>(comp);
	return comp;
}

KS_Shaft::~KS_Shaft(void){
}

bool KS_Shaft::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_SHAFT);
	fprintf(f, "%s\n", str);

	writeBaseComponentToFile(f);

	str = getKSString(KS_RADIUS);
	fprintf(f, "\t%s %lf\n", str, radius);

	str = getKSString(KS_LENGTH);
	fprintf(f, "\t%s %lf\n", str, length);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}


bool KS_Shaft::loadFromFile(FILE* f){
	if (f == NULL){
		logPrint("KS_Shaft: Cannot load input file.\n");
		return false;
	}
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, sizeof(buffer));
		char *line = lTrim(buffer);

		if (KS_MechanicalComponent::processInputLine(line))
			continue;

		int lineType = getKSLineType(line);
		switch (lineType) {
			case KS_RADIUS:
				if (sscanf(line, "%lf", &radius) != 1) assert(false);
				break;
			case KS_LENGTH:
				if (sscanf(line, "%lf", &length) != 1) assert(false);
				break;
			case KS_END:
				setupGeometry();
				return true;
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_Shaft: Warning - end of file met before END primitive\n");
	return false;
}

void KS_Shaft::setupGeometry(){
	addCylinderMesh(15, radius, length, Vector3d(0,0,0), Vector3d(0,0,1), true);
}
