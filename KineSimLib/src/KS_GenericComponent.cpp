#include "KineSimLib/KS_GenericComponent.h"
#include "KineSimLib/KS_LoaderUtils.h"

KS_GenericComponent::KS_GenericComponent(char* rName) : KS_MechanicalComponent(rName) {
	meshName[0] = '\0';
}

KS_GenericComponent* KS_GenericComponent::clone() const {
	KS_GenericComponent* comp = new KS_GenericComponent(*this);
	for(uint i=0;i<meshes.size();i++)
		comp->meshes[i]=this->meshes[i]->clone();
	for(uint i=0;i<this->tracerParticles.size();i++)
		comp->tracerParticles[i].mc=dynamic_cast<KS_MechanicalComponent*>(comp);
	return comp;
}

KS_GenericComponent::~KS_GenericComponent(void){
}

bool KS_GenericComponent::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_GENERIC_COMPONENT);
	fprintf(f, "%s\n", str);

	writeBaseComponentToFile(f);

	if (strlen(meshName) > 0){
		str = getKSString(KS_INPUT_MESH);
		fprintf(f, "\t%s %s\n", str, meshName);
	}

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}


bool KS_GenericComponent::loadFromFile(FILE* f){
	if (f == NULL){
		logPrint("KS_GenericComponent: Cannot load input file.\n");
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
			case KS_END:
				return true;
				break;
			case KS_INPUT_MESH:
				strcpy(meshName, trim(line));
				loadTriangleMeshFromFile(meshName);
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_GenericComponent: Warning - end of file met before END primitive\n");
	return false;
}