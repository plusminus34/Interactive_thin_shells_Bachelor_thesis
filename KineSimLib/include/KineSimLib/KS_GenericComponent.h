#pragma once
#include "KS_MechanicalComponent.h"


class KS_GenericComponent : public KS_MechanicalComponent{
private:
	char meshName[200];

public:
	KS_GenericComponent(char* rName);
	~KS_GenericComponent(void);
	
	virtual void addMesh(GLMesh* m,char* name){
		meshes.push_back(m);
		strcpy(meshName,name);
	}

	virtual void setupGeometry();
	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);
	virtual KS_GenericComponent* clone() const;

};

