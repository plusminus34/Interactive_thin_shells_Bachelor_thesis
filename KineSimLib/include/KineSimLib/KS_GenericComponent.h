#pragma once
#include "KS_MechanicalComponent.h"


class KS_GenericComponent : public KS_MechanicalComponent{
friend class KS_GUIDesigner;
private:
	char meshName[200];

public:
	KS_GenericComponent(char* rName);
	~KS_GenericComponent(void);
	
	virtual void addMesh(GLMesh* m,char* name){
		m->setColour(meshColor[0], meshColor[1], meshColor[2], 1);// todo: add this function or remove it
		meshes.push_back(m);
		strcpy(meshName,name);
	}

	virtual void setupGeometry() {assert(false); }
	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);
	virtual KS_GenericComponent* clone() const;

};

