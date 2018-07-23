#pragma once

#include <stdio.h>
#include <MathLib/MathLib.h>
#include <MathLib/P3D.h>
#include "KS_MechanicalComponent.h"

class KS_MechanicalComponent;
class KS_MechanicalAssembly;
class KS_Constraint;
class GLMesh;

class KS_Connection{
public:
	KS_Connection(void);
	virtual ~KS_Connection(void);
	virtual void connect(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) = 0;
	void assignConnectedComponents(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut);
	KS_MechanicalComponent* getInput()   { return m_compIn; }
	KS_MechanicalComponent* getOutput()  { return m_compOut; }

	virtual bool loadFromFile(FILE* f, KS_MechanicalAssembly* ma) = 0;
	virtual bool writeToFile(FILE* f) = 0;
	virtual KS_Connection* clone(KS_MechanicalComponent* pCompIn, KS_MechanicalComponent* pCompOut) const = 0;

	void writeBaseConnectionToFile(FILE* f);

	virtual void addConstraintsToList(DynamicArray<KS_Constraint*>& constraints) {assert(false);}

	//GLMesh* getPin(){return m_compOut->getMesh(m_pin);}
	void setPin(int p){m_pin=p;}
	int getPin(){return m_pin;}

protected:
	KS_MechanicalComponent* m_compIn;
	KS_MechanicalComponent* m_compOut;

	int m_pin;

	char compInName[200];
	char compOutName[200];

	//returns true if the input line was processed, false otherwise
	bool processInputLine(char* line, KS_MechanicalAssembly* ma);
};

