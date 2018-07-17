#pragma once

#include "KS_MechanicalComponent.h"
#include "KS_Connection.h"
#include <MathLib/Ray.h>
#include "KS_AssemblyConstraintEnergy.h"

class KS_MechanicalAssembly{
friend class KineSimApp;
friend class KS_MechanismController;
friend class KS_UIMechanismController;
friend class KS_IKMechanismController;
friend class KS_IKConstraintEnergy;

public:
	typedef std::vector<KS_MechanicalComponent*> ComponentArray;
	typedef std::vector<KS_Connection*> ConnectionArray;
	typedef ComponentArray::iterator ComponentIt;
	typedef ConnectionArray::iterator ConnectionIt;

public:

	KS_MechanicalAssembly(void);
	KS_MechanicalAssembly(KS_MechanicalAssembly &a);
	~KS_MechanicalAssembly(void);
	//init from file
	bool readFromFile(const char* szFile);
	void readMechStateFromFile(const char* fName);
	void writeToFile(const char* szFile);

	bool addComponent(KS_MechanicalComponent* pComp);
	void removeComponent(uint i);
	void addConnection(KS_Connection* pConn);
	void removeConnection(uint i);
	int getComponentCount() const {return (int)m_components.size();}
	void setComponentCount(int i){m_components.resize(i,NULL);}
	int getConnectionCount() const {return (int)m_connections.size();}
	void setConnectionCount(int i){m_connections.resize(i,NULL);}
	void stepAssembly();

	KS_MechanicalComponent* getComponent(int i){return m_components[i];}
	KS_Connection* getConnection(int i){return m_connections[i];}
	KS_MechanicalComponent* getComponentByName(char* cName){
		for (uint i=0;i<m_components.size(); i++)
			if (strcmp(m_components[i]->getName(), cName) == 0)
				return m_components[i];
		return NULL;
	}

	void getAssemblyState(dVector& state);
	void setAssemblyState(const dVector& state);
	//have constraints determine the global indeces of the dofs that they work on
	void initConstraints();

	const ComponentArray& getComponents() const { return m_components; }
	const ConnectionArray& getConnections() const { return m_connections; }

	void draw();

	void writeMeshToFile(const char* objFName);

	KS_AssemblyConstraintEnergy* AConstraintEnergy;

	virtual void solveAssembly();

	void logMechS(const char* szFile);

	void setActuatedConnections();

	void updateActuatedConnections();

protected:
	bool newtonSolver = true, bfgsSolver=false;
	ComponentArray m_components;
	ConnectionArray m_connections;

	//collect all the actuated connections for which isActuated flag is set to true; this is passed to the controller class
	ConnectionArray actuated_connections;

	// preparing for solveAssembly
	dVector s, sSolver;

};

