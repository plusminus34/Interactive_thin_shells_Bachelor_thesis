#pragma once

#include "KS_MechanicalComponent.h"
#include "KS_Ticker.h"
#include "KS_Connection.h"
#include <MathLib/Ray.h>
#include "KS_AssemblyConstraintEnergy.h"

class KS_MechanicalAssembly{
friend class APP_KSSimulator;
friend class APP_KSEditor;
friend class APP_KSMotionCurveOptimizer;
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
	void writeToFile(const char* szFile);

	static void createParameterizedAssemblyFrom(const char* inputKSFile, const char* outputParameterizedAssemblyFile);

	bool addComponent(KS_MechanicalComponent* pComp);
	void removeComponent(uint i);
	void addConnection(KS_Connection* pConn);
	void removeConnection(uint i);
	int getComponentCount() const {return (int)m_components.size();}
	void setComponentCount(int i){m_components.resize(i,NULL);}
	int getConnectionCount() const {return (int)m_connections.size();}
	void setConnectionCount(int i){m_connections.resize(i,NULL);}
	//assumes a single input driver for now
	void stepAssembly(bool updateDriver = true);
	void restartTicker();
	void updateTracerParticles();
	void drawTracerParticles();

	void clearTracerParticles();
	void getTracerParticles(DynamicArray<P3D>& tracerParticleList);

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

	//KS_MechanicalComponent* getFirstComponentIntersectedByRay(const Ray& ray);

	//KS_MechanicalComponent* getFirstComponentIntersectedByRay(const Ray& ray, P3D& worldP);


	void setTickerValue(double val){
		m_ticker.setTickerValue(val);

		for (uint i=0;i<m_connections.size();i++)
			m_connections[i]->updateConnection();
	}

	double getTickerValue(){
		return m_ticker.tick();
	}

	//AABoundingBox computeAABB();

	void writeMeshToFile(const char* objFName);

	KS_AssemblyConstraintEnergy* AConstraintEnergy;

	virtual void solveAssembly();

protected:
	ComponentArray m_components;
	ConnectionArray m_connections;

	KS_Ticker m_ticker;

	// preparing for solveAssembly
	dVector s, sSolver;


};

