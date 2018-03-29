#pragma once

#include <LazyFEMSimLib/SimulationMesh.h>

/**
This class implements a FlexiFrame structure: collection of nodes connected to each other by springs of arbitrary stiffnesses.
*/
class MassSpringSimulationMesh3D : public SimulationMesh {
private:


public:
    MassSpringSimulationMesh3D();
    ~MassSpringSimulationMesh3D();

	static void generateTestMassSpringSystem(char* fName);
    void readMeshFromFile(const char* fName);

    virtual int getSelectedNodeID(Ray ray);
	virtual void setPinnedNode(int ID, const P3D& p);
};

