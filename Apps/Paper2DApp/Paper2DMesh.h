#pragma once

#include <FEMSimLib/SimulationMesh.h>

/**
This class is mostly copy-pasted from the MassSpringSimulationMesh3D
*/
class Paper2DMesh : public SimulationMesh {
private:

public:
	Paper2DMesh();
    ~Paper2DMesh();

	static void generateTestMassSpringSystem(char* fName);
    void readMeshFromFile(const char* fName);

    virtual int getSelectedNodeID(Ray ray);
	virtual void setPinnedNode(int ID, const P3D& p);
};

