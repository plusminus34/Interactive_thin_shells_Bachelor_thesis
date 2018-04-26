#pragma once

#include <FEMSimLib/SimulationMesh.h>

class Paper3DMesh : public SimulationMesh {
private:

public:
	Paper3DMesh();
    ~Paper3DMesh();

	static void generateTestSystem(char* fName, int num_nodes);
    void readMeshFromFile(const char* fName);

    virtual int getSelectedNodeID(Ray ray);
	virtual void setPinnedNode(int ID, const P3D& p);
};

