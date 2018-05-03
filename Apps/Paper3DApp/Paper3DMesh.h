#pragma once

#include <FEMSimLib/SimulationMesh.h>

class Paper3DMesh : public SimulationMesh {
private:

public:
	Paper3DMesh();
    ~Paper3DMesh();

	static void generateTestSystem(char* fName, int num_nodes);
	static void generateRectangleSystem(char* fName, int nodes_x, int nodes_y, double length_x, double length_y);
    void readMeshFromFile(const char* fName);

    virtual int getSelectedNodeID(Ray ray);
	virtual void setPinnedNode(int ID, const P3D& p);
};

