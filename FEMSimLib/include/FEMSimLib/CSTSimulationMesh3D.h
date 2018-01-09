#pragma once

#include <FEMSimLib/SimulationMesh.h>

/**
This class implements a FlexiFrame structure: collection of nodes connected to each other by springs of arbitrary stiffnesses.
*/
class CSTSimulationMesh3D : public SimulationMesh {
private:

public:
    CSTSimulationMesh3D();
    ~CSTSimulationMesh3D();

    void readMeshFromFile(const char* fName);
	void readMeshFromFile_ply(char* fName);

    static void generateCubeTriMesh(char* fName);

    virtual int getSelectedNodeID(Ray ray);
	virtual void setPinnedNode(int ID, const P3D& p);
};

