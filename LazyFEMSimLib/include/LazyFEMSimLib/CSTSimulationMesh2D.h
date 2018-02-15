#pragma once

#include <LazyFEMSimLib/SimulationMesh.h>

/**
	This class implements a FlexiFrame structure: collection of nodes connected to each other by springs of arbitrary stiffnesses. 
*/
class CSTSimulationMesh2D : public SimulationMesh {
private:

public:
	CSTSimulationMesh2D();
	~CSTSimulationMesh2D();

	void readMeshFromFile(const char* fName);
	static void generateSquareTriMesh(char* fName, double startX=-4.5, double startY=0, double dX=1, double dY=1, int xSize=10, int ySize=10);
	virtual int getSelectedNodeID(Ray const & ray);
	virtual void setPinnedNode(int ID, const P3D& p);
};
