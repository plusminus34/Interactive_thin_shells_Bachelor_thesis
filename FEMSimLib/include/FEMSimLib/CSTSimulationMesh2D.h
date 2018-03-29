#pragma once

#include <FEMSimLib/SimulationMesh.h>
#include <FEMSimLib/FixedPointSpring2D.h>


/**
	This class implements a FlexiFrame structure: collection of nodes connected to each other by springs of arbitrary stiffnesses. 
*/
class CSTSimulationMesh2D : public SimulationMesh {
private:

public:
	CSTSimulationMesh2D();
	~CSTSimulationMesh2D();

	virtual void prepareForDraw();

	void readMeshFromFile(const char* fName);
	static void generateSquareTriMesh(char* fName, double startX=-4.5, double startY=0, double dX=1, double dY=1, int xSize=10, int ySize=10);
	virtual int getSelectedNodeID(Ray ray);
	virtual int getSelectedElementID(Ray ray);



	virtual void setPinnedNode(int ID, const P3D& p);
	virtual void unpinNode(int ID) {
		for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it) {
			FixedPointSpring2D* fps = dynamic_cast<FixedPointSpring2D*>(*it);
			if (fps->node == nodes[ID]) {
				pinnedNodeElements.erase(it);
				break;
			}
		}
	}
};
