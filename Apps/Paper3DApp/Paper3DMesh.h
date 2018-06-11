#pragma once

#include "Pin.h"
#include <FEMSimLib/SimulationMesh.h>

class Paper3DMesh : public SimulationMesh {
private:
	// M by 3 matrix to store node indices of each triangle
	Eigen::MatrixXi triangles;
	// N by 2 matrix to store edges between two triangles
	Eigen::MatrixXi edges;

	// Helper data structures for cutting
	// Is node i on the boundary?
	DynamicArray<bool> boundary;
	// Store (ordered) neighbours of each node
	DynamicArray<DynamicArray<int>> orderedAdjacentNodes;

	//generate elements after defining triangles
	void init();

	void cutAtNode(int n_prev, int n, int n_next, int &copy_index);
public:
	Paper3DMesh();
    ~Paper3DMesh();

	static void generateTestSystem(char* fName, int num_nodes);
	static void generateRectangleSystem(char* fName, int nodes_x, int nodes_y, double length_x, double length_y);
    void readMeshFromFile(const char* fName);

    virtual int getSelectedNodeID(Ray ray);
	virtual void setPinnedNode(int ID, const P3D& p);
	virtual void unpinNode(int ID);

	virtual void replacePin(int ID, Pin* replacement);
	virtual void deletePin(int ID);

	virtual void makeCut(const DynamicArray<uint>& path);
};

