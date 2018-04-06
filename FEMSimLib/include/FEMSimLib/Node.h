#pragma once

#include <MathLib/MathLib.h>
#include <MathLib/P3D.h>
#include <MathLib/V3D.h>

/**
	This class implements a node in a FlexiFrame structure. Different types of edge elements are used to connect pairs of nodes
*/

class SimulationMesh;
class SimMeshElement;

class Node{
public:
	//and this is the index of the node in the sim mesh
	int nodeIndex;
	//in the arrays of the simulation mesh, this is where the data associated with this node lives - masses, deformed and un-deformed configurations, etc...
	int dataStartIndex;
	//and this is the dimension size for this type of node
	int dimSize;

	//the simulation mesh, which is where all the data is stored...
	SimulationMesh* mesh;

	//flag that indicates whether this node is selected or not...
	bool selected;
	 
	bool fixed = false; P3D fixedPos;

	double avgDefEnergyForDrawing = 0;

	DynamicArray<SimMeshElement*> adjacentElements;

public:
	Node(SimulationMesh* m, int nodeIndex, int dataStartIndex, int dimSize);
	~Node();

	void draw(V3D const & color = V3D(1.0,0.0,0.0), double size = 0.005);

	P3D getWorldPosition();
	void setWorldPosition(const P3D& newPos);
	P3D getUndeformedPosition();
	V3D getVelocity();
	void setVelocity(const V3D& newVel);
	V3D getExternalForce();

	P3D getCoordinates(const dVector& x);
	void setCoordinates(const P3D& coords, dVector& x);

	//different elements may want to divide their mass to the nodes that define them...
	void addMassContribution(double m);
};

