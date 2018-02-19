#pragma once

#include <MathLib/mathLib.h>
#include <MathLib/P3D.h>
#include <MathLib/V3D.h>

// Forward declarations.
class SimulationMesh;

class Node {

public:
	// Pointer back to the mesh.
	SimulationMesh* mesh;
	// Index of node in simulation mesh.
	int nodeIndex;
	//in the arrays of the simulation mesh, this is where the data associated with this node lives - masses, deformed and un-deformed configurations, etc...
	int dataStartIndex;
	int dimSize;

	// Flags.
	bool selected;
	bool pinned;

public:
	Node(SimulationMesh* m, int nodeIndex, int dataStartIndex, int dimSize);
	~Node();

	void draw();

	P3D  getCurrentPosition();
	void setCurrentPosition(const P3D &);
	P3D  getTargetPosition();
	void setTargetPosition(const P3D &);
	P3D  getUndeformedPosition();
	void setUndeformedPosition(const P3D &);
	V3D  getVelocity();
	void setVelocity(const V3D &);
	V3D  getExternalForce();

	P3D getCoordinates(const dVector& x);
	void setCoordinates(const P3D& coords, dVector& x);

	//different elements may want to divide their mass to the nodes that define them...
	void addMassContribution(double m);
	double getMass();
};

