#pragma once

#include <LazyFEMSimLib/BaseEnergyUnit.h>
#include <MathLib/P3D.h>

class SimulationMesh;
/**
	This class implements a base class for generic elements. Every simulation mesh is decomposed into a set of simple elements (triangles, tets, etc).
*/
class SimMeshElement : public BaseEnergyUnit {
protected:
	//this is the simulation mesh that this element belongs to
	SimulationMesh* simMesh;

public:
	SimMeshElement(SimulationMesh* simMesh);
	~SimMeshElement();

	virtual double getMass() = 0;

	virtual double getEnergy(const dVector& x, const dVector& X) = 0;
	virtual void drawRestConfiguration(const dVector& X) = 0;
	virtual void draw(const dVector& x) = 0;

};


