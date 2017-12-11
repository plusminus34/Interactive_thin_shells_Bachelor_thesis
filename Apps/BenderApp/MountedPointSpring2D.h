#include "FEMSimLib/FixedPointSpring2D.h"
#include "Mount.h"



class MountedPointSpring2D : public FixedPointSpring2D {

public: 
	Mount * mount;

public: 
	MountedPointSpring2D(SimulationMesh * simMesh, Node * node, P3D referencePosition, Mount * mount);
	~MountedPointSpring2D();

	virtual double getEnergy(const dVector& x, const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries);
	//virtual void draw(const dVector& x);

};