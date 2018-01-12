#include <type_traits>

#include "FEMSimLib/FixedPointSpring2D.h"
#include "FEMSimLib/FixedPointSpring3D.h"
#include "Mount.h"


template <int NDim>
class MountedPointSpring
: 	public std::conditional<NDim == 2, FixedPointSpring2D, FixedPointSpring3D>::type {

public:
	Mount * mount;

public: 
	MountedPointSpring(SimulationMesh * simMesh, 
					   Node * node, 
					   P3D referencePosition, 
					   Mount * mount, 
					   double K);

	~MountedPointSpring();

	virtual double getEnergy(const dVector& x, 
						     const dVector& X);
	virtual void addEnergyGradientTo(const dVector& x, 
								     const dVector& X, 
									 dVector& grad);
	virtual void addEnergyHessianTo(const dVector& x, 
									const dVector& X, 
									std::vector<MTriplet>& hesEntries);

	virtual void addDeltaFDeltaXi(std::vector<dVector> & dfdxi);	// each dVector is the dF for one parameter xi
	virtual void draw(const dVector& x);
	void draw(const dVector& x, double size, double r, double g, double b);

};