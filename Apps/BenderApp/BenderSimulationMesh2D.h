
#include <array>

#include "FEMSimLib/CSTSimulationMesh2D.h"
#include "Mount.h"


class BenderSimulationMesh2D : public CSTSimulationMesh2D {

public:
	std::array<Mount*, 2> mounts;

public:
	BenderSimulationMesh2D();
	~BenderSimulationMesh2D();

	virtual void setMountedNode(int nodeID, const P3D & x0, int mountID);


};
