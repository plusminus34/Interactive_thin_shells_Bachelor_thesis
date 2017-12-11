
#include <array>

#include "FEMSimLib/CSTSimulationMesh2D.h"
#include "Mount.h"
#include "MeshObjective.h"


class BenderSimulationMesh2D : public CSTSimulationMesh2D {

public:
	std::array<Mount*, 2> mounts;
	std::vector<MeshObjective *> objectives;

public:
	BenderSimulationMesh2D();
	~BenderSimulationMesh2D();

	void setMountedNode(int nodeID, const P3D & x0, int mountID);

	double computeO(double & o);
	void computeDoDx(dVector & dodx);

};
