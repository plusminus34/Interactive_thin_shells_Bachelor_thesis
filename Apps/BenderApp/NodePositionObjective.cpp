
#include "MathLib/V3D.h"

#include "NodePositionObjective.h"

void NodePositionObjective::addO(const dVector & x, const dVector & X, double & O) const
{
	V3D d = mesh->nodes[nodeID]->getCoordinates(x) - targetPosition;
	O += 0.5 * d.dot(d);
}


void NodePositionObjective::addDoDx(const dVector & x, const dVector & X, dVector & dodx) const
{
	Node const *& node = mesh->drawNodes[nodeID];
	for(int i = 0; i < node->dimSize; ++i) {
		dodx[node->dataStartIndex + i] += x[node->dataStartIndex + i];
	}
}