#pragma once

#include "FEMSimLib/FixedPointSpring2D.h"


class Mount {
public:
	std::vector<int> node_id;
	std::vector<P3D> position;

	void clear();
	void assignPinnedNode(int id,P3D const & pos);
	void unassignPinnedNode(int id);
	void shift(V3D const & delta);
	void rotate2D(P3D const & origin,double alpha);


};
