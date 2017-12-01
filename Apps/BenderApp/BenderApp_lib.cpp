#include "BenderApp_lib.h"
#include "FEMSimLib/FixedPointSpring2D.h"




void Mount::clear()
{
	node_id.resize(0);
	position.resize(0);
}

void Mount::assignPinnedNode(int id, P3D const & pos)
{
	node_id.push_back(id);
	position.push_back(pos);
}

void Mount::unassignPinnedNode(int id)
{
	for (size_t i = 0; i < node_id.size(); ++i) {
		if (node_id[i] == id) {
			node_id.erase(node_id.begin() + i);
			position.erase(position.begin() + i);
		}
	}
}

void Mount::shift(V3D const & delta)
{
	for (P3D & pos : position) {
		pos += delta;
	}
}

void Mount::rotate2D(P3D const & origin, double alpha)
{
	Matrix3x3 T_rot;
	T_rot << std::cos(alpha), -std::sin(alpha), 0,
		     std::sin(alpha), std::cos(alpha),  0,
		     0,               0,                1;
	//P3D origin2D = origin;
	//origin2D(2) = 0;
	V3D shift_temp = (Matrix3x3::Identity() - T_rot) * origin;
	for(P3D & pos : position) {
		V3D pos_new = (T_rot * pos + shift_temp);
		pos = pos_new;
	}
}