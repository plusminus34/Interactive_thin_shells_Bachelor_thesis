#include "../include/MathLib/Capsule.h"


double Capsule::collisionWithCapsule(const Capsule& cap)
{
	Segment shortestSeg = seg.getShortestSegmentTo(cap.seg);
	double dist = (shortestSeg.a - shortestSeg.b).norm();

	return std::max(0.0, r + cap.r - dist);
}
