#pragma once
#include "P3D.h"
#include "Segment.h"
#include "MathLib.h"

class Capsule
{
public:
	Segment seg;
	double r = 0.0;

public:
	Capsule(){}
	Capsule(const P3D& _a, const P3D& _b) : seg(Segment(_a, _b)) {}
	Capsule(const P3D& _a, const P3D& _b, double _r) : seg(Segment(_a, _b)), r(_r) {}
	Capsule(const Segment& _seg) : seg(_seg) {}
	Capsule(const Segment& _seg, double _r): seg(_seg), r(_r) {}
	~Capsule() {}

public:
	double collisionWithCapsule(const Capsule& cap);
};

