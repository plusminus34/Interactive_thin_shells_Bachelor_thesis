#pragma once

#include <MathLib/Box.h>
#include <Utils/Utils.h>

class WOBox : public Box{
public:
	bool canBeSteppedOn;
public:
	WOBox(bool canBeSteppedOn = true) : canBeSteppedOn(canBeSteppedOn) {}
	WOBox(double heading, const P3D &pos, const P3D &c1, const P3D &c2, bool canBeSteppedOn = true) : Box(heading,pos,c1,c2), canBeSteppedOn(canBeSteppedOn) {}
	WOBox(const Quaternion &q, const P3D &pos, const P3D &c1, const P3D &c2, bool canBeSteppedOn = true) : Box(q,pos,c1,c2), canBeSteppedOn(canBeSteppedOn) {}
	virtual ~WOBox() {}
};

class WorldOracle{
public:
	DynamicArray<WOBox> boxes;
	//we assume there is always a ground plane. If it's not needed, it can just be moved below...
	Plane groundPlane;
	//and the world oracle also needs to know which way is up...
	V3D worldUp;

public:
	WorldOracle(const V3D& worldUp, const Plane& groundPlane);
	virtual ~WorldOracle(void);

	/**
		Returns the world height at the given world-space location.
		Only the horizontal components of worldLoc are relevant for this.
		The parameter maxHeight specifies a world-space height which is assumed the maximum,
		i.e., all world objects above this threshold are ignored.
	*/
	virtual double getWorldHeightAt(const P3D& worldLoc);
	
	virtual void writeRBSFile(const char* fName);

private:

};



