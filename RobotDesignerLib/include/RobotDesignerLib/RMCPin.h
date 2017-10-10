#pragma once
#include <RobotDesignerLib/RMCJoint.h>
#include <RobotDesignerLib/RMC.h>
#include <MathLib/Ray.h>
#include <set>

#define PICK_THRESHOLD 0.002

using namespace std;

class RMC;
class RMCJoint;

enum RMCPinType
{
	NORMAL_PIN,
	HORN_PIN
};

enum LivingPinType
{
	LIVING_BODY_PIN,
	LIVING_HORN_PIN,
	NON_LIVING
};

struct PinFace
{
	vector<P3D> vertices;
	P3D center;
	V3D normal;
};

class RMCPin
{
public:
	static bool showCoordinate;

public:
	string name;
	int id;
	// parent RMC
	RMC* rmc;
	// connected RMCJoint
	RMCJoint* joint;
	// transformation relative to parent rmc
	Transformation transformation;
	// if the joint is idle
	bool idle;
	// face for connection
	PinFace face;
	// connecting geometry for the pin
	GLMesh* mesh = NULL;

	set<string> compatibleMap;

	RMCPinType type = NORMAL_PIN;
	LivingPinType livingType = NON_LIVING;

public:
	RMCPin(RMC* _rmc, int _id);
	RMCPin(RMC* _rmc, const Transformation& _transformation, int _id);
	~RMCPin();

	void detach();
	void draw(const V3D& color);
	void loadFromFile(FILE* fp);
	bool isPicked(Ray& ray);
	bool isCompatible(RMCPin* pin);
	RMCPin* getConnectedPin();
};

