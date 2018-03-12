#pragma once
#include <RBSimLib/RigidBody.h>
#include <MathLib/Ray.h>
#include <RobotDesignerLib/RMCPin.h>
#include <BulletCollision/btBulletCollisionCommon.h>
#include <RobotDesignerLib/RMCBulletObject.h>
#include <GUILib/GLIncludes.h>

#define SHOW_PINS 0x0080

class RMCPin;
class RMCJoint;
class RMCBulletObject;

using namespace std;

enum RMCType{
	GENERIC_RMC,
	MOTOR_RMC,
	PLATE_RMC,
	EE_RMC,

	LIVING_CONNECTOR,
	LIVING_SPHERE_EE,
	LIVING_WHEEL_EE,
	LIVING_CONNECTOR_HUB
};

class RMC : public RigidBody{
public:
	vector<RMCPin> pins;
	RMCType type = GENERIC_RMC;

	RMCPin* pickedPin = NULL;

	GLShaderMaterial material;

public:
	RMC() {}
	RMC(GLMesh* mesh);
	virtual ~RMC();

	virtual RMC* clone();

	void copyBasePropertiesTo(RMC* other, bool includePinInfo);

	bool pickPin(Ray& ray);
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void drawPins();

	virtual void loadFromFile(FILE* fp);

	//should return true if it's parsed it, false otherwise...
	virtual bool interpretInputLine(FILE* fp, char* line);

	virtual void update() {}

	virtual void syncSymmParameters(RMC* other){}


	RMCJoint* getParentJoint() {
		if (pJoints.empty())
			return NULL;
		else
			return (RMCJoint*)pJoints[0];
	}

	RMCJoint* getChildJoint(int index) {
		return (RMCJoint*)cJoints[index];
	}

	int getChildJointCount() {
		return (int)cJoints.size();
	}

	void removeParentJoint(Joint* joint) {
		for (uint i = 0; i < pJoints.size(); i++) {
			if (pJoints[i] == joint)
			{
				pJoints.erase(pJoints.begin() + i);
				break;
			}
		}
	}

	void removeChildJoint(Joint* joint) {
		for (uint i = 0; i < cJoints.size(); i++) {
			if (cJoints[i] == joint)
			{
				cJoints.erase(cJoints.begin() + i);
				break;
			}
		}
	}

	bool isConnected(RMC* rmc);

	bool isMovable();

	virtual void exportMeshes(const char* dirName, int index) {}

	virtual void processInputKeyPress(int key) {}

	virtual void writeParamsToCommandLine(char* cmdLine) {
		Quaternion q = state.orientation;
		P3D pos = state.position;

		sprintf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf", q[0], q[1], q[2], q[3], pos[0], pos[1], pos[2]);
	}

	virtual void readParamsFromCommandLine(char* cmdLine) {
		sscanf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf",
			&state.orientation[0], &state.orientation[1], &state.orientation[2], &state.orientation[3],
			&state.position[0], &state.position[1], &state.position[2]);
	}

};

