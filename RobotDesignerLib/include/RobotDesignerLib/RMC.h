#pragma once
#include <RBSimLib/RigidBody.h>
#include <MathLib/Ray.h>
#include <RobotDesignerLib/RMCPin.h>
#include <BulletCollision/btBulletCollisionCommon.h>
#include <RobotDesignerLib/RMCBulletObject.h>
#include <RobotDesignerLib/LivingHornBracket.h>

#define SHOW_PINS 0x0080

class RMCPin;
class RMCJoint;
class RMCBulletObject;

using namespace std;

enum RMCType
{
	MOTOR_RMC,
	BRACKET_RMC,
	HORN_BRACKET_RMC,
	PLATE_RMC,
	CONNECTOR_RMC,
	EE_RMC,
	EE_HORN_RMC,
	LIVING_MOTOR,
	LIVING_CONNECTOR,
	LIVING_EE
};

class RMC : public RigidBody
{
public:
	vector<RMCPin> pins;
	RMCType type = BRACKET_RMC;
	V3D motorAxis;
	double motorAngle = 0;
	double backupMotorAngle = 0;

	GLMesh* carveMesh = NULL;
	GLMesh* carveMeshEx = NULL;

	RMCPin* pickedPin = NULL;

	GLShaderMaterial material;

	// for collisions
	DynamicArray<RMCBulletObject*>bulletCollisionObjects;

public:
	RMC() {}
	RMC(GLMesh* mesh);
	virtual ~RMC();

	virtual RMC* clone();
	bool pickPin(Ray& ray);
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void loadFromFile(FILE* fp);
	virtual void update() {}

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

	void addBulletObjectsToList(DynamicArray<AbstractBulletObject*>& list);
};

