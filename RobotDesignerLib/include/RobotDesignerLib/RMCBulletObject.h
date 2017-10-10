#pragma once

#include <BulletCollision/btBulletCollisionCommon.h>
#include <RobotDesignerLib/AbstractBulletObject.h>
#include <RobotDesignerLib/RMC.h>

class RMC;

class RMCBulletObject : public AbstractBulletObject {

public:
	RMC* parent;

	BT_DECLARE_ALIGNED_ALLOCATOR();

	// constructor
	RMCBulletObject(RMC* p);

	~RMCBulletObject() {
		delete colShape;
	};

	RMCBulletObject* clone() {
		RMCBulletObject* newBulletObject = new RMCBulletObject(parent);
		newBulletObject->colShape = new btBoxShape(*((btBoxShape*)colShape));
		newBulletObject->geometryCenterLocalOffset = geometryCenterLocalOffset;
		return newBulletObject;
	}

	void loadFromFile(FILE* fp);
	btCollisionObject* getCollisionObject();

};
