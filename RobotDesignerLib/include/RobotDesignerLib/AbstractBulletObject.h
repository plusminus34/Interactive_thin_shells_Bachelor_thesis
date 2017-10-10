#pragma once


#include <BulletCollision/btBulletCollisionCommon.h>
#include <MathLib/V3D.h>

enum BulletObjectType {
	RMC_OBJECT,
	SEARCH_NODE_OBJECT
};

class AbstractBulletObject {

public:
	BulletObjectType type;
	btCollisionObject collisionObject;
	btCollisionShape* colShape;
	V3D geometryCenterLocalOffset;

	BT_DECLARE_ALIGNED_ALLOCATOR();

	// constructor
	AbstractBulletObject() {};

	virtual ~AbstractBulletObject() {
		delete colShape;
	};

	virtual AbstractBulletObject* clone() = 0;

	virtual void loadFromFile(FILE* fp) = 0;
	virtual btCollisionObject* getCollisionObject() = 0;

};
