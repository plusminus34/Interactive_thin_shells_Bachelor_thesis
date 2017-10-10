#include <RobotDesignerLib/RMCBulletObject.h>

RMCBulletObject::RMCBulletObject(RMC* p) {
	parent = p;
	type = RMC_OBJECT;
}

btCollisionObject* RMCBulletObject::getCollisionObject() {
	P3D	position = parent->getWorldCoordinates(P3D() + geometryCenterLocalOffset);
	Quaternion rotation = parent->getOrientation();
	collisionObject.getWorldTransform().setOrigin(btVector3((btScalar)position[0], (btScalar)position[1], (btScalar)position[2]));
	collisionObject.getWorldTransform().setRotation(btQuaternion((btScalar)rotation.v[0], (btScalar)rotation.v[1], (btScalar)rotation.v[2], (btScalar)rotation.s));
	colShape->setMargin(0.0005f);
	collisionObject.setCollisionShape(colShape);
	collisionObject.setUserPointer(this);
	collisionObject.setCollisionFlags(0);

	return &collisionObject;
}

void RMCBulletObject::loadFromFile(FILE* fp) {

	char buffer[200];
	char keyword[50];

	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		sscanf(line, "%s", keyword);
		Logger::print("%s ", keyword);

		if (strcmp(keyword, "Size") == 0)
		{
			V3D size;
			sscanf(line + strlen(keyword), "%lf %lf %lf", &size[0], &size[1], &size[2]);
			colShape = new btBoxShape(btVector3((btScalar)(size[0]), (btScalar)(size[1]), (btScalar)(size[2])));
		}
		else if (strcmp(keyword, "Offset") == 0)
		{
			sscanf(line + strlen(keyword), "%lf %lf %lf", &geometryCenterLocalOffset[0], &geometryCenterLocalOffset[1], &geometryCenterLocalOffset[2]);
		}
		else if (strcmp(keyword, "EndBox") == 0)
		{
			break;
		}
	}
}