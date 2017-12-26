#include <RobotDesignerLib/RMC.h>
#include <GUILib/GLApplication.h>
#include <RobotDesignerLib/RMCBulletObject.h>

RMC::RMC(GLMesh* mesh)
{
	this->meshes.push_back(mesh);
}


RMC::~RMC()
{
	/*for (auto bulletObj : bulletCollisionObjects)
	{
		delete bulletObj;
	}*/
}

RMC* RMC::clone()
{
	RMC* new_rmc = new RMC();
	new_rmc->state = state;
	new_rmc->rbProperties = rbProperties;

	new_rmc->meshes = meshes;
	new_rmc->name = name;
	new_rmc->id = id;
	new_rmc->type = type;
	new_rmc->motorAxis = motorAxis;
	new_rmc->motorAngle = motorAngle;

	new_rmc->material = material;

	new_rmc->mappingInfo = mappingInfo;

	// shouldn't copy these old pointers.
	//new_rmc->cJoints = cJoints;
	//new_rmc->pJoints = pJoints;

	new_rmc->pins = pins;
	for (uint i = 0; i < pins.size(); i++)
		new_rmc->pins[i].rmc = new_rmc;

	for (uint i = 0; i < bulletCollisionObjects.size(); i++) {
		new_rmc->bulletCollisionObjects.push_back(bulletCollisionObjects[i]->clone());
		new_rmc->bulletCollisionObjects.back()->parent = new_rmc;
	}	

	return new_rmc;
}

bool RMC::isConnected(RMC* rmc) {

	return (getParentJoint() && getParentJoint()->getParent() == rmc) || (rmc->getParentJoint() && rmc->getParentJoint()->getParent() == this);
}

bool RMC::isMovable()
{
	if (pJoints.empty()) return true;

	for (auto pJoint : pJoints)
	{
		RMC* rmc = dynamic_cast<RMC*>(pJoint->parent);
		if (rmc->type != LIVING_CONNECTOR) return false;
	}

	return true;
}

bool RMC::pickPin(Ray& ray)
{
	pickedPin = NULL;
	for (uint i = 0; i < pins.size(); i++)
	{
		if (pins[i].idle && pins[i].isPicked(ray))
		{
			pickedPin = &pins[i];
			return true;
		}
	}

	return false;
}

bool RMC::pickMesh(Ray& ray, double* closestDist)
{
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return meshes[0]->getDistanceToRayOriginIfHit(newRay, closestDist);
}

void RMC::draw(int flags, const Vector4d& color)
{
	glEnable(GL_LIGHTING);
	if (flags & SHOW_PINS)
		for (uint i = 0; i < pins.size(); i++) {
			if ((&pins[i]) == pickedPin)
				pins[i].draw(V3D(1, 0, 0));
			else
				pins[i].draw(V3D(0, 1, 1));
		}

	if (color.isZero())
	{	
		meshes[0]->setMaterial(material);
	}
	else {
		GLShaderMaterial colorMat;
		colorMat.setColor(color[0], color[1], color[2], color[3]);
		meshes[0]->setMaterial(colorMat);
	}

	RigidBody::draw(flags);
}

void RMC::loadFromFile(FILE* fp)
{
	char buffer[200];
	char keyword[50];
	char content[200];

	RMCBulletObject* bulletObject = NULL;

	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		sscanf(line, "%s", keyword);
		Logger::print("%s ", keyword);

		if (strcmp(keyword, "Name") == 0)
		{
			sscanf(line + strlen(keyword), "%s", content);
			name = content;
			Logger::print("%s", name.c_str());
		}
		else if (strcmp(keyword, "Plate") == 0)
		{
			type = PLATE_RMC;
		}
		else if (strcmp(keyword, "HornBracket") == 0)
		{
			type = HORN_BRACKET_RMC;
		}
		else if (strcmp(keyword, "Bracket") == 0)
		{
			type = BRACKET_RMC;
		}
		else if (strcmp(keyword, "MotorAxis") == 0)
		{
			if (type != LIVING_MOTOR)
				type = MOTOR_RMC;
			sscanf(line + strlen(keyword), "%lf %lf %lf", &motorAxis[0], &motorAxis[1], &motorAxis[2]);
			Logger::print("%lf %lf %lf", motorAxis[0], motorAxis[1], motorAxis[2]);
		}
		else if (strcmp(keyword, "Connector") == 0)
		{
			type = CONNECTOR_RMC;
		}
		else if (strcmp(keyword, "EE") == 0)
		{
			type = EE_RMC;
		}
		else if (strcmp(keyword, "EEHorn") == 0)
		{
			type = EE_HORN_RMC;
		}
		else if (strcmp(keyword, "EndEffector") == 0)
		{
			type = EE_RMC;
			P3D ee;
			sscanf(line + strlen(keyword), "%lf %lf %lf", &ee[0], &ee[1], &ee[2]);
			rbProperties.endEffectorPoints.push_back(ee);
			Logger::print("%lf %lf %lf", ee[0], ee[1], ee[2]);
		}
		else if (strcmp(keyword, "FeaturePoint") == 0)
		{
			P3D FP;
			sscanf(line + strlen(keyword), "%lf %lf %lf", &FP[0], &FP[1], &FP[2]);
			rbProperties.bodyPointFeatures.push_back(FP);
			Logger::print("%lf %lf %lf", FP[0], FP[1], FP[2]);
		}
		else if (strcmp(keyword, "Mass") == 0)
		{
			sscanf(line + strlen(keyword), "%lf", &rbProperties.mass);
			Logger::print("%lf", rbProperties.mass);
		}
		else if (strcmp(keyword, "MOI") == 0)
		{
			Matrix3x3& MOI = rbProperties.MOI_local;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf %lf %lf %lf", &MOI(0, 0), &MOI(1, 1), &MOI(2, 2)
				, &MOI(0, 1), &MOI(0, 2), &MOI(1, 2));
			if (num < 6)
				throwError("Not enough transformation parameters!");

			for (int i = 0; i < 9; i++)
				Logger::print("%lf ", rbProperties.MOI_local(i / 3, i % 3));
		}
		else if (strcmp(keyword, "Mesh") == 0)
		{
			sscanf(line + strlen(keyword), "%s", content);
			GLMesh* mesh = GLContentManager::getGLMesh(content);
			mesh->calBoundingBox();
//			mesh->translate(-mesh->getCenterOfMass());
			mesh->getMaterial().setColor(1.0, 1.0, 1.0, 0.8);
			meshes.push_back(mesh);
		}
		else if (strcmp(keyword, "Material") == 0)
		{
			sscanf(line + strlen(keyword), "%s", content);
			material.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
			material.setTextureParam(content, GLContentManager::getTexture(content));
		}
		else if (strcmp(keyword, "Pin") == 0)
		{
			pins.push_back(RMCPin(this, (int)pins.size()));
			Logger::print("\n");
			pins.back().loadFromFile(fp);
		}
		else if (strcmp(keyword, "Box") == 0)
		{
			bulletCollisionObjects.push_back(new RMCBulletObject(this));
			bulletCollisionObjects.back()->loadFromFile(fp);
		}
		else if (strcmp(keyword, "EndRMC") == 0)
		{
			Logger::print("\n");
			break;
		}
		Logger::print("\n");
	}
}

void RMC::addBulletObjectsToList(DynamicArray<AbstractBulletObject*>& list) {
	for (uint i = 0; i < bulletCollisionObjects.size(); i++) {
		list.push_back(bulletCollisionObjects[i]);
	}
}