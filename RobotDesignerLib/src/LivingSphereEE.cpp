#include <RobotDesignerLib/LivingSphereEE.h>
#include <GUILib/GLContentManager.h>
#include <GUILib/GLUtils.h>

LivingSphereEE::LivingSphereEE()
{
	type = LIVING_SPHERE_EE;
	{
		RMCPin pin(this, Transformation(), pins.size());
		pin.type = NORMAL_PIN;
		pin.livingType = LIVING_BODY_PIN;
		pin.name = "LivingSphereEE-pin1";
		pins.push_back(pin);
	}

	update();
}


LivingSphereEE::~LivingSphereEE()
{
	delete eeMesh;
}

LivingSphereEE* LivingSphereEE::clone()
{
	LivingSphereEE* new_rmc = new LivingSphereEE();
	new_rmc->sphereRadius = sphereRadius;
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

	// for living connector, the pins are generated by itself.
	/*new_rmc->pins = pins;
	for (uint i = 0; i < pins.size(); i++)
	new_rmc->pins[i].rmc = new_rmc;*/

	for (uint i = 0; i < pins.size(); i++)
	{
		new_rmc->pins[i].compatibleMap = pins[i].compatibleMap;
	}

	for (uint i = 0; i < bulletCollisionObjects.size(); i++) {
		new_rmc->bulletCollisionObjects.push_back(bulletCollisionObjects[i]->clone());
		new_rmc->bulletCollisionObjects.back()->parent = new_rmc;
	}

	return new_rmc;
}

bool LivingSphereEE::pickMesh(Ray& ray, double* closestDist /*= NULL*/)
{
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));
	P3D p;
	double dist = newRay.getDistanceToPoint(P3D(), &p);
	bool res = dist < sphereRadius;

	if (res && closestDist) {
		*closestDist = (p - ray.origin).norm();
	}

	return res;
}

void LivingSphereEE::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/)
{
	if (flags & SHOW_PINS)
		for (uint i = 0; i < pins.size(); i++) {
			if ((&pins[i]) == pickedPin)
				pins[i].draw(V3D(1, 0, 0));
			else
				pins[i].draw(V3D(0, 1, 1));
		}

	if (flags & SHOW_MESH) {
		if (color.isZero())
		{
			eeMesh->setMaterial(material);
		}
		else {
			GLShaderMaterial colorMat;
			colorMat.setColor(color[0], color[1], color[2], color[3]);
			eeMesh->setMaterial(colorMat);
		}

		glEnable(GL_LIGHTING);
		glPushMatrix();
		glTranslated(state.position[0], state.position[1], state.position[2]);
		//and rotation part
		V3D rotAxis; double rotAngle;
		state.orientation.getAxisAngle(rotAxis, rotAngle);
		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);

		eeMesh->drawMesh();
		glPopMatrix();
	}

	return;
}

void LivingSphereEE::update()
{
	// update ee mesh
	delete eeMesh;
	eeMesh = new GLMesh();
	eeMesh->addSphere(P3D(), sphereRadius, 16);
	eeMesh->computeNormals();
	eeMesh->calBoundingBox();

	// update pins
	auto& pin = pins[0];
	pin.transformation = Transformation();
	pin.face.center = P3D();
	pin.face.normal = V3D(0, 1, 0);
	pin.face.vertices.clear();
	int n = 12;
	for (int i = 0; i < n; i++)
	{
		double theta = i * 2 * PI / n;
		pin.face.vertices.push_back(P3D(sphereRadius * cos(theta), 0, sphereRadius * sin(theta)));
	}

	// update EE
	rbProperties.endEffectorPoints.clear();
	rbProperties.addEndEffectorPoint(P3D(0, 0, 0), 0.01);
}

void LivingSphereEE::syncSymmParameters(LivingSphereEE* refEE)
{
	sphereRadius = refEE->sphereRadius;
}

void LivingSphereEE::exportMeshes(const char* dirName, int index)
{
	string sphereEEFileName = dirName + string("LivingSphereEEMesh") + to_string(index) + string(".obj");

	eeMesh->path = sphereEEFileName;
	eeMesh->writeTriangulatedMeshToObj(sphereEEFileName.c_str());
	GLMesh* nMesh = eeMesh->clone();
	nMesh->setMaterial(material);
	GLContentManager::addMeshFileMapping(nMesh, sphereEEFileName.c_str());
}


/** -------------------------------------------------------------------------------- **/
LivingWheelEE::LivingWheelEE(const char* LMType){
	this->LMType = string(LMType);
	type = LIVING_WHEEL_EE;

	if (strcmp(trim((char*)LMType), "TGY306GActiveWheel") == 0) {
		originalWheelMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-wheel3.obj");
		isActive = true;
		motorMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_wheelMotor.obj");
		bodyMaterial.setColor(0.15, 0.15, 0.15, 1.0);
		motorBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_wheelMotorBracket_w.obj");

		radius = 0.03;
	}
	else if (strcmp(trim((char*)LMType), "TGY306GPassiveWheel") == 0) {
		originalWheelMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-wheelPassive_w.obj");
		motorBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-passiveWheelBracket_w.obj");
		radius = 0.03;
		isActive = false;
	} 
	else {
		originalWheelMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-wheel3.obj");
		radius = 0.05;
		isActive = true;
	}

	rbProperties.addEndEffectorPoint(P3D(0, 0, 0), radius);
	update();
}


LivingWheelEE::~LivingWheelEE()
{
	delete wheelMesh;
}

LivingWheelEE* LivingWheelEE::clone(){
	LivingWheelEE* new_rmc = new LivingWheelEE(LMType.c_str());
	new_rmc->radius = radius;
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

	for (uint i = 0; i < pins.size(); i++)
	{
		new_rmc->pins[i].compatibleMap = pins[i].compatibleMap;
	}

	for (uint i = 0; i < bulletCollisionObjects.size(); i++) {
		new_rmc->bulletCollisionObjects.push_back(bulletCollisionObjects[i]->clone());
		new_rmc->bulletCollisionObjects.back()->parent = new_rmc;
	}

	return new_rmc;
}

bool LivingWheelEE::pickMesh(Ray& ray, double* closestDist /*= NULL*/){
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return (wheelMesh->getDistanceToRayOriginIfHit(newRay, closestDist)
		|| (motorMesh && motorMesh->getDistanceToRayOriginIfHit(newRay, closestDist))
		|| (motorBracketMesh && motorBracketMesh->getDistanceToRayOriginIfHit(newRay, closestDist))
		);

}

void LivingWheelEE::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/)
{
	if (flags & SHOW_PINS)
		for (uint i = 0; i < pins.size(); i++) {
			if ((&pins[i]) == pickedPin)
				pins[i].draw(V3D(1, 0, 0));
			else
				pins[i].draw(V3D(0, 1, 1));
		}

	if (flags & SHOW_MESH) {
		if (color.isZero())
		{
			wheelMesh->setMaterial(material);
			if (motorMesh)
				motorMesh->setMaterial(bodyMaterial);
			if (motorBracketMesh)
				motorBracketMesh->setMaterial(material);
		}
		else {
			GLShaderMaterial colorMat;
			colorMat.setColor(color[0], color[1], color[2], color[3]);
			wheelMesh->setMaterial(colorMat);
			if (motorMesh)
				motorMesh->setMaterial(colorMat);
			if (motorBracketMesh)
				motorBracketMesh->setMaterial(colorMat);
		}

		glEnable(GL_LIGHTING);
		glPushMatrix();
		glTranslated(state.position[0], state.position[1], state.position[2]);
		//and rotation part
		V3D rotAxis; double rotAngle;
		state.orientation.getAxisAngle(rotAxis, rotAngle);
		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);

		wheelMesh->drawMesh();

		if (motorMesh)
			motorMesh->drawMesh();
		if (motorBracketMesh)
			motorBracketMesh->drawMesh();

		glPopMatrix();
	}

	return;
}

void LivingWheelEE::update(){
	// update ee mesh
	delete wheelMesh;
	wheelMesh = originalWheelMesh->clone();
	//now scale it according to the radius...
	for (int i = 0; i < wheelMesh->getVertexCount() * 3; i++) {
		wheelMesh->getVertexArray()[i] *= radius;
	}
	wheelMesh->calBoundingBox();
}

void LivingWheelEE::syncSymmParameters(LivingWheelEE* refWEE)
{
	radius = refWEE->radius;
}

void LivingWheelEE::exportMeshes(const char* dirName, int index)
{
	string wheelEEFileName = dirName + string("LivingWheelEEMesh") + to_string(index) + string(".obj");

	wheelMesh->path = wheelEEFileName;
	wheelMesh->writeTriangulatedMeshToObj(wheelEEFileName.c_str());
	GLMesh* nMesh = wheelMesh->clone();
	nMesh->setMaterial(material);
	GLContentManager::addMeshFileMapping(nMesh, wheelEEFileName.c_str());
}
