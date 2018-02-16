#include <RobotDesignerLib/LivingEE.h>
#include <GUILib/GLContentManager.h>
#include <GUILib/GLUtils.h>

SphereEE_RMC::SphereEE_RMC(){
	name = "SphereEE";

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

SphereEE_RMC::~SphereEE_RMC(){
	delete eeMesh;
}

SphereEE_RMC* SphereEE_RMC::clone(){
	SphereEE_RMC* new_rmc = new SphereEE_RMC();
	new_rmc->sphereRadius = sphereRadius;
	copyBasePropertiesTo(new_rmc, false);

	for (uint i = 0; i < pins.size(); i++)
		new_rmc->pins[i].compatibleMap = pins[i].compatibleMap;
	new_rmc->update();
	return new_rmc;
}

bool SphereEE_RMC::pickMesh(Ray& ray, double* closestDist /*= NULL*/){
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));
	P3D p;
	double dist = newRay.getDistanceToPoint(P3D(), &p);
	bool res = dist < sphereRadius;

	if (res && closestDist)
		*closestDist = (p - ray.origin).norm();

	return res;
}

void SphereEE_RMC::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (flags & SHOW_PINS)
		drawPins();

	if (flags & SHOW_MESH) {
		if (color.isZero())
			eeMesh->setMaterial(material);
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

void SphereEE_RMC::update(){
	delete eeMesh;
	eeMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-sphere.obj")->clone();
	//now scale it according to the radius...
	for (int i = 0; i < eeMesh->getVertexCount() * 3; i++)
		eeMesh->getVertexArray()[i] *= sphereRadius;

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

void SphereEE_RMC::syncSymmParameters(RMC* refEE){
	SphereEE_RMC* sEE = dynamic_cast<SphereEE_RMC*>(refEE);
	if (sEE)
		sphereRadius = sEE->sphereRadius;
}

void SphereEE_RMC::exportMeshes(const char* dirName, int index){
	string sphereEEFileName = dirName + string("LivingSphereEEMesh") + to_string(index) + string(".obj");

	eeMesh->path = sphereEEFileName;
	eeMesh->writeTriangulatedMeshToObj(sphereEEFileName.c_str());
	GLMesh* nMesh = eeMesh->clone();
	nMesh->setMaterial(material);
	GLContentManager::addMeshFileMapping(nMesh, sphereEEFileName.c_str());
}

/** -------------------------------------------------------------------------------- **/
WheelEE_RMC::WheelEE_RMC(const char* LMType){
	this->LMType = string(trim((char*)LMType));
	type = LIVING_WHEEL_EE;
	name = this->LMType;

	if (strcmp(trim((char*)LMType), "ActiveWheel_20gServos") == 0) {
		originalWheelMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-wheel3.obj");
		isActive = true;
		motorMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_wheelMotor.obj");
		bodyMaterial.setColor(0.15, 0.15, 0.15, 1.0);
		motorBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_wheelMotorBracket_w.obj");
	}
	if (strcmp(trim((char*)LMType), "ActiveWheel_XM430") == 0) {
		originalWheelMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-wheel3.obj");
		isActive = true;
		motorMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_wheelMotor.obj");
		bodyMaterial.setColor(0.15, 0.15, 0.15, 1.0);
		motorBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_wheelMotorBracket_w.obj");
	}
	else if (strcmp(trim((char*)LMType), "PassiveWheel") == 0) {
		originalWheelMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-wheelPassive_w.obj");
		// uncomment the following line to show passive wheel brackets
		// motorBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-passiveWheelBracket_w.obj");
		isActive = false;
	}
	else {
		originalWheelMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/3DP-wheel3.obj");
		radius = 0.05;
		isActive = true;
		name = "GenericWheel";
	}

	rbProperties.addEndEffectorPoint(P3D(0, 0, 0), radius);
	update();
}


WheelEE_RMC::~WheelEE_RMC(){
	delete wheelMesh;
}

WheelEE_RMC* WheelEE_RMC::clone(){
	WheelEE_RMC* new_rmc = new WheelEE_RMC(LMType.c_str());
	new_rmc->radius = radius;
	copyBasePropertiesTo(new_rmc, true);

	for (uint i = 0; i < pins.size(); i++)
		new_rmc->pins[i].compatibleMap = pins[i].compatibleMap;
	new_rmc->update();
	return new_rmc;
}

bool WheelEE_RMC::pickMesh(Ray& ray, double* closestDist /*= NULL*/){
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return (wheelMesh->getDistanceToRayOriginIfHit(newRay, closestDist)
		|| (motorMesh && motorMesh->getDistanceToRayOriginIfHit(newRay, closestDist))
		|| (motorBracketMesh && motorBracketMesh->getDistanceToRayOriginIfHit(newRay, closestDist))
		);

}

void WheelEE_RMC::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (flags & SHOW_PINS)
		drawPins();

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

void WheelEE_RMC::update(){
	// update ee mesh
	delete wheelMesh;
	wheelMesh = originalWheelMesh->clone();
	//now scale it according to the radius...
	for (int i = 0; i < wheelMesh->getVertexCount() * 3; i++) {
		wheelMesh->getVertexArray()[i] *= radius;
	}
	wheelMesh->calBoundingBox();
}

void WheelEE_RMC::syncSymmParameters(RMC* refEE){
	WheelEE_RMC* wEE = dynamic_cast<WheelEE_RMC*>(refEE);
	if (wEE)
		radius = wEE->radius;
}

void WheelEE_RMC::exportMeshes(const char* dirName, int index){
	string wheelEEFileName = dirName + string("LivingWheelEEMesh") + to_string(index) + string(".obj");

	wheelMesh->path = wheelEEFileName;
	wheelMesh->writeTriangulatedMeshToObj(wheelEEFileName.c_str());
	GLMesh* nMesh = wheelMesh->clone();
	nMesh->setMaterial(material);
	GLContentManager::addMeshFileMapping(nMesh, wheelEEFileName.c_str());
}
