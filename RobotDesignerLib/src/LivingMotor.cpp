#include <RobotDesignerLib/LivingMotor.h>
#include <GUILib/GLApplication.h>
//#include <MathLib/MeshBoolean.h>

Motor_RMC::Motor_RMC(const char* LMType){
	this->LMType = string(LMType);
	name = "Motor";

	if (strcmp(trim((char*)LMType), "TGY306G") == 0) {
		bodyBracket = new Motor_RMC_BodyBracket_TGY306G();
		hornBracket = new Motor_RMC_HornBracket_TGY306G();

		motorBodyMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_parent.obj"); motorBodyMesh->getMaterial().setColor(0.15, 0.15, 0.15, 1.0);
		motorHornMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_child.obj"); motorHornMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

		bodyMaterial.setColor(0.15, 0.15, 0.15, 1.0);
		hornMaterial.setColor(0.7, 0.7, 0.7, 1.0);
	} else if (strcmp(trim((char*)LMType), "BK3002") == 0) {
		bodyBracket = new Motor_RMC_BodyBracket_BK3002();
		hornBracket = new Motor_RMC_HornBracket_BK3002();

		motorBodyMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_parent.obj");

		bodyMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
		string mat = "../data/textures/matcap/red_specular.bmp";
		bodyMaterial.setTextureParam(mat.c_str(), GLContentManager::getTexture(mat.c_str()));
		motorBodyMesh->setMaterial(bodyMaterial);

		motorHornMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_child.obj");
		hornMaterial.setColor(0.7, 0.7, 0.7, 1.0);
	}
	else {
		//default for XM430 motors
		bodyBracket = new Motor_RMC_BodyBracket_XM430();
		hornBracket = new Motor_RMC_HornBracket_XM430();

//		motorBodyMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_parent.obj"); motorBodyMesh->getMaterial().setColor(0.15, 0.15, 0.15, 1.0);
//		motorHornMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_child.obj"); motorHornMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

		motorBodyMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_parent_lo.obj"); motorBodyMesh->getMaterial().setColor(0.15, 0.15, 0.15, 1.0);
		motorHornMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_child_lo.obj"); motorHornMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

		bodyMaterial.setColor(0.15, 0.15, 0.15, 1.0);
		hornMaterial.setColor(0.7, 0.7, 0.7, 1.0);

	}

	type = MOTOR_RMC;

	generatePins();
}

Motor_RMC::~Motor_RMC(){
	delete bodyBracket;
	delete hornBracket;
}

void Motor_RMC::setColor(const Vector4d& color/* = Vector4d(0, 0, 0, 0)*/) {
	bodyBracket->setColor(color);
	hornBracket->setColor(color);

	if (color.isZero()) {
		motorBodyMesh->setMaterial(bodyMaterial);
		motorHornMesh->setMaterial(hornMaterial);
	}
	else {
		GLShaderMaterial defaultMat;
		defaultMat.setColor(color[0], color[1], color[2], color[3]);
		motorBodyMesh->setMaterial(defaultMat);
		motorHornMesh->setMaterial(defaultMat);
	}
}

Motor_RMC* Motor_RMC::clone(){
	Motor_RMC* new_rmc = new Motor_RMC(LMType.c_str());
	copyBasePropertiesTo(new_rmc, false);
	new_rmc->hornBracket->copyBracketProperties(this->hornBracket, false);

	new_rmc->motorAxis = motorAxis;
	new_rmc->motorAngle = motorAngle;

	for (uint i = 0; i < pins.size(); i++)
		new_rmc->pins[i].compatibleMap = pins[i].compatibleMap;
	new_rmc->update();
	return new_rmc;
}

bool Motor_RMC::pickMesh(Ray& ray, double* closestDist /*= NULL*/){
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return motorBodyMesh->getDistanceToRayOriginIfHit(newRay, closestDist)
		|| bodyBracket->bodyBracketMesh->getDistanceToRayOriginIfHit(newRay, closestDist)
		|| hornBracket->bracketMesh->getDistanceToRayOriginIfHit(newRay, closestDist);
}

void Motor_RMC::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (flags & SHOW_PINS)
		drawPins();

	if (flags & SHOW_MESH){
		setColor(color);

		glPushMatrix();
		glTranslated(state.position[0], state.position[1], state.position[2]);
		//and rotation part
		V3D rotAxis; double rotAngle;
		state.orientation.getAxisAngle(rotAxis, rotAngle);
		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);

		glEnable(GL_LIGHTING);

		// ******************* draw motor and motor body bracket *******************

		bodyBracket->draw();

		if (motorBodyMesh)
			motorBodyMesh->drawMesh();


		// ******************* draw horn and horn bracket *******************
		glPushMatrix();
		glRotated(motorAngle, motorAxis[0], motorAxis[1], motorAxis[2]);

		if (motorHornMesh)
			motorHornMesh->drawMesh();

		if (hornBracket)
			hornBracket->draw();

		glPopMatrix();

		glPopMatrix();
	}

	return;
}

void Motor_RMC::update(){
	hornBracket->generateBracketMesh();
	bodyBracket->generateBracketMesh();
	for (auto& pin : pins){
		if (pin.livingType == LIVING_HORN_PIN){
			Transformation motorRotTrans(getRotationQuaternion(RAD(motorAngle), motorAxis).getRotationMatrix());

			pin.transformation = motorRotTrans * hornBracket->getPinTransformation();
			pin.face.vertices = hornBracket->featurePoints;
			for (auto& v : pin.face.vertices)
				v = motorRotTrans.transform(v);
			pin.face.center = motorRotTrans.transform(hornBracket->pinPosition);
			pin.face.normal = motorRotTrans.transform(hornBracket->pinOrientation.rotate(V3D(0, 1, 0)));
		}
	}

	switchToBestBodyPin();
}

void Motor_RMC::generatePins(){
	{
		RMCPin livingPin(this, hornBracket->getPinTransformation(), pins.size());
		livingPin.type = HORN_PIN;
		livingPin.livingType = LIVING_HORN_PIN;
		livingPin.name = "LivingHornBracketPin";
		livingPin.face.vertices = hornBracket->featurePoints;
		livingPin.face.center = hornBracket->pinPosition;
		livingPin.face.normal = hornBracket->pinOrientation.rotate(V3D(0, 1, 0));
		pins.push_back(livingPin);
	}

	for (int i = 0; i < (int)bodyBracket->pinInfos.size(); i++)
	{
		PinInfo& pinInfo = bodyBracket->pinInfos[i];
		RMCPin bracketPin(this, pinInfo.trans, pins.size());
		bracketPin.livingType = LIVING_BODY_PIN;
		bracketPin.name = "LivingBodyBracketPin";
		bracketPin.face.vertices = pinInfo.featurePoints;
		bracketPin.face.center = pinInfo.center;
		bracketPin.face.normal = pinInfo.normal;
		candidatePins.push_back(bracketPin);
	}

	pins.push_back(candidatePins[0]);
	activeBodyPinID = 0;
}

void Motor_RMC::exportMeshes(const char* dirName, int index){
	//export only those meshes that are somehow custom-made... stock meshes need not be processed here...

	// *************************** Horn Bracket Mesh ***************************
	string bracketFileName = dirName + string("LivingBracketMesh") + to_string(index) + string(".obj");

	//tmpMesh will from now on be managed by the content manager...
	hornBracket->bracketMesh->path = bracketFileName;
	GLMesh* tmpMesh = hornBracket->bracketMesh->clone();
	tmpMesh->writeTriangulatedMeshToObj(bracketFileName.c_str());
	GLContentManager::addMeshFileMapping(tmpMesh, bracketFileName.c_str());
}

void Motor_RMC::syncSymmParameters(RMC* refMotor){
	Motor_RMC* motor = dynamic_cast<Motor_RMC*>(refMotor);
	if (motor) {
		hornBracket->copyBracketProperties(motor->hornBracket, true);
		motorAngle = -motor->motorAngle;
	}
}

//TODO: this needs to also have different types of body brackets that can be instantiated... then it would make a lot of sense...
void Motor_RMC::switchToBestBodyPin(){
	RMCPin* bodyPin = &pins[1];
	if (bodyPin->idle) return;

	double maxCost = DBL_MAX;
	int bestPinID = -1;
	RMCPin* bestPin = NULL;
	RMCPin* connectedPin = bodyPin->getConnectedPin();
	RMC* connectedRMC = connectedPin->rmc;
	if (connectedRMC->type != LIVING_CONNECTOR) return;

	RMCPin* oppositePin = NULL;
	for (auto& pin : connectedRMC->pins){
		if ((&pin) != connectedPin && !pin.idle)
			oppositePin = pin.getConnectedPin();
	}
	if (!oppositePin) return;

	for (int i = 0; i < (int)candidatePins.size(); i++)
	{
		RMCPin& candPin = candidatePins[i];
		PinFace& face1 = candPin.face;
		PinFace& face2 = oppositePin->face;
		RMC* rmc1 = candPin.rmc;
		RMC* rmc2 = oppositePin->rmc;
		V3D centerVec = (rmc2->getWorldCoordinates(face2.center) - rmc1->getWorldCoordinates(face1.center)).normalized();
		double cost = rmc2->getWorldCoordinates(face2.normal).dot(centerVec) - rmc1->getWorldCoordinates(face1.normal).dot(centerVec);
		
		if (cost < maxCost)
		{
			maxCost = cost;
			bestPin = &candPin;
			bestPinID = i;
		}
	}

	bodyPin->face = bestPin->face;
	bodyPin->mesh = bestPin->mesh;
	bodyPin->transformation = bestPin->transformation;
	activeBodyPinID = bestPinID;
}

