#include <RobotDesignerLib/LivingMotor.h>
#include <GUILib/GLApplication.h>
//#include <MathLib/MeshBoolean.h>


LivingMotor::LivingMotor(const char* LMType){
	this->LMType = string(LMType);

	if (strcmp(trim((char*)LMType), "TGY306G")) {
		bodyBracket = new LivingMotorBodyBracket_XM430();
		hornBracket = new LivingHornBracket_XM430();

		motorBodyMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_parent.obj"); motorBodyMesh->getMaterial().setColor(0.15, 0.15, 0.15, 1.0);
		motorHornMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_child.obj"); motorHornMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

		bodyMaterial.setColor(0.15, 0.15, 0.15, 1.0);
		hornMaterial.setColor(0.7, 0.7, 0.7, 1.0);
	}
	else {
		//default for XM430 motors
		bodyBracket = new LivingMotorBodyBracket_XM430();
		hornBracket = new LivingHornBracket_XM430();

		motorBodyMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_parent.obj"); motorBodyMesh->getMaterial().setColor(0.15, 0.15, 0.15, 1.0);
		motorHornMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_child.obj"); motorHornMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

		bodyMaterial.setColor(0.15, 0.15, 0.15, 1.0);
		hornMaterial.setColor(0.7, 0.7, 0.7, 1.0);

	}

	type = LIVING_MOTOR;

	generatePins();
}

LivingMotor::~LivingMotor(){
	delete bodyBracket;
	delete hornBracket;
}

void LivingMotor::setColor(const Vector4d& color/* = Vector4d(0, 0, 0, 0)*/) {
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


LivingMotor* LivingMotor::clone(){
	LivingMotor* new_rmc = new LivingMotor(LMType.c_str());

	new_rmc->hornBracket->copyBracketProperties(this->hornBracket, false);

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

bool LivingMotor::pickMesh(Ray& ray, double* closestDist /*= NULL*/){
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return motorBodyMesh->getDistanceToRayOriginIfHit(newRay, closestDist)
		|| bodyBracket->bodyBracketMesh->getDistanceToRayOriginIfHit(newRay, closestDist)
		|| hornBracket->bracketMesh->getDistanceToRayOriginIfHit(newRay, closestDist);
}

void LivingMotor::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (flags & SHOW_PINS)
		for (uint i = 0; i < pins.size(); i++) {
			if ((&pins[i]) == pickedPin)
				pins[i].draw(V3D(1, 0, 0));
			else
				pins[i].draw(V3D(0, 1, 1));
		}

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

void LivingMotor::update(){
	hornBracket->generateBracketMesh();
	bodyBracket->generateBracketMesh();
	for (auto& pin : pins)
	{
		if (pin.livingType == LIVING_HORN_PIN)
		{
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

void LivingMotor::generatePins()
{
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

void LivingMotor::exportMeshes(const char* dirName, int index){
	//export only those meshes that are somehow custom-made... stock meshes need not be processed here...

	// *************************** Horn Bracket Mesh ***************************
	string bracketFileName = dirName + string("LivingBracketMesh") + to_string(index) + string(".obj");

	hornBracket->bracketMesh->path = bracketFileName;
	GLContentManager::addMeshFileMapping(hornBracket->bracketMesh, bracketFileName.c_str());
	hornBracket->bracketMesh->writeTriangulatedMeshToObj(bracketFileName.c_str());
}

void LivingMotor::syncSymmParameters(LivingMotor* refMotor){
	hornBracket->copyBracketProperties(refMotor->hornBracket, true);
}

void LivingMotor::switchToBestBodyPin(){
	RMCPin* bodyPin = &pins[1];
	if (bodyPin->idle) return;

	double maxCost = DBL_MAX;
	int bestPinID = -1;
	RMCPin* bestPin = NULL;
	RMCPin* connectedPin = bodyPin->getConnectedPin();
	RMC* connectedRMC = connectedPin->rmc;
	if (connectedRMC->type != LIVING_CONNECTOR) return;

	RMCPin* oppositePin = NULL;
	for (auto& pin : connectedRMC->pins)
	{
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

