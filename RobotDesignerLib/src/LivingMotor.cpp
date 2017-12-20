#include <RobotDesignerLib/LivingMotor.h>
#include <GUILib/GLApplication.h>
//#include <MathLib/MeshBoolean.h>


LivingMotor::LivingMotor(LivingHornBracket* lbh)
{
	motor = new LivingBracketMotor_XM430();
	bracket = new LivingHornBracket_XM430(motor, lbh);

	type = LIVING_MOTOR;

	generatePins();
}

LivingMotor::~LivingMotor()
{
	delete bracket;
	delete motor;
}

LivingMotor* LivingMotor::clone()
{
	LivingMotor* new_rmc = new LivingMotor(bracket);
	new_rmc->state = state;
	new_rmc->rbProperties = rbProperties;

	new_rmc->meshes = meshes;
	new_rmc->name = name;
	new_rmc->id = id;
	new_rmc->type = type;
	new_rmc->motorAxis = motorAxis;
	new_rmc->motorAngle = motorAngle;
	new_rmc->carveMesh = carveMesh;
	new_rmc->carveMeshEx = carveMeshEx;

	new_rmc->material = material;

	new_rmc->mappingInfo = mappingInfo;

	// shouldn't copy these old pointers.
	//new_rmc->cJoints = cJoints;
	//new_rmc->pJoints = pJoints;

	// for living motor, the pins are generated by itself
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

bool LivingMotor::pickMesh(Ray& ray, double* closestDist /*= NULL*/)
{
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return motor->motorBodyMesh->getDistanceToRayOriginIfHit(newRay, closestDist)
		|| bracket->bracketMesh->getDistanceToRayOriginIfHit(newRay, closestDist);
}

void LivingMotor::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/)
{
	if (flags & SHOW_PINS)
		for (uint i = 0; i < pins.size(); i++) {
			if ((&pins[i]) == pickedPin)
				pins[i].draw(V3D(1, 0, 0));
			else
				pins[i].draw(V3D(0, 1, 1));
		}

	if (flags & SHOW_MESH)
	{
		bracket->setColor(color);
		motor->setColor(color);

		glPushMatrix();
		glTranslated(state.position[0], state.position[1], state.position[2]);
		//and rotation part
		V3D rotAxis; double rotAngle;
		state.orientation.getAxisAngle(rotAxis, rotAngle);
		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);

		motor->draw();

		// ******************* draw bracket *******************
		glPushMatrix();
		glRotated(motorAngle, motorAxis[0], motorAxis[1], motorAxis[2]);
		bracket->draw();
		glPopMatrix();

		glPopMatrix();
	}

	return;
}

void LivingMotor::update()
{
	bracket->generateBracketMesh();
	for (auto& pin : pins)
	{
		if (pin.livingType == LIVING_HORN_PIN)
		{
			Transformation motorRotTrans(getRotationQuaternion(RAD(motorAngle), motorAxis).getRotationMatrix());

			pin.transformation = motorRotTrans * bracket->getPinTransformation();
			pin.face.vertices = bracket->featurePoints;
			for (auto& v : pin.face.vertices)
				v = motorRotTrans.transform(v);
			pin.face.center = motorRotTrans.transform(bracket->pinPosition);
			pin.face.normal = motorRotTrans.transform(bracket->pinOrientation.rotate(V3D(0, 1, 0)));
		}
	}

	switchToBestBodyPin();
}

void LivingMotor::generatePins()
{
	{
		RMCPin livingPin(this, bracket->getPinTransformation(), pins.size());
		livingPin.type = HORN_PIN;
		livingPin.livingType = LIVING_HORN_PIN;
		livingPin.name = "LivingHornBracketPin";
		livingPin.face.vertices = bracket->featurePoints;
		livingPin.face.center = bracket->pinPosition;
		livingPin.face.normal = bracket->pinOrientation.rotate(V3D(0, 1, 0));
		pins.push_back(livingPin);
	}

	for (int i = 0; i < (int)motor->pinInfos.size(); i++)
	{
		PinInfo& pinInfo = motor->pinInfos[i];
		RMCPin bracketPin(this, pinInfo.trans, pins.size());
		bracketPin.livingType = LIVING_BODY_PIN;
		bracketPin.name = "LivingBodyBracketPin";
		bracketPin.face.vertices = pinInfo.featurePoints;
		bracketPin.face.center = pinInfo.center;
		bracketPin.face.normal = pinInfo.normal;
		bracketPin.mesh = motor->bracketCarvingMeshes[i];
		candidatePins.push_back(bracketPin);
	}

	pins.push_back(candidatePins[0]);
	activeBodyPinID = 0;
}

void LivingMotor::exportMeshes(const char* dirName, int index, bool mergeMeshes)
{
	// *************************** Horn Bracket Mesh ***************************
	string bracketFileName = dirName + string("LivingBracketMesh") + to_string(index) + string(".obj");
	if (mergeMeshes)
	{
            // This was disabled because MeshBoolean.exe is for Windows only
            // If this is needed, we can re-enable it.
            throw std::runtime_error("This functionality is not available.");
//            GLMesh* resMesh = bracket->bridgeMesh->clone();
//            meshBooleanIntrusive(resMesh, bracket->leftSideMesh, "Union");
//            meshBooleanIntrusive(resMesh, bracket->rightSideMesh, "Union");
//            bracket->outputMesh = resMesh;
        }
	else {
		bracket->outputMesh = bracket->bracketMesh->clone();
	}

	bracket->outputMesh->path = bracketFileName;
	GLContentManager::addMeshFileMapping(bracket->outputMesh, bracketFileName.c_str());
	bracket->outputMesh->writeTriangulatedMeshToObj(bracketFileName.c_str());

	// ******************* Body Carving Mesh *******************
	if (mergeMeshes)
	{
		string bodyCarvingMeshName = dirName + string("LivingBodyCarvingMesh") + to_string(index) + string(".obj");
		GLMesh* bodyCarvingMesh = new GLMesh();
		RMCPin* bodyPin = &pins[1];

		// int noholePin = bodyPin->idle ? 0 : activeBodyPinID;
		int noholePin = 0;

		for (int i = 0; i < (int)candidatePins.size(); i++)
		{
			RMCPin& candPin = candidatePins[i];
			if (i == noholePin || !candPin.mesh) continue;	
			bodyCarvingMesh->append(candPin.mesh);
		}
		
		bodyCarvingMesh->path = bodyCarvingMeshName;
		GLContentManager::addMeshFileMapping(bodyCarvingMesh, bodyCarvingMeshName.c_str());
		bodyCarvingMesh->writeTriangulatedMeshToObj(bodyCarvingMeshName.c_str());
		motor->bodyCarvingMesh = bodyCarvingMesh;
	}
}

void LivingMotor::syncSymmParameters(LivingMotor* refMotor)
{
	bracket->bracketInitialAngle = -refMotor->bracket->bracketInitialAngle;
	bracket->bracketConnectorAngle = -refMotor->bracket->bracketConnectorAngle;
	motor->rotAngleMax = -refMotor->motor->rotAngleMin;
	motor->rotAngleMin = -refMotor->motor->rotAngleMax;
}

void LivingMotor::switchToBestBodyPin()
{
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

