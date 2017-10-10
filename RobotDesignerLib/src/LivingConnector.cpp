#include <RobotDesignerLib/LivingConnector.h>
#include <GUILib/GLApplication.h>
#include <GUILib/OBJReader.h>
#include <MathLib/ConvexHull3D.h>


LivingConnector::LivingConnector()
{
	type = LIVING_CONNECTOR;

	for (int i = 0; i < 2; i++)
	{
		RMCPin livingPin(this, Transformation(), pins.size());
		livingPin.type = NORMAL_PIN;
		livingPin.livingType = LIVING_BODY_PIN;
		livingPin.name = "LivingConnectorPin-" + to_string(i + 1);
		pins.push_back(livingPin);
	}

	updateMeshAndPinByDefault();
}

LivingConnector::~LivingConnector()
{
	delete connectorMesh;
}

LivingConnector* LivingConnector::clone()
{
	LivingConnector* new_rmc = new LivingConnector();
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

bool LivingConnector::pickMesh(Ray& ray, double* closestDist /*= NULL*/)
{
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return connectorMesh->getDistanceToRayOriginIfHit(newRay, closestDist);
}

void LivingConnector::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/)
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
			connectorMesh->setMaterial(material);
		}
		else {
			GLShaderMaterial colorMat;
			colorMat.setColor(color[0], color[1], color[2], color[3]);
			connectorMesh->setMaterial(colorMat);
		}

		glEnable(GL_LIGHTING);
		glPushMatrix();
		glTranslated(state.position[0], state.position[1], state.position[2]);
		//and rotation part
		V3D rotAxis; double rotAngle;
		state.orientation.getAxisAngle(rotAxis, rotAngle);
		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);

		connectorMesh->drawMesh();
		glPopMatrix();
	}

	return;
}

void LivingConnector::update()
{
	if (isFullyConnected())
	{
		updateMeshAndPinImplicit();
	}
	else {
		updateMeshAndPinByDefault();
	}
}

bool LivingConnector::isFullyConnected()
{
	for (auto& pin : pins)
	{
		if (pin.idle)
			return false;
	}

	return true;
}

void LivingConnector::updateMeshAndPinByDefault()
{
	delete connectorMesh;
	connectorMesh = OBJReader::loadOBJFile("../data/3dModels/cube.obj");
	
	Transformation trans1, trans2;
	trans1.R = getRotationQuaternion(RAD(180), V3D(1, 0, 0)).getRotationMatrix();
	trans1.T[1] = 0.015;
	trans2.T[1] = -0.015;

	pins[0].transformation = trans1;
	pins[1].transformation = trans2;
}

void LivingConnector::updateMeshAndPinImplicit()
{

	// ******************* generate connector mesh *******************
	vector<P3D> featurePoints;

	for (auto& pin : pins)
	{
		RMCPin* connectedPin = pin.getConnectedPin();
		vector<P3D>& pinFP = connectedPin->face.vertices;
		for (auto& p : pinFP)
		{
			featurePoints.push_back(connectedPin->rmc->state.getWorldCoordinates(p));
		}
	}
	connectorMesh->clear();
	// Logger::print("size: %d\n", featurePoints.size());
	ConvexHull3D::computeConvexHullFromSetOfPoints(featurePoints, connectorMesh, true);

	// ******************* update pin transformation *******************
	P3D COM = connectorMesh->getCenterOfMass();

	for (auto& pin : pins)
	{
		RMCPin* connectedPin = pin.getConnectedPin();
		P3D pinPos(connectedPin->transformation.T);
		P3D pinWorldPos = connectedPin->rmc->state.getWorldCoordinates(pinPos);
		pin.transformation.R = connectedPin->rmc->state.orientation.getRotationMatrix() * connectedPin->transformation.R;
		pin.transformation.T = pinWorldPos - COM;
	}

	connectorMesh->translate(-COM);
	connectorMesh->calBoundingBox();
}

void LivingConnector::exportMeshes(const char* dirName, int index)
{
	string connectorFileName = dirName + string("LivingConnectorMesh") + to_string(index) + string(".obj");

	connectorMesh->path = connectorFileName;
	connectorMesh->writeTriangulatedMeshToObj(connectorFileName.c_str());
	GLMesh* nMesh = connectorMesh->clone();
	nMesh->setMaterial(material);
	GLContentManager::addMeshFileMapping(nMesh, connectorFileName.c_str());
}

