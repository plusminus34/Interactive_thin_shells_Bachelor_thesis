#include <RobotDesignerLib/LivingConnector.h>
#include <GUILib/GLApplication.h>
#include <GUILib/OBJReader.h>
#include <MathLib/ConvexHull3D.h>

LivingConnector::LivingConnector(){
	type = LIVING_CONNECTOR;
	name = "LivingConnector";

	for (int i = 0; i < 2; i++)	{
		RMCPin livingPin(this, Transformation(), pins.size());
		livingPin.type = NORMAL_PIN;
		livingPin.livingType = LIVING_BODY_PIN;
		livingPin.name = "LivingConnectorPin-" + to_string(i + 1);
		pins.push_back(livingPin);
	}

	updateMeshAndPinByDefault();
}

LivingConnector::~LivingConnector(){
	delete connectorMesh;
}

LivingConnector* LivingConnector::clone(){
	LivingConnector* new_rmc = new LivingConnector();
	copyBasePropertiesTo(new_rmc, false);
	new_rmc->scale = scale;

	for (uint i = 0; i < pins.size(); i++){
		new_rmc->pins[i].compatibleMap = pins[i].compatibleMap;
	}

	new_rmc->update();

	return new_rmc;
}

bool LivingConnector::pickMesh(Ray& ray, double* closestDist /*= NULL*/){
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return connectorMesh->getDistanceToRayOriginIfHit(newRay, closestDist);
}

void LivingConnector::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (flags & SHOW_PINS)
		drawPins();

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

void LivingConnector::update(){
	if (isFullyConnected())
	{
		updateMeshAndPinImplicit();
	}
	else {
		updateMeshAndPinByDefault();
	}
}

bool LivingConnector::isFullyConnected(){
	for (auto& pin : pins)
	{
		if (pin.idle)
			return false;
	}

	return true;
}

void LivingConnector::updateMeshAndPinByDefault(){
	delete connectorMesh;
	connectorMesh = OBJReader::loadOBJFile("../data/robotDesigner/meshes/cube.obj");
	
	for (int i = 0; i < connectorMesh->getVertexCount() * 3; i++) {
		connectorMesh->getVertexArray()[i] *= scale;
	}

	Transformation trans1, trans2;
	trans1.R = getRotationQuaternion(RAD(180), V3D(1, 0, 0)).getRotationMatrix();
	trans1.T[1] = 0.015 * scale;
	trans2.T[1] = -0.015 * scale;

	pins[0].transformation = trans1;
	pins[1].transformation = trans2;
}

void LivingConnector::updateMeshAndPinImplicit(){
	//Logger::consolePrint("living connector is updating its mesh...\n");

	// ******************* generate connector mesh *******************
	vector<P3D> featurePoints;

	for (auto& pin : pins)
	{
		RMCPin* connectedPin = pin.getConnectedPin();
		vector<P3D>& pinFP = connectedPin->face.vertices;
		for (auto& p : pinFP)
		{
			featurePoints.push_back(getLocalCoordinates(connectedPin->rmc->state.getWorldCoordinates(p)));
		}
	}
	connectorMesh->clear();
	//Logger::consolePrint("size: %d\n", featurePoints.size());
	ConvexHull3D::computeConvexHullFromSetOfPoints(featurePoints, connectorMesh, true);

	// ******************* update pin transformation *******************
	for (auto& pin : pins){
		RMCPin* connectedPin = pin.getConnectedPin();
		P3D pinPos = getLocalCoordinates(connectedPin->rmc->state.getWorldCoordinates(P3D(connectedPin->transformation.T)));
		pin.transformation.R = (state.orientation.getComplexConjugate() * connectedPin->rmc->state.orientation).getRotationMatrix() * connectedPin->transformation.R;
		pin.transformation.T = pinPos;
	}

	connectorMesh->calBoundingBox();
}

void LivingConnector::exportMeshes(const char* dirName, int index){
	string connectorFileName = dirName + string("LivingConnectorMesh") + to_string(index) + string(".obj");

	connectorMesh->path = connectorFileName;
	connectorMesh->writeTriangulatedMeshToObj(connectorFileName.c_str());
	GLMesh* nMesh = connectorMesh->clone();
	nMesh->setMaterial(material);
	GLContentManager::addMeshFileMapping(nMesh, connectorFileName.c_str());
}

/** -------------------------------------------------------------------------------- **/
ConnectorHUB_RMC::ConnectorHUB_RMC() {
	type = LIVING_CONNECTOR_HUB;
	name = "GenericConnectorHUB";

	update();
}

ConnectorHUB_RMC::~ConnectorHUB_RMC() {
	delete mesh;
}

ConnectorHUB_RMC* ConnectorHUB_RMC::clone() {
	ConnectorHUB_RMC* new_rmc = new ConnectorHUB_RMC();
	copyBasePropertiesTo(new_rmc, true);
	new_rmc->size = size;
	new_rmc->connectorMeshName = connectorMeshName;

	for (uint i = 0; i < pins.size(); i++)
		new_rmc->pins[i].compatibleMap = pins[i].compatibleMap;

	new_rmc->update();

	return new_rmc;
}

bool ConnectorHUB_RMC::pickMesh(Ray& ray, double* closestDist /*= NULL*/) {
	Transformation invTrans = Transformation(state.orientation.getRotationMatrix(), state.position).inverse();
	Ray newRay(invTrans.transform(ray.origin), invTrans.transform(ray.direction));

	return (mesh->getDistanceToRayOriginIfHit(newRay, closestDist));
}

void ConnectorHUB_RMC::draw(int flags, const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/) {
	if (flags & SHOW_PINS)
		drawPins();

	if (flags & SHOW_MESH && mesh) {
		if (color.isZero()) {
			mesh->setMaterial(material);
		}
		else {
			GLShaderMaterial colorMat;
			colorMat.setColor(color[0], color[1], color[2], color[3]);
			mesh->setMaterial(colorMat);
		}

		glEnable(GL_LIGHTING);
		glPushMatrix();
		glTranslated(state.position[0], state.position[1], state.position[2]);
		//and rotation part
		V3D rotAxis; double rotAngle;
		state.orientation.getAxisAngle(rotAxis, rotAngle);
		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);

		mesh->drawMesh();
		glPopMatrix();
	}

	return;
}

void ConnectorHUB_RMC::update() {
	delete mesh; mesh = NULL;
	if (connectorMeshName.length() > 0){
		mesh = GLContentManager::getGLMesh(connectorMeshName.c_str())->clone();
		//now scale it according to the radius...
		for (int i = 0; i < mesh->getVertexCount() * 3; i++) {
			mesh->getVertexArray()[i] *= size;
		}
		mesh->calBoundingBox();
	}

	for (auto& pin : pins) {
		for (auto& v : pin.face.vertices)
			v = P3D() + V3D(P3D(), v).unit() * size * sqrt(3);
		pin.face.center = P3D() + V3D(P3D(), pin.face.center).unit() * size;
		pin.transformation.T = pin.transformation.T.normalized() * size;
	}
}

void ConnectorHUB_RMC::syncSymmParameters(RMC* ref) {
	ConnectorHUB_RMC* con = dynamic_cast<ConnectorHUB_RMC*>(ref);
	if (con)
		size = con->size;
}

void ConnectorHUB_RMC::exportMeshes(const char* dirName, int index) {
	string fileName = dirName + string("LivingConnectorMesh") + to_string(index) + string(".obj");

	mesh->path = fileName;
	mesh->writeTriangulatedMeshToObj(fileName.c_str());
	GLMesh* nMesh = mesh->clone();
	nMesh->setMaterial(material);
	GLContentManager::addMeshFileMapping(nMesh, fileName.c_str());
}

