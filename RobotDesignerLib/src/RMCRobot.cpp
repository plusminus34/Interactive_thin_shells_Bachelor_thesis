#include <RobotDesignerLib/RMCRobot.h>
#include <RBSimLib/HingeJoint.h>
#include <ControlLib/Robot.h>
#include <RBSimLib/ODERBEngine.h>
//#include <MathLib/MeshBoolean.h>
#include <GUILib/GLContentManager.h>
#include <RobotDesignerLib/LivingMotor.h>
#include <RobotDesignerLib/LivingConnector.h>
#include <RobotDesignerLib/LivingEE.h>

// TODO: find where min and max where defined!
#undef min
#undef max

RMCRobot::RMCRobot(map<string, vector<Transformation>>& _transformationMap)
	: transformationMap(_transformationMap)
{

}

RMCRobot::RMCRobot(RMC* _root, map<string, vector<Transformation>>& _transformationMap)
	: root(_root), transformationMap(_transformationMap)
{
	
}


RMCRobot::~RMCRobot()
{
	if (disposed)
		return;

	delete root;
	for (uint i = 0; i < jointList.size(); i++) {
		delete jointList[i]->child;
		delete jointList[i];
	}
}

void RMCRobot::updateComponents() {
	if (root->type != LIVING_CONNECTOR)
		root->update();

	for (int i = 0; i < getJointCount(); i++)
		if (getJoint(i)->getChild()->type != LIVING_CONNECTOR)
			getJoint(i)->getChild()->update();

	if (root->type == LIVING_CONNECTOR)
		root->update();
	for (int i = 0; i < getJointCount(); i++)
		if (getJoint(i)->getChild()->type == LIVING_CONNECTOR)
			getJoint(i)->getChild()->update();

}

void RMCRobot::fixLivingBracketState(RMC* rmc) {
	LivingConnector* lc = dynamic_cast<LivingConnector*> (rmc);
	if (lc == NULL)
		return;
	if (lc->isFullyConnected() == false)
		return;
	P3D pos;

	for (auto& pin : lc->pins) {
		RMCPin* connectedPin = pin.getConnectedPin();
		P3D pinPos = connectedPin->rmc->state.getWorldCoordinates(P3D(connectedPin->transformation.T));
		pos += pinPos / lc->pins.size();
	}

	lc->state.position = pos;
	lc->update();
}


void RMCRobot::fixConstraints() {
	for (size_t j = 0; j < jointList.size(); j++)
		jointList[j]->fixRMCConstraint();

	fixLivingBracketState(root);

	for (int i = 0; i < getJointCount(); i++)
		fixLivingBracketState(getJoint(i)->getChild());
}

void RMCRobot::fixPlateStateByMotor()
{
	if (root->type == PLATE_RMC && root->cJoints.size() > 0)
	{
		RMCJoint* motorJoint = dynamic_cast<RMCJoint*>(root->cJoints[0]);
		motorJoint->fixRMCConstraintInverse();
	}
}

bool RMCRobot::connectRMCRobot(RMCRobot* child, RMCPin* parentPin, RMCPin* childPin, int relTransId)
{
	vector<Transformation> tmpTransList;
	string key = parentPin->name + '+' + childPin->name;
	
	if (transformationMap.count(key)){
		tmpTransList = transformationMap[key];
	}
	else return false;

	RMCJoint* joint = new RMCJoint(parentPin, childPin, tmpTransList, relTransId);
	jointList.push_back(joint);

	for (int i = 0; i < child->getJointCount(); i++)
	{
		jointList.push_back(child->getJoint(i));
	}
	child->disposed = true;
	delete child;

	fixConstraints();
	updateComponents();

	return true;
}

bool RMCRobot::connectRMCRobotDirectly(RMCRobot* child, RMC* parentRMC, int relTransId)
{
	RMC* childRMC = child->getRoot();
	vector<Transformation> tmpTransList;

	Transformation childTrans(childRMC->state.orientation.getRotationMatrix(), childRMC->state.position);
	Transformation parentTrans(parentRMC->state.orientation.getRotationMatrix(), parentRMC->state.position);
	tmpTransList.push_back(parentTrans.inverse() * childTrans);

	RMCJoint* joint = new RMCJoint(parentRMC, childRMC, tmpTransList, relTransId);
	jointList.push_back(joint);

	for (int i = 0; i < child->getJointCount(); i++)
	{
		jointList.push_back(child->getJoint(i));
	}
	child->disposed = true;
	delete child;

	fixConstraints();

	return true;

}

bool RMCRobot::connectRMC(RMC* child, RMCPin* parentPin, RMCPin* childPin, int relTransId)
{
	vector<Transformation> tmpTransList;
	string key = parentPin->name + '+' + childPin->name;

	if (transformationMap.count(key)) {
		tmpTransList = transformationMap[key];
	}
	else return false;

	RMCJoint* joint = new RMCJoint(parentPin, childPin, tmpTransList, relTransId);
	jointList.push_back(joint);

	fixConstraints();

	return true;
}

void RMCRobot::deleteSubTree(RMCJoint* joint, bool subRoot)
{
	RMC* child = joint->getChild();
	for (int i = 0; i < child->getChildJointCount(); i++)
	{
		deleteSubTree(child->getChildJoint(i), false);
	}

	if (subRoot) {
		joint->getParent()->removeChildJoint(joint);
		joint->parentPin->detach();
	}
		

	removeJoint(joint);
	delete joint;
	delete child;
}

RMCRobot* RMCRobot::clone()
{
	map<RMC*, RMC*> RMCMap;
	RMCMap[root] = root->clone();
	RMCRobot* newRobot = new RMCRobot(RMCMap[root], transformationMap);

	for (uint i = 0; i < jointList.size(); i++)
	{
		RMCJoint* oldJoint = jointList[i];
		RMC* oldRmc = oldJoint->getChild();
		RMC* childRmc = RMCMap[oldRmc] = oldRmc->clone();
		RMC* parentRmc = RMCMap[oldJoint->getParent()];
		RMCPin* parentPin = &parentRmc->pins[oldJoint->parentPin->id];
		RMCPin* childPin = &childRmc->pins[oldJoint->childPin->id];
		newRobot->addJoint(new RMCJoint(parentPin, childPin, oldJoint->transformations, oldJoint->curTransIndex));
	}
	
	newRobot->updateComponents();
	newRobot->fixConstraints();

	return newRobot;
}

RMCRobot* RMCRobot::cloneSubTree(RMC* rmc){
	map<RMC*, RMC*> RMCMap;
	cloneSubTreeHelper(rmc, RMCMap);
	RMCRobot* newRobot = new RMCRobot(RMCMap[rmc], transformationMap);

	for (uint i = 0; i < jointList.size(); i++)
	{
		RMCJoint* oldJoint = jointList[i];
		RMC* oldRmc = oldJoint->getChild();
		if (oldRmc == rmc) {
			RMCMap[oldRmc]->pins[oldJoint->childPin->id].detach();
			continue;
		}

		if (RMCMap.count(oldRmc) == 0) continue;

		RMC* childRmc = RMCMap[oldRmc];
		RMC* parentRmc = RMCMap[oldJoint->getParent()];
		RMCPin* parentPin = &parentRmc->pins[oldJoint->parentPin->id];
		RMCPin* childPin = &childRmc->pins[oldJoint->childPin->id];
		newRobot->addJoint(new RMCJoint(parentPin, childPin, oldJoint->transformations, oldJoint->curTransIndex));
	}
	newRobot->updateComponents();
	newRobot->fixConstraints();

	return newRobot;
}

void RMCRobot::cloneSubTreeHelper(RMC* rmc, map<RMC*, RMC*>& rmcMap){
	rmcMap[rmc] = rmc->clone();

	for (int i = 0; i < rmc->getChildJointCount(); i++){
		cloneSubTreeHelper(rmc->getChildJoint(i)->getChild(), rmcMap);
	}
}

void RMCRobot::getAvailableCompatiblePins(RMCPin* candidatePin, vector<RMCPin *>& availablePins)
{
	availablePins.clear();
	for (uint i = 0; i < root->pins.size(); i++) {
		RMCPin* pin = &root->pins[i];
		if (pin->idle && pin->isCompatible(candidatePin)) {
			availablePins.push_back(pin);
		}
	}

	for (uint i = 0; i < jointList.size(); i++){
		RMC* rmc = jointList[i]->getChild();
		for (uint j = 0; j < rmc->pins.size(); j++) {
			RMCPin* pin = &rmc->pins[j];
			if (pin->idle && pin->isCompatible(candidatePin)) {
				availablePins.push_back(pin);
			}
		}
	}
}

bool RMCRobot::pickPin(Ray& ray)
{
	selectedPin = NULL;
	highlightedPin = NULL;
	if (root->pickPin(ray))
	{
		highlightedPin = root->pickedPin;
		return true;
	}
	
	// *** can only pick pins on root, becasue only root can be attached to another robot.

	/*for (uint i = 0; i < jointList.size(); i++)
	{
		RMC* rmc = (RMC*)jointList[i]->child;
		if (rmc->pickPin(ray))
		{
			pickedPin = rmc->pickedPin;
			return true;
		}
	}*/

	return false;
}

void RMCRobot::clearPinPick()
{
	selectedPin = NULL;
	root->pickedPin = NULL;
	for (uint i = 0; i < jointList.size(); i++)
	{
		jointList[i]->getChild()->pickedPin = NULL;
	}
}

bool RMCRobot::pickRMC(Ray& ray, double* closestDist)
{
	highlightedRMC = NULL;

	double minDist = 1e10;
	bool intersect = false;
	double dist;
	bool res = root->pickMesh(ray, &dist);
	if (res && dist < minDist){
		intersect = true;
		minDist = dist;
		highlightedRMC = root;
	}

	for (uint i = 0; i < jointList.size(); i++) {
		bool res = jointList[i]->getChild()->pickMesh(ray, &dist);
		if (res && dist < minDist && jointList[i]->getChild()->type != LIVING_CONNECTOR) {
			intersect = true;
			minDist = dist;
			highlightedRMC = jointList[i]->getChild();
		}
	}
	
	if (closestDist)
		*closestDist = minDist;

	return intersect;
}

void RMCRobot::draw(int flags, const Vector4d& root_color, const Vector4d& highlight_color, const Vector4d& color)
{
	if (root == highlightedRMC || (highlightedRMC == NULL && root == selectedRMC))
		root->draw(flags, highlight_color);
	else
		root->draw(flags, root_color);

	for (uint i = 0; i < jointList.size(); i++) {
		if (jointList[i]->child == highlightedRMC || (highlightedRMC == NULL && jointList[i]->child == selectedRMC))
			jointList[i]->getChild()->draw(flags, highlight_color);
		else
			jointList[i]->getChild()->draw(flags, color);
	}
		
}

void RMCRobot::exportMeshes(const char* fName)
{
	FILE* fp1 = fopen(fName, "w+");
	uint fpIndex1 = 0;

	for (uint i = 0; i < jointList.size(); i++)
	{
		RMCJoint* joint = jointList[i];
		RMC* childRMC = joint->getChild();
		RMC* parentRMC = joint->getParent();

		if (childRMC->type == PLATE_RMC) continue;

		for (uint k=0;k<childRMC->meshes.size();k++){
			childRMC->meshes[k]->renderToObjFile(fp1, fpIndex1, childRMC->state.orientation, childRMC->state.position);
			fpIndex1 += childRMC->meshes[k]->getVertexCount();
		}
	}

	fclose(fp1);
}

void RMCRobot::saveToFile(const char* fName)
{
	FILE* fp = fopen(fName, "w+");
	saveToFile(fp);
	fclose(fp);
}

void RMCRobot::saveToFile(FILE* fp){
	fprintf(fp, "RMCRobot\n");
	
	Quaternion q = root->state.orientation;
	fprintf(fp, "Orientation %lf %lf %lf %lf\n", q[0], q[1], q[2], q[3]);

	P3D pos = root->state.position;
	fprintf(fp, "Position %lf %lf %lf\n", pos[0], pos[1], pos[2]);

	vector<RMC*> RMCVec;
	map<RMC*, int> RMCMap;
	
	RMCVec.push_back(root);
	for (uint i = 0; i < jointList.size(); i++)
		RMCVec.push_back(jointList[i]->getChild());

	char tmpLine[1000];

	for (uint i = 0; i < RMCVec.size(); i++) {
		RMC* rmc = RMCVec[i];
		RMCMap[rmc] = (int)i;
		tmpLine[0] = '\0';
		fprintf(fp, "RMC %s ", rmc->name.c_str());
		rmc->writeParamsToCommandLine(tmpLine);
		fprintf(fp, "%s \n", tmpLine);
	}

	for (uint i = 0; i < jointList.size(); i++) {
		RMC* parent = jointList[i]->getParent();
		RMC* child = jointList[i]->getChild();
		fprintf(fp, "RMCJoint %d %d %d %d %d\n", RMCMap[parent], RMCMap[child],
			jointList[i]->parentPin->id, jointList[i]->childPin->id, jointList[i]->curTransIndex);
	}
		
	fprintf(fp, "EndRMCRobot\n\n\n");
}

void RMCRobot::restoreAllMotorAngles() {
	vector<RMC*> RMCVec;

	RMCVec.push_back(root);
	for (uint i = 0; i < jointList.size(); i++)
		RMCVec.push_back(jointList[i]->getChild());

	for (uint i = 0; i < RMCVec.size(); i++) {
		Motor_RMC* rmc = dynamic_cast<Motor_RMC*>(RMCVec[i]);
		if (!rmc) continue;
		rmc->motorAngle = rmc->backupMotorAngle;
		rmc->update();
		fixConstraints();
	}
}

void RMCRobot::resetAllMotorAngles() {
	vector<RMC*> RMCVec;

	RMCVec.push_back(root);
	for (uint i = 0; i < jointList.size(); i++)
		RMCVec.push_back(jointList[i]->getChild());

	for (uint i = 0; i < RMCVec.size(); i++) {
		Motor_RMC* rmc = dynamic_cast<Motor_RMC*>(RMCVec[i]);
		if (!rmc) continue;
		rmc->backupMotorAngle = rmc->motorAngle;
		rmc->motorAngle = 0;
		rmc->update();
		fixConstraints();
	}
}


void RMCRobot::loadFromFile(const char* fName, map<string, RMC*>& rmcNameMap)
{
	FILE* fp = fopen(fName, "r");
	loadFromFile(fp, rmcNameMap);
	fclose(fp);
}

void RMCRobot::loadFromFile(FILE* fp, map<string, RMC*>& rmcNameMap){
	char buffer[200];
	char keyword[50];
	vector<RMC*> tmpRmcs;
	map<RMC*, P3D> posMap;
	map<RMC*, Quaternion> qMap;
	Quaternion q;
	P3D pos;
	
	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 500);
		if (strlen(buffer) > 495)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		sscanf(line, "%s", keyword);
		//Logger::print("%s ", keyword);

		if (strcmp(keyword, "RMC") == 0) {
			char name[50];
			sscanf(line + strlen(keyword), "%s", name);
			RMC* newRMC = rmcNameMap[name]->clone();

			newRMC->readParamsFromCommandLine(line + strlen(keyword) + strlen(name) + 1);
			newRMC->update();

			tmpRmcs.push_back(newRMC);

			if (!root)
				root = newRMC;

			posMap[newRMC] = newRMC->state.position;
			qMap[newRMC] = newRMC->state.orientation;
		} else if (strcmp(keyword, "RMCJoint") == 0) {
			int parentId, childId;
			int parentPinId, childPinId;
			int curTransIndex;
			sscanf(line + strlen(keyword), "%d %d %d %d %d", &parentId, &childId, &parentPinId, &childPinId, &curTransIndex);
			
			RMC* parentRmc = tmpRmcs[parentId];
			RMC* childRmc = tmpRmcs[childId];
			parentPinId = std::min(parentPinId, (int)parentRmc->pins.size() - 1);
			childPinId = std::min(childPinId, (int)childRmc->pins.size() - 1);
			RMCPin* parentPin = &parentRmc->pins[parentPinId];
			RMCPin* childPin = &childRmc->pins[childPinId];
			string key = parentPin->name + '+' + childPin->name;
			addJoint(new RMCJoint(parentPin, childPin, transformationMap[key], curTransIndex));
		} else if (strcmp(keyword, "Orientation") == 0) {
			sscanf(line + strlen(keyword),"%lf %lf %lf %lf\n", &q[0], &q[1], &q[2], &q[3]);
		} else if (strcmp(keyword, "Position") == 0) {
			sscanf(line + strlen(keyword), "%lf %lf %lf\n", &pos[0], &pos[1], &pos[2]);
		}else if (strcmp(keyword, "EndRMCRobot") == 0) {
			break;
		}

	}

	root->state.position = pos;
	root->state.orientation = q;

	for (auto& itr : posMap){
		RMC* rmc = itr.first;
		if (rmc == root) continue;
		
		rmc->state.position = itr.second;
		rmc->state.orientation = qMap[rmc];
	}

	updateComponents();
	fixConstraints();
}


void RMCRobot::saveToRBSFile(const char* fName, const string& robotMeshDir, Robot* templateRobot, bool freezeRoot){
	map<RMC*, double> jointMotorAngleMap;
	for (uint i = 0; i < jointList.size(); i++){
		RMCJoint* joint = jointList[i];
		RMC* parentRMC = jointList[i]->getParent();
		RMC* childRMC = jointList[i]->getChild();

		if (joint->parentPin && joint->parentPin->type == HORN_PIN) {
			if (Motor_RMC* tmpMotor = dynamic_cast<Motor_RMC*>(parentRMC))
				jointMotorAngleMap[parentRMC] = RAD(tmpMotor->motorAngle);
		}
		else if (joint->childPin && joint->childPin->type == HORN_PIN) {
			if (Motor_RMC* tmpMotor = dynamic_cast<Motor_RMC*>(childRMC))
				jointMotorAngleMap[childRMC] = RAD(-tmpMotor->motorAngle);
		}
		else
			continue;
	}

	resetAllMotorAngles();

	int meshID = 0;
	for (int i = 0; i < getJointCount(); i++) {
		RMC* rmc = getJoint(i)->getChild();
		rmc->exportMeshes(robotMeshDir.c_str(), meshID++);
	}

	FILE* fp = fopen(fName, "w+");
	
	int RBIndex = 1;
	vector<std::pair<RMC*, int>> RBIndexMap;
	getRMCToRBIndexMap(root, 0, RBIndex, RBIndexMap);

	vector<RigidBody> tmpRBs(RBIndex);
	for (uint i = 0; i < tmpRBs.size(); i++){
		if (freezeRoot && i == 0)
			tmpRBs[i].rbProperties.isFrozen = true;
		tmpRBs[i].rbProperties.mass = 0;
		tmpRBs[i].rbProperties.MOI_local.setZero();
		tmpRBs[i].state.position.setZero();
		tmpRBs[i].name = "rb" + to_string(i);
	}

	// update mass
	for (auto itr = RBIndexMap.begin(); itr != RBIndexMap.end(); itr++)
	{
		RMC* rmc = itr->first;
		RigidBody& rb = tmpRBs[itr->second];
		rb.rbProperties.mass += rmc->rbProperties.mass;
		rb.state.position += rmc->state.position * rmc->rbProperties.mass;
	}

	// calculate COM
	for (uint i = 0; i < tmpRBs.size(); i++)
	{
		if (templateRobot)
		{
			tmpRBs[i].state.position = templateRobot->getRBByName(tmpRBs[i].name.c_str())->state.position;
		}
		else {
			tmpRBs[i].state.position /= tmpRBs[i].rbProperties.mass;
		}
	}

	// update MOI
	for (auto itr = RBIndexMap.begin(); itr != RBIndexMap.end(); itr++)
	{
		RMC* rmc = itr->first;
		RigidBody& rb = tmpRBs[itr->second];
		rb.rbProperties.MOI_local += rmc->getWorldMOIAboutPoint(rb.state.position);

		if (rmc->rbProperties.endEffectorPoints.size() > 0){
			rb.mappingInfo = rmc->mappingInfo;
		}

		for (uint i = 0; i < rmc->rbProperties.endEffectorPoints.size(); i++){
			P3D endEffectorPos = rb.getLocalCoordinates(rmc->getWorldCoordinates(rmc->rbProperties.endEffectorPoints[i].coords));
			rb.rbProperties.endEffectorPoints.push_back(endEffectorPos);

			if (WheelEE_RMC* wheelEE = dynamic_cast <WheelEE_RMC*>(rmc)) {
				rb.rbProperties.endEffectorPoints.back().setMode((wheelEE->isActive) ? EE_ACTIVE_WHEEL : EE_PASSIVE_WHEEL);
				rb.rbProperties.endEffectorPoints.back().featureSize = wheelEE->radius;
				rb.rbProperties.endEffectorPoints.back().localCoordsWheelAxis = rb.getLocalCoordinates(rmc->getWorldCoordinates(V3D(0, -1, 0)));
				//rb.cdps.push_back(new SphereCDP(endEffectorPos, wheelEE->radius));
			}
			else if (SphereEE_RMC* sphereEE = dynamic_cast <SphereEE_RMC*>(rmc)) {
				rb.rbProperties.endEffectorPoints.back().setMode(EE_WELDED_WHEEL);
				rb.rbProperties.endEffectorPoints.back().featureSize = sphereEE->sphereRadius;
				rb.rbProperties.endEffectorPoints.back().localCoordsWheelAxis = rb.getLocalCoordinates(rmc->getWorldCoordinates(V3D(0, -1, 0)));
				rb.cdps.push_back(new SphereCDP(endEffectorPos , sphereEE->sphereRadius));
			}
			else {
//				rb.cdps.push_back(new SphereCDP(endEffectorPos + V3D(0, 1, 0) * 0.01, 0.01));
				rb.cdps.push_back(new SphereCDP(endEffectorPos + V3D(0, 1, 0) * 0.00, 0.01));
			}
		}

		for (uint i = 0; i < rmc->rbProperties.bodyPointFeatures.size(); i++)
		{
			P3D FP = rb.getLocalCoordinates(rmc->getWorldCoordinates(rmc->rbProperties.bodyPointFeatures[i].coords));
			rb.rbProperties.bodyPointFeatures.push_back(FP);
		}

		if (Motor_RMC* livingRMC = dynamic_cast<Motor_RMC*>(rmc)){
			RMC* bracketConnectedRMC = NULL;
			string jointName;
			bool isParent = true;
			for (auto& pin : rmc->pins)
			{
				if (pin.livingType == LIVING_HORN_PIN && !pin.idle)
				{
					if (pin.joint->getParent() == rmc) {
						bracketConnectedRMC = pin.joint->getChild();
					}
					else {
						bracketConnectedRMC = pin.joint->getParent();
						isParent = false;
					}
				}
			}
			if (bracketConnectedRMC)
			{
				auto it = std::find_if(RBIndexMap.begin(), RBIndexMap.end(),
					[bracketConnectedRMC](const auto &el) {return el.first == bracketConnectedRMC; });
				RigidBody& bracketConnectedRB = tmpRBs[it->second];
				bracketConnectedRB.meshes.push_back(livingRMC->hornBracket->bracketMesh);
				Transformation trans;
				trans.R = rmc->state.orientation.getRotationMatrix();
				trans.T = rmc->state.position - bracketConnectedRB.state.position;
				bracketConnectedRB.meshTransformations.push_back(trans);
				jointName = isParent ? rb.name + "_" + bracketConnectedRB.name :
					bracketConnectedRB.name + "_" + rb.name;
				bracketConnectedRB.meshDescriptions.push_back(jointName);

				bracketConnectedRB.meshes.push_back(livingRMC->motorHornMesh);
				bracketConnectedRB.meshTransformations.push_back(trans);
				bracketConnectedRB.meshDescriptions.push_back(jointName);
			}

			{
				Transformation trans;
				trans.R = rmc->state.orientation.getRotationMatrix();
				trans.T = rmc->state.position - rb.state.position;

				rb.meshes.push_back(livingRMC->motorBodyMesh);
				rb.meshTransformations.push_back(trans);
				rb.meshDescriptions.push_back(jointName);

				rb.meshes.push_back(livingRMC->bodyBracket->bodyBracketMesh);
				rb.meshTransformations.push_back(trans);
				rb.meshDescriptions.push_back(jointName);
			}
		}

		if (LivingConnector* livingConnector = dynamic_cast<LivingConnector*>(rmc)){
			rb.meshes.push_back(livingConnector->connectorMesh);
			Transformation trans;
			trans.R = rmc->state.orientation.getRotationMatrix();
			trans.T = rmc->state.position - rb.state.position;
			rb.meshTransformations.push_back(trans);
			rb.meshDescriptions.push_back("skeleton");
		}

		if (SphereEE_RMC* sphereEE = dynamic_cast<SphereEE_RMC*>(rmc)){
			rb.meshes.push_back(sphereEE->eeMesh);
			Transformation trans;
			trans.R = rmc->state.orientation.getRotationMatrix();
			trans.T = rmc->state.position - rb.state.position;
			rb.meshTransformations.push_back(trans);
			rb.meshDescriptions.push_back("skeleton");
		}

		else if (ConnectorHUB_RMC* connector = dynamic_cast<ConnectorHUB_RMC*>(rmc)){
			rb.meshes.push_back(connector->mesh);
			Transformation trans;
			trans.R = rmc->state.orientation.getRotationMatrix();
			trans.T = rmc->state.position - rb.state.position;
			rb.meshTransformations.push_back(trans);
			rb.meshDescriptions.push_back("skeleton");
		}

		else if (WheelEE_RMC* wheelEE = dynamic_cast<WheelEE_RMC*>(rmc)){
			rb.meshes.push_back(wheelEE->wheelMesh);
			//keep track of the mesh associated with this end effector - we will need to keep track of it such as to be able to spin it appropriately... 
			rb.rbProperties.endEffectorPoints.back().meshIndex = rb.meshes.size() - 1;
			Transformation trans;
			trans.R = rmc->state.orientation.getRotationMatrix();
			trans.T = rmc->state.position - rb.state.position;
			rb.meshTransformations.push_back(trans);
			rb.meshDescriptions.push_back("skeleton");

			if (wheelEE->motorMesh) {
				rb.meshes.push_back(wheelEE->motorMesh);
				rb.meshTransformations.push_back(trans);
				rb.meshDescriptions.push_back("skeleton");
			}

			if (wheelEE->motorBracketMesh) {
				rb.meshes.push_back(wheelEE->motorBracketMesh);
				rb.meshTransformations.push_back(trans);
				rb.meshDescriptions.push_back("skeleton");
			}

		}

		if (!rmc->meshes.empty()) {
			rb.meshes.push_back(rmc->meshes[0]);
			Transformation trans;
			trans.R = rmc->state.orientation.getRotationMatrix();
			trans.T = rmc->state.position - rb.state.position;
			rb.meshTransformations.push_back(trans);
			rb.meshDescriptions.push_back("skeleton");
		}
	}

		// not merging meshes
		for (auto& rb : tmpRBs)
		{
			for (auto& description : rb.meshDescriptions)
			{
				if (description != "skeleton")
					description = "motor";
			}
		}

	//make a collision primitive for the root as well...
	bool first = true;
	AxisAlignedBoundingBox aabBox(root->cJoints[0]->pJPos, root->cJoints[0]->pJPos);
	for (uint i = 0; i < jointList.size(); i++) {
		RMCJoint* joint = jointList[i];
		RMC* parentRMC = jointList[i]->getParent();
		RMC* childRMC = jointList[i]->getChild();
		auto it = std::find_if(RBIndexMap.begin(), RBIndexMap.end(),
			[parentRMC](const auto &el) {return el.first == parentRMC; });
		RigidBody* parentRB = &tmpRBs[it->second];
		if (parentRB == &tmpRBs[0]) {
			RMC* motorRMC = NULL;
			if (joint->parentPin && joint->parentPin->type == HORN_PIN) {
				motorRMC = parentRMC;
			}
			else if (joint->childPin && joint->childPin->type == HORN_PIN) {
				motorRMC = childRMC;
			}
			else
				continue;

			P3D wJPos = motorRMC->getWorldCoordinates(P3D());
			P3D pJPos = parentRB->getLocalCoordinates(wJPos);

			if (first) {
				aabBox = AxisAlignedBoundingBox(pJPos, pJPos);
				first = false;
			}else
				aabBox.addPoint(pJPos);
		}
	}

	if (!first) {
		aabBox.setbmin(aabBox.bmin() + V3D(-0.025, -0.025, -0.025));
		aabBox.setbmax(aabBox.bmax() + V3D(0.025, 0.025, 0.025));
		tmpRBs[0].cdps.push_back(new BoxCDP(aabBox.bmin(), aabBox.bmax()));
	}

	// write RigidBody
	for (uint i = 0; i < tmpRBs.size(); i++)
	{
		tmpRBs[i].writeToFile(fp);
	}

	map<string, Quaternion> jointRelQMap;
	// write HingeJoint
	for (uint i = 0; i < jointList.size(); i++)
	{
		RMCJoint* joint = jointList[i];
		RMC* parentRMC = jointList[i]->getParent();
		RMC* childRMC = jointList[i]->getChild();
		auto it = std::find_if(RBIndexMap.begin(), RBIndexMap.end(),
			[parentRMC](const auto &el) {return el.first == parentRMC; });
		RigidBody* parentRB = &tmpRBs[it->second];
		it = std::find_if(RBIndexMap.begin(), RBIndexMap.end(),
			[childRMC](const auto &el) {return el.first == childRMC; });
		RigidBody* childRB = &tmpRBs[it->second];
		Motor_RMC* motorRMC = NULL;
		string jointName = parentRB->name + "_" + childRB->name;

		if (joint->parentPin && joint->parentPin->type == HORN_PIN) {
			if (motorRMC = dynamic_cast<Motor_RMC*>(parentRMC))
				jointRelQMap[jointName] = getRotationQuaternion(RAD(motorRMC->motorAngle), motorRMC->getWorldCoordinates(motorRMC->motorAxis));
		}
		else if (joint->childPin && joint->childPin->type == HORN_PIN) {
			if (motorRMC = dynamic_cast<Motor_RMC*>(childRMC))
				jointRelQMap[jointName] = getRotationQuaternion(RAD(-motorRMC->motorAngle), motorRMC->getWorldCoordinates(motorRMC->motorAxis));
		}
		else
			continue;
		
		P3D wJPos = motorRMC->getWorldCoordinates(P3D());
		V3D wJAxis = motorRMC->getWorldCoordinates(motorRMC->motorAxis);
		P3D pJPos = parentRB->getLocalCoordinates(wJPos);
		P3D cJpos = childRB->getLocalCoordinates(wJPos);
		V3D pJAxis = parentRB->getLocalCoordinates(wJAxis);
		HingeJoint hingeJoint;
		hingeJoint.pJPos = pJPos;
		hingeJoint.cJPos = cJpos;
		hingeJoint.rotationAxis = pJAxis;
		hingeJoint.name = jointName;
		hingeJoint.parent = parentRB;
		hingeJoint.child = childRB;
		hingeJoint.mappingInfo = motorRMC->mappingInfo;
		hingeJoint.defaultAngle = jointMotorAngleMap[motorRMC];

		hingeJoint.writeToFile(fp);
	}

	fclose(fp);

	//for (auto itr = jointRelQMap.begin(); itr != jointRelQMap.end(); itr++)
	//{
	//	Logger::print("%s %lf %lf %lf %lf\n", itr->first.c_str(), itr->second[0], itr->second[1], itr->second[2], itr->second[3]);
	//}
/*
	ODERBEngine rbEngine;
	rbEngine.loadRBsFromFile(fName);
	Robot robot(rbEngine.rbs[0]);
	RobotState tmpState(&robot);
	tmpState.setPosition(tmpRBs[0].state.position);
	tmpState.setOrientation(tmpRBs[0].state.orientation);

	for (int i = 0; i < robot.getJointCount(); i++)
	{
		RigidBody* parentRB = robot.getJoint(i)->parent;
		RigidBody* childRB = robot.getJoint(i)->child;
		string jointName = robot.getJoint(i)->name;

		//Logger::print("%s\n",jointName.c_str());

		if (jointRelQMap.count(jointName))
			tmpState.setJointRelativeOrientation(jointRelQMap[jointName], i);
	}

	tmpState.writeToFile("../out/tmpState.txt");
*/

	restoreAllMotorAngles();

	//and now save the corresponding rs file...

}

void RMCRobot::getRMCToRBIndexMap(RMC* node, int curIndex, int& RBIndex, vector<std::pair<RMC*, int>>& RBIndexMap){		
	RBIndexMap.push_back(std::pair<RMC*, int>(node, curIndex));

	// TODO
	/*if (node->getChildJointCount() == 0 && node->rbProperties.endEffectorPoints.empty())
	{
		node->rbProperties.endEffectorPoints.push_back(P3D(node->getLocalCoordinates(V3D(0, -0.03, 0))));
	}*/

	for (int i = 0; i < node->getChildJointCount(); i++)
	{
		RMCJoint* joint = node->getChildJoint(i);
		RMC* rmc = joint->getChild();
		if ((joint->parentPin && joint->parentPin->type == HORN_PIN) || (joint->childPin && joint->childPin->type == HORN_PIN)){
			curIndex = RBIndex++;
		}
		
		getRMCToRBIndexMap(rmc, curIndex, RBIndex, RBIndexMap);
	}
}

