#include <RobotDesignerLib/RMCRobot.h>
#include <RBSimLib/HingeJoint.h>
#include <ControlLib/Robot.h>
#include <RBSimLib/ODERBEngine.h>
//#include <MathLib/MeshBoolean.h>
#include <GUILib/GLContentManager.h>
#include <RobotDesignerLib/LivingMotor.h>
#include <RobotDesignerLib/LivingConnector.h>
#include <RobotDesignerLib/LivingSphereEE.h>

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

void RMCRobot::fixJointConstraints(){
	for (size_t j = 0; j < jointList.size(); j++) {
		jointList[j]->fixRMCConstraint();
	}
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

	fixJointConstraints();

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

	fixJointConstraints();

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

	fixJointConstraints();

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
	
	newRobot->updateAllLivingMotor();
	newRobot->fixJointConstraints();

	return newRobot;
}

RMCRobot* RMCRobot::cloneSubTree(RMC* rmc)
{
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
	newRobot->updateAllLivingMotor();
	newRobot->fixJointConstraints();

	return newRobot;
}

void RMCRobot::cloneSubTreeHelper(RMC* rmc, map<RMC*, RMC*>& rmcMap)
{
	rmcMap[rmc] = rmc->clone();

	for (int i = 0; i < rmc->getChildJointCount(); i++)
	{
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
		if (res && dist < minDist) {
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

void RMCRobot::exportMeshes(const char* fName, const char* carvefName)
{
	FILE* fp1 = fopen(fName, "w+");
	FILE* fp2 = fopen(carvefName, "w+");
	uint fpIndex1 = 0;
	uint fpIndex2 = 0;

	for (uint i = 0; i < jointList.size(); i++)
	{
		RMCJoint* joint = jointList[i];
		RMC* childRMC = joint->getChild();
		RMC* parentRMC = joint->getParent();

		if (childRMC->type == PLATE_RMC) continue;

		if (childRMC->type == MOTOR_RMC && parentRMC->type == PLATE_RMC && childRMC->carveMesh)
		{
			childRMC->carveMesh->renderToObjFile(fp2, fpIndex2, childRMC->state.orientation, childRMC->state.position);
			fpIndex2 += childRMC->carveMesh->getVertexCount();
		}
		
		childRMC->meshes[0]->renderToObjFile(fp1, fpIndex1, childRMC->state.orientation, childRMC->state.position);
		fpIndex1 += childRMC->meshes[0]->getVertexCount();
	}

	fclose(fp1);
	fclose(fp2);
}

void RMCRobot::saveToFile(const char* fName)
{
	FILE* fp = fopen(fName, "w+");
	saveToFile(fp);
	fclose(fp);
}

void RMCRobot::saveToFile(FILE* fp)
{
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

	for (uint i = 0; i < RMCVec.size(); i++) {
		RMC* rmc = RMCVec[i];
		RMCMap[rmc] = (int)i;
		q = rmc->state.orientation;
		pos = rmc->state.position;

		if (rmc->type == LIVING_MOTOR)
		{
			LivingMotor* livingRMC = dynamic_cast<LivingMotor*>(rmc);
			
			fprintf(fp, "LivingMotor %s ", rmc->getName().c_str());
			fprintf(fp, "%lf %lf %lf %lf %lf ", rmc->motorAngle, livingRMC->motor->rotAngleMax,
				livingRMC->motor->rotAngleMin, livingRMC->bracket->bracketInitialAngle, livingRMC->bracket->bracketConnectorAngle);
			fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf", q[0], q[1], q[2], q[3], pos[0], pos[1], pos[2]);
		}
		else if (rmc->type == LIVING_EE)
		{
			LivingSphereEE* livingRMC = dynamic_cast<LivingSphereEE*>(rmc);

			fprintf(fp, "LivingSphereEE %s ", rmc->getName().c_str());
			fprintf(fp, "%lf ", livingRMC->sphereRadius);
			fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf", q[0], q[1], q[2], q[3], pos[0], pos[1], pos[2]);
		}
		else {
			fprintf(fp, "RMC %s ", rmc->getName().c_str());
			fprintf(fp, "%lf ", rmc->motorAngle);
			fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf", q[0], q[1], q[2], q[3], pos[0], pos[1], pos[2]);
		}		
		fprintf(fp, "\n");
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
		RMC* rmc = RMCVec[i];
		rmc->motorAngle = rmc->backupMotorAngle;
		rmc->update();
		fixJointConstraints();
	}
}

void RMCRobot::resetAllMotorAngles() {
	vector<RMC*> RMCVec;

	RMCVec.push_back(root);
	for (uint i = 0; i < jointList.size(); i++)
		RMCVec.push_back(jointList[i]->getChild());

	for (uint i = 0; i < RMCVec.size(); i++) {
		RMC* rmc = RMCVec[i];
		rmc->backupMotorAngle = rmc->motorAngle;
		rmc->motorAngle = 0;
		rmc->update();
		fixJointConstraints();
	}
}


void RMCRobot::loadFromFile(const char* fName, map<string, RMC*>& rmcNameMap)
{
	FILE* fp = fopen(fName, "r");
	loadFromFile(fp, rmcNameMap);
	fclose(fp);
}

void RMCRobot::loadFromFile(FILE* fp, map<string, RMC*>& rmcNameMap)
{
	char buffer[200];
	char keyword[50];
	vector<RMC*> tmpRmcs;
	map<RMC*, P3D> posMap;
	map<RMC*, Quaternion> qMap;
	Quaternion q;
	P3D pos;
	
	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		sscanf(line, "%s", keyword);
		//Logger::print("%s ", keyword);

		if (strcmp(keyword, "RMC") == 0)
		{
			char name[50];
			sscanf(line + strlen(keyword), "%s", name);
			RMC* rmc = rmcNameMap[name]->clone();
			tmpRmcs.push_back(rmc);
			if (!root)
				root = rmc;

			P3D pos;
			Quaternion q;

			int num = sscanf(line + strlen(keyword) + strlen(name) + 1, "%lf %lf %lf %lf %lf %lf %lf %lf", &rmc->motorAngle,
				&q[0], &q[1], &q[2], &q[3], &pos[0], &pos[1], &pos[2]);

			posMap[rmc] = pos;
			qMap[rmc] = q;
		}
		else if (strcmp(keyword, "LivingMotor") == 0)
		{
			char name[50];
			sscanf(line + strlen(keyword), "%s", name);
			LivingMotor* livingRMC = dynamic_cast<LivingMotor*>(rmcNameMap[name])->clone();
			tmpRmcs.push_back(livingRMC);
			if (!root)
				root = livingRMC;

			P3D pos;
			Quaternion q;

			int num = sscanf(line + strlen(keyword) + strlen(name) + 1, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &livingRMC->motorAngle, &livingRMC->motor->rotAngleMax,
				&livingRMC->motor->rotAngleMin, &livingRMC->bracket->bracketInitialAngle, &livingRMC->bracket->bracketConnectorAngle, &q[0], &q[1], &q[2], &q[3], &pos[0], &pos[1], &pos[2]);
			livingRMC->update();

			posMap[livingRMC] = pos;
			qMap[livingRMC] = q;
		}
		else if (strcmp(keyword, "LivingSphereEE") == 0)
		{
			char name[50];
			sscanf(line + strlen(keyword), "%s", name);
			LivingSphereEE* livingRMC = dynamic_cast<LivingSphereEE*>(rmcNameMap[name])->clone();
			tmpRmcs.push_back(livingRMC);
			if (!root)
				root = livingRMC;

			P3D pos;
			Quaternion q;

			int num = sscanf(line + strlen(keyword) + strlen(name) + 1, "%lf %lf %lf %lf %lf %lf %lf %lf", &livingRMC->sphereRadius, &q[0], &q[1], &q[2], &q[3], &pos[0], &pos[1], &pos[2]);
			livingRMC->update();

			posMap[livingRMC] = pos;
			qMap[livingRMC] = q;
		}
		else if (strcmp(keyword, "RMCJoint") == 0)
		{
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
		}
		else if (strcmp(keyword, "Orientation") == 0) 
		{
			sscanf(line + strlen(keyword),"%lf %lf %lf %lf\n", &q[0], &q[1], &q[2], &q[3]);
		}
		else if (strcmp(keyword, "Position") == 0)
		{
			sscanf(line + strlen(keyword), "%lf %lf %lf\n", &pos[0], &pos[1], &pos[2]);
		}
		else if (strcmp(keyword, "EndRMCRobot") == 0)
		{
			break;
		}
	}

	root->state.position = pos;
	root->state.orientation = q;

	fixJointConstraints();

	for (auto& itr : posMap)
	{
		RMC* rmc = itr.first;
		if (rmc == root) continue;
		
		rmc->state.position = itr.second;
		rmc->state.orientation = qMap[rmc];
	}

	updateAllLivingMotor();
}


void RMCRobot::saveToRBSFile(const char* fName, Robot* templateRobot, bool freezeRoot, bool mergeMeshes, bool forFabrication){
	FILE* fp = fopen(fName, "w+");
	
	int RBIndex = 1;
	map<RMC*, int> RBIndexMap;
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

		if (rmc->rbProperties.endEffectorPoints.size() > 0)
		{
			rb.mappingInfo = rmc->mappingInfo;
		}

		for (uint i = 0; i < rmc->rbProperties.endEffectorPoints.size(); i++)
		{
			P3D endEffectorPos = rb.getLocalCoordinates(rmc->getWorldCoordinates(rmc->rbProperties.endEffectorPoints[i].coords));
			rb.rbProperties.endEffectorPoints.push_back(endEffectorPos);
//			rb.cdps.push_back(new SphereCDP(endEffectorPos + V3D(0, 1, 0) * 0.01, 0.01));
			rb.cdps.push_back(new SphereCDP(endEffectorPos + V3D(0, 1, 0) * 0.00, 0.01));	
		}

		for (uint i = 0; i < rmc->rbProperties.bodyPointFeatures.size(); i++)
		{
			P3D FP = rb.getLocalCoordinates(rmc->getWorldCoordinates(rmc->rbProperties.bodyPointFeatures[i].coords));
			rb.rbProperties.bodyPointFeatures.push_back(FP);
		}

		if (rmc->type == LIVING_MOTOR)
		{
			LivingMotor* livingRMC = dynamic_cast<LivingMotor*>(rmc);
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
				RigidBody& bracketConnectedRB = tmpRBs[RBIndexMap[bracketConnectedRMC]];
				bracketConnectedRB.meshes.push_back(livingRMC->bracket->outputMesh);
				bracketConnectedRB.carveMeshes.push_back(NULL);
				Transformation trans;
				trans.R = rmc->state.orientation.getRotationMatrix();
				trans.T = rmc->state.position - bracketConnectedRB.state.position;
				bracketConnectedRB.meshTransformations.push_back(trans);
				jointName = isParent ? rb.name + "_" + bracketConnectedRB.name :
					bracketConnectedRB.name + "_" + rb.name;
				bracketConnectedRB.meshDescriptions.push_back(jointName);

				// horn carving mesh
				if (mergeMeshes)
				{
					bracketConnectedRB.meshes.push_back(livingRMC->motor->hornCarvingMesh);
					bracketConnectedRB.carveMeshes.push_back(NULL);
					bracketConnectedRB.meshTransformations.push_back(trans);
					bracketConnectedRB.meshDescriptions.push_back("carving");
				}
			}

			{
				rb.meshes.push_back(livingRMC->motor->motorWholeMesh);
				rb.carveMeshes.push_back(NULL);
				Transformation trans;
				trans.R = rmc->state.orientation.getRotationMatrix();
				trans.T = rmc->state.position - rb.state.position;
				rb.meshTransformations.push_back(trans);
				rb.meshDescriptions.push_back(jointName);

				rb.meshes.push_back(livingRMC->motor->bodyBracketMesh);
				rb.carveMeshes.push_back(NULL);
				rb.meshTransformations.push_back(trans);
				rb.meshDescriptions.push_back(jointName);

				// body carving mesh
				if (mergeMeshes)
				{
					rb.meshes.push_back(livingRMC->motor->bodyCarvingMesh);
					rb.carveMeshes.push_back(NULL);
					rb.meshTransformations.push_back(trans);
					rb.meshDescriptions.push_back("carving");
				}
			}
		}

		if (rmc->type == LIVING_CONNECTOR)
		{
			LivingConnector* livingConnector = dynamic_cast<LivingConnector*>(rmc);
			rb.meshes.push_back(livingConnector->connectorMesh);
			rb.carveMeshes.push_back(NULL);
			Transformation trans;
			trans.R = rmc->state.orientation.getRotationMatrix();
			trans.T = rmc->state.position - rb.state.position;
			rb.meshTransformations.push_back(trans);
			rb.meshDescriptions.push_back("skeleton");
		}

		if (rmc->type == LIVING_EE)
		{
			LivingSphereEE* sphereEE = dynamic_cast<LivingSphereEE*>(rmc);
			rb.meshes.push_back(sphereEE->eeMesh);
			rb.carveMeshes.push_back(NULL);
			Transformation trans;
			trans.R = rmc->state.orientation.getRotationMatrix();
			trans.T = rmc->state.position - rb.state.position;
			rb.meshTransformations.push_back(trans);
			rb.meshDescriptions.push_back("skeleton");
		}

		if (!rmc->meshes.empty()) {
			rb.meshes.push_back(rmc->meshes[0]);
			rb.carveMeshes.push_back(rmc->carveMeshEx);
			Transformation trans;
			trans.R = rmc->state.orientation.getRotationMatrix();
			trans.T = rmc->state.position - rb.state.position;
			rb.meshTransformations.push_back(trans);
			rb.meshDescriptions.push_back("skeleton");
		}
	}

	if (mergeMeshes)
	{
        throw std::runtime_error("This functionality is not available.");
//		set<string> motorMeshFiles = {
//			"../data/robotDesigner/motorMeshes/XM-430.obj"
//		};

//		set<string> junkMeshFiles = {
//			"../data/robotDesigner/motorMeshes/XM-430-MotorPlate.obj"
//		};

//		for (auto& rb : tmpRBs)
//		{
//			vector<GLMesh*> motorMeshes;
//			vector<Transformation> motorTrans;
//			vector<GLMesh*> motorCarveMeshes;
//			vector<string> motorDescritions;
//			GLMesh* rbMesh = NULL;
//			for (int i = 0; i < (int)rb.meshes.size(); i++)
//			{
//				if (motorMeshFiles.count(rb.meshes[i]->path)) {
//					motorMeshes.push_back(rb.meshes[i]);
//					motorTrans.push_back(rb.meshTransformations[i]);
//					motorCarveMeshes.push_back(NULL);
//					motorDescritions.push_back("motor");

//					motorMeshes.push_back(rb.meshes[i]);
//					motorTrans.push_back(rb.meshTransformations[i]);
//					motorCarveMeshes.push_back(NULL);
//					motorDescritions.push_back(rb.meshDescriptions[i]);
//				}
//				else {
//					if (junkMeshFiles.count(rb.meshes[i]->path)) continue;

//					if (rb.meshDescriptions[i] != "carving")
//					{
//						GLMesh* tMesh = rb.meshes[i]->clone();
//						tMesh->transform(rb.meshTransformations[i]);
//						tMesh->scale(1.0001, tMesh->getCenterOfMass());
//						if (rbMesh)
//						{
//							meshBooleanIntrusive(rbMesh, tMesh, "Union");
//							delete tMesh;
//						}
//						else {
//							rbMesh = tMesh;
//						}
//					}

//					if (rb.meshDescriptions[i] != "skeleton") {
//						motorMeshes.push_back(rb.meshes[i]);
//						motorTrans.push_back(rb.meshTransformations[i]);
//						motorCarveMeshes.push_back(NULL);
//						motorDescritions.push_back(rb.meshDescriptions[i]);
//					}
//				}
//			}

//			if (forFabrication)
//			{
//				for (int i = 0; i < (int)rb.meshes.size(); i++)
//				{
//					if (!rbMesh || motorMeshFiles.count(rb.meshes[i]->path)) continue;
//					if (junkMeshFiles.count(rb.meshes[i]->path)) continue;

//					if (rb.meshDescriptions[i] == "carving")
//					{
//						GLMesh* tMesh = rb.meshes[i]->clone();
//						tMesh->transform(rb.meshTransformations[i]);
//						meshBooleanIntrusive(rbMesh, tMesh, "Minus");
//						delete tMesh;
//					}
//				}
//			}

//			if (!rbMesh) continue;

//			rbMesh->path = "../out/" + rb.name + "_merge.obj";
//			rbMesh->writeTriangulatedMeshToObj(rbMesh->path.c_str());
//			GLContentManager::addMeshFileMapping(rbMesh, rbMesh->path.c_str());
//			rb.meshes = motorMeshes;
//			rb.meshTransformations = motorTrans;
//			rb.carveMeshes = motorCarveMeshes;
//			rb.meshDescriptions = motorDescritions;
//			rb.meshes.push_back(rbMesh);
//			rb.meshTransformations.push_back(Transformation());
//			rb.carveMeshes.push_back(NULL);
//			rb.meshDescriptions.push_back("skeleton");
//		}
	}
	else {
		// not merging meshes
		for (auto& rb : tmpRBs)
		{
			for (auto& description : rb.meshDescriptions)
			{
				if (description != "skeleton")
					description = "motor";
			}
		}
	}
	

	//make a collision primitive for the root as well...
	bool first = true;
	AxisAlignedBoundingBox aabBox(root->cJoints[0]->pJPos, root->cJoints[0]->pJPos);
	for (uint i = 0; i < jointList.size(); i++) {
		RMCJoint* joint = jointList[i];
		RMC* parentRMC = jointList[i]->getParent();
		RMC* childRMC = jointList[i]->getChild();
		RigidBody* parentRB = &tmpRBs[RBIndexMap[parentRMC]];
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
		RigidBody* parentRB = &tmpRBs[RBIndexMap[parentRMC]];
		RigidBody* childRB = &tmpRBs[RBIndexMap[childRMC]];
		RMC* motorRMC;
		string jointName = parentRB->name + "_" + childRB->name;

		if (joint->parentPin && joint->parentPin->type == HORN_PIN) {
			jointRelQMap[jointName] = getRotationQuaternion(RAD(parentRMC->motorAngle), parentRMC->getWorldCoordinates(parentRMC->motorAxis));
			motorRMC = parentRMC;
		}
		else if (joint->childPin && joint->childPin->type == HORN_PIN) {
			jointRelQMap[jointName] = getRotationQuaternion(RAD(-childRMC->motorAngle), childRMC->getWorldCoordinates(childRMC->motorAxis));
			motorRMC = childRMC;
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
		hingeJoint.writeToFile(fp);
	}

	fclose(fp);

	//for (auto itr = jointRelQMap.begin(); itr != jointRelQMap.end(); itr++)
	//{
	//	Logger::print("%s %lf %lf %lf %lf\n", itr->first.c_str(), itr->second[0], itr->second[1], itr->second[2], itr->second[3]);
	//}

	ODERBEngine rbEngine;
	rbEngine.loadRBsFromFile(fName);
	Robot robot(rbEngine.rbs[0]);
	ReducedRobotState tmpState(&robot);
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

	//tmpState.writeToFile("../out/tmpState.txt");
}

//it is assumed that the robot was created from this design and follows all the same naming convention
ReducedRobotState RMCRobot::getReducedRobotState(Robot* r) {
	ReducedRobotState tmpState(r);
	tmpState.setPosition(r->root->state.position);
	tmpState.setOrientation(r->root->state.orientation);

	int RBIndex = 1;
	map<RMC*, int> RBIndexMap;
	getRMCToRBIndexMap(root, 0, RBIndex, RBIndexMap);

	vector<RigidBody> tmpRBs(RBIndex);
	for (uint i = 0; i < tmpRBs.size(); i++) {
		tmpRBs[i].name = "rb" + to_string(i);
	}

	map<string, double> jointMotorAngleMap;
	for (uint i = 0; i < jointList.size(); i++){
		RMCJoint* joint = jointList[i];
		RMC* parentRMC = jointList[i]->getParent();
		RMC* childRMC = jointList[i]->getChild();
		RigidBody* parentRB = &tmpRBs[RBIndexMap[parentRMC]];
		RigidBody* childRB = &tmpRBs[RBIndexMap[childRMC]];
		string jointName = parentRB->name + "_" + childRB->name;

		if (joint->parentPin && joint->parentPin->type == HORN_PIN)
			jointMotorAngleMap[jointName] = RAD(parentRMC->motorAngle);
		else if (joint->childPin && joint->childPin->type == HORN_PIN)
			jointMotorAngleMap[jointName] = RAD(-childRMC->motorAngle);
		else
			continue;
	}

	for (int i = 0; i < r->getJointCount(); i++){
		string jointName = r->getJoint(i)->name;
		HingeJoint* j = dynamic_cast<HingeJoint*> (r->getJoint(i));
		if (!j) continue;

		if (jointMotorAngleMap.count(jointName))
			tmpState.setJointRelativeOrientation(getRotationQuaternion(jointMotorAngleMap[jointName], j->rotationAxis), i);
	}

	return tmpState;
}

void RMCRobot::getRMCToRBIndexMap(RMC* node, int curIndex, int& RBIndex, map<RMC*, int>& RBIndexMap)
{		
	RBIndexMap[node] = curIndex;

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

void RMCRobot::addBulletObjectsToList(DynamicArray<AbstractBulletObject*>& list) {
	root->addBulletObjectsToList(list);
	for (uint i = 0; i < jointList.size(); i++)
	{
		getJoint(i)->getChild()->addBulletObjectsToList(list);
	}
}

void RMCRobot::updateAllLivingMotor()
{
	if (root->type == LIVING_MOTOR)
	{
		root->update();
	}

	for (int i = 0; i < getJointCount(); i++)
	{
		RMC* rmc = getJoint(i)->getChild();
		if (rmc->type == LIVING_MOTOR)
		{
			rmc->update();
		}
	}

	for (int i = 0; i < getJointCount(); i++)
	{
		RMC* rmc = getJoint(i)->getChild();
		if (rmc->type == LIVING_EE)
		{
			rmc->update();
		}
	}

	for (int i = 0; i < getJointCount(); i++)
	{
		RMC* rmc = getJoint(i)->getChild();
		if (rmc->type == LIVING_CONNECTOR)
		{
			rmc->update();
		}
	}

	// fixJointConstraints();
}
