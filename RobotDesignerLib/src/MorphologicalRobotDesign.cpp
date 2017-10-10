#include <RobotDesignerLib/MorphologicalRobotDesign.h>



MorphologicalRobotDesign::MorphologicalRobotDesign(Robot* _robot) : ParameterizedRobotDesign(_robot)
{
	robot = _robot;
	for (int i = 0; i < robot->getJointCount(); i++)
	{
		initialMorphology.push_back(JointParameters(robot->getJoint(i)));
	}
}


MorphologicalRobotDesign::~MorphologicalRobotDesign()
{
}

void MorphologicalRobotDesign::getCurrentSetOfParameters(DynamicArray<double>& params) {
	
	params.clear();	
	RigidBody* root = robot->getRoot();

	for (int j = 0; j < 3; j += 2)
	{
		double scale = 0.0;
		int count = 0;
		for (uint k = 0; k < root->cJoints.size(); k++)
		{
			int cJoint = root->cJoints[k]->jIndex;
			if (initialMorphology[cJoint].pJPos[0] != 0) {
				scale += root->cJoints[k]->pJPos[j] / initialMorphology[cJoint].pJPos[j];
				count++;
			}
		}
		if (count > 0)
			scale /= count;
		else
			scale = 1.0;
		params.push_back(scale);
	}
	
	for (int i = 1; i < robot->getRigidBodyCount(); i++) {
		RigidBody* rb = robot->getRigidBody(i);
		int pJoint = rb->pJoints.empty() ? -1 : rb->pJoints[0]->jIndex;
		int cJoint = rb->cJoints.empty() ? -1 : rb->cJoints[0]->jIndex;
		double scale = 0;
		int count = 0;

		for (int j = 0; j < 3; j++) {
			if (cJoint >= 0 && initialMorphology[cJoint].pJPos[j] != 0) {
				scale += robot->getJoint(cJoint)->pJPos[j] / initialMorphology[cJoint].pJPos[j];
				count++;
			}
			if (pJoint >= 0 && initialMorphology[pJoint].cJPos[j] != 0) {
				scale += robot->getJoint(pJoint)->cJPos[j] / initialMorphology[pJoint].cJPos[j];
				count++;
			}
		}
		if (count > 0)
			scale /= count;
		else
			scale = 1.0;
		params.push_back(scale);
	}
}

void MorphologicalRobotDesign::setParameters(const DynamicArray<double>& params) {

	RigidBody* root = robot->getRoot();

	for (int j = 0; j < 2; j++)
	{
		for (uint k = 0; k < root->cJoints.size(); k++)
		{
			int cJoint = root->cJoints[k]->jIndex;
			robot->getJoint(cJoint)->pJPos[2 * j] = initialMorphology[cJoint].pJPos[2 * j] * params[j];
		}
	}

	for (int i = 1; i < robot->getRigidBodyCount(); i++) {
		RigidBody* rb = robot->getRigidBody(i);
		for (auto joint : rb->pJoints)
		{
			int pJoint = joint->jIndex;
			robot->getJoint(pJoint)->cJPos = initialMorphology[pJoint].cJPos * params[i + 1];
		}
		for (auto joint : rb->cJoints)
		{
			int cJoint = joint->jIndex;
			robot->getJoint(cJoint)->pJPos = initialMorphology[cJoint].pJPos * params[i + 1];
		}
	}
}

void MorphologicalRobotDesign::loadParamsFromFile(const char* fName)
{
	vector<double> params;
	int count;

	FILE* fp = fopen(fName, "r");
	fscanf(fp, "%d", &count);
	params.resize(count);
	for (int i = 0; i < count; i++)
	{
		fscanf(fp, "%lf", &params[i]);
	}
	fclose(fp);

	setParameters(params);
}
