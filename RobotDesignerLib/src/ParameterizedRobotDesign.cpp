#include <RobotDesignerLib/ParameterizedRobotDesign.h>
#include <GUILib/GLContentManager.h>

ParameterizedRobotDesign::ParameterizedRobotDesign(Robot* robot) {
	this->robot = robot;

	for (int i = 0; i < robot->getJointCount(); i++)
		initialMorphology.push_back(JointParameters(robot->getJoint(i)));
}

ParameterizedRobotDesign::~ParameterizedRobotDesign() {
}

void ParameterizedRobotDesign::getCurrentSetOfParameters(DynamicArray<double>& params) {
	params.clear();
	double scale = 0;
	int count = 0;

	for (int i = 0; i < robot->getJointCount(); i++) {
		for (int j = 0; j < 3; j++) {
			if (initialMorphology[i].pJPos[j] != 0) {
				scale += robot->getJoint(i)->pJPos[j] / initialMorphology[i].pJPos[j];
				count++;
			}
			if (initialMorphology[i].cJPos[j] != 0) {
				scale += robot->getJoint(i)->cJPos[j] / initialMorphology[i].cJPos[j];
				count++;
			}
		}
	}

	if (count > 0)
		scale /= count;

	params.push_back(scale);
}

void ParameterizedRobotDesign::setParameters(const DynamicArray<double>& params) {
	for (int i = 0; i < robot->getJointCount(); i++) {
		robot->getJoint(i)->pJPos = initialMorphology[i].pJPos * params[0];
		robot->getJoint(i)->cJPos = initialMorphology[i].cJPos * params[0];
	}
}


/*
#include <RobotDesignerLib/SpotMiniParamterizedDesign.h>
#include <RobotDesignerLib/GUILib/GLContentManager.h>

SpotMiniParameterizedDesign::SpotMiniParameterizedDesign(Robot* robot) : ParameterizedRobotDesign (robot){
//should probably check that this really is the morphology we expect...
currentParams.resize(4, 0);
currentParams.resize(12, 0);

}

SpotMiniParameterizedDesign::~SpotMiniParameterizedDesign() {
}

void SpotMiniParameterizedDesign::getCurrentSetOfParameters(DynamicArray<double>& params) {
params = currentParams;
}

void SpotMiniParameterizedDesign::setParameters(const DynamicArray<double>& params) {
currentParams = params;

//offset the positions of the hip joints, symetrically...

robot->getJoint(0)->pJPos.x() = initialMorphology[0].pJPos.x() + params[0];
robot->getJoint(1)->pJPos.x() = initialMorphology[1].pJPos.x() - params[0];
robot->getJoint(2)->pJPos.x() = initialMorphology[2].pJPos.x() + params[0];
robot->getJoint(3)->pJPos.x() = initialMorphology[3].pJPos.x() - params[0];

robot->getJoint(0)->pJPos.z() = initialMorphology[0].pJPos.z() + params[1];
robot->getJoint(1)->pJPos.z() = initialMorphology[1].pJPos.z() + params[1];
robot->getJoint(2)->pJPos.z() = initialMorphology[2].pJPos.z() - params[1];
robot->getJoint(3)->pJPos.z() = initialMorphology[3].pJPos.z() - params[1];

robot->getJoint(0)->pJPos.y() = initialMorphology[0].pJPos.y() + params[2];
robot->getJoint(1)->pJPos.y() = initialMorphology[1].pJPos.y() + params[2];
robot->getJoint(2)->pJPos.y() = initialMorphology[2].pJPos.y() + params[3];
robot->getJoint(3)->pJPos.y() = initialMorphology[3].pJPos.y() + params[3];

robot->getJoint(4)->pJPos.x() = initialMorphology[4].pJPos.x() + params[4];
robot->getJoint(5)->pJPos.x() = initialMorphology[5].pJPos.x() - params[4];
robot->getJoint(6)->pJPos.x() = initialMorphology[6].pJPos.x() + params[4];
robot->getJoint(7)->pJPos.x() = initialMorphology[7].pJPos.x() - params[4];

robot->getJoint(4)->pJPos.z() = initialMorphology[4].pJPos.z() + params[5];
robot->getJoint(5)->pJPos.z() = initialMorphology[5].pJPos.z() + params[5];
robot->getJoint(6)->pJPos.z() = initialMorphology[6].pJPos.z() - params[5];
robot->getJoint(7)->pJPos.z() = initialMorphology[7].pJPos.z() - params[5];

robot->getJoint(4)->pJPos.y() = initialMorphology[4].pJPos.y() + params[6];
robot->getJoint(5)->pJPos.y() = initialMorphology[5].pJPos.y() + params[6];
robot->getJoint(6)->pJPos.y() = initialMorphology[6].pJPos.y() + params[7];
robot->getJoint(7)->pJPos.y() = initialMorphology[7].pJPos.y() + params[7];

robot->getJoint(8)->pJPos.x() = initialMorphology[8].pJPos.x() + params[8];
robot->getJoint(9)->pJPos.x() = initialMorphology[9].pJPos.x() - params[8];
robot->getJoint(10)->pJPos.x() = initialMorphology[10].pJPos.x() + params[8];
robot->getJoint(11)->pJPos.x() = initialMorphology[11].pJPos.x() - params[8];

robot->getJoint(8)->pJPos.z() = initialMorphology[8].pJPos.z() + params[9];
robot->getJoint(9)->pJPos.z() = initialMorphology[9].pJPos.z() + params[9];
robot->getJoint(10)->pJPos.z() = initialMorphology[10].pJPos.z() - params[9];
robot->getJoint(11)->pJPos.z() = initialMorphology[11].pJPos.z() - params[9];

robot->getJoint(8)->pJPos.y() = initialMorphology[8].pJPos.y() + params[10];
robot->getJoint(9)->pJPos.y() = initialMorphology[9].pJPos.y() + params[10];
robot->getJoint(10)->pJPos.y() = initialMorphology[10].pJPos.y() + params[11];
robot->getJoint(11)->pJPos.y() = initialMorphology[11].pJPos.y() + params[11];

}
*/



