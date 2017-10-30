#pragma once
#include <MathLib/V3D.h>
#include <vector>

#include <MathLib/MathLib.h>
#include <MathLib/Quaternion.h>

#include <ControlLib/Robot.h>

using namespace std;

class JointParameters {
public:
	P3D pJPos, cJPos;

	JointParameters(Joint* j) {
		pJPos = j->pJPos;
		cJPos = j->cJPos;
	}

};

/**
Given a set of parameters, instances of this class will output new robot morphologies.
*/

class ParameterizedRobotDesign {
public:
	Robot* robot;

	//the morphology of the design is given by the parameterization of each joint (e.g. position in child and parent coords, rotation axis, etc).
	//we will store the initial morphology here
	DynamicArray<JointParameters> initialMorphology;

	ParameterizedRobotDesign(Robot* robot);
	~ParameterizedRobotDesign();
	virtual int getNumberOfParameters() = 0;
	virtual void getCurrentSetOfParameters(DynamicArray<double>& params) = 0;
	virtual void setParameters(const DynamicArray<double>& params) = 0;
};


class TestParameterizedRobotDesign : public ParameterizedRobotDesign {
public:
	DynamicArray<double> currentParams;
	TestParameterizedRobotDesign(Robot* robot) : ParameterizedRobotDesign(robot) {
		//should probably check that this really is the morphology we expect...
		currentParams.resize(14, 0);
	}

	~TestParameterizedRobotDesign() {
	}

	int getNumberOfParameters() { return currentParams.size(); }
	void getCurrentSetOfParameters(DynamicArray<double>& params) {
		params = currentParams;
	}
	void getCurrentSetOfParameters(dVector& params) {
		params = Eigen::Map<dVector>(currentParams.data(),currentParams.size());
	}
	void setParameters(const DynamicArray<double>& params) {
		currentParams = params;

		//offset the positions of the hip joints, symetrically...

		updatePositions(params);
	}

	void setParameters(const dVector& params) {
		assert(currentParams.size() == params.size());
		dVector::Map(&currentParams[0], params.size()) = params;
		//offset the positions of the hip joints, symetrically...
		updatePositions(currentParams);
	}

	void updatePositions(const DynamicArray<double>& params)
	{
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

		robot->getJoint(8)->cJPos.y() = initialMorphology[8].cJPos.y() + params[12];
		robot->getJoint(9)->cJPos.y() = initialMorphology[9].cJPos.y() + params[12];
		robot->getJoint(10)->cJPos.y() = initialMorphology[10].cJPos.y() + params[13];
		robot->getJoint(11)->cJPos.y() = initialMorphology[11].cJPos.y() + params[13];
	}
};


