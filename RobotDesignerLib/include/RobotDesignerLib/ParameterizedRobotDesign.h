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

	JointParameters() {

	}

};

class EEParameters {
public:
	P3D initialCoords;

	EEParameters(RBEndEffector* ree) {
		initialCoords = ree->coords;
	}

	EEParameters() {

	}
};

/**
Given a set of parameters, instances of this class will output new robot morphologies.
*/

class ParameterizedRobotDesign {
public:
	Robot* robot;
	ReducedRobotState defaultRobotState;

	//the morphology of the design is given by the parameterization of each joint (e.g. position in child and parent coords, rotation axis, etc).
	//we will store the initial morphology here

	map<Joint*, JointParameters> initialJointMorphology;
	map<RBEndEffector*, EEParameters> initialEEMorphology;

	ParameterizedRobotDesign(Robot* robot);
	virtual ~ParameterizedRobotDesign();
	virtual int getNumberOfParameters() = 0;
	virtual void getCurrentSetOfParameters(DynamicArray<double>& params) = 0;
	virtual void setParameters(const DynamicArray<double>& params) = 0;

	void updateMorphology();

	virtual void getCurrentSetOfParameters(dVector& params) {
		DynamicArray<double> tempparams;
		getCurrentSetOfParameters(tempparams);
		params = Eigen::Map<dVector>(tempparams.data(), tempparams.size());
	}

	virtual void setParameters(const dVector& params) {
		assert(currentParams.size() == params.size());

		DynamicArray<double> tempparams(params.size());

		dVector::Map(&tempparams[0], tempparams.size()) = params;

		setParameters(tempparams);
	}
	
};


struct ModifiableParam {
	int index;
	double xModifier;
	ModifiableParam(int index, double zModifier) {
		this->index = index;
		this->xModifier = zModifier;
	}

	ModifiableParam() {
		this->index = -1;
		this->xModifier = 0;
	}

};

/*
	This class will parameterize the position of each joint, as well as the length of each bone (body not included). Symmetry is taken into account (based on a plane of symmetry)

	TODO1: I suppose we want end effectors to be parameterized sorta like joints are... will it even work to sync with design function though? Or is it possible that pairs of translations for parent joints have a similar effect?
	TODO2: bone scales....
	TODO3: if we were to move end effectors, then we'd have to update collision detection primitives too...
	
*/
class SymmetricParameterizedRobotDesign : public ParameterizedRobotDesign {
private:
	DynamicArray<double> currentParams;

	map<Joint*, Joint*> jointMirrorMap;
	map<RigidBody*, RigidBody*> boneMirrorMap;

	//we will use the two maps below both to figure out how to interpret the parameters (e.g. which parameters mean what) as well as to know, for a selected bone or joint, where to write parameters that might be input in an indirect way via a GUI

public:
	//tells you where in the parameter list can we find the params for each joint (translational offsets per joint)
	map<Joint*, ModifiableParam> jointParamMap;
	//tells you where in the parameter list can we find the params that control the end effector placements (translational offsets)
	map<RigidBody*, ModifiableParam> eeParamMap;

	using ParameterizedRobotDesign::getCurrentSetOfParameters;
	using ParameterizedRobotDesign::setParameters;


public:
	SymmetricParameterizedRobotDesign(Robot* robot) : ParameterizedRobotDesign(robot) {

		//we'll compute mirror maps in a naive way... but it only has to be done once...
		for (int i = 0; i < robot->getJointCount(); i++) {
			P3D pos1 = robot->jointList[i]->getWorldPosition();
			for (int j = i + 1; j < robot->getJointCount(); j++) {
				P3D pos2 = robot->jointList[j]->getWorldPosition();
				pos2.x() *= -1;
				if (IS_ZERO(V3D(pos1, pos2).length())) {
					Logger::print("joints %d and %d are symmetric...\n", i, j);
					//we found a symmetric pair...
					jointMirrorMap[robot->jointList[i]] = robot->jointList[j];
					jointMirrorMap[robot->jointList[j]] = robot->jointList[i];
				}
			}
		}

		//we'll compute mirror maps in a naive way... but it only has to be done once...
		for (int i = 0; i < robot->getRigidBodyCount(); i++) {
			P3D pos1 = robot->getRigidBody(i)->getWorldCoordinates(P3D());
			for (int j = i + 1; j < robot->getRigidBodyCount(); j++) {
				P3D pos2 = robot->getRigidBody(j)->getWorldCoordinates(P3D());
				pos2.x() *= -1;
				if (IS_ZERO(V3D(pos1, pos2).length())) {
					Logger::print("bones %d and %d are symmetric...\n", i, j);
					//we found a symmetric pair...
					boneMirrorMap[robot->getRigidBody(i)] = robot->getRigidBody(j);
					boneMirrorMap[robot->getRigidBody(j)] = robot->getRigidBody(i);
				}
			}
		}

		//for every rigid body that has at least one end effector, add parameters that control its location in local coordinates
		for (int i = 1; i < robot->getRigidBodyCount(); i++) {
			if (robot->getRigidBody(i)->rbProperties.endEffectorPoints.size() == 0)
				continue;

			if (boneMirrorMap.count(robot->getRigidBody(i))) {
				RigidBody* mirrorBone = boneMirrorMap[robot->getRigidBody(i)];
				//if we've already pushed a parameter for this bone's mirror, then make sure we're using the same index
				if (eeParamMap.count(mirrorBone))
					eeParamMap[robot->getRigidBody(i)] = eeParamMap[mirrorBone];
			}
			if (!eeParamMap.count(robot->getRigidBody(i))) {
				//create a new set of parameters for this joint specifically, and point to it...
				eeParamMap[robot->getRigidBody(i)] = ModifiableParam(currentParams.size(), 1);
				currentParams.push_back(0);	currentParams.push_back(0); currentParams.push_back(0);
			}
			if (robot->getRigidBody(i)->getWorldCoordinates(P3D()).x() < 0)
				eeParamMap[robot->getRigidBody(i)].xModifier = -1;
			else
				eeParamMap[robot->getRigidBody(i)].xModifier = 1;
		}

		//now, for every joint we will have 3 parameters that correspond to its offset...
		for (int i = 0; i < robot->getJointCount(); i++) {
			if (jointMirrorMap.count(robot->jointList[i])) {
				Joint* mirrorJoint = jointMirrorMap[robot->jointList[i]];
				//if we've already pushed parameters for this joint's mirror, then make sure we're using the same index
				if (jointParamMap.count(mirrorJoint))
					jointParamMap[robot->jointList[i]] = jointParamMap[mirrorJoint];
			}
			if (!jointParamMap.count(robot->jointList[i])){
				//if we haven't found a mirror, or if the mirror is not yet mapped, create a new set of parameters for this joint specifically, and point to it...
				jointParamMap[robot->jointList[i]] = ModifiableParam(currentParams.size(), 1);
				//these correspond to a change only in pJPos (equivalent to propagating these changes to the entire rest of the structure)
				currentParams.push_back(0);	currentParams.push_back(0); currentParams.push_back(0);
				//these are equivalent to a change both in pJPos and cJPos, which is then equivalent to moving just the joint, and not the rest of the structure
				currentParams.push_back(0);	currentParams.push_back(0); currentParams.push_back(0);
			}

			if (robot->jointList[i]->getWorldPosition().x() < 0)
				jointParamMap[robot->jointList[i]].xModifier = -1;
			else
				jointParamMap[robot->jointList[i]].xModifier = 1;
		}

		//and so... all done...
	}

	int getNumberOfParameters() { return currentParams.size(); }

	virtual void getCurrentSetOfParameters(DynamicArray<double>& params) {
		params = currentParams;
	}

	virtual void setParameters(const DynamicArray<double>& params) {
		currentParams = params;

		ReducedRobotState rs(robot);
		robot->setState(&defaultRobotState);

		// go through each joint and apply its offset as specified in the param list...
		for (int i = 0; i < robot->getRigidBodyCount(); i++) {
			if (!eeParamMap.count(robot->getRigidBody(i)))
				continue;

			int pIndex = eeParamMap[robot->getRigidBody(i)].index;

			V3D eeOffset(params[pIndex + 0] * eeParamMap[robot->getRigidBody(i)].xModifier, params[pIndex + 1], params[pIndex + 2]);

			for (uint j = 0; j < robot->getRigidBody(i)->rbProperties.endEffectorPoints.size(); j++) {
				robot->getRigidBody(i)->rbProperties.endEffectorPoints[j].coords = initialEEMorphology[&robot->getRigidBody(i)->rbProperties.endEffectorPoints[j]].initialCoords + eeOffset;
			}
		}

		for (int i = 0; i < robot->getJointCount(); i++) {
			int pIndex = jointParamMap[robot->jointList[i]].index;

//			Logger::print("joint: %d has param indices start at: %d\n", i, pIndex);

			V3D jointOffset1(params[pIndex + 0] * jointParamMap[robot->jointList[i]].xModifier, params[pIndex + 1], params[pIndex + 2]);
			V3D jointOffset2(params[pIndex + 3] * jointParamMap[robot->jointList[i]].xModifier, params[pIndex + 4], params[pIndex + 5]);

			robot->jointList[i]->pJPos = initialJointMorphology[robot->jointList[i]].pJPos + jointOffset1;
			robot->jointList[i]->cJPos = initialJointMorphology[robot->jointList[i]].cJPos + jointOffset2;
		}
		robot->setState(&rs);
	}

};
