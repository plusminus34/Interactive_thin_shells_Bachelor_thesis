#pragma once
#include <RobotDesignerLib/RMC.h>
#include <RobotDesignerLib/RMCJoint.h>
#include <RBSimLib/ODERBEngine.h>
#include <ControlLib/Robot.h>

class RMCRobot;

struct PossibleConnection
{
	P3D position;
	Quaternion orientation;
	RMCRobot* parentRobot;
	RMCPin* parentPin;
	RMCPin* childPin;
};

class RMCRobot
{
public:
	RMC* root = NULL;
	//keep a list of the joints of the virtual robot, for easy access
	vector<RMCJoint*> jointList;

	map<string, vector<Transformation>>& transformationMap;

	// an indicator that joints and rmcs are taken over by other RMCRobot, so no memory free in the destructor
	bool disposed = false;

	RMC* highlightedRMC = NULL;
	RMC* selectedRMC = NULL;
	RMCPin* highlightedPin = NULL;
	RMCPin* selectedPin = NULL;

public:
	RMCRobot(map<string, vector<Transformation>>& _transformationMap);
	RMCRobot(RMC* _root, map<string, vector<Transformation>>& _transformationMap);
	~RMCRobot();

	void fixJointConstraints(bool ignoreMotorAngle = false);
	void fixPlateStateByMotor();

	void draw(int flags, const Vector4d& root_color = Vector4d(0, 0, 0, 0), const Vector4d& highlight_color = Vector4d(0, 0, 0, 0), const Vector4d& color = Vector4d(0, 0, 0, 0));

	bool pickPin(Ray& ray);
	void clearPinPick();

	// if ray intersect with any RMC, return the distance to ray's origin; else, return -1
	bool pickRMC(Ray& ray, double* closestDist = NULL);
	
	// connect child RMCRobot by pins, parentPin belongs to RMC that already in the robot, while childPin belong to the RMCRobot to be connected.
	bool connectRMCRobot(RMCRobot* child, RMCPin* parentPin, RMCPin* childPin, int relTransId = 0);

	// connect child RMCRobot directly, only used for assemble different body parts.
	bool connectRMCRobotDirectly(RMCRobot* child, RMC* parentRMC, int relTransId = 0);

	// connect child RMC by pins
	bool connectRMC(RMC* child, RMCPin* parentPin, RMCPin* childPin, int relTransId = 0);

	// get all pins in the robot that can be connected to the candidate pin.
	void getAvailableCompatiblePins(RMCPin* candidatePin, vector<RMCPin*>& availablePins);

	// delete the sub tree of the joint
	void deleteSubTree(RMCJoint* joint, bool subRoot = true);

	// clone a new robot
	RMCRobot* clone();
	RMCRobot* cloneSubTree(RMC* rmc);

	void exportMeshes(const char* fName, const char* carvefName);

	// save to file (different from save to .rbs file)
	void saveToFile(const char* fName);
	void saveToFile(FILE* fp);

	// save to .rbs file
	ReducedRobotState saveToRBSFile(const char* fName, Robot* templateRobot = NULL, bool freezeRoot = false, bool mergeMeshes = false, bool forFabrication = false);
	void getRMCToRBIndexMap(RMC* node, int curIndex, int& RBIndex, map<RMC*, int>& RBIndexMap);

	// get mesh vertice for rigid bodies
	void getMeshVerticesForRBs(Robot* templateRobot, map<RigidBody*, vector<P3D>>& rbVertices);

	// load from file, rmcNameMap is for cloning RMC from the rmcWarehouse
	void loadFromFile(const char* fName, map<string, RMC*>& rmcNameMap);
	void loadFromFile(FILE* fp, map<string, RMC*>& rmcNameMap);

	void addJoint(RMCJoint* joint) {
		jointList.push_back(joint);
	}

	void removeJoint(RMCJoint* joint) {
		for (uint i = 0; i < jointList.size(); i++)
		{
			if (jointList[i] == joint)
			{
				jointList.erase(jointList.begin() + i);
				break;
			}
		}
	}

	/**
	This method is used to return the number of joints of the robot.
	*/
	int getJointCount() {
		return (int)jointList.size();
	}

	RMCJoint* getJoint(uint index) {
		return jointList[index];
	}

	/**
	this method is used to return a reference to the articulated figure's rigid body whose name is passed in as a parameter,
	or NULL if it is not found.
	*/
	RMC* getRMCByName(const char* jName) {
		for (uint i = 0; i<jointList.size(); i++) {
			if (strcmp(jointList[i]->parent->name.c_str(), jName) == 0)
				return jointList[i]->getParent();
			if (strcmp(jointList[i]->child->name.c_str(), jName) == 0)
				return jointList[i]->getChild();
		}
		return NULL;
	}

	/**
	returns the root of the current articulated figure.
	*/
	RMC* getRoot() {
		return root;
	}

	int getRMCCount() {
		return (int)jointList.size() + 1;
	}

	RMC* getRMC(int i) {
		return i == 0 ? root : jointList[i-1]->getChild();
	}

	void addBulletObjectsToList(DynamicArray<AbstractBulletObject*>& list);

	void updateAllLivingMotor();

protected:
	void cloneSubTreeHelper(RMC* rmc, map<RMC*, RMC*>& rmcMap);


};

