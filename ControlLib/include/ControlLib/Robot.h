#pragma once

#include <RBSimLib/RigidBody.h>
#include <RBSimLib/Joint.h>
#include <Utils/Utils.h>
#include <ControlLib/BodyFrame.h>

class RobotState;

/**
	Robots are articulated figures (i.e. tree structures starting at a root).
*/
class Robot {
	friend class RobotState;
	friend class LocomotionEngineMotionPlan;
	friend class GeneralizedCoordinatesRobotRepresentation;
public:
	RigidBody*	root = NULL;
	//keep a list of the joints of the virtual robot, for easy access
	DynamicArray<Joint*> jointList;

	//this is a list of auxiliary joints, created on demand. They can be used, for example, to create wheels or other specific function accessories for the robot...
	DynamicArray<Joint*> auxiliaryJointList;

	double		mass = 0;

	BodyFrame*	bFrame=NULL;

	//keep track of the forward and right directions of the robot...
	V3D			forward = V3D(1, 0, 0);
	V3D			right = V3D(0, 0, 1);

	/**
		This method is used to compute the center of mass of the robot.
	*/
	P3D computeCOM();

	/**
		This method is used to compute the velocity of the center of mass of the articulated figure.
	*/
	V3D computeCOMVelocity();

	/**
		the constructor
	*/
	Robot(RigidBody* root);

	/**
		the destructor
	*/
	~Robot(void);

	/**
		This method renders all the rigid bodies as a set of vertices
		and faces that will be appended to the passed OBJ file.

		vertexIdxOffset indicates the index of the first vertex for this object, this makes it possible to render
		multiple different meshes to the same OBJ file

		Returns the number of vertices written to the file
	*/
	void renderMeshesToFile(char* fName) {
		FILE* fp = fopen(fName, "w");
		uint vertexIdxOffset = 0;

		uint nbVerts = 0;
		for (int i = 0; i<getRigidBodyCount(); i++)
			nbVerts += getRigidBody(i)->renderToObjFile(fp, vertexIdxOffset + nbVerts);

		fclose(fp);
	}
	
	/**
		This method is used to populate the relative orientation of the parent and child bodies of joint i.
	*/
	Quaternion getRelativeOrientationForJoint(Joint* joint) {
		return joint->computeRelativeOrientation();
	}

	/**
		This method is used to get the relative angular velocities of the parent and child bodies of joint i,
		expressed in parent's local coordinates. 
	*/
	V3D getRelativeLocalCoordsAngularVelocityForJoint(Joint* joint) {
		//we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
		return joint->parent->getLocalCoordinates(joint->child->state.angularVelocity - joint->parent->state.angularVelocity);
	}

	void setRelativeOrientationForJoint(Joint* joint, const Quaternion& qRel) {
		joint->child->state.orientation = joint->parent->state.orientation * qRel;
	}

	void setRelativeLocalCoordsAngularVelocityForJoint(Joint* joint, const V3D& relAngVel) {
		//assume relAngVel is stored in the parent's coordinate frame, to get an orientation invariant expression for it
		joint->child->state.angularVelocity = joint->parent->state.angularVelocity + joint->parent->getWorldCoordinates(relAngVel);
	}

	inline int getJointCount() {
		return (int)jointList.size();
	}

	inline int getAuxiliaryJointCount() {
		return (int)auxiliaryJointList.size();
	}

	Joint* getJoint(int i) const{
		if (i < 0 || i > (int)jointList.size()-1)
			return NULL;
		return jointList[i];
	}

	Joint* getAuxiliaryJoint(int i) const {
		if (i < 0 || i >(int)auxiliaryJointList.size() - 1)
			return NULL;
		return auxiliaryJointList[i];
	}

	int getRigidBodyCount() {
		return (int)jointList.size() + 1;
	}

	/**
		returns a pointer to the ith rigid body of the virtual robot, where the root is at 0, and the rest follow afterwards...
	*/
	RigidBody* getRigidBody(int i) {
		if (i == 0)
			return root;
		if (i <= (int)jointList.size())
			return jointList[i - 1]->child;
		return NULL;
	}

	/**
		this method is used to read the reduced state of the robot from the file
	*/
	void loadReducedStateFromFile(const char* fName);

	/**
		this method is used to write the reduced state of the robot to a file
	*/
	void saveReducedStateToFile(const char* fName);

	/**
		uses the state of the robot to populate the input
	*/
	void populateState(RobotState* state, bool useDefaultAngles = false);

	/**
		sets the state of the robot using the input
	*/
	void setState(RobotState* state);

	/**
		makes sure the state of the robot is consistent with all the joint types...
	*/
	void fixJointConstraints();


	/**
		this method is used to return the current heading of the robot
	*/
	inline Quaternion getHeading(){
		return computeHeading(root->state.orientation, Globals::worldUp);
	}

	/**
		this method is used to return the current heading of the robot, specified as an angle measured in radians
	*/
	inline double getHeadingAngle(){
		return computeHeading(root->state.orientation, Globals::worldUp).getRotationAngle(Globals::worldUp);
	}

	/**
		this method is used to rotate the robot about the vertical axis, so that it's default heading has the value that is given as a parameter
	*/
	void setHeading(double val);


	/**
		this method is used to return a reference to the joint whose name is passed as a parameter, or NULL
		if it is not found.
	*/
	inline Joint* getJointByName(const char* jName){
		for (uint i=0;i<jointList.size();i++)
			if (strcmp(jointList[i]->name.c_str(), jName) == 0)
				return jointList[i];
		return NULL;
	}

	/**
		this method is used to return the index of the joint (whose name is passed as a parameter) in the articulated figure hierarchy.
	*/
	inline int getJointIndex(const char* jName){
		for (uint i=0;i<jointList.size();i++)
			if (strcmp(jointList[i]->name.c_str(), jName) == 0)
				return i;
		return -1;
	}

	/**
		this method is used to return a reference to the articulated figure's rigid body whose name is passed in as a parameter, 
		or NULL if it is not found.
	*/
	RigidBody* getRBByName(const char* jName);

	/**
		returns the root of the current articulated figure.
	*/
	inline RigidBody* getRoot(){
		return root;
	}

	inline void draw(int flags) {
		//draw the robot configuration...
		getRoot()->draw(flags);
		for (int i = 0; i<getJointCount(); i++)
			getJoint(i)->child->draw(flags);
	}

	double getMass(){
		return mass;
	}

	virtual void addWheelsAsAuxiliaryRBs(AbstractRBEngine* rbEngine);

};

class JointState {
public:
	Quaternion qRel;
	V3D angVelRel;
};

class RobotState{
private:
	Quaternion rootQ;
	P3D rootPos;
	V3D rootVel;
	V3D rootAngVel;

	//to compute headings, we need to know which axis defines it (the yaw)
	V3D headingAxis = V3D(0, 1, 0);

	DynamicArray<JointState> joints;
	DynamicArray<JointState> auxiliaryJoints;

public:
	~RobotState() {
	}

	RobotState(int jCount = 0, int aJCount = 0){
		joints.resize(jCount);
		auxiliaryJoints.resize(aJCount);
	}

	RobotState(Robot* robot, bool useDefaultAngles = false){
		robot->populateState(this, useDefaultAngles);
	}

	void setJointCount(int jCount) {
		joints.resize(jCount);
	}

	void setAuxiliaryJointCount(int jCount) {
		auxiliaryJoints.resize(jCount);
	}

	void setHeadingAxis(V3D v){
		headingAxis = v;
	}

	V3D getHeadingAxis() {
		return headingAxis;
	}

	int getJointCount() const{
		return joints.size();
	}

	int getAuxiliaryJointCount() const {
		return auxiliaryJoints.size();
	}

	P3D getPosition() const{
		return rootPos;
	}

	void setPosition(P3D p){
		rootPos = p;
	}

	Quaternion getOrientation() const{
		return rootQ;
	}

	void setOrientation(Quaternion q){
		rootQ = q;
	}

	V3D getVelocity() const{
		return rootVel;
	}

	void setVelocity(V3D v){
		rootVel = v;
	}

	V3D getAngularVelocity() const{
		return rootAngVel;
	}

	void setAngularVelocity(V3D v){
		rootAngVel = v;
	}
	
	Quaternion getJointRelativeOrientation(int jIndex) const{
		if ((uint)jIndex < joints.size())
			return joints[jIndex].qRel;
	//	exit(0);
		return Quaternion();
	}

	V3D getJointRelativeAngVelocity(int jIndex) const {
		if ((uint)jIndex < joints.size())
			return joints[jIndex].angVelRel;
	//	exit(0);
		return V3D();
	}

	void setJointRelativeOrientation(Quaternion q, int jIndex){
		if ((uint)jIndex < joints.size())
			joints[jIndex].qRel = q;
	//	else
	//		exit(0);

	}

	void setJointRelativeAngVelocity(V3D w, int jIndex) {
		if ((uint)jIndex < joints.size())
			joints[jIndex].angVelRel = w;
	//	else
	//		exit(0);
	}

	Quaternion getAuxiliaryJointRelativeOrientation(int jIndex) const {
		if ((uint)jIndex < auxiliaryJoints.size())
			return auxiliaryJoints[jIndex].qRel;
	//	exit(0);
		return Quaternion();
	}

	V3D getAuxiliaryJointRelativeAngVelocity(int jIndex) const {
		if ((uint)jIndex < auxiliaryJoints.size())
			return auxiliaryJoints[jIndex].angVelRel;
	//	exit(0);
		return V3D();
	}

	void setAuxiliaryJointRelativeOrientation(Quaternion q, int jIndex) {
		if ((uint)jIndex < auxiliaryJoints.size())
			auxiliaryJoints[jIndex].qRel = q;
	//	else
	//		exit(0);

	}

	void setAuxiliaryJointRelativeAngVelocity(V3D w, int jIndex) {
		if ((uint)jIndex < auxiliaryJoints.size())
			auxiliaryJoints[jIndex].angVelRel = w;
	//	else
	//		exit(0);
	}

	void writeToFile(const char* fName, Robot* robot = NULL) {
		if (fName == NULL)
			throwError("cannot write to a file whose name is NULL!");

		FILE* fp = fopen(fName, "w");

		if (fp == NULL)
			throwError("cannot open the file \'%s\' for reading...", fName);

		V3D velocity = getVelocity();
		Quaternion orientation = getOrientation();
		V3D angVelocity = getAngularVelocity();
		P3D position = getPosition();

		double heading = getHeading();
		//setHeading(0);

		fprintf(fp, "# order is:\n# Heading Axis\n# Heading\n# Position\n# Orientation\n# Velocity\n# AngularVelocity\n\n# Relative Orientation\n# Relative Angular Velocity\n#----------------\n\n# Heading Axis\n %lf %lf %lf\n# Heading\n%lf\n\n", headingAxis[0], headingAxis[1], headingAxis[2], heading);

		if (robot != NULL)
			fprintf(fp, "# Root(%s)\n", robot->root->name.c_str());

		fprintf(fp, "%lf %lf %lf\n", position[0], position[1], position[2]);
		fprintf(fp, "%lf %lf %lf %lf\n", orientation.s, orientation.v[0], orientation.v[1], orientation.v[2]);
		fprintf(fp, "%lf %lf %lf\n", velocity[0], velocity[1], velocity[2]);
		fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1], angVelocity[2]);

		fprintf(fp, "# number of joints\n%d\n\n", getJointCount());

		for (int i = 0; i < getJointCount(); i++) {
			orientation = getJointRelativeOrientation(i);
			angVelocity = getJointRelativeAngVelocity(i);
			if (robot != NULL)
				fprintf(fp, "# %s\n", robot->jointList[i]->name.c_str());
			fprintf(fp, "%lf %lf %lf %lf\n", orientation.s, orientation.v[0], orientation.v[1], orientation.v[2]);
			fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1], angVelocity[2]);
		}

		fprintf(fp, "# number of auxiliary joints\n%d\n\n", getAuxiliaryJointCount());

		for (int i = 0; i < getAuxiliaryJointCount(); i++) {
			orientation = getAuxiliaryJointRelativeOrientation(i);
			angVelocity = getAuxiliaryJointRelativeAngVelocity(i);
			if (robot != NULL)
				fprintf(fp, "# %s\n", robot->auxiliaryJointList[i]->name.c_str());
			fprintf(fp, "%lf %lf %lf %lf\n", orientation.s, orientation.v[0], orientation.v[1], orientation.v[2]);
			fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1], angVelocity[2]);
		}

		fclose(fp);
		//now restore the state of this reduced state...
		//setHeading(heading);
	}


	void readFromFile	(const char* fName) {
		if (fName == NULL)
			throwError("cannot read a file whose name is NULL!");

		FILE* fp = fopen(fName, "r");
		if (fp == NULL)
			throwError("cannot open the file \'%s\' for reading...", fName);

		double temp1, temp2, temp3, temp4;

		char line[100];

		//read the heading first...
		double heading;
		readValidLine(line, 100, fp);
		sscanf(line, "%lf %lf %lf", &headingAxis[0], &headingAxis[1], &headingAxis[2]);

		readValidLine(line, 100, fp);
		sscanf(line, "%lf", &heading);

		readValidLine(line, 100, fp);
		sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
		setPosition(P3D(temp1, temp2, temp3));
		readValidLine(line, 100, fp);
		sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
		setOrientation(Quaternion(temp1, temp2, temp3, temp4).toUnit());
		readValidLine(line, 100, fp);
		sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
		setVelocity(V3D(temp1, temp2, temp3));
		readValidLine(line, 100, fp);
		sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
		setAngularVelocity(V3D(temp1, temp2, temp3));

		int jCount = 0;

		readValidLine(line, 100, fp);
		sscanf(line, "%d", &jCount);
		joints.resize(jCount);

		for (int i = 0; i<jCount; i++) {
			readValidLine(line, 100, fp);
			sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
			setJointRelativeOrientation(Quaternion(temp1, temp2, temp3, temp4).toUnit(), i);
			readValidLine(line, 100, fp);
			sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
			setJointRelativeAngVelocity(V3D(temp1, temp2, temp3), i);
		}

		int auxJCount = 0;

		readValidLine(line, 100, fp);
		sscanf(line, "%d", &auxJCount);
		auxiliaryJoints.resize(auxJCount);

		for (int i = 0; i<auxJCount; i++) {
			readValidLine(line, 100, fp);
			sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
			setAuxiliaryJointRelativeOrientation(Quaternion(temp1, temp2, temp3, temp4).toUnit(), i);
			readValidLine(line, 100, fp);
			sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
			setAuxiliaryJointRelativeAngVelocity(V3D(temp1, temp2, temp3), i);
		}

		//now set the heading...
		setHeading(heading);

		fclose(fp);
	}

	//setting the heading...
	void setHeading(double heading) {
		//this means we must rotate the angular and linear velocities of the COM, and augment the orientation
		Quaternion oldHeading, newHeading, qRoot;
		//get the current root orientation, that contains information regarding the current heading
		qRoot = getOrientation();
		//get the twist about the vertical axis...
		oldHeading = computeHeading(qRoot, headingAxis);
		//now we cancel the initial twist and add a new one of our own choosing
		newHeading = getRotationQuaternion(heading, headingAxis) * oldHeading.getComplexConjugate();
		//add this component to the root.
		setOrientation(newHeading * qRoot);
		//and also update the root velocity and angular velocity
		setVelocity(newHeading.rotate(getVelocity()));
		setAngularVelocity(newHeading.rotate(getAngularVelocity()));
	}

	double getHeading(){
		//first we need to get the current heading of the robot. 
		return computeHeading(getOrientation(), headingAxis).getRotationAngle(headingAxis);
	}

	bool isSameAs(const RobotState& other) {
		if (getJointCount() != other.getJointCount()) {
//			Logger::consolePrint("jCount: %d vs %d\n", getJointCount(), other.getJointCount());
			return false;
		}

		if (V3D(getPosition(), other.getPosition()).length() > TINY) {
//			Logger::consolePrint("pos: %lf %lf %lf vs %lf %lf %lf\n", 
//				getPosition().x(), getPosition().y(), getPosition().z(), 
//				other.getPosition().x(), other.getPosition().y(), other.getPosition().z());
			return false;
		}

		Quaternion q1 = getOrientation();
		Quaternion q2 = other.getOrientation();

		if (q1 != q2 && q1 != (q2 * -1)) {
//			Logger::consolePrint("orientation: %lf %lf %lf %lf vs %lf %lf %lf %lf\n", q1.s, q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(), q2.v.y(), q2.v.z());
			return false;
		}

		if ((getVelocity() - other.getVelocity()).length() > TINY) {
//			Logger::consolePrint("vel: %lf %lf %lf vs %lf %lf %lf\n",
//				getVelocity().x(), getVelocity().y(), getVelocity().z(),
//				other.getVelocity().x(), other.getVelocity().y(), other.getVelocity().z());
			return false;
		}

		if ((getAngularVelocity() - other.getAngularVelocity()).length() > TINY) {
//			Logger::consolePrint("ang vel: %lf %lf %lf vs %lf %lf %lf\n",
//				getAngularVelocity().x(), getAngularVelocity().y(), getAngularVelocity().z(),
//				other.getAngularVelocity().x(), other.getAngularVelocity().y(), other.getAngularVelocity().z());
			return false;
		}



		for (int i = 0; i < getJointCount(); i++) {
			if ((getJointRelativeAngVelocity(i) - other.getJointRelativeAngVelocity(i)).length() > TINY) {
//				Logger::consolePrint("joint %d ang vel: %lf %lf %lf vs %lf %lf %lf\n", i,
//					getJointRelativeAngVelocity(i).x(), getJointRelativeAngVelocity(i).y(), getJointRelativeAngVelocity(i).z(),
//					other.getJointRelativeAngVelocity(i).x(), other.getJointRelativeAngVelocity(i).y(), other.getJointRelativeAngVelocity(i).z());
				return false;
			}

			Quaternion q1 = getJointRelativeOrientation(i);
			Quaternion q2 = other.getJointRelativeOrientation(i);

			if (q1 != q2 && q1 != (q2 * -1)) {
//				Logger::consolePrint("joint %d orientation: %lf %lf %lf %lf vs %lf %lf %lf %lf\n", i, q1.s, q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(), q2.v.y(), q2.v.z());
				return false;
			}
		}

		return true;
	}

};

void setupSimpleRobotStructure(Robot* robot);



