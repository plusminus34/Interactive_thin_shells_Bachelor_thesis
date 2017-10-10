#pragma once

#include <RBSimLib/RigidBody.h>
#include <RBSimLib/Joint.h>
#include <Utils/Utils.h>
#include <ControlLib\BodyFrame.h>

class ReducedRobotState;

/**
	Robots are articulated figures (i.e. tree structures starting at a root).
*/
class Robot {
	friend class ReducedRobotState;
	friend class LocomotionEngineMotionPlan;
	friend class GeneralizedCoordinatesRobotRepresentation;
public:
	RigidBody*	root = NULL;
	//keep a list of the joints of the virtual robot, for easy access
	DynamicArray<Joint*> jointList;
	double		mass = 0;

	BodyFrame*	bFrame=NULL;

	//keep track of the forward and right directions of the robot...
	V3D			forward = V3D(1, 0, 0);
	V3D			right = V3D(0, 0, 1);

	///< Assuming hinge joints !
	int getDim() { return getJointCount() + 6; }

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
		This method is used to populate the relative orientation of the parent and child bodies of joint i.
	*/
	void getRelativeOrientationForJoint(int i, Quaternion* qRel);

	/**
		This method is used to get the relative angular velocities of the parent and child bodies of joint i,
		expressed in parent's local coordinates. 
	*/
	void getRelativeAngularVelocityForJoint(int i, V3D* wRel);

	/*
		* Maybe make a robot for HingeOnlyRobot.
	*/
	/*
		* For hinge joints only, retrieves the relative angle of the joint using the reduced robot state (not the state of the current robot).
	*/
	double	getJointRelativeAngle	(ReducedRobotState* _pState, int _jIndex) const;
	V3D		getJointAxis			(const int _j) const;

	/**
	*	Retrieves the relative angles of the joint using the reduced robot states.
	*	Depending on the number of degrees of freedom the value for angle1, angle2 and angle3 
	*	is set either to the corresponding angle or NULL if the joint has less degrees of freedom.
	*/
	void getJointRelativeAngles(ReducedRobotState* _pState, int jIndex, DynamicArray<double>& angles);


	/**
		Returns a pointer to the ith joint of the virtual robot
	*/
	Joint* getJoint(int i) const{
		if (i < 0 || i > (int)jointList.size()-1)
			return NULL;
		return jointList[i];
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
		Creates a linear list of rigid bodies contained in the robot.  
		Note: consider the ordering of rigid bodies.
	*/
	DynamicArray<RigidBody*> getBodies();

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
	void populateState(ReducedRobotState* state);

	/**
		sets the state of the robot using the input
	*/
	void setState(ReducedRobotState* state);

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
		This method is used to return the number of joints of the robot.
	*/
	inline int getJointCount(){
		return (int)jointList.size();
	}

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

	std::vector<RigidBody*> getEndEffectorRBs();

	int getEndEffectorCount();

	P3D getEndEffectorWorldPosition(int _i);

	double getEndEffectorRadius(int _i);

	/**
		returns the root of the current articulated figure.
	*/
	inline RigidBody* getRoot(){
		return root;
	}

	double getMass(){
		return mass;
	}

	/**
		This method returns the dimension of the state.
	*/
	inline int getReducedStateDimension() {
		//13 (x, v, theta, d_theta/dt for the root, and 7 for every other body (and each body is introduced by a joint).
		return 13 + 7 * (int)jointList.size();
	}

	/**
		This method is used to save the RBS corresponding to a virtual robot to file.
	*/
	void saveRBSToFile(char* fName);

	double getApproxBodyFrameHeight();

};

class ReducedRobotState{
private:
	DynamicArray<double> state;
	//to compute headings, we need to know which axis defines it (the yaw)
	V3D headingAxis;

	int getOffsetForJoint(int jIndex) const{
		return 13 + 7 * jIndex;
	}

public:

	~ReducedRobotState() {
		state.clear();
	}

	ReducedRobotState(int stateSize){
		state = DynamicArray<double>(stateSize);
		setPosition(P3D());
		setVelocity(V3D());
		setOrientation(Quaternion());
		setAngularVelocity(V3D());
		int jCount = getJointCount();
		for (int i = 0; i < jCount; i++){
			setJointRelativeAngVelocity(V3D(), i);
			setJointRelativeOrientation(Quaternion(), i);
		}
		headingAxis = V3D(0,1,0);
	}

	int getStateSize() const{
		return state.size();
	}


	ReducedRobotState(Robot* robot){
		state = DynamicArray<double>(robot->getReducedStateDimension());
		robot->populateState(this);
	}

	ReducedRobotState(ReducedRobotState* start, ReducedRobotState* end, double t);

	void setJointsToZero()
	{
		for (int i = 0; i < getJointCount(); ++i)
			setJointRelativeOrientation(Quaternion::ExpMap(V3D(0, 0, 0)), i);
	}

	void setHeadingAxis(V3D v){
		headingAxis = v;
	}

	V3D getHeadingAxis() {
		return headingAxis;
	}

	int getJointCount(){
		return (getStateSize() - 13) / 7;
	}

	DynamicArray<double>& getState(){
		return state;
	}

	void getState(dVector& _state) {
		if (_state.size() != state.size())
			_state.resize(state.size());

		for (int i = 0; i < _state.size(); ++i)
			_state[i] = state[i];

	}	

	void setState(const DynamicArray<double>& otherState){
		if ((int)otherState.size() != getStateSize())
			throwError("incompatible state sizes");
		for (int i=0;i<getStateSize();i++)
			state[i] = otherState[i];
	}

	void setState(const dVector& otherState) {
		if ((int)otherState.size() != getStateSize())
			throwError("incompatible state sizes");
		for (int i = 0; i<getStateSize(); i++)
			state[i] = otherState[i];
	}

	/**
	 *		 gets the root position.
	 */
	P3D getPosition() const{
		return P3D(state[0], state[1], state[2]);
	}

	/**
	 *		 sets the root position.
	 */
	void setPosition(P3D p){
		state[0] = p[0];
		state[1] = p[1];
		state[2] = p[2];
	}

	/**
	 *		 gets the root orientation.
	 */
	Quaternion getOrientation() const{
		return Quaternion(state[3], state[4], state[5], state[6]);
	}

	/**
	 *		 sets the root orientation.
	 */
	void setOrientation(Quaternion q){
		state[3] = q.s;
		state[4] = q.v[0];
		state[5] = q.v[1];
		state[6] = q.v[2];
	}

	/**
	 *		 gets the root velocity.
	 */
	V3D getVelocity() const{
		return V3D(state[7], state[8], state[9]);
	}

	/**
	 *		 sets the root velocity.
	 */
	void setVelocity(V3D v){
		state[7] = v[0];
		state[8] = v[1];
		state[9] = v[2];
	}

	/**
	 *		 gets the root angular velocity.
	*/
	V3D getAngularVelocity() const{
		return V3D(state[10], state[11], state[12]);
	}

	/**
	 *		 sets the root angular velocity.
	 */
	void setAngularVelocity(V3D v){
		state[10] = v[0];
		state[11] = v[1];
		state[12] = v[2];
	}
	
	/**
	 *		 gets the relative orientation for joint jIndex
	 */
	Quaternion getJointRelativeOrientation(int jIndex) const{
		int offset = getOffsetForJoint(jIndex);
		return Quaternion(state[0 + offset], state[1 + offset], state[2 + offset], state[3 + offset]);
	}


	/**
	 *		 sets the orientation for joint jIndex
	 */
	void setJointRelativeOrientation(Quaternion q, int jIndex){
		int offset = getOffsetForJoint(jIndex);
		state[0 + offset] = q.s;
		state[1 + offset] = q.v[0];
		state[2 + offset] = q.v[1];
		state[3 + offset] = q.v[2];
	}

	/**
	 *		 gets the relative angular velocity for joint jIndex
	 */
	V3D getJointRelativeAngVelocity(int jIndex) const{
		int offset = getOffsetForJoint(jIndex);
		return V3D(state[4 + offset], state[5 + offset], state[6 + offset]);
	}

	/**
	 *		 sets the orientation for joint jIndex
	 */
	void setJointRelativeAngVelocity(V3D w, int jIndex){
		int offset = getOffsetForJoint(jIndex);
		state[4 + offset] = w[0];
		state[5 + offset] = w[1];
		state[6 + offset] = w[2];
	}

	void clearVelocities() {
		setVelocity(V3D(0, 0, 0));
		setAngularVelocity(V3D(0, 0, 0));
		for (int i = 0; i < getJointCount(); i++)
			setJointRelativeAngVelocity(V3D(0, 0, 0), i);
	}

	void writeToFile	(const char* fName, Robot* robot = NULL);
	bool isSameAs		(const ReducedRobotState& other);
	void readFromFile	(const char* fName);

	//setting the heading...
	void setHeading		(double heading);

	double getHeading(){
		//first we need to get the current heading of the robot. 
		return computeHeading(getOrientation(), headingAxis).getRotationAngle(headingAxis);
	}

};

void setupSimpleRobotStructure(Robot* robot);



