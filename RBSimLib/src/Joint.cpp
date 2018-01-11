#include <RBSimLib/Joint.h>
#include <RBSimLib/AbstractRBEngine.h>

Joint::Joint(void){
}

Joint::~Joint(void){
}

/**
	Returns the rbEngine position of the joint
*/
P3D Joint::getWorldPosition(){
	return (child->getWorldCoordinates(cJPos) + parent->getWorldCoordinates(pJPos))/2.0;
}

/**
	This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in 
	the frame coordinate of the parent.
*/
Quaternion Joint::computeRelativeOrientation(){
	//if qp is the quaternion that gives the orientation of the parent, and qc gives the orientation of the child, then  qp^-1 * qc gives the relative
	//orientation between the child and the parent (child to parent)
	return (parent->state.orientation.getComplexConjugate() * child->state.orientation).toUnit();
}

/**
	This method is used to fix the errors in the joints (i.e. project state of child such that joint configuration is consistent). The state
	of the parent does not change.
*/
void Joint::fixJointConstraints(bool fixPositions, bool fixOrientations, bool fixLinVelocities, bool fixAngularVelocities){
	assert(child && parent);
    
	//first fix the relative orientation
	if (fixOrientations)
		fixOrientationConstraint();

	//now worry about the joint positions
	if (fixPositions) {
		//compute the vector rc from the child's joint position to the child's center of mass (in rbEngine coordinates)
		V3D rc = child->getWorldCoordinates(V3D(cJPos, P3D(0, 0, 0)));
		//and the vector rp that represents the same quanity but for the parent
		V3D rp = parent->getWorldCoordinates(V3D(pJPos, P3D(0, 0, 0)));

		//the location of the child's CM is now: pCM - rp + rc
        V3D axis;
        double angle;
        child->state.orientation.getAxisAngle(axis, angle);
        /*printf("c axis %.3lf %.3lf %.3lf\n", axis.x(), axis.y(), axis.z());
        printf("c angle %.3lf\n", angle);
        parent->state.orientation.getAxisAngle(axis, angle);
        printf("p axis %.3lf %.3lf %.3lf\n", axis.x(), axis.y(), axis.z());
        printf("p angle %.3lf\n", angle);
        printf("rc %.3lf %.3lf %.3lf\n", rc.x(), rc.y(), rc.z());
        printf("rp %.3lf %.3lf %.3lf\n", rp.x(), rp.y(), rp.z());
        printf("position %.3lf %.3lf %.3lf\n", (parent->state.position + (rc - rp)).x(), (parent->state.position + (rc - rp)).y(), (parent->state.position + (rc - rp)).z());*/
		child->state.position = parent->state.position + (rc - rp);
	}

	if (fixAngularVelocities)
		fixAngularVelocityConstraint();

	//fix the velocities, if need be
	if (fixLinVelocities){
		//we want to get the relative velocity at the joint to be 0. This can be accomplished in many different ways, but this approach
		//only changes the linear velocity of the child
		V3D pJPosVel = parent->getAbsoluteVelocityForLocalPoint(pJPos);
		V3D cJPosVel = child->getAbsoluteVelocityForLocalPoint(cJPos);
		child->state.velocity -= cJPosVel - pJPosVel;
		assert(IS_ZERO((parent->getAbsoluteVelocityForLocalPoint(pJPos) - child->getAbsoluteVelocityForLocalPoint(cJPos)).length()));
	}
}

/**
	writes the joint info to file...
*/
void Joint::writeCommonAttributesToFile(FILE* fp){
	char* str;

	str = getRBString(RB_NAME);
	fprintf(fp, "\t\t%s %s\n", str, this->name.c_str());

	str = getRBString(RB_PARENT);
	fprintf(fp, "\t\t%s %s\n", str, this->parent->name.c_str());

	str = getRBString(RB_CHILD);
	fprintf(fp, "\t\t%s %s\n", str, this->child->name.c_str());

	str = getRBString(RB_CPOS);
	fprintf(fp, "\t\t%s %lf %lf %lf\n", str, cJPos[0], cJPos[1], cJPos[2]);

	str = getRBString(RB_PPOS);
	fprintf(fp, "\t\t%s %lf %lf %lf\n", str, pJPos[0], pJPos[1], pJPos[2]);

	str = getRBString(RB_JOINT_END);
	fprintf(fp, "\t%s\n\n\n", str);
}


/**
	This method is used to load the details of a joint from file. The PhysicalWorld parameter points to the rbEngine in which the objects
	that need to be linked live in.
*/
void Joint::loadFromFile(FILE* f, AbstractRBEngine* rbEngine){
	if (f == NULL)
		throwError("Invalid file pointer.");
	if (rbEngine == NULL)
		throwError("A valid physical rbEngine must be passed in as a parameter");
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char tempName[100];

	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);

		if (processInputLine(line))
			continue;

		int lineType = getRBLineType(line);
		switch (lineType) {
			case RB_NAME:
				this->name = std::string() + trim(line);
				break;
			case RB_PARENT:
				sscanf(line, "%s", tempName);
				if (parent != NULL)
					throwError("This joint already has a parent");
				parent = rbEngine->getRBByName(tempName);
				if (parent == NULL)
					throwError("The articulated rigid body \'%s\' cannot be found!", tempName);
				break;
			case RB_CHILD:
				sscanf(line, "%s", tempName);
				if (child != NULL)
					throwError("This joint already has a parent");
				child = rbEngine->getRBByName(tempName);
				if (child == NULL)
					throwError("The articulated rigid body \'%s\' cannot be found!", tempName);
				break;
			case RB_CPOS:
				sscanf(line, "%lf %lf %lf", &cJPos[0], &cJPos[1], &cJPos[2]);
				break;
			case RB_PPOS:
				sscanf(line, "%lf %lf %lf",&pJPos[0], &pJPos[1], &pJPos[2]);
				break;
			case RB_JOINT_CONTROL_MODE:
				sscanf(line, "%d", &controlMode);
				break;
			case RB_JOINT_END:
				//we now have to link together the child and parent bodies
				if (child == NULL)
					throwError("A joint has been found that does not have a child rigid body");
				if (parent == NULL)
					throwError("A parent has been found that does not have a child rigid body");
				child->pJoints.push_back(this);
				parent->cJoints.push_back(this);
                fixJointConstraints(true, false, false, false);

				return;//and... done
				break;
			case RB_NOT_IMPORTANT:
				if (strlen(line)!=0 && line[0] != '#')
					Logger::consolePrint("Ignoring input line: \'%s\'\n", line);
				break;
			default:
				throwError("Incorrect articulated body input file: \'%s\' - unexpected line.", buffer);
		}
	}
	throwError("Incorrect articulated body input file! No /ArticulatedFigure found");
}

