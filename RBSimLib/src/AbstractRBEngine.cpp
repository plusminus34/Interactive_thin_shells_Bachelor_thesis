
#include <Utils/Utils.h>
#include <GUILib/GLUtils.h>
#include <GUILib/GLContentManager.h>

#include <RBSimLib/AbstractRBEngine.h>
#include <RBSimLib/RBUtils.h>
#include <RBSimLib/BallAndSocketJoint.h>
#include <RBSimLib/HingeJoint.h>
#include <RBSimLib/UniversalJoint.h>
#include <RBSimLib/FixedJoint.h>

AbstractRBEngine::AbstractRBEngine(void){
}

AbstractRBEngine::~AbstractRBEngine(void){
	destroy();
}

void AbstractRBEngine::destroy() {
	//delete all the rigid bodies in this world
	for (uint i = 0; i < rbs.size();i++)
		delete rbs[i];
	rbs.clear();
	for (uint i = 0; i < joints.size();i++)
		delete joints[i];
	joints.clear();

	contactForces.clear();
}

void AbstractRBEngine::markRBContacts(double fMagTreshold) {
	for (uint i = 0; i < rbs.size(); i++)
		rbs[i]->inContact = false;

	for (uint i = 0; i < contactForces.size(); i++) {
		if (contactForces[i].f.length() >= fMagTreshold) {
			if (contactForces[i].rb1) contactForces[i].rb1->inContact = true;
			if (contactForces[i].rb2) contactForces[i].rb2->inContact = true;
		}
	}
}

void AbstractRBEngine::drawContactForces(){
	glColor3d(0.0, 0.8, 0.0);
	V3D netSum;
	P3D meanCp;
	double netSumOfMag = 0;
	for (uint i = 0; i < contactForces.size(); i++){
		drawArrow(contactForces[i].cp, contactForces[i].cp +contactForces[i].f*0.01, 0.004);
		netSum = netSum + contactForces[i].f;
		meanCp = meanCp + contactForces[i].cp*contactForces[i].f.length();
		netSumOfMag += contactForces[i].f.length();
	}
//	glColor3d(0.0, 0.0, 0.8);
//	if (netSumOfMag > 0)
//		drawArrow(meanCp / netSumOfMag, meanCp / netSumOfMag + netSum*0.0001, 0.04);
}

/**
	This method is used to draw all the rigid bodies in the world
*/
void AbstractRBEngine::drawRBs(int flags){
	for (uint i=0;i<rbs.size();i++)
		rbs[i]->draw(flags);
}


/**
	This method returns the reference to the rigid body with the given name, or NULL if it is not found
*/
RigidBody* AbstractRBEngine::getRBByName(char* name){
	if (name == NULL)
		return NULL;
//	for (uint i=0;i<this->rbs.size();i++)
	for (int i=(int)rbs.size()-1;i>=0;i--)
		if (strcmp(name, rbs[i]->name.c_str()) == 0)
			return rbs[i];
	return NULL;
}

/**
	This method returns the reference to the joint whose name matches, or NULL if it is not found
*/
Joint* AbstractRBEngine::getJointByName(char* name) {
	if (name == NULL)
		return NULL;
	for (int i = (int)joints.size() - 1;i >= 0;i--)
		if (strcmp(name, joints[i]->name.c_str()) == 0)
		return joints[i];
	return NULL;
}



/**
	This method reads a list of rigid bodies from the specified file.
*/
void AbstractRBEngine::loadRBsFromFile(const char* fName){
	if (fName == NULL)
		throwError("NULL file name provided.");
	FILE *f = fopen(fName, "r");
	if (f == NULL)
		throwError("Could not open file: %s", fName);

	int nRBsAlreadyLoaded = rbs.size();

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	RigidBody* newBody = NULL;
	//ArticulatedFigure* newFigure = NULL;
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, 200);
		if (strlen(buffer)>195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		int lineType = getRBLineType(line);
		switch (lineType) {
			case RB_RB:
				//create a new rigid body and have it load its own info...
				newBody = new RigidBody();
				newBody->loadFromFile(f);
				rbs.push_back(newBody);
				break;
			case RB_BALL_AND_SOCKET_JOINT:
				joints.push_back(new BallAndSocketJoint());
				joints[joints.size()-1]->loadFromFile(f, this);
				break;
			case RB_HINGE_JOINT:
				joints.push_back(new HingeJoint());
				joints[joints.size() - 1]->loadFromFile(f, this);
				break;
			case RB_UNIVERSAL_JOINT:
				joints.push_back(new UniversalJoint());
				joints[joints.size() - 1]->loadFromFile(f, this);
				break;
			case RB_WELDED_JOINT:
				joints.push_back(new FixedJoint());
				joints[joints.size() - 1]->loadFromFile(f, this);
				break;

//			case RB_ARTICULATED_FIGURE:
//				//we have an articulated figure to worry about...
//               newFigure = new ArticulatedFigure();
//				AFs.push_back(newFigure);
//				newFigure->loadFromFile(f, this);
//				newFigure->addJointsToList(&jts);
//				break;
			case RB_MATERIAL_DEFINITION: {
					GLShaderMaterial* material = new GLShaderMaterial();
					material->readFromFile(f);
					GLContentManager::addShaderMaterial(material->getMaterialName().c_str(), material);
				}
				break;
			case RB_NOT_IMPORTANT:
				if (strlen(line)!=0 && line[0] != '#')
					Logger::consolePrint("Ignoring input line: \'%s\'\n", line);
				break;
			default:
				throwError("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
		}
	}

	fclose(f);

	if (autoGenerateCDPs)
		for (uint i = nRBsAlreadyLoaded; i < rbs.size(); i++)
			rbs[i]->autoGenerateCDPs();
	

}

/**
	This method is used to get the state of all the rigid body in this collection.
*/
void AbstractRBEngine::getState(DynamicArray<double>* state){
	for (uint i=0;i<this->rbs.size();i++){
		state->push_back(rbs[i]->state.position[0]);
		state->push_back(rbs[i]->state.position[1]);
		state->push_back(rbs[i]->state.position[0]);

		state->push_back(rbs[i]->state.orientation.s);
		state->push_back(rbs[i]->state.orientation.v[0]);
		state->push_back(rbs[i]->state.orientation.v[1]);
		state->push_back(rbs[i]->state.orientation.v[0]);

		state->push_back(rbs[i]->state.velocity[0]);
		state->push_back(rbs[i]->state.velocity[1]);
		state->push_back(rbs[i]->state.velocity[0]);

		state->push_back(rbs[i]->state.angularVelocity[0]);
		state->push_back(rbs[i]->state.angularVelocity[1]);
		state->push_back(rbs[i]->state.angularVelocity[0]);
	}
}

/**
	This method is used to set the state of all the rigid body in this collection.
*/
void AbstractRBEngine::setState(DynamicArray<double>* state, int start){
	int i = start;
	for (uint j=0;j<this->rbs.size();j++){
		rbs[j]->state.position = P3D((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
		rbs[j]->state.orientation = Quaternion((*state)[i+0], (*state)[i+1], (*state)[i+2], (*state)[i+3]);
		i+=4;
		rbs[j]->state.velocity = V3D((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
		rbs[j]->state.angularVelocity = V3D((*state)[i+0], (*state)[i+1], (*state)[i+2]);
		i+=3;
	}
}

