#include <ControlLib/GenericLimb.h>

GenericLimb::GenericLimb(void){
	name = "unnamedLeg";
	origin = NULL;
	parentBodyFrame = NULL;
}

GenericLimb::~GenericLimb(void){
}

/**
	this method is used to return the list of joints of the current leg...
*/
DynamicArray<Joint*>* GenericLimb::getJointList(){
	if (jointList.size() == 0)
		initializeJointList();

	return &jointList;
}

bool GenericLimb::isPartOfLimb(RigidBody* rb){
	if (rb == NULL)
		return false;

	DynamicArray<Joint*> *legJointListP = getJointList();
	for (uint i=0;i<legJointListP->size();i++)
		if (rb == legJointListP->at(i)->child)
			return true;
	return false;
}
