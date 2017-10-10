#include <RBSimLib/RBState.h>

/**
	Default constructor - populate the data members using safe values..
*/
RBState::RBState(void){
	//guess some default values if this constructor is used...
	this->position = P3D(0,0,0);
	this->orientation = Quaternion(1,V3D(0,0,0));
	this->velocity = V3D(0,0,0);
	this->angularVelocity = V3D(0,0,0);
}

/**
	A copy constructor.
*/
RBState::RBState(const RBState& other){
	this->position = other.position;
	this->orientation = other.orientation;
	this->velocity = other.velocity;
	this->angularVelocity = other.angularVelocity;
}

/**
	and a copy operator	
*/
RBState& RBState::operator = (const RBState& other){
	this->position = other.position;
	this->orientation = other.orientation;
	this->velocity = other.velocity;
	this->angularVelocity = other.angularVelocity;
	return *this;
}

RBState::~RBState(void){
		
}
