#include <RBSimLib/RBProperties.h>
#include <RBSimLib/RigidBody.h>

Vector3d RBEndEffector::getWheelAxis(RigidBody *rb) {
	return rb->getWorldCoordinates(localCoordsWheelAxis).normalized();
}

Vector3d RBEndEffector::getWheelYawAxis(RigidBody *rb) {
	return Vector3d(0, 1, 0);
}

Vector3d RBEndEffector::getWheelTiltAxis(RigidBody *rb) {
	return getWheelAxis(rb).cross(getWheelYawAxis(rb)).normalized();
}

Vector3d RBEndEffector::getWheelRho(RigidBody *rb) {
	double wheelRadius = featureSize;
	return getWheelTiltAxis(rb).cross(getWheelAxis(rb)).normalized() * wheelRadius;
}

/**
	default constructor.
*/
RBProperties::RBProperties(){

}

/**
	default destructor.
*/
RBProperties::~RBProperties(){
	//nothing to do here...
}

/**
			set the moment of inertia of the rigid body - symmetric 3x3 matrix, so we need the six values for it.
*/
void RBProperties::setMOI(double moi00, double moi11, double moi22, double moi01, double moi02, double moi12){
	// Set MOI

	MOI_local <<
		moi00, moi01, moi02,
		moi01, moi11, moi12,
		moi02, moi12, moi22;

}
