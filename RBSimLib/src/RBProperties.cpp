#include <RBSimLib/RBProperties.h>
#include <RBSimLib/RigidBody.h>

Vector3d RBEndEffector::getWheelAxis() const {
	return localCoordsWheelAxis.normalized();
}

Vector3d RBEndEffector::getWheelYawAxis() const {
	return Vector3d(0, 1, 0);
}

Vector3d RBEndEffector::getWheelTiltAxis() const {
	return getWheelAxis().cross(getWheelYawAxis()).normalized();
}

Vector3d RBEndEffector::getWheelRho() const {
	double wheelRadius = featureSize;
	return getWheelTiltAxis().cross(getWheelAxis()).normalized() * wheelRadius;
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
