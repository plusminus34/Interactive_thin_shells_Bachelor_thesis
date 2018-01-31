

//#include "RBSimLib/AbstractRBEngine.h"
#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>

#include "RobotMount.h"	// defines RobotParameters


using MatrixNxM_bool = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;



class RBSphereCollisionObjective : public ObjectiveFunction {

public:
	RobotParameters * robotParameters;
	std::vector<RigidBody *> rbs;
	SoftUnilateralConstraint constraint;

	MatrixNxM_bool check_collision_RBs;

	RBSphereCollisionObjective(RobotParameters * robotParameters, 
							   std::vector<RigidBody *> rbs,
							   double l, double stiffness, double epsilon);

	void setSphereCDPsFromFile(std::string const & dirName);



	double distance(RigidBody * rb1, RigidBody * rb2);
	double distance(RigidBody * rb1, RigidBody * rb2, SphereCDP *& cdp1_min, SphereCDP *& cdp2_min);

	void dDistanceDpar(RigidBody * rb1, RigidBody * rb2, dVector & grad);

	virtual double computeValue(const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
	//virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);
};