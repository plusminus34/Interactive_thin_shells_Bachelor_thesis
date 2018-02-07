

//#include "RBSimLib/AbstractRBEngine.h"
#include <OptimizationLib/ObjectiveFunction.h>
#include <OptimizationLib/SoftUnilateralConstraint.h>

#include "RobotMount.h"	// defines RobotParameters


using MatrixNxM_bool = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;



class RBSphereCollisionObjective : public ObjectiveFunction {

private:
	// precomputed helpers
	std::vector<std::vector<std::pair<P3D,double> > > cr;	// centers of all sqheres, in world coordinates, plus the radius
	std::vector<std::vector<P3D> > c_local; // centers of all sqheres, in local coordinates
	//std::vector<std::vector<SphereCDP*> > sphereCDPs;	// lookup table for 
	MatrixNxM	d_min;	// minimum distance in-betewen each pair of RBs
	std::vector<std::vector<std::pair<int,int> > > cdpIdx_d_min;	// indices of sphere-pare that are responsible for the minimum distance between two RBs



public:
	RobotParameters * robotParameters;
	std::vector<RigidBody *> rbs;
	SoftUnilateralConstraint constraint;

	MatrixNxM_bool check_collision_RBs;

	RBSphereCollisionObjective(RobotParameters * robotParameters, 
							   std::vector<RigidBody *> rbs,
							   double l, double stiffness, double epsilon);

	void setSphereCDPsFromFile(std::string const & dirName);



	//double distance(RigidBody * rb1, RigidBody * rb2);
	//double distance(RigidBody * rb1, RigidBody * rb2, SphereCDP *& cdp1_min, SphereCDP *& cdp2_min);
	double distance(int rb1_idx, int rb2_idx,  int & cdp1_idx, int & cdp2_idx);

	//void dDistanceDpar(RigidBody * rb1, RigidBody * rb2, dVector & grad);
	void dDistanceDpar(int rb1_idx, int rb2_idx, dVector & grad);

	virtual double computeValue(const dVector& p);
	virtual void addGradientTo(dVector& grad, const dVector& p);
	//virtual void addHessianEntriesTo(DynamicArray<MTriplet>& hessianEntries, const dVector& p);



	void precompute_coordinates();
	void precompute_min_distance();

	void prepare();
};