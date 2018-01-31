

#include <fstream>

#include "RBSphereCollision.h"





RBSphereCollisionObjective::RBSphereCollisionObjective(RobotParameters * robotParameters, 
													   std::vector<RigidBody *> rbs,
													   double l, double stiffness, double epsilon)
	: robotParameters(robotParameters), rbs(rbs), constraint(l, stiffness, epsilon)
{
	// read spheres
	setSphereCDPsFromFile("../data/rbs/yumi/spheres/");

	// construct lookup matrix for which RB pairs to test for distance (exclude direct neighbours);
	check_collision_RBs = MatrixNxM_bool::Constant(rbs.size(), rbs.size(), true);
	for(int idx_parent = 0; idx_parent < rbs.size(); ++idx_parent) {
		RigidBody * rbParent = rbs[idx_parent];
		for(Joint * joint : rbParent->cJoints) {
			if(joint->child) {
				RigidBody * rbChild = joint->child;
				// search for the index of the rb that is the child of the outer-loop rb
				for(int idx_child = 0; idx_child < rbs.size(); ++idx_child) {
					if(rbs[idx_child] == rbChild) {
						check_collision_RBs(idx_parent, idx_child) = false;
						check_collision_RBs(idx_child, idx_parent) = false;
						break;
					}
				}
			}
		}
		// also: no self collision
		check_collision_RBs(idx_parent, idx_parent) = false;
	}
}



void RBSphereCollisionObjective::setSphereCDPsFromFile(std::string const & dirName)
{

	// compute center of bounding box of mesh (attention: very hacky)
	std::vector<P3D> center_bb;
	for(RigidBody * rb : rbs) {
		center_bb.push_back( rb->meshes[0]->calBoundingBox().center());
		//center_bb.push_back( rb->meshes[0]->getCenterOfMass());
	}

	int k = -1;
	for(RigidBody * rb : rbs) {
		++k;
		// clear all existing CDPs
		for(CollisionDetectionPrimitive* cdp : rb->cdps) {
			delete cdp;
		}
		rb->cdps.resize(0);
		// construct file name
		std::string fileName = dirName + "/" + rb->name + ".sph";
		std::ifstream infile(fileName);
		if(!infile.is_open()) {continue;}
		// read spheres
		int nLevels;
		int BranchFactor;
		infile >> nLevels >> BranchFactor;
		for(int i = 0; i < nLevels; ++i) {
			int nSpheresLevel = std::pow(BranchFactor, i);
			for(int j = 0; j < nSpheresLevel; ++j) {
				P3D c;
				double r, w;
				infile >> c(0) >> c(1) >> c(2) >> r >> w;

				if(i == nLevels-1) {
					c += center_bb[k];
					c = rb->meshTransformations[0].transform(c);
					rb->cdps.push_back(new SphereCDP(c, r));
				}
			}
		}
	}
}




double RBSphereCollisionObjective::distance(RigidBody * rb1, RigidBody * rb2)
{
	SphereCDP * cdp1_min = nullptr;
	SphereCDP * cdp2_min = nullptr;
	return(distance(rb1, rb2, cdp1_min, cdp2_min));
}



double RBSphereCollisionObjective::distance(RigidBody * rb1, RigidBody * rb2, SphereCDP *& cdp1_min, SphereCDP *& cdp2_min)
{
	//double d_min = std::numeric_limits<double>::max();
	double d_min = 1.0e50;
	cdp1_min = nullptr;
	cdp2_min = nullptr;
	// loop through all pairs of (shperical) cdps between the rigid bodys
	for(int i = 0; i < rb1->cdps.size(); ++i) {
		SphereCDP * cdp1 = dynamic_cast<SphereCDP *>(rb1->cdps[i]);
		if(!cdp1) {continue;}
		P3D c1 = robotParameters->robotParameters->getWorldCoordinatesFor(cdp1->p, rb1);
		for(int j = 0; j < rb2->cdps.size(); ++j) {
			SphereCDP * cdp2 = dynamic_cast<SphereCDP *>(rb2->cdps[i]);
			if(!cdp2) {continue;}
			P3D c2 = robotParameters->robotParameters->getWorldCoordinatesFor(cdp2->p, rb2);
			double d = (c2 - c1).length() - (cdp1->r + cdp2->r);
			if(d < d_min) {
				d_min = d;
				cdp1_min = cdp1;
				cdp2_min = cdp2;
			}
		}
	}
	return(d_min);
}


void RBSphereCollisionObjective::dDistanceDpar(RigidBody * rb1, RigidBody * rb2, dVector & grad) // gradient with respect to the parameters of the RobotParameters set
{
	// find cdp pair with the minimum distance
	SphereCDP * cdp1 = nullptr;
	SphereCDP * cdp2 = nullptr;
	distance(rb1, rb2, cdp1, cdp2);

	MatrixNxM dc1dq;	// dq here refers to the parameter set of the GeneralizedCoordinatesRobotRepresentation
	MatrixNxM dc2dq;

	robotParameters->robotParameters->compute_dpdq(cdp1->p, rb1, dc1dq);
	robotParameters->robotParameters->compute_dpdq(cdp2->p, rb2, dc2dq);

	V3D deltac = (cdp2->p - cdp1->p);
	MatrixNxM ddeltacdq = dc2dq - dc1dq;
	

	int n = robotParameters->getNPar();
	grad.resize(n);

	for(int i = 0; i < n; ++i) {
		grad[i] = deltac.unit().dot(static_cast<V3D>(ddeltacdq.col(i+6)));
	}
}



double RBSphereCollisionObjective::computeValue(const dVector& p)
{
	// set (robot joint angle) parameters from global parameter list
	int i_io = robotParameters->parametersStartIndex;
	robotParameters->setFromList(p, i_io);

	double result = 0.0;
	// loop through all pairs of rigid bodies
	for(int i = 0; i < rbs.size(); ++i) {
		for(int j = i+1; j < rbs.size(); ++j) {
			// check if the pair needs to be considered
			if(!check_collision_RBs(i, j)) {continue;}
			double d = distance(rbs[i], rbs[j]);
			result += constraint.computeValue(d);
		}
	}
	return(result);
}


void RBSphereCollisionObjective::addGradientTo(dVector& grad, const dVector& p) 
{
	// set (robot joint angle) parameters from global parameter list
	int i_io = robotParameters->parametersStartIndex;
	robotParameters->setFromList(p, i_io);

	// loop through all pairs of rigid bodies
	for(int i = 0; i < rbs.size(); ++i) {
		for(int j = i+1; j < rbs.size(); ++j) {
			// check if the pair needs to be considered
			if(!check_collision_RBs(i, j)) {continue;}
			dVector ddrobotpar;
			dDistanceDpar(rbs[i], rbs[j], ddrobotpar);
			for(int k = 0; k < ddrobotpar.size(); ++k) {
				grad[robotParameters->parametersStartIndex + k] += ddrobotpar[k];
			}
		}
	}

}




