

#include <fstream>

#include "RBSphereCollision.h"





RBSphereCollisionObjective::RBSphereCollisionObjective(RobotParameters * robotParameters, 
													   std::vector<RigidBody *> rbs,
													   double l, double stiffness, double epsilon)
	: robotParameters(robotParameters), rbs(rbs), constraint(l, stiffness, epsilon)
{
	double d_margin = 0.03;
	
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

	// further: ignore all pairs of rb that initially collide or are very close (at the time this class is constructed)
	
	prepare();
	for(int i = 0; i < rbs.size(); ++i) {
		for(int j = 0; j < rbs.size(); ++j) {
			if(!check_collision_RBs(i,j)){continue;}
			for(auto & sdp : sphereDistPairs(i, j)) {
				double d = std::get<2>(sdp);
				if(d < l + epsilon + d_margin) {
					check_collision_RBs(i, j) = false;
					check_collision_RBs(j, i) = false;
					break;
				}
			}
		}
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


double RBSphereCollisionObjective::distance(int rb1_idx, int rb2_idx, int & cdp1_idx, int & cdp2_idx)
{
	double d_min = 1.0e50;
	// loop through all pairs of (shperical) cdps between the rigid bodys
	for(size_t i = 0; i < cr[rb1_idx].size(); ++i) {
		P3D c1 = cr[rb1_idx][i].first;
		double r1 = cr[rb1_idx][i].second;
		for(size_t j = 0; j < cr[rb2_idx].size(); ++j) {
			P3D c2 = cr[rb2_idx][j].first;
			double r2 = cr[rb2_idx][j].second;
			double d = (c2-c1).length() - (r1 + r2);
			if(d < d_min) {
				d_min = d;
				cdp1_idx = i;
				cdp2_idx = j;
			}
		}
	}
	return(d_min);
}

void RBSphereCollisionObjective::dDistanceDpar(int rb1_idx, int rb2_idx, int cdp1_idx, int cdp2_idx, dVector & grad) // gradient with respect to the parameters of the RobotParameters set
{
	// compute derivative of c with respect to the parameters q
	MatrixNxM dc1dq;	// dq here refers to the parameter set of the GeneralizedCoordinatesRobotRepresentation
	MatrixNxM dc2dq;

	robotParameters->robotParameters->compute_dpdq(c_local[rb1_idx][cdp1_idx], rbs[rb1_idx], dc1dq);
	robotParameters->robotParameters->compute_dpdq(c_local[rb2_idx][cdp2_idx], rbs[rb2_idx], dc2dq);

	P3D c1 = cr[rb1_idx][cdp1_idx].first;
	P3D c2 = cr[rb2_idx][cdp2_idx].first;
	V3D deltac = (c2 - c1);
	MatrixNxM ddeltacdq = dc2dq - dc1dq;
	
	int n = robotParameters->getNPar();
	grad.resize(n);

	for(int i = 0; i < n; ++i) {
		grad[i] = deltac.unit().dot(static_cast<V3D>(ddeltacdq.col(i+6)));
	}
}

double RBSphereCollisionObjective::computeValue(const dVector& p)
{
	prepare();
	
	// set (robot joint angle) parameters from global parameter list
	int i_io = robotParameters->parametersStartIndex;
	robotParameters->setFromList(p, i_io);

	double result = 0.0;
	// loop through all pairs of rigid bodies
	for(int i = 0; i < rbs.size(); ++i) {
		for(int j = i+1; j < rbs.size(); ++j) {
			// check if the pair needs to be considered
			if(!check_collision_RBs(i, j)) {continue;}
			for(auto & sdp : sphereDistPairs(i,j)) {
				double d = std::get<2>(sdp);
				result += constraint.computeValue(d);
			}
		}
	}
	return(result);
}

void RBSphereCollisionObjective::addGradientTo(dVector& grad, const dVector& p) 
{
	prepare();

	// set (robot joint angle) parameters from global parameter list
	int i_io = robotParameters->parametersStartIndex;
	robotParameters->setFromList(p, i_io);

	dVector ddistancedrobotpar;
	
	// loop through all pairs of rigid bodies
	for(int i = 0; i < rbs.size(); ++i) {
		for(int j = i+1; j < rbs.size(); ++j) {
			// check if the pair needs to be considered
			if(!check_collision_RBs(i, j)) {continue;}
			
			for(auto & sdp : sphereDistPairs(i,j)) {
				int cdp1_idx = std::get<0>(sdp);
				int cdp2_idx = std::get<1>(sdp);
				double d =     std::get<2>(sdp);
				double dOdd = constraint.computeDerivative(d);
				dDistanceDpar(i, j, cdp1_idx, cdp2_idx, ddistancedrobotpar);
				for(int k = 0; k < ddistancedrobotpar.size(); ++k) {
					grad[robotParameters->parametersStartIndex + k] += ddistancedrobotpar[k] * dOdd;
				}
				
			}
		}
	}
}



void RBSphereCollisionObjective::prepare() 
{
	precompute_coordinates();
	precompute_relevant_distances();
}


void RBSphereCollisionObjective::precompute_coordinates()
{
	cr.resize(rbs.size());
	c_local.resize(rbs.size());

	for(size_t i = 0; i < rbs.size(); ++i) {
		cr[i].resize(0);
		c_local[i].resize(0);
		for(size_t j = 0; j < rbs[i]->cdps.size(); ++j) {
			SphereCDP * cdp = dynamic_cast<SphereCDP *>(rbs[i]->cdps[j]);
			if(!cdp) {continue;}
			P3D c = robotParameters->robotParameters->getWorldCoordinatesFor(cdp->p, rbs[i]);
			cr[i].push_back(std::pair<P3D, double>(c, cdp->r));
			c_local[i].push_back(cdp->p);
		}
	}
}


void RBSphereCollisionObjective::precompute_relevant_distances()
{
	// resize
	sphereDistPairs.resize(cr.size(), cr.size());
	for(size_t i = 0; i < cr.size(); ++i) {
		for(size_t j = i+1; j < cr.size(); ++j) {
			sphereDistPairs(i,j).resize(0);
		}
	}

	// find relevant distances
#pragma omp parallel for
	for(int i = 0; i < (int)cr.size(); ++i) {
		for(size_t j = i+1; j < cr.size(); ++j) {
			if(!check_collision_RBs(i,j)) {continue;}
			int i_cdp, j_cdp;
			double d = distance(i, j, i_cdp, j_cdp);
			if( ! IS_ZERO(constraint.computeValue(d)) ) {
				sphereDistPairs(i,j).push_back(std::make_tuple(i_cdp, j_cdp, d));
				sphereDistPairs(j,i).push_back(std::make_tuple(j_cdp, i_cdp, d));
			}
		}
	}
	
}


