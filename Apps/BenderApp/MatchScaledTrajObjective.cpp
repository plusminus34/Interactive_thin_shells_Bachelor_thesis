#include <iostream>


#include <GUILib/GLUtils.h>

#include "MatchScaledTrajObjective.h"



MatchScaledTrajObjective::MatchScaledTrajObjective(DynamicArray<Node * > matchedFiber, Trajectory3Dplus targetTrajectory)
	: matchedFiber(matchedFiber)
{
	setTargetTrajectory(targetTrajectory);
}


void MatchScaledTrajObjective::addO(const dVector & x, const dVector & X, double & o)
{
	update_tNode(x);

	int n = matchedFiber.size();
	for(int i = 0; i < n; ++i) {
		o += computeOofNode(i, x);
	}
}



void MatchScaledTrajObjective::addDoDx(const dVector & x, const dVector & X, dVector & dodx)
{
	update_tNode(x);
	prepareDtTargetDxNode(x);

	int n = matchedFiber.size();
	for(int i = 0; i < n; ++i) {
		addDoDxEachNode(i, x, dodx);
	}
}

void MatchScaledTrajObjective::addError(const dVector & x, double & e)
{
}


void MatchScaledTrajObjective::setTargetTrajectory(Trajectory3Dplus & traj, int nKnotsApproxT) 
{
	traj.createDiscreteSpline(nKnotsApproxT, targetTrajectory);
}





double MatchScaledTrajObjective::computeOofNode(int nodeID_local, const dVector & x)
{
	P3D pt_tgt = targetTrajectory.evaluate_catmull_rom(tNodeTarget[nodeID_local], false);
	P3D pt_node = matchedFiber[nodeID_local]->getCoordinates(x);
	double o = 0.5 * (pt_tgt - pt_node).length2();
	return(o);
}


void MatchScaledTrajObjective::addDoDxEachNode(int nodeID_local, const dVector & x, dVector & dodx)
{
	int n = matchedFiber.size();
	Node * node = matchedFiber[nodeID_local];

	// dO/dd where d is distance node->tgt
	P3D pt_tgt = targetTrajectory.evaluate_catmull_rom(tNodeTarget[nodeID_local], false);
	P3D pt_node = node->getCoordinates(x);
	V3D dOdd = pt_node - pt_tgt;

	// movement of the current node relative to the target-point
	for(int i = 0; i < node->dimSize; ++i) {
		dodx[node->dataStartIndex + i] += dOdd[i];
	}
	
	// movement of the target point due to movement of any of the nodes in the fiber
	V3D dPtTgtDt = targetTrajectory.evaluate_gradient_catmull_rom(tNodeTarget[nodeID_local], false);
	for(int j = 0; j < n; ++j) {
		
		V3D dOdxj = dOdd * dPtTgtDt.dot(dtTargetDxNode[j][nodeID_local]);
		
		for(int i = 0; i < node->dimSize; ++i) {
			dodx[node->dataStartIndex + i] += dOdxj[i];
		}
	}
	

}



void MatchScaledTrajObjective::update_tNode(dVector const & x) 
{
	int n = matchedFiber.size();
	// length of fiber
	tNodeFiber.resize(n);
	tNodeFiber[0] = 0.0;
	for(int i = 1; i < n; ++i) {
		tNodeFiber[i] = tNodeFiber[i-1] + (matchedFiber[i]->getCoordinates(x) - matchedFiber[i-1]->getCoordinates(x)).length();
	}

	// normalize with length of target trajectory
	double t_tgt = targetTrajectory.getMaxPosition();
	double t_fiber = tNodeFiber.back();
	tNodeTarget.resize(n);
	for(int i = 0; i < n; ++i) {
		tNodeTarget[i] = tNodeFiber[i] * t_tgt / t_fiber;
	}
}



void MatchScaledTrajObjective::prepareDtTargetDxNode(dVector const & x) 
{

	int n = tNodeFiber.size();
	dtTargetDxNode.resize(n);
	
	double tFiberTot = tNodeFiber.back();
	double tFiberTot_inv = 1.0 / tFiberTot;
	double tTargetTot = tNodeTarget.back();

	for(int j = 0; j < n; ++j) {
		dtTargetDxNode[j].resize(n);

		V3D dtFiber_jm1_dx = (j == 0)   ? V3D(0.0,0.0,0.0) : (matchedFiber[j]->getCoordinates(x) - matchedFiber[j-1]->getCoordinates(x)) / (tNodeFiber[j] - tNodeFiber[j-1]);
		V3D dtFiber_jp1_dx = (j == n-1) ? V3D(0.0,0.0,0.0) : (matchedFiber[j]->getCoordinates(x) - matchedFiber[j+1]->getCoordinates(x)) / (tNodeFiber[j+1] - tNodeFiber[j]);

		V3D dtFiberTotal = dtFiber_jm1_dx + dtFiber_jp1_dx;

		for(int i = 0; i < n; ++i) {
			V3D dtFiberNodei;
			if(j < i) {
				dtFiberNodei = dtFiber_jm1_dx + dtFiber_jp1_dx;
			}
			else if(j > i) {
				dtFiberNodei = V3D(0.0,0.0,0.0);
			}
			else // j == i
			{
				dtFiberNodei = dtFiber_jm1_dx;
			}
			dtTargetDxNode[j][i] =  (  dtFiberNodei * tFiberTot_inv
									 - dtFiberTotal * tFiberTot_inv*tFiberTot_inv * tNodeFiber[i]
									) * tTargetTot;
		}
	}

}


void MatchScaledTrajObjective::draw(dVector const & x) {
	
	update_tNode(x);

	int n = tNodeTarget.size();

	// draw links
	for(int i = 0; i < n; ++i) {
		glColor3d(0, 1, 0);
		P3D pi = (matchedFiber[i]->getCoordinates(x));
		P3D pj = targetTrajectory.evaluate_catmull_rom(tNodeTarget[i], false);
		glBegin(GL_LINES);
		glVertex3d(pi[0], pi[1], pi[2]);
		glVertex3d(pj[0], pj[1], pj[2]);
		glEnd();
	}

	// draw target trajectory
	targetTrajectory.draw(V3D(0.3, 0.3, 0.3), 2, V3D(0, 0.8, 0), -0.003);

	// draw matched fiber
	glColor3d(0.3, 0.15, 0.15);
	glLineWidth((GLfloat)2.0);
	glBegin(GL_LINE_STRIP);
	for(Node * node : matchedFiber) {
		P3D pt = node->getCoordinates(x);
		glVertex3d(pt[0], pt[1], pt[2]);
	}
	glEnd();
	glLineWidth(1);
}

