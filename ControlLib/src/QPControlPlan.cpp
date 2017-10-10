#include <ControlLib/QPControlPlan.h>
#include <ControlLib/BodyFrame.h>
#include <ControlLib/GenericLimb.h>
#include <Utils/Utils.h>
#include <MathLib/MathLib.h>
#include <GUILib/GLUtils.h>
#include <MathLib/ConvexHull3D.h>
	

QPControlPlan::QPControlPlan(Robot* robot){
	this->robot = robot;
	this->robotRepresentation = new GeneralizedCoordinatesRobotRepresentation(robot);
	resize(a, robotRepresentation->getDimensionCount());
	resize(u, robotRepresentation->getDimensionCount());

//	Logger::consolePrint("there are %d end effectors: \n", contactEndEffectors.size());
	resize(targetGeneralizedAccelerations, robotRepresentation->getDimensionCount());
	dofUsedInSwingLimbs.resize(robotRepresentation->getDimensionCount(), false);

	contactEndEffectors.reserve(100);
	generalEndEffectors.reserve(100);
	swingLimbEndEffectors.reserve(100);

	initializeControlStep();
}



QPControlPlan::~QPControlPlan(void){
	delete robotRepresentation;
}

//this method should be called before the qp solver is invoked...
void QPControlPlan::initializeControlStep() {
	robotRepresentation->syncGeneralizedCoordinatesWithRobotState();

	//precompute quantities that depend only on q and qDot... relevant jacobians, generalized mass matrix, generalized forces, etc...
	robotRepresentation->computeMassMatrix(M);
	robotRepresentation->computeCoriolisAndCentrifugalForcesTerm(C);

	//consolidate all gravitational forces into a vector...

	//NOTE: this is one way to compute the gravitational forces, another one (slower, but more intuitive) in the comment below...
	V3D gravity = Globals::worldUp * Globals::g;
	resize(qDotDotTmp, robotRepresentation->getDimensionCount());
	for (int i = 0; i < 3; i++) qDotDotTmp[i] = gravity[i];

	gravitationalForces = M * qDotDotTmp;

/*
	dVector gravitationalForces_test;
	resize(gravitationalForces_test, robotRepresentation->getDimensionCount());
	MatrixNxM J;
	robotRepresentation->compute_dpdq(P3D(), robot->root, J);
	gravitationalForces_test.noalias() += robot->root->rbProperties.mass * J.transpose() * gravity;
	for (uint i = 0; i < robot->jointList.size(); ++i) {
		robotRepresentation->compute_dpdq(P3D(), robot->jointList[i]->child, J);
		gravitationalForces_test.noalias() += robot->jointList[i]->child->rbProperties.mass * J.transpose() * gravity;
	}


	//NOTE: now test them out...
	if ((gravitationalForces_test - gravitationalForces).norm() > 0.000001)
		Logger::consolePrint("NOTE: there is a problem with gravitational force computation (%lf mismatch)\n", (gravitationalForces_test - gravitationalForces).norm());

	//this explains the method implemented above -> gravitational forces (in generalized coordinates) should induce an acceleration (in generalized coordinates) that only has a non-zero component for the body (e.g. no joint angle accelerations due just to gravity).
	dVector aTmp = M.ldlt().solve(gravitationalForces);
	print("..\\out\\M.m", M);
	print("..\\out\\gForce.m", gravitationalForces);
	print("..\\out\\a.m", aTmp);
	exit(0);
*/
}


void QPControlPlan::computeSwingLimbJointAccelerationTargets() {
	for (int i = 0; i < robotRepresentation->getDimensionCount();i++)
		dofUsedInSwingLimbs[i] = false;

	if (swingLimbEndEffectors.size() == 0)
		return;

	resize(tmpA, robotRepresentation->getDimensionCount(), robotRepresentation->getDimensionCount());
	resize(tmpRHS, robotRepresentation->getDimensionCount());

	robotRepresentation->getQDot(qDotTmp);

	for (uint i = 0; i < swingLimbEndEffectors.size(); i++) {
		//we have: eDotDot = J*qDotDot + JDot*qDot. This means, that given a target, a, we want to minimize the difference between eDotDot and it...
		tmpA.noalias() += swingLimbEndEffectors[i].J.transpose() * swingLimbEndEffectors[i].J;
		tmpRHS -= swingLimbEndEffectors[i].J.transpose() * (swingLimbEndEffectors[i].Jdot * qDotTmp - swingLimbEndEffectors[i].targetEEAcceleration);
	}

	//now add a regularizer to eliminate null space...
	for (int i = 0; i < robotRepresentation->getDimensionCount(); i++)
		tmpA.coeffRef(i, i) += 0.000001;

	//now, we need to filter away the DOFs that are not directly a part of the swing limbs - their accelerations should be set to zero (this includes constituents of the body frame, including the root, as well as "parallel" chains)

	for (uint i = 0; i < swingLimbEndEffectors.size(); i++) {
		RigidBody* rb = swingLimbEndEffectors[i].endEffectorRB;
		while (robot->bFrame->isPartOfBodyFrame(rb) == false) {
			dofUsedInSwingLimbs[robotRepresentation->getQIndexForJoint(rb->pJoints[0]->jIndex)] = true;
			rb = rb->pJoints[0]->parent;
		}
	}
	for (int i = 0; i < robotRepresentation->getDimensionCount(); i++) {
		if (dofUsedInSwingLimbs[i] == false) {
			tmpA.row(i).setZero();
			tmpA.col(i).setZero();
			tmpA.coeffRef(i, i) = 1;
			tmpRHS[i] = 0;
		}
	}

	//and finally, solve the system to retrieve the optimal joint acceleration values...
	tmpV = tmpA.ldlt().solve(tmpRHS);
/*
	//test the solution...
	for (uint i = 0; i < swingLimbEndEffectors.size(); i++) {
		//we have: eDotDot = J*qDotDot + JDot*qDot. This means, that given a target, a, we want to minimize the difference between eDotDot and it...
		dVector eeAcceleration = swingLimbEndEffectors[i].J * tmpV + swingLimbEndEffectors[i].Jdot * qDotTmp;
		dVector error = eeAcceleration - swingLimbEndEffectors[i].targetEEAcceleration;
		Logger::consolePrint("end effector %d has an error of %lf (%lf %lf %lf vs %lf %lf %lf)\n", i, error.norm(), eeAcceleration[0], eeAcceleration[1], eeAcceleration[2], swingLimbEndEffectors[i].targetEEAcceleration[0], swingLimbEndEffectors[i].targetEEAcceleration[1], swingLimbEndEffectors[i].targetEEAcceleration[2]);
	}
	print("../out/xDotDot.m", tmpV);
*/
	//now that we have the solution, we'll need to set the appropriate joint acceleration constraints...
	for (int i = 0; i < robotRepresentation->getDimensionCount(); i++)
		if (dofUsedInSwingLimbs[i] == true)
			targetGeneralizedAccelerations[i] = tmpV[i];
}


