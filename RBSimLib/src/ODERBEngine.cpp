#include <RBSimLib/ODERBEngine.h>
#include <Utils/Utils.h>

//TODO: once joints are set to position mode, the motors are persistent. We should perhaps have a way to turn them off, if we want, for instance, to transition between passive and active, or between position and control mode...

/**
Default constructor
*/
ODERBEngine::ODERBEngine() : AbstractRBEngine() {
	//Initialize the world, simulation space and joint groups
	dInitODE();
	worldID = dWorldCreate();
	spaceID = dHashSpaceCreate(0);
	contactGroupID = dJointGroupCreate(0);

	//make sure that when we destroy the space group, we destroy all the geoms inside it
	dSpaceSetCleanup(spaceID, 1);

	//set a few of the constants that ODE needs to be aware of
	dWorldSetContactSurfaceLayer(worldID, 0.001);							// the ammount of interpenetration allowed between rbs
	dWorldSetContactMaxCorrectingVel(worldID, 1.0);							// maximum velocity that contacts are allowed to generate  

	V3D gravity = Globals::worldUp * Globals::g;
	dWorldSetGravity(worldID, gravity[0], gravity[1], gravity[2]);
}

/**
destructor
*/
ODERBEngine::~ODERBEngine(void) {
	//destroy the ODE physical world, simulation space and joint group
	dJointGroupDestroy(contactGroupID);
	dSpaceDestroy(spaceID);
	dWorldDestroy(worldID);
	dCloseODE();
}


/**
	this method is used to set up an ODE sphere geom. NOTE: ODE only allows planes to
	be specified in world coordinates, not attached to a body, so we need to fix it once and
	for all.
*/
dGeomID ODERBEngine::getPlaneGeom(PlaneCDP* p) {
	//and create the ground plane
	V3D n = p->p.n;
	V3D o = V3D(p->p.p);
	dGeomID g = dCreatePlane(spaceID, n[0], n[1], n[2], o.dot(n));
	return g;
}

/**
	this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
*/
dGeomID ODERBEngine::getSphereGeom(SphereCDP* s) {
	dGeomID g = dCreateSphere(0, s->r);
	P3D c = s->p;
	dGeomSetPosition(g, c[0], c[1], c[2]);
	return g;
}

/**
	this method is used to set up an ODE box geom. It is properly placed in body coordinates.
*/
dGeomID ODERBEngine::getBoxGeom(BoxCDP* b) {
	dGeomID g = dCreateBox(0, b->getXLen(), b->getYLen(), b->getZLen());
	P3D c = b->getCenter();
	dGeomSetPosition(g, c[0], c[1], c[2]);
	return g;
}

void ODERBEngine::setCapsuleGeomTransformation(CapsuleCDP* c, dGeomID g) {
	V3D ab = V3D(c->p1, c->p2);

	if(c->hasFlatCaps)
		dGeomCylinderSetParams(g, c->r, ab.length());
	else
		dGeomCapsuleSetParams(g, c->r, ab.length());

	P3D cen = c->p1*0.5 + c->p2*0.5;
	dGeomSetPosition(g, cen[0], cen[1], cen[2]);

	V3D defA(0, 0, 1);

	V3D axis = defA.cross(ab);

	if (!IS_ZERO(axis.length())){

		axis.toUnit();
		double rotAngle = defA.angleWith(ab);

		Quaternion relOrientation = getRotationQuaternion(rotAngle, axis);

		dQuaternion q;
		q[0] = relOrientation.s;
		q[1] = relOrientation.v[0];
		q[2] = relOrientation.v[1];
		q[3] = relOrientation.v[2];

		dGeomSetQuaternion(g, q);
	}
}

/**
	this method is used to set up an ODE sphere geom. It is properly placed in body coordinates.
*/
dGeomID ODERBEngine::getCapsuleGeom(CapsuleCDP* c) {
	P3D a = c->p1;
	P3D b = c->p2;
	V3D ab(a, b);
	dGeomID g;

	if (c->hasFlatCaps)
		g = dCreateCylinder(0, c->r, ab.length());
	else
		g = dCreateCapsule(0, c->r, ab.length());

	setCapsuleGeomTransformation(c, g);

	return g;
}

/**
this method is used to process the collision between the two rbs passed in as parameters. More generally,
it is used to determine if the collision should take place, and if so, it calls the method that generates the
contact points.
*/
void ODERBEngine::processCollisions(dGeomID o1, dGeomID o2, DynamicArray<ContactForce> &contactForces) {
	dBodyID b1, b2;
	RigidBody *rb1, *rb2;
	b1 = dGeomGetBody(o1);
	b2 = dGeomGetBody(o2);
	rb1 = (RigidBody*)dGeomGetData(o1);
	rb2 = (RigidBody*)dGeomGetData(o2);

	bool joined = b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact);

	if (joined)
		return;
	if (!rb1->rbProperties.isFrozen && !rb2->rbProperties.isFrozen)
		return;
//	if (rb1->rbProperties.isFrozen && rb2->rbProperties.isFrozen)
//		return;
//	if (rb1->rbProperties.collisionGroupID == rb2->rbProperties.collisionGroupID && rb1->rbProperties.collisionGroupID != -1)
//		return;

	//we'll use the minimum of the two coefficients of friction of the two bodies.
	double mu1 = rb1->getFrictionCoefficient();
	double mu2 = rb2->getFrictionCoefficient();
	double mu_to_use = std::min(mu1, mu2);

	//	mu_to_use = 0;

	double eps1 = rb1->getRestitutionCoefficient();
	double eps2 = rb2->getRestitutionCoefficient();
	double eps_to_use = std::min(eps1, eps2);

	int maxContactCount = sizeof(cps) / sizeof(cps[0]);
	int num_contacts = dCollide(o1, o2, maxContactCount, &(cps[0].geom), sizeof(dContact));

	// and now add them contact points to the simulation
	for (int i = 0;i<num_contacts;i++) {
		//fill in the missing properties for the contact points
		cps[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactBounce;
		cps[i].surface.mu = mu_to_use;

		cps[i].surface.bounce = eps_to_use;
		cps[i].surface.bounce_vel = 0.00001;

		cps[i].surface.soft_cfm = contactDampingCoefficient;
		cps[i].surface.soft_erp = contactStiffnessCoefficient;

		//create a joint, and link the two geometries.
		dJointID c = dJointCreateContact(worldID, contactGroupID, &cps[i]);
		dJointAttach(c, b1, b2);

		if (jointFeedbackCount >= MAX_CONTACT_FEEDBACK)
			Logger::consolePrint("Warning: too many contacts are established. Some of them will not be reported.\n");
		else {
			if (contactForces.size() != jointFeedbackCount) {
				Logger::consolePrint("Warning: Contact forces need to be cleared after each simulation, otherwise the results are not predictable.\n");
			}
			contactForces.push_back(ContactForce());
			//now we'll set up the feedback for this contact joint
			contactForces[jointFeedbackCount].rb1 = rb1;
			contactForces[jointFeedbackCount].rb2 = rb2;
			contactForces[jointFeedbackCount].cp = P3D(cps[i].geom.pos[0], cps[i].geom.pos[1], cps[i].geom.pos[2]);
			contactForces[jointFeedbackCount].n = V3D(cps[i].geom.normal[0], cps[i].geom.normal[1], cps[i].geom.normal[2]);
			dJointSetFeedback(c, &(jointFeedback[jointFeedbackCount]));
			jointFeedbackCount++;
		}
	}
}

/**
this method is used to create ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
*/
void ODERBEngine::createODECollisionPrimitives(RigidBody* body) {
	//now we'll set up the body's collision detection primitives
	for (uint j = 0;j<body->cdps.size();j++) {
		dGeomID g;
		if (SphereCDP* cdp = dynamic_cast<SphereCDP*>(body->cdps[j])) {
			g = getSphereGeom(cdp);
		}else if (CapsuleCDP* cdp = dynamic_cast<CapsuleCDP*>(body->cdps[j])) {
			g = getCapsuleGeom(cdp);
		}else if (BoxCDP* cdp = dynamic_cast<BoxCDP*>(body->cdps[j])) {
			g = getBoxGeom(cdp);
		}else if (PlaneCDP* cdp = dynamic_cast<PlaneCDP*>(body->cdps[j])) {
			g = getPlaneGeom(cdp);
			dGeomSetData(g, body);
			continue;
		}else {
			throwError("Unknown type of collision encountered...\n");
		}
		//now associate the geom to the rigid body that it belongs to, so that we can look up the properties we need later...
		dGeomSetData(g, body);

		//now we've created a geom for the current body. Note: g will be rotated relative to t, so that it is positioned
		//well in body coordinates, and then t will be attached to the body.
		dGeomID t = dCreateGeomTransform(spaceID);
		//make sure that when we destroy the transfromation, we destroy the encapsulated rbs as well.
		dGeomTransformSetCleanup(t, 1);

		//associate the transform geom with the body as well
		dGeomSetData(t, body);

		//if the object is fixed, then we want the geometry to take into account the initial position and orientation of the rigid body
		if (body->rbProperties.isFrozen) {
			dGeomSetPosition(t, body->state.position[0], body->state.position[1], body->state.position[2]);
			dQuaternion q;
			q[0] = body->state.orientation.s;
			q[1] = body->state.orientation.v[0];
			q[2] = body->state.orientation.v[1];
			q[3] = body->state.orientation.v[2];
			dGeomSetQuaternion(t, q);
		}

		dGeomTransformSetGeom(t, g);
		//now add t (which contains the correctly positioned geom) to the body, but only if the body isn't static...
		dGeomSetBody(t, odeToRbs[body->id].id);

		odeToCDP.push_back(ODE_CDP_Map_struct(g, t, body, j));
	}
}

/**
	this method is used to update ODE geoms for all the collision primitives of the rigid body that is passed in as a paramter
*/
void ODERBEngine::updateODECollisionPrimitives(RigidBody* body){
	uint nCdps = body->cdps.size();
	for (uint j = 0;j<nCdps;j++) {
		//find the geom that belongs to this CDP
		dGeomID g;
		dGeomID t;

		bool found = false;
		for (uint k = 0; k < odeToCDP.size(); k++){
			if (odeToCDP[k].rb == body && odeToCDP[k].cdpIndex == j) {
				g = odeToCDP[k].id;
				t = odeToCDP[k].t;
				found = true;
				break;
			}
		}
		if (!found) continue;

		if (SphereCDP* cdp = dynamic_cast<SphereCDP*>(body->cdps[j])) {
			dGeomSphereSetRadius(g, cdp->r);
			dGeomSetPosition(g, cdp->p[0], cdp->p[1], cdp->p[2]);
		}
		else if (CapsuleCDP* cdp = dynamic_cast<CapsuleCDP*>(body->cdps[j])) {
			setCapsuleGeomTransformation(cdp, g);
		}
		else if (BoxCDP* cdp = dynamic_cast<BoxCDP*>(body->cdps[j])) {
			dGeomBoxSetLengths(g, cdp->getXLen(), cdp->getYLen(), cdp->getZLen());
			P3D c = cdp->getCenter();
			dGeomSetPosition(g, c[0], c[1], c[2]);
		}
		else if (PlaneCDP* cdp = dynamic_cast<PlaneCDP*>(body->cdps[j])) {
			V3D n = cdp->p.n;
			V3D o = V3D(cdp->p.p);
			dGeomPlaneSetParams(g, n[0], n[1], n[2], o.dot(n));
		}
		else {
			throwError("Unknown type of collision encountered...\n");
		}

		//if the object is fixed, then we want the geometry to take into account the initial position and orientation of the rigid body, otherwise, the
		//transform should already be linked to the right ODE body, which is linked to the right rigid body...
		if (body->rbProperties.isFrozen) {
			dGeomSetPosition(t, body->state.position[0], body->state.position[1], body->state.position[2]);
			dQuaternion q;
			q[0] = body->state.orientation.s;
			q[1] = body->state.orientation.v[0];
			q[2] = body->state.orientation.v[1];
			q[3] = body->state.orientation.v[2];
			dGeomSetQuaternion(t, q);
		}

	}
}

/**
	this method is used to copy the state of the ith rigid body to its ode counterpart.
*/
void ODERBEngine::setODEStateFromRB(int i) {
	if (i<0 || (uint)i >= odeToRbs.size())
		return;

	dQuaternion tempQ;
	tempQ[0] = odeToRbs[i].rb->state.orientation.s;
	tempQ[1] = odeToRbs[i].rb->state.orientation.v[0];
	tempQ[2] = odeToRbs[i].rb->state.orientation.v[1];
	tempQ[3] = odeToRbs[i].rb->state.orientation.v[2];

	dBodySetPosition(odeToRbs[i].id, odeToRbs[i].rb->state.position[0], odeToRbs[i].rb->state.position[1], odeToRbs[i].rb->state.position[2]);
	dBodySetQuaternion(odeToRbs[i].id, tempQ);
	dBodySetLinearVel(odeToRbs[i].id, odeToRbs[i].rb->state.velocity[0], odeToRbs[i].rb->state.velocity[1], odeToRbs[i].rb->state.velocity[2]);
	dBodySetAngularVel(odeToRbs[i].id, odeToRbs[i].rb->state.angularVelocity[0], odeToRbs[i].rb->state.angularVelocity[1], odeToRbs[i].rb->state.angularVelocity[2]);
}

/**
this method is used to copy the state of the ith rigid body, from the ode object to its rigid body counterpart
*/
void ODERBEngine::setRBStateFromODE(int i) {
	const dReal *tempData;

	tempData = dBodyGetPosition(odeToRbs[i].id);
	odeToRbs[i].rb->state.position[0] = tempData[0];
	odeToRbs[i].rb->state.position[1] = tempData[1];
	odeToRbs[i].rb->state.position[2] = tempData[2];

	tempData = dBodyGetQuaternion(odeToRbs[i].id);
	odeToRbs[i].rb->state.orientation.s = tempData[0];
	odeToRbs[i].rb->state.orientation.v[0] = tempData[1];
	odeToRbs[i].rb->state.orientation.v[1] = tempData[2];
	odeToRbs[i].rb->state.orientation.v[2] = tempData[3];

	tempData = dBodyGetLinearVel(odeToRbs[i].id);
	odeToRbs[i].rb->state.velocity[0] = tempData[0];
	odeToRbs[i].rb->state.velocity[1] = tempData[1];
	odeToRbs[i].rb->state.velocity[2] = tempData[2];

	tempData = dBodyGetAngularVel(odeToRbs[i].id);
	odeToRbs[i].rb->state.angularVelocity[0] = tempData[0];
	odeToRbs[i].rb->state.angularVelocity[1] = tempData[1];
	odeToRbs[i].rb->state.angularVelocity[2] = tempData[2];
}

int ODERBEngine::getODEMotorForJoint(Joint* j) {
	HingeJoint* hj = dynamic_cast<HingeJoint*> (j);
	int motorID = -1;
	//create a motor if desired...
	if (hj) {
		dJointID aMotor = NULL;
		// Retrieve motor from map
		for (uint m = 0; m < motorToJointmap.size(); m++) {
			if (motorToJointmap[m].joint == hj) {
				aMotor = motorToJointmap[m].motorID;
				motorID = (int)m;
				break;
			}
		}
		if (aMotor == NULL) {
			aMotor = dJointCreateAMotor(worldID, 0);
			motorToJointmap.push_back(ODE_Motor_Joint_Map(aMotor, hj));
			motorID = motorToJointmap.size()-1;
			dJointAttach(aMotor, odeToRbs[(int)(hj->parent->id)].id, odeToRbs[(int)(hj->child->id)].id);
			dJointSetAMotorMode(aMotor, dAMotorUser);
			dJointSetAMotorNumAxes(aMotor, 1);
			dJointSetAMotorParam(aMotor, dParamCFM, hj->motorConstraintForceRegularizer);
			dJointSetAMotorParam(aMotor, dParamFMax, hj->maxTorque);
			//just in case the configuration of the joint changes (i.e. rotation axis), update the motor axis
			V3D a = hj->parent->getWorldCoordinates(hj->rotationAxis);
			dJointSetAMotorAxis(aMotor, 0, 1, a[0], a[1], a[2]);
		}
	}else
		Logger::consolePrint("Joint motors are only implemented for Hinge Joints for now!\n");

	return motorID;

}

/**
	This method is used to set up an ode hinge joint, based on the information in the hinge joint passed in as a parameter
*/
void ODERBEngine::setupODEHingeJoint(HingeJoint* hj) {
	dJointID j;
	uint k = 0;
	for (k = 0; k < odeToJoints.size(); k++){
		if (odeToJoints[k].j == hj){
			j = odeToJoints[k].id;
			break;
		}
	}

	if (k == odeToJoints.size()){
		j = dJointCreateHinge(worldID, 0);
		odeToJoints.push_back(ODE_Joint_Map_struct(j, hj));
		dJointAttach(j, odeToRbs[(int)(hj->child->id)].id, odeToRbs[(int)(hj->parent->id)].id);
	}

	P3D p = hj->child->getWorldCoordinates(hj->cJPos);
	dJointSetHingeAnchor(j, p[0], p[1], p[2]);
	V3D a = hj->parent->getWorldCoordinates(hj->rotationAxis);
	dJointSetHingeAxis(j, a[0], a[1], a[2]);

	//now set the joint limits
	if (hj->shouldUseJointLimits() == false)
		return;

	dJointSetHingeParam(j, dParamLoStop, hj->minAngle);
	dJointSetHingeParam(j, dParamHiStop, hj->maxAngle);
}

/**
This method is used to set up an ode universal joint, based on the information in the universal joint passed in as a parameter
*/
void ODERBEngine::setupODEUniversalJoint(UniversalJoint* uj) {
	dJointID j;
	uint k = 0;
	for (; k < odeToJoints.size(); k++){
		if (odeToJoints[k].j == uj){
			j = odeToJoints[k].id;
			break;
		}
	}

	if (k == odeToJoints.size()){
		j = dJointCreateUniversal(worldID, 0);
		odeToJoints.push_back(ODE_Joint_Map_struct(j, uj));
		dJointAttach(j, odeToRbs[(int)(uj->child->id)].id, odeToRbs[(int)(uj->parent->id)].id);
	}

	P3D p = uj->child->getWorldCoordinates(uj->cJPos);
	dJointSetUniversalAnchor(j, p[0], p[1], p[2]);

	V3D a = uj->parent->getWorldCoordinates(uj->rotAxisParent);
	V3D b = uj->child->getWorldCoordinates(uj->rotAxisChild);

	dJointSetUniversalAxis1(j, b[0], b[1], b[2]);
	dJointSetUniversalAxis2(j, a[0], a[1], a[2]);

	//now set the joint limits
	if (uj->shouldUseJointLimits() == false)
		return;

	dJointSetUniversalParam(j, dParamLoStop2, uj->minAnglePRA);
	dJointSetUniversalParam(j, dParamHiStop2, uj->maxAnglePRA);
	dJointSetUniversalParam(j, dParamLoStop, uj->minAngleCRA);
	dJointSetUniversalParam(j, dParamHiStop, uj->maxAngleCRA);
}

/**
This method is used to set up an ode ball-and-socket joint, based on the information in the ball in socket joint passed in as a parameter
*/
void ODERBEngine::setupODEBallAndSocketJoint(BallAndSocketJoint* basj) {
	dJointID j;
	uint k = 0;
	for (; k < odeToJoints.size(); k++) {
		if (odeToJoints[k].j == basj) {
			j = odeToJoints[k].id;
			break;
		}
	}
	if (k == odeToJoints.size()) {
		j = dJointCreateBall(worldID, 0);
		odeToJoints.push_back(ODE_Joint_Map_struct(j, basj));
		dJointAttach(j, odeToRbs[(int)(basj->child->id)].id, odeToRbs[(int)(basj->parent->id)].id);
	}

	P3D p = basj->child->getWorldCoordinates(basj->cJPos);
	//now we'll set the world position of the ball-and-socket joint. It is important that the bodies are placed in the world
	//properly at this point
	dJointSetBallAnchor(j, p[0], p[1], p[2]);

	//now deal with the joint limits
	if (basj->shouldUseJointLimits() == true) {
		Logger::consolePrint("Joint limits for ball and socket joints are not yet implemented!\n");
	}
}

/**
	this method is used to transfer the state of the rigid bodies, from ODE to the rigid body wrapper
*/
void ODERBEngine::setRBStateFromEngine(DynamicArray<RigidBody*> &rbs) {
	//now update all the rigid bodies...
	for (uint i = 0;i<rbs.size();i++) {
		setRBStateFromODE(i);
	}
}

/**
	this method is used to transfer the state of the rigid bodies, from the rigid body wrapper to ODE's rigid bodies
*/
void ODERBEngine::setEngineStateFromRB(DynamicArray<RigidBody*> &rbs) {
	//now update all the rigid bodies...
	for (uint i = 0;i<rbs.size();i++) {
		setODEStateFromRB(i);
	}
}

void ODERBEngine::addRigidBodyToEngine(RigidBody* rb) {
	AbstractRBEngine::addRigidBodyToEngine(rb);

	//create and link rigid body to ODE corresponding body
	dBodyID newBody = dBodyCreate(worldID);
	odeToRbs.push_back(ODE_RB_Map(newBody, rb));
	//the ID of this rigid body will be its index in the
	rb->setID(rbs.size() - 1);
	//we will use the user data of the object to store the index in this mapping as well, for easy retrieval
	dBodySetData(odeToRbs[rb->id].id, (void*)rb->id);
	//PROCESS THE COLLISION PRIMITIVES OF THE BODY
	createODECollisionPrimitives(rb);

	//if the body is fixed, we'll create constraints to keep it in place...
	if (rb->rbProperties.isFrozen) {
		dBodySetKinematic(newBody);
	}
	else {
		//SET THE INERTIAL PARAMETERS
		dMass m;

		//set the mass and principal moments of inertia for this object
		m.setZero();

		m.setParameters(rb->rbProperties.mass, 0, 0, 0,
			rb->rbProperties.MOI_local(0, 0),
			rb->rbProperties.MOI_local(1, 1),
			rb->rbProperties.MOI_local(2, 2),
			rb->rbProperties.MOI_local(0, 1),
			rb->rbProperties.MOI_local(0, 2),
			rb->rbProperties.MOI_local(1, 2));

		dBodySetMass(odeToRbs[rb->id].id, &m);
	}

	setODEStateFromRB(rb->id);
}

void ODERBEngine::addJointToEngine(Joint* j) {
	AbstractRBEngine::addJointToEngine(j);

	//update the ODE state for the parent and child rigid bodies, since the joint axis/position will be set up in world coordinates
	setODEStateFromRB(j->parent->id);
	setODEStateFromRB(j->child->id);

	//connect the joint to the two bodies
	if (BallAndSocketJoint* bJoint = dynamic_cast<BallAndSocketJoint*>(j)) {
		setupODEBallAndSocketJoint(bJoint);
	}
	else if (HingeJoint* hJoint = dynamic_cast<HingeJoint*>(j)) {
		setupODEHingeJoint(hJoint);
	}
	else if (UniversalJoint* uJoint = dynamic_cast<UniversalJoint*>(j)) {
		setupODEUniversalJoint(uJoint);
	}
	else
		throwError("Ooops.... Only BallAndSocket, Hinge and Universal joints are currently supported.\n");
}

/**
	Update ODERBEngine using current rigidbodies.
*/
void ODERBEngine::updateODERBEngineFromRBs(){
	//now we'll make sure that the joint constraints are satisfied
	for (uint i = 0;i<rbs.size();i++) {

		dBodyID odeBody = NULL;
		for (uint k = 0; k < odeToRbs.size(); k++){
			if (odeToRbs[k].rb == rbs[i]){
				odeBody = odeToRbs[k].id;
				break;
			}
		}
		if (odeBody == NULL)
			throwError("RigidBody not gound...\n");

		//PROCESS THE COLLISION PRIMITIVES OF THE BODY
		updateODECollisionPrimitives(rbs[i]);

		//reset the inertial parameters, in case they've changed...
		if (rbs[i]->rbProperties.isFrozen == false) {
			dMass m;

			//set the mass and principal moments of inertia for this object
			m.setZero();
			m.setParameters(odeToRbs[i].rb->rbProperties.mass, 0, 0, 0,
				odeToRbs[i].rb->rbProperties.MOI_local(0, 0),
				odeToRbs[i].rb->rbProperties.MOI_local(1, 1),
				odeToRbs[i].rb->rbProperties.MOI_local(2, 2),
				odeToRbs[i].rb->rbProperties.MOI_local(0, 1),
				odeToRbs[i].rb->rbProperties.MOI_local(0, 2),
				odeToRbs[i].rb->rbProperties.MOI_local(1, 2));

			dBodySetMass(odeToRbs[i].id, &m);
			setODEStateFromRB(i);
		}
	}

	//now we will go through all the joints, and update them (setup...() does update them if they exist already)
	for (uint i = 0;i<joints.size();i++) {
		//connect the joint to the two bodies
		if (BallAndSocketJoint* bJoint = dynamic_cast<BallAndSocketJoint*>(joints[i])) {
			setupODEBallAndSocketJoint(bJoint);
		}
		else if (HingeJoint* hJoint = dynamic_cast<HingeJoint*>(joints[i])) {
			setupODEHingeJoint(hJoint);
		}
		else if (UniversalJoint* uJoint = dynamic_cast<UniversalJoint*>(joints[i])) {
			setupODEUniversalJoint(uJoint);
		}
		else
			throwError("Ooops.... Only BallAndSocket, Hinge and Universal joints are currently supported.\n");
	}
}

/**
This method is used to set the state of all the rigid body in this collection.
*/
void ODERBEngine::setState(DynamicArray<double>* state, int start) {
	AbstractRBEngine::setState(state, start);
}

/**
	call back function that passes the message to the world whose rbs are being acted upon.
*/
void collisionCallBack(void* rbEngine, dGeomID o1, dGeomID o2) {
	ODERBEngine *pWorld = (ODERBEngine*)rbEngine;
	pWorld->processCollisions(o1, o2, pWorld->contactForces);
}

void ODERBEngine::setGlobalCFM(double CFM){
	dWorldSetCFM(worldID, CFM);
}

void ODERBEngine::setMotorsCFMAndFMax(double CFM, double FMAX){
	for (uint j = 0; j < joints.size(); j++){
		Joint *hj = joints[j];

		dJointID aMotor;
		bool found = false;
		// Retrieve motor from map
		for (uint m = 0; m < motorToJointmap.size(); m++){
			if (motorToJointmap[m].joint == hj){
				aMotor = motorToJointmap[m].motorID;
				found = true;
			}
		}
		if (!found)
			continue;

		dJointSetAMotorParam(aMotor, dParamCFM, CFM);
		dJointSetAMotorParam(aMotor, dParamFMax, FMAX);
	}
}

/**
This method is used to integrate the forward simulation in time.
*/
void ODERBEngine::step(double deltaT) {
	//make sure that the state of the RB's is synchronized with the engine...
	setEngineStateFromRB(rbs);

	//restart the counter for the joint feedback terms
	jointFeedbackCount = 0;

	//apply control inputs as needed...
	for (uint j = 0;j<joints.size();j++) {
		if (joints[j]->controlMode == POSITION_MODE || joints[j]->controlMode == VELOCITY_MODE) {
			int motorID = getODEMotorForJoint(joints[j]);
			if (motorID >= 0) {
				//the motor may have been placed in limbo, so reattach it to its rigid bodies...
				dJointAttach(motorToJointmap[motorID].motorID, odeToRbs[(int)(joints[j]->parent->id)].id, odeToRbs[(int)(joints[j]->child->id)].id);

				if (joints[j]->controlMode == POSITION_MODE) {
					Quaternion qRel = joints[j]->computeRelativeOrientation();//this is the current relative orientation of the 
					Quaternion qRelD = motorToJointmap[motorID].joint->desiredRelativeOrientation;//and this is the desired relative orientation for the joint
					//q and -q represent the same rotation, so make sure we try to get the shortest path from qRel to qRelD...
					if (qRelD.s * qRel.s < 0) qRelD *= -1;
					Quaternion qErr = qRel.getInverse() * qRelD;
					//ODE works in world coordinates, so...
					V3D rotAxis = joints[j]->parent->getWorldCoordinates(qErr.v.unit());
					if (rotAxis.length() < 0.9) rotAxis = Globals::worldUp;
					double rotAngle = qErr.getRotationAngle(qErr.v);
					double desSpeed = -rotAngle / deltaT;
					boundToRange(&desSpeed, -joints[j]->maxSpeed, joints[j]->maxSpeed);
//					Logger::consolePrint("joint: %d - desired speed: %lf (%lf %lf %lf %lf)\n", j, desSpeed, qErr.s, qErr.v[0], qErr.v[1], qErr.v[2]);
//					Logger::consolePrint("joint: %d\t%lf\n", j, desSpeed);
					dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamCFM, joints[j]->motorConstraintForceRegularizer);
					dJointSetAMotorAxis(motorToJointmap[motorID].motorID, 0, 1, rotAxis[0], rotAxis[1], rotAxis[2]);
					dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamVel, desSpeed);
				} else {
					//the desired relative angular velocity needs to be expressed in world coordinates...
					V3D rotAxis = joints[j]->parent->getWorldCoordinates(joints[j]->desiredRelativeAngularVelocity.unit());
					rotAxis.toUnit(); if (rotAxis.length() < 0.9) rotAxis = V3D(1,0,0);
					double angVelocity = joints[j]->desiredRelativeAngularVelocity.length();
					dJointSetAMotorNumAxes(motorToJointmap[motorID].motorID, 1);
					dJointSetAMotorAxis(motorToJointmap[motorID].motorID, 0, 1, rotAxis[0], rotAxis[1], rotAxis[2]);
					dJointSetAMotorParam(motorToJointmap[motorID].motorID, dParamVel, -angVelocity);
				}
				//setup the feedback structure so that we can read off the torques that were applied
				if (j < MAX_AMOTOR_FEEDBACK)
					dJointSetFeedback(motorToJointmap[motorID].motorID, &(amotorFeedback[j]));
			}
			else
				Logger::consolePrint("Warning: control mode for joint %d requires a motor, but none has been created...\n");
		}

		if (joints[j]->controlMode == PASSIVE) {
			int motorID = getODEMotorForJoint(joints[j]);
			//we need to place the motor in limbo, such that it does not interfere with torque controller...
			if (motorID >= 0)
				dJointAttach(motorToJointmap[motorID].motorID, 0, 0);
		}

		if (joints[j]->controlMode == TORQUE_MODE) {
			int motorID = getODEMotorForJoint(joints[j]);
			//we need to place the motor in limbo, such that it does not interfere with torque controller...
			if (motorID >= 0)
				dJointAttach(motorToJointmap[motorID].motorID, 0, 0);

			V3D t = joints[j]->desiredJointTorque;
			//we will apply to the parent a positive torque, and to the child a negative torque
			dBodyAddTorque(odeToRbs[joints[j]->parent->id].id, -t[0], -t[1], -t[2]);
			dBodyAddTorque(odeToRbs[joints[j]->child->id].id, t[0], t[1], t[2]);
		}
	}

	//get an up-to-date list of contact forces...
	contactForces.clear();
	dJointGroupEmpty(contactGroupID);
	//initiate the collision detection
	dSpaceCollide(spaceID, this, &collisionCallBack);

	//advance the simulation
	if (iterativeSolution == false)
		dWorldStep(worldID, deltaT);
	else
		dWorldQuickStep(worldID, deltaT);

	//copy over the state of the ODE bodies to the rigid bodies...
	setRBStateFromEngine(rbs);

	//copy over the torque information...
	for (uint j = 0;j<joints.size();j++) {
		if (joints[j]->controlMode != POSITION_MODE && joints[j]->controlMode != VELOCITY_MODE)
			continue;
		int motorID = getODEMotorForJoint(joints[j]);

		if (motorID >= 0)// If this is a hinge joint with a motor attached, then we've used the motors, so see what torque they ended up applying...
			if (j < MAX_AMOTOR_FEEDBACK)
				joints[j]->desiredJointTorque = V3D(amotorFeedback[j].t1[0], amotorFeedback[j].t1[1], amotorFeedback[j].t1[2]);
	}

	//copy over the force information for the contact forces
	for (int i = 0;i<jointFeedbackCount;i++) {
		contactForces[i].f = V3D(jointFeedback[i].f1[0], jointFeedback[i].f1[1], jointFeedback[i].f1[2]);
		//make sure that the force always points away from the static rbs
		if (contactForces[i].rb1->rbProperties.isFrozen && !contactForces[i].rb2->rbProperties.isFrozen) {
			contactForces[i].n = contactForces[i].n * (-1);
			contactForces[i].f = contactForces[i].f * (-1);
			RigidBody* tmpBdy = contactForces[i].rb1;
			contactForces[i].rb1 = contactForces[i].rb2;
			contactForces[i].rb2 = tmpBdy;
		}
	}
	markRBContacts(0.1);
}


/**
this method applies a force to a rigid body, at the specified point. The point is specified in local coordinates,
and the force is specified in world coordinates.
*/
void ODERBEngine::applyForceTo(RigidBody* b, const V3D& f, const P3D& p) {
	if (!b)
		return;
	dBodyAddForceAtRelPos(odeToRbs[b->id].id, f[0], f[1], f[2], p[0], p[1], p[2]);
}


/**
this method applies a torque to a rigid body. The torque is specified in world coordinates.
*/
void ODERBEngine::applyTorqueTo(RigidBody* b, const V3D& t) {
	if (!b)
		return;
	dBodyAddTorque(odeToRbs[b->id].id, t[0], t[1], t[2]);
}

/**
this method applies a torque to a rigid body. The torque is specified in relative body coordinates.
*/
void ODERBEngine::applyRelativeTorqueTo(RigidBody* b, const V3D& t) {
	if (!b)
		return;
	dBodyAddRelTorque(odeToRbs[b->id].id, t[0], t[1], t[2]);
}


/*
this method finds the total contact force applied on the given rigid body if it is in contact with another body.
(e.g, finding ground reaction forces applied on the foot of the character)
*/
DynamicArray<ContactForce> ODERBEngine::getContactForceOnRB(RigidBody* bs) {
	
	DynamicArray<ContactForce> grfs;

	for (std::vector<ContactForce>::iterator it = contactForces.begin(); it != contactForces.end(); ++it) 
		if (((*it).rb1->id == bs->id) || ((*it).rb2->id == bs->id)) {
			grfs.push_back(*it);
		}

	return grfs;
}
