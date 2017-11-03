#include <RobotDesignerLib/LocomotionEngineMotionPlan.h>
#include <ControlLib/BodyFrame.h>
#include <ControlLib/GenericLimb.h>
#include <Utils/Utils.h>
#include <MathLib/MathLib.h>
#include <GUILib/GLUtils.h>
#include <MathLib/ConvexHull3D.h>
#include <set>

LocomotionEngineMotionPlan::LocomotionEngineMotionPlan(Robot* robot, int nSamplingPoints){
	wrapAroundBoundaryIndex = -1;
	optimizeCOMPositions = true;
	optimizeEndEffectorPositions = false;
	optimizeBarycentricWeights = true;
	optimizeRobotStates = true;
	optimizeContactForces = true;
	enforceGRFConstraints = false;
	desTurningAngle = 0;

	this->robot = robot;
	this->nSamplePoints = nSamplingPoints;

	//first off, compute the current position of the COM, and ensure the whole robot lies on the ground
	totalMass = 0;
	P3D comPos;

	//this is the position of the "COM"
	for (uint k = 0; k<robot->bFrame->bodyLinks.size(); k++) {
		totalMass += robot->bFrame->bodyLinks[k]->rbProperties.mass;
		comPos += robot->bFrame->bodyLinks[k]->getCMPosition() * robot->bFrame->bodyLinks[k]->rbProperties.mass;
	}
	comPos /= totalMass;

	//approximate the inertia of the body
	double Ixx = robot->root->rbProperties.MOI_local.coeff(0, 0); // = m(y2 + z2)/12
	double Iyy = robot->root->rbProperties.MOI_local.coeff(1, 1); // = m(z2 + x2)/12
	double Izz = robot->root->rbProperties.MOI_local.coeff(2, 2); // = m(y2 + x2)/12
	double x = sqrt((Iyy + Izz - Ixx) * 6 / robot->root->rbProperties.mass);
	double y = sqrt((Izz + Ixx - Iyy) * 6 / robot->root->rbProperties.mass);
	double z = sqrt((Ixx + Iyy - Izz) * 6 / robot->root->rbProperties.mass);
	double bodyVolume = x * y * z;
	double approxDim = pow(bodyVolume, 1 / 3.0);
	totalInertia = totalMass * (approxDim*approxDim + approxDim*approxDim) / 12;

	//and average position of the feet. We want to make sure they end up touching the ground
	double eeY = 0;
	int totalNEEs = 0;
	int nLegs = this->robot->bFrame->limbs.size();
	for (int i = 0; i<nLegs; i++) {
		int nEEs = this->robot->bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPointCount();
		for (int j = 0; j<nEEs; j++) {
			P3D eeLocalCoords = this->robot->bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPoint(j);
			P3D eeWorldCoords = this->robot->bFrame->limbs[i]->getLastLimbSegment()->getWorldCoordinates(eeLocalCoords);
			eeY += eeWorldCoords[1];
			totalNEEs++;
		}
	}
	if (totalNEEs > 0) eeY /= totalNEEs;
	comPos[1] -= eeY;
	desCOMHeight = comPos[1];
	defaultCOMPosition = comPos;
	//move the whole robot so that the feet are on the ground
	ReducedRobotState rs(robot);
	P3D rootPos = rs.getPosition();
	rootPos.addOffsetToComponentAlong(V3D(0, 1, 0), -eeY);
	rs.setPosition(rootPos);
	robot->setState(&rs);

	//and now proceed to initialize everything else...
	this->robotRepresentation = new GeneralizedCoordinatesRobotRepresentation(robot);
	//create the end effector trajectories here based on the robot configuration...
	for (int i=0;i<nLegs;i++){
		int nEEs = this->robot->bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPointCount();
		for (int j=0; j<nEEs; j++){
			P3D eeLocalCoords = this->robot->bFrame->limbs[i]->getLastLimbSegment()->rbProperties.getEndEffectorPoint(j);
			P3D eeWorldCoords = this->robot->bFrame->limbs[i]->getLastLimbSegment()->getWorldCoordinates(eeLocalCoords);
			eeWorldCoords[1] = 0;

			endEffectorTrajectories.push_back(LocomotionEngine_EndEffectorTrajectory(nSamplingPoints));
			int index = endEffectorTrajectories.size() - 1;
			endEffectorTrajectories[index].CPIndex = j;
			endEffectorTrajectories[index].targetOffsetFromCOM = V3D(this->robot->getRoot()->getCMPosition(), eeWorldCoords);

			endEffectorTrajectories[index].theLimb = this->robot->bFrame->limbs[i];
			endEffectorTrajectories[index].endEffectorRB = this->robot->bFrame->limbs[i]->getLastLimbSegment();
			endEffectorTrajectories[index].endEffectorLocalCoords = eeLocalCoords;

			for (int k=0;k<nSamplingPoints;k++){
				endEffectorTrajectories[index].EEPos[k] = eeWorldCoords;
				endEffectorTrajectories[index].defaultEEPos[k] = endEffectorTrajectories[index].EEPos[k];
				endEffectorTrajectories[index].contactFlag[k] = 1.0;
			}
		}
	}

	robotStateTrajectory.robotRepresentation = this->robotRepresentation;
	robotStateTrajectory.initialize(nSamplingPoints);

	COMTrajectory.initialize(nSamplePoints, comPos, robotRepresentation->getQAxis(3),
		robotRepresentation->getQAxis(4), robotRepresentation->getQAxis(5));

	verticalGRFLowerBoundVal = fabs(totalMass * Globals::g / endEffectorTrajectories.size() / 5.0) * 2;
//	verticalGRFLowerBoundVal = fabs(totalMass * Globals::g / robot->bFrame->limbs.size() / 5.0) * 2;
//	minVerticalGRFEpsilon = fabs(totalMass * Globals::g / robot->bFrame->limbs.size() / 5.0) * 0.5;
}

//syncs the footfall pattern with the current motion plan
void LocomotionEngineMotionPlan::syncFootFallPatternWithMotionPlan(FootFallPattern& ffp) {
	int nPoints = nSamplePoints;
	//if this is a periodic motion, then we know we know the last data point is the exact same as the first...
	if (wrapAroundBoundaryIndex != -1)
		nPoints -= 1;

	ffp = FootFallPattern();
	ffp.strideSamplePoints = nPoints;
	for (uint i = 0; i < robot->bFrame->limbs.size(); i++) {
		//find an end effector that belongs to this limb...
		int eeIndex = 0;
		for (uint j = 0; j < endEffectorTrajectories.size(); j++)
			if (endEffectorTrajectories[j].theLimb == robot->bFrame->limbs[i])
				eeIndex = j;
		//now we have a few different cases to handle... the limb can be always in swing, always in stance, or a mix...
		int firstSwingIndex = -1;
		bool alwaysInSwing = true; 
		bool alwaysInStance = true;
		for (int j = 0; j < nPoints; j++) {
			if (endEffectorTrajectories[eeIndex].contactFlag[j] == 0) //0 means in swing
				alwaysInStance = false;
			else
				alwaysInSwing = false;
			if (endEffectorTrajectories[eeIndex].contactFlag[j] == 0 && firstSwingIndex == -1)
				firstSwingIndex = j;
		}

		//so, if always in swing...
		if (alwaysInSwing){
			ffp.addStepPattern(endEffectorTrajectories[eeIndex].theLimb, 0, nPoints-1);
			continue;
		}
		
		//if always in stance...
		if (alwaysInStance) {
//			ffp.addStepPattern(endEffectorTrajectories[eeIndex].theLimb, 0, 0);
			continue;
		}

		//ok, otherwise, we need to find the proper start and end...
		if (firstSwingIndex == 0) {
			if (endEffectorTrajectories[eeIndex].contactFlag[nPoints-1] == 0){
				firstSwingIndex = nPoints - 1;
				while (endEffectorTrajectories[eeIndex].contactFlag[firstSwingIndex - 1] == 0) firstSwingIndex -= 1;
			}
		}
		int lastSwingIndex = firstSwingIndex;
		while (endEffectorTrajectories[eeIndex].contactFlag[(lastSwingIndex + 1) % nPoints] == 0) lastSwingIndex = (lastSwingIndex + 1) % nPoints;
		ffp.addStepPattern(endEffectorTrajectories[eeIndex].theLimb, firstSwingIndex, lastSwingIndex);
	}
}

void LocomotionEngineMotionPlan::syncMotionPlanWithFootFallPattern(FootFallPattern& ffp){
	std::vector<std::vector<double> > yPositions;
	for (uint i=0;i<endEffectorTrajectories.size();i++){
		yPositions.push_back(std::vector<double>());
		GenericLimb* limb = endEffectorTrajectories[i].theLimb;

		for (int j=0;j<nSamplePoints;j++){
			yPositions.back().push_back(0);

			if (ffp.isInSwing(limb, j)){
				if (!ffp.isStart(limb, j) || ffp.isAlwaysInSwing(limb)){
					double swingPhase = ffp.getSwingPhaseForTimeIndex(limb, j);
					double heightRatio = 1 - fabs(0.5 - swingPhase) / 0.5;
//					Logger::consolePrint("swing phase: %lf height: %lf\n", swingPhase, heightRatio);
//					heightRatio = 1.0;
					yPositions.back().back() = swingFootHeight * heightRatio;
				}
			}
		}
	}
	syncMotionPlanWithFootFallPattern(ffp,yPositions);
}

void LocomotionEngineMotionPlan::syncMotionPlanWithFootFallPattern(FootFallPattern& ffp, const std::vector<std::vector<double> > &yPositions){
	for (int j=0;j<nSamplePoints;j++){
		int nStanceLimbs = 0;
		double sumWeights = 0;
		for (uint i=0;i<this->endEffectorTrajectories.size();i++){
			GenericLimb* limb = endEffectorTrajectories[i].theLimb;
			if (ffp.isInStance(limb, j)){
				nStanceLimbs += 1;
				sumWeights += endEffectorTrajectories[i].EEWeights[j];
			}
		}

		for (uint i=0;i<endEffectorTrajectories.size();i++){
			GenericLimb* limb = endEffectorTrajectories[i].theLimb;
			endEffectorTrajectories[i].contactFlag[j] = 1;
			endEffectorTrajectories[i].EEPos[j][1] = 0;

			if (ffp.isInSwing(limb, j)){
				endEffectorTrajectories[i].contactFlag[j] = 0;
//				endEffectorTrajectories[i].EEPos[j][1] = yPositions[i][j];
			}

			if (ffp.isInStance(limb, j)){
				if (sumWeights <= 0){
					endEffectorTrajectories[i].EEWeights[j] = 1.0 / nStanceLimbs;
				}else
					endEffectorTrajectories[i].EEWeights[j] /= sumWeights;
			}
		}
	}

/*
	//debug...
	FootFallPattern other;
	syncFootFallPatternWithMotionPlan(other);
	if (ffp.isSameAs(other))
		Logger::consolePrint("footfall patterns are the same!\n");
	else {
		Logger::consolePrint("footfall patterns are NOT the same!\n");
		ffp.writeToFile("../out/ffpOrig.ffp");
		other.writeToFile("../out/ffpOther.ffp");
//		exit(0);
	}
*/
}

LocomotionEngineMotionPlan::~LocomotionEngineMotionPlan(void){
	delete robotRepresentation;
}

void LocomotionEngineMotionPlan::updateRobotRepresentation(){
//	this->robotRepresentation->setupDOFAxesAndParentOffsets();
	for(uint i = 0; i < endEffectorTrajectories.size(); i++)
		endEffectorTrajectories[i].endEffectorLocalCoords = endEffectorTrajectories[i].theLimb->getLastLimbSegment()->rbProperties.getEndEffectorPoint(endEffectorTrajectories[i].CPIndex);
}

/*
void LocomotionEngineMotionPlan::writeRobotMotionAnglesToFile(const char* fName)
{
	auto& qArray = robotStateTrajectory.qArray;
	int nStateDim = robotStateTrajectory.nStateDim;
	set<int> flipIndice = {1, 3, 4, 6, 8, 10, 12, 14};

	FILE* fp = fopen(fName, "w+");
	fprintf(fp, "StateNum: %d, MotorNum: %d\n", qArray.size() - 1, robot->getJointCount());
	fprintf(fp, "{\n");
	//every line in the file will correspond to all joint angles of one state
	for (int i = 0; i < (int)qArray.size() - 1; i++) {
		// hard code for wheel example.
		fprintf(fp, "{");
		for (int j = 0; j < robot->getJointCount(); j++) {
			double scale = flipIndice.count(j) ? -1 : 1;
			int jIndex = robotRepresentation->getQIndexForJoint(robot->getJoint(j));
			fprintf(fp, "%lf", DEG(qArray[i][jIndex]) * scale);
			if (j < robot->getJointCount() - 1)
				fprintf(fp, ", ");
		}
		fprintf(fp, "},\n");
	}
	fprintf(fp, "},\n");

	fclose(fp);
}
*/
P3D LocomotionEngineMotionPlan::getCOP(int tIndex) {
	P3D x = COMTrajectory.getCOMPositionAtTimeIndex(tIndex);
	double h = x[1];
	x[1] = 0;

	//GLUtils::glLColor(1,1,1);
	//GLUtils::drawSphere(x, 0.01);
	int nIntervalCounts = nSamplePoints - 1;
	double t = motionPlanDuration / (nIntervalCounts);
	int jmm, jm, jp, jpp;
	getAccelerationTimeIndicesFor(tIndex, jmm, jm, jp, jpp);
	if (jmm >= 0 && jm >= 0 && jp >= 0 && jpp >= 0) {
		V3D vp = (COMTrajectory.getCOMPositionAtTimeIndex(jpp) - COMTrajectory.getCOMPositionAtTimeIndex(jp)) / t;
		V3D vm = (COMTrajectory.getCOMPositionAtTimeIndex(jm) - COMTrajectory.getCOMPositionAtTimeIndex(jmm)) / t;
		V3D xDotDot = (vp - vm) / t;
		double hDotDot = xDotDot[1];
		xDotDot[1] = 0;

		hDotDot = 0;
		h = desCOMHeight;
		V3D zmp = V3D(x) - xDotDot * (h / (hDotDot + fabs(Globals::g)));
		return P3D() + zmp;
	}
	return x;
}

void LocomotionEngineMotionPlan::getVelocityTimeIndicesFor(int tIndex, int &tm, int &tp, bool wrapAround) const {
	tp = tIndex; tm = tIndex-1;
	if (tm < 0){
		if (wrapAroundBoundaryIndex==0 && wrapAround)
			tm = nSamplePoints-2;
		else
			tm = -1;
	}

	if (tp == nSamplePoints-1){
		if (wrapAroundBoundaryIndex >= 0 && wrapAround){
			tp = wrapAroundBoundaryIndex;
		}
		else{
			tp = -1;
		}
	}
}

void LocomotionEngineMotionPlan::getAccelerationTimeIndicesFor(int tIndex, int& tmm, int& tm, int& tp, int& tpp, bool wrapAround){
	tp = tIndex, tpp = tIndex+1, tm = tIndex, tmm=tIndex-1;
	if (tm == 0){
		if (wrapAroundBoundaryIndex==0 && wrapAround){
			tm = nSamplePoints-1;
			tmm = nSamplePoints-2;
		}
		else{
			tm = -1;
			tmm = -1;
		}
	}

	if (tp == nSamplePoints-1){
		if (wrapAroundBoundaryIndex >= 0 && wrapAround){
			tp = wrapAroundBoundaryIndex;
			tpp = wrapAroundBoundaryIndex+1;
		}
		else{
			tp = -1;
			tpp = -1;
		}
	}
}

bool LocomotionEngineMotionPlan::getJointAngleVelocityProfile(std::vector<JointVelocity> &velocityProfile, std::string &error) const
{
	if (robotStatesParamsStartIndex < 0)
	{
		error = "robotStatesParamsStartIndex < 0";
		return false;
	}

	int startQIndex = 6;
	int endQIndex = robotRepresentation->getDimensionCount() - 1;

	int nTimeSteps = nSamplePoints;
	if (wrapAroundBoundaryIndex >= 0) nTimeSteps--;

	for (int j=0; j<nTimeSteps; j++){

		int jm, jp;

		getVelocityTimeIndicesFor(j, jm, jp);
		if (jm == -1 || jp == -1) continue;

		double dt = motionPlanDuration / nSamplePoints;
		double t = (double)j*dt;

		for (int i=startQIndex; i<=endQIndex; i++){
			double velocity = (robotStateTrajectory.qArray[jp][i] - robotStateTrajectory.qArray[jm][i]) / dt;
			velocityProfile.push_back(JointVelocity(t, i, velocity));
		}
	}

	return true;
}

//TODO: redo motion plan animation to update state in terms of deltas, rather than the animation cycle thing?
void LocomotionEngineMotionPlan::drawMotionPlan(double f, int animationCycle, bool drawRobot, bool drawSkeleton, bool drawPlanDetails, bool drawContactForces, bool drawOrientation){
	if (drawPlanDetails){
		//draw the support polygon...
		DynamicArray<int> stanceLimbs;
		DynamicArray<P3D> limbPositions;
		for (uint i=0;i<endEffectorTrajectories.size();i++){
			if (endEffectorTrajectories[i].isInStance(f)){
				stanceLimbs.push_back(i);
				limbPositions.push_back(endEffectorTrajectories[i].getEEPositionAt(f));
			}
		}
		DynamicArray<ConvexHull2D_Vertex> convexHullVertices;
		ConvexHull3D::planarConvexHullFromSetOfPoints(limbPositions, V3D(0, 1, 0), convexHullVertices);

		glDisable(GL_LIGHTING);
		glColor3d(1,0,0);
		glLineWidth(3.0);
		glBegin(GL_LINE_LOOP);
		for (uint i=0;i<convexHullVertices.size();i++){
			int limbIndex = stanceLimbs[convexHullVertices[i].index];
			P3D p = endEffectorTrajectories[limbIndex].getEEPositionAt(f);
			glVertex3d(p[0], p[1]+0.001, p[2]);
		}
		glEnd();
		glLineWidth(1.0);
		//-----------------------------------

		//draw the COP and projected COM
		//glEnable(GL_LIGHTING);

		//----------------------------------

		//draw the COP trajectory
		if (!drawContactForces){
			int nIntervalCounts = nSamplePoints - 1;
			int tIndex = (int)(f * nIntervalCounts);

			glColor3d(0.776, 0.733, 0.223);
			drawSphere(getCOP(tIndex), 0.003);

			glLineWidth(5);
			glColor3d(0.776, 0.733, 0.223);
			glBegin(GL_LINE_STRIP);
				for (int i=0; i<nSamplePoints;i++){
					glColor3d(0.8,0.8,0.8);
					//GLUtils::drawSphere(x, 0.01);
					P3D zmp = getCOP(i);
					glVertex3d(zmp[0], 0.0001, zmp[2]);
				}
			glEnd();
			glLineWidth(1.0);
		}
		//------------------------------------

		//draw end effector trajectories
		glColor3d(0.3, 0.3, 0.8);
		for (uint i=0;i<endEffectorTrajectories.size();i++){
			glBegin(GL_LINE_STRIP);
			for (double j=0;j<=1; j+=0.01){
				P3D p = endEffectorTrajectories[i].getEEPositionAt(j);
				glVertex3d(p[0], p[1], p[2]);
			}
			glEnd();
		}
		//----------------------------

		//draw the COM trajectory
		glColor3d(1, 0, 0);
		P3D comP = COMTrajectory.getCOMPositionAt(f);
		drawSphere(comP, 0.003);
		comP.y() = 0.001;
		drawSphere(comP, 0.003);

		glLineWidth(5);
		glColor3d(0.7, 0.7, 0.7);
		glBegin(GL_LINE_STRIP);
			for (int i=0; i<nSamplePoints;i++){
				glVertex3d(COMTrajectory.pos[0][i], 0.001, COMTrajectory.pos[2][i]);
			}
		glEnd();
		glLineWidth(1.0);

		glLineWidth(5);
		glColor3d(0.0, 0.5, 0.0);
		glBegin(GL_LINE_STRIP);
		for (int i = 0; i<nSamplePoints; i++) {
			glVertex3d(COMTrajectory.pos[0][i], COMTrajectory.pos[1][i], COMTrajectory.pos[2][i]);
		}
		glEnd();
		glLineWidth(1.0);

		//----------------------------

		//draw end effector positions...
		for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
			if (endEffectorTrajectories[i].isInStance(f))
				glColor3d(1, 0, 0);
			else
				glColor3d(0, 1, 0);
			drawSphere(endEffectorTrajectories[i].getEEPositionAt(f), 0.003);
		}
	}


	if (robot && (drawRobot || drawSkeleton)){
		ReducedRobotState oldState(robot);
		////draw the robot's COM trajectory
		//glLineWidth(5);
		//glColor3d(0.3, 0.5, 0.3);
		//glBegin(GL_LINE_STRIP);
		//	for (double j=0;j<=1; j+=0.01){
		//		ReducedRobotState robotState(robot);
		//		robotStateTrajectory.getRobotStateAt(j, robotState);
		//		robot->setState(&robotState);
		//		P3D p = robot->bFrame->bodyState.position;
		//		glVertex3d(p[0], p[1]*0+0.001, p[2]);
		//	}
		//glEnd();
		//------------------------------------
		ReducedRobotState robotState(robot);
		robotStateTrajectory.getRobotPoseAt(f, robotState);

		dVector q; 
		robotStateTrajectory.getQAtTimeIndex(0, q);
		Quaternion q0 = getRotationQuaternion(q[3], robotStateTrajectory.robotRepresentation->getQAxis(3)) *
			getRotationQuaternion(q[4], robotStateTrajectory.robotRepresentation->getQAxis(4)) *
			getRotationQuaternion(q[5], robotStateTrajectory.robotRepresentation->getQAxis(5));

		robotStateTrajectory.getQAtTimeIndex(nSamplePoints-1, q);
		Quaternion q1 = getRotationQuaternion(q[3], robotStateTrajectory.robotRepresentation->getQAxis(3)) *
			getRotationQuaternion(q[4], robotStateTrajectory.robotRepresentation->getQAxis(4)) *
			getRotationQuaternion(q[5], robotStateTrajectory.robotRepresentation->getQAxis(5));


		Quaternion qRel = q0.getInverse() * q1;
		V3D pRel = robotStateTrajectory.getBodyPositionAt(1) - robotStateTrajectory.getBodyPositionAt(0);

		Quaternion qRoot = robotState.getOrientation();
		P3D pRoot = robotStateTrajectory.getBodyPositionAt(0);
		V3D vecToRoot(robotStateTrajectory.getBodyPositionAt(0), robotState.getPosition());
		for (int ii = 0; ii < animationCycle; ii++){
			qRoot = qRel * qRoot;
			pRoot += pRel;
			pRel = qRel * pRel;
			vecToRoot = qRel * vecToRoot;
		}

//		if (f == 0) {
//			Logger::consolePrint("Starting at: %lf %lf %lf\n", pRoot[0], pRoot[1], pRoot[2]);
//			Logger::consolePrint("  Ending at: %lf %lf %lf\n", pRoot[0] + pRel[0], pRoot[1] + pRel[1], pRoot[2] + pRel[2]);
//		}

		robotState.setOrientation(qRoot);
		robotState.setPosition(pRoot + vecToRoot);
		robot->setState(&robotState);
		
		
		//draw the axes of rotation...
		//for (int i=0;i<robot->getJointCount();i++){
		//	HingeJoint* hj = dynamic_cast<HingeJoint*> (robot->getJoint(i));
		//	V3D v = hj->child->getWorldCoordinates(hj->getRotAxisA()) * 0.1;
		//	P3D p = hj->child->getWorldCoordinates(hj->cJPos);
		//	glBegin(GL_LINES);
		//		glVertex3d(p[0], p[1], p[2]);
		//		glVertex3d(p[0]+v[0], p[1]+v[1], p[2]+v[2]);
		//	glEnd();
		//}
		//----------------------------

		glEnable(GL_LIGHTING);


		//--------------------------------------

//		int flags = SHOW_ABSTRACT_VIEW;

		int flags = SHOW_MESH;
		if (drawPlanDetails || drawSkeleton)
			flags = SHOW_ABSTRACT_VIEW;

		//draw the robot configuration...
		robot->getRoot()->draw(flags);
		for (int i=0;i<robot->getJointCount();i++)
			robot->getJoint(i)->child->draw(flags);
		//-------------------------------------

		robot->setState(&oldState);	
	}

	glEnable(GL_LIGHTING);

	// drawContactForce
	if (drawPlanDetails && drawContactForces) {
		glColor3d(0.0, 1.0, 0.0);
		for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
			V3D contactForce = endEffectorTrajectories[i].getContactForceAt(f);
			P3D EEPos = endEffectorTrajectories[i].getEEPositionAt(f);
			drawArrow(EEPos, EEPos + contactForce * 0.05, 0.007, 12);
		}
	}

	// drawOrientation
	if (drawPlanDetails && drawOrientation) {
		glPushMatrix();
		P3D comPos = COMTrajectory.getCOMPositionAt(f);
		glTranslated(comPos[0], comPos[1], comPos[2]);
		//and rotation part
		V3D rotAxis; double rotAngle;
		Quaternion q = COMTrajectory.getCOMOrientationAt(f);
		q.getAxisAngle(rotAxis, rotAngle);
		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);

		glColor4d(0.0, 1.0, 1.0, 0.3);
		drawMOIApproximation(Matrix3x3::Identity() * totalInertia, totalMass);
		/*glColor3d(1.0, 1.0, 0.0);
		drawArrow(P3D(), P3D(0.1, 0, 0), 0.005, 12);
		glColor3d(0.0, 1.0, 1.0);
		drawArrow(P3D(), P3D(0, 0.1, 0), 0.005, 12);
		glColor3d(1.0, 0.0, 1.0);
		drawArrow(P3D(), P3D(0, 0, 0.1), 0.005, 12);*/
		glPopMatrix();
	}

	glDisable(GL_LIGHTING);
}

void LocomotionEngineMotionPlan::drawMotionPlan2(double f, int animationCycle, bool drawRobotPose, bool drawPlanDetails) {
	int nIntervalCounts = nSamplePoints - 1;
	int tIndex = (int)(f * nIntervalCounts);

	double alphaVal = 1.0;
	if (animationCycle == 6)
		alphaVal = 1-mapTo01Range(f, 0, 0.2);
	if (animationCycle > 6)
		alphaVal = 0.0;

	//draw the robot in a neutral stance
	if (animationCycle == 0 && f < 0.95) {
		ReducedRobotState oldState(robot);
		ReducedRobotState robotState(robot);
		robotStateTrajectory.getRobotPoseAt(0, robotState);
		robotState.setOrientation(Quaternion());
		P3D pos = (COMTrajectory.getCOMPositionAt(1) + COMTrajectory.getCOMPositionAt(0)) / 2;
		P3D oldPos = robotState.getPosition();
		pos[1] = oldPos[1];
		robotState.setPosition(pos);
		for (int i = 0; i < robot->getJointCount(); i++)
			robotState.setJointRelativeOrientation(Quaternion(), i);
		robot->setState(&robotState);
		glEnable(GL_LIGHTING);

		//draw end effector positions...
		for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
			if (endEffectorTrajectories[i].isInSwing(f))
				glColor3d(1, 0, 0);
			else
				glColor3d(0, 1, 0);
			drawSphere(endEffectorTrajectories[i].theLimb->getLastLimbSegment()->getWorldCoordinates(endEffectorTrajectories[i].theLimb->getLastLimbSegment()->rbProperties.getEndEffectorPoint(endEffectorTrajectories[i].CPIndex)), 0.02);
		}

		//draw the robot configuration...
		robot->getRoot()->draw(SHOW_ABSTRACT_VIEW);
		for (int i = 0; i<robot->getJointCount(); i++)
			robot->getJoint(i)->child->draw(SHOW_ABSTRACT_VIEW);

		glDisable(GL_LIGHTING);

		robot->setState(&oldState);
	}

	//draw the entire COM trajectory, together with the COM position at intermediate points...
	if ((animationCycle >= 2 && alphaVal > 0) || (animationCycle == 0 && f < 0.5)) {
		glEnable(GL_LIGHTING);
		glColor4d(0x00/255.0, 0x3a/255.0, 0x6f/255.0, alphaVal);
		for (int i = 1; i<nSamplePoints; i++) 
			drawCapsule(P3D(COMTrajectory.pos[0][i-1], COMTrajectory.pos[1][i-1], COMTrajectory.pos[2][i-1]), P3D(COMTrajectory.pos[0][i], COMTrajectory.pos[1][i], COMTrajectory.pos[2][i]), 0.001);

		drawSphere(COMTrajectory.getCOMPositionAtTimeIndex(tIndex), 0.01);
	}

	//draw the end effectors
	if (animationCycle >= 1 && alphaVal > 0) {
		glEnable(GL_LIGHTING);
		glColor4d(0x6e / 255.0, 0xc5 / 255.0, 0xe9 / 255.0, alphaVal);
		for (uint i = 0; i<endEffectorTrajectories.size(); i++) {
			for (double j = 0.01; j <= 1; j += 0.01) {
				P3D p1 = endEffectorTrajectories[i].getEEPositionAt(j);
				P3D p2 = endEffectorTrajectories[i].getEEPositionAt(j-0.01);
				drawCapsule(p1, p2, 0.001);
			}
		}
		glEnable(GL_LIGHTING);
		for (uint i = 0; i<endEffectorTrajectories.size(); i++)
			drawSphere(endEffectorTrajectories[i].getEEPositionAt(f), 0.01);
		
		glEnd();
	}

	//draw the COP points/trajectory
	if (animationCycle >= 3 && alphaVal > 0) {
		glEnable(GL_LIGHTING);
		glColor4d(0.75 * 0xff / 255.0, 0.75 * 0xcf / 255.0, 0.75 * 0x6c / 255.0, alphaVal);
		for (int i = 1; i<nSamplePoints; i++) {
			P3D zmp1 = getCOP(i); zmp1[1] = 0.001;
			P3D zmp2 = getCOP(i-1); zmp2[1] = 0.001;

			drawCapsule(zmp1, zmp2, 0.001);
		}
		drawSphere(getCOP(tIndex), 0.01);
	}

	//draw the support polygon
	if (animationCycle >= 4 && alphaVal > 0) {
		//draw the support polygon...
		DynamicArray<int> stanceLimbs;
		DynamicArray<P3D> limbPositions;
		for (uint i = 0; i<endEffectorTrajectories.size(); i++) {
			if (endEffectorTrajectories[i].isInStance(f)) {
				stanceLimbs.push_back(i);
				limbPositions.push_back(endEffectorTrajectories[i].getEEPositionAt(f));
			}
		}
		DynamicArray<ConvexHull2D_Vertex> convexHullVertices;
		ConvexHull3D::planarConvexHullFromSetOfPoints(limbPositions, V3D(0, 1, 0), convexHullVertices);
		glColor4d(0xff / 255.0, 0x59 / 255.0, 0x59 / 255.0, alphaVal);
		glEnable(GL_LIGHTING);
		for (uint i = 0; i<convexHullVertices.size(); i++) {
			int limbIndex1 = stanceLimbs[convexHullVertices[i].index];
			P3D p1 = endEffectorTrajectories[limbIndex1].getEEPositionAt(f); p1[1] += 0.001;
			int limbIndex2 = stanceLimbs[convexHullVertices[(i+1)%(convexHullVertices.size())].index];
			P3D p2 = endEffectorTrajectories[limbIndex2].getEEPositionAt(f); p2[1] += 0.001;
			drawCapsule(p1, p2, 0.001);
		}
	}

	if (animationCycle >= 5) {
		ReducedRobotState oldState(robot);
		ReducedRobotState robotState(robot);
		robotStateTrajectory.getRobotPoseAt(f, robotState);
		V3D val = robotStateTrajectory.getBodyPositionAt(1) - robotStateTrajectory.getBodyPositionAt(0);
		robotState.setPosition(robotState.getPosition() + val * (animationCycle-5));
		robot->setState(&robotState);

		glEnable(GL_LIGHTING);

		//draw end effector positions...
		for (uint i = 0; i < endEffectorTrajectories.size(); i++) {
			if (endEffectorTrajectories[i].isInSwing(f))
				glColor3d(1, 0, 0);
			else
				glColor3d(0, 1, 0);
			drawSphere(endEffectorTrajectories[i].theLimb->getLastLimbSegment()->getWorldCoordinates(endEffectorTrajectories[i].theLimb->getLastLimbSegment()->rbProperties.getEndEffectorPoint(endEffectorTrajectories[i].CPIndex)), 0.02);
		}

		//draw the robot configuration...
		robot->getRoot()->draw(SHOW_ABSTRACT_VIEW);
		for (int i = 0; i<robot->getJointCount(); i++)
			robot->getJoint(i)->child->draw(SHOW_ABSTRACT_VIEW);

		glDisable(GL_LIGHTING);

		robot->setState(&oldState);
	}
}
