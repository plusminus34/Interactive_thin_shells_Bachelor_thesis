#pragma warning(disable : 4996)

#include <RobotDesignerLib/IntelligentRobotEditingWindow.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include <RBSimLib/ODERBEngine.h>

//maybe this editing window should just extend mopt, and then we create it directly (or not for "normal" execution...)


//move joints. When ALT is down (or the one that is same as in design window!) then move just joint and/or everything lower down in hierarchy
//resize bones. When ALT is down, move only child. Otherwise move both the child and parent joints...

IntelligentRobotEditingWindow::IntelligentRobotEditingWindow(int x, int y, int w, int h, RobotDesignerApp* rdApp) : GLWindow3D(x, y, w, h) {
	this->rdApp = rdApp;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;

	dynamic_cast<GLTrackingCamera*>(this->camera)->setCameraTarget(P3D(0, -0.25, 0));
}

IntelligentRobotEditingWindow::~IntelligentRobotEditingWindow(){

}

void IntelligentRobotEditingWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);
}

//triggered when using the mouse wheel
bool IntelligentRobotEditingWindow::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (highlightedRigidBody) {
		if (highlightedRigidBody->pJoints.size() == 1 && (highlightedRigidBody->cJoints.size() > 0 || highlightedRigidBody->rbProperties.endEffectorPoints.size() > 0)){
			ReducedRobotState rs(rdApp->robot);
			rdApp->robot->setState(&rdApp->prd->defaultRobotState);

			P3D p1 = highlightedRigidBody->pJoints[0]->getWorldPosition();
			P3D p2;

			if (highlightedRigidBody->cJoints.size() == 1){
				p2 = highlightedRigidBody->cJoints[0]->getWorldPosition();
			} else {
				for (auto& eeItr : highlightedRigidBody->rbProperties.endEffectorPoints){
					p2 += highlightedRigidBody->getWorldCoordinates(eeItr.coords);
				}
				p2 /= highlightedRigidBody->rbProperties.endEffectorPoints.size();
			}

			V3D vec = V3D(p1, p2) * ((100 + yOffset) / 100.0);
			P3D midPoint = (p1 + p2) / 2;
			dVector currentDesignParameters;
			rdApp->prd->getCurrentSetOfParameters(currentDesignParameters);

//			Logger::consolePrint("offset: %lf, vec: %lf %lf %lf\n", (100 + yOffset) / 100.0, vec[0], vec[1], vec[2]);

			V3D offset1(p1, midPoint + vec  * -0.5);
			V3D offset2(p2, midPoint + vec * 0.5);

			offset1 = highlightedRigidBody->getLocalCoordinates(offset1);
			offset2 = highlightedRigidBody->getLocalCoordinates(offset2);

			if (glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
				//scale the selected bone by moving just the child... and propagate all changes throughout the offspring hierarchy...
	
				//adapt the child coords of the parent joint
				int pStartIndex = rdApp->prd->jointParamMap[highlightedRigidBody->pJoints[0]].index;
				for (int i = 0; i < 3; i++)
					currentDesignParameters(pStartIndex + 3 + i) += offset1[i];

				//and now, the parent coords of the child joint
				if (highlightedRigidBody->cJoints.size() == 1) {
					int pStartIndex = rdApp->prd->jointParamMap[highlightedRigidBody->cJoints[0]].index;
					for (int i = 0; i < 3; i++)
						currentDesignParameters(pStartIndex + i) += offset2[i];
				}
				else {
					int pStartIndex = rdApp->prd->eeParamMap[highlightedRigidBody].index;
					for (int i = 0; i < 3; i++)
						currentDesignParameters(pStartIndex + i) += offset2[i];
				}
			}
			else {
				//rescaling the bone with as few global changes as possible

				int pStartIndex = rdApp->prd->jointParamMap[highlightedRigidBody->pJoints[0]].index;

				//for the parent joint, adjust its coordinates in the child (e.g. highlighted) rigid body frame
				for (int i = 0; i < 3;i++)
					currentDesignParameters(pStartIndex + 3 + i) += offset1[i];

				//and then, for the same joint, adjust its coordinates in the frame of the parent rigid body...
				V3D offset1P = highlightedRigidBody->pJoints[0]->parent->getLocalCoordinates(highlightedRigidBody->getWorldCoordinates(offset1));
				for (int i = 0; i < 3; i++)
					currentDesignParameters(pStartIndex + i) += offset1P[i];

				if (highlightedRigidBody->cJoints.size() == 1){

					//adjust the coordinates of the child joint, both in coord frame of its parent and child rigid bodies...
					int pStartIndex = rdApp->prd->jointParamMap[highlightedRigidBody->cJoints[0]].index;
					for (int i = 0; i < 3; i++)
						currentDesignParameters(pStartIndex + i) += offset2[i];

					V3D offset2P = highlightedRigidBody->cJoints[0]->child->getLocalCoordinates(highlightedRigidBody->getWorldCoordinates(offset2));
					for (int i = 0; i < 3; i++)
						currentDesignParameters(pStartIndex + 3 + i) += offset2P[i];
				}
				else {
					int pStartIndex = rdApp->prd->eeParamMap[highlightedRigidBody].index;
					for (int i = 0; i < 3; i++)
						currentDesignParameters(pStartIndex + i) += offset2[i];
				}
			}

			rdApp->robot->setState(&rs);
			updateParamsAndMotion(currentDesignParameters);
			syncSliders();
		}
		return true;
	}

	return GLWindow3D::onMouseWheelScrollEvent(xOffset, yOffset);
}

bool IntelligentRobotEditingWindow::onMouseMoveEvent(double xPos, double yPos){
	if (Robot* robot = rdApp->robot) {

		ReducedRobotState rs(robot);
		robot->setState(&rdApp->prd->defaultRobotState);

		preDraw();
		Ray ray = getRayFromScreenCoords(xPos, yPos);
		postDraw();

		for (int i = 0; i < robot->getRigidBodyCount(); i++)
			robot->getRigidBody(i)->selected = false;

		highlightedRigidBody = NULL;
		P3D pLocal;
		double tMin = DBL_MAX;
		for (int i = 0; i < robot->getRigidBodyCount(); i++){
			if (robot->getRigidBody(i)->getRayIntersectionPointTo(ray, &pLocal)) {
				double tVal = ray.getRayParameterFor(robot->getRigidBody(i)->getWorldCoordinates(pLocal));

				if (tVal < tMin) {
					tMin = tVal;
					highlightedRigidBody = robot->getRigidBody(i);
				}
			}
		}

		robot->setState(&rs);

		if (highlightedRigidBody) {
			highlightedRigidBody->selected = true;
//			Logger::consolePrint("highlighted rb %s, positioned at %lf %lf %lf\n", highlightedRigidBody->name.c_str(), highlightedRigidBody->getCMPosition().x(), highlightedRigidBody->getCMPosition().y(), highlightedRigidBody->getCMPosition().z());
		}
	}
	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool IntelligentRobotEditingWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	int shiftDown = glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_SHIFT);
	if (GlobalMouseState::lButtonPressed && shiftDown == GLFW_PRESS) {
		return true;
	}

	return GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos);
}

void IntelligentRobotEditingWindow::setViewportParameters(int posX, int posY, int sizeX, int sizeY){
	GLWindow3D::setViewportParameters(posX, posY, sizeX, sizeY);
}

void IntelligentRobotEditingWindow::resetParams()
{
	dVector p;	rdApp->prd->getCurrentSetOfParameters(p);
	p.setZero();
	rdApp->prd->setParameters(p);
	syncSliders();
}

void IntelligentRobotEditingWindow::compute_dmdp_Jacobian()
{
	//evaluates dm/dp at (m,p). It is assumed that m corresponds to a minimum of the energy (i.e. m = arg min (E(m(p)))
	// Let g = \partial E / \partial m
	// partial g / partial p + partial g / partial m * dm/dp = dg/dp = 0
	// dm/dp = -partial g / partial m ^ -1 * partial g / partial p

#ifdef USE_MATLAB
	igl::matlab::mlinit(&matlabengine);
	igl::matlab::mleval(&matlabengine, "desktop");
#endif

	dVector m; rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
	m0 = m;

	DynamicArray<double> p;	rdApp->prd->getCurrentSetOfParameters(p);
	p0 = Eigen::Map<dVector>(p.data(), p.size());

	SparseMatrix dgdm;
	dVector dgdpi, dmdpi;
	MatrixNxM dgdp(m.size(), p.size());
	dVector g_m, g_p;

	resize(dgdm, m.size(), m.size());
	resize(dmdp, m.size(), p.size());
	resize(dgdpi, m.size());

	Timer timerN; timerN.restart();
	DynamicArray<MTriplet> triplets;
	rdApp->moptWindow->locomotionManager->energyFunction->addHessianEntriesTo(triplets, m);
	dgdm.setFromTriplets(triplets.begin(), triplets.end());
	
	Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
	//	Eigen::SparseLU<SparseMatrix> solver;
	solver.compute(dgdm);
	Logger::consolePrint("Time to construct dgdm: %lf\n", timerN.timeEllapsed());
	double dp = 0.001;
	//now, for every design parameter, estimate change in gradient, and use that to compute the corresponding entry in dm/dp...
	timerN.restart();
	for (uint i = 0; i < p.size(); i++) {
		resize(g_m, m.size());
		resize(g_p, m.size());

		double pVal = p[i];
		p[i] = pVal + dp;
		rdApp->prd->setParameters(p);
		rdApp->moptWindow->locomotionManager->energyFunction->addGradientTo(g_p, m);
		p[i] = pVal - dp;
		rdApp->prd->setParameters(p);
		rdApp->moptWindow->locomotionManager->energyFunction->addGradientTo(g_m, m);
		p[i] = pVal;
		rdApp->prd->setParameters(p);

		dgdp.col(i) = (g_p - g_m) / (2 * dp);
	}
	Logger::consolePrint("Time to construct dgdp: %lf\n", timerN.timeEllapsed());
	timerN.restart();
	dmdp = solver.solve(dgdp) * -1;
	Logger::consolePrint("Time to solve for J: %lf\n", timerN.timeEllapsed());

	if (useSVD)
	{
		Eigen::JacobiSVD<MatrixNxM> svd(dmdp, Eigen::ComputeThinU | Eigen::ComputeThinV);
		dmdp_V = svd.matrixV();
	}
#ifdef USE_MATLAB
	RUN_IN_MATLAB(
		igl::matlab::mlsetmatrix(&matlabengine, "dmdp", dmdp);
	igl::matlab::mlsetmatrix(&matlabengine, "dmdp_V", dmdp_V);
	)
#endif
}

void IntelligentRobotEditingWindow::test_dmdp_Jacobian() {
	dVector m; rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
	DynamicArray<double> p;	rdApp->prd->getCurrentSetOfParameters(p);

	//dm/dp, the analytic version. 
	compute_dmdp_Jacobian();

	//Now estimate this jacobian with finite differences...
	MatrixNxM dmdp_FD;
	dVector m_initial, m_m, m_p;
	resize(dmdp_FD, m.size(), p.size());
	double dp = 0.001;
	rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m_initial);
	//now, for every design parameter, estimate change in gradient, and use that to compute the corresponding entry in dm/dp...
	for (uint i = 0; i < p.size(); i++) {
		resize(m_m, m.size());
		resize(m_p, m.size());

		double pVal = p[i];
		p[i] = pVal + dp;
		rdApp->prd->setParameters(p);
		//now we must solve this thing a loooot...

		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		for (int j = 0; j < 300; j++)
			rdApp->moptWindow->runMOPTStep();

		rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m_m);
		p[i] = pVal - dp;
		rdApp->prd->setParameters(p);
		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		for (int j = 0; j < 300; j++)
			rdApp->moptWindow->runMOPTStep();
		rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m_p);

		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		p[i] = pVal;
		rdApp->prd->setParameters(p);

		dmdp_FD.col(i) = (m_p - m_m) / (2 * dp);
	}

	print("../out/dmdp.m", dmdp);
	print("../out/dmdp_FD.m", dmdp_FD);
}

void IntelligentRobotEditingWindow::testOptimizeDesign() {
	dVector m; rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
	DynamicArray<double> p;	rdApp->prd->getCurrentSetOfParameters(p);
	MatrixNxM dmdp; compute_dmdp_Jacobian();

	//If we have some objective O, expressed as a function of m, then dO/dp = dO/dm * dm/dp
	dVector dOdm;
	dVector dOdp;
	resize(dOdm, m.size());
	resize(dOdp, p.size());

	rdApp->moptWindow->locomotionManager->energyFunction->objectives[11]->addGradientTo(dOdm, m);

	dOdp = dmdp.transpose() * dOdm;

	Logger::consolePrint("dOdp[0]: %lf\n", dOdp[0]);

	double len = dOdp.norm();
	if (len > 0.01)
		dOdp = dOdp / len * 0.01;

	Logger::consolePrint("p[0] before: %lf\n", p[0]);

	for (uint i = 0; i < p.size(); i++)
		p[i] -= dOdp[i];
	Logger::consolePrint("p[0] after: %lf\n", p[0]);
	rdApp->prd->setParameters(p);
}

void IntelligentRobotEditingWindow::showMenu()
{
	if (!menu)
		CreateParametersDesignWindow();
	menu->setVisible(true);
}

void IntelligentRobotEditingWindow::hideMenu()
{
	if (menu)
		menu->setVisible(false);
}

void IntelligentRobotEditingWindow::syncSliders()
{
	DynamicArray<double> p;
	rdApp->prd->getCurrentSetOfParameters(p);
	auto removeTrailingZeros = [](string &&s) {return s.erase(s.find_last_not_of('0') + 1, string::npos); };
	for (int i = 0 ; i < sliders.size(); i++)
	{
		sliders[i]->setValue(p[i]);
		textboxes[i]->setValue(removeTrailingZeros(to_string(p[i])));
	}
}
void IntelligentRobotEditingWindow::CreateParametersDesignWindow()
{
	if (menu)
		menu->dispose();

	menu = rdApp->mainMenu->addWindow(Eigen::Vector2i(viewportX , viewportY), "Design Parameters");
	rdApp->mainMenu->addButton("Reset Params", [this]() { resetParams(); });
	rdApp->mainMenu->addButton("Compute Jacobian", [this]() { compute_dmdp_Jacobian(); });

	rdApp->mainMenu->addVariable("Update Jacobian continuously", updateJacobiancontinuously);
	rdApp->mainMenu->addVariable("Use Jacobian", updateMotionBasedOnJacobian);
	rdApp->mainMenu->addVariable("Use SVD", useSVD);

	using namespace nanogui;
	DynamicArray<double> p;	rdApp->prd->getCurrentSetOfParameters(p);

	Widget *panel = new Widget(rdApp->mainMenu->window());
	GridLayout *layout =
		new GridLayout(Orientation::Horizontal, 2,
			Alignment::Middle, 15, 5);
	layout->setColAlignment(
	{ Alignment::Maximum, Alignment::Fill });
	layout->setSpacing(0, 10);
	panel->setLayout(layout);


	auto removeTrailingZeros = [](string &&s) {return s.erase(s.find_last_not_of('0') + 1, string::npos); };
	sliders.resize(rdApp->prd->getNumberOfParameters());
	textboxes.resize(rdApp->prd->getNumberOfParameters());
	for (int i = 0; i < rdApp->prd->getNumberOfParameters(); i++)
	{
		Slider *slider = new Slider(panel);
		slider->setValue((float)p[i]);
		slider->setRange({ -0.1,0.1 });
		slider->setFixedWidth(150);
		TextBox *textBox = new TextBox(panel);
		textBox->setValue(removeTrailingZeros(to_string(p[i])));
		textBox->setFixedWidth(50);
		textBox->setEditable(true);
		textBox->setFixedHeight(18);

		slider->setCallback([&, i, textBox](float value) {
			updateParamsUsingSliders(i, value);
			textBox->setValue(removeTrailingZeros(to_string(value)));
		});
		textBox->setCallback([&, i, slider](const std::string &str) {
			updateParamsUsingSliders(i, std::stod(str));
			slider->setValue((float)std::stod(str));
			return true;
		});
		sliders[i] = slider;
		textboxes[i] = textBox;
	}
	rdApp->mainMenu->addWidget("", panel);
	rdApp->menuScreen->performLayout();
	slidervalues.resize(rdApp->prd->getNumberOfParameters());
	slidervalues.setZero();
}
void IntelligentRobotEditingWindow::updateParamsUsingSliders(int paramIndex, double value)
{
	slidervalues(paramIndex) = value;
	dVector p;
	if (!useSVD) {
		rdApp->prd->getCurrentSetOfParameters(p);
		p(paramIndex) = value;
	}
	else {
		p = p0 + dmdp_V*slidervalues;
	}
	updateParamsAndMotion(p);
}
void IntelligentRobotEditingWindow::updateParamsAndMotion(dVector p)
{
	if (updateJacobiancontinuously)
	{
		Timer timerN; timerN.restart();
		compute_dmdp_Jacobian();
		Logger::consolePrint("Total time to compute J: %lf\n", timerN.timeEllapsed());
	}
	rdApp->prd->setParameters(p);
	if (updateMotionBasedOnJacobian) {
		dVector m; rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
		m = m0 + dmdp*(p-p0);
		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m);
	}
}

void IntelligentRobotEditingWindow::setupLights() {
	GLfloat bright[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLfloat mediumbright[] = { 0.3f, 0.3f, 0.3f, 1.0f };

	glLightfv(GL_LIGHT1, GL_DIFFUSE, bright);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, mediumbright);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, mediumbright);
	glLightfv(GL_LIGHT4, GL_DIFFUSE, mediumbright);


	GLfloat light0_position[] = { 0.0f, 10000.0f, 10000.0f, 0.0f };
	GLfloat light0_direction[] = { 0.0f, -10000.0f, -10000.0f, 0.0f };

	GLfloat light1_position[] = { 0.0f, 10000.0f, -10000.0f, 0.0f };
	GLfloat light1_direction[] = { 0.0f, -10000.0f, 10000.0f, 0.0f };

	GLfloat light2_position[] = { 0.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light2_direction[] = { 0.0f, 10000.0f, -0.0f, 0.0f };

	GLfloat light3_position[] = { 10000.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light3_direction[] = { -10000.0f, 10000.0f, -0.0f, 0.0f };

	GLfloat light4_position[] = { -10000.0f, -10000.0f, 0.0f, 0.0f };
	GLfloat light4_direction[] = { 10000.0f, 10000.0f, -0.0f, 0.0f };


	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
	glLightfv(GL_LIGHT1, GL_POSITION, light1_position);
	glLightfv(GL_LIGHT2, GL_POSITION, light2_position);
	glLightfv(GL_LIGHT3, GL_POSITION, light3_position);
	glLightfv(GL_LIGHT4, GL_POSITION, light4_position);


	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, light0_direction);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, light1_direction);
	glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, light2_direction);
	glLightfv(GL_LIGHT3, GL_SPOT_DIRECTION, light3_direction);
	glLightfv(GL_LIGHT4, GL_SPOT_DIRECTION, light4_direction);


	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
}

void IntelligentRobotEditingWindow::drawScene() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);
	drawDesignEnvironmentBox(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
	glEnable(GL_LIGHTING);

	if (!rdApp->robot)
		return;

	int flags = SHOW_ABSTRACT_VIEW | SHOW_BODY_FRAME | SHOW_JOINTS | HIGHLIGHT_SELECTED;

	ReducedRobotState rs(rdApp->robot);
	rdApp->robot->setState(&rdApp->prd->defaultRobotState);

	glEnable(GL_LIGHTING);
	if (rdApp->simWindow->rbEngine)
		rdApp->simWindow->rbEngine->drawRBs(flags);

	//	for (int i = 0; i < rdApp->robot->getJointCount(); i++)
	//		drawSphere(rdApp->prd->initialJointMorphology[rdApp->robot->getJoint(i)].worldCoords, 0.015);

	if (highlightedRigidBody && highlightedRigidBody->pJoints.size() == 1 && (highlightedRigidBody->cJoints.size() > 0 || highlightedRigidBody->rbProperties.endEffectorPoints.size() > 0)) {
		P3D p1 = highlightedRigidBody->pJoints[0]->getWorldPosition();
		P3D p2;
		if (highlightedRigidBody->cJoints.size() == 1)
			p2 = highlightedRigidBody->cJoints[0]->getWorldPosition();
		else {
			for (auto& eeItr : highlightedRigidBody->rbProperties.endEffectorPoints) {
				p2 += highlightedRigidBody->getWorldCoordinates(eeItr.coords);
			}
			p2 /= highlightedRigidBody->rbProperties.endEffectorPoints.size();
		}

		drawSphere(p1, 0.015);
		drawSphere(p2, 0.015);

	}


	rdApp->robot->setState(&rs);
}
