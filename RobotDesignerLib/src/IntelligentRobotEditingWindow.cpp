#pragma warning(disable : 4996)

#include <RobotDesignerLib/IntelligentRobotEditingWindow.h>
#include <RobotDesignerLib/LocomotionEngineManagerGRF.h>
#include <RobotDesignerLib/LocomotionEngineManagerIP.h>
#include <RBSimLib/ODERBEngine.h>

//maybe this editing window should just extend mopt, and then we create it directly (or not for "normal" execution...)

//TODO: add also parameterization for rotation axis of joints/wheels?!?


//clicking on joints/end effectors adds a translate widget. Right click to make it go away
//move joints. When ALT is down (or the one that is same as in design window!) then move just joint and/or everything lower down in hierarchy
//resize bones via mouse scroll. When ALT is down, move only child. Otherwise move both the child and parent joints...

IntelligentRobotEditingWindow::IntelligentRobotEditingWindow(int x, int y, int w, int h, RobotDesignerApp* rdApp) : GLWindow3D(x, y, w, h) {
	this->rdApp = rdApp;

	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutRightAxis = 0.25;
	dynamic_cast<GLTrackingCamera*>(this->camera)->rotAboutUpAxis = 0.95;
	dynamic_cast<GLTrackingCamera*>(this->camera)->camDistance = -1.5;

	dynamic_cast<GLTrackingCamera*>(this->camera)->setCameraTarget(P3D(0, -0.25, 0));

	tWidget = new TranslateWidget(AXIS_X | AXIS_Y | AXIS_Z);
	tWidget->visible = false;
	lbfgsMinimizer = make_unique<BFGSHessianApproximator>(20);
}

void IntelligentRobotEditingWindow::addMenuItems() {

}

IntelligentRobotEditingWindow::~IntelligentRobotEditingWindow(){
	delete tWidget;
}

void IntelligentRobotEditingWindow::drawAuxiliarySceneInfo(){
	glClear(GL_DEPTH_BUFFER_BIT);

	preDraw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);
}

bool isEERB(RigidBody* rb) {
	return rb->rbProperties.endEffectorPoints.size() > 0 || rb->cJoints.size() == 0;
}

//triggered when using the mouse wheel
bool IntelligentRobotEditingWindow::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (highlightedRigidBody) {
		if (highlightedRigidBody->pJoints.size() == 1 && (highlightedRigidBody->cJoints.size() > 0 || highlightedRigidBody->rbProperties.endEffectorPoints.size() > 0)){
			RobotState rs(rdApp->robot);
			rdApp->robot->setState(&rdApp->prd->defaultRobotState);

			P3D p1 = highlightedRigidBody->pJoints[0]->getWorldPosition();
			P3D p2;

			if (!isEERB(highlightedRigidBody)){
				p2 = highlightedRigidBody->cJoints[0]->getWorldPosition();
			} else {
				for (auto& eeItr : highlightedRigidBody->rbProperties.endEffectorPoints){
					p2 += highlightedRigidBody->getWorldCoordinates(eeItr.coords);
				}
				p2 /= highlightedRigidBody->rbProperties.endEffectorPoints.size();
			}

			V3D vec = V3D(p1, p2) * ((100 + yOffset) / 100.0);
			P3D midPoint = (p1 + p2) / 2;
			DynamicArray<double> currentDesignParameters;
			rdApp->prd->getCurrentSetOfParameters(currentDesignParameters);

//			Logger::consolePrint("offset: %lf, vec: %lf %lf %lf\n", (100 + yOffset) / 100.0, vec[0], vec[1], vec[2]);

			V3D offset1(p1, midPoint + vec  * -0.5);
			V3D offset2(p2, midPoint + vec * 0.5);

			offset1 = highlightedRigidBody->getLocalCoordinates(offset1);
			offset2 = highlightedRigidBody->getLocalCoordinates(offset2);

			if (glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
				//scale the selected bone by moving just the child... and propagate all changes throughout the offspring hierarchy...
	
				//adapt the child coords of the parent joint
				int pStartIndex =  rdApp->prd->jointParamMap[highlightedRigidBody->pJoints[0]].index;
				V3D modifier = V3D(rdApp->prd->jointParamMap[highlightedRigidBody->pJoints[0]].xModifier, 1, 1);

				for (int i = 0; i < 3; i++)
					currentDesignParameters[pStartIndex + 3 + i] += offset1[i] * modifier[i];

				//and now, the parent coords of the child joint
				if (!isEERB(highlightedRigidBody)) {
					int pStartIndex =  rdApp->prd->jointParamMap[highlightedRigidBody->cJoints[0]].index;
					V3D modifier = V3D(rdApp->prd->jointParamMap[highlightedRigidBody->cJoints[0]].xModifier, 1, 1);

					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + i] += offset2[i] * modifier[i];
				}
				else {
					for (uint i=0;i<highlightedRigidBody->rbProperties.endEffectorPoints.size();i++){
						int pStartIndex =  rdApp->prd->eeParamMap[&highlightedRigidBody->rbProperties.endEffectorPoints[i]].index;
						V3D modifier = V3D(rdApp->prd->eeParamMap[&highlightedRigidBody->rbProperties.endEffectorPoints[i]].xModifier, 1, 1);

						for (int j = 0; j < 3; j++)
							currentDesignParameters[pStartIndex + j] += offset2[j] * modifier[j];
					}
				}
			}
			else {
				//rescaling the bone with as few global changes as possible

				int pStartIndex =  rdApp->prd->jointParamMap[highlightedRigidBody->pJoints[0]].index;
				V3D modifier = V3D(rdApp->prd->jointParamMap[highlightedRigidBody->pJoints[0]].xModifier, 1, 1);

				//for the parent joint, adjust its coordinates in the child (e.g. highlighted) rigid body frame
				for (int i = 0; i < 3;i++)
					currentDesignParameters[pStartIndex + 3 + i] += offset1[i] * modifier[i];

				//and then, for the same joint, adjust its coordinates in the frame of the parent rigid body...
				V3D offset1P = highlightedRigidBody->pJoints[0]->parent->getLocalCoordinates(highlightedRigidBody->getWorldCoordinates(offset1));
				for (int i = 0; i < 3; i++)
					currentDesignParameters[pStartIndex + i] += offset1P[i] * modifier[i];

				if (!isEERB(highlightedRigidBody)){

					//adjust the coordinates of the child joint, both in coord frame of its parent and child rigid bodies...
					int pStartIndex =  rdApp->prd->jointParamMap[highlightedRigidBody->cJoints[0]].index;
					V3D modifier = V3D(rdApp->prd->jointParamMap[highlightedRigidBody->cJoints[0]].xModifier, 1, 1);

					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + i] += offset2[i] * modifier[i];

					V3D offset2P = highlightedRigidBody->cJoints[0]->child->getLocalCoordinates(highlightedRigidBody->getWorldCoordinates(offset2));
					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + 3 + i] += offset2P[i] * modifier[i];
				}
				else {
					for (uint i = 0; i<highlightedRigidBody->rbProperties.endEffectorPoints.size(); i++) {
						int pStartIndex =  rdApp->prd->eeParamMap[&highlightedRigidBody->rbProperties.endEffectorPoints[i]].index;
						V3D modifier = V3D(rdApp->prd->eeParamMap[&highlightedRigidBody->rbProperties.endEffectorPoints[i]].xModifier, 1, 1);
						for (int j = 0; j < 3; j++)
							currentDesignParameters[pStartIndex + j] += offset2[j] * modifier[j];
					}
				}
			}

			rdApp->robot->setState(&rs);
			updateParamsAndMotion(Eigen::Map<dVector>(currentDesignParameters.data(), currentDesignParameters.size()));
			syncSliders();
		}

		return true;
	}

	return GLWindow3D::onMouseWheelScrollEvent(xOffset, yOffset);
}

bool IntelligentRobotEditingWindow::onMouseMoveEvent(double xPos, double yPos){
	if (Robot* robot = rdApp->robot) {
		preDraw();
		Ray ray = getRayFromScreenCoords(xPos, yPos);
		postDraw();

		if (tWidget->visible) {
			preDraw();
			bool clickProcessed = false;
			if ((clickProcessed = tWidget->onMouseMoveEvent(xPos, yPos)) == true) {
				RobotState rs(robot);
				robot->setState(&rdApp->prd->defaultRobotState);

				P3D pOriginal;
				int pStartIndex = -1;
				V3D modifier = V3D(1, 1, 1);

				if (highlightedEE){
					pOriginal = highlightedEEParent->getWorldCoordinates(highlightedEE->coords);
					pStartIndex =  rdApp->prd->eeParamMap[highlightedEE].index;
					modifier.x() = rdApp->prd->eeParamMap[highlightedEE].xModifier;
				}
				else {
					pOriginal = highlightedJoint->getWorldPosition();
					pStartIndex =  rdApp->prd->jointParamMap[highlightedJoint].index;
					modifier.x() = rdApp->prd->jointParamMap[highlightedJoint].xModifier;
				}

				V3D offset(pOriginal, tWidget->pos);

				DynamicArray<double> currentDesignParameters;
				rdApp->prd->getCurrentSetOfParameters(currentDesignParameters);


				if (highlightedEE) {
					offset = highlightedEEParent->getLocalCoordinates(offset);
					for (int i = 0; i < 3; i++)
						currentDesignParameters[pStartIndex + i] += offset[i] * modifier[i];
				}
				else {
					if (glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_ALT) == GLFW_PRESS) {
						offset = highlightedJoint->parent->getLocalCoordinates(offset);
						for (int i = 0; i < 3; i++)
							currentDesignParameters[pStartIndex + i] += offset[i] * modifier[i];
					}
					else {
						V3D offsetP = highlightedJoint->parent->getLocalCoordinates(offset);
						for (int i = 0; i < 3; i++)
							currentDesignParameters[pStartIndex + i] += offsetP[i] * modifier[i];
						V3D offsetC = highlightedJoint->child->getLocalCoordinates(offset);
						for (int i = 0; i < 3; i++)
							currentDesignParameters[pStartIndex + 3 + i] += offsetC[i] * modifier[i];
					}

				}

				robot->setState(&rs);
				updateParamsAndMotion(Eigen::Map<dVector>(currentDesignParameters.data(), currentDesignParameters.size()));
				syncSliders();
			}
			postDraw();
			if (clickProcessed)
				return true;
		}
		else {
			RobotState rs(robot);
			robot->setState(&rdApp->prd->defaultRobotState);

			//check if any of the joints are highlighted...
			for (int i = 0; i < robot->getJointCount(); i++)
				robot->getJoint(i)->selected = false;

			highlightedJoint = NULL;
			double tMinJ = DBL_MAX;

			for (int i = 0; i < robot->getJointCount(); i++) {
				P3D p;
				double dist = ray.getDistanceToPoint(robot->getJoint(i)->getWorldPosition(), &p);
				double tVal = ray.getRayParameterFor(p);
				if (dist < robot->getJoint(i)->parent->abstractViewCylinderRadius * 1.2 && tVal < tMinJ) {
					tMinJ = tVal;
					highlightedJoint = robot->getJoint(i);
				}
			}

			for (int i = 0; i < robot->getRigidBodyCount(); i++)
				for (uint j = 0; j < robot->getRigidBody(i)->rbProperties.endEffectorPoints.size(); j++)
					robot->getRigidBody(i)->rbProperties.endEffectorPoints[j].selected = false;

			highlightedEE = NULL;
			highlightedEEParent = NULL;
			double tMinEE = DBL_MAX;

			for (int i = 0; i < robot->getRigidBodyCount(); i++)
				for (uint j = 0; j < robot->getRigidBody(i)->rbProperties.endEffectorPoints.size(); j++) {
					P3D p;
					double dist = ray.getDistanceToPoint(robot->getRigidBody(i)->getWorldCoordinates(robot->getRigidBody(i)->rbProperties.endEffectorPoints[j].coords), &p);
					double tVal = ray.getRayParameterFor(p);
					if (dist < robot->getRigidBody(i)->abstractViewCylinderRadius * 1.2 && tVal < tMinEE) {
						tMinEE = tVal;
						highlightedEE = &robot->getRigidBody(i)->rbProperties.endEffectorPoints[j];
						highlightedEEParent = robot->getRigidBody(i);
					}
				}



			for (int i = 0; i < robot->getRigidBodyCount(); i++)
				robot->getRigidBody(i)->selected = false;

			highlightedRigidBody = NULL;

			P3D pLocal;
			double tMinB = DBL_MAX;
			for (int i = 0; i < robot->getRigidBodyCount(); i++) {
				if (robot->getRigidBody(i)->getRayIntersectionPointTo(ray, &pLocal)) {
					double tVal = ray.getRayParameterFor(robot->getRigidBody(i)->getWorldCoordinates(pLocal));

					if (tVal < tMinB) {
						tMinB = tVal;
						highlightedRigidBody = robot->getRigidBody(i);
					}
				}
			}

			if (highlightedJoint && highlightedRigidBody)
				if (highlightedJoint->parent == highlightedRigidBody || highlightedJoint->child == highlightedRigidBody) {
					highlightedRigidBody = NULL;
					tMinB = DBL_MAX;
				}

			if (highlightedEE && highlightedRigidBody)
				if (highlightedEEParent == highlightedRigidBody) {
					highlightedRigidBody = NULL;
					tMinB = DBL_MAX;
				}

			if (highlightedJoint && (tMinJ > tMinB || tMinJ > tMinEE))
				highlightedJoint = NULL;

			if (highlightedEE && (tMinEE > tMinB || tMinEE > tMinJ))
				highlightedEE = NULL;

			if (highlightedRigidBody && (tMinB > tMinEE || tMinB > tMinJ))
				highlightedRigidBody = NULL;

			if (highlightedJoint)
				highlightedJoint->selected = true;

			if (highlightedEE)
				highlightedEE->selected = true;

			if (highlightedRigidBody)
				highlightedRigidBody->selected = true;

			robot->setState(&rs);

		}

	}


	return GLWindow3D::onMouseMoveEvent(xPos, yPos);
}

bool IntelligentRobotEditingWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos){
	int ctrlDown = glfwGetKey(this->rdApp->glfwWindow, GLFW_KEY_LEFT_CONTROL);
	if (((GlobalMouseState::lButtonPressed && ctrlDown == GLFW_PRESS) || (GlobalMouseState::rButtonPressed)) && tWidget->visible) {
		tWidget->visible = false;
		highlightedEE = NULL;
		highlightedJoint = NULL;
		highlightedRigidBody = NULL;
		return true;
	}

	if (GlobalMouseState::lButtonPressed) {
		tWidget->visible = false;

		RobotState rs(rdApp->robot);
		rdApp->robot->setState(&rdApp->prd->defaultRobotState);


		//if a joint or EE is selected, turn tWidget on and sync it ...
		if (highlightedEE) {
			tWidget->visible = true;
			tWidget->pos = highlightedEEParent->getWorldCoordinates(highlightedEE->coords);
			if (highlightedEE->wheelJoint)
			Logger::consolePrint("Clicked on wheel \'%s\' with ID %d\n", highlightedEE->wheelJoint->name.c_str(), highlightedEE->wheelJoint->jIndex);
		}

		if (highlightedJoint) {
			tWidget->visible = true;
			tWidget->pos = highlightedJoint->getWorldPosition();
			Logger::consolePrint("Clicked on joint \'%s\' with ID %d\n", highlightedJoint->name.c_str(), highlightedJoint->jIndex);
		}
		rdApp->robot->setState(&rs);
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
	setParamsAndUpdateMOPT(p);
	syncSliders();
}

void IntelligentRobotEditingWindow::update_dmdX()
{
	//evaluates dm/dp at (m,p). It is assumed that m corresponds to a minimum of the energy (i.e. m = arg min (E(m(p)))
	// Let g = \partial E / \partial m
	// partial g / partial p + partial g / partial m * dm/dp = dg/dp = 0
	// dm/dp = -partial g / partial m ^ -1 * partial g / partial p
	
	// disable Hessian hacking first, to get the true hessian
	
	LocomotionEngine_EnergyFunction *totalEnergy = rdApp->moptWindow->locomotionManager->energyFunction;
	int nE = totalEnergy->objectives.size();
	std::vector<bool> hackHessian(nE);
	for (int i = 0; i < nE; i++)
	{
		hackHessian[i] = totalEnergy->objectives[i]->hackHessian;
		totalEnergy->objectives[i]->hackHessian = false;
	}

	dVector m; rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
	m0 = m;
	SparseMatrix dgdm;
	resize(dgdm, m.size(), m.size());
	DynamicArray<MTriplet> triplets;
	totalEnergy->addHessianEntriesTo(triplets, m);
	dgdm.setFromTriplets(triplets.begin(), triplets.end());

	Eigen::SimplicialLDLT<SparseMatrix, Eigen::Lower> solver;
	//	Eigen::SparseLU<SparseMatrix> solver;
	solver.compute(dgdm);

	// The next part depends on mode
	if (mode == Mode::design)
	{
		DynamicArray<double> p;	rdApp->prd->getCurrentSetOfParameters(p);
		p0 = Eigen::Map<dVector>(p.data(), p.size());

		dgdX.resize(m.size(), p.size());
		dVector g_m, g_p;

		resize(dmdX, m.size(), p.size());

		double dp = 0.001;
		//now, for every design parameter, estimate change in gradient, and use that to compute the corresponding entry in dm/dp...
		if (compute_dgdp_With_FD)
		{
			for (uint i = 0; i < p.size(); i++) {
				resize(g_m, m.size());
				resize(g_p, m.size());

				double pVal = p[i];
				p[i] = pVal + dp;
				setParamsAndUpdateMOPT(p);
				totalEnergy->addGradientTo(g_p, m);
				p[i] = pVal - dp;
				setParamsAndUpdateMOPT(p);
				totalEnergy->addGradientTo(g_m, m);
				p[i] = pVal;
				setParamsAndUpdateMOPT(p);

				dgdX.col(i) = (g_p - g_m) / (2 * dp);
			}
		}
		else
		{

		}
	}
	else
	{
		dgdX.resize(m.size(), nE);
		dVector grad(m.size());
		w0.resize(nE);
		for (int i = 0; i < nE; i++)
		{
			w0(i) = totalEnergy->objectives[i]->weight;
			totalEnergy->objectives[i]->weight = 1;
			dVector temp(m.size());
			temp.setZero();
			totalEnergy->objectives[i]->addGradientTo(grad, m0);
			dgdX.col(i) = grad;
			totalEnergy->objectives[i]->weight = w0(i);
		}
	}

	dmdX = solver.solve(dgdX) * -1;

	if (useSVD)
	{
		Eigen::JacobiSVD<MatrixNxM> svd(dmdX, Eigen::ComputeThinU | Eigen::ComputeThinV);
		dmdX_V = svd.matrixV();
	}

	for (int i = 0; i < nE; i++)
		totalEnergy->objectives[i]->hackHessian = hackHessian[i];
}


void IntelligentRobotEditingWindow::DoDesignParametersOptimizationStep(ObjectiveFunction* objFunction) {
	update_dmdX();
	LocomotionEngine_EnergyFunction *totalEnergy = rdApp->moptWindow->locomotionManager->energyFunction;


	//If we have some objective O, expressed as a function of m, then dO/dp = dO/dm * dm/dp
	dVector dOdm;
	dVector dOdX;
	dVector X, m;
	resize(dOdm, m0.size());
	objFunction->addGradientTo(dOdm, m0);

	
	double currStepSize;

	//resize(dOdX, p0.size());
	dOdX = dOdm.transpose() * dmdX;
	if (useLBFGS)
	{
		lbfgsMinimizer->add_x_and_dfdx_to_history(p0, dOdX);
		dOdX = lbfgsMinimizer->compute_Hinv_v(dOdX);
		currStepSize = 1;
	}
	else
	{
		double maxCoeff = dOdX.maxCoeff();
		if(maxCoeff>11)
			dOdX /= maxCoeff;
		currStepSize = stepSize;
	}
	dVector dmdXdOdx = dmdX * dOdX;
	double O0 = objFunction->computeValue(m0);
	double E0 = totalEnergy->computeValue(m0);
	double Ocurr, Ecurr;
	int i;
	for(i=0; i<15; i++)
	{
		if (mode == Mode::design)
		{
			X = p0 - currStepSize * dOdX;
			setParamsAndUpdateMOPT(X);
		}
		else
		{
			X = w0 - currStepSize * dOdX;
			X = X.unaryExpr([](double val) {return (val > 0) ? val : 0; });
			totalEnergy->setWeights(X);
		}
		m = m0 - currStepSize * dmdXdOdx;
		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m);
		Ocurr = objFunction->computeValue(m);
		Ecurr = totalEnergy->computeValue(m);
		if (Ocurr < O0)
			break;
		currStepSize /= 2;
	}

	Logger::consolePrint("Param optimization line search number of steps: %d", i);
}

void IntelligentRobotEditingWindow::showMenu(){
	if (!rdApp->robot)
		return;
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
	for (uint i = 0; i < sliders.size(); i++)
	{
		sliders[i]->setValue((float)p[i]);
		textboxes[i]->setValue(removeTrailingZeros(to_string(p[i])));
	}
}
void IntelligentRobotEditingWindow::CreateParametersDesignWindow()
{
	if (menu)
		menu->dispose();

	menu = rdApp->mainMenu->addWindow(Eigen::Vector2i(viewportX, viewportY), "Design Parameters");
	rdApp->mainMenu->addVariable("mode", mode)->setItems({ "design","weights" });
	rdApp->mainMenu->addButton("Reset Params", [this]() { resetParams(); });
	rdApp->mainMenu->addButton("Compute dmdp", [this]() { update_dmdX(); });

	rdApp->mainMenu->addVariable("Update dmdp continuously", updateJacobiancontinuously);
	rdApp->mainMenu->addVariable("Use dmdp", updateMotionBasedOnJacobian);
	rdApp->mainMenu->addVariable("Use SVD", useSVD);

	rdApp->mainMenu->addVariable("Use lbfgs", useLBFGS);
	rdApp->mainMenu->addVariable("Step Size", stepSize);

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
		p = p0 + dmdX_V*slidervalues;
	}
	updateParamsAndMotion(p);
}
void IntelligentRobotEditingWindow::setParamsAndUpdateMOPT(const dVector& p) {
	rdApp->prd->setParameters(p);
	rdApp->moptWindow->locomotionManager->motionPlan->updateEEs();
}
void IntelligentRobotEditingWindow::setParamsAndUpdateMOPT(const std::vector<double>& p) {
	rdApp->prd->setParameters(p);
	rdApp->moptWindow->locomotionManager->motionPlan->updateEEs();
}
void IntelligentRobotEditingWindow::updateParamsAndMotion(dVector p)
{
	if (updateJacobiancontinuously)
	{
		Timer timerN; timerN.restart();
		update_dmdX();
		Logger::consolePrint("Total time to compute J: %lf\n", timerN.timeEllapsed());
	}
	setParamsAndUpdateMOPT(p);
	if (updateMotionBasedOnJacobian) {
		dVector m; rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
		m = m0 + dmdX*(p - p0);
		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m);
	}
}
void IntelligentRobotEditingWindow::drawScene() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);
	drawDesignEnvironmentBox(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
	glEnable(GL_LIGHTING);

	if (!rdApp->robot)
		return;

	tWidget->draw();

	int flags = SHOW_ABSTRACT_VIEW | SHOW_BODY_FRAME | SHOW_JOINTS | HIGHLIGHT_SELECTED;

	RobotState rs(rdApp->robot);
	rdApp->robot->setState(&rdApp->prd->defaultRobotState);

	glEnable(GL_LIGHTING);
	if (rdApp->simWindow->rbEngine)
		rdApp->simWindow->rbEngine->drawRBs(flags);

/*
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
*/

	rdApp->robot->setState(&rs);
}

void IntelligentRobotEditingWindow::test_dmdp_Jacobian() {
	dVector m; rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m);
	DynamicArray<double> p;	rdApp->prd->getCurrentSetOfParameters(p);

	//dm/dp, the analytic version. 
	update_dmdX();

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
		setParamsAndUpdateMOPT(p);
		//now we must solve this thing a loooot...

		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		for (int j = 0; j < 300; j++)
			rdApp->moptWindow->runMOPTStep();

		rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m_m);
		p[i] = pVal - dp;
		setParamsAndUpdateMOPT(p);
		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		for (int j = 0; j < 300; j++)
			rdApp->moptWindow->runMOPTStep();
		rdApp->moptWindow->locomotionManager->motionPlan->writeMPParametersToList(m_p);

		rdApp->moptWindow->locomotionManager->motionPlan->setMPParametersFromList(m_initial);
		p[i] = pVal;
		setParamsAndUpdateMOPT(p);

		dmdp_FD.col(i) = (m_p - m_m) / (2 * dp);
	}

	print("../out/dmdp.m", dmdX);
	print("../out/dmdp_FD.m", dmdp_FD);
}
