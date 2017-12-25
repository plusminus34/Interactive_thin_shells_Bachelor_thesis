#pragma warning(disable : 4996)

#include <GUILib/GLUtils.h>
#include <RobotDesignerLib/ModularDesignWindow.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <MathLib/ConvexHull2D.h>
#include <MathLib/ConvexHull3D.h>
#include <RobotDesignerLib/ModuleDisplayWindow.h>
#include <iostream>
#include <RobotDesignerLib/LivingMotor.h>
#include <RobotDesignerLib/LivingConnector.h>
#include <RobotDesignerLib/LivingSphereEE.h>

/* 
pick an RMCRobot by clicking on it, translation and rotation widgets will show on its root.
	And its picked RMC will be painted in orange. (Root is painted in green)

Once you select an RMCRobot:
key 'Q': clone the RMCRobot
key 'V': save the picked RMCRobot to ../out/tmpRMCRobot.mrb
key 'F': switch the parent joint of the picked RMC to the next relative transformation.
key 'D': delete the subtree from the selected RMC

key 'S': save the whole design to ../out/tmpModularDesign.dsn
key 'R': load design from ../out/tmpModularDesign.dsn

key 'Z': add a body feature
key '-' and '=': resize selected body feature

// Fabrication
key 'J': output fabricatable rigidbody meshes in ../out/. File format: $(rbName)_merged.obj

// Modifier
TIP 1: Select one RMC, click on another RMC while holding SHIFT to make them symmetric.
TIP 2: Hold ALT while changing the position and orientation of motors to keep child components rigidly attached.
TIP 3: Hold CTRL to change the orientation of motors discretely.
TIP 4: When selecting body plates, use CTRL to create only 1, or otherwise a symmetric pair will be generated.
TIP 5: With a motor selected, F1/F2 change its angle value, while F3 resets it to zero. This does not change the assembly configuration!


*/

using namespace std;

ModularDesignWindow::ModularDesignWindow(int x, int y, int w, int h, GLApplication* glApp, const char* libraryDefinitionFileName) : AbstractDesignWindow(x, y, w, h){
	this->glApp = glApp;
	((GLTrackingCamera*)camera)->camDistance = -0.5;
	((GLTrackingCamera*)camera)->rotAboutRightAxis = 0.3;
	((GLTrackingCamera*)camera)->rotAboutUpAxis = 0.3;

	componentLibrary = new GLWindowContainer(3, 3, x, (int)(h * 3.0 / 4), (int)(w), (int)(h /4.0));

	tWidget = new TranslateWidget(AXIS_X | AXIS_Y | AXIS_Z);
	//	rWidget = new RotateWidgetV1();
	rWidget = new RotateWidgetV2();
	tWidget->visible = rWidget->visible = false;

	loadConfig(libraryDefinitionFileName);

	//	TwAddVarRW(glApp->mainMenuBar, "ShowBodyFeature", TW_TYPE_BOOLCPP, &showBodyFeature, "");

	Logger::consolePrint("Library contains %d modular components...\n", rmcWarehouse.size());
	for (uint i = 0; i < rmcWarehouse.size(); i++){
		componentLibrary->addSubWindow(new ModuleDisplayWindow(rmcWarehouse[i]));
	}

	string bodyTexture = "../data/textures/matcap/whitefluff2.bmp";
	bodyMesh = new GLMesh();
	bodyMesh->path = robotMeshDir + "BodyMesh.obj";
	
	bodyMesh->getMaterial().setColor(1.0, 1.0, 1.0, 0.8);

	sphereMesh = new GLMesh();
	sphereMesh->addSphere(P3D(), 1, 12);
}

ModularDesignWindow::~ModularDesignWindow(void){

	for (uint i = 0; i < rmcRobots.size(); i++)
		delete rmcRobots[i];

	for (uint i = 0; i < rmcWarehouse.size(); i++)
		delete rmcWarehouse[i];
	
	delete guidingMesh;
	delete bodyMesh;
	delete windowSelectedRobot;
	delete componentLibrary;
	delete rWidget;
	delete tWidget;
	delete sphereMesh;
}

bool ModularDesignWindow::process() {

	if (!runTask) return false;
	// Logger::print("Running task!\n");

	timer.restart();

	double elapsedTime = timer.timeEllapsed();
	iterNum = max((int)(1.0 / 20 / elapsedTime), 1);
	
	return true;
}


//triggered when mouse moves
bool ModularDesignWindow::onMouseMoveEvent(double xPos, double yPos) {
	if (componentLibrary->onMouseMoveEvent(xPos, yPos) == true) return true;
	preDraw();

    PressedModifier mod = getPressedModifier(glApp->glfwWindow);

	for (uint i = 0; i < rmcRobots.size(); i++)
		rmcRobots[i]->highlightedRMC = NULL;
	hightlightedRobot = NULL;
	highlightedFP = NULL;

	bool bodyChanged = false;
	if (tWidget->onMouseMoveEvent(xPos, yPos) == true && dragging) {
		// move selected body feature points
		if (selectedFP)
		{
			selectedFP->coords = tWidget->pos;
			propagatePosToMirrorFp(selectedFP);
			bodyChanged = true;
		}
		
		if (selectedRobot){

			if (isSelectedRMCMovable() && selectedRobot->selectedRMC != selectedRobot->root){
				P3D pos = selectedRobot->selectedRMC->state.position = rWidget->pos = tWidget->pos;
				propagatePosToMirrorRMC(selectedRobot->selectedRMC);
//				if (GetAsyncKeyState(VK_LMENU) < 0)
                if(mod == PressedModifier::LEFT_ALT)
                {
					updateParentConnector(selectedRobot->selectedRMC);
					if (rmcMirrorMap.count(selectedRobot->selectedRMC))
						updateParentConnector(rmcMirrorMap[selectedRobot->selectedRMC]);
				}
				else
					updateLivingBracket();
			}
			else {
				if (selectedRobot->getRoot()->type == PLATE_RMC) {
					tWidget->pos[1] = bodyPlaneHeight;
					if (mirrorMap.count(selectedRobot) && mirrorMap[selectedRobot] && mirrorMap[selectedRobot]->root) {
						mirrorMap[selectedRobot]->root->state.position = tWidget->pos;
						mirrorMap[selectedRobot]->root->state.position[0] *= -1;
						mirrorMap[selectedRobot]->fixJointConstraints();
					}
					bodyChanged = true;
				}
				selectedRobot->root->state.position = rWidget->pos = tWidget->pos;
			}
			for (auto robot : rmcRobots)
				robot->fixJointConstraints();
		}
		if (pickedGuidingMesh){
			guidingMeshPos = rWidget->pos = tWidget->pos;
		}

		if (bodyChanged) createBodyMesh3D();

		postDraw();
		return true;
	}
	if (rWidget->onMouseMoveEvent(xPos, yPos) == true && dragging) {
		if (selectedRobot) {
			Quaternion q = rWidget->getOrientation();
//			if (GetAsyncKeyState(VK_LCONTROL) < 0)
            if(mod == PressedModifier::LEFT_CTRL)
			{
				V3D axis = q.v; axis.toUnit();
				double rotAngle = q.getRotationAngle(axis);
				if (rotAngle < 0) {
					rotAngle = -rotAngle;
					axis = -axis;
				}

				if (rotAngle < RAD(30)) {
					postDraw();
					return true;
				}
				else {
					q = getRotationQuaternion(RAD(30), axis);
				}
			}
			
			if (isSelectedRMCMovable() && selectedRobot->selectedRMC != selectedRobot->root)
			{
				Quaternion tQ = selectedRobot->selectedRMC->state.orientation = q * selectedRobot->selectedRMC->state.orientation;
				propagateOrientToMirrorRMC(selectedRobot->selectedRMC);
//				if (GetAsyncKeyState(VK_LMENU) < 0)
                if(mod == PressedModifier::LEFT_ALT)
				{
					updateParentConnector(selectedRobot->selectedRMC);
					if (rmcMirrorMap.count(selectedRobot->selectedRMC))
						updateParentConnector(rmcMirrorMap[selectedRobot->selectedRMC]);
				}
				else
					updateLivingBracket();
			}
			else {
				selectedRobot->root->state.orientation = q * selectedRobot->root->state.orientation;
				if (selectedRobot->getRoot()->type == PLATE_RMC) {
					propagateOrientToMirrorRMC(selectedRobot->root);
					bodyChanged = true;
				}
			}
			rWidget->setOrientation(Quaternion());
			for (auto robot : rmcRobots)
				robot->fixJointConstraints();

			if (bodyChanged) createBodyMesh3D();
		}
		if (pickedGuidingMesh)
		{
			guidingMeshRot = rWidget->getOrientation();
		}
		postDraw();
		return true;
	}
	Ray mouseRay = camera->getRayFromScreenCoords(xPos, yPos);
	postDraw();

	if (windowSelectedRobot)
	{
		if (!possibleConnections.empty()){
			double closestDist;
			P3D closestPoint;
			PossibleConnection* closestConnection = getClosestConnnection(mouseRay, possibleConnections, closestPoint, closestDist);
			windowSelectedRobot->getRoot()->state.orientation = closestConnection->orientation;
			windowSelectedRobot->getRoot()->state.position = closestPoint;
			windowSelectedRobot->fixJointConstraints();
			snappable = closestDist < SNAP_THRESHOLD;
		}
		else {
			P3D pos;
			if (windowSelectedRobot->getRoot()->type == PLATE_RMC) {
				Plane bodyPlane(P3D(0, bodyPlaneHeight, 0), V3D(0, 1, 0));
				mouseRay.getDistanceToPlane(bodyPlane, &pos);
			}
			else
				mouseRay.getDistanceToPoint(P3D(), &pos);

			if (noMirror) pos[0] = 0;
			windowSelectedRobot->getRoot()->state.position = pos;
			snappable = false;				
		}
		return true;
	}
	
	// pick highlighted body feature points
	if (!dragging) 
	{
		pickBodyFeaturePts(mouseRay);
		if (highlightedFP) return true;
	}

	// only triggered when dragging
	if (dragging)
	{
		snappable = false;
		if (selectedRobot && selectedRobot->selectedPin && possibleConnections.size() > 0)
		{
			double closestDist;
			P3D closestPoint;
			PossibleConnection* closestConnection = getClosestConnnection(mouseRay, possibleConnections, closestPoint, closestDist);
			selectedRobot->getRoot()->state.orientation = closestConnection->orientation;
			selectedRobot->getRoot()->state.position = closestPoint;
			selectedRobot->fixJointConstraints();
			snappable = closestDist < SNAP_THRESHOLD;
				
			return true;
		}
	}
	else {
		for (uint i = 0; i < rmcRobots.size(); i++)
		{
			if (rmcRobots[i]->getRoot()->type == PLATE_RMC) continue;
			
			rmcRobots[i]->highlightedRMC = NULL;
			if (rmcRobots[i]->pickPin(mouseRay))
			{
				hightlightedRobot = rmcRobots[i];
				return true;
			}
		}

		double closestDist = 1e10;
		for (uint i = 0; i < rmcRobots.size(); i++)
		{
			double dist;
			bool res = rmcRobots[i]->pickRMC(mouseRay, &dist);
			if (res && dist < closestDist)
			{
				// clear previous closest RMC
				if (hightlightedRobot)
					hightlightedRobot->highlightedRMC = NULL;

				hightlightedRobot = rmcRobots[i];
				closestDist = dist;
			}
			else {
				rmcRobots[i]->highlightedRMC = NULL;
		}
		}
		if (hightlightedRobot)
			return true;

	}
	

	if (GLWindow3D::onMouseMoveEvent(xPos, yPos) == true) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool ModularDesignWindow::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	
	preDraw();
	Ray mouseRay = camera->getRayFromScreenCoords(xPos, yPos);
	postDraw();

	// new RMCRobot initialization
	if (componentLibrary->onMouseButtonEvent(button, action, mods, xPos, yPos)) {
		if (action == 1){

			if (selectedRobot && selectedRobot->selectedRMC){
				unloadParametersForLivingBracket();
				selectedRobot->selectedRMC = NULL;
				selectedRobot = NULL;
				rWidget->visible = tWidget->visible = false;
			}

			int selectedRMC = -1;
			for (uint i = 0; i < componentLibrary->subWindows.size(); i++)
			{
				if (componentLibrary->subWindows[i]->isSelected()) {
					selectedRMC = i;
					break;
				}
			}

			if (selectedRMC >= 0){
				delete windowSelectedRobot;
				windowSelectedRobot = new RMCRobot(rmcWarehouse[selectedRMC]->clone(), transformationMap);
				noMirror = (bool)(mods == GLFW_MOD_CONTROL);
				
				if (windowSelectedRobot->getRoot()->type == PLATE_RMC) {
					windowSelectedRobot->root->state.position = V3D(0, 100, 0);
					return true;
				}
				possibleConnections.clear();
				for (uint k = 0; k < windowSelectedRobot->root->pins.size(); k++)
				{
					RMCPin* candidatePin = &windowSelectedRobot->root->pins[k];
					for (uint i = 0; i < rmcRobots.size(); i++)
					{
						vector<RMCPin*> availablePins;
						rmcRobots[i]->getAvailableCompatiblePins(candidatePin, availablePins);
						for (uint j = 0; j < availablePins.size(); j++)
						{
							if (availablePins[j]->rmc->type == PLATE_RMC && candidatePin->rmc->type != LIVING_CONNECTOR) continue;
							
							previewConnectRMCRobot(availablePins[j], candidatePin, windowSelectedRobot, false);
							PossibleConnection connection;
							connection.orientation = windowSelectedRobot->getRoot()->state.orientation;
							connection.position = windowSelectedRobot->getRoot()->state.position;
							connection.parentPin = availablePins[j];
							connection.childPin = candidatePin;
							connection.parentRobot = rmcRobots[i];
							possibleConnections.push_back(connection);
						}
					}
				}
				// make the robot out of sight initially
				windowSelectedRobot->root->state.position = V3D(0, 100, 0);
			}
			else {
				possibleConnections.clear();
				delete windowSelectedRobot;
				windowSelectedRobot = NULL;
			}
			return true;
		}
	}
	else if (windowSelectedRobot) {
		
		double closestDist;
		P3D closestPoint;
		PossibleConnection* closestConnection = getClosestConnnection(mouseRay, possibleConnections, closestPoint, closestDist);

		if (closestConnection && closestDist < SNAP_THRESHOLD){
			closestConnection->parentRobot->connectRMCRobot(windowSelectedRobot, closestConnection->parentPin, closestConnection->childPin);
			buildRMCMirrorMap();
		}
		else {
			P3D pos;
			bool isBody = false;
			if (windowSelectedRobot->getRoot()->type == PLATE_RMC) {
				isBody = true;
				Plane bodyPlane(P3D(0, bodyPlaneHeight, 0), V3D(0, 1, 0));
				mouseRay.getDistanceToPlane(bodyPlane, &pos);

				if (!noMirror)
				{
					RMCRobot* mirrorRobot = new RMCRobot(windowSelectedRobot->getRoot()->clone(), transformationMap);
					mirrorRobot->getRoot()->state.position = P3D(-pos[0], pos[1], pos[2]);
					rmcRobots.push_back(mirrorRobot);
					mirrorMap[windowSelectedRobot] = mirrorRobot;
					mirrorMap[mirrorRobot] = windowSelectedRobot;
				}
			}
			else 
				mouseRay.getDistanceToPoint(P3D(), &pos);
			rmcRobots.push_back(windowSelectedRobot);
			if (noMirror) pos[0] = 0;
			rmcRobots.back()->getRoot()->state.position = pos;	
			if (isBody) createBodyMesh3D();
		}

		windowSelectedRobot = NULL;
		possibleConnections.clear();
		return true;
	}

	if (button == 0)
	{
		if (action == 1)
		{
			dragging = true;

			preDraw();
			bool res = rWidget->onMouseMoveEvent(xPos, yPos);
			postDraw();
			if (res) return true;

			preDraw();
			res = tWidget->onMouseMoveEvent(xPos, yPos);
			postDraw();
			if (res) return	true;

			tWidget->visible = rWidget->visible = false;
			pickedGuidingMesh = false;

			if (((mods & GLFW_MOD_SHIFT) > 0) && hightlightedRobot && hightlightedRobot->highlightedRMC
				&& selectedRobot && selectedRobot->selectedRMC)
			{
				makeSelectedRMCSymmtry();
			}

			if (((mods & GLFW_MOD_SHIFT) > 0) && highlightedFP && selectedFP)
			{
				makeSelectedFpSymmtry();
			}

			// reset pick pointers
			unloadParametersForLivingBracket();
			if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_MOTOR)
			{
//				TwRemoveVar(glApp->mainMenuBar, "motor rotation angle");
				motorRotAngle = 0;
			}

			for (uint i = 0; i < rmcRobots.size(); i++) {
				rmcRobots[i]->selectedRMC = NULL;
				rmcRobots[i]->selectedPin = NULL;
			}
			possibleConnections.clear();

			selectedRobot = hightlightedRobot;
			if (selectedRobot) {
				selectedRobot->selectedRMC = selectedRobot->highlightedRMC;
				selectedRobot->selectedPin = selectedRobot->highlightedPin;
			}

			selectedFP = highlightedFP;
			if (selectedFP)
			{
				tWidget->visible = true;
				tWidget->pos = selectedFP->coords;
				return true;
			}

//			need to design two custom brackets to flip the top motor on the hind leg and make the second one look nicer...

			// if some RMCPin is selected, generate possible states.	
			if (selectedRobot && selectedRobot->selectedPin)
			{
				Logger::consolePrint("Select pin %s\n", selectedRobot->selectedPin->name.c_str());

				map<RMC*, RBState> stateMap;
				stateMap[selectedRobot->getRoot()] = selectedRobot->getRoot()->state;
				for (int i = 0; i < selectedRobot->getJointCount(); i++)
				{
					RMC* rmc = selectedRobot->getJoint(i)->getChild();
					stateMap[rmc] = rmc->state;
				}

				for (uint i = 0; i < rmcRobots.size(); i++)
				{
					if (rmcRobots[i] == selectedRobot) continue;

					vector<RMCPin*> availablePins;
					rmcRobots[i]->getAvailableCompatiblePins(selectedRobot->selectedPin, availablePins);
					for (uint j = 0; j < availablePins.size(); j++)
					{
						previewConnectRMCRobot(availablePins[j], selectedRobot->selectedPin, selectedRobot, false);
						PossibleConnection connection;
						connection.orientation = selectedRobot->getRoot()->state.orientation;
						connection.position = selectedRobot->getRoot()->state.position;
						connection.parentPin = availablePins[j];
						connection.childPin = selectedRobot->selectedPin;
						connection.parentRobot = rmcRobots[i];
						possibleConnections.push_back(connection);
					}
				}

				for (auto& itr : stateMap)
				{
					itr.first->state = itr.second;
				}
			}

			// if some RMC is selected, enable transformation widgets.
			if (selectedRobot && selectedRobot->selectedRMC)
			{
				Logger::consolePrint("Select RMC %s\n", selectedRobot->selectedRMC->name.c_str());

				if (showWidgets)
				{
					rWidget->visible = tWidget->visible = true;
					rWidget->pos = tWidget->pos = isSelectedRMCMovable() ? selectedRobot->selectedRMC->state.position : selectedRobot->root->state.position;
					rWidget->setOrientation(Quaternion());
				}
				loadParametersForLivingBracket();
				if (selectedRobot->selectedRMC->type == LIVING_MOTOR) {
					motorStartOrient = selectedRobot->selectedRMC->state.orientation;
					motorRotAngle = 0;
//					TwAddVarRW(glApp->mainMenuBar, "motor rotation angle", TW_TYPE_DOUBLE, &motorRotAngle, "min=-180 max=180 step=1 group='LivingBracket'");
				}					
			}

			if (selectedRobot && (selectedRobot->selectedRMC || selectedRobot->selectedPin))
				return true;

			if (guidingMesh) {
				Transformation invTrans = Transformation(guidingMeshRot.getRotationMatrix(), guidingMeshPos).inverse();
				Ray newRay(invTrans.transform(mouseRay.origin) / guidingMeshScale, invTrans.transform(mouseRay.direction) / guidingMeshScale);
				pickedGuidingMesh = guidingMesh->getDistanceToRayOriginIfHit(newRay);
			}
				
			if (pickedGuidingMesh) {
				if (showWidgets)
				{
					rWidget->visible = tWidget->visible = true;
					rWidget->pos = tWidget->pos = guidingMeshPos;
					rWidget->setOrientation(guidingMeshRot);
				}
				return true;
			}
		}

		if (action == 0) {

			dragging = false;

			rWidget->setOrientation(Quaternion());

			if (selectedRobot && selectedRobot->selectedPin)
			{
				double closestDist;
				P3D closestPoint;
				PossibleConnection* closestConnection = getClosestConnnection(mouseRay, possibleConnections, closestPoint, closestDist);

				if (closestConnection && closestDist < SNAP_THRESHOLD)
				{
					closestConnection->parentRobot->connectRMCRobot(selectedRobot, closestConnection->parentPin, selectedRobot->selectedPin);
					removeRMCRobot(selectedRobot);
					closestConnection->parentRobot->clearPinPick();
					buildRMCMirrorMap();
				}	
				else {
					selectedRobot->clearPinPick();
				}
				possibleConnections.clear();
				selectedRobot = NULL;
				return true;
			}
		}
		
	}
	

	if (GLWindow3D::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool ModularDesignWindow::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (componentLibrary->onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	if (pickedGuidingMesh)	{
		guidingMeshScale *= (1 + yOffset * 0.05);
		return true;
	}

	if (selectedFP)
	{
		selectedFP->featureSize *= (1 + yOffset * 0.05);
		propagatePosToMirrorFp(selectedFP);
		createBodyMesh3D();
		return true;
	}

	if (GLWindow3D::onMouseWheelScrollEvent(xOffset, yOffset)) return true;
	return false;
}

bool ModularDesignWindow::onKeyEvent(int key, int action, int mods) {
	if (componentLibrary->onKeyEvent(key, action, mods)) return true;

	// switch to the next joint transformation
	if (key == GLFW_KEY_F && action == GLFW_PRESS)
	{
		if (selectedRobot && selectedRobot->selectedRMC)
		{
			RMCJoint* pJoint = selectedRobot->selectedRMC->getParentJoint();
			if (pJoint)
				pJoint->switchToNextTransformation();
			selectedRobot->fixJointConstraints();

			buildRMCMirrorMap();
		}
	}

	if (key == GLFW_KEY_E && action == GLFW_PRESS)
	{
		saveToRBSFile("../out/tmpRobot.rbs");
	}

	if (key == GLFW_KEY_K && action == GLFW_PRESS)
	{
		exportMeshes();
	}

	if (key == GLFW_KEY_V && action == GLFW_PRESS)
	{
		if (selectedRobot && selectedRobot->selectedRMC)
		{
			Logger::consolePrint("Save picked RMCRobot to file '../out/tmpRMCRobot.mrb'");
			selectedRobot->saveToFile("../out/tmpRMCRobot.mrb");
		}
	}

	// clone the picked robot
	if (key == GLFW_KEY_Q && action == GLFW_PRESS)
	{
		if (selectedRobot && selectedRobot->selectedRMC)
		{
			RMCRobot* newRobot = selectedRobot->cloneSubTree(selectedRobot->selectedRMC);
			rmcRobots.push_back(newRobot);	

			if (selectedRobot->selectedRMC->type == PLATE_RMC)
				newRobot->root->state.position = selectedRobot->root->state.position + V3D(0, 0, -0.05);
			else
				newRobot->root->state.position = P3D(0, 0.05, 0);
			newRobot->fixJointConstraints();

			// mirror new robot
			if (selectedRobot->selectedRMC->type == PLATE_RMC &&
				selectedRobot->selectedRMC == selectedRobot->root && mirrorMap.count(selectedRobot))
			{
				RMCRobot* mirrorSelRobot = mirrorMap[selectedRobot];
				RMCRobot* mirrorNewRobot = mirrorSelRobot->cloneSubTree(mirrorSelRobot->root);
				rmcRobots.push_back(mirrorNewRobot);

				if (mirrorSelRobot->root->type == PLATE_RMC)
					mirrorNewRobot->root->state.position = mirrorSelRobot->root->state.position + V3D(0, 0, -0.05);
				else
					mirrorNewRobot->root->state.position = P3D(0, 0.05, 0);
				mirrorNewRobot->fixJointConstraints();

				mirrorMap[newRobot] = mirrorNewRobot;
				mirrorMap[mirrorNewRobot] = newRobot;
				rmcMirrorMap[newRobot->root] = mirrorNewRobot->root;
				rmcMirrorMap[mirrorNewRobot->root] = newRobot->root;
			}

			if (selectedRobot->selectedRMC->type == PLATE_RMC)
				createBodyMesh3D();
		}
	}

	// delete the sub tree structure from the selected RMC
	if (key == GLFW_KEY_D && action == GLFW_PRESS)
	{
		if (selectedFP)
		{
			for (uint i = 0; i < bodyFeaturePts.size(); i++)
			{
				if (&bodyFeaturePts[i] == selectedFP)
				{
					bodyFeaturePts.erase(bodyFeaturePts.begin() + i);
					break;
				}
			}
			createBodyMesh3D();
			buildRMCMirrorMap();
		}

		if (selectedRobot && selectedRobot->selectedRMC)
		{
			unloadParametersForLivingBracket();
			if (selectedRobot->selectedRMC == selectedRobot->root) {
				removeRMCRobot(selectedRobot);
				if (mirrorMap.count(selectedRobot))
				{
					RMCRobot* mirrorRobot = mirrorMap[selectedRobot];
					removeRMCRobot(mirrorRobot);
					mirrorMap.erase(selectedRobot);
					mirrorMap.erase(mirrorRobot);
				}
				if (selectedRobot->selectedRMC->type == PLATE_RMC)
					createBodyMesh3D();

				rWidget->visible = tWidget->visible = false;
				selectedRobot = NULL;
			}
			else {
				if (isSelectedRMCMovable()) {
					rWidget->visible = tWidget->visible = false;
				}
				selectedRobot->deleteSubTree(selectedRobot->selectedRMC->getParentJoint());
				selectedRobot->selectedRMC = NULL;
			}
			buildRMCMirrorMap();
		}
	}
	
	if (key == GLFW_KEY_A && action == GLFW_PRESS)
	{
		if (selectedRobot && selectedRobot->selectedRMC)
		{
			if (selectedRobot->selectedRMC != selectedRobot->root) {
				RMCRobot* newRobot = selectedRobot->cloneSubTree(selectedRobot->selectedRMC);
				rmcRobots.push_back(newRobot);
				newRobot->root->state.position = V3D(0, 0.05, 0);
				newRobot->fixJointConstraints();
				selectedRobot->deleteSubTree(selectedRobot->selectedRMC->getParentJoint());
				buildRMCMirrorMap();
			}
		}
	}

	if (key == GLFW_KEY_R && action == GLFW_PRESS)
	{
		loadDesignFromFile("../out/tmpModularRobotDesign.dsn");
	}

	if (key == GLFW_KEY_S && action == GLFW_PRESS)
	{
		saveFile("../out/tmpModularRobotDesign.dsn");
	}

	if (key == GLFW_KEY_MINUS && action == GLFW_PRESS) {
		if (selectedFP){
			selectedFP->featureSize = max(0.01, selectedFP->featureSize - 0.005);
			createBodyMesh3D();
		}
		if (selectedRobot && selectedRobot->selectedRMC) {
			if (LivingSphereEE* selectedSphere = dynamic_cast<LivingSphereEE*> (selectedRobot->selectedRMC)) {
				selectedSphere->sphereRadius = max(0.01, selectedSphere->sphereRadius - 0.005);
				selectedSphere->update();
				updateLivingBracket();
			}
		}
		if (selectedRobot && selectedRobot->selectedRMC) {
			if (LivingWheelEE* selectedWheel = dynamic_cast<LivingWheelEE*> (selectedRobot->selectedRMC)) {
				selectedWheel->radius = max(0.01, selectedWheel->radius - 0.005);
				selectedWheel->update();
				updateLivingBracket();
			}
		}
	}


	if (selectedRobot && selectedRobot->selectedRMC && action == GLFW_PRESS) {
		if (LivingMotor* selectedMotor = dynamic_cast<LivingMotor*> (selectedRobot->selectedRMC)) {
			if (key == GLFW_KEY_LEFT_BRACKET){
				selectedMotor->hornBracket->goToNextMountingPosition();
				Logger::consolePrint("horn bracket mounting angle: %lf\n", selectedMotor->hornBracket->bracketMountingAngle);
			}
			if (key == GLFW_KEY_RIGHT_BRACKET){
				selectedMotor->hornBracket->goToPreviousMountingPosition();
				Logger::consolePrint("horn bracket moounting angle: %lf\n", selectedMotor->hornBracket->bracketMountingAngle);
			}

			selectedMotor->update();
			updateLivingBracket();
		}
	}


	if (key == GLFW_KEY_EQUAL && action == GLFW_PRESS) {
		if (selectedFP){
			selectedFP->featureSize = selectedFP->featureSize + 0.005;
			createBodyMesh3D();
		}
		if (selectedRobot && selectedRobot->selectedRMC) {
			if (LivingSphereEE* selectedSphere = dynamic_cast<LivingSphereEE*> (selectedRobot->selectedRMC)) {
				selectedSphere->sphereRadius = selectedSphere->sphereRadius + 0.005;
				selectedSphere->update();
				updateLivingBracket();
			}
		}
		if (selectedRobot && selectedRobot->selectedRMC) {
			if (LivingWheelEE* selectedWheel = dynamic_cast<LivingWheelEE*> (selectedRobot->selectedRMC)) {
				selectedWheel->radius = selectedWheel->radius + 0.005;
				selectedWheel->update();
				updateLivingBracket();
			}
		}
	}


	if (key == GLFW_KEY_Z && action == GLFW_PRESS)
	{
		bodyFeaturePts.push_back(RBFeaturePoint(P3D(0, 0, 0), 0.02));
		createBodyMesh3D();
	}

	if (key == GLFW_KEY_X && action == GLFW_PRESS){
		showWidgets = !showWidgets;
		if (showWidgets && selectedRobot && selectedRobot->selectedRMC)
		{
			rWidget->visible = tWidget->visible = true;
			rWidget->pos = tWidget->pos = isSelectedRMCMovable() ? selectedRobot->selectedRMC->state.position : selectedRobot->root->state.position;
			rWidget->setOrientation(Quaternion());
		}
		if (showWidgets && pickedGuidingMesh)
		{
			rWidget->visible = tWidget->visible = true;
			rWidget->pos = tWidget->pos = guidingMeshPos;
			rWidget->setOrientation(guidingMeshRot);
		}
		if (!showWidgets)
		{
			rWidget->visible = tWidget->visible = false;
		}
	}

	if (key == GLFW_KEY_V && action == GLFW_PRESS)
	{
		createBodyMesh3D();
	}

	if (key == GLFW_KEY_Z && action == GLFW_PRESS)
	{
		if (selectedRobot && selectedRobot->selectedRMC && mirrorMap.count(selectedRobot) == 0)
		{
			selectedRobot->getRoot()->state.position[0] = 0;
			selectedRobot->fixJointConstraints();
			createBodyMesh3D();
			rWidget->pos = tWidget->pos = selectedRobot->getRoot()->state.position;
		}
	}


	if (key == GLFW_KEY_F1 && (action == GLFW_REPEAT || action == GLFW_PRESS))
	{
		if (selectedRobot && selectedRobot->selectedRMC && (selectedRobot->selectedRMC->type == MOTOR_RMC
			|| selectedRobot->selectedRMC->type == LIVING_MOTOR))
		{
			selectedRobot->selectedRMC->motorAngle += 5;
			Logger::consolePrint("Motor angle is now: %lf\n", selectedRobot->selectedRMC->motorAngle);
			selectedRobot->selectedRMC->update();

			// ******************* handle mirror RMC *******************
			if (rmcMirrorMap.count(selectedRobot->selectedRMC))
			{
				RMC* mirrorRMC = rmcMirrorMap[selectedRobot->selectedRMC];
				mirrorRMC->motorAngle -= 5;
				mirrorRMC->update();
			}

			for (auto robot : rmcRobots)
				robot->fixJointConstraints();
		}
	}

	if (key == GLFW_KEY_F2 && (action == GLFW_REPEAT || action == GLFW_PRESS))
	{
		if (selectedRobot && selectedRobot->selectedRMC && (selectedRobot->selectedRMC->type == MOTOR_RMC
			|| selectedRobot->selectedRMC->type == LIVING_MOTOR))
		{
			selectedRobot->selectedRMC->motorAngle -= 5;
			Logger::consolePrint("Motor angle is now: %lf\n", selectedRobot->selectedRMC->motorAngle);
			selectedRobot->selectedRMC->update();
			
			// ******************* handle mirror RMC *******************
			if (rmcMirrorMap.count(selectedRobot->selectedRMC))
			{
				RMC* mirrorRMC = rmcMirrorMap[selectedRobot->selectedRMC];
				mirrorRMC->motorAngle += 5;
				mirrorRMC->update();
			}

			for (auto robot : rmcRobots)
				robot->fixJointConstraints();
		}
	}

	if (key == GLFW_KEY_F3 && (action == GLFW_REPEAT || action == GLFW_PRESS))
	{
		if (selectedRobot && selectedRobot->selectedRMC && (selectedRobot->selectedRMC->type == MOTOR_RMC
			|| selectedRobot->selectedRMC->type == LIVING_MOTOR))
		{
			selectedRobot->selectedRMC->motorAngle = 0.0;
			selectedRobot->selectedRMC->update();

			// ******************* handle mirror RMC *******************
			if (rmcMirrorMap.count(selectedRobot->selectedRMC))
			{
				RMC* mirrorRMC = rmcMirrorMap[selectedRobot->selectedRMC];
				mirrorRMC->motorAngle = 0.0;
				mirrorRMC->update();
			}

			for (auto robot : rmcRobots)
				robot->fixJointConstraints();
		}

	}

	if (key == GLFW_KEY_F4 && (action == GLFW_REPEAT || action == GLFW_PRESS))
	{
		if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->isMovable())
		{
			selectedRobot->selectedRMC->state.orientation = getRotationQuaternion(RAD(90), V3D(0, 0, 1)) * selectedRobot->selectedRMC->state.orientation;
			selectedRobot->updateAllLivingMotor();
			selectedRobot->fixJointConstraints();
		}
	}


	if (GLWindow3D::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool ModularDesignWindow::onCharacterPressedEvent(int key, int mods) {
	if (GLWindow3D::onCharacterPressedEvent(key, mods)) return true;

	return false;
}


void ModularDesignWindow::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);
	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt == "dsn"){
		loadDesignFromFile(fName);
	}
	else if (fNameExt == "mrb") {
		RMCRobot* newRobot = new RMCRobot(transformationMap);
		newRobot->loadFromFile("../out/tmpRMCRobot.mrb", rmcNameMap);
		rmcRobots.push_back(newRobot);
	}
	else if (fNameExt == "obj"){
		guidingMesh = GLContentManager::getGLMesh(fName);
		guidingMesh->getMaterial().setColor(0.8, 0.8, 1.0, 0.4);
		pickedGuidingMesh = false;
	}
}

void ModularDesignWindow::saveFile(const char* fName) {
	//Logger::consolePrint("SAVE FILE: Design file saved to \'%s\'\n", fName);

	std::string fileName;
	fileName.assign(fName);
	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt == "dsn")
	{
		saveDesignToFile(fName);
	}
	else if (fNameExt == "rbs")
	{
		saveToRBSFile(fName);
	}
}

void ModularDesignWindow::drawRefAxis(const P3D& pos)
{
	glColor3d(1.0, 1.0, 0.0);
	drawArrow(pos, pos + V3D(0.01, 0, 0), 0.001, 12);
	glColor3d(0.0, 1.0, 1.0);
	drawArrow(pos, pos + V3D(0, 0.01, 0), 0.001, 12);
	glColor3d(1.0, 0.0, 1.0);
	drawArrow(pos, pos + V3D(0, 0, 0.01), 0.001, 12);
}

void ModularDesignWindow::drawRMCRobot()
{
	updateLivingBracket();

	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		rmcRobots[i]->draw(SHOW_PINS);
		if (rmcRobots[i] == selectedRobot && selectedRobot->selectedPin && snappable)
			rmcRobots[i]->draw(SHOW_MESH, Vector4d(1, 0, 0, 0.8), Vector4d(1, 0, 0, 0.8), Vector4d(1, 0, 0, 0.8));
		else
			rmcRobots[i]->draw(SHOW_MESH, Vector4d(0, 0, 0, 0), Vector4d(1, 0.5, 0, 1));

//		if (showMOIBox)
//			rmcRobots[i]->draw(SHOW_MOI_BOX);
	}

}

void ModularDesignWindow::drawConnectionPreview()
{
	if (selectedRobot && selectedRobot->selectedPin)
	{
		RBState origState = selectedRobot->root->state;

		for (uint i = 0; i < possibleConnections.size(); i++)
		{
			selectedRobot->root->state.position = possibleConnections[i].position;
			selectedRobot->root->state.orientation = possibleConnections[i].orientation;
			selectedRobot->fixJointConstraints();
			selectedRobot->draw(SHOW_MESH, Vector4d(1, 0, 0, 0.4), Vector4d(1, 0, 0, 0.4), Vector4d(1, 0, 0, 0.4));
		}

		selectedRobot->root->state = origState;
		selectedRobot->fixJointConstraints();
	}
}

void ModularDesignWindow::drawWindowRMCConnectionPreview()
{
	if (windowSelectedRobot)
	{
		RBState origState = windowSelectedRobot->root->state;
		if (snappable)
			windowSelectedRobot->draw(SHOW_MESH, Vector4d(1, 0, 0, 0.8));
		else if (windowSelectedRobot->getRoot()->type == PLATE_RMC) {
			windowSelectedRobot->draw(SHOW_MESH, Vector4d(1, 0, 1, 0.8));
			if (!noMirror){
				windowSelectedRobot->getRoot()->state.position[0] *= -1;
				Quaternion q = windowSelectedRobot->getRoot()->state.orientation;
				double angle = q.getRotationAngle(V3D(0, 1, 0));
				windowSelectedRobot->getRoot()->state.orientation = getRotationQuaternion(-angle, V3D(0, 1, 0));
				windowSelectedRobot->draw(SHOW_MESH, Vector4d(1, 0, 1, 0.8));
			}
		}
		else
			windowSelectedRobot->draw(SHOW_MESH);

		
		for (uint i = 0; i < possibleConnections.size(); i++)
		{
			windowSelectedRobot->root->state.position = possibleConnections[i].position;
			windowSelectedRobot->root->state.orientation = possibleConnections[i].orientation;
			windowSelectedRobot->fixJointConstraints();
			windowSelectedRobot->draw(SHOW_MESH, Vector4d(1, 0, 0, 0.4), Vector4d(1, 0, 0, 0.4), Vector4d(1, 0, 0, 0.4));
		}
		windowSelectedRobot->root->state = origState;
	}

}

void ModularDesignWindow::drawBodyPlane() {

	if (windowSelectedRobot && windowSelectedRobot->root->type == PLATE_RMC
		|| selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == PLATE_RMC)
	{
		glColor4d(0.8, 0.8, 1.0, 0.4);
		drawBox(P3D(-1, bodyPlaneHeight - 0.001, -1), P3D(1, bodyPlaneHeight + 0.001, 1));
	}
}

void ModularDesignWindow::drawGuildingMesh() {
	if (guidingMesh) {
		
		glEnable(GL_NORMALIZE);
		glPushMatrix();
		glTranslated(guidingMeshPos[0], guidingMeshPos[1], guidingMeshPos[2]);
		//and rotation part
		//guidingMeshRot = getRotationQuaternion(RAD(360), V3D(1, 0, 0));
		V3D rotAxis; double rotAngle;
		guidingMeshRot.getAxisAngle(rotAxis, rotAngle);
		//Logger::print("Axis:%lf %lf %lf, Angle: %lf\n", rotAxis[0], rotAxis[1], rotAxis[2], rotAngle);

		glRotated(DEG(rotAngle), rotAxis[0], rotAxis[1], rotAxis[2]);
		glScaled(guidingMeshScale, guidingMeshScale, guidingMeshScale);
		guidingMesh->drawMesh();

		glPopMatrix();
		glDisable(GL_NORMALIZE);
	}
		
}

void ModularDesignWindow::drawBodyFeaturePts()
{
	if (!showBodyFeature) return;

	glDisable(GL_TEXTURE_2D);
	for (auto& fp : bodyFeaturePts)
	{
		if (&fp == highlightedFP || &fp == selectedFP)
			glColor4d(1.0, 0.0, 0.0, 1.0);
		else
			glColor4d(1.0, 0.8, 0.8, 1.0);
		drawSphere(fp.coords, fp.featureSize * 0.95, 12);
	}
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void ModularDesignWindow::drawScene() {

	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
 
	glEnable(GL_LIGHTING);

	drawRMCRobot();
	
	drawConnectionPreview();
	drawWindowRMCConnectionPreview();

	drawBodyFeaturePts();
	if (bodyMesh)
		bodyMesh->drawMesh();

	drawGuildingMesh();

	drawBodyPlane();
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void ModularDesignWindow::drawAuxiliarySceneInfo() {
	//clear the depth buffer so that the widgets show up on top of the object primitives
	
	
	glClear(GL_DEPTH_BUFFER_BIT);

	// draw the widgets
	preDraw();
	tWidget->draw();
	rWidget->draw();
	postDraw();

	glClear(GL_DEPTH_BUFFER_BIT);
	componentLibrary->draw();
	
}


void ModularDesignWindow::loadConfig(const char* fName)
{
	configFileName = fName;

	FILE* fp = fopen(fName, "r");

	char buffer[200];
	char keyword[50];
	RigidBody* newBody = NULL;
	//ArticulatedFigure* newFigure = NULL;
	//this is where it happens.
	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		sscanf(line, "%s", keyword);
		Logger::print("%s\n", keyword);

		if (strcmp(keyword, "RMC") == 0)
		{
			RMC* rmc = new RMC();
			rmcWarehouse.push_back(rmc);
			rmcWarehouse.back()->loadFromFile(fp);
			rmcNameMap[rmcWarehouse.back()->getName()] = rmc;
			
			for (uint i = 0; i < rmc->pins.size(); i++)
			{
				rmcPinNameMap[rmc->pins[i].name] = &rmc->pins[i];
			}
		}
		else if (strcmp(keyword, "LivingMotor") == 0)
		{
			RMC* rmc = new LivingMotor();
			rmcWarehouse.push_back(rmc);
			rmcWarehouse.back()->loadFromFile(fp);
			rmcNameMap[rmcWarehouse.back()->getName()] = rmc;

			for (uint i = 0; i < rmc->pins.size(); i++)
			{
				rmcPinNameMap[rmc->pins[i].name] = &rmc->pins[i];
			}
		}
		else if (strcmp(keyword, "LivingConnector") == 0)
		{
			RMC* rmc = new LivingConnector();
			rmcWarehouse.push_back(rmc);
			rmcWarehouse.back()->loadFromFile(fp);
			rmcNameMap[rmcWarehouse.back()->getName()] = rmc;

			for (uint i = 0; i < rmc->pins.size(); i++)
			{
				rmcPinNameMap[rmc->pins[i].name] = &rmc->pins[i];
			}
		}
		else if (strcmp(keyword, "LivingSphereEE") == 0)
		{
			RMC* rmc = new LivingSphereEE();
			rmcWarehouse.push_back(rmc);
			rmcWarehouse.back()->loadFromFile(fp);
			rmcNameMap[rmcWarehouse.back()->getName()] = rmc;

			for (uint i = 0; i < rmc->pins.size(); i++)
			{
				rmcPinNameMap[rmc->pins[i].name] = &rmc->pins[i];
			}
		}
		else if (strcmp(keyword, "LivingWheelEE") == 0)
		{
			RMC* rmc = new LivingWheelEE();
			rmcWarehouse.push_back(rmc);
			rmcWarehouse.back()->loadFromFile(fp);
			rmcNameMap[rmcWarehouse.back()->getName()] = rmc;

			for (uint i = 0; i < rmc->pins.size(); i++)
			{
				rmcPinNameMap[rmc->pins[i].name] = &rmc->pins[i];
			}
		}
		else if (strcmp(keyword, "RobotMeshDir") == 0)
		{
			char content[200];
			int num = sscanf(line + strlen(keyword), "%s", content);
			robotMeshDir = content;
		}
		else if (strcmp(keyword, "TransformationMap") == 0)
		{
			loadTransformationMap(fp);
		}
	}

	fclose(fp);
}

void ModularDesignWindow::loadTransformationMap(FILE* fp)
{
	char buffer[200];
	char keyword[50];
	vector<Transformation>* transList1 = NULL;
	vector<Transformation>* transList2 = NULL;
	Matrix3x3 R;
	V3D axis;


	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		sscanf(line, "%s", keyword);
		Logger::print("%s ", keyword);

		if (strcmp(keyword, "RMCPinPair") == 0)
		{
			char name1[200];
			char name2[200];
			sscanf(line + strlen(keyword), "%s %s", name1, name2);
			rmcPinNameMap[name1]->compatibleMap.insert(name2);
			rmcPinNameMap[name2]->compatibleMap.insert(name1);

			string key1 = string(name1) + '+' + string(name2);
			string key2 = string(name2) + '+' + string(name1);
			Logger::print("%s", key1.c_str());
			transformationMap[key1] = vector<Transformation>();
			transList1 = &transformationMap[key1];
			if (key1 != key2){
				transformationMap[key2] = vector<Transformation>();
				transList2 = &transformationMap[key2];
			}
			else {
				transList2 = NULL;
			}
			R.setIdentity();
		}
		else if (strcmp(keyword, "AngleAxis") == 0)
		{
			V3D axis;
			double angle;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf %lf",
				&angle, &axis[0], &axis[1], &axis[2]);
			if (num < 4)
				throwError("Not enough transformation parameters!");

			R *= AngleAxisd(RAD(angle), axis).toRotationMatrix();

			for (int i = 0; i < 9; i++)
				Logger::print("%lf ", R(i / 3, i % 3));
		}
		else if (strcmp(keyword, "RelativeAxis") == 0)
		{
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf",
					&axis[0], &axis[1], &axis[2]);
			if (num < 3)
				throwError("Not enough transformation parameters!");

			for (int i = 0; i < 3; i++)
				Logger::print("%lf ", axis[i]);
		}
		else if (strcmp(keyword, "RelativeAngle") == 0)
		{
			string str = line + strlen(keyword);
			int index = str.find_first_not_of(' ');
			bool stop = false;

			while (!stop && index != string::npos)
			{
				Transformation trans;
				double angle;
				int sIndex = str.find_first_of(' ', index);
				if (sIndex == string::npos) {
					angle = atof(str.substr(index).c_str());
					stop = true;
				}
				else {
					angle = atof(str.substr(index, sIndex - index).c_str());	
				}

				Matrix3x3 relR = AngleAxisd(RAD(angle), axis).toRotationMatrix();
				trans.R = relR * R;
				Logger::print("%lf ", angle);
				if (transList1)
					transList1->push_back(trans);
				if (transList2)
					transList2->push_back(trans.inverse());

				index = str.find_first_not_of(' ', sIndex);
			}
		}
		else if (strcmp(keyword, "Transformation") == 0)
		{
			Transformation trans;
			Matrix3x3& R = trans.R;
			Vector3d& T = trans.T;
			int num = sscanf(line + strlen(keyword), "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
				&R(0, 0), &R(0, 1), &R(0, 2), &R(1, 0), &R(1, 1), &R(1, 2), &R(2, 0), &R(2, 1), &R(2, 2),
				&T[0], &T[1], &T[2]);
			if (num < 12)
				throwError("Not enough transformation parameters!");

			for (int i = 0; i < 9; i++)
				Logger::print("%lf ", R(i / 3, i % 3));
			for (int i = 0; i < 3; i++)
				Logger::print("%lf ", T[i]);

			if (transList1)
				transList1->push_back(trans);
			if (transList2)
				transList2->push_back(trans.inverse());
		}
		else if (strcmp(keyword, "EndTransformationMap") == 0)
		{
			Logger::print("\n");
			break;
		}
		Logger::print("\n");
	}

	//**TEST
	for (auto itr = transformationMap.begin(); itr != transformationMap.end(); itr++)
	{
		Logger::print("%s\n", itr->first.c_str());
		vector<Transformation>& tmpList = itr->second;
		for (uint i = 0; i < tmpList.size(); i++)
		{
			Matrix3x3& R = tmpList[i].R;
			Vector3d& T = tmpList[i].T;

			for (int i = 0; i < 9; i++)
				Logger::print("%lf ", R(i / 3, i % 3));
			for (int i = 0; i < 3; i++)
				Logger::print("%lf ", T[i]);
			Logger::print("\n");
		}
	}
	
	
}

void ModularDesignWindow::saveDesignToFile(const char* fName)
{
	Logger::consolePrint("save design file to '%s'!\n", fName);

	FILE* fp = fopen(fName, "w+");

	fprintf(fp, "#### config file: %s\n\n", configFileName.c_str());

	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		rmcRobots[i]->saveToFile(fp);
	}

	fprintf(fp, "MirrorMap\n");
	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		RMCRobot* mirrorRobot = mirrorMap[rmcRobots[i]];
		for (uint j = i + 1; j < rmcRobots.size(); j++)
		{
			if (rmcRobots[j] == mirrorRobot)
			{
				fprintf(fp, "MirrorPair %d %d\n", i, j);
				fprintf(fp, "MirrorPair %d %d\n", j, i);
				break;
			}
		}
	}
	fprintf(fp, "EndMirrorMap\n\n\n");

	if (guidingMesh)
	{
		fprintf(fp, "MeshPath %s\n", guidingMesh->path.c_str());
		fprintf(fp, "MeshParam %lf %lf %lf %lf %lf %lf %lf %lf\n", guidingMeshPos[0], guidingMeshPos[1], guidingMeshPos[2],
			guidingMeshRot[0], guidingMeshRot[1], guidingMeshRot[2], guidingMeshRot[3], guidingMeshScale);
	}

	for (auto& bodyFp : bodyFeaturePts)
	{
		fprintf(fp, "BodyFeaturePoint %lf %lf %lf %lf\n", bodyFp.coords[0], bodyFp.coords[1], bodyFp.coords[2], bodyFp.featureSize);
	}
	
	fclose(fp);
}

void ModularDesignWindow::loadDesignFromFile(const char* fName)
{

	// clean up guiding mesh
	delete guidingMesh;
	guidingMeshPos = P3D();
	guidingMeshRot = Quaternion();
	guidingMeshScale = 1.0;

	bodyFeaturePts.clear();

	for (uint i = 0; i < rmcRobots.size(); i++)
		delete rmcRobots[i];
	rmcRobots.clear();
	int startIndex = (int)rmcRobots.size();

	unloadParametersForLivingBracket();
	selectedRobot = NULL;
	rWidget->visible = tWidget->visible = false;

	FILE* fp = fopen(fName, "r");

	char buffer[200];
	char keyword[50];


	while (!feof(fp)) {
		//get a line from the file...
		readValidLine(buffer, fp, 200);
		if (strlen(buffer) > 195)
			throwError("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		if (strlen(line) == 0) continue;
		sscanf(line, "%s", keyword);
		//Logger::print("%s ", keyword);

		if (strcmp(keyword, "RMCRobot") == 0)
		{
			RMCRobot* newRobot = new RMCRobot(transformationMap);
			newRobot->loadFromFile(fp, rmcNameMap);
			rmcRobots.push_back(newRobot);
		}
		else if (strcmp(keyword, "MirrorPair") == 0)
		{
			int index1, index2;
			sscanf(line + strlen(keyword), "%d %d", &index1, &index2);
			mirrorMap[rmcRobots[startIndex + index1]] = rmcRobots[startIndex + index2];
			mirrorMap[rmcRobots[startIndex + index2]] = rmcRobots[startIndex + index1];
		}
		else if (strcmp(keyword, "MeshPath") == 0)
		{
			sscanf(line + strlen(keyword), "%s", buffer);
			guidingMesh = GLContentManager::getGLMesh(buffer);
			guidingMesh->getMaterial().setColor(0.8, 0.8, 1.0, 0.4);
		}
		else if (strcmp(keyword, "MeshParam") == 0)
		{
			sscanf(line + strlen(keyword), "%lf %lf %lf %lf %lf %lf %lf %lf", &guidingMeshPos[0], &guidingMeshPos[1], &guidingMeshPos[2],
				&guidingMeshRot[0], &guidingMeshRot[1], &guidingMeshRot[2], &guidingMeshRot[3], &guidingMeshScale);
		}
		else if (strcmp(keyword, "BodyFeaturePoint") == 0)
		{
			RBFeaturePoint bodyFp(P3D(), 0.02);
			sscanf(line + strlen(keyword), "%lf %lf %lf %lf", &bodyFp.coords[0], &bodyFp.coords[1], &bodyFp.coords[2], &bodyFp.featureSize);
			bodyFeaturePts.push_back(bodyFp);
		}
	}

	fclose(fp);

	createBodyMesh3D();
	buildRMCMirrorMap();
}

bool ModularDesignWindow::previewConnectRMCRobot(RMCPin* parentPin, RMCPin* childPin, RMCRobot* childRobot, bool rotationOnly)
{
	string key = parentPin->name + '+' + childPin->name;
	Transformation trans;
	
	if (transformationMap.count(key)) {
		Transformation parentTrans = parentPin->transformation;
		Transformation childTrans = childPin->transformation;
		if (parentPin->rmc->type == MOTOR_RMC && parentPin->type == HORN_PIN)
		{
			RMC* parentRMC = parentPin->rmc;
			parentTrans *= Transformation(getRotationQuaternion(RAD(parentRMC->motorAngle), parentRMC->motorAxis).getRotationMatrix());
		}
		if (childPin->rmc->type == MOTOR_RMC && childPin->type == HORN_PIN)
		{
			RMC* childRMC = childPin->rmc;
			childTrans *= Transformation(getRotationQuaternion(RAD(childRMC->motorAngle), childRMC->motorAxis).getRotationMatrix());
		}
		trans = parentTrans * transformationMap[key][0] * childTrans.inverse();
	}
	else return false;

	RBState& parentState = parentPin->rmc->state;
	RBState& childState = childPin->rmc->state;
	Matrix3x3 rot = parentState.orientation.getRotationMatrix() * trans.R;
	childState.orientation.setRotationFrom(rot);
	if (!rotationOnly)
		childState.position = parentState.position + parentState.orientation.rotate(trans.T);

	childRobot->fixJointConstraints();

	return true;
}

PossibleConnection* ModularDesignWindow::getClosestConnnection(Ray& ray, vector<PossibleConnection>& connections, P3D& closestPoint, double& closestDist)
{
	PossibleConnection* closestConnection = NULL;
	closestDist = 1e10;

	for (uint i = 0; i < connections.size(); i++)
	{
		P3D p = connections[i].position;
		P3D cp;
		double dist = ray.getDistanceToPoint(p, &cp);
		if (dist < closestDist)
		{
			closestConnection = &connections[i];
			closestPoint = cp;
			closestDist = dist;
		}
	}

	return closestConnection;
}

void ModularDesignWindow::exportMeshes()
{
	Logger::consolePrint("Export design meshes!");
	RMCRobot* robot = new RMCRobot(new RMC(), transformationMap);

	FILE* fp = fopen(bodyMesh->path.c_str(), "w+");
	bodyMesh->renderToObjFile(fp, 0, Quaternion(), P3D());
	fclose(fp);

	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		if (rmcRobots[i]->getRoot()->type == PLATE_RMC)
		{
			robot->connectRMCRobotDirectly(rmcRobots[i]->clone(), robot->getRoot());
		}
	}
	robot->exportMeshes("../out/tmpModularRobotMeshes.obj", "../out/tmpModularRobotCarveMeshes.obj");

	delete robot;
}


/**
	Saves an rbs file. No motor rotations will be baked in...
*/
void ModularDesignWindow::saveToRBSFile(const char* fName, Robot* templateRobot){
	Logger::consolePrint("Save picked RMCRobot to RBS file '%s'\n", fName);

	RMCRobot* robot = new RMCRobot(new RMC(), transformationMap);
	robot->root->rbProperties.mass = 0;
	robot->root->rbProperties.MOI_local.setZero();

	robot->root->meshes.push_back(bodyMesh);
	FILE* fp = fopen(bodyMesh->path.c_str(), "w+");
	bodyMesh->renderToObjFile(fp, 0, Quaternion(), P3D());
	fclose(fp);

	GLContentManager::addMeshFileMapping(bodyMesh->clone(), bodyMesh->path.c_str());

	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		RMCRobot* rmcRobot = rmcRobots[i];
		for (int j = 0; j < rmcRobot->getRMCCount(); j++)
		{
			RMC* rmc = rmcRobot->getRMC(j);
			if (rmc->type == MOTOR_RMC || rmc->type == LIVING_MOTOR || rmc->type == EE_RMC)
			{
				rmc->mappingInfo.index1 = i;
				rmc->mappingInfo.index2 = j;
			}
		}
	}

	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		if (rmcRobots[i]->getRoot()->type == PLATE_RMC)
		{
			robot->connectRMCRobotDirectly(rmcRobots[i]->clone(), robot->getRoot());
		}
	}
	robot->fixJointConstraints();

	robot->saveToRBSFile(fName, robotMeshDir, templateRobot, freezeRobotRoot);

	//saverbs should also save an rs file, right after having restored the joint angles... I think it's cleaner that way...
//	ReducedRobotState tmpState = robot->getReducedRobotState(rbRobot);
//	tmpState.writeToFile(fName);

	delete robot;
}



void ModularDesignWindow::removeRMCRobot(RMCRobot* robot)
{
	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		if (rmcRobots[i] == robot)
		{
			rmcRobots.erase(rmcRobots.begin() + i);
			break;
		}
	}
}

void ModularDesignWindow::createBodyMesh2D()
{
	bodyMesh->clear();
	double halfHeight = 0.002;
	vector<Point2D> points;

	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		if (rmcRobots[i]->root->type == PLATE_RMC)
		{
			P3D p = rmcRobots[i]->root->state.position;
			points.push_back(Point2D(p[0], p[2]));
		}
	}
	if (points.size() < 3) return;

	points = ConvexHull2D::GrahamScan(points);

	P3D center;
	for (uint i = 0; i < points.size(); i++)
	{
		center += V3D(points[i].x, 0, points[i].y);
	}
	center /= points.size();

	bodyMesh->addVertex(center + V3D(0, halfHeight, 0));
	bodyMesh->addVertex(center + V3D(0, -halfHeight, 0));

	for (uint i = 0; i < points.size(); i++)
	{
		bodyMesh->addVertex(P3D(points[i].x, halfHeight, points[i].y));
		bodyMesh->addVertex(P3D(points[i].x, -halfHeight, points[i].y));
	}

	for (uint i = 0; i < points.size(); i++)
	{
		int pIndex1 = 2 * i + 2;
		int pIndex2 = 2 * i + 3;
		int pIndex3 = (i == points.size() - 1) ? 2 : 2 * i + 4;
		int pIndex4 = (i == points.size() - 1) ? 3 : 2 * i + 5;

		bodyMesh->addPoly(GLIndexedTriangle(0, pIndex1, pIndex3));
		bodyMesh->addPoly(GLIndexedTriangle(1, pIndex4, pIndex2));
		bodyMesh->addPoly(GLIndexedTriangle(pIndex1, pIndex2, pIndex3));
		bodyMesh->addPoly(GLIndexedTriangle(pIndex2, pIndex4, pIndex3));
	}

	bodyMesh->computeNormals();
}

void ModularDesignWindow::createBodyMesh3D()
{
	bodyMesh->clear();
	double halfHeight = 0.002;
	vector<P3D> points;

	for (uint i = 0; i < rmcRobots.size(); i++)
	{
		RMC* root = rmcRobots[i]->root;
		if (root->type == PLATE_RMC)
		{
			AxisAlignedBoundingBox bbox = root->meshes[0]->calBoundingBox();
			bbox.setbmin(bbox.bmin() + V3D(1, 1, 1) * 1e-5);
			bbox.setbmax(bbox.bmax() + V3D(1, 1, 1) * -1e-5);

			for (int j = 0; j < 8; j++)
				points.push_back(root->getWorldCoordinates(bbox.getVertex(j)));
		}
	}

	for (auto& fp : bodyFeaturePts)
	{
		for (int i = 0; i < sphereMesh->getVertexCount(); i++)
		{
			P3D v = sphereMesh->getVertex(i);
			points.push_back(v * fp.featureSize + fp.coords);
		}
	}

	if (points.size() < 4) return;

	ConvexHull3D::computeConvexHullFromSetOfPoints(points, bodyMesh, true);

	bodyMesh->computeNormals();
}


void ModularDesignWindow::loadParametersForLivingBracket()
{
	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_MOTOR)
	{
//		LivingHornBracket* lbh = ((LivingMotor*)selectedRobot->selectedRMC)->hornBracket;
//		TwAddVarRW(glApp->mainMenuBar, "initial angle", TW_TYPE_DOUBLE, &lbh->bracketInitialAngle, "min=-3.14 max=3.14 step=0.1 group='LivingBracket'");
//		TwAddVarRW(glApp->mainMenuBar, "motor angle min", TW_TYPE_DOUBLE, &lbh->motor->rotAngleMin, "min=-3.14 max=3.14 step=0.1 group='LivingBracket'");
//		TwAddVarRW(glApp->mainMenuBar, "motor angle max", TW_TYPE_DOUBLE, &lbh->motor->rotAngleMax, "min=-3.14 max=3.14 step=0.1 group='LivingBracket'");
//		TwAddVarRW(glApp->mainMenuBar, "bracket connector angle", TW_TYPE_DOUBLE, &lbh->bracketConnectorAngle, "min=-3.14 max=3.14 step=0.1 group='LivingBracket'");
	}

	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_SPHERE_EE)
	{
		LivingSphereEE* sphereEE = dynamic_cast<LivingSphereEE*>(selectedRobot->selectedRMC);
//		TwAddVarRW(glApp->mainMenuBar, "sphere radius", TW_TYPE_DOUBLE, &(sphereEE->sphereRadius), "min=0.001 max=0.1 step=0.001 group='LivingBracket'");
	}
	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_WHEEL_EE)
	{
		LivingWheelEE* wheelEE = dynamic_cast<LivingWheelEE*>(selectedRobot->selectedRMC);
//		TwAddVarRW(glApp->mainMenuBar, "wheel radius", TW_TYPE_DOUBLE, &(wheelEE->radius), "min=0.001 max=0.1 step=0.001 group='LivingBracket'");
	}

}

void ModularDesignWindow::unloadParametersForLivingBracket()
{
	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_MOTOR)
	{
//		TwRemoveVar(glApp->mainMenuBar, "initial angle");
//		TwRemoveVar(glApp->mainMenuBar, "motor angle min");
//		TwRemoveVar(glApp->mainMenuBar, "motor angle max");
//		TwRemoveVar(glApp->mainMenuBar, "bracket connector angle");
	}

	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_SPHERE_EE)
	{
		//		TwRemoveVar(glApp->mainMenuBar, "sphere radius");
	}
	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_WHEEL_EE)
	{
		//		TwRemoveVar(glApp->mainMenuBar, "wheel radius");
	}
}

void ModularDesignWindow::updateLivingBracket(){
    PressedModifier mod = getPressedModifier(glApp->glfwWindow);

	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_MOTOR)
	{
		if (rmcMirrorMap.count(selectedRobot->selectedRMC))
		{
			RMC* mirrorRMC = rmcMirrorMap[selectedRobot->selectedRMC];
			dynamic_cast<LivingMotor*>(mirrorRMC)->syncSymmParameters(dynamic_cast<LivingMotor*>(selectedRobot->selectedRMC));
		}

		if (motorRotAngle != 0)
		{
			V3D axis = selectedRobot->selectedRMC->state.getWorldCoordinates(selectedRobot->selectedRMC->motorAxis).normalized();
			Quaternion motorRotQ = getRotationQuaternion(RAD(motorRotAngle), axis);
			selectedRobot->selectedRMC->state.orientation = motorRotQ * motorStartOrient;
			propagateOrientToMirrorRMC(selectedRobot->selectedRMC);
			
//			if (GetAsyncKeyState(VK_LMENU) < 0)
            if(mod == PressedModifier::LEFT_ALT)
			{
				updateParentConnector(selectedRobot->selectedRMC);
				if (rmcMirrorMap.count(selectedRobot->selectedRMC))
					updateParentConnector(rmcMirrorMap[selectedRobot->selectedRMC]);
				for (auto robot : rmcRobots)
					robot->fixJointConstraints();
			}
		}
	}

	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_SPHERE_EE)
	{
		if (rmcMirrorMap.count(selectedRobot->selectedRMC))
		{
			RMC* mirrorRMC = rmcMirrorMap[selectedRobot->selectedRMC];
			dynamic_cast<LivingSphereEE*>(mirrorRMC)->syncSymmParameters(dynamic_cast<LivingSphereEE*>(selectedRobot->selectedRMC));
		}
	}

	if (selectedRobot && selectedRobot->selectedRMC && selectedRobot->selectedRMC->type == LIVING_WHEEL_EE)
	{
		if (rmcMirrorMap.count(selectedRobot->selectedRMC))
		{
			RMC* mirrorRMC = rmcMirrorMap[selectedRobot->selectedRMC];
			dynamic_cast<LivingWheelEE*>(mirrorRMC)->syncSymmParameters(dynamic_cast<LivingWheelEE*>(selectedRobot->selectedRMC));
		}
	}

	for (auto robot : rmcRobots)
	{
		robot->updateAllLivingMotor();
		robot->fixJointConstraints();
	}
}

bool ModularDesignWindow::isSelectedRMCMovable(){
	return selectedRobot->selectedRMC->isMovable();
}

void ModularDesignWindow::matchDesignWithRobot(Robot* tRobot, ReducedRobotState* initialRobotState){
	bool incompatible = false;

	ReducedRobotState currentRobotState(tRobot);
	tRobot->setState(initialRobotState);

	for (auto joint : tRobot->jointList){
		P3D wjPos = joint->getWorldPosition();
		MappingInfo& mappingInfo = joint->mappingInfo;
		if (mappingInfo.index1 >= 0 && mappingInfo.index1 < (int)rmcRobots.size()){
			RMCRobot* rmcRobot = rmcRobots[mappingInfo.index1];
			if (mappingInfo.index2 >= 0 && mappingInfo.index2 < rmcRobot->getRMCCount())
			{
				RMC* rmc = rmcRobot->getRMC(mappingInfo.index2);
				rmc->state.position = wjPos;
			}
			else {
				incompatible = true;
				break;
			}
		}
		else {
			incompatible = true;
			break;
		}
	}

	for (int i = 0; i < tRobot->getRigidBodyCount(); i++){
		RigidBody* rb = tRobot->getRigidBody(i);
		MappingInfo& mappingInfo = rb->mappingInfo;
		if (rb->rbProperties.endEffectorPoints.empty()) continue;

		P3D EEPos = rb->getWorldCoordinates(rb->rbProperties.endEffectorPoints[0].coords);

		if (mappingInfo.index1 >= 0 && mappingInfo.index1 < (int)rmcRobots.size())
		{
			RMCRobot* rmcRobot = rmcRobots[mappingInfo.index1];
			if (mappingInfo.index2 >= 0 && mappingInfo.index2 < rmcRobot->getRMCCount())
			{
				RMC* rmc = rmcRobot->getRMC(mappingInfo.index2);
				if (rmc->type == EE_RMC){
					rmc->state.position = EEPos + rmc->state.orientation.rotate(-rmc->rbProperties.endEffectorPoints[0].coords);
				}
				else {
					incompatible = true;
					break;
				}
			}
			else {
				incompatible = true;
				break;
			}
		}
		else {
			incompatible = true;
			break;
		}
	}

	if (incompatible){
		tRobot->setState(&currentRobotState);
		Logger::consolePrint("This .rbs is not compatible with current design!");
		return;
	}

	// move the plates with motors
	for (auto rmcRobot : rmcRobots)
	{
		rmcRobot->fixPlateStateByMotor();
	}

	updateLivingBracket();

	createBodyMesh3D();

	tRobot->setState(&currentRobotState);
}




void ModularDesignWindow::transferMeshes(Robot* tRobot, ReducedRobotState* initialRobotState){
	// adjust design
	matchDesignWithRobot(tRobot, initialRobotState);

	for (auto tmpBot : rmcRobots)
		tmpBot->resetAllMotorAngles();

	//TODO: there must be a more direct way than saving the file to disk and reloading...

	for (auto tmpBot : rmcRobots)
		tmpBot->restoreAllMotorAngles();

/*
	saveToRBSFile("../out/tmpRobot.rbs", tRobot, mergeMeshes);

	AbstractRBEngine* rbEngine = new ODERBEngine();
	rbEngine->loadRBsFromFile("../out/tmpRobot.rbs");

	Robot* robot = new Robot(rbEngine->rbs[0]);
	matchDesignWithRobot(robot, initialRobotState);

	// transfer meshes
	for (int i = 0; i < robot->getRigidBodyCount(); i++){
		RigidBody* rb = robot->getRigidBody(i);
		RigidBody* t_rb = tRobot->getRigidBody(i);

		//TODO: hmmm, who creates and deletes all these meshes?!?
		t_rb->meshes = rb->meshes;
		t_rb->meshTransformations = rb->meshTransformations;
		t_rb->carveMeshes = rb->carveMeshes;
		t_rb->meshDescriptions = rb->meshDescriptions;
	}

	delete robot;
	delete rbEngine;
*/
}

void ModularDesignWindow::buildRMCMirrorMap()
{
	rmcMirrorMap.clear();
	bodyFpMirrorMap.clear();

	for (auto rmcRobotA : rmcRobots)
	{
		for (int i = 0; i < rmcRobotA->getRMCCount(); i++)
		{
			RMC* rmcA = rmcRobotA->getRMC(i);
			if (!rmcA->isMovable()) continue;
			P3D posA = rmcA->state.position;

			for (auto rmcRobotB : rmcRobots)
			{
				for (int j = 0; j < rmcRobotB->getRMCCount(); j++)
				{
					RMC* rmcB = rmcRobotB->getRMC(j);
					if (rmcA == rmcB || !rmcB->isMovable()) continue;
					
					P3D posB = rmcB->state.position;
					posB[0] *= -1;
					if ((posA - posB).norm() < 1e-4)
					{
						rmcMirrorMap[rmcA] = rmcB;
					}
				}
			}
		}
	}

	for (auto& fpA : bodyFeaturePts)
	{
		P3D posA = fpA.coords;
		for (auto& fpB : bodyFeaturePts)
		{
			if (&fpA == &fpB) continue;

			P3D posB = fpB.coords;
			posB[0] *= -1;
			if ((posA - posB).norm() < 1e-4)
			{
				bodyFpMirrorMap[&fpA] = &fpB;
			}
		}
	}
	// Logger::print("mirror map size: %d\n", rmcMirrorMap.size());
}

void ModularDesignWindow::makeSelectedRMCSymmtry()
{
	if (selectedRobot->selectedRMC->type == PLATE_RMC && hightlightedRobot->highlightedRMC->type == PLATE_RMC)
	{
		mirrorMap[selectedRobot] = hightlightedRobot;
		mirrorMap[hightlightedRobot] = selectedRobot;
		rmcMirrorMap[selectedRobot->selectedRMC] = hightlightedRobot->highlightedRMC;
		rmcMirrorMap[hightlightedRobot->highlightedRMC] = selectedRobot->selectedRMC;
		propagatePosToMirrorRMC(selectedRobot->selectedRMC);
		propagateOrientToMirrorRMC(selectedRobot->selectedRMC);
		hightlightedRobot->fixJointConstraints();
		createBodyMesh3D();
	}
	else if (selectedRobot->selectedRMC->isMovable() && hightlightedRobot->highlightedRMC->isMovable())
	{
		rmcMirrorMap[selectedRobot->selectedRMC] = hightlightedRobot->highlightedRMC;
		rmcMirrorMap[hightlightedRobot->highlightedRMC] = selectedRobot->selectedRMC;
		propagatePosToMirrorRMC(selectedRobot->selectedRMC);
		propagateOrientToMirrorRMC(selectedRobot->selectedRMC);
		updateLivingBracket();
	}

	
}

void ModularDesignWindow::propagatePosToMirrorRMC(RMC* rmc)
{
	P3D pos = rmc->state.position;
	if (rmcMirrorMap.count(rmc))
	{
		pos[0] *= -1;
		RMC* mirrorRMC = rmcMirrorMap[rmc];
		mirrorRMC->state.position = pos;
	}
}

void ModularDesignWindow::propagateOrientToMirrorRMC(RMC* rmc)
{
	Quaternion q = rmc->state.orientation;
	if (rmcMirrorMap.count(rmc))
	{
		RMC* mirrorRMC = rmcMirrorMap[rmc];
		double alpha, beta, gamma;
		computeEulerAnglesFromQuaternion(q, V3D(1, 0, 0), V3D(0, 0, 1), V3D(0, 1, 0), alpha, beta, gamma);
		mirrorRMC->state.orientation = getRotationQuaternion(-gamma, V3D(0, 1, 0)) * getRotationQuaternion(-beta, V3D(0, 0, 1)) * getRotationQuaternion(alpha, V3D(1, 0, 0));
	}
}

void ModularDesignWindow::updateParentConnector(RMC* rmc)
{
	if (rmc->getParentJoint())
	{
		RMC* parentRMC = rmc->getParentJoint()->getParent();
		if (parentRMC->type == LIVING_CONNECTOR)
		{
			((LivingConnector*)parentRMC)->update();
		}
	}
}

void ModularDesignWindow::pickBodyFeaturePts(Ray& ray)
{
	double closestDist = 1e10;

	for (auto& fp : bodyFeaturePts)
	{
		double dist = ray.getDistanceToPoint(fp.coords);
		if (dist < fp.featureSize && dist < closestDist)
		{
			closestDist = dist;
			highlightedFP = &fp;
		}
	}
}

void ModularDesignWindow::makeSelectedFpSymmtry()
{
	bodyFpMirrorMap[selectedFP] = highlightedFP;
	bodyFpMirrorMap[highlightedFP] = selectedFP;

	propagatePosToMirrorFp(selectedFP);
	createBodyMesh3D();
}

void ModularDesignWindow::propagatePosToMirrorFp(RBFeaturePoint* fp)
{
	if (bodyFpMirrorMap.count(fp))
	{
		P3D pos = fp->coords;
		pos[0] *= -1;
		RBFeaturePoint* mirrorFp = bodyFpMirrorMap[fp];
		mirrorFp->coords = pos;
		mirrorFp->featureSize = fp->featureSize;
    }
}

ModularDesignWindow::PressedModifier ModularDesignWindow::getPressedModifier(GLFWwindow *window){
	if (glfwGetKey(window, GLFW_KEY_LEFT_ALT)) return PressedModifier::LEFT_ALT;
	if (glfwGetKey(window, GLFW_KEY_RIGHT_ALT)) return PressedModifier::RIGHT_ALT;
	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL)) return PressedModifier::LEFT_CTRL;

    return PressedModifier::NONE;
}
