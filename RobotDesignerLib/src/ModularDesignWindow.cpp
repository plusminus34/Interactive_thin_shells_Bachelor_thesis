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
#include <RobotDesignerLib/LivingEE.h>

//TODO: parts of this code are still messy... although functional, it would benefit from some cleanup...

/*
- add eyes...
*/

/* 
pick an RMCRobot by clicking on it, translation and rotation widgets will show on its root.
	And its picked RMC will be painted in orange. (Root is painted in green)

Once you select an RMCRobot:
key 'Q': clone the RMCRobot
key 'V': save the picked RMCRobot to ../out/tmpRMCRobot.mrb
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
	((GLTrackingCamera*)camera)->camDistance = -1.5;
	((GLTrackingCamera*)camera)->rotAboutRightAxis = 0.3;
	((GLTrackingCamera*)camera)->rotAboutUpAxis = 0.3;

	componentLibrary = new GLWindowContainer(2, 4, x, (int)(h * 3.0 / 4), (int)(w), (int)(h /4.0));

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

	bodyMesh = new GLMesh();
	bodyMesh->path = robotMeshDir + "BodyMesh.obj";
	
	string mat = "../data/textures/matcap/black2.bmp";
	bodyMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	bodyMaterial.setTextureParam(mat.c_str(), GLContentManager::getTexture(mat.c_str()));
	bodyMesh->setMaterial(bodyMaterial);

	sphereMesh = new GLMesh();
	sphereMesh->addSphere(P3D(), 1, 12);

	bgColorR = bgColorG = bgColorB = bgColorA = 1.0;

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
	pushViewportTransformation();

	for (uint i = 0; i < rmcRobots.size(); i++)
		rmcRobots[i]->highlightedRMC = NULL;
	hightlightedRobot = NULL;
	highlightedFP = NULL;

	bool tWidgetActive = tWidget->onMouseMoveEvent(xPos, yPos) == true;
	bool rWidgetActive = rWidget->onMouseMoveEvent(xPos, yPos) == true;

	if ((tWidgetActive || rWidgetActive) && dragging) {
		Quaternion q = rWidget->getOrientation();

		if (glfwGetKey(glApp->glfwWindow, GLFW_KEY_LEFT_CONTROL)) {
			V3D axis = q.v; axis.toUnit();
			double rotAngle = q.getRotationAngle(axis);
			if (rotAngle < 0) {
				rotAngle = -rotAngle;
				axis = -axis;
			}

			if (rotAngle < RAD(30)) {
				popViewportTransformation();
				return true;
			}
			else {
				q = getRotationQuaternion(RAD(30), axis);
			}
		}

		// move selected body feature points
		if (selectedFP && tWidgetActive) {
			selectedFP->coords = tWidget->pos;
			propagatePosToMirrorFp(selectedFP);
			createBodyMesh3D();
		}
	
		if (selectedRobot && isSelectedRMCMovable()){
			if (tWidgetActive)
				selectedRobot->selectedRMC->state.position = rWidget->pos = tWidget->pos;
			else {
				selectedRobot->selectedRMC->state.orientation = q * selectedRobot->selectedRMC->state.orientation;
				rWidget->setOrientation(Quaternion());
			}

			propagatePosToMirrorRMC(selectedRobot->selectedRMC);
			propagateOrientToMirrorRMC(selectedRobot->selectedRMC);

			if (glfwGetKey(glApp->glfwWindow, GLFW_KEY_LEFT_ALT)) {
				updateParentConnector(selectedRobot->selectedRMC);
				if (rmcMirrorMap.count(selectedRobot->selectedRMC))
					updateParentConnector(rmcMirrorMap[selectedRobot->selectedRMC]);
			}

			for (auto robot : rmcRobots) {
				if (!glfwGetKey(glApp->glfwWindow, GLFW_KEY_LEFT_ALT))
					robot->updateComponents();
				robot->fixConstraints();
				robot->updateComponents();
			}
			createBodyMesh3D();
		}

		if (pickedGuidingMesh)
			guidingMeshPos = rWidget->pos = tWidget->pos;

		popViewportTransformation();
		return true;
	}

	Ray mouseRay = camera->getRayFromScreenCoords(xPos, yPos);
	popViewportTransformation();

	bodyMeshSelected = false;
//	Logger::consolePrint("checking for body intersections\n");
	if (bodyMesh && bodyMesh->getDistanceToRayOriginIfHit(mouseRay)) {
//		Logger::consolePrint("and a hit...\n");
		bodyMeshSelected = true;
	}

	if (windowSelectedRobot){
		if (!possibleConnections.empty()){
			double closestDist;
			P3D closestPoint;
			PossibleConnection* closestConnection = getClosestConnnection(mouseRay, possibleConnections, closestPoint, closestDist);
			windowSelectedRobot->getRoot()->state.orientation = closestConnection->orientation;
			windowSelectedRobot->getRoot()->state.position = closestPoint;
			windowSelectedRobot->fixConstraints();
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
	if (!dragging) {
		pickBodyFeaturePts(mouseRay);
		if (highlightedFP) return true;
	}

	// only triggered when dragging
	if (dragging){
		snappable = false;
		if (selectedRobot && selectedRobot->selectedPin && possibleConnections.size() > 0)
		{
			double closestDist;
			P3D closestPoint;
			PossibleConnection* closestConnection = getClosestConnnection(mouseRay, possibleConnections, closestPoint, closestDist);
			selectedRobot->getRoot()->state.orientation = closestConnection->orientation;
			selectedRobot->getRoot()->state.position = closestPoint;
			selectedRobot->fixConstraints();
			snappable = closestDist < SNAP_THRESHOLD;
				
			return true;
		}
	}
	else {
		for (uint i = 0; i < rmcRobots.size(); i++)	{
			if (rmcRobots[i]->getRoot()->type == PLATE_RMC) continue;
			
			rmcRobots[i]->highlightedRMC = NULL;
			if (rmcRobots[i]->pickPin(mouseRay))
			{
				hightlightedRobot = rmcRobots[i];
				return true;
			}
		}

		double closestDist = 1e10;
		for (uint i = 0; i < rmcRobots.size(); i++)	{
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
	
	pushViewportTransformation();
	Ray mouseRay = camera->getRayFromScreenCoords(xPos, yPos);
	popViewportTransformation();

	// picking a new component from the RMC shelf...
	if (componentLibrary->onMouseButtonEvent(button, action, mods, xPos, yPos)) {
		if (action == GLFW_PRESS){

			if (selectedRobot && selectedRobot->selectedRMC){
				selectedRobot->selectedRMC = NULL;
				selectedRobot = NULL;
				rWidget->visible = tWidget->visible = false;
			}

			int selectedRMC = -1;
			for (uint i = 0; i < componentLibrary->subWindows.size(); i++){
				if (componentLibrary->subWindows[i]->isSelected()) {
					selectedRMC = i;
					break;
				}
			}

			if (selectedRMC >= 0){
				delete windowSelectedRobot;
				windowSelectedRobot = new RMCRobot(rmcWarehouse[selectedRMC]->clone(), transformationMap);
				noMirror = (bool)(mods & GLFW_MOD_CONTROL);
				
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
	else //instantiate the RMC...
		if (windowSelectedRobot) {
		
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

			pushViewportTransformation();
			bool res = rWidget->onMouseMoveEvent(xPos, yPos);
			popViewportTransformation();
			if (res) return true;

			pushViewportTransformation();
			res = tWidget->onMouseMoveEvent(xPos, yPos);
			popViewportTransformation();
			if (res) return	true;

			tWidget->visible = rWidget->visible = false;
			pickedGuidingMesh = false;

			if (((mods & GLFW_MOD_SHIFT) > 0) && hightlightedRobot && hightlightedRobot->highlightedRMC
				&& selectedRobot && selectedRobot->selectedRMC){
				makeSelectedRMCSymmetric();
			}

			if (((mods & GLFW_MOD_SHIFT) > 0) && highlightedFP && selectedFP){
				makeSelectedFPSymmtric();
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

				if (rWidget->visible || tWidget->visible){
					rWidget->pos = tWidget->pos = isSelectedRMCMovable() ? selectedRobot->selectedRMC->state.position : selectedRobot->root->state.position;
					rWidget->setOrientation(Quaternion());
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
				if (rWidget->visible || tWidget->visible){
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

void ModularDesignWindow::makeRMCsSymmetricRecursive(RMC* originalRMC, RMC* mirroredRMC) {
	rmcMirrorMap[originalRMC] = mirroredRMC;
	rmcMirrorMap[mirroredRMC] = originalRMC;
	propagatePosToMirrorRMC(originalRMC);
	propagateOrientToMirrorRMC(originalRMC);
	mirroredRMC->syncSymmParameters(originalRMC);

	if (originalRMC->getChildJointCount() == mirroredRMC->getChildJointCount())
		for (int i = 0; i < originalRMC->getChildJointCount(); i++)
			makeRMCsSymmetricRecursive(originalRMC->getChildJoint(i)->getChild(), mirroredRMC->getChildJoint(i)->getChild());
}

bool ModularDesignWindow::onKeyEvent(int key, int action, int mods) {
	if (componentLibrary->onKeyEvent(key, action, mods)) return true;

	if (key == GLFW_KEY_E && action == GLFW_PRESS){
		Logger::consolePrint("Saving robot to \'../out/tmpRobot.rbs\'...\n");
		saveToRBSFile("../out/tmpRobot.rbs");
	}

	// clone the picked robot
	if (key == GLFW_KEY_Q && action == GLFW_PRESS){
		if (selectedRobot && selectedRobot->selectedRMC){
			if (selectedRobot->selectedRMC->type == PLATE_RMC && mirrorMap.count(selectedRobot) && mirrorMap[selectedRobot]->jointList.size() == 0 && selectedRobot->selectedRMC->getChildJointCount() == 1) {
				Logger::consolePrint("Transplanting selected robot...\n");
				//we need to transplant the duplicated robot
				RMCRobot* parent = mirrorMap[selectedRobot];
				RMCRobot* newRobot = selectedRobot->cloneSubTree(selectedRobot->selectedRMC->getChildJoint(0)->getChild());
				int childId = selectedRobot->selectedRMC->getChildJoint(0)->childPin->id;
				int parentId = selectedRobot->selectedRMC->getChildJoint(0)->parentPin->id;
				parent->connectRMCRobot(newRobot, &parent->root->pins[parentId], &newRobot->root->pins[childId]);
				parent->fixConstraints();

				makeRMCsSymmetricRecursive(selectedRobot->selectedRMC->getChildJoint(0)->getChild(), parent->root->getChildJoint(0)->getChild());
			}
			else {
				Logger::consolePrint("Cloning selected robot...\n");

				RMCRobot* newRobot = selectedRobot->cloneSubTree(selectedRobot->selectedRMC);
				rmcRobots.push_back(newRobot);

				if (selectedRobot->selectedRMC->type == PLATE_RMC)
					newRobot->root->state.position = selectedRobot->root->state.position + V3D(0, 0, -0.05);
				else
					newRobot->root->state.position = P3D(0, 0.05, 0);
				newRobot->fixConstraints();

				if (selectedRobot->selectedRMC->type == PLATE_RMC && mirrorMap.count(selectedRobot)){
					RMCRobot* mirrorSelRobot = mirrorMap[selectedRobot];
					RMCRobot* mirrorNewRobot = mirrorSelRobot->cloneSubTree(mirrorSelRobot->root);
					rmcRobots.push_back(mirrorNewRobot);

					mirrorNewRobot->root->state.position = mirrorSelRobot->root->state.position + V3D(0, 0, -0.05);
					mirrorNewRobot->fixConstraints();

					mirrorMap[newRobot] = mirrorNewRobot;
					mirrorMap[mirrorNewRobot] = newRobot;
					rmcMirrorMap[newRobot->root] = mirrorNewRobot->root;
					rmcMirrorMap[mirrorNewRobot->root] = newRobot->root;
				}
			}

			if (selectedRobot->selectedRMC->type == PLATE_RMC)
				createBodyMesh3D();
		}
	}

	// delete the sub tree structure from the selected RMC
	if (key == GLFW_KEY_D && action == GLFW_PRESS){
		Logger::consolePrint("Deleting selected robot...\n");
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

		if (selectedRobot && selectedRobot->selectedRMC){
			RMC* rmcToDelete = selectedRobot->selectedRMC;
			if (rmcToDelete->getParentJoint() && rmcToDelete->getParentJoint()->getParent() && rmcToDelete->getParentJoint()->getParent()->type == LIVING_CONNECTOR)
				rmcToDelete = rmcToDelete->getParentJoint()->getParent();

			if (rmcToDelete == selectedRobot->root) {
				removeRMCRobot(selectedRobot);
				if (mirrorMap.count(selectedRobot))
				{
					RMCRobot* mirrorRobot = mirrorMap[selectedRobot];
					removeRMCRobot(mirrorRobot);
					mirrorMap.erase(selectedRobot);
					mirrorMap.erase(mirrorRobot);
				}
				if (rmcToDelete->type == PLATE_RMC)
					createBodyMesh3D();

				rWidget->visible = tWidget->visible = false;
				selectedRobot = NULL;
			}
			else {
				if (isSelectedRMCMovable()) {
					rWidget->visible = tWidget->visible = false;
				}
				selectedRobot->deleteSubTree(rmcToDelete->getParentJoint());
				selectedRobot->selectedRMC = NULL;
			}
			buildRMCMirrorMap();
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

	if (selectedRobot && selectedRobot->selectedRMC && action == GLFW_PRESS) {
		selectedRobot->selectedRMC->processInputKeyPress(key);
		selectedRobot->selectedRMC->update();

		if (rmcMirrorMap.count(selectedRobot->selectedRMC)){
			rmcMirrorMap[selectedRobot->selectedRMC]->syncSymmParameters(selectedRobot->selectedRMC);
			rmcMirrorMap[selectedRobot->selectedRMC]->update();
		}

		//this will propagate changes as needed...
		for (auto robot : rmcRobots){
			if (selectedRobot->selectedRMC->type == MOTOR_RMC)
				robot->fixConstraints();
			else
				robot->updateComponents();
		}
	}

	if (key == GLFW_KEY_MINUS && action == GLFW_PRESS) {
		if (selectedFP){
			selectedFP->featureSize = max(0.005, selectedFP->featureSize - 0.005);
			createBodyMesh3D();
		}
	}

	if (key == GLFW_KEY_EQUAL && action == GLFW_PRESS) {
		if (selectedFP) {
			selectedFP->featureSize = selectedFP->featureSize + 0.005;
			createBodyMesh3D();
		}
	}

	if (key == GLFW_KEY_Z && action == GLFW_PRESS){
		bodyFeaturePts.push_back(RBFeaturePoint(P3D(0, 0, 0), 0.02));
		createBodyMesh3D();
	}

	if (key == GLFW_KEY_X && action == GLFW_PRESS){
		tWidget->visible = !tWidget->visible;
		if (tWidget->visible && selectedRobot && selectedRobot->selectedRMC){
			rWidget->pos = tWidget->pos = isSelectedRMCMovable() ? selectedRobot->selectedRMC->state.position : selectedRobot->root->state.position;
			rWidget->setOrientation(Quaternion());
		}
		if (tWidget->visible && pickedGuidingMesh){
			rWidget->pos = tWidget->pos = guidingMeshPos;
			rWidget->setOrientation(guidingMeshRot);
		}
	}

	if (key == GLFW_KEY_C && action == GLFW_PRESS) {
		rWidget->visible = !rWidget->visible;
		if (rWidget->visible && selectedRobot && selectedRobot->selectedRMC) {
			rWidget->pos = tWidget->pos = isSelectedRMCMovable() ? selectedRobot->selectedRMC->state.position : selectedRobot->root->state.position;
			rWidget->setOrientation(Quaternion());
		}
		if (rWidget->visible && pickedGuidingMesh) {
			rWidget->pos = tWidget->pos = guidingMeshPos;
			rWidget->setOrientation(guidingMeshRot);
		}
	}

	if (key == GLFW_KEY_V && action == GLFW_PRESS){
		createBodyMesh3D();
	}

	if (key == GLFW_KEY_Z && action == GLFW_PRESS){
		if (selectedRobot && selectedRobot->selectedRMC && mirrorMap.count(selectedRobot) == 0){
			selectedRobot->getRoot()->state.position[0] = 0;
			selectedRobot->fixConstraints();
			createBodyMesh3D();
			rWidget->pos = tWidget->pos = selectedRobot->getRoot()->state.position;
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
/* - these should already be updated. If they're not, nothaving this in will hopefully help with debugging...
	{
		for (auto robot : rmcRobots) {
			robot->updateAllComponents();
			robot->fixJointConstraints();
		}
	}
*/

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
			selectedRobot->fixConstraints();
			selectedRobot->draw(SHOW_MESH, Vector4d(1, 0, 0, 0.4), Vector4d(1, 0, 0, 0.4), Vector4d(1, 0, 0, 0.4));
		}

		selectedRobot->root->state = origState;
		selectedRobot->fixConstraints();
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
			windowSelectedRobot->fixConstraints();
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
//	for (auto robot : rmcRobots) {
//		robot->fixConstraints();
//		robot->updateComponents();
//	}

	glEnable(GL_LIGHTING);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
 
	glEnable(GL_LIGHTING);


//	drawDesignEnvironment();

	drawRMCRobot();
	
	drawConnectionPreview();
	drawWindowRMCConnectionPreview();

	drawBodyFeaturePts();

	if (bodyMeshSelected == false && highlightedFP == nullptr && selectedFP == nullptr)
		bodyMesh->setMaterial(bodyMaterial);
	else {
		GLShaderMaterial colorMat;
		colorMat.setColor(1.0, 1.0, 1.0, 0.7);
		bodyMesh->setMaterial(colorMat);
	}


	if (bodyMesh)
		bodyMesh->drawMesh();

	drawGuildingMesh();

	drawBodyPlane();

	//draw a little blob so we know where the origin is...
	glColor4d(0.8, 0.8, 0.8, 0.2);
	drawSphere(P3D(), 0.005, 12);
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

void ModularDesignWindow::loadConfig(const char* fName){
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

		RMC* rmc = NULL;

		if (strcmp(keyword, "RMC") == 0)
			rmc = new RMC();
		else if (strncmp(keyword, "Motor", strlen("Motor")) == 0)
			rmc = new Motor_RMC(line + strlen(keyword));
		else if (strcmp(keyword, "LivingConnector") == 0)
			rmc = new LivingConnector();
		else if (strcmp(keyword, "SphereEE") == 0)
			rmc = new SphereEE_RMC();
		else if (strcmp(keyword, "ConnectorHUB") == 0)
			rmc = new ConnectorHUB_RMC();
		else if (strcmp(keyword, "WheelEE") == 0)
			rmc = new WheelEE_RMC(line + strlen(keyword));
		else if (strcmp(keyword, "RobotMeshDir") == 0)		{
			char content[200];
			int num = sscanf(line + strlen(keyword), "%s", content);
			robotMeshDir = content;
		} else if (strcmp(keyword, "TransformationMap") == 0)
			loadTransformationMap(fp);

		if (rmc) {
			rmcWarehouse.push_back(rmc);
			rmcWarehouse.back()->loadFromFile(fp);
			rmcNameMap[rmcWarehouse.back()->getName()] = rmc;

			for (uint i = 0; i < rmc->pins.size(); i++)
				rmcPinNameMap[rmc->pins[i].name] = &rmc->pins[i];
		}
	}

	fclose(fp);
}

void ModularDesignWindow::loadTransformationMap(FILE* fp){
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

bool ModularDesignWindow::previewConnectRMCRobot(RMCPin* parentPin, RMCPin* childPin, RMCRobot* childRobot, bool rotationOnly){
	string key = parentPin->name + '+' + childPin->name;
	Transformation trans;
	
	if (transformationMap.count(key)) {
		Transformation parentTrans = parentPin->transformation;
		Transformation childTrans = childPin->transformation;
		trans = parentTrans * transformationMap[key][0] * childTrans.inverse();
	}
	else return false;

	RBState& parentState = parentPin->rmc->state;
	RBState& childState = childPin->rmc->state;
	Matrix3x3 rot = parentState.orientation.getRotationMatrix() * trans.R;
	childState.orientation.setRotationFrom(rot);
	if (!rotationOnly)
		childState.position = parentState.position + parentState.orientation.rotate(trans.T);

	childRobot->fixConstraints();

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
			if (rmc->type == MOTOR_RMC || rmc->type == EE_RMC)
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
	robot->fixConstraints();

	robot->saveToRBSFile(fName, robotMeshDir, templateRobot, freezeRobotRoot);

	//saverbs should also save an rs file, right after having restored the joint angles... I think it's cleaner that way...
//	RobotState tmpState = robot->getReducedRobotState(rbRobot);
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

	ConvexHull3D::computeConvexHullFromSetOfPoints(points, bodyMesh, false);

	bodyMesh->calBoundingBox();
	bodyMesh->computeNormals();
}

bool ModularDesignWindow::isSelectedRMCMovable(){
	return selectedRobot->selectedRMC->isMovable();
}

//TODO: This method no longer works, but it is very useful, so it should get fixed at some point!!!
void ModularDesignWindow::matchDesignWithRobot(Robot* tRobot, RobotState* initialRobotState){
	bool incompatible = false;

	RobotState currentRobotState(tRobot);
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
	for (auto rmcRobot : rmcRobots)	{
		rmcRobot->fixPlateStateByMotor();
	}

	{
		if (rmcMirrorMap.count(selectedRobot->selectedRMC))
			rmcMirrorMap[selectedRobot->selectedRMC]->syncSymmParameters(selectedRobot->selectedRMC);
		for (auto robot : rmcRobots) {
			robot->updateComponents();
			robot->fixConstraints();
		}
	}

	createBodyMesh3D();

	tRobot->setState(&currentRobotState);
}




void ModularDesignWindow::transferMeshes(Robot* tRobot, RobotState* initialRobotState){
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

void ModularDesignWindow::makeSelectedRMCSymmetric()
{
	if (selectedRobot->selectedRMC->type == PLATE_RMC && hightlightedRobot->highlightedRMC->type == PLATE_RMC)
	{
		mirrorMap[selectedRobot] = hightlightedRobot;
		mirrorMap[hightlightedRobot] = selectedRobot;
		rmcMirrorMap[selectedRobot->selectedRMC] = hightlightedRobot->highlightedRMC;
		rmcMirrorMap[hightlightedRobot->highlightedRMC] = selectedRobot->selectedRMC;
		propagatePosToMirrorRMC(selectedRobot->selectedRMC);
		propagateOrientToMirrorRMC(selectedRobot->selectedRMC);
		hightlightedRobot->fixConstraints();
		createBodyMesh3D();
		Logger::consolePrint("Body plates are now symmetric!\n");
	}
	else if (selectedRobot->selectedRMC->isMovable() && hightlightedRobot->highlightedRMC->isMovable())
	{
		rmcMirrorMap[selectedRobot->selectedRMC] = hightlightedRobot->highlightedRMC;
		rmcMirrorMap[hightlightedRobot->highlightedRMC] = selectedRobot->selectedRMC;
		propagatePosToMirrorRMC(selectedRobot->selectedRMC);
		propagateOrientToMirrorRMC(selectedRobot->selectedRMC);

		rmcMirrorMap[selectedRobot->selectedRMC]->syncSymmParameters(selectedRobot->selectedRMC);
		for (auto robot : rmcRobots) {
			robot->updateComponents();
			robot->fixConstraints();
		}
		Logger::consolePrint("Components are now symmetric!\n");
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

void ModularDesignWindow::propagateOrientToMirrorRMC(RMC* rmc){
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

void ModularDesignWindow::makeSelectedFPSymmtric()
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
