#include <RobotDesignerLib/FootFallPatternViewer.h>

#include <GUILib/GLContentManager.h>
#include <GUILib/FreeType.h>
#include <RobotDesignerLib/FootFallPattern.h>
#include <GUILib/GLUtils.h>
#include <stdlib.h>
#include <math.h>
#include <Utils/Logger.h>
#include <GUILib/GLApplication.h>

// TODO: find where min and max where defined!
#undef min
#undef max

FootFallPatternViewer::FootFallPatternViewer(int posX, int posY, int sizeX, int sizeY) : GLWindow2D(posX,posY, sizeX, sizeY){
	ffp = NULL;

	cursorPosition = 0;

	labelsStart = 0.05;
	boxStart = labelsStart + 0.15;
	boxLength = 0.75;

	selectedLimbIndex = -1;
	selectedColIndex = -1;
	oldCol = -1;

	bgColorR = bgColorG = bgColorB = bgColorA = 1.0;

}

FootFallPatternViewer::~FootFallPatternViewer(void){
}

void FootFallPatternViewer::draw(){
	if (ffp == NULL)
		return;
	preDraw();

	int numFeet = (int)ffp->stepPatterns.size();
	int numSections = numFeet;
	
	double boxWidth = (0.8) / numSections;

	glColor3d(0.0,0.0,0.0);

	//now draw the labels...
	for (uint i=0;i<ffp->stepPatterns.size();i++){
		FreeTypeFont* font = GLContentManager::getFont("../data/fonts/arial.ttf 14");
		glColor3d(1 - bgColorR, 1 - bgColorG, 1 - bgColorB);
		font->print(labelsStart*viewportWidth, (0.1 + i*boxWidth + boxWidth / 3.0)*viewportHeight, ffp->stepPatterns[i].limb->name);
	}

	//draw all the regions where the legs are supposed to be in swing mode
	int start = 0;
	int end = ffp->strideSamplePoints;
	for (int j=start;j<end;j++){
		double intervalStart = ((double)j - start) / (end - start);
		double intervalEnd = ((double)j+1 - start) / (end - start);

		//dark blue: glColor3d(70/255.0, 80/255.0, 140/255.0);
		//light blue: glColor3d(130/255.0,170/255.0,210/255.0);
		//orange: glColor3d(255/255.0, 158/255.0, 1/255.0);

		for (uint k=0;k<ffp->stepPatterns.size(); k++){
			if (ffp->isInSwing(ffp->stepPatterns[k].limb, j)){
				glBegin(GL_QUADS);
					if (k == selectedLimbIndex)
						if (ffp->isStart(ffp->stepPatterns[k].limb, j) && j == selectedColIndex)
							glColor3d(255/255.0, 100/255.0, 0/255.0);
						else
							glColor3d(255/255.0, 180/255.0, 25/255.0);
					else
						glColor3d(238/255.0,59/255.0,59/255.0);

					glVertex3d(boxStart + intervalStart*boxLength, 0.1 + (k+1)*boxWidth, 0);
					glVertex3d(boxStart + intervalStart*boxLength, 0.1 + k*boxWidth, 0);
					
					if (k == selectedLimbIndex)
						if (ffp->isEnd(ffp->stepPatterns[k].limb, j) && j == selectedColIndex)
							glColor3d(255/255.0, 100/255.0, 0/255.0);
						else
							glColor3d(255/255.0, 180/255.0, 25/255.0);
					else
						glColor3d(238/255.0,59/255.0,59/255.0);

					glVertex3d(boxStart + intervalEnd*boxLength, 0.1 + k*boxWidth, 0);
					glVertex3d(boxStart + intervalEnd*boxLength, 0.1 + (k+1)*boxWidth, 0);
					
				glEnd();
			}else{
				if (intervalStart > 0 && j!=0 && j!=ffp->strideSamplePoints) 
					glColor3d(0.95, 0.95, 0.95);
				else 
					glColor3d(0.0,0.0,0.0);
				glBegin(GL_LINES);
					glVertex3d(boxStart + intervalStart*boxLength, 0.1 + k*boxWidth, 0);
					glVertex3d(boxStart + intervalStart*boxLength, 0.1 + (k+1)*boxWidth, 0);
				glEnd();
			}
		}
	}

	//draw the intermediate gait events
	glColor3d(0,0,0);
	
	//draw the vertical lines that delineate the different cycles
	glColor3d(0.7, 0.7, 0.7);
	glBegin(GL_LINES);
	for (int i=0;i<=1;i++){
		glVertex2d(boxStart + i*boxLength, 0.1);
		glVertex2d(boxStart + i*boxLength, 0.9);		
	}
	glEnd();


	//draw the horizontal lines that delineate the different legs used...
	glBegin(GL_LINES);
	for (int i=0;i<=numSections;i++){
		glVertex2d(labelsStart, 0.1 + i*boxWidth);
		glVertex2d(boxStart + 1 * boxLength, 0.1 + i*boxWidth);
	}
	glEnd();


	//finally, draw the cursor
	if (cursorPosition < 0)	cursorPosition += 1;
	if (cursorPosition > 1) cursorPosition -= 1;

	glColor3d(0, 0, 0.8);
	glLineWidth(2.0);
	glBegin(GL_LINES);
		glVertex2d(boxStart + cursorPosition*boxLength, 0.1);
		glVertex2d(boxStart + cursorPosition*boxLength, 0.9);
	glEnd();
	glLineWidth(1.0);

	glBegin(GL_TRIANGLES);
		glVertex2d(boxStart + cursorPosition*boxLength, 0.1);
		glVertex2d(boxStart + cursorPosition*boxLength-0.025, 0.0);
		glVertex2d(boxStart + cursorPosition*boxLength+0.025, 0.0);

		glVertex2d(boxStart + cursorPosition*boxLength, 0.9);
		glVertex2d(boxStart + cursorPosition*boxLength-0.025, 1);
		glVertex2d(boxStart + cursorPosition*boxLength+0.025, 1);
	glEnd();

	//AND DONE!

	// Restore attributes
	postDraw();
}

//-need to look at why the plan is so conservative - feet can be on the ground, but they are not considered in stance yet - why ?


//triggered when mouse buttons are pressed
bool FootFallPatternViewer::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (GLWindow2D::onMouseButtonEvent(button, action, mods, xPos, yPos))
		return true;

	double x = this->getRelativeXFromViewportX(getViewportXFromWindowX(xPos));
	double y = this->getRelativeYFromViewportY(getViewportYFromWindowY(yPos));

	if (x < boxStart || x > boxStart + boxLength || y < 0.1 || y > 0.9)
		return true;

	x = mapTo01Range(x, boxStart, boxStart + boxLength);
	y = mapTo01Range(y, 0.1, 0.9);

	int start = 0;
	int end = ffp->strideSamplePoints;

	int row = (int)(y * ffp->stepPatterns.size());
	int col = (int)(x * (end - start));

	oldCol = col;

	selectedLimbIndex = -1;
	if (ffp->isInSwing(ffp->stepPatterns[row].limb, start + col))
		selectedLimbIndex = row;

	return true;
}

//triggered when mouse moves
bool FootFallPatternViewer::onMouseMoveEvent(double xPos, double yPos) {
	if (GLWindow2D::onMouseMoveEvent(xPos, yPos))
		return true;

	if (mouseIsWithinWindow(xPos, yPos) == false && GlobalMouseState::lButtonPressed == false && GlobalMouseState::rButtonPressed == false) {
		selectedLimbIndex = -1;
		selectedColIndex = -1;
		return false;
	}

	double x = this->getRelativeXFromViewportX(getViewportXFromWindowX(xPos));
	double y = this->getRelativeYFromViewportY(getViewportYFromWindowY(yPos));

	if (cursorMovable && GlobalMouseState::lButtonPressed && selectedLimbIndex < 0) {
		cursorPosition = mapTo01Range(x, boxStart, boxStart + boxLength);
		int shiftDown = glfwGetKey(GLApplication::getGLAppInstance()->glfwWindow, GLFW_KEY_LEFT_SHIFT);
		if (shiftDown)
			cursorPosition = (int)(cursorPosition * (double)ffp->strideSamplePoints) / (double)ffp->strideSamplePoints;

		//Logger::consolePrint("cursorPosition: %lf\n", cursorPosition);
	}

	if (x < boxStart || x > boxStart + boxLength || y < 0.1 || y > 0.9) {
		if (GlobalMouseState::lButtonPressed == false && GlobalMouseState::rButtonPressed == false) {
			selectedLimbIndex = -1;
			selectedColIndex = -1;
		}
		return true;
	}

	x = mapTo01Range(x, boxStart, boxStart + boxLength);
	y = mapTo01Range(y, 0.1, 0.9);

	int start = 0;
	int end = ffp->strideSamplePoints;

	int row = (int)(y * ffp->stepPatterns.size());
	int col = (int)(x * (end - start));

	selectedColIndex = col;

	if (GlobalMouseState::lButtonPressed == false && GlobalMouseState::rButtonPressed == false) {
		selectedLimbIndex = -1;
		selectedColIndex = -1;
		if (ffp->isInSwing(ffp->stepPatterns[row].limb, start + col))
			selectedLimbIndex = row;
		selectedColIndex = col;
	}
	else if(ffpMovable && selectedLimbIndex >= 0 && selectedLimbIndex < (int)ffp->stepPatterns.size())
	{
		int offset = oldCol - col;

		if (GlobalMouseState::lButtonPressed == true && oldCol != col) {
			ffp->stepPatterns[selectedLimbIndex].startIndex -= offset;
			ffp->stepPatterns[selectedLimbIndex].endIndex -= offset;
			ffp->dirty = true;
		}

		if (GlobalMouseState::rButtonPressed == true && oldCol != col) {
			if (ffp->isStart(ffp->stepPatterns[selectedLimbIndex].limb, oldCol)) {
				int newIntervalLength = ffp->stepPatterns[selectedLimbIndex].endIndex - ffp->stepPatterns[selectedLimbIndex].startIndex + offset;
				if (newIntervalLength >= 1 && newIntervalLength < ffp->strideSamplePoints) {
					ffp->stepPatterns[selectedLimbIndex].startIndex -= offset;
					ffp->dirty = true;
				}
					
			}
			if (ffp->isEnd(ffp->stepPatterns[selectedLimbIndex].limb, oldCol)) {
				int newIntervalLength = ffp->stepPatterns[selectedLimbIndex].endIndex - ffp->stepPatterns[selectedLimbIndex].startIndex - offset;
				if (newIntervalLength >= 1 && newIntervalLength < ffp->strideSamplePoints) {
					ffp->stepPatterns[selectedLimbIndex].endIndex -= offset;
					ffp->dirty = true;
				}				
			}
		}

		if (ffp->stepPatterns[selectedLimbIndex].startIndex < 0){
			ffp->stepPatterns[selectedLimbIndex].startIndex += ffp->strideSamplePoints;
			ffp->stepPatterns[selectedLimbIndex].endIndex += ffp->strideSamplePoints;
		}
		if (ffp->stepPatterns[selectedLimbIndex].startIndex >= ffp->strideSamplePoints){
			ffp->stepPatterns[selectedLimbIndex].startIndex -= ffp->strideSamplePoints;
			ffp->stepPatterns[selectedLimbIndex].endIndex -= ffp->strideSamplePoints;
		}

		oldCol = col;
	}



	return true;
}

