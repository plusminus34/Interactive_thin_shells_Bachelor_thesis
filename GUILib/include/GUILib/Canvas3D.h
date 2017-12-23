#pragma once

#include "GLIncludes.h"
#include "GLUtils.h"
#include "GLCamera.h"
#include "GLContentManager.h"
#include <Utils/utils.h>

class Canvas3D {
public:

	Canvas3D() {}
	~Canvas3D() {}

	//background color
	double bgColorR = 0.75, bgColorG = 0.75, bgColorB = 0.75, bgColorA = 0.5;
	//camera
	GLCamera* camera;

	GLCamera* getCamera() { return camera; }

	bool showReflections = false;
	bool showGroundPlane = false;
	bool followCameraTarget = false;
	bool showDesignEnvironmentBox = false;


	virtual void setupLights() {
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

	virtual void drawReflections() {
		glDisable(GL_LIGHTING);
		glDisable(GL_TEXTURE_2D);
		glClear(GL_STENCIL_BUFFER_BIT);

		glEnable(GL_STENCIL_TEST); //Enable using the stencil buffer
		glColorMask(0, 0, 0, 0); //Disable drawing colors to the screen
		glDisable(GL_DEPTH_TEST); //Disable depth testing
		glStencilFunc(GL_ALWAYS, 1, 1); //Make the stencil test always pass
										//Make pixels in the stencil buffer be set to 1 when the stencil test passes
		glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		drawGround();
		glDisable(GL_CULL_FACE);

		glColorMask(1, 1, 1, 1); //Enable drawing colors to the screen
		glEnable(GL_DEPTH_TEST); //Enable depth testing
								 //Make the stencil test pass only when the pixel is 1 in the stencil buffer
		glStencilFunc(GL_EQUAL, 1, 1);
		glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP); //Make the stencil buffer not change
		glPushMatrix();

		V3D scale(1, 1, 1);
		scale -= Globals::groundPlane.n * 2;

		glScalef((float)scale[0], (float)scale[1], (float)scale[2]);

//		glEnable(GL_CLIP_PLANE0);
//		double plane[4];
//		Globals::groundPlane.getCartesianEquationCoefficients(plane[0], plane[1], plane[2], plane[3]);
//		glClipPlane(GL_CLIP_PLANE0, plane);
		drawScene();
		glPopMatrix();

//		glDisable(GL_CLIP_PLANE0);
		glDisable(GL_STENCIL_TEST);
	}

	virtual void drawGroundAndReflections() {
		if (showReflections) {
			drawReflections();
			glColor4f(1, 1, 1, 0.85f);
		}
		else
			glColor4d(1, 1, 1, 1);

		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);
		glDisable(GL_LIGHTING);
		drawGround();
		glDisable(GL_BLEND);
	}

	virtual void drawDesignEnvironment() {
		glColor4d(1, 1, 1, 1);
		glDisable(GL_LIGHTING);
		drawDesignEnvironmentBox(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
	}

	virtual void drawGround() {
		drawTexturedGround(GLContentManager::getTexture("../data/textures/ground_TileLight2.bmp"));
	}

	virtual void drawScene() {};

};

