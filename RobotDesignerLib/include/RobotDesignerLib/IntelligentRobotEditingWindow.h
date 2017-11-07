#pragma once

#include <../Apps/RobotDesignerApp/RobotDesignerApp.h>

class IntelligentRobotEditingWindow : public GLWindow3D {
public:
	RobotDesignerApp* rdApp;

	RigidBody* highlightedRigidBody = NULL;
public:
	IntelligentRobotEditingWindow(int x, int y, int w, int h, RobotDesignerApp* rdApp);
	~IntelligentRobotEditingWindow();

	virtual void drawScene();
	void compute_dmdp_Jacobian();
	void test_dmdp_Jacobian();
	void testOptimizeDesign();
	void showMenu();
	void hideMenu();
	void CreateParametersDesignWindow();
	void updateParamsAndMotion(int paramIndex, double value);
	virtual void drawAuxiliarySceneInfo();
	virtual void setupLights();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);
	nanogui::Window * menu = nullptr;

	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);
private:
	bool updateMotionBasedOnJacobian;
	MatrixNxM dmdp; //The jacobian at a point
	dVector m0;
	bool useSVD = false;
	MatrixNxM dmdp_V;
	dVector p0;
	dVector slidervalues;
};

