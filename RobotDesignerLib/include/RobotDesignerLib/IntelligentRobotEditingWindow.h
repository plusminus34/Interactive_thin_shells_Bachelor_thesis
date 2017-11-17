#pragma once

#include <../Apps/RobotDesignerApp/RobotDesignerApp.h>

class IntelligentRobotEditingWindow : public GLWindow3D {
public:
	RobotDesignerApp* rdApp;

	RigidBody* highlightedRigidBody = NULL;
	Joint* highlightedJoint = NULL;
	RBEndEffector* highlightedEE = NULL;
	RigidBody* highlightedEEParent = NULL;

	TranslateWidget* tWidget = NULL;

	void addMenuItems();

public:
	IntelligentRobotEditingWindow(int x, int y, int w, int h, RobotDesignerApp* rdApp);
	~IntelligentRobotEditingWindow();

	virtual void drawScene();
	void updateJacobian();
	void test_dmdp_Jacobian();
	void DoDesignParametersOptimizationStep();
	void showMenu();
	void hideMenu();
	void syncSliders();
	void CreateParametersDesignWindow();
	void updateParamsUsingSliders(int paramIndex, double value);
	void updateParamsAndMotion(dVector p);
	virtual void drawAuxiliarySceneInfo();
	virtual void setupLights();

	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);
	nanogui::Window * menu = nullptr;

	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);
	void resetParams();
private:
	bool updateMotionBasedOnJacobian = false;
	bool useSVD = false;
	bool updateJacobiancontinuously = false;
	MatrixNxM dmdp; //The jacobian at a point
	dVector m0;
	MatrixNxM dmdp_V;
	MatrixNxM dgdp;
	dVector p0;
	dVector slidervalues;
	std::vector<nanogui::Slider*> sliders;
	std::vector<nanogui::TextBox*> textboxes;
	bool compute_dgdp_With_FD = true;
	int optimizeEnergyNum = 11;
	double stepSize = 0.01;
};

