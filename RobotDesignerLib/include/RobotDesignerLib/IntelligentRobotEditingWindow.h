#pragma once
#include <OptimizationLib/BFGSHessianApproximator.h>
#include "../../Apps/RobotDesignerApp/RobotDesignerApp.h"
#include <memory>

class RobotDesignerApp;
class BFGSHessianApproximator;
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
	void update_dmdX();
	void test_dmdp_Jacobian();
	void DoDesignParametersOptimizationStep(ObjectiveFunction* objFunction);
	void showMenu();
	void hideMenu();
	void syncSliders();
	void CreateParametersDesignWindow();
	void updateParamsUsingSliders(int paramIndex, double value);
	void updateParamsAndMotion(dVector p);

	void updatePUsingSynergies(dVector &p);

	virtual void drawAuxiliarySceneInfo();

	void updateParams(const dVector& p);
	void updateParams(const std::vector<double>& p);


	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

	void offsetDesignParameters(double yOffset, DynamicArray<double> &currentDesignParameters);

	nanogui::Window * menu = nullptr;

	virtual void setViewportParameters(int posX, int posY, int sizeX, int sizeY);
	void resetParams();
	enum class Mode { design, weights } mode = Mode::design;
	enum class MotionParamSet { Joints, EndEffectors, Forces, Wheels, BodyPosition, BodyOrientation, All } motionParamSet = MotionParamSet::BodyOrientation;

private:
	bool updateMotionBasedOnJacobian = false;
	bool useSVD = false;
	bool updateJacobiancontinuously = false;
	MatrixNxM dmdX; //The jacobian at a point
	dVector m0;
	MatrixNxM dmdX_V;
	MatrixNxM dgdX;
	dVector p0,w0;
	dVector slidervalues;
	std::vector<nanogui::Slider*> sliders;
	std::vector<nanogui::TextBox*> textboxes;
	bool compute_dgdp_With_FD = true;
	int optimizeEnergyNum = 11;
	double stepSize = 0.01;
	std::unique_ptr<BFGSHessianApproximator> lbfgsMinimizer;
	bool useLBFGS = false;
	void onModeChange();
	set<int> fixedDesignParamSet;
	bool useSynergies = false;
	dVector prev_p;
	void takeMotionParamSnapshot();
	dVector m_snap;
	bool useSnapshot = false;
};

