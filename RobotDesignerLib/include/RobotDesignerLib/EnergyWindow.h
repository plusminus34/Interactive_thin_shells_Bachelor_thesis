#pragma once
#include <../Apps/RobotDesignerApp/RobotDesignerApp.h>
#include <GUILib/GLIncludes.h>
#include <nanogui/nanogui.h>
#include <GUILib/Plot.h>
#include <RobotDesignerLib/LocomotionEngineEnergyFunction.h>
#include <RobotDesignerLib/IntelligentRobotEditingWindow.h>

class RobotDesignerApp;
class IntelligentRobotEditingWindow;

class EnergyWindow
{
public:
	EnergyWindow(RobotDesignerApp *app);

	void createEnergyMenu(LocomotionEngine_EnergyFunction *energyFunction, nanogui::Screen *screen);

	void updateEnergiesWith(LocomotionEngine_EnergyFunction *energyFunction, const dVector &params);

	void setVisible(bool visible);
	RobotDesignerApp *rdApp;

private:
	void hideEnergyGroup(bool visible, const std::string &groupName);

private:
	nanogui::Window * window = nullptr;
	struct EnergyUIElement {
		nanogui::Label *label;
		nanogui::Button *optimizeEnergy;
		nanogui::CheckBox *checkBoxEnergyActive;
		nanogui::CheckBox *checkBoxHackHessian;
		nanogui::Slider* slider;
		nanogui::FloatBox<double>* textbox;
		nanogui::FloatBox<double>* weightTextbox;
	};
	std::map<std::string, std::vector<EnergyUIElement>> energyUIRows;

	std::map<std::string, bool> energyGroupVisible;

	Plot *energyPlot = nullptr;
	std::map<std::string, std::vector<float>> energyHist;

	const int numPlotValues = 100;
	Eigen::VectorXf plotXValues;
};
