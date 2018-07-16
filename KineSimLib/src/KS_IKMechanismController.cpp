#pragma once

#include <stdio.h>
#include <MathLib/MathLib.h>
#include <KineSimLib/KS_MechanismController.h>
#include <KineSimLib/KS_IKMechanismController.h>
#include <iomanip>
#include <OptimizationLib/BFGSFunctionMinimizer.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>


KS_IKMechanismController::KS_IKMechanismController(KS_MechanicalAssembly* mech) : KS_MechanismController(mech)
{
	ikConstraintEnergy = NULL;
}

KS_IKMechanismController::KS_IKMechanismController(KS_MechanicalAssembly * mech, KS_MechanicalComponent* compEE, GLApplication * app) : KS_MechanismController(mech)
{ 
	this->mechanism = mech;
	this->eeComponent = compEE;
	this->app = app;
	xEEd = 0; yEEd = 0; zEEd = 0; alphaEEd = 0; betaEEd = 0; gammaEEd = 0;
	xEEDOF = false; yEEDOF = false; zEEDOF = false; alphaEEDOF=false; betaEEDOF = false; gammaEEDOF=false;
	xEE.clear(); xEEOld.clear(); xEEWidget.clear();
	yEE.clear(); yEEOld.clear(); yEEWidget.clear();
	zEE.clear(); zEEOld.clear(); zEEWidget.clear();

	motorAngleValuesSolver.resize(motorAngleValues.size()); motorAngleValuesSolver.setZero();

	for (int i = 0; i < getActuatedConnectionCount(); i++) {
		xEE["xEE"] = 0; xEEOld["xEE"] = 0;	yEE["yEE"] = 0; yEEOld["yEE"] = 0; zEE["zEE"] = 0; zEEOld["zEE"] = 0;
	}
	ikConstraintEnergy = new KS_IKConstraintEnergy();
	ikConstraintEnergy->initialize(mech, this);
}

KS_IKMechanismController::~KS_IKMechanismController(void)
{
	delete ikConstraintEnergy;
}

void KS_IKMechanismController::setMotorAngleValues()
{
	Logger::print("actuated connection count %d\n", getActuatedConnectionCount());
		if (IKControlWindow && app->menuScreen)
			app->menuScreen->removeChild(IKControlWindow);
		IKControlWindow = new nanogui::Window(app->menuScreen, "IK control");
		IKControlWindow->setPosition(Eigen::Vector2i(0, app->mainMenu->window()->size()(1) + 200));
		IKControlWindow->setWidth(app->mainMenu->window()->width());
		nanogui::GroupLayout *groupLayout = new nanogui::GroupLayout();
		IKControlWindow->setLayout(groupLayout);

		new nanogui::Label(IKControlWindow, "EE prms");
		for (auto &p : xEE) {
			xEEWidget[p.first] = addSliderTextVariable(p.first, &p.second, std::pair<double, double>(-3.14, 3.14), IKControlWindow, "", 3);
		}
		
		for (auto &p : yEE) {
			yEEWidget[p.first] = addSliderTextVariable(p.first, &p.second, std::pair<double, double>(-3.14, 3.14), IKControlWindow, "", 3);
		}

		for (auto &p : zEE) {
			zEEWidget[p.first] = addSliderTextVariable(p.first, &p.second, std::pair<double, double>(-3.14, 3.14), IKControlWindow, "", 3);
		}
		nanogui::Button* button = new nanogui::Button(IKControlWindow, "Reset EE prms");
		button->setCallback([this]() {
			//for (int i = 0; i < getActuatedConnectionCount(); i++) {
				xEE["xEE"] = xEEOld["xEE"]; yEE["yEE"] = yEEOld["yEE"]; zEE["zEE"] = zEEOld["zEE"];
			//}
			updateUI();
		});
}

void KS_IKMechanismController::activateMechanismController()
{
	xEEd = xEE["xEE"]; yEEd = yEE["yEE"]; zEEd = zEE["zEE"];
	solveMotorAngles();
	ikConstraintEnergy->setCurrentBestSolution(motorAngleValues);
}

void KS_IKMechanismController::updateUI()
{
	for (auto &w : xEEWidget) {
		w.second.slider->setValue(xEE[w.first]);
		w.second.textBox->setValue(toString(xEE[w.first], 3));
	}
	for (auto &w : yEEWidget) {
		w.second.slider->setValue(yEE[w.first]);
		w.second.textBox->setValue(toString(yEE[w.first], 3));
	}
	for (auto &w : zEEWidget) {
		w.second.slider->setValue(zEE[w.first]);
		w.second.textBox->setValue(toString(zEE[w.first], 3));
	}

}

KS_IKMechanismController::SliderText KS_IKMechanismController::addSliderTextVariable(const std::string &name, double *var, const std::pair<double, double> &range, nanogui::Widget *widget, std::string units, int precision)
{
	nanogui::Widget *panel = new nanogui::Widget(widget);
	panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 20));

	new nanogui::Label(panel, name);

	nanogui::Slider *slider = new nanogui::Slider(panel);
	slider->setValue(*var);
	slider->setRange(range);
	slider->setFixedWidth(140);

	nanogui::TextBox *textBox = new nanogui::TextBox(panel);
	textBox->setFixedSize(Eigen::Vector2i(80, 25));
	textBox->setValue(toString(*var, precision));
	textBox->setUnits(units);
	textBox->setEditable(true);
	textBox->setFontSize(20);
	textBox->setAlignment(nanogui::TextBox::Alignment::Right);

	slider->setCallback([var, textBox, precision](double value) {
		*var = value;
		textBox->setValue(toString(value, precision));
	});

	textBox->setCallback([var, slider, precision](const std::string &str) {
		double value = std::atof(str.c_str());
		*var = value;
		slider->setValue(value);
		return true;
	});

	SliderText st;
	st.slider = slider;
	st.textBox = textBox;
	return st;
}

template<class T>
std::string KS_IKMechanismController::toString(T value, int precision)
{
	std::stringstream stream;
	stream << std::fixed << std::setprecision(precision) << value;
	return stream.str();
}


void KS_IKMechanismController::solveMotorAngles()
{
	motorAngleValuesSolver = motorAngleValues;
	dVector currenMechState = mechanism->s;
	dVector currentMotorValues = motorAngleValues;

	if (1) {
		ikConstraintEnergy->testGradientWithFD(motorAngleValuesSolver);
		//ikConstraintEnergy->testHessianWithFD(motorAngleValuesSolver);
	}

	motorAngleValuesSolver = currentMotorValues;
	motorAngleValues = currentMotorValues;
	updateMotorConnections();
	mechanism->s = currenMechState;
	mechanism->setAssemblyState(currenMechState);

	double functionValue = ikConstraintEnergy->computeValue(motorAngleValuesSolver);
	Logger::consolePrint("IK energy value before solve C: %lf\n", functionValue);

	BFGSFunctionMinimizer minimizer(10);
	minimizer.printOutput = false;
	minimizer.minimize(ikConstraintEnergy, motorAngleValuesSolver, functionValue);



	Logger::consolePrint("IK energy value after solve C: %lf\n", functionValue);


	motorAngleValues = motorAngleValuesSolver;

}