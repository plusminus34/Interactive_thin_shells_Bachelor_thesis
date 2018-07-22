#pragma once

#include <stdio.h>
#include <MathLib/MathLib.h>
#include "..\include\KineSimLib\KS_MechanismController.h"
#include "..\include\KineSimLib\KS_UIMechanismController.h"
#include <iomanip>

KS_UIMechanismController::KS_UIMechanismController(KS_MechanicalAssembly* mech) : KS_MechanismController(mech)
{
}

KS_UIMechanismController::KS_UIMechanismController(KS_MechanicalAssembly * mech, GLApplication * app) : KS_MechanismController(mech)
{ 
	this->mechanism = mech;
	this->app = app;
	motorAngles.clear();
	motorAnglesOld.clear();
	motorAnglesWidget.clear();

	for (int i = 0; i < getActuatedConnectionCount();i++){
		motorAngles["motor " + std::to_string(i+1)] = 0;
		motorAnglesOld["motor " + std::to_string(i+1)] = 0;
	}
	
}

KS_UIMechanismController::~KS_UIMechanismController(void)
{
}

void KS_UIMechanismController::setMotorAngleValues()
{
	Logger::print("actuated connection count %d\n", getActuatedConnectionCount());
	if (boxes) {
		for (int i = 0; i < getActuatedConnectionCount(); i++) {
			app->mainMenu->addVariable("Motor:   " + std::to_string(i + 1), motorAngleValues[i]);
		}
	}
	if (sliders) {
		if (motorAnglesControlWindow && app->menuScreen)
			app->menuScreen->removeChild(motorAnglesControlWindow);
		motorAnglesControlWindow = new nanogui::Window(app->menuScreen, "Motor angle control");
		motorAnglesControlWindow->setPosition(Eigen::Vector2i(0, app->mainMenu->window()->size()(1) + 200));
		motorAnglesControlWindow->setWidth(app->mainMenu->window()->width());
		nanogui::GroupLayout *groupLayout = new nanogui::GroupLayout();
		motorAnglesControlWindow->setLayout(groupLayout);

		new nanogui::Label(motorAnglesControlWindow, "Motor Angles");
		for (auto &p : motorAngles) {
			motorAnglesWidget[p.first] = addSliderTextVariable(p.first, &p.second, std::pair<double, double>(-3.14, 3.14), motorAnglesControlWindow, "", 3);
		}
		nanogui::Button* button = new nanogui::Button(motorAnglesControlWindow, "Reset motor angles to a specified one");
		button->setCallback([this]() {
			//for (auto &p : points_rbd)
			//p.second =0;
			for (int i = 0; i < getActuatedConnectionCount(); i++) {
				motorAngles["motor " + std::to_string(i + 1)] = motorAnglesOld["motor " + std::to_string(i + 1)];
			}
			updateUI();
		});
	}

}

void KS_UIMechanismController::activateMechanismController()
{
	if (sliders) {
		for (int i = 0; i <getActuatedConnectionCount(); i++) {
			motorAngleValues[i] = motorAngles["motor " + std::to_string(i+1)];
		}
	}
	updateMotorConnections();
	mechanism->solveAssembly();
	mechanism->AConstraintEnergy->setCurrentBestSolution(mechanism->s);
}

void KS_UIMechanismController::updateUI()
{
	for (auto &w : motorAnglesWidget) {
		w.second.slider->setValue((float)motorAngles[w.first]);
		w.second.textBox->setValue(toString(motorAngles[w.first], 3));
	}

}

KS_UIMechanismController::SliderText KS_UIMechanismController::addSliderTextVariable(const std::string &name, double *var, const std::pair<double, double> &range, nanogui::Widget *widget, std::string units, int precision)
{
	nanogui::Widget *panel = new nanogui::Widget(widget);
	panel->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 0, 20));

	new nanogui::Label(panel, name);

	nanogui::Slider *slider = new nanogui::Slider(panel);
	slider->setValue((float)*var);
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
		slider->setValue((float)value);
		return true;
	});

	SliderText st;
	st.slider = slider;
	st.textBox = textBox;
	return st;
}

template<class T>
std::string KS_UIMechanismController::toString(T value, int precision)
{
	std::stringstream stream;
	stream << std::fixed << std::setprecision(precision) << value;
	return stream.str();
}