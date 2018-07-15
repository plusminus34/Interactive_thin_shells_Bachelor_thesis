#pragma once

#include <stdio.h>
#include <MathLib/MathLib.h>
#include <KineSimLib\KS_MechanismController.h>
#include <GUILib/GLApplication.h>
#include <map>

class KS_UIMechanismController : public KS_MechanismController {
public:
	KS_UIMechanismController(KS_MechanicalAssembly* mech);
	KS_UIMechanismController(KS_MechanicalAssembly* mech, GLApplication* app);

	virtual ~KS_UIMechanismController(void);
	virtual void setMotorAngleValues(); //each controller will have its own way of specifying the target motor angle values
	void activateMechanismController();
	struct SliderText {
		nanogui::Slider *slider;
		nanogui::TextBox *textBox;
	};
	void updateUI();
	static SliderText addSliderTextVariable(const std::string &name, double *var, const std::pair<double, double> &range, nanogui::Widget *panel, std::string units = "", int precision = 3);
	template<class T>
	static std::string toString(T value, int precision = 3);
	
	GLApplication* app;
	std::map<std::string, double> motorAngles;
	std::map<std::string, double> motorAnglesOld;
	nanogui::Window *motorAnglesControlWindow = nullptr;
	

	std::map<std::string, SliderText> motorAnglesWidget;
	bool boxes=false, sliders=false;

	
};


