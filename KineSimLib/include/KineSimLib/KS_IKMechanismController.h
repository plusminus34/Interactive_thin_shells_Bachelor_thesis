#pragma once

#include <stdio.h>
#include <MathLib/MathLib.h>
#include <KineSimLib/KS_MechanismController.h>
#include <GUILib/GLApplication.h>
#include <map>
#include <KineSimLib/KS_IKConstraintEnergy.h>

class KS_IKMechanismController : public KS_MechanismController {
public:
	KS_IKMechanismController(KS_MechanicalAssembly* mech);
	KS_IKMechanismController(KS_MechanicalAssembly * mech, KS_MechanicalComponent* compEE, GLApplication * app);
	virtual ~KS_IKMechanismController(void);

	virtual void setMotorAngleValues(); //each controller will have its own way of specifying the target motor angle values
	void activateMechanismController();
	void solveMotorAngles();

	struct SliderText {
		nanogui::Slider *slider;
		nanogui::TextBox *textBox;
	};
	void updateUI();
	static SliderText addSliderTextVariable(const std::string &name, double *var, const std::pair<double, double> &range, nanogui::Widget *panel, std::string units = "", int precision = 3);
	template<class T>
	static std::string toString(T value, int precision = 3);
	
	GLApplication* app;
	std::map<std::string, double> xEE;
	std::map<std::string, double> xEEOld;
	std::map<std::string, double> yEE;
	std::map<std::string, double> yEEOld;
	std::map<std::string, double> zEE;
	std::map<std::string, double> zEEOld;
	/*std::map<std::string, double> alphaEE;
	std::map<std::string, double> alphaEEOld;
	std::map<std::string, double> betaEE;
	std::map<std::string, double> betaEEOld;
	std::map<std::string, double> gammaEE;
	std::map<std::string, double> gammaEEOld;*/
	
	nanogui::Window *IKControlWindow = nullptr;
	
	std::map<std::string, SliderText> xEEWidget;
	std::map<std::string, SliderText> yEEWidget;
	std::map<std::string, SliderText> zEEWidget;
	/*std::map<std::string, SliderText> alphaEEWidget;
	std::map<std::string, SliderText> betaEEWidget;
	std::map<std::string, SliderText> gammaEEWidget;*/

	KS_IKConstraintEnergy* ikConstraintEnergy = NULL;
	double xEEd, yEEd, zEEd, alphaEEd, betaEEd, gammaEEd;
	bool xEEDOF, yEEDOF, zEEDOF, alphaEEDOF, betaEEDOF, gammaEEDOF;
	KS_MechanicalComponent* eeComponent;
	dVector motorAngleValuesSolver;
};


