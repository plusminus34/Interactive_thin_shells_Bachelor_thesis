#include <RobotDesignerLib/EnergyWindow.h>

EnergyWindow::EnergyWindow()
{

}

void EnergyWindow::createEnergyMenu(LocomotionEngine_EnergyFunction *energyFunction, nanogui::Screen *screen)
{
	using namespace nanogui;

	if (window)
		window->dispose();
	window = new Window(screen, "Motion Plan Analysis");
	window->setLayout(new GridLayout(Orientation::Vertical, 2, Alignment::Middle, 15, 5));
	window->setVisible(true);
	window->setPosition({1100, 0});

	Widget *panel = new Widget(window);
	GridLayout *layout = new GridLayout(Orientation::Horizontal, 5, Alignment::Fill);
	layout->setColAlignment({ Alignment::Minimum, Alignment::Fill });
	layout->setSpacing(0, 10);
	panel->setLayout(layout);

	energyUIRows.clear();

	for (const auto &objGroup : energyFunction->objGroups) {

		Label *l = new Label(panel, objGroup.first, "sans-bold");
		l->setColor(Color(0.2f, 0.7f, 1.0f, 1.f));
		new Label(panel, "", "");
		new Label(panel, "", "");
		new Label(panel, "", "");
		new Label(panel, "", "");

		for (ObjectiveFunction *obj : objGroup.second) {

			EnergyUIElement el;

			new Label(panel, obj->description, "sans");
			CheckBox *chkBox = new CheckBox(panel,"");
			chkBox->setChecked(obj->isActive);
			chkBox->setCallback([obj](bool value){obj->isActive = value; });
			Slider *slider = new Slider(panel);
			slider->setValue(0);
			slider->setRange({ 0.0,1.0 });
			slider->setFixedWidth(100);
			el.slider = slider;

			FloatBox<double> *textBox = new FloatBox<double>(panel);
			textBox->setFixedWidth(80);
			textBox->setEditable(false);
			textBox->setFixedHeight(18);
			el.textbox = textBox;

			textBox = new FloatBox<double>(panel);
			textBox->setFixedWidth(80);
			textBox->setFixedHeight(18);
			textBox->setEditable(true);
			textBox->setValue(obj->weight);
			textBox->setCallback([obj](double value){obj->weight=value; });
			el.weightTextbox = textBox;

			energyUIRows[objGroup.first].push_back(el);
		}
	}

	screen->performLayout();
}

void EnergyWindow::updateEnergiesWith(LocomotionEngine_EnergyFunction *energyFunction, const dVector &params)
{
	auto double2string = [](double val) {
		std::ostringstream s;
		s.precision(2);
		s << val;
		return s.str();
	};

	double maxValue = -HUGE_VAL;
	int NE = energyFunction->objectives.size();
	std::vector<double> values(NE); int index = 0;
	for (const auto &objGroup : energyFunction->objGroups) {
		for (ObjectiveFunction *obj : objGroup.second){
			double value = obj->computeValue(params);
			values[index++] = value;
			if(obj->isActive)
				maxValue = std::max(maxValue, value);
		}
	}

	index = 0;
	for (auto &energyRow : energyUIRows) {
		for (EnergyUIElement &el : energyRow.second){
			double value = values[index++];
			el.slider->setValue((float)(value/maxValue));
			el.textbox->TextBox::setValue(double2string(value));
		}
	}
}

void EnergyWindow::setVisible(bool visible)
{
	if(window)
		window->setVisible(visible);
}
