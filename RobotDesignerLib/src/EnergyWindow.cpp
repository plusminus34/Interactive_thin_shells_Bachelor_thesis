#include <RobotDesignerLib/EnergyWindow.h>
#include <GUILib/ColorMaps.h>

EnergyWindow::EnergyWindow()
{
	plotXValues.resize(numPlotValues);
	for (int i = 0; i < numPlotValues; ++i) {
		plotXValues[i] = (float)i;
	}
}

void EnergyWindow::createEnergyMenu(LocomotionEngine_EnergyFunction *energyFunction, nanogui::Screen *screen)
{
	using namespace nanogui;

	if (window)
		window->dispose();
	window = new Window(screen, "Motion Plan Analysis");
	window->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 15, 5));
	window->setVisible(false);
	window->setPosition({1100, 0});

	// make energy panel
	{
		Widget *energyPanel = new Widget(window);
		GridLayout *layout = new GridLayout(Orientation::Horizontal, 5, Alignment::Fill);
		layout->setColAlignment({ Alignment::Minimum, Alignment::Fill });
		layout->setSpacing(0, 10);
		energyPanel->setLayout(layout);

		energyUIRows.clear();

		for (const auto &objGroup : energyFunction->objGroups) {

			const std::string &groupName = objGroup.first;

			Button *b = new Button(energyPanel, objGroup.first);
//			b->setTextColor(Color(0.2f, 0.7f, 1.0f, 1.f));
//			b->setBackgroundColor(Color(0.2f, 0.7f, 1.0f, 0.3f));
//			b->setTextColor(Color(0, 0, 255, 180));
			b->setBackgroundColor(Color(0, 0, 255, 25));
			b->setFlags(Button::ToggleButton);
			b->setChangeCallback([this, groupName, screen](bool state){
				for (EnergyUIElement &el : energyUIRows[groupName]) {
					el.label->setVisible(!state);
					el.checkBox->setVisible(!state);
					el.slider->setVisible(!state);
					el.textbox->setVisible(!state);
					el.weightTextbox->setVisible(!state);
				}
				screen->performLayout();
			});
			b->setFontSize(14);

			new Label(energyPanel, "", "");
			new Label(energyPanel, "", "");
			new Label(energyPanel, "", "");
			new Label(energyPanel, "", "");

			for (ObjectiveFunction *obj : objGroup.second) {

				EnergyUIElement el;

				el.label = new Label(energyPanel, obj->description, "sans");
				CheckBox *checkBox = new CheckBox(energyPanel,"");
				checkBox->setChecked(obj->isActive);
				checkBox->setCallback([obj](bool value){obj->isActive = value; });
				el.checkBox = checkBox;

				Slider *slider = new Slider(energyPanel);
				slider->setValue(0);
				slider->setRange({ 0.0,1.0 });
				slider->setFixedWidth(100);
				el.slider = slider;
				slider->setHighlightColor(Color(0, 0, 255, 25));

				FloatBox<double> *textBox = new FloatBox<double>(energyPanel);
				textBox->setFixedWidth(80);
				textBox->setEditable(false);
				textBox->setFixedHeight(18);
				el.textbox = textBox;

				textBox = new FloatBox<double>(energyPanel);
				textBox->setFixedWidth(80);
				textBox->setFixedHeight(18);
				textBox->setEditable(true);
				textBox->setValue(obj->weight);
				textBox->setCallback([obj](double value){obj->weight=value; });
				el.weightTextbox = textBox;

				energyUIRows[objGroup.first].push_back(el);
			}
		}
	}

	// make energy plot
	{
		Widget *plotPanel = new Widget(window);
		GridLayout *layout = new GridLayout(Orientation::Horizontal, 5, Alignment::Fill);
		layout->setColAlignment({ Alignment::Minimum, Alignment::Fill });
		layout->setSpacing(0, 10);
		plotPanel->setLayout(layout);

		energyPlot = new Plot(plotPanel, "Energy Plot");

		energyPlot->setSize(Vector2i(600, 1000));
		energyPlot->setNumTicks(Vector2i(10, 5));
		energyPlot->setShowTicks(true);
		energyPlot->setShowLegend(true);
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

	// update energy menu
	index = 0;
	for (auto &energyRow : energyUIRows) {
		for (EnergyUIElement &el : energyRow.second){
			double value = values[index++];
			el.slider->setValue((float)(value/maxValue));
			el.textbox->TextBox::setValue(double2string(value));
		}
	}

	// update energy plot
	energyPlot->clearPlotData();
	index = 0;
	for (const auto &objGroup : energyFunction->objGroups) {
		for (ObjectiveFunction *obj : objGroup.second){
			if(obj->isActive)
			{
				float value = values[index];
				std::vector<float> &eValues = energyHist[obj->description];
				eValues.push_back(value);

				int size = std::min(numPlotValues, (int)eValues.size());
				int start = std::max(0, (int)eValues.size()-size);
				Eigen::Map<Eigen::VectorXf> y(&eValues[start], size);

				start = std::max(0, (int)plotXValues.size()-size);
				Eigen::Map<Eigen::VectorXf> x(&plotXValues[start], size);

				PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::plasma, (float)index/(float)(40), 0.2f), 1.f));
				energyPlot->setPlotData(obj->description, data);
			}
			index++;
		}
	}
	energyPlot->updateMinMax();
}

void EnergyWindow::setVisible(bool visible)
{
	if(window)
		window->setVisible(visible);
}
