#include <RobotDesignerLib/EnergyWindow.h>
#include <GUILib/ColorMaps.h>

EnergyWindow::EnergyWindow(RobotDesignerApp *app)
{
	this->rdApp = app;
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
	window = new Window(screen, "Energy Window");
	window->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Middle, 15, 5));
	window->setVisible(false);
	window->setPosition({1100, 0});

	// make energy panel
	VScrollPanel *vscroll = new VScrollPanel(window);
	vscroll->setHeight(screen->height()-200);

	Widget *energyPanel;
	{
		energyPanel = new Widget(vscroll);
		GridLayout *layout = new GridLayout(Orientation::Horizontal, 7, Alignment::Fill);
		layout->setColAlignment({ Alignment::Minimum, Alignment::Fill });
		layout->setSpacing(0, 10);
		energyPanel->setLayout(layout);

		energyUIRows.clear();

		for (const auto &objGroup : energyFunction->objGroups) {

			const std::string &groupName = objGroup.first;

			Button *buttonHideGroup = new Button(energyPanel, objGroup.first);
//			b->setTextColor(Color(0.2f, 0.7f, 1.0f, 1.f));
//			b->setBackgroundColor(Color(0.2f, 0.7f, 1.0f, 0.3f));
//			b->setTextColor(Color(0, 0, 255, 180));
			buttonHideGroup->setBackgroundColor(Color(0, 0, 255, 25));
			buttonHideGroup->setFlags(Button::ToggleButton);
			buttonHideGroup->setChangeCallback([this, groupName, screen](bool state){
				this->hideEnergyGroup(!state, groupName);
				screen->performLayout();
			});
			buttonHideGroup->setFontSize(14);

			Button *button = new Button(energyPanel, "");
			button->setIcon(ENTYPO_ICON_CLASSIC_COMPUTER);
			button->setCallback([=] {DoParameterOptimizationStep(energyFunction); 
									 updateWeightTextboxes(energyFunction); });
			button->setFontSize(14);

			new Label(energyPanel, "Is Active", "sans");
			new Label(energyPanel, "Hack Hessian", "sans");
			new Label(energyPanel, "", "");
			new Label(energyPanel, "", "");
			new Label(energyPanel, "", "");

			for (ObjectiveFunction *obj : objGroup.second) {
				EnergyUIElement el;
				el.objective = obj;
				el.label = new Label(energyPanel, obj->description, "sans");

				Button *button = new Button(energyPanel, "");
				button->setIcon(ENTYPO_ICON_CLASSIC_COMPUTER);
				button->setCallback([=]() {DoParameterOptimizationStep(obj); 
																updateWeightTextboxes(energyFunction);});
				button->setFontSize(14);
				el.optimizeEnergy = button;

				CheckBox *checkBox = new CheckBox(energyPanel,"");
				checkBox->setChecked(obj->isActive);
				checkBox->setCallback([=](bool value){obj->isActive = value; });
				el.checkBoxEnergyActive = checkBox;

				checkBox = new CheckBox(energyPanel, "");
				checkBox->setChecked(obj->hackHessian);
				checkBox->setCallback([obj](bool value) {obj->hackHessian = value; });
				el.checkBoxHackHessian = checkBox;

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

			if(energyGroupVisible.find(groupName) == energyGroupVisible.end())
				energyGroupVisible[groupName] = true;
			else
				hideEnergyGroup(energyGroupVisible[groupName], groupName);
			buttonHideGroup->setPushed(!energyGroupVisible[groupName]);
		}
	}

	// make energy plot
	Widget *plotPanel;
	{
		plotPanel = new Widget(window);
		GridLayout *layout = new GridLayout(Orientation::Horizontal, 5, Alignment::Fill);
		layout->setColAlignment({ Alignment::Minimum, Alignment::Fill });
		layout->setRowAlignment({ Alignment::Minimum, Alignment::Fill });
		layout->setSpacing(0, 10);
		plotPanel->setLayout(layout);

		Widget *buttonSubPanel = new Widget(window);
		layout = new GridLayout(Orientation::Horizontal, 3, Alignment::Fill);
		layout->setColAlignment({ Alignment::Minimum, Alignment::Fill });
		layout->setRowAlignment({ Alignment::Minimum, Alignment::Fill });
		layout->setSpacing(0, 10);
		buttonSubPanel->setLayout(layout);

		Button *togglePlot = new Button(buttonSubPanel, "Energy Plot");
		togglePlot->setFlags(Button::ToggleButton);

		Button *btnSaveData = new Button(buttonSubPanel, "Save data");
		btnSaveData->setCallback([&] {saveData(); });

		Button *btnResetData = new Button(buttonSubPanel, "Reset");
		btnResetData->setCallback([&] {resetData(); });


		energyPlot = new Plot(plotPanel, "Energy Plot");
		energyPlot->setSize(Vector2i(600, 600));
		energyPlot->setNumTicks(Vector2i(10, 5));
		energyPlot->setShowTicks(true);
		energyPlot->setShowLegend(true);
		energyPlot->setVisible(false);


		togglePlot->setChangeCallback([this, screen](bool state){
			energyPlot->setVisible(state);
			screen->performLayout();
		});
	}



	screen->performLayout();
}

void EnergyWindow::DoParameterOptimizationStep(ObjectiveFunction * energyFunction)
{
	TotalEnergyForDesignOptimization.push_back(energyHist["Total energy"].back());
	rdApp->iEditWindow->DoDesignParametersOptimizationStep(energyFunction);

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
	int NE = energyFunction->objectives.size()+1;
	std::vector<double> values(NE); int index = 0;
	values[index++] = energyFunction->computeValue(params);
	for (const auto &objGroup : energyFunction->objGroups) {
		for (ObjectiveFunction *obj : objGroup.second){
			double value = obj->computeValue(params);
			values[index++] = value;
			if(obj->isActive)
				maxValue = std::max(maxValue, value);
		}
	}

	// update energy menu
	index = 1;
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
	
	{  //total energy plot
		float value = (float)values[index];
		std::vector<float> &eValues = energyHist[energyFunction->description];
		eValues.push_back(value);

		int size = std::min(numPlotValues, (int)eValues.size());
		int start = std::max(0, (int)eValues.size() - size);
		Eigen::Map<Eigen::VectorXf> y(&eValues[start], size);

		start = std::max(0, (int)plotXValues.size() - size);
		Eigen::Map<Eigen::VectorXf> x(&plotXValues[start], size);

		PlotData data(x, y, nanogui::Color(ColorMaps::getColorAt(ColorMaps::plasma, (float)index / (float)(40), 0.2f), 1.f));
		energyPlot->setPlotData(energyFunction->description, data);
		index++;
	}
	for (const auto &objGroup : energyFunction->objGroups) {
		for (ObjectiveFunction *obj : objGroup.second){
			if(obj->isActive)
			{
				float value = (float)values[index];
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

void EnergyWindow::hideEnergyGroup(bool visible, const std::string &groupName)
{
	for (EnergyUIElement &el : energyUIRows[groupName]) {
		el.label->setVisible(visible);
		el.optimizeEnergy->setVisible(visible);
		el.checkBoxEnergyActive->setVisible(visible);
		el.checkBoxHackHessian->setVisible(visible);
		el.slider->setVisible(visible);
		el.textbox->setVisible(visible);
		el.weightTextbox->setVisible(visible);
	}

	energyGroupVisible[groupName] = visible;
}

void EnergyWindow::updateWeightTextboxes(LocomotionEngine_EnergyFunction* energyFunction)
{
	using namespace nanogui;
	for (auto &objGroup : energyFunction->objGroups)
	{
		for(auto &el : energyUIRows[objGroup.first])
		{
			el.weightTextbox->setValue(el.objective->weight);
		}
	}
}

void EnergyWindow::resetData()
{
	for (auto& energy : energyHist)
	{
		energy.second.clear();
	}
}

void EnergyWindow::saveData()
{
	ofstream outputfile;
	outputfile.open("energyPerMotionIteration.txt");
	for (const auto& energy : energyHist)
	{
		outputfile << energy.first << ",";
	}
	outputfile << endl;
	for (uint i = 0; i < (*energyHist.begin()).second.size(); i++)
	{
		for (const auto& energy : energyHist)
		{
			outputfile << energy.second[i] << ",";
		}
		outputfile << endl;
	}
	outputfile.close();
	outputfile.open("energyPerDesignIteration.txt");
	for (uint i = 0; i < TotalEnergyForDesignOptimization.size(); i++)
		outputfile << TotalEnergyForDesignOptimization[i] << endl;
	outputfile.close();
}
