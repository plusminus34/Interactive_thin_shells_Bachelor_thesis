#pragma once

#include <nanogui/nanogui.h>

#include <vector>
#include <string>


class ScreenRecorder {

public:
	enum ImageFormat {FORMAT_RAW, FORMAT_BMP, FORMAT_PNG};
	
	

	std::string dirName;
	std::string fileBaseName;


	// elements of the menu
	nanogui::FormHelper * guiMenu = NULL;
	
	nanogui::Button *      recordButton;
	nanogui::Button *      saveButton;
	nanogui::Label *       bufferStatusLabel;
	nanogui::ProgressBar * bufferFillStatusBar;
	nanogui::Slider *      deleteSlider;


private:
	bool record = false;

	ImageFormat recordingFormat = FORMAT_RAW;
	ImageFormat outputFormat = FORMAT_BMP;

	std::vector<unsigned char> imageBuffer;
	std::vector<size_t> imageStartIdx;
	std::vector<size_t> imageEndIdx;
	std::vector<ImageFormat> imageFormat;
	std::vector<int> imageWidth;
	std::vector<int> imageHeight;

	std::vector<unsigned char> singleRawBuffer;

	


public:

	ScreenRecorder(size_t bufferSize = 100'000'000);


	int call(GLFWwindow * glfwWindow);
	int save(std::string const & dirName, std::string const & fileBaseName);
	int reset();

	size_t getBufferSize() {return(imageBuffer.capacity());}
	size_t getBufferFilledSize() {return(imageBuffer.size());}
	float getBufferFilledFraction() {return(static_cast<float>(getBufferFilledSize())/static_cast<float>(getBufferSize()));}

	
	void setRecord(int state);
	int setBufferSize(size_t newSize);
	


	// events
	virtual void eventFullBuffer();

	// GUI
	int attachToNanoGui(nanogui::FormHelper* menu);
	void updateGuiMenu();

};



