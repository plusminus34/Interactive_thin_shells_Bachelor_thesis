#pragma once

#include <nanogui/nanogui.h>

#include "Utils/Timer.h"

#include <vector>
#include <string>

#include <fstream>


class ScreenRecorder {

public:
	enum ImageFormat {FORMAT_RAW, FORMAT_RAWFILE, FORMAT_BMP, FORMAT_PNG};
	
	std::string dirName;
	std::string fileBaseName;

	// elements of the menu
	nanogui::FormHelper * guiMenu = NULL;
	
	nanogui::Button *      recordButton;
	nanogui::Button *      saveButton;
	nanogui::Label *       bufferStatusLabel;
	nanogui::Label *       infoLabel;
	nanogui::ProgressBar * bufferFillStatusBar;
	nanogui::Slider *      deleteSlider;


private:
	bool record = false;

	int instanceID;
	static int instanceCounter;

	ImageFormat recordingFormat = FORMAT_RAWFILE;
	ImageFormat outputFormat = FORMAT_PNG;

	std::vector<unsigned char> imageBuffer;
	std::vector<size_t> imageStartIdx;
	std::vector<size_t> imageEndIdx;
	std::vector<ImageFormat> imageFormat;
	std::vector<int> imageWidth;
	std::vector<int> imageHeight;
	std::vector<int> imageChannels;

	std::vector<unsigned char> singleRawBuffer;
	std::fstream fileRawBuffer;
	std::string fileRawBufferName;

	// info
	size_t size_last_frame = 0;
	double compressionRatio_last = 0.0;
	double time_recording_last = 0.0;

	// others
	Timer timer;


public:

	ScreenRecorder(size_t bufferSize = 0);
	~ScreenRecorder();

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



