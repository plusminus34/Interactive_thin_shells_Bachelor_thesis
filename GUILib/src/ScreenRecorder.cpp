#include <GUILib/GLContentManager.h>
#include <GUILib/GLUtils.h>

#include "GUILib/ScreenRecorder.h"
#include "Utils/Image.h"
#include "Utils/BMPIO.h"

#include <sstream>
#include <iomanip>


ScreenRecorder::ScreenRecorder(size_t bufferSize)
{
	setBufferSize(bufferSize);
	dirName = "../screenShots/";
	fileBaseName = "frame";
}



int ScreenRecorder::call(GLFWwindow * glfwWindow) 
{
	if(!record) {
		return(0);
	}

	int viewportSettings[4];//x, y, w, h
	glGetIntegerv(GL_VIEWPORT, viewportSettings);
	int w, h;
	glfwGetWindowSize(glfwWindow, &w, &h);
	glReadBuffer(GL_BACK);

	if(recordingFormat == FORMAT_RAW) {
		// estimate size of raw image
		size_t imageSize = w * h * 3;
		if(imageSize <= imageBuffer.capacity() - imageBuffer.size()) {
			
			size_t idxStart = imageBuffer.size();
			imageBuffer.resize(imageBuffer.size() + imageSize);
			size_t idxEnd = imageBuffer.size();

			glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, &imageBuffer[idxStart]);

			imageStartIdx.push_back(idxStart);
			imageEndIdx.push_back(idxEnd);
			imageFormat.push_back(recordingFormat);
			imageWidth.push_back(w);
			imageHeight.push_back(h);

		}
		else {
			eventFullBuffer();
			updateGuiMenu();
			return(1);
		}
	}
	updateGuiMenu();
	return(0);
}


int ScreenRecorder::save(std::string const & dirName, std::string const & fileBaseName)
{
	
	for(size_t i = 0; i < imageStartIdx.size(); ++i) {
		if(imageFormat[i] == FORMAT_RAW && outputFormat == FORMAT_BMP) {
			// create file name
			std::stringstream fname_stream;
			fname_stream << dirName << "/" << fileBaseName;
			fname_stream << std::setfill('0') << std::setw(5) << i;
			fname_stream << ".bmp";			
			
			// create image
			Image img(3, imageWidth[i], imageHeight[i], &imageBuffer[imageStartIdx[i]]);
			// write to file
			BMPIO b(fname_stream.str().c_str());
			b.writeToFile(&img);

		}
		else {
			return(1);
		}
	}

	reset();

	return(0);
}

int ScreenRecorder::reset()
{
	imageBuffer.resize(0);
	imageStartIdx.resize(0);
	imageEndIdx.resize(0);
	imageFormat.resize(0);
	imageWidth.resize(0);
	imageHeight.resize(0);

	updateGuiMenu();
	return(0);
}


void ScreenRecorder::setRecord(int state) 
{
	record = state; 
	updateGuiMenu();
}


int ScreenRecorder::setBufferSize(size_t newSize)
{
	size_t oldSize = imageBuffer.capacity();

	int ret = 0;

	if(newSize >= imageBuffer.size()) {
		imageBuffer.shrink_to_fit();	// first, shrink buffer to make room to try and allocate a new buffer
		try{
			imageBuffer.reserve(newSize);	// try to reserve requested size size
		}
		catch(...) {
			try {							// if requested size not possible, try to allocate old size again
				imageBuffer.reserve(oldSize);
				ret = 1;
			}
			catch(...) {					// if that fails -> too bad.
				ret = 1;
			}
		}
	}
	else {
		imageBuffer.shrink_to_fit();
		ret = 1;
	}

	updateGuiMenu();
	return(ret);
}



void ScreenRecorder::eventFullBuffer() 
{
	record = false;
}



int ScreenRecorder::attachToNanoGui(nanogui::FormHelper* menu) 
{
	if(guiMenu != NULL) {
		return(1);
	}
	guiMenu = menu;

	guiMenu->addGroup("Recording");
	//guiMenu->addVariable("Record", this->record);
	//guiMenu->addButton("save record", [this](){this->save(dirName, fileBaseName);});

	// record / save / delete
	nanogui::Widget *recordTools = new nanogui::Widget(menu->window());
	guiMenu->addWidget("", recordTools);
	recordTools->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	recordButton = new nanogui::Button(recordTools, "RECORD");
	recordButton->setFlags(nanogui::Button::ToggleButton);
	recordButton->setChangeCallback([&](bool state){record = state;});

	saveButton = new nanogui::Button(recordTools, "Save");
	saveButton->setCallback([&](){this->save(dirName, fileBaseName);});

	nanogui::Button * deleteButton = new nanogui::Button(recordTools, "Clear");
	deleteSlider = new nanogui::Slider(recordTools);
	deleteSlider->setValue(0.0f);
	deleteButton->setCallback([&](){if(deleteSlider->value() > 0.99f) {this->reset();}});


	// buffer status 
	nanogui::Widget *bufferStatus = new nanogui::Widget(menu->window());
	guiMenu->addWidget("", bufferStatus);
	bufferStatus->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Vertical,
		nanogui::Alignment::Fill, 0, 4));

	bufferStatusLabel = new nanogui::Label(bufferStatus, "Buffer status: ");
	bufferFillStatusBar = new nanogui::ProgressBar(bufferStatus);

	// buffer control
	nanogui::Widget *bufferControl = new nanogui::Widget(menu->window());
	guiMenu->addWidget("", bufferControl);
	bufferControl->setLayout(new nanogui::BoxLayout(nanogui::Orientation::Horizontal,
		nanogui::Alignment::Middle, 0, 4));

	nanogui::Button * b;
	b = new nanogui::Button(bufferControl, "Decr. buff. size");
	b->setCallback([&](){this->setBufferSize(this->getBufferSize()-100'000'000);});
	b = new nanogui::Button(bufferControl, "Incr. buff. size");
	b->setCallback([&](){this->setBufferSize(this->getBufferSize()+100'000'000);});


	updateGuiMenu();
	return(0);

}

void ScreenRecorder::updateGuiMenu()
{
	if(guiMenu == NULL) {
		return;
	}
	
	// record
	recordButton->setPushed(record);

	// buffer status
	std::stringstream bufferstat_sstream;
	bufferstat_sstream << "Buffer status: " << getBufferFilledSize()/1'000'000 << " / " << getBufferSize()/1'000'000 << " MB";
	bufferStatusLabel->setCaption(bufferstat_sstream.str());
	bufferFillStatusBar->setValue(getBufferFilledFraction());

	// delete
	deleteSlider->setValue(0.0f);

}