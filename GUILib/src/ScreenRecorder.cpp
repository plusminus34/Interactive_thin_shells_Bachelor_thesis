#include <GUILib/GLContentManager.h>
#include <GUILib/GLUtils.h>

#include "GUILib/ScreenRecorder.h"
#include "Utils/Image.h"
#include "Utils/BMPIO.h"

#include "LodePNG/lodepng.h"

#include <sstream>
#include <iomanip>
#include <fstream>
#include <cstdio>
#include <cstdint>


int ScreenRecorder::instanceCounter = 0;

ScreenRecorder::ScreenRecorder(size_t bufferSize)
{
	instanceID = instanceCounter++;

	setBufferSize(bufferSize);
	dirName = "../screenShots/";
	fileBaseName = "frame";

	fileRawBufferName = dirName + "rawFileBufferSCPScreenRecorder" + std::to_string(instanceID);

	if(recordingFormat == FORMAT_RAWFILE) {
		fileRawBuffer.open(fileRawBufferName, 
						   std::fstream::binary 
						   | std::fstream::in | std::fstream::out
						   | std::fstream::trunc);
	}
}

ScreenRecorder::~ScreenRecorder()
{
	std::cout << "ScreenRecorderDestructor" << std::endl;
	if(fileRawBuffer.is_open()) {
		std::cout << "removing buffer" << std::endl;
		fileRawBuffer.close();
		std::remove(fileRawBufferName.c_str());
	}
}


int ScreenRecorder::call(GLFWwindow * glfwWindow) 
{
	
	if(!record) {
		return(0);
	}

	timer.restart();

	int ret = 0;

	int w, h, c = 3;
	glfwGetWindowSize(glfwWindow, &w, &h);
	// note: the rows in openGL are by default aligned with steps of 4 bytes.
	// to avoid gaps between the lines, w*c (assuming 8-bit-depth) should be devisable by 4.	
	if(w*c % 4 != 0) {
		c = 4;
	}

	glReadBuffer(GL_BACK);

	if(recordingFormat == FORMAT_RAW) {
		// estimate size of raw image
		size_t imageSize = w * h * c;
		if(imageSize <= imageBuffer.capacity() - imageBuffer.size()) {
			
			size_t idxStart = imageBuffer.size();
			imageBuffer.resize(imageBuffer.size() + imageSize);
			size_t idxEnd = imageBuffer.size();

			glReadPixels(0, 0, w, h, (c==4?GL_RGBA:GL_RGB), GL_UNSIGNED_BYTE, &imageBuffer[idxStart]);

			imageStartIdx.push_back(idxStart);
			imageEndIdx.push_back(idxEnd);
			imageFormat.push_back(recordingFormat);
			imageWidth.push_back(w);
			imageHeight.push_back(h);
			imageChannels.push_back(c);
		}
		else {
			eventFullBuffer();
			ret = 1;
		}
	}
	else if(recordingFormat == FORMAT_RAWFILE) {
		// estimate size of raw image
		size_t imageSize = w * h * c;

		singleRawBuffer.resize(imageSize);
		glReadPixels(0, 0, w, h, (c==4?GL_RGBA:GL_RGB), GL_UNSIGNED_BYTE, &singleRawBuffer[0]);

		size_t idxStart = (imageEndIdx.size() == 0) ? 0 : imageEndIdx.back();
		size_t idxEnd = idxStart + imageSize;

		fileRawBuffer.write(reinterpret_cast<char *>(singleRawBuffer.data()), imageSize);
		
		imageStartIdx.push_back(idxStart);
		imageEndIdx.push_back(idxEnd);
		imageFormat.push_back(recordingFormat);
		imageWidth.push_back(w);
		imageHeight.push_back(h);
		imageChannels.push_back(c);
	}
	else if(recordingFormat == FORMAT_PNG) {

		// estimate size of raw image
		size_t imageSize = w * h * c;
		// allocate raw buffer
		singleRawBuffer.resize(imageSize);
		
		glReadPixels(0, 0, w, h, (c==4?GL_RGBA:GL_RGB), GL_UNSIGNED_BYTE, &singleRawBuffer[0]);

		// convert to png
		unsigned char * out;
		size_t outsize;
		lodepng_encode_memory(&out, &outsize,
			&singleRawBuffer[0], 
			static_cast<unsigned int>(w), static_cast<unsigned int>(h),
			LodePNGColorType::LCT_RGBA, 8);
		// (try to) dump to buffer
		if(outsize <= imageBuffer.capacity() - imageBuffer.size()) {
			size_t idxStart = imageBuffer.size();
			imageBuffer.insert(imageBuffer.end(), out, out+outsize);
			size_t idxEnd = imageBuffer.size();

			imageStartIdx.push_back(idxStart);
			imageEndIdx.push_back(idxEnd);
			imageFormat.push_back(recordingFormat);
			imageWidth.push_back(w);
			imageHeight.push_back(h);
			imageChannels.push_back(c);
		}
		else {
			eventFullBuffer();
			ret = 1;
		}
		free(out);
	}
	
	if(imageStartIdx.size() > 0) {
		size_last_frame = imageEndIdx.back() - imageStartIdx.back();
		compressionRatio_last = static_cast<double>(w*h*c) / static_cast<double>(size_last_frame);
	}

	time_recording_last = timer.timeEllapsed();


	updateGuiMenu();
	return(ret);
}


int ScreenRecorder::save(std::string const & dirName, std::string const & fileBaseName)
{
	
	// file name
	auto getFileName = [&] (size_t i) -> std::string {
		std::string fileEnding;
		switch(outputFormat) {
		case FORMAT_BMP :
			fileEnding = ".bmp";
			break;
		case FORMAT_PNG : 
			fileEnding = ".png";
			break;
		}
		std::stringstream fname_stream;
		fname_stream << dirName << "/" << fileBaseName;
		fname_stream << std::setfill('0') << std::setw(5) << i;
		fname_stream << fileEnding;
		return(fname_stream.str());
	};

	auto writeIthPNG = [&](size_t i, unsigned char *raw_buffer) {
		unsigned char * out;
		size_t outsize;
		lodepng_encode_memory(&out, &outsize,
			raw_buffer, 
			static_cast<unsigned int>(imageWidth[i]), static_cast<unsigned int>(imageHeight[i]),
			(imageChannels[i]==4?LodePNGColorType::LCT_RGBA:LodePNGColorType::LCT_RGB), 8);
		std::ofstream outfile(getFileName(i), std::ofstream::binary);
		outfile.write(reinterpret_cast<char *>(out), outsize);
		free(out);
	};

	if(fileRawBuffer.is_open()) {
		fileRawBuffer.clear();
		fileRawBuffer.seekg(0);
	}



	for(size_t i = 0; i < imageStartIdx.size(); ++i) {
		if(imageFormat[i] == FORMAT_RAW && outputFormat == FORMAT_BMP) {
			// create image
			Image img(imageChannels[i], imageWidth[i], imageHeight[i], &imageBuffer[imageStartIdx[i]]);
			// write to file
			BMPIO b(getFileName(i).c_str());
			b.writeToFile(&img);

		}
		else if(imageFormat[i] == FORMAT_RAW && outputFormat == FORMAT_PNG) {
			writeIthPNG(i, &imageBuffer[imageStartIdx[i]]);
		}
		else if(imageFormat[i] == FORMAT_PNG && outputFormat == FORMAT_PNG) {
			std::ofstream outfile(getFileName(i), std::ofstream::binary);
			outfile.write(reinterpret_cast<char *>(&imageBuffer[imageStartIdx[i]]), imageEndIdx[i] - imageStartIdx[i]);
		}
		else if(imageFormat[i] == FORMAT_RAWFILE && outputFormat == FORMAT_PNG) {
			size_t imageSize = imageEndIdx[i] - imageStartIdx[i];
			singleRawBuffer.resize(imageSize);
			fileRawBuffer.read(reinterpret_cast<char*>(singleRawBuffer.data()), imageSize);
			writeIthPNG(i, singleRawBuffer.data());
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
	imageChannels.resize(0);

	if(fileRawBuffer.is_open()) {
		fileRawBuffer.clear();
		fileRawBuffer.seekg(0);
	}

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
	saveButton->setCallback([&](){if(!record){this->save(dirName, fileBaseName);}});

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
	infoLabel = new nanogui::Label(bufferStatus, "");
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
	bufferstat_sstream << "Buffer status: " << getBufferFilledSize()/1'000'000 << " / " << getBufferSize()/1'000'000 << " MB"
						<< "  images: " << imageStartIdx.size();
	bufferStatusLabel->setCaption(bufferstat_sstream.str());
	bufferFillStatusBar->setValue(getBufferFilledFraction());

	// info
	std::stringstream info_sstream;
	info_sstream << "time: " << static_cast<int>(time_recording_last*1000) << "ms"
				 << "  comp. ratio: " << std::setprecision(1) << std::fixed << compressionRatio_last
				 << "  size last: " << std::setprecision(1) << std::fixed << static_cast<double>(size_last_frame)/1.0e6 << "MB";
	
	infoLabel->setCaption(info_sstream.str());

	// delete
	deleteSlider->setValue(0.0f);

}




