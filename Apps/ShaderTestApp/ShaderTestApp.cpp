#include <GUILib/GLUtils.h>
#include "ShaderTestApp.h"
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>

ShaderTestApp::ShaderTestApp() {
	setWindowTitle("Test Application for shaders and materials...");

	//add menu entries for the objects/textures that will be loaded...
//	TwAddVarRW(mainMenuBar, "LoadedObjs", TwDefineEnumFromString("ObjFiles", "No OBJs loaded"), &selectedObjFile, NULL);
//	TwAddVarRW(mainMenuBar, "LoadedMaterials", TwDefineEnumFromString("Materials", "No materials loaded"), &selectedMaterialTexture, NULL);
    
//	TwAddVarRW(mainMenuBar, "obj color", TW_TYPE_COLOR4F, &modelColor, " label='model color' ");

	showGroundPlane = false;

	menuScreen->performLayout();


}

ShaderTestApp::~ShaderTestApp(void){
}


//triggered when mouse moves
bool ShaderTestApp::onMouseMoveEvent(double xPos, double yPos) {
	if (GLApplication::onMouseMoveEvent(xPos, yPos) == true) return true;

	return false;
}

//triggered when mouse buttons are pressed
bool ShaderTestApp::onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos) {
	if (GLApplication::onMouseButtonEvent(button, action, mods, xPos, yPos)) return true;

	return false;
}

//triggered when using the mouse wheel
bool ShaderTestApp::onMouseWheelScrollEvent(double xOffset, double yOffset) {
	if (GLApplication::onMouseWheelScrollEvent(xOffset, yOffset)) return true;

	return false;
}

bool ShaderTestApp::onKeyEvent(int key, int action, int mods) {
	if (GLApplication::onKeyEvent(key, action, mods)) return true;

	return false;
}

bool ShaderTestApp::onCharacterPressedEvent(int key, int mods) {
	if (GLApplication::onCharacterPressedEvent(key, mods)) return true;

	return false;
}

void ShaderTestApp::loadFile(const char* fName) {
	Logger::consolePrint("Loading file \'%s\'...\n", fName);
	std::string fileName;
	fileName.assign(fName);

	std::string fNameExt = fileName.substr(fileName.find_last_of('.') + 1);

	if (fNameExt.compare("obj") == 0) {
		loadedObjs.push_back(fileName);
		selectedObjFile = (loadedObjs.size() - 1);
//		generateMenuEnumFromFileList("MainMenuBar/LoadedObjs", loadedObjs);
		return;
	}

	if (fNameExt.compare("bmp") == 0) {
		materialTextures.push_back(fileName);
		selectedMaterialTexture = (materialTextures.size() - 1);
//		generateMenuEnumFromFileList("MainMenuBar/LoadedMaterials", materialTextures);
		return;
	}

}

void ShaderTestApp::saveFile(const char* fName) {
	Logger::consolePrint("SAVE FILE: Do not know what to do with file \'%s\'\n", fName);
}


// Run the App tasks
void ShaderTestApp::process() {
	//do the work here...
}

// Draw the App scene - camera transformations, lighting, shadows, reflections, etc apply to everything drawn by this method
void ShaderTestApp::drawScene() {
	glDisable(GL_LIGHTING);
	glDisable(GL_TEXTURE_2D);
	glColor3d(1,1,1);

	glEnable(GL_LIGHTING);


	GLShaderMaterial tmpMat;
	tmpMat.r = modelColor[0]; tmpMat.g = modelColor[1]; tmpMat.b = modelColor[2]; tmpMat.a = modelColor[3];
	if (materialTextures.size() > 0) {
		tmpMat.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
		std::string textureFileName = materialTextures[selectedMaterialTexture];
		tmpMat.setTextureParam(textureFileName.c_str(), GLContentManager::getTexture(textureFileName.c_str()));
	}
	else {
		// TODO: use CMake to copy resource files to build dir (https://stackoverflow.com/a/18047175)
		FILE* fp = fopen("../data/shaders/materials/toon.mat", "r");
		tmpMat.readFromFile(fp);
		fclose(fp);
	}

	if (loadedObjs.size() > 0) {
		std::string objFileName = loadedObjs[selectedObjFile];
		if (objFileName.length() > (uint)0) {
			GLMesh* theMesh = GLContentManager::getGLMesh(objFileName.c_str());
			theMesh->setMaterial(tmpMat);
			theMesh->drawMesh();
		}
	}
	else {
		tmpMat.apply();
		drawSphere(P3D(0,0,0), 0.5, 20);
//		drawCylinder(P3D(0, 1, 0), P3D(0,2,0), 0.5, nSphereDiscretizaiton);

		tmpMat.end();
	}
}

// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
void ShaderTestApp::drawAuxiliarySceneInfo() {

}

// Restart the application.
void ShaderTestApp::restart() {

}

bool ShaderTestApp::processCommandLine(const std::string& cmdLine) {

	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

