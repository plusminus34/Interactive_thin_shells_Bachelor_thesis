#pragma once


#include <string>
#include <map>
#include <GUILib/GLApplication.h>
#include <GUILib/GLWindowContainer.h>
#include <GUILib/TranslateWidget.h>
#include <GUILib/RotateWidgetV1.h>
#include <GUILib/RotateWidgetV2.h>
#include <RobotDesignerLib/RMCRobot.h>
#include <RobotDesignerLib/RMC.h>
#include <ControlLib/Robot.h>
#include <RobotDesignerLib/AbstractDesignWindow.h>

#define SNAP_THRESHOLD 0.03 //0.01

using namespace std;

/**
 * Test App
 */
class ModularDesignWindow : public AbstractDesignWindow {
	friend class RobotDesignerApp;
private:
	GLApplication* glApp;
	GLWindowContainer* componentLibrary = NULL;
	TranslateWidget* tWidget = NULL;
	RotateWidget* rWidget = NULL;
	Timer timer;
	
	int iterNum = 1;
	
	double bodyPlaneHeight = 0;

	bool dragging = false;
	bool snappable = false;

	bool runTask = false;
	bool noMirror = false;

public:
	vector<RMCRobot*> rmcRobots;
	vector<RMC*> rmcWarehouse;
	map<string, RMC*> rmcNameMap;
	map<string, RMCPin*> rmcPinNameMap;
	map<string, vector<Transformation>> transformationMap;
	map<RMCRobot*, RMCRobot*> mirrorMap;
	map<RMC*, RMC*> rmcMirrorMap; // mainly used for living brackets
	map<RBFeaturePoint*, RBFeaturePoint*> bodyFpMirrorMap;

	vector<PossibleConnection> possibleConnections; // possible transformation for current selected RMCRobot
	RMCRobot* selectedRobot = NULL;
	RMCRobot* hightlightedRobot = NULL;
	RMCRobot* windowSelectedRobot = NULL;

	GLMesh* bodyMesh = NULL;
	bool bodyMeshSelected = false;
	GLShaderMaterial bodyMaterial;

	vector<RBFeaturePoint> bodyFeaturePts;
	RBFeaturePoint* highlightedFP = NULL;
	RBFeaturePoint* selectedFP = NULL;
	GLMesh* sphereMesh = NULL;
	bool showBodyFeature = true;

	GLMesh* guidingMesh = NULL;
	bool pickedGuidingMesh = false;
	P3D guidingMeshPos;
	Quaternion guidingMeshRot;
	double guidingMeshScale = 1.0;
	
	string robotMeshDir = "../out/";
	string configFileName;

	bool hasDesign() {
		return rmcRobots.size() > 0;
	}
public:
	// constructor
	ModularDesignWindow(int x, int y, int w, int h, GLApplication* glApp, const char* libraryDefinitionFileName);
	// destructor
	virtual ~ModularDesignWindow(void);

	// Draw the AppRobotDesigner scene - camera transformations, lighting, shadows, reflections, etc AppRobotDesignerly to everything drawn by this method
	virtual void drawScene();
	// This is the wild west of drawing - things that want to ignore depth buffer, camera transformations, etc. Not pretty, quite hacky, but flexible. Individual apps should be careful with implementing this method. It always gets called right at the end of the draw function
	virtual void drawAuxiliarySceneInfo();

	//mouse & keyboard event callbacks...

	//all these methods should returns true if the event is processed, false otherwise...
	//any time a physical key is pressed, this event will trigger. Useful for reading off special keys...
	virtual bool onKeyEvent(int key, int action, int mods);
	//this one gets triggered on UNICODE characters only...
	virtual bool onCharacterPressedEvent(int key, int mods);
	//triggered when mouse buttons are pressed
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	//triggered when mouse moves
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	//triggered when using the mouse wheel
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);

	virtual void saveFile(const char* fName);
	virtual void loadFile(const char* fName);

	void removeRMCRobot(RMCRobot* robot);

	void drawRefAxis(const P3D& pos);
	void drawRMCRobot();
	void drawConnectionPreview();
	void drawWindowRMCConnectionPreview();
	void drawBodyPlane();
	void drawGuildingMesh();
	void drawBodyFeaturePts();

	void loadConfig(const char* fName);
	void loadTransformationMap(FILE* fp);

	void saveDesignToFile(const char* fName);
	void loadDesignFromFile(const char* fName);

	void saveToRBSFile(const char* fName, Robot* templateRobot = NULL);

	// transform the child RMC to get a preview.
	bool previewConnectRMCRobot(RMCPin* parentPin, RMCPin* childPin, RMCRobot* childRobot, bool rotationOnly);

	PossibleConnection* getClosestConnnection(Ray& ray, vector<PossibleConnection>& connections, P3D& closestPoint, double& closestDist);

	void createBodyMesh2D();
	void createBodyMesh3D();

	bool process();

	bool isSelectedRMCMovable();

	void matchDesignWithRobot(Robot* tRobot, RobotState* initialRobotState);
	void transferMeshes(Robot* tRobot, RobotState* initialRobotState);

	void buildRMCMirrorMap();
	void makeSelectedRMCSymmetric();
	void propagatePosToMirrorRMC(RMC* rmc);
	void propagateOrientToMirrorRMC(RMC* rmc);

	void makeRMCsSymmetricRecursive(RMC* originalRMC, RMC* mirroredRMC);

	void updateParentConnector(RMC* rmc);

	void pickBodyFeaturePts(Ray& ray);
	void makeSelectedFPSymmtric();
	void propagatePosToMirrorFp(RBFeaturePoint* fp);

private:

};



