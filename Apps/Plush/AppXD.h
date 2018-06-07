

#include "PlushApplication.h"
#include "CSTSimulationMesh2D.h"
#include "CSTSimulationMesh3D.h"

class AppXD : public PlushApplication {

public: 
	AppXD();
	inline virtual ~AppXD(void) {}   // I never write destructors, and rarely free memory.
	// Process, draw, process, draw, process, draw...
	virtual void process();
	virtual void drawScene();

public:
    CSTSimulationMesh2D *tri_mesh;
    CSTSimulationMesh3D *tet_mesh; 

public:
	inline virtual void drawAuxiliarySceneInfo() {}

	virtual bool onKeyEvent(int key, int action, int mods);
	virtual bool onCharacterPressedEvent(int key, int mods);
	virtual bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	virtual bool onMouseMoveEvent(double xPos, double yPos);
	virtual bool onMouseWheelScrollEvent(double xOffset, double yOffset);
	virtual bool processCommandLine(const std::string& cmdLine);
	
	inline virtual void restart() {}
	inline virtual void saveFile(const char* fName) {}
	inline virtual void loadFile(const char* fName) {}
};



