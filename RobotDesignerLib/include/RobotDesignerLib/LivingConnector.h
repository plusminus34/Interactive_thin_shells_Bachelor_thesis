#pragma once
#include <RobotDesignerLib/RMC.h>

class LivingConnector : public RMC {
public:
	GLMesh* connectorMesh = NULL;
	double scale = 1.0;
public:
	LivingConnector();
	~LivingConnector();

	virtual LivingConnector* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	bool isFullyConnected();

	void updateMeshAndPinByDefault();
	void updateMeshAndPinImplicit();

	void exportMeshes(const char* dirName, int index);

	//should return true if it's parsed it, false otherwise...
	virtual bool interpretInputLine(FILE* fp, char* line) {
		char keyword[50];
		sscanf(line, "%s", keyword);

		if (strcmp(keyword, "Name") == 0)
			return true;

		if (RMC::interpretInputLine(fp, line))
			return true;

		if (strcmp(keyword, "Scale") == 0) {
			sscanf(line + strlen(keyword), "%lf", &scale);
			update();
			return true;
		}

		return false;
	}

	virtual void writeParamsToCommandLine(char* cmdLine) {
		//nothing to do... everything gets generated automatically
	}

	virtual void readParamsFromCommandLine(char* cmdLine) {
		//nothing to do... everything gets generated automatically
	}

};

class ConnectorHUB_RMC : public RMC {
public:
	GLMesh* mesh = NULL;

	double size = 0.0075;
	std::string connectorMeshName;

public:
	ConnectorHUB_RMC();
	~ConnectorHUB_RMC();

	virtual ConnectorHUB_RMC* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	virtual void syncSymmParameters(RMC* refEE);
	void exportMeshes(const char* dirName, int index);

	//should return true if it's parsed it, false otherwise...
	virtual bool interpretInputLine(FILE* fp, char* line) {
		char keyword[50];
		sscanf(line, "%s", keyword);

		if (RMC::interpretInputLine(fp, line))
			return true;

		if (strcmp(keyword, "Size") == 0) {
			sscanf(line + strlen(keyword), "%lf", &size);
			update();
			return true;
		}

		if (strcmp(keyword, "ConnectorMeshName") == 0) {
			char tmpName[200];
			sscanf(line + strlen(keyword), "%s", tmpName);
			connectorMeshName = tmpName;
			update();
			return true;
		}

		return false;
	}

	virtual void processInputKeyPress(int key) {
		if (key == GLFW_KEY_EQUAL)
			size = max(0.005, size - 0.005);
		if (key == GLFW_KEY_MINUS)
			size = size + 0.005;
	}

	virtual void writeParamsToCommandLine(char* cmdLine) {
		Quaternion q = state.orientation;
		P3D pos = state.position;
		sprintf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf %lf ", q[0], q[1], q[2], q[3],
			pos[0], pos[1], pos[2],
			size);
	}

	virtual void readParamsFromCommandLine(char* cmdLine) {
		sscanf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf %lf",
			&state.orientation[0], &state.orientation[1], &state.orientation[2], &state.orientation[3],
			&state.position[0], &state.position[1], &state.position[2],
			&size);
	}

};

