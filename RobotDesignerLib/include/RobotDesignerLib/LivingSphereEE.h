#pragma once
#include <RobotDesignerLib/RMC.h>

class SphereEE_RMC : public RMC{
public:
	GLMesh* eeMesh = NULL;

	double sphereRadius = 0.01;

public:
	SphereEE_RMC();
	~SphereEE_RMC();

	virtual SphereEE_RMC* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	void syncSymmParameters(RMC* refEE);
	void exportMeshes(const char* dirName, int index);

	//should return true if it's parsed it, false otherwise...
	virtual bool interpretInputLine(FILE* fp, char* line) {
		char keyword[50];
		sscanf(line, "%s", keyword);

		if (strcmp(keyword, "Name") == 0)
			return true;

		if (RMC::interpretInputLine(fp, line))
			return true;

		if (strcmp(keyword, "Radius") == 0) {
			sscanf(line + strlen(keyword), "%lf", &sphereRadius);
			update();
			return true;
		}

		return false;
	}

	virtual void processInputKeyPress(int key) {
		if (key == GLFW_KEY_EQUAL)
			sphereRadius = max(0.005, sphereRadius - 0.005);
		if (key == GLFW_KEY_MINUS)
			sphereRadius = sphereRadius + 0.005;

	}

	virtual void writeParamsToCommandLine(char* cmdLine) {
		Quaternion q = state.orientation;
		P3D pos = state.position;
		sprintf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf %lf ", q[0], q[1], q[2], q[3],
			pos[0], pos[1], pos[2],
			sphereRadius);
	}

	virtual void readParamsFromCommandLine(char* cmdLine) {
		sscanf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf %lf",
			&state.orientation[0], &state.orientation[1], &state.orientation[2], &state.orientation[3],
			&state.position[0], &state.position[1], &state.position[2],
			&sphereRadius);
	}

};

class WheelEE_RMC : public RMC{
public:
	GLMesh* originalWheelMesh = NULL;
	GLMesh* wheelMesh = NULL;

	GLShaderMaterial bodyMaterial;
	GLMesh* motorMesh = NULL;
	GLMesh* motorBracketMesh = NULL;

	double radius = 0.05; // the radius is specified in meters
	bool isActive = true;
	string LMType;

public:
	WheelEE_RMC(const char* LMType);
	~WheelEE_RMC();

	virtual WheelEE_RMC* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	void syncSymmParameters(RMC* refEE);
	void exportMeshes(const char* dirName, int index);

	//should return true if it's parsed it, false otherwise...
	virtual bool interpretInputLine(FILE* fp, char* line) {
		char keyword[50];
		sscanf(line, "%s", keyword);

		if (strcmp(keyword, "Name") == 0)
			return true;

		if (RMC::interpretInputLine(fp, line))
			return true;

		if (strcmp(keyword, "Radius") == 0) {
			sscanf(line + strlen(keyword), "%lf", &radius);
			update();
			return true;
		}

		return false;
	}

	virtual void processInputKeyPress(int key) {
		if (key == GLFW_KEY_EQUAL)
			radius = max(0.005, radius - 0.005);
		if (key == GLFW_KEY_MINUS)
			radius = radius + 0.005;
	}

	virtual void writeParamsToCommandLine(char* cmdLine) {
		Quaternion q = state.orientation;
		P3D pos = state.position;
		sprintf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf %lf ", q[0], q[1], q[2], q[3],
			pos[0], pos[1], pos[2],
			radius);
	}

	virtual void readParamsFromCommandLine(char* cmdLine) {
		sscanf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf %lf",
			&state.orientation[0], &state.orientation[1], &state.orientation[2], &state.orientation[3],
			&state.position[0], &state.position[1], &state.position[2],
			&radius);
	}
};
