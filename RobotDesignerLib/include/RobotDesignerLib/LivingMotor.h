#pragma once
#include <RobotDesignerLib/RMC.h>
#include <RobotDesignerLib/LivingBracket.h>

class Motor_RMC : public RMC{
public:
	Motor_RMC_BodyBracket* bodyBracket = NULL;
	Motor_RMC_HornBracket* hornBracket = NULL;

	GLMesh* motorBodyMesh = NULL;
	GLMesh* motorHornMesh = NULL;
	GLShaderMaterial bodyMaterial;
	GLShaderMaterial hornMaterial;

	vector<RMCPin> candidatePins;
	int activeBodyPinID = -1;

	string LMType;

	V3D motorAxis;
	double motorAngle = 0;
	double backupMotorAngle = 0;

public:
	Motor_RMC(const char* LMType);
	~Motor_RMC();

	virtual Motor_RMC* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	virtual void generatePins();

	void exportMeshes(const char* dirName, int index);

	void syncSymmParameters(RMC* other);

	void switchToBestBodyPin();

	void setColor(const Vector4d& color = Vector4d(0, 0, 0, 0));

	//should return true if it's parsed it, false otherwise...
	virtual bool interpretInputLine(FILE* fp, char* line) {
		char keyword[50];
		sscanf(line, "%s", keyword);

		if (strcmp(keyword, "Name") == 0)
			return true;

		if (RMC::interpretInputLine(fp, line))
			return true;
		
		if (strcmp(keyword, "MotorAxis") == 0){
			sscanf(line + strlen(keyword), "%lf %lf %lf", &motorAxis[0], &motorAxis[1], &motorAxis[2]);
			return true;
		}

		return false;
	}

	virtual void processInputKeyPress(int key) {
		if (key == GLFW_KEY_LEFT_BRACKET) {
			hornBracket->goToNextMountingPosition();
			Logger::consolePrint("horn bracket mounting angle: %lf\n", hornBracket->bracketMountingAngle);
		}
		if (key == GLFW_KEY_RIGHT_BRACKET) {
			hornBracket->goToPreviousMountingPosition();
			Logger::consolePrint("horn bracket moounting angle: %lf\n", hornBracket->bracketMountingAngle);
		}
		if (key == GLFW_KEY_MINUS) {
			motorAngle += 5;
			Logger::consolePrint("Motor angle is now: %lf\n", motorAngle);
		}
		if (key == GLFW_KEY_EQUAL) {
			motorAngle -= 5;
			Logger::consolePrint("Motor angle is now: %lf\n", motorAngle);
		}
		if (key == GLFW_KEY_0) {
			motorAngle = 0;
			Logger::consolePrint("Motor angle is now: %lf\n", motorAngle);
		}
	}

	virtual void writeParamsToCommandLine(char* cmdLine) {
		Quaternion q = state.orientation;
		P3D pos = state.position;
		sprintf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf %lf %lf ", q[0], q[1], q[2], q[3], 
			pos[0], pos[1], pos[2],
			motorAngle, hornBracket->bracketMountingAngle);
	}

	virtual void readParamsFromCommandLine(char* cmdLine) {
		sscanf(cmdLine, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&state.orientation[0], &state.orientation[1], &state.orientation[2], &state.orientation[3],
			&state.position[0], &state.position[1], &state.position[2],
			&motorAngle, &hornBracket->bracketMountingAngle);
		hornBracket->shouldRegenerateBracketMesh = true;
	}

};

