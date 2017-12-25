#pragma once
#include <RobotDesignerLib/RMC.h>
#include <RobotDesignerLib/LivingHornBracket.h>
#include <RobotDesignerLib/LivingBracketMotor.h>

class LivingMotor : public RMC{
public:
	LivingMotorBodyBracket* bodyBracket = NULL;
	LivingHornBracket* hornBracket = NULL;

	GLMesh* motorBodyMesh = NULL;
	GLMesh* motorHornMesh = NULL;
	GLShaderMaterial bodyMaterial;
	GLShaderMaterial hornMaterial;

	vector<RMCPin> candidatePins;
	int activeBodyPinID = -1;

public:
	LivingMotor();
	~LivingMotor();

	virtual LivingMotor* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	virtual void generatePins();

	void exportMeshes(const char* dirName, int index);
	void syncSymmParameters(LivingMotor* refMotor);
	void switchToBestBodyPin();

	void setColor(const Vector4d& color = Vector4d(0, 0, 0, 0));

};

