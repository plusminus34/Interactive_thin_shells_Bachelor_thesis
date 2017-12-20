#pragma once
#include <RobotDesignerLib/RMC.h>

class LivingMotor : public RMC
{
public:
	LivingBracketMotor* motor = NULL;
	LivingHornBracket* bracket = NULL;
	vector<RMCPin> candidatePins;
	int activeBodyPinID = -1;

public:
	LivingMotor(LivingHornBracket* lbh = NULL);
	~LivingMotor();

	virtual LivingMotor* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	virtual void generatePins();

	void exportMeshes(const char* dirName, int index);
	void syncSymmParameters(LivingMotor* refMotor);
	void switchToBestBodyPin();
};

