#pragma once
#include <RobotDesignerLib/RMC.h>

class LivingConnector : public RMC
{
public:
	GLMesh* connectorMesh = NULL;

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
};

