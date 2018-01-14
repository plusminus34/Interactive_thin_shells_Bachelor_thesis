#pragma once
#include <RobotDesignerLib/RMC.h>

class LivingSphereEE : public RMC
{
public:
	GLMesh* eeMesh = NULL;

	double sphereRadius = 0.03;

public:
	LivingSphereEE();
	~LivingSphereEE();

	virtual LivingSphereEE* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	void syncSymmParameters(LivingSphereEE* refEE);
	void exportMeshes(const char* dirName, int index);
};

class Living6FaceConnector : public RMC
{
public:
	GLMesh* mesh = NULL;

	double size = 0.0075;

public:
	Living6FaceConnector();
	~Living6FaceConnector();

	virtual Living6FaceConnector* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	void syncSymmParameters(Living6FaceConnector* refEE);
	void exportMeshes(const char* dirName, int index);
};


class LivingWheelEE : public RMC
{
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
	LivingWheelEE(const char* LMType);
	~LivingWheelEE();

	virtual LivingWheelEE* clone();
	virtual bool pickMesh(Ray& ray, double* closestDist = NULL);
	virtual void draw(int flags, const Vector4d& color = Vector4d(0, 0, 0, 0));
	virtual void update();

	void syncSymmParameters(LivingWheelEE* refEE);
	void exportMeshes(const char* dirName, int index);
};
