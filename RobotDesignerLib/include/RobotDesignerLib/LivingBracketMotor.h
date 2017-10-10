#pragma once

#include <MathLib/V3D.h>
#include <MathLib/P3D.h>
#include <MathLib/boundingBox.h>
#include <GUILib/GLMesh.h>
#include <vector>

using namespace std;

struct PinInfo 
{
	Transformation trans;
	string name;
	vector<P3D> featurePoints;
	P3D center;
	V3D normal;

	PinInfo(const Transformation& _trans, const string& _name, vector<P3D>& _featurePoints, P3D& _center, V3D& _normal)
		: trans(_trans), name(_name), featurePoints(_featurePoints), center(_center), normal(_normal) {}

};

class LivingHornBracket;

class LivingBracketMotor{
public:
	LivingBracketMotor();
	~LivingBracketMotor(void);

	//range of motion of the bracket relative to the motor it will connect to
	double rotAngleMin = -1.4;
	double rotAngleMax = 1.4;

	double rotAngle = 0;

	double hornRadius = 0.1;
	double hornThickness = 0.02;
	//We assume that the motor is enclosed into an axis-aligned box - this will be used to figure out the geometry of the bracket...
	AxisAlignedBoundingBox boundingBox = AxisAlignedBoundingBox(P3D(-0.2, -0.5, -0.2), P3D(0.2, 0.2, 0.2));

	GLMesh* motorBodyMesh = NULL;
	GLMesh* motorHornMesh = NULL;
	GLMesh* motorWholeMesh = NULL;
	GLMesh* bodyBracketMesh = NULL;
	GLMesh* hornCarvingMesh = NULL;
	GLMesh* bodyCarvingMesh = NULL;

	vector<GLMesh*> bracketCarvingMeshes;
	vector<PinInfo> pinInfos;

	LivingHornBracket* hornBracket = NULL;

	GLShaderMaterial bodyMaterial;
	GLShaderMaterial hornMaterial;
	GLShaderMaterial bodyBracketMaterial;

	virtual void draw();
	virtual void setColor(const Vector4d& color = Vector4d(0, 0, 0, 0));

	virtual void generateBodyBracketMeshes() {};
};

class LivingBracketMotor_XM430 : public LivingBracketMotor {
public:
	LivingBracketMotor_XM430();
	~LivingBracketMotor_XM430(void);

	void generateBodyBracketMeshes();
};

