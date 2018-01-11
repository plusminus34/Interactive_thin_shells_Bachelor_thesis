#pragma once

#include <MathLib/V3D.h>
#include <MathLib/P3D.h>
#include <MathLib/boundingBox.h>
#include <GUILib/GLMesh.h>
#include <vector>

using namespace std;

//TODO: could have alternate body brackets that get instantiated based on which one of the pins is chosen...

struct PinInfo {
	Transformation trans;
	string name;
	vector<P3D> featurePoints;
	P3D center;
	V3D normal;

    PinInfo(const Transformation& _trans, const string& _name, const vector<P3D>& _featurePoints, const P3D& _center, const V3D& _normal)
		: trans(_trans), name(_name), featurePoints(_featurePoints), center(_center), normal(_normal) {}

};

class LivingHornBracket;

class LivingMotorBodyBracket{
public:
	LivingMotorBodyBracket();
	~LivingMotorBodyBracket(void);

	GLMesh* bodyBracketMesh = NULL;

	vector<PinInfo> pinInfos;

	LivingHornBracket* hornBracket = NULL;

	GLShaderMaterial bodyBracketMaterial;

	virtual void draw();
	virtual void setColor(const Vector4d& color = Vector4d(0, 0, 0, 0));

	virtual void generateBracketMesh() {};
};

class LivingMotorBodyBracket_XM430 : public LivingMotorBodyBracket {
public:

public:
	LivingMotorBodyBracket_XM430();
	~LivingMotorBodyBracket_XM430(void);

	void generateBracketMesh();
};

class LivingMotorBodyBracket_TGY306G : public LivingMotorBodyBracket {
public:

public:
	LivingMotorBodyBracket_TGY306G();
	~LivingMotorBodyBracket_TGY306G(void);

	void generateBracketMesh();
};

class LivingMotorBodyBracket_BK3002 : public LivingMotorBodyBracket {
public:

public:
	LivingMotorBodyBracket_BK3002();
	~LivingMotorBodyBracket_BK3002(void);

	void generateBracketMesh();
};
