#include <GUILib/GLUtils.h>

#include <RobotDesignerLib/LivingBracketMotor.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/Quaternion.h>
#include <RobotDesignerLib/LivingHornBracket.h>

LivingMotorBodyBracket::LivingMotorBodyBracket() {
}

LivingMotorBodyBracket::~LivingMotorBodyBracket() {

}

void LivingMotorBodyBracket::draw() {
	if (bodyBracketMesh)
		bodyBracketMesh->drawMesh();
}

void LivingMotorBodyBracket::setColor(const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (color.isZero()){
		bodyBracketMesh->setMaterial(bodyBracketMaterial);
	}
	else {
		GLShaderMaterial defaultMat;
		defaultMat.setColor(color[0], color[1], color[2], color[3]);
		bodyBracketMesh->setMaterial(defaultMat);
	}
}

LivingMotorBodyBracket_XM430::LivingMotorBodyBracket_XM430() {
	bodyBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_bodyBracket_w.obj"); bodyBracketMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

	string whiteMat = "../data/textures/matcap/whitefluff2.bmp";
	bodyBracketMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	bodyBracketMaterial.setTextureParam(whiteMat.c_str(), GLContentManager::getTexture(whiteMat.c_str()));

	generateBracketMesh();
}

LivingMotorBodyBracket_XM430::~LivingMotorBodyBracket_XM430() {

}

void LivingMotorBodyBracket_XM430::generateBracketMesh(){
	pinInfos.clear();

	P3D center = P3D(0, -0.038, 0);
	double lenX = 0.0175;
	double lenZ = 0.0205;

	Transformation pinTrans(getRotationQuaternion(RAD(180), V3D(1, 0, 0)).getRotationMatrix(), V3D(center));

	vector<P3D> FPs;
	FPs.push_back(center + P3D(lenX, 0, lenZ));
	FPs.push_back(center + P3D(-lenX, 0, lenZ));
	FPs.push_back(center + P3D(-lenX, 0, -lenZ));
	FPs.push_back(center + P3D(lenX, 0, -lenZ));

	pinInfos.push_back(PinInfo(pinTrans, "BottomBracketPin", FPs, center, V3D(0, -1, 0)));
}



LivingMotorBodyBracket_TGY306G::LivingMotorBodyBracket_TGY306G() {
	bodyBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_bodyBracket_w.obj"); bodyBracketMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

	string whiteMat = "../data/textures/matcap/whitefluff2.bmp";
	bodyBracketMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	bodyBracketMaterial.setTextureParam(whiteMat.c_str(), GLContentManager::getTexture(whiteMat.c_str()));

	generateBracketMesh();
}

LivingMotorBodyBracket_TGY306G::~LivingMotorBodyBracket_TGY306G() {

}

void LivingMotorBodyBracket_TGY306G::generateBracketMesh() {
	pinInfos.clear();

	P3D center = P3D(0, -0.022, 0);
	double lenX = 0.008;
	double lenZ = 0.015;

	Transformation pinTrans(getRotationQuaternion(RAD(180), V3D(1, 0, 0)).getRotationMatrix(), V3D(center));

	vector<P3D> FPs;
	FPs.push_back(center + P3D(lenX, 0, lenZ /*/ 2*/));
	FPs.push_back(center + P3D(-lenX, 0, lenZ /*/ 2*/));
	FPs.push_back(center + P3D(-lenX, 0, -lenZ));
	FPs.push_back(center + P3D(lenX, 0, -lenZ));

	pinInfos.push_back(PinInfo(pinTrans, "BottomBracketPin", FPs, center, V3D(0, -1, 0)));
}
