#include <RobotDesignerLib/LivingHornBracket.h>
#include <MathLib/ConvexHull3D.h>
#include <GUILib/GLContentManager.h>

Motor_RMC_HornBracket::Motor_RMC_HornBracket(){

}

Motor_RMC_HornBracket::~Motor_RMC_HornBracket(void){

}

void Motor_RMC_HornBracket::generateBracketMesh() {
	if (!shouldRegenerateBracketMesh)
		return;
	shouldRegenerateBracketMesh = false;
	//apply the transformation due to the mounting angle
	Quaternion bracketMountingOrientation = getRotationQuaternion(bracketMountingAngle, V3D(0, 0, 1));
	Quaternion rotation = bracketMountingOrientation * bracketMesh->tmpAppliedRotation.getInverse();
	bracketMesh->rotate(rotation, P3D());
	bracketMesh->tmpAppliedRotation = bracketMountingOrientation;

	bracketMesh->computeNormals();
	bracketMesh->calBoundingBox();

	setDefaultFeaturePointList();

	pinPosition = P3D();
	pinOrientation = Quaternion();

	for (uint i = 0; i < featurePoints.size(); i++) {
		featurePoints[i] = bracketMountingOrientation * V3D(featurePoints[i]);
		pinPosition += featurePoints[i];
	}

	pinPosition /= featurePoints.size();
	pinOrientation = bracketMountingOrientation;
}


Transformation Motor_RMC_HornBracket::getPinTransformation(){
	return Transformation(pinOrientation.getRotationMatrix(), pinPosition);
}

void Motor_RMC_HornBracket::setColor(const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (color.isZero()){
		bracketMesh->setMaterial(shaderMaterial);
	}
	else {
		GLShaderMaterial defaultMat;
		defaultMat.setColor(color[0], color[1], color[2], color[3]);
		bracketMesh->setMaterial(defaultMat);
	}
}

void Motor_RMC_HornBracket::draw() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);

	glEnable(GL_LIGHTING);
	bracketMesh->drawMesh();
}

/* ---------------------------------------------------------------------------- */

Motor_RMC_HornBracket_XM430::Motor_RMC_HornBracket_XM430() : Motor_RMC_HornBracket() {
	shaderMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	string mat = "../data/textures/matcap/whitefluff2.bmp";
	shaderMaterial.setTextureParam(mat.c_str(), GLContentManager::getTexture(mat.c_str()));

//	this->bracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_hornBracket_w.obj")->clone();
	this->bracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_hornBracket_lo.obj")->clone();

	generateBracketMesh();
	bracketMesh->setMaterial(shaderMaterial);
}

Motor_RMC_HornBracket_XM430::~Motor_RMC_HornBracket_XM430(void){
	delete bracketMesh;
}

void Motor_RMC_HornBracket_XM430::setDefaultFeaturePointList() {
	featurePoints.clear();
	featurePoints.push_back(P3D(0.017, 0.022, 0.023));
	featurePoints.push_back(P3D(0.017, 0.022, -0.023));
	featurePoints.push_back(P3D(-0.017, 0.022, -0.023));
	featurePoints.push_back(P3D(-0.017, 0.022, 0.023));
}


/* ---------------------------------------------------------------------------- */
Motor_RMC_HornBracket_TGY306G::Motor_RMC_HornBracket_TGY306G() : Motor_RMC_HornBracket() {
	shaderMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	string mat = "../data/textures/matcap/whitefluff2.bmp";
	shaderMaterial.setTextureParam(mat.c_str(), GLContentManager::getTexture(mat.c_str()));

	this->bracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_hornBracket_w.obj")->clone();

	generateBracketMesh();
	bracketMesh->setMaterial(shaderMaterial);
}

Motor_RMC_HornBracket_TGY306G::~Motor_RMC_HornBracket_TGY306G(void) {
	delete bracketMesh;
}

void Motor_RMC_HornBracket_TGY306G::setDefaultFeaturePointList() {
	featurePoints.clear();
	double lenX = 0.008;
	double lenZ = 0.012;

	featurePoints.push_back(P3D(lenX, 0.02, lenZ));
	featurePoints.push_back(P3D(lenX, 0.02, -lenZ));
	featurePoints.push_back(P3D(-lenX, 0.02, -lenZ));
	featurePoints.push_back(P3D(-lenX, 0.02, lenZ));
}

/* ---------------------------------------------------------------------------- */
Motor_RMC_HornBracket_BK3002::Motor_RMC_HornBracket_BK3002() : Motor_RMC_HornBracket() {
	shaderMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	string mat = "../data/textures/matcap/blackFluff.bmp";
	shaderMaterial.setTextureParam(mat.c_str(), GLContentManager::getTexture(mat.c_str()));

	this->bracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/BK3002_hornBracket_d.obj")->clone();

	generateBracketMesh();
	bracketMesh->setMaterial(shaderMaterial);
}

Motor_RMC_HornBracket_BK3002::~Motor_RMC_HornBracket_BK3002(void) {
	delete bracketMesh;
}

void Motor_RMC_HornBracket_BK3002::setDefaultFeaturePointList() {
	featurePoints.clear();
	double lenX = 0.008;
	double lenZ = 0.012;

	featurePoints.push_back(P3D(lenX, 0.02, lenZ));
	featurePoints.push_back(P3D(lenX, 0.02, -lenZ));
	featurePoints.push_back(P3D(-lenX, 0.02, -lenZ));
	featurePoints.push_back(P3D(-lenX, 0.02, lenZ));
}

/*****************************************************************************************/

Motor_RMC_BodyBracket::Motor_RMC_BodyBracket() {

}

Motor_RMC_BodyBracket::~Motor_RMC_BodyBracket() {

}

void Motor_RMC_BodyBracket::draw() {
	if (bodyBracketMesh)
		bodyBracketMesh->drawMesh();
}

void Motor_RMC_BodyBracket::setColor(const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (color.isZero()){
		bodyBracketMesh->setMaterial(bodyBracketMaterial);
	}
	else {
		GLShaderMaterial defaultMat;
		defaultMat.setColor(color[0], color[1], color[2], color[3]);
		bodyBracketMesh->setMaterial(defaultMat);
	}
}

Motor_RMC_BodyBracket_XM430::Motor_RMC_BodyBracket_XM430() {
//	bodyBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_bodyBracket_w.obj"); bodyBracketMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);
	bodyBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/XM-430_bodyBracket_lo.obj"); bodyBracketMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

	string whiteMat = "../data/textures/matcap/whitefluff2.bmp";
	bodyBracketMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	bodyBracketMaterial.setTextureParam(whiteMat.c_str(), GLContentManager::getTexture(whiteMat.c_str()));

	generateBracketMesh();
}

Motor_RMC_BodyBracket_XM430::~Motor_RMC_BodyBracket_XM430() {

}

void Motor_RMC_BodyBracket_XM430::generateBracketMesh(){
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



Motor_RMC_BodyBracket_TGY306G::Motor_RMC_BodyBracket_TGY306G() {
	bodyBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/TGY306G_bodyBracket_w.obj"); bodyBracketMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

	string whiteMat = "../data/textures/matcap/whitefluff2.bmp";
	bodyBracketMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	bodyBracketMaterial.setTextureParam(whiteMat.c_str(), GLContentManager::getTexture(whiteMat.c_str()));

	generateBracketMesh();
}

Motor_RMC_BodyBracket_TGY306G::~Motor_RMC_BodyBracket_TGY306G() {

}

void Motor_RMC_BodyBracket_TGY306G::generateBracketMesh() {
	pinInfos.clear();

	P3D center = P3D(0, -0.024, 0);
	double lenX = 0.008;
	double lenZ = 0.012;

	Transformation pinTrans(getRotationQuaternion(RAD(180), V3D(1, 0, 0)).getRotationMatrix(), V3D(center));

	vector<P3D> FPs;
	FPs.push_back(center + P3D(lenX, 0, lenZ /*/ 2*/));
	FPs.push_back(center + P3D(-lenX, 0, lenZ /*/ 2*/));
	FPs.push_back(center + P3D(-lenX, 0, -lenZ));
	FPs.push_back(center + P3D(lenX, 0, -lenZ));

	pinInfos.push_back(PinInfo(pinTrans, "BottomBracketPin", FPs, center, V3D(0, -1, 0)));
}

Motor_RMC_BodyBracket_BK3002::Motor_RMC_BodyBracket_BK3002() {
	bodyBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/meshes/BK3002_bodyBracket_d.obj"); bodyBracketMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);

	string whiteMat = "../data/textures/matcap/blackFluff.bmp";
	bodyBracketMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	bodyBracketMaterial.setTextureParam(whiteMat.c_str(), GLContentManager::getTexture(whiteMat.c_str()));

	generateBracketMesh();
}

Motor_RMC_BodyBracket_BK3002::~Motor_RMC_BodyBracket_BK3002() {

}

void Motor_RMC_BodyBracket_BK3002::generateBracketMesh() {
	pinInfos.clear();

	P3D center = P3D(0, -0.024, 0);
	double lenX = 0.008;
	double lenZ = 0.012;

	Transformation pinTrans(getRotationQuaternion(RAD(180), V3D(1, 0, 0)).getRotationMatrix(), V3D(center));

	vector<P3D> FPs;
	FPs.push_back(center + P3D(lenX, 0, lenZ /*/ 2*/));
	FPs.push_back(center + P3D(-lenX, 0, lenZ /*/ 2*/));
	FPs.push_back(center + P3D(-lenX, 0, -lenZ));
	FPs.push_back(center + P3D(lenX, 0, -lenZ));

	pinInfos.push_back(PinInfo(pinTrans, "BottomBracketPin", FPs, center, V3D(0, -1, 0)));
}

