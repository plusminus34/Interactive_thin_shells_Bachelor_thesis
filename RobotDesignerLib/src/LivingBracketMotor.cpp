#include <GUILib/GLUtils.h>

#include <RobotDesignerLib/LivingBracketMotor.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/Quaternion.h>
#include <RobotDesignerLib/LivingHornBracket.h>

LivingBracketMotor::LivingBracketMotor() {
}

LivingBracketMotor::~LivingBracketMotor() {

}

void LivingBracketMotor::draw() {
	glEnable(GL_LIGHTING);
	if (motorBodyMesh)
		motorBodyMesh->drawMesh();
	glDisable(GL_LIGHTING);

	double hornOffset = boundingBox.halfSides().z();

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	drawCylinder(P3D(0, 0, hornOffset), P3D(0, 0, hornOffset + hornThickness), hornRadius, 12);
	drawCylinder(P3D(0, 0, -hornOffset), P3D(0, 0, -hornOffset - hornThickness), hornRadius, 12);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glPushMatrix();
	glRotated(DEG(rotAngle), 0, 0, 1);
	if (hornBracket)
		hornBracket->draw();
	glEnable(GL_LIGHTING);
	if (motorHornMesh)
		motorHornMesh->drawMesh();
	if (bodyBracketMesh)
		bodyBracketMesh->drawMesh();
	glDisable(GL_LIGHTING);
	glPopMatrix();
}

void LivingBracketMotor::setColor(const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/)
{
	if (color.isZero())
	{
		motorBodyMesh->setMaterial(bodyMaterial);
		motorHornMesh->setMaterial(hornMaterial);
		bodyBracketMesh->setMaterial(bodyBracketMaterial);
	}
	else {
		GLShaderMaterial defaultMat;
		defaultMat.setColor(color[0], color[1], color[2], color[3]);
		motorBodyMesh->setMaterial(defaultMat);
		motorHornMesh->setMaterial(defaultMat);
		bodyBracketMesh->setMaterial(defaultMat);
	}
}

LivingBracketMotor_XM430::LivingBracketMotor_XM430() {
	motorBodyMesh = GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430_parent.obj"); motorBodyMesh->getMaterial().setColor(0.15, 0.15, 0.15, 1.0);
	motorHornMesh = GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430_child.obj"); motorHornMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);
	motorWholeMesh = GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430.obj"); motorWholeMesh->getMaterial().setColor(0.15, 0.15, 0.15, 1.0);
	bodyBracketMesh = GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430-BodyBracket.obj"); bodyBracketMesh->getMaterial().setColor(0.7, 0.7, 0.7, 1.0);
	hornCarvingMesh = GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430_hornCarving.obj");
	// bodyCarvingMesh = GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430_bodyCarving.obj");

	bodyMaterial.setColor(0.15, 0.15, 0.15, 1.0);
	hornMaterial.setColor(0.7, 0.7, 0.7, 1.0);
	string whiteMat = "../data/textures/matcap/whitefluff2.bmp";
	bodyBracketMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	bodyBracketMaterial.setTextureParam(whiteMat.c_str(), GLContentManager::getTexture(whiteMat.c_str()));


	boundingBox = AxisAlignedBoundingBox(P3D(-0.0143, -0.0353, -0.0172), P3D(0.0143, 0.0113, 0.0172));
	hornRadius = 0.01;
	hornThickness = 0.002;

	generateBodyBracketMeshes();
}

LivingBracketMotor_XM430::~LivingBracketMotor_XM430() {

}

void LivingBracketMotor_XM430::generateBodyBracketMeshes()
{
	pinInfos.clear();

	double hornStart = boundingBox.halfSides().z();
	double hornEnd = hornStart + 2 * hornThickness;
	double clearance = 0.0005;

	{
		P3D p1 = boundingBox.bmin();
		P3D p2 = boundingBox.bmax();
		p1[1] -= clearance;
		p2[1] = p1[1];

		P3D center = (p1 + p2) * 0.5;
		Transformation pinTrans(getRotationQuaternion(RAD(180), V3D(1, 0, 0)).getRotationMatrix(), V3D(center));

		vector<P3D> FPs;
		FPs.push_back(p1);
		FPs.push_back(P3D(p2[0], p1[1], p1[2]));
		FPs.push_back(P3D(p2[0], p1[1], p2[2]));
		FPs.push_back(P3D(p1[0], p1[1], p2[2]));

		pinInfos.push_back(PinInfo(pinTrans, "BottomBracketPin", FPs, center, V3D(0, -1, 0)));
	
		bracketCarvingMeshes.push_back(GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430_bottomCarving.obj"));
	}

	{
		P3D p1 = boundingBox.bmin();
		P3D p2 = boundingBox.bmax();
		p1[0] -= clearance;
		p2[0] = p1[0];
		P3D center = (p1 + p2) * 0.5;
		Transformation pinTrans(getRotationQuaternion(RAD(90), V3D(0, 0, 1)).getRotationMatrix(), V3D(center));

		vector<P3D> FPs;
		FPs.push_back(p1);
		FPs.push_back(P3D(p1[0], p2[1], p1[2]));
		FPs.push_back(P3D(p1[0], p2[1], p2[2]));
		FPs.push_back(P3D(p1[0], p1[1], p2[2]));
		
		pinInfos.push_back(PinInfo(pinTrans, "LeftBracketPin", FPs, center, V3D(-1, 0, 0)));
	
		bracketCarvingMeshes.push_back(GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430_leftCarving.obj"));
	}

	{
		P3D p1 = boundingBox.bmin();
		P3D p2 = boundingBox.bmax();
		p2[0] += clearance;
		p1[0] = p2[0];
		P3D center = (p1 + p2) * 0.5;
		Transformation pinTrans(getRotationQuaternion(RAD(-90), V3D(0, 0, 1)).getRotationMatrix(), V3D(center));

		vector<P3D> FPs;
		FPs.push_back(p2);
		FPs.push_back(P3D(p2[0], p2[1], p1[2]));
		FPs.push_back(P3D(p2[0], p1[1], p1[2]));
		FPs.push_back(P3D(p2[0], p1[1], p2[2]));

		pinInfos.push_back(PinInfo(pinTrans, "RightBracketPin", FPs, center, V3D(1, 0, 0)));
	
		bracketCarvingMeshes.push_back(GLContentManager::getGLMesh("../data/robotDesigner/motorMeshes/XM-430_rightCarving.obj"));
	}

	/*{
		P3D p1 = boundingBox.bmin();
		P3D p2 = boundingBox.bmax();
		p1[2] -= clearance;
		p2[2] = p1[2];
		P3D center = (p1 + p2) * 0.5;
		Transformation pinTrans(getRotationQuaternion(RAD(-90), V3D(1, 0, 0)).getRotationMatrix(), V3D(center));

		vector<P3D> FPs;
		FPs.push_back(p1);
		FPs.push_back(P3D(p2[0], p1[1], p1[2]));
		FPs.push_back(P3D(p2[0], p2[1], p1[2]));
		FPs.push_back(P3D(p1[0], p2[1], p1[2]));

		pinInfos.push_back(PinInfo(pinTrans, "BackBracketPin", FPs, center, V3D(0, 0, -1)));
		
		bracketCarvingMeshes.push_back(NULL);
	}

	{
		P3D p1 = boundingBox.bmin();
		P3D p2 = boundingBox.bmax();
		p2[2] += clearance;
		p1[2] = p2[2];
		P3D center = (p1 + p2) * 0.5;
		Transformation pinTrans(getRotationQuaternion(RAD(90), V3D(1, 0, 0)).getRotationMatrix(), V3D(center));

		vector<P3D> FPs;
		FPs.push_back(p2);
		FPs.push_back(P3D(p2[0], p1[1], p2[2]));
		FPs.push_back(P3D(p1[0], p1[1], p2[2]));
		FPs.push_back(P3D(p1[0], p2[1], p2[2]));

		pinInfos.push_back(PinInfo(pinTrans, "FrontBracketPin", FPs, center, V3D(0, 0, 1)));
	
		bracketCarvingMeshes.push_back(NULL);
	}*/
}
