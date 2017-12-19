#include <RobotDesignerLib/LivingHornBracket.h>
#include <MathLib/ConvexHull3D.h>
#include <GUILib/GLContentManager.h>

LivingHornBracket::LivingHornBracket(LivingBracketMotor* motor, LivingHornBracket* lbh)
{
	this->motor = motor;

	shaderMaterial.setShaderProgram(GLContentManager::getShaderProgram("matcap"));
	string mat = "../data/textures/matcap/whitefluff2.bmp";
	shaderMaterial.setTextureParam(mat.c_str(), GLContentManager::getTexture(mat.c_str()));
}

LivingHornBracket::~LivingHornBracket(void)
{

}

Transformation LivingHornBracket::getPinTransformation(){
	return Transformation(pinOrientation.getRotationMatrix(), pinPosition);
}

void LivingHornBracket::setColor(const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	GLShaderMaterial defaultMat;
	defaultMat.setColor(color[0], color[1], color[2], color[3]);
}


/* ---------------------------------------------------------------------------- */

LivingHornBracket_XM430::LivingHornBracket_XM430(LivingBracketMotor* motor, LivingHornBracket* lbh) : LivingHornBracket(motor, lbh) {
	this->bracketMesh = new GLMesh();
	this->leftSideMesh = new GLMesh();
	this->rightSideMesh = new GLMesh();
	this->bridgeMesh = new GLMesh();

	if (LivingHornBracket_XM430* lbhxm430 = dynamic_cast<LivingHornBracket_XM430*>(lbh)){
		bracketInitialAngle = lbhxm430->bracketInitialAngle;
		bracketConnectorAngle = lbhxm430->bracketConnectorAngle;
		motor->rotAngleMin = lbhxm430->motor->rotAngleMin;
		motor->rotAngleMax = lbhxm430->motor->rotAngleMax;
	}
	generateBracketMesh();

	bracketMesh->setMaterial(shaderMaterial);
}

LivingHornBracket_XM430::~LivingHornBracket_XM430(void){
	delete bracketMesh;
}

void LivingHornBracket_XM430::draw() {
	double hornOffset = motor->boundingBox.halfSides().z() + 2 * motor->hornThickness + 0.00001;
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);

	glEnable(GL_LIGHTING);
	bracketMesh->drawMesh();
}

void addBracketFacePoly(GLMesh* bracketMesh, const DynamicArray<P3D>& bracketFace, double faceOffset, bool reversed) {
	int pIndex = bracketMesh->getVertexCount();
	for (uint i = 0; i < bracketFace.size(); i++) {
		bracketMesh->addVertex(bracketFace[i] + V3D(0, 0, faceOffset));
	}

	for (uint i = 1; i < bracketFace.size() - 1; i++) {
		bracketMesh->addPoly(GLIndexedTriangle(pIndex + 0, pIndex + i, pIndex + i + 1, reversed));
	}
}

void stitchBracketFaces(GLMesh* bracketMesh, const DynamicArray<P3D>& bracketFace, double faceOffset1, double faceOffset2, bool reversed) {
	int pIndex = bracketMesh->getVertexCount();
	int nPts = (int)bracketFace.size();
	for (uint i = 0; i < bracketFace.size(); i++)
		bracketMesh->addVertex(bracketFace[i] + V3D(0, 0, faceOffset1));

	for (uint i = 0; i < bracketFace.size(); i++)
		bracketMesh->addVertex(bracketFace[i] + V3D(0, 0, faceOffset2));


	for (uint i = 0; i < bracketFace.size(); i++) {
		bracketMesh->addPoly(GLIndexedTriangle(pIndex + i, pIndex + nPts + i, pIndex + nPts + (i + 1) % nPts, reversed), true);
		bracketMesh->addPoly(GLIndexedTriangle(pIndex + i, pIndex + nPts + (i + 1) % nPts, pIndex + (i + 1) % nPts, reversed), true);
	}
}

void LivingHornBracket_XM430::generateBracketMesh() {
	bracketMesh->clear();
	leftSideMesh->clear();
	rightSideMesh->clear();
	bridgeMesh->clear();

	double hornStart = motor->boundingBox.halfSides().z();
	double hornEnd = hornStart + 2 * motor->hornThickness;

	DynamicArray<P3D> bracketFace, bracketBridge, bracketConnector;
	generatePointLists(bracketFace, bracketBridge, bracketConnector);

	addBracketFacePoly(rightSideMesh, bracketFace, hornStart, false);
	addBracketFacePoly(rightSideMesh, bracketFace, hornEnd, true);
	stitchBracketFaces(rightSideMesh, bracketFace, hornStart, hornEnd, false);
	bracketMesh->append(rightSideMesh);

	addBracketFacePoly(leftSideMesh, bracketFace, -hornStart, true);
	addBracketFacePoly(leftSideMesh, bracketFace, -hornEnd, false);
	stitchBracketFaces(leftSideMesh, bracketFace, -hornStart, -hornEnd, true);
	bracketMesh->append(leftSideMesh);

	int nPts = bracketConnector.size();
	for (int i = 0; i < nPts; i++)
		bracketConnector.push_back(bracketConnector[nPts - i - 1] + V3D(0, 1, 0).rotate(bracketConnectorAngle + bracketInitialAngle, V3D(0, 0, 1)) * bracketConnectorThickness);
	for (int i = 0; i < nPts; i++)
		bracketConnector[i] += V3D(0, 1, 0).rotate(bracketConnectorAngle + bracketInitialAngle, V3D(0, 0, 1)) * bracketConnectorThickness * -0.01;

	addBracketFacePoly(bridgeMesh, bracketConnector, hornEnd, false);
	addBracketFacePoly(bridgeMesh, bracketConnector, -hornEnd, true);
	stitchBracketFaces(bridgeMesh, bracketConnector, -hornEnd, hornEnd, true);

	bracketMesh->append(bridgeMesh);

	bracketMesh->computeNormals();
	bracketMesh->calBoundingBox();
}

void LivingHornBracket_XM430::setColor(const Vector4d& color /*= Vector4d(0, 0, 0, 0)*/){
	if (color.isZero())
	{
		bracketMesh->setMaterial(shaderMaterial);
	}
	else {
		GLShaderMaterial defaultMat;
		defaultMat.setColor(color[0], color[1], color[2], color[3]);
		bracketMesh->setMaterial(defaultMat);
	}
}

void LivingHornBracket_XM430::generatePointLists(DynamicArray<P3D>& bracketFace, DynamicArray<P3D>& bracketBridge, DynamicArray<P3D>& bracketConnector) {
	//the width of the bracket - it should depend on the motor dimension (horn width, etc...)
	double bracketWidth = motor->boundingBox.halfSides().x() * 2;

	SweptMotorShape sms(motor->boundingBox.bmin().x(), motor->boundingBox.bmax().x(), motor->boundingBox.bmin().y(), motor->boundingBox.bmax().y(), motor->rotAngleMin, motor->rotAngleMax);
	//	sms.draw();
	double bracketMaxHeight = 0;
	double bracketMinHeight = DBL_MAX;
	for (double ang = bracketInitialAngle - 1.0; ang < bracketInitialAngle + 1.0; ang += 0.01) {
		P3D p = sms.getPointOnInterfaceAtPhase(ang);
		V3D v(P3D(), p);
		v = v.rotate(-bracketInitialAngle, V3D(0, 0, 1));
		if (v.x() < bracketWidth / 2 && v.x() > -bracketWidth / 2) {
			bracketBridge.push_back(p);
			bracketMaxHeight = max(bracketMaxHeight, v.y());
			bracketMinHeight = min(bracketMinHeight, v.y());
		}
	}

	P3D p1(-bracketWidth / 2, bracketMaxHeight, 0);
	V3D topCorner1 = V3D(-bracketWidth / 2, 0).rotate(bracketConnectorAngle, V3D(0, 0, 1));
	V3D topCorner2 = V3D(bracketWidth / 2, 0).rotate(bracketConnectorAngle, V3D(0, 0, 1));
	V3D faceOffset(bracketWidth / 2.0 - cos(bracketConnectorAngle) * bracketWidth / 2.0, fabs(sin(bracketConnectorAngle)) * bracketWidth / 2.0, 0);
	if (bracketConnectorAngle > 0) faceOffset.x() *= -1;
	topCorner1 += faceOffset;
	topCorner2 += faceOffset;

	P3D p2(topCorner1.x(), topCorner1.y() + bracketMinHeight, 0);
	P3D p3(topCorner2.x(), topCorner2.y() + bracketMinHeight, 0);
	P3D p4(bracketWidth / 2, bracketMaxHeight, 0);

	Segment s1(p1, p2), s2(p2, p3), s3(p3, p4);

	double bracketMaxHeightOffset = DBL_MAX;
	for (uint i = 0; i < bracketBridge.size(); i++) {
		P3D p = bracketBridge[i];
		p = P3D() + V3D(P3D(), p).rotate(-bracketInitialAngle, V3D(0, 0, 1));

		if (p.x() >= s1.a.x() && p.x() < s1.b.x()) {
			double t = (p.x() - s1.a.x()) / (s1.b.x() - s1.a.x());
			double y = s1.a.y() * (1 - t) + s1.b.y() * (t);
			bracketMaxHeightOffset = min(bracketMaxHeightOffset, y - p.y());
		}
		else if (p.x() >= s2.a.x() && p.x() < s2.b.x()) {
			double t = (p.x() - s2.a.x()) / (s2.b.x() - s2.a.x());
			double y = s2.a.y() * (1 - t) + s2.b.y() * (t);
			bracketMaxHeightOffset = min(bracketMaxHeightOffset, y - p.y());
		}
		else if (p.x() >= s3.a.x() && p.x() < s3.b.x()) {
			double t = (p.x() - s3.a.x()) / (s3.b.x() - s3.a.x());
			double y = s3.a.y() * (1 - t) + s3.b.y() * (t);
			bracketMaxHeightOffset = min(bracketMaxHeightOffset, y - p.y());
		}
	}

	p2.y() -= bracketMaxHeightOffset;
	p3.y() -= bracketMaxHeightOffset;

	bracketMinHeight -= motor->boundingBox.halfSides().y() * 0.2;

	Quaternion q = getRotationQuaternion(bracketInitialAngle, V3D(0, 0, 1));

	bracketFace.push_back(P3D() + q.rotate(V3D(-motor->hornRadius, -motor->hornRadius, 0)));
	bracketFace.push_back(P3D() + q.rotate(V3D(-bracketWidth / 2, bracketMinHeight, 0)));


	bracketFace.push_back(P3D() + q.rotate(V3D(p2.x(), p2.y(), 0)));
	bracketFace.push_back(P3D() + q.rotate(V3D(p3.x(), p3.y(), 0)));

	bracketFace.push_back(P3D() + q.rotate(V3D(bracketWidth / 2, bracketMinHeight, 0)));
	bracketFace.push_back(P3D() + q.rotate(V3D(motor->hornRadius, -motor->hornRadius, 0)));

	bracketConnector.push_back(P3D() + q.rotate(V3D(p2.x(), p2.y(), 0)));
	bracketConnector.push_back(P3D() + q.rotate(V3D(p3.x(), p3.y(), 0)));

	double hornStart = motor->boundingBox.halfSides().z();
	double hornEnd = hornStart + 2 * motor->hornThickness;
	P3D rq1(q.rotate(V3D(p2.x(), p2.y(), 0)));
	P3D rq2(q.rotate(V3D(p3.x(), p3.y(), 0)));
	rq1 += V3D(0, 1, 0).rotate(bracketConnectorAngle + bracketInitialAngle, V3D(0, 0, 1)) * bracketConnectorThickness;
	rq2 += V3D(0, 1, 0).rotate(bracketConnectorAngle + bracketInitialAngle, V3D(0, 0, 1)) * bracketConnectorThickness;

	featurePoints.clear();
	featurePoints.push_back(rq1 + V3D(0, 0, hornEnd));
	featurePoints.push_back(rq1 + V3D(0, 0, -hornEnd));
	featurePoints.push_back(rq2 + V3D(0, 0, hornEnd));
	featurePoints.push_back(rq2 + V3D(0, 0, -hornEnd));

	pinPosition = (rq1 + rq2) * 0.5;
	pinOrientation = getRotationQuaternion(bracketConnectorAngle + bracketInitialAngle, V3D(0, 0, 1));

}

