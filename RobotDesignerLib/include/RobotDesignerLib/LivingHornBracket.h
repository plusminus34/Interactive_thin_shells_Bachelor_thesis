#pragma once

#include <MathLib/V3D.h>
#include <MathLib/P3D.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLUtils.h>
#include <vector>
#include <MathLib/mathLib.h>
#include <MathLib/Quaternion.h>
#include <MathLib/Segment.h>
#include <MathLib/boundingBox.h>
#include <RobotDesignerLib/LivingBracketMotor.h>

class SweptMotorShapePrimitive {
public:
	//each corner of the motor will sweep a circle arc. Keep track of the radius of this circle, as well as the start and end angles for it, and the line segments that provide a good bound...
	//we will assume that the motor rotates about the origin
	double phiStart = 0;
	double phiEnd = 0;
	double radius = 1.0;
	V3D v;
	Segment s1;
	Segment s2;

	SweptMotorShapePrimitive() {}

	SweptMotorShapePrimitive(double angleMin, double angleMax, double cornerX, double cornerY) {
		//account for the fact that the bracket moves relative to the motor...
		swap(angleMin, angleMax);
		angleMin *= -1; angleMax *= -1;

		V3D corner = V3D(cornerX, cornerY, 0);
		this->radius = corner.length();
		V3D rotAxis = V3D(0, 0, 1);
		double angleOffset = V3D(0, 1, 0).angleWith(corner, rotAxis);
		v = V3D(0, 1, 0) * radius;

		this->phiStart = angleMin + angleOffset;
		this->phiEnd = angleMax + angleOffset;

		V3D cornerPosM = corner.rotate(angleMin, rotAxis);
		V3D cornerPosP = corner.rotate(angleMax, rotAxis);

		if (cornerX * cornerY > 0) {
			s1 = Segment(P3D() + V3D(cornerX, 0, 0).rotate(angleMin, rotAxis), P3D() + cornerPosM);
			s2 = Segment(P3D() + cornerPosP, P3D() + V3D(0, cornerY, 0).rotate(angleMax, rotAxis));
		}
		else {
			s1 = Segment(P3D() + V3D(0, cornerY, 0).rotate(angleMin, rotAxis), P3D() + cornerPosM);
			s2 = Segment(P3D() + cornerPosP, P3D() + V3D(cornerX, 0, 0).rotate(angleMax, rotAxis));
		}
	}

	void drawCircleArc(V3D v, double angleMin, double angleMax) {
		Logger::consolePrint("drawing arc between %lf and %lf\n", angleMin, angleMax);

		glBegin(GL_LINE_STRIP);
		for (double i = 0; i < 1; i += 0.01) {
			V3D vRot = v.rotate(angleMin + i * (angleMax - angleMin), V3D(0, 0, 1));
			glVertex3d(vRot.x(), vRot.y(), 0.0);
		}
		glEnd();
	}

	void draw() {
		glLineWidth(3.0);
		glPushMatrix();
		glTranslated(0, 0, 0.01);

		glBegin(GL_LINES);
		glColor3d(1, 0, 0);
		glVertex3d(s1.a[0], s1.a[1], s1.a[2]);
		glColor3d(0, 1, 0);
		glVertex3d(s1.b[0], s1.b[1], s1.b[2]);
		glEnd();

		drawCircleArc(v, phiStart, phiEnd);

		glBegin(GL_LINES);
		glColor3d(0, 1, 0);
		glVertex3d(s2.a[0], s2.a[1], s2.a[2]);
		glColor3d(0, 0, 1);
		glVertex3d(s2.b[0], s2.b[1], s2.b[2]);
		glEnd();

		glPopMatrix();
		glLineWidth(1.0);
	}

	//NOTE: return -1 if the current primitive does not "cover" this segment...
	double getRadialDistanceAtPhase(double phi) {
		if ((phi >= phiStart && phi <= phiEnd) || (phi + 2 * PI >= phiStart && phi + 2 * PI <= phiEnd) || (phi - 2 * PI >= phiStart && phi - 2 * PI <= phiEnd))
			return radius;

		Segment seg(P3D(), P3D() + V3D(0, 1, 0).rotate(phi, V3D(0, 0, 1)));
		Segment closestSeg1 = seg.getShortestSegmentTo(s1);

		if (IS_ZERO(V3D(closestSeg1.a, closestSeg1.b).length()))
			return V3D(P3D(), closestSeg1.a).length();

		Segment closestSeg2 = seg.getShortestSegmentTo(s2);

		if (IS_ZERO(V3D(closestSeg2.a, closestSeg2.b).length()))
			return V3D(P3D(), closestSeg2.a).length();
		return -1;
	}
};

class SweptMotorShape {
public:
	SweptMotorShapePrimitive p1, p2, p3, p4;

	SweptMotorShape(double dimMinX, double dimMaxX, double dimMinY, double dimMaxY, double minAngle, double maxAngle) {
		p1 = SweptMotorShapePrimitive(minAngle, maxAngle, dimMinX, dimMinY);
		p2 = SweptMotorShapePrimitive(minAngle, maxAngle, dimMinX, dimMaxY);
		p3 = SweptMotorShapePrimitive(minAngle, maxAngle, dimMaxX, dimMaxY);
		p4 = SweptMotorShapePrimitive(minAngle, maxAngle, dimMaxX, dimMinY);
	}

	void draw() {
		p1.draw();
		p2.draw();
		p3.draw();
		p4.draw();
	}

	double getRadialDistanceAtPhase(double phi) {
		double minVal = 0;
		minVal = max(minVal, p1.getRadialDistanceAtPhase(phi));
		minVal = max(minVal, p2.getRadialDistanceAtPhase(phi));
		minVal = max(minVal, p3.getRadialDistanceAtPhase(phi));
		minVal = max(minVal, p4.getRadialDistanceAtPhase(phi));

		return minVal;
	}

	P3D getPointOnInterfaceAtPhase(double phi) {
		return P3D() + V3D(0, 1, 0).rotate(phi, V3D(0, 0, 1)) * getRadialDistanceAtPhase(phi);
	}

};

/*
	Horn brackets whose geometry is determined by several parameters that indicate how it will move.
*/
class LivingHornBracket {
public:
	void draw();

	//this is the motor the horn bracket is attached to
	LivingBracketMotor* motor;

	// position and orientation of the pin
	P3D pinPosition;
	Quaternion pinOrientation;

	// feature points
	vector<P3D> featurePoints;

	//the initial angle of the bracket - brackets will always be mounted at 0. This angle tells the bracket what the offset from zero is...
	double bracketInitialAngle = 0;
	//the bracket ends with a top surface that will accomodate a connector. The angle of this surface, relative to the orientation of the bracket, is specified here...
	double bracketConnectorAngle = 0.0;

	//in m(eters)
	double bracketConnectorThickness = 0.003;

	GLShaderMaterial shaderMaterial;

	GLMesh* bracketMesh;
	GLMesh* leftSideMesh;
	GLMesh* rightSideMesh;
	GLMesh* bridgeMesh;
	GLMesh* outputMesh;

private:

public:
	LivingHornBracket(LivingBracketMotor* motor, LivingHornBracket* lbh = NULL);

	virtual ~LivingHornBracket(void);

	//returns the distance from the ray's origin if the ray hits the body part, or -1 otherwise...
	double getDistanceToRayOriginIfHit(const Ray& ray);

	void generatePointLists(DynamicArray<P3D>& bracketFace, DynamicArray<P3D>& bracketBridge, DynamicArray<P3D>& bracketConnector);
	void generateBracketMesh();

	Transformation getPinTransformation();
	void setColor(const Vector4d& color = Vector4d(0, 0, 0, 0));
};

