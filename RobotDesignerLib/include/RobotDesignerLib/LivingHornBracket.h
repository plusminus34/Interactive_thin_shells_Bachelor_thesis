#pragma once

#include <MathLib/V3D.h>
#include <MathLib/P3D.h>
#include <GUILib/GLMesh.h>
#include <GUILib/GLUtils.h>
#include <vector>
#include <MathLib/MathLib.h>
#include <MathLib/Quaternion.h>
#include <MathLib/Segment.h>
#include <MathLib/boundingBox.h>
#include "LivingMotor.h"

/*
	Horn brackets whose geometry is determined by several parameters that indicate how it will move.
*/
class LivingHornBracket {
public:
	virtual void draw();

	// feature points of the bracket... used to attach to living connectors
	vector<P3D> featurePoints;

	// position and orientation of the pin
	P3D pinPosition;
	Quaternion pinOrientation;

	GLMesh* bracketMesh = NULL;

	//the initial attachment angle of the bracket relative to the motor. This allows us to offset the range of motion of the motor.
	double bracketMountingAngle = 0;

	//TODO: the connector face could be parameterized such that it can rotate+translate relative to its default configuration
//	double connectorFaceAngle1 = 0, connectorFaceAngle2 = 0;

	GLShaderMaterial shaderMaterial;

	virtual void generateBracketMesh();

	virtual void setDefaultFeaturePointList() = 0;

	bool shouldRegenerateBracketMesh = true;
	virtual void goToNextMountingPosition() {}
	virtual void goToPreviousMountingPosition() {}

public:
	LivingHornBracket();
	virtual ~LivingHornBracket(void);

	virtual void setColor(const Vector4d& color = Vector4d(0, 0, 0, 0));
	Transformation getPinTransformation();

	virtual void copyBracketProperties(LivingHornBracket* lbh, bool mirror) {
		if (lbh) {
			if (mirror = false){
				bracketMountingAngle = lbh->bracketMountingAngle;
				shouldRegenerateBracketMesh = true;
//				connectorFaceAngle1 = lbh->connectorFaceAngle1;
//				connectorFaceAngle2 = lbh->connectorFaceAngle2;
			}
			else {
				bracketMountingAngle = -lbh->bracketMountingAngle;
				shouldRegenerateBracketMesh = true;
//				connectorFaceAngle1 = -lbh->connectorFaceAngle1;
//				connectorFaceAngle2 = -lbh->connectorFaceAngle2;
			}
		}
	}

};


/*
	Horn brackets whose geometry is determined by several parameters that indicate how it will move.
*/
class LivingHornBracket_XM430 : public LivingHornBracket {
public:
	LivingHornBracket_XM430();

	virtual ~LivingHornBracket_XM430(void);

	virtual void setDefaultFeaturePointList();

	virtual void goToNextMountingPosition() { bracketMountingAngle += RAD(45.0); shouldRegenerateBracketMesh = true; }

	virtual void goToPreviousMountingPosition() { bracketMountingAngle -= RAD(45.0); shouldRegenerateBracketMesh = true; }

};


/*
Horn brackets whose geometry is determined by several parameters that indicate how it will move.
*/
class LivingHornBracket_TGY306G : public LivingHornBracket {
public:
	LivingHornBracket_TGY306G();

	virtual ~LivingHornBracket_TGY306G(void);

	virtual void setDefaultFeaturePointList();

	virtual void goToNextMountingPosition() { bracketMountingAngle += RAD(15.0); shouldRegenerateBracketMesh = true; }

	virtual void goToPreviousMountingPosition() { bracketMountingAngle -= RAD(15.0); shouldRegenerateBracketMesh = true; }
};
