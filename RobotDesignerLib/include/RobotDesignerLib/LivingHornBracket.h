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

/*
	Horn brackets whose geometry is determined by several parameters that indicate how it will move.
*/
class Motor_RMC_HornBracket {
public:
	virtual void draw();

	// feature points of the bracket... used to attach to living connectors
	DynamicArray<P3D> featurePoints;

	// position and orientation of the pin
	P3D pinPosition;
	Quaternion pinOrientation;

	GLShaderMaterial shaderMaterial;

	GLMesh* bracketMesh = NULL;

	//the initial attachment angle of the bracket relative to the motor. This allows us to offset the range of motion of the motor.
	double bracketMountingAngle = 0;

	virtual void generateBracketMesh();

	virtual void setDefaultFeaturePointList() = 0;

	bool shouldRegenerateBracketMesh = true;
	virtual void goToNextMountingPosition() {}
	virtual void goToPreviousMountingPosition() {}

public:
	Motor_RMC_HornBracket();
	virtual ~Motor_RMC_HornBracket(void);

	virtual void setColor(const Vector4d& color = Vector4d(0, 0, 0, 0));
	Transformation getPinTransformation();

	virtual void copyBracketProperties(Motor_RMC_HornBracket* lbh, bool mirror) {
		if (lbh) {
			if (mirror == false){
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
class Motor_RMC_HornBracket_XM430 : public Motor_RMC_HornBracket {
public:
	Motor_RMC_HornBracket_XM430();

	virtual ~Motor_RMC_HornBracket_XM430(void);

	virtual void setDefaultFeaturePointList();

	virtual void goToNextMountingPosition() { bracketMountingAngle += RAD(45.0); shouldRegenerateBracketMesh = true; }

	virtual void goToPreviousMountingPosition() { bracketMountingAngle -= RAD(45.0); shouldRegenerateBracketMesh = true; }

};


/*
Horn brackets whose geometry is determined by several parameters that indicate how it will move.
*/
class Motor_RMC_HornBracket_TGY306G : public Motor_RMC_HornBracket {
public:
	Motor_RMC_HornBracket_TGY306G();

	virtual ~Motor_RMC_HornBracket_TGY306G(void);

	virtual void setDefaultFeaturePointList();

	virtual void goToNextMountingPosition() { bracketMountingAngle += RAD(360./(double)numMountingPositions); shouldRegenerateBracketMesh = true; }

	virtual void goToPreviousMountingPosition() { bracketMountingAngle -= RAD(360./ (double)numMountingPositions); shouldRegenerateBracketMesh = true; }

private:
	// The TGY 306G servo motor has 27 mounting positions
	const int numMountingPositions = 27;
};

/*
Horn brackets whose geometry is determined by several parameters that indicate how it will move.
*/
class Motor_RMC_HornBracket_BK3002 : public Motor_RMC_HornBracket {
public:
	Motor_RMC_HornBracket_BK3002();

	virtual ~Motor_RMC_HornBracket_BK3002(void);

	virtual void setDefaultFeaturePointList();

	virtual void goToNextMountingPosition() { bracketMountingAngle += RAD(15.0); shouldRegenerateBracketMesh = true; }

	virtual void goToPreviousMountingPosition() { bracketMountingAngle -= RAD(15.0); shouldRegenerateBracketMesh = true; }
};
