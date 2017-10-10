#include "../include/MathLib/Transformation.h"
//#include <GUILib/GLApplication.h>


Transformation::~Transformation()
{
}

// TODO: put this somewhere else
//void Transformation::applyGLMatrixTransform()
//{
//	Matrix4x4 transMat;
//	transMat.setIdentity();
//	transMat.topLeftCorner(3, 3) = R;
//	transMat.topRightCorner(3, 1) = T;
//
//	GLdouble m[16];
//	for (int i = 0; i < 16; i++){
//		m[i] = transMat(i % 4, i / 4);
//	}
//
//	glMultMatrixd(m);
//}