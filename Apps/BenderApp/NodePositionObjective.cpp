
#include <cmath>

#include <GUILib/GLUtils.h>
#include "MathLib/V3D.h"

#include "NodePositionObjective.h"



NodePositionObjective::NodePositionObjective(Node * node, P3D const & targetPosition)
	: node(node), targetPosition(targetPosition)
{
}


void NodePositionObjective::addO(const dVector & x, const dVector & X, double & o)
{
	V3D d = node->getCoordinates(x) - targetPosition;
	o += 0.5 * d.dot(d);
}


void NodePositionObjective::addDoDx(const dVector & x, const dVector & X, dVector & dodx)
{
	for(int i = 0; i < node->dimSize; ++i) {
		dodx[node->dataStartIndex + i] += x[node->dataStartIndex + i] - targetPosition[i];
	}
}


void NodePositionObjective::addError(const dVector & x, double & e)
{
	V3D d = node->getCoordinates(x) - targetPosition;
	e += sqrt(d.dot(d));
}



void NodePositionObjective::draw(dVector const & x) 
{
	glColor3d(0, 1, 0);
	P3D pi = (node->getCoordinates(x));
	P3D pj = targetPosition;
	glBegin(GL_LINES);
	glVertex3d(pi[0], pi[1], 0);
	glVertex3d(pj[0], pj[1], 0);
	glEnd();
}