
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



void NodePositionObjective::draw(dVector const & x, HighlightLevel level) 
{
	if(level == HighlightLevel::HIDE) {return;}


	P3D pi = (node->getCoordinates(x));
	P3D pj = targetPosition;

	double r = 0.0;
	if(level ==  HighlightLevel::NONE) {
		r = 0.002;
		glColor3d(1.0, 0.5, 0);
	}
	if(level ==  HighlightLevel::HOVERED) {
		r = 0.002;
		glColor3d(1.0, 0.1, 0);
	}
	if(level ==  HighlightLevel::SELECTED) {
		r = 0.0025;
		glColor3d(1.0, 0.1, 0);
	}

	drawArrow(pi, pj, r*0.75);

	// sphere on base
	drawSphere(pi, r);
	// sphere at target
	if(level == HighlightLevel::HOVERED || level == HighlightLevel::SELECTED) {
		drawSphere(pj, r);
		
	}

}