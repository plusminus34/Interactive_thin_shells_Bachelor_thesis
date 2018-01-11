#include <cmath>

#include <GUILib/GLUtils.h>
#include "MathLib/V3D.h"

#include "LinePositionObjective.h"



LinePositionObjective::LinePositionObjective(Node * node1, Node * node2, P3D const & targetPosition)
	: node1(node1), node2(node2), targetPosition(targetPosition)
{
}


void LinePositionObjective::addO(const dVector & x, const dVector & X, double & o) const
{
	
	V3D v(node1->getCoordinates(x), targetPosition);
	V3D n(node1->getCoordinates(x), node2->getCoordinates(x));
	V3D d = v - v.getProjectionOn(n);
	
	o += 0.5 * d.dot(d);
}


void LinePositionObjective::addDoDx(const dVector & x, const dVector & X, dVector & dodx) const
{



}

void LinePositionObjective::addError(const dVector & x, double & e) const
{

}