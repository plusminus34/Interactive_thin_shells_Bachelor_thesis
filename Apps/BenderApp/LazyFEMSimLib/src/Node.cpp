#include <GUILib/GLUtils.h>
#include <LazyFEMSimLib/Node.h>
#include <LazyFEMSimLib/SimulationMesh.h>

Node::Node(SimulationMesh* m, int nodeIndex, int dataStartIndex, int dimSize){
	this->mesh = m;
	this->nodeIndex = nodeIndex;
	this->dataStartIndex = dataStartIndex;
	this->dimSize = dimSize;
	selected = false;
}

Node::~Node(){

}

void Node::draw(V3D const & color, double size)
{
	glColor3d(color(0), color(1), color(2));
	if (selected || fixed || 1) {
		//glColor3d(1, 0, 0);
		drawSphere(getWorldPosition(), size);
	}
}


void Node::addMassContribution(double m){
	for (int i=0;i<dimSize;i++)
		mesh->m[dataStartIndex + i] += m;
}

P3D Node::getWorldPosition(){
	return getCoordinates(mesh->x);
}

void Node::setWorldPosition(const P3D& newPos){
	setCoordinates(newPos, mesh->x);
}

V3D Node::getVelocity(){
	return V3D(getCoordinates(mesh->v));
}

void Node::setVelocity(const V3D& newVel){
	for (int i=0;i<dimSize;i++)
		mesh->v[dataStartIndex + i] = newVel[i];
}

P3D Node::getUndeformedPosition(){
	return getCoordinates(mesh->X);
}

V3D Node::getExternalForce(){
	return V3D(getCoordinates(mesh->f_ext));
}

P3D Node::getCoordinates(const dVector& x) {
	P3D p;
	for (int i = 0;i<dimSize;i++)
		p[i] = x[dataStartIndex + i];
	return p;
}

void Node::setCoordinates(const P3D& coords, dVector& x) {
	for (int i = 0;i<dimSize;i++)
		x[dataStartIndex + i] = coords[i];
}

