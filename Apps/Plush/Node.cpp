#include <GUILib/GLUtils.h>
#include "Node.h"
#include "SimulationMesh.h"
#include <PlushHelpers/helpers_star.h>

Node::Node(SimulationMesh* m, int nodeIndex, int dataStartIndex, int dimSize){
	this->mesh = m;
	this->nodeIndex = nodeIndex;
	this->dataStartIndex = dataStartIndex;
	this->dimSize = dimSize;
	selected = false;
	pinned = false;
}

Node::~Node(){

}

void Node::draw(){
	drawSphere(getCurrentPosition(), 0.01);
}

void Node::addMassContribution(double m) {
	for (int i=0;i<dimSize;i++)
		mesh->m[dataStartIndex + i] += m;
}

P3D Node::getCurrentPosition() {
	return getCoordinates(mesh->x);
}

void Node::setCurrentPosition(const P3D& newPos) {
	setCoordinates(newPos, mesh->x);
}

P3D Node::getTargetPosition() {
	return getCoordinates(mesh->x_prime);
}

void Node::setTargetPosition(const P3D& newPos){
	setCoordinates(newPos, mesh->x_prime);
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

void Node::setUndeformedPosition(const P3D& newPos){
	setCoordinates(newPos, mesh->X);
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

double Node::getMass() {
	return mesh->m[nodeIndex];
}
