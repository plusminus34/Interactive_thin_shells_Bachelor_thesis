#include <LazyFEMSimLib/SimulationMesh.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <GUILib/GLUtils.h>

SimulationMesh::SimulationMesh(){
	energyFunction = NULL;
	checkDerivatives = false;
}

SimulationMesh::~SimulationMesh(){
	delete energyFunction;
}

void SimulationMesh::drawNodes(){
	for (uint i=0; i<nodes.size(); i++)
		nodes[i]->draw();
}

void SimulationMesh::drawSimulationMesh(V3D const & edgeColor, double elementSize, 
										V3D const & pinnedNodeColor, double pinnedNodeSize,
										V3D const & nodeColor, double nodeSize)
{

	// elements
	glColor3d(edgeColor(0), edgeColor(1), edgeColor(2));
	glLineWidth((GLfloat)elementSize);
	for (uint i=0; i<elements.size();i++)
		elements[i]->draw(x);
	// pinned nodes (lines only)
	glColor3d(pinnedNodeColor(0), pinnedNodeColor(1), pinnedNodeColor(2));
	glLineWidth((GLfloat)pinnedNodeSize);
	for (uint i=0;i<pinnedNodeElements.size();i++)
		pinnedNodeElements[i]->draw(x);
	// nodes
	for (uint i = 0; i < nodes.size(); i++) {
		nodes[i]->draw(nodeColor, nodeSize);
	}
}

void SimulationMesh::drawExternalForces(){
	for (uint i=0; i<nodes.size(); i++)
		drawArrow(nodes[i]->getWorldPosition(), nodes[i]->getWorldPosition() + nodes[i]->getExternalForce(), 0.1);
}

void SimulationMesh::drawRestConfiguration(){
	for (uint i=0; i<elements.size();i++)
		elements[i]->drawRestConfiguration(X);
}

void SimulationMesh::addGravityForces(const V3D& g){
	for (uint i=0;i<nodes.size();i++)
		for (int j=0;j<nodes[i]->dimSize;j++)
			f_ext[nodes[i]->dataStartIndex + j] = g[j] * m[nodes[i]->dataStartIndex + j];
}

void SimulationMesh::clear(){
	nodes.clear();
	elements.clear();
	pinnedNodeElements.clear();
	delete energyFunction;
	energyFunction = NULL;
}

void SimulationMesh::solve_dynamics(double dt){
	xSolver = x;
	energyFunction->setToDynamicsMode(dt);

	if (checkDerivatives){
		energyFunction->testGradientWithFD(xSolver);
		energyFunction->testHessianWithFD(xSolver);
	}

	double functionValue = energyFunction->computeValue(xSolver);
//	Logger::consolePrint("energy value before solve: %lf\n", functionValue);
   
	NewtonFunctionMinimizer minimizer(3);
	minimizer.printOutput = true;
	minimizer.minimize(energyFunction, xSolver, functionValue);

//	Logger::consolePrint("energy value after solve: %lf\n", functionValue);

	//update the velocity for each node now...
	v = (xSolver - x) / dt;
	x = xSolver;
}

void SimulationMesh::solve_statics(){
	xSolver = x;
	energyFunction->setToStaticsMode(0.01);

	if (checkDerivatives){
		energyFunction->testGradientWithFD(xSolver);
		energyFunction->testHessianWithFD(xSolver);
	}

	double functionValue = energyFunction->computeValue(xSolver);
//	Logger::consolePrint("energy value before solve: %lf\n", functionValue);

	NewtonFunctionMinimizer minimizer(50);
	minimizer.printOutput = false;
	minimizer.minimize(energyFunction, xSolver, functionValue);

//	Logger::consolePrint("energy value after solve: %lf\n", functionValue);

	v.setZero();
	x = xSolver;
}

Node* SimulationMesh::getNodeIntersectedBy(const Ray& ray){
	for (uint i=0;i<nodes.size();i++){
		if (ray.getDistanceToPoint(nodes[i]->getWorldPosition()) < 0.05)
			return nodes[i];
	}
	return NULL;
}

//projects positions of nodes to lie on the plane, rather than under it, and kills normal component of their velocities
void SimulationMesh::fakeContactWithPlane(const Plane& plane){
	for (uint i=0;i<nodes.size();i++){
		double dist = plane.getSignedDistanceToPoint(nodes[i]->getWorldPosition());
		if (dist < 0){
			P3D newPos = nodes[i]->getWorldPosition() + plane.n * (-dist);
			nodes[i]->setWorldPosition(newPos);
			V3D vel = nodes[i]->getVelocity();
			if (vel.dot(plane.n) < 0){
				vel.setComponentAlong(plane.n, 0);
				nodes[i]->setVelocity(vel);
			}
		}
	}
}


