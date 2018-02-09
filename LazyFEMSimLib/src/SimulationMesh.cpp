
#include <iostream>

#include <LazyFEMSimLib/SimulationMesh.h>
#include <LazyFEMSimLib/CSTElement3D.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <GUILib/GLUtils.h>

SimulationMesh::SimulationMesh(){
	energyFunction = new FEMEnergyFunction();
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




void SimulationMesh::initializeStructure()
{
	// initialize the energy Function
	energyFunction->initialize(this);

	// precomputations based on the structure
	n_elements = elements.size();
	
	elementNodes.resize(n_elements);
	elementNodeStarts.resize(n_elements);
	for(int i = 0; i < n_elements; ++i) {
		CSTElement3D * element = dynamic_cast<CSTElement3D *>(elements[i]);
		if(!element) {std::cerr << "Error: not implemented " << __FILE__ << ":" << __LINE__ << std::endl;}
		for(int j = 0; j < 4; ++j) {
			elementNodes[i][j] = element->n[j]->nodeIndex;
			elementNodeStarts[i][j] = element->n[j]->dataStartIndex;
		}
	}



	// precomputations based on state
	xSolver = x;
	initializeState_xSolver();
}



// attention: consideres the sotate of xSolver, NOT of x
void SimulationMesh::initializeState_xSolver()
{
	prepare_upto_energy(xSolver);
	prepare_upto_hessian(xSolver);
}


void SimulationMesh::prepare_upto_energy(dVector const & x)
{

	computeDeformationGradients(x);


	energy = 0.0;
	energy += energyElements(x);
	energy += energyPinnedNodeElements(x);
}

void SimulationMesh::prepare_upto_hessian(dVector const & x)
{

}


double SimulationMesh::energyElement_i(int i, dVector const & x)
{
	CSTElement3D * element = static_cast<CSTElement3D *>(elements[i]);
	double shearM = element->shearModulus;
	double bulkM = element->bulkModulus;
	double restShapeVolume = element->restShapeVolume;

	double energyDensity = 0.5 * shearM * (dxdX_norm2[i] - 2.0) - shearM * dxdX_logdet[i] + 0.5 * bulkM * dxdX_logdet[i] * dxdX_logdet[i];
	energyDensity *= restShapeVolume;

	return(energyDensity);
}


double SimulationMesh::energyElements(dVector const & x)
{
	// NEO HOOKEAN material model!
	double result = 0.0;
	
	for (uint i = 0; i < elements.size() ; i++) {
		result += energyElement_i(i, x);
	}
	
	return(result);
}



double SimulationMesh::energyPinnedNodeElements(dVector const & x)
{
	double result = 0.0;
	for (uint i = 0; i < pinnedNodeElements.size(); i++) {
		result += pinnedNodeElements[i]->getEnergy(x, X);
	}
	return(result);
}


void SimulationMesh::computeDeformationGradients(dVector const & x)
{
	dxdX.resize(n_elements);
	dxdX_norm2.resize(n_elements);
	dxdX_logdet.resize(n_elements);
	
	Matrix3x3 dx;
	for(int i = 0; i < n_elements; ++i) {
		// todo: make nicer
		int j0 = elementNodeStarts[i][0];
		int j1 = elementNodeStarts[i][1];
		int j2 = elementNodeStarts[i][2];
		int j3 = elementNodeStarts[i][3];
		dx(0,0) = x[j1] - x[j0];
		dx(0,1) = x[j2] - x[j0];
		dx(0,2) = x[j3] - x[j0];

		dx(1,0) = x[j1+1] - x[j0+1];
		dx(1,1) = x[j2+1] - x[j0+1];
		dx(1,2) = x[j3+1] - x[j0+1];

		dx(2,0) = x[j1+2] - x[j0+2];
		dx(2,1) = x[j2+2] - x[j0+2];
		dx(2,2) = x[j3+2] - x[j0+2];

		dxdX[i] = dx * static_cast<CSTElement3D*>(elements[i])->dXInv;

		dxdX_norm2[i] = dxdX[i].squaredNorm();
		dxdX_logdet[i] = log(dxdX[i].determinant());
	}
}









