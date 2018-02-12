
#include <iostream>

#include <LazyFEMSimLib/SimulationMesh.h>
#include <LazyFEMSimLib/CSTElement3D.h>
#include <GUILib/GLUtils.h>

#include <omp.h>

SimulationMesh::SimulationMesh()
	: minimizer(50)
{
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
	/*
	for (uint i = 0; i < nodes.size(); i++) {
		nodes[i]->draw(nodeColor, nodeSize);
	}
	*/
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
   
	LazyNewtonFunctionMinimizer minimizer(3);
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

	minimizer.newHessianStructure = true;

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
	double e = 0.0;

	int n = elements.size();

//#pragma omp parallel for default(shared) reduction(+:e) num_threads(2) 
	for (int i = 0; i < n; i++) {
		CSTElement3D* element = static_cast<CSTElement3D*>(elements[i]);
		element->computeDeformationGradient(x, X);
		element->computeEnergy();
		e += element->getEnergy(x, X);
	}
	energy = e;
	/*
	for (size_t i = 0; i < elements.size(); i++) {
		CSTElement3D* element = static_cast<CSTElement3D*>(elements[i]);
		element->computeEnergy();
		energy += element->getEnergy(x, X);
	}
	*/

	for (size_t i = 0; i < pinnedNodeElements.size(); i++) {
		energy += pinnedNodeElements[i]->getEnergy(x, X);
	}
}

void SimulationMesh::prepare_upto_hessian(dVector const & x)
{

	// prepare (parts of) gradient
	gradient.resize(x.size());
	gradient.setZero();
	hessianTriplets.resize(0);

	// CST elements: prepare
	int n = elements.size();
//#pragma omp parallel for default(shared) num_threads(2)
	for (int i = 0; i < n; i++) {
		CSTElement3D* element = static_cast<CSTElement3D*>(elements[i]);
		element->computeGradientComponents();
		element->computeHessianComponents();
	}

	// CST elements: add
	for (size_t i = 0; i < elements.size(); i++) {
		CSTElement3D* element = static_cast<CSTElement3D*>(elements[i]);
		element->addEnergyGradientTo(x, X, gradient);
	}
	for (size_t i = 0; i < elements.size(); i++) {
		CSTElement3D* element = static_cast<CSTElement3D*>(elements[i]);
		element->addEnergyHessianTo(x, X, hessianTriplets);
	}

	// pinned node elements
	addGradientPinnedNodeElements(x, gradient);
	addHessianPinnedNodeElements(x, hessianTriplets);
}



void SimulationMesh::addGradientPinnedNodeElements(dVector const & x, dVector & grad)
{
	for (size_t i = 0; i < pinnedNodeElements.size(); i++) {
		pinnedNodeElements[i]->addEnergyGradientTo(x, X, grad);
	}
}



void SimulationMesh::addHessianPinnedNodeElements(dVector const & x, std::vector<MTriplet> & hessianTriplets)
{
	for (size_t i = 0; i < pinnedNodeElements.size(); i++) {
		pinnedNodeElements[i]->addEnergyHessianTo(x, X, hessianTriplets);
	}
}











