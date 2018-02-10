
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


	// precompute a term for the Hessian
	dFdXij_all.resize(n_elements);
	for(int i_e = 0; i_e < n_elements; ++i_e) {
		CSTElement3D * element = dynamic_cast<CSTElement3D *>(elements[i_e]);
		std::array<std::array<Matrix3x3,3>,4> & dFdXij = dFdXij_all[i_e];

		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 3; ++j) {
				dFdXij[i][j].setZero();
				if (i > 0) {
					dFdXij[i][j](j, i - 1) = 1;
				}
				else {
					dFdXij[i][j](j, 0) = dFdXij[i][j](j, 1) = dFdXij[i][j](j, 2) = -1;
				}
				dFdXij[i][j] = dFdXij[i][j] * element->dXInv;
			}
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
	// prepare (parts of) gradient
	gradient.resize(x.size());
	gradient.setZero();

	addGradientElements(x, gradient);
	addGradientPinnedNodeElements(x, gradient);

	// prepare (parts of) hessian
	//hessianTriplets_elements.resize(n_elements * n_triplets_element);
	hessianTriplets.resize(0);
	addHessianElements(x, hessianTriplets);
	addHessianPinnedNodeElements(x, hessianTriplets);

}


double SimulationMesh::energyElement_i(int i, dVector const & x)
{
	// NEO HOOKEAN material model!

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
	double result = 0.0;
	
	for (size_t i = 0; i < elements.size() ; i++) {
		result += energyElement_i(i, x);
	}
	
	return(result);
}



double SimulationMesh::energyPinnedNodeElements(dVector const & x)
{
	double result = 0.0;
	for (size_t i = 0; i < pinnedNodeElements.size(); i++) {
		result += pinnedNodeElements[i]->getEnergy(x, X);
	}
	return(result);
}


void SimulationMesh::addGradientElement_i(int i, dVector const & x, dVector & grad)
{
	// NEO HOOKEAN material model!

	CSTElement3D * element = static_cast<CSTElement3D *>(elements[i]);
	Matrix3x3 & dEdF = element->dEdF;
	Matrix3x3 & dXInv = element->dXInv;
	std::array<V3D,4> dEdx;

	double shearM = element->shearModulus;
	double bulkM = element->bulkModulus;
	double restShapeVolume = element->restShapeVolume;

	// compute gradient components
	dEdF = dxdX[i] * shearM + dxdX_invT[i] * (-shearM + bulkM * dxdX_logdet[i]);
	
	dEdx[1] = V3D(dEdF(0, 0) * dXInv(0, 0) + dEdF(0, 1) * dXInv(0, 1) + dEdF(0, 2) * dXInv(0, 2), dEdF(1, 0) * dXInv(0, 0) + dEdF(1, 1) * dXInv(0, 1) + dEdF(1, 2) * dXInv(0, 2), dEdF(2, 0) * dXInv(0, 0) + dEdF(2, 1) * dXInv(0, 1) + dEdF(2, 2) * dXInv(0, 2)) * restShapeVolume;
	dEdx[2] = V3D(dEdF(0, 0) * dXInv(1, 0) + dEdF(0, 1) * dXInv(1, 1) + dEdF(0, 2) * dXInv(1, 2), dEdF(1, 0) * dXInv(1, 0) + dEdF(1, 1) * dXInv(1, 1) + dEdF(1, 2) * dXInv(1, 2), dEdF(2, 0) * dXInv(1, 0) + dEdF(2, 1) * dXInv(1, 1) + dEdF(2, 2) * dXInv(1, 2)) * restShapeVolume;
	dEdx[3] = V3D(dEdF(0, 0) * dXInv(2, 0) + dEdF(0, 1) * dXInv(2, 1) + dEdF(0, 2) * dXInv(2, 2), dEdF(1, 0) * dXInv(2, 0) + dEdF(1, 1) * dXInv(2, 1) + dEdF(1, 2) * dXInv(2, 2), dEdF(2, 0) * dXInv(2, 0) + dEdF(2, 1) * dXInv(2, 1) + dEdF(2, 2) * dXInv(2, 2)) * restShapeVolume;
	dEdx[0] = -dEdx[1] - dEdx[2] - dEdx[3];

	for (int j = 0; j<4; j++) {
		for (int k = 0; k<3; k++) {
			grad[elementNodeStarts[i][j] + k] += dEdx[j][k];
		}
	}
}

void SimulationMesh::addGradientElements(dVector const & x, dVector & grad)
{
	for (size_t i = 0; i < elements.size(); i++) {
		addGradientElement_i(i, x, grad);
	}
}

void SimulationMesh::addGradientPinnedNodeElements(dVector const & x, dVector & grad)
{
	for (size_t i = 0; i < pinnedNodeElements.size(); i++) {
		pinnedNodeElements[i]->addEnergyGradientTo(x, X, grad);
	}
}



void SimulationMesh::addHessianElement_i(int i_e, dVector const & x, std::vector<MTriplet> & hessianTriplets)
{
	// NEO HOOKEAN material model!
	CSTElement3D * element = static_cast<CSTElement3D *>(elements[i_e]);
	Matrix3x3 & Finv = dxdX_inv[i_e];
	Matrix3x3 & FinvT = dxdX_invT[i_e];
	double Flogdet = dxdX_logdet[i_e];
	Matrix3x3 & dXInv = element->dXInv;
	double & shearModulus = element->shearModulus;
	double & bulkModulus = element->bulkModulus;
	double & restShapeVolume = element->restShapeVolume;

	std::array<std::array<Matrix3x3,3>,4> & dFdXij = dFdXij_all[i_e];

	for (int i = 0; i < 4; ++i) {
		for(int j = 0; j < 3; ++j) {
			Matrix3x3 dF = dFdXij[i][j];
			Matrix3x3 dP = shearModulus * dF;
			dP = dP + (shearModulus - bulkModulus * Flogdet) * FinvT * dF.transpose() * FinvT;
			Matrix3x3 tmpM = Finv * dF;
			dP = dP + bulkModulus * tmpM.trace() * FinvT;
			Matrix3x3 dH = restShapeVolume * dP * dXInv.transpose();

			for (int ii = 0;ii < 3;++ii) {
				for (int jj = 0;jj < 3;++jj) {
					element->ddEdxdx[ii + 1][i](jj, j) = dH(jj, ii);
				}
			}
			element->ddEdxdx[0][i](0, j) = -dH(0, 2) - dH(0, 1) - dH(0, 0);
			element->ddEdxdx[0][i](1, j) = -dH(1, 2) - dH(1, 1) - dH(1, 0);
			element->ddEdxdx[0][i](2, j) = -dH(2, 2) - dH(2, 1) - dH(2, 0);
		}
	}

	for (int i = 0;i < 4;i++) {
		for (int j = 0;j < 4;j++) {
			addSparseMatrixDenseBlockToTriplet(hessianTriplets, elementNodeStarts[i_e][i], elementNodeStarts[i_e][j], element->ddEdxdx[i][j], true);
		}
	}



}

void SimulationMesh::addHessianElements(dVector const & x, std::vector<MTriplet> & hessianTriplets)
{
	for (size_t i = 0; i < elements.size(); i++) {
		//elements[i]->addEnergyHessianTo(x, X, hessianTriplets);
		addHessianElement_i(i, x, hessianTriplets);
	}
}


void SimulationMesh::addHessianPinnedNodeElements(dVector const & x, std::vector<MTriplet> & hessianTriplets)
{
	for (size_t i = 0; i < pinnedNodeElements.size(); i++) {
		pinnedNodeElements[i]->addEnergyHessianTo(x, X, hessianTriplets);
	}
}


void SimulationMesh::computeDeformationGradients(dVector const & x)
{
	dxdX.resize(n_elements);
	dxdX_norm2.resize(n_elements);
	dxdX_logdet.resize(n_elements);
	dxdX_inv.resize(n_elements);
	dxdX_invT.resize(n_elements);
	
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
		dxdX_inv[i] = dxdX[i].inverse();
		dxdX_invT[i] = dxdX_inv[i].transpose();
	}
}









