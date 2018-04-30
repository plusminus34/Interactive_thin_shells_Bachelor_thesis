#include <GUILib/GLUtils.h>
#include "CSTElement2D.h"
#include "SimulationMesh.h"
#include <PlushHelpers/helpers_star.h>

#define SCALE_FACTOR 10000000.0

CSTElement2D::CSTElement2D(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3) : SimMeshElement(simMesh) {
//	shearModulus = 0.3 * 10e9 / SCALE_FACTOR;
//	bulkModulus = 1.5 * 10e9 / SCALE_FACTOR;

	this->n[0] = n1;
	this->n[1] = n2;
	this->n[2] = n3;

	this->i_vec_.resize(3, -1);
	this->i_vec_[0] = n1->nodeIndex;
	this->i_vec_[1] = n2->nodeIndex;
	this->i_vec_[2] = n3->nodeIndex;

	setRestShapeFromCurrentConfiguration();

	//distribute the mass of this element to the nodes that define it...
	for (int i = 0;i<3;i++)
		n[i]->addMassContribution(getMass() / 3.0);

//	matModel = MaterialModel2D::MM_LINEAR_ISOTROPIC;
//	matModel = MaterialModel2D::MM_STVK;
	matModel = MaterialModel2D::MM_NEO_HOOKEAN;

}

CSTElement2D::~CSTElement2D(){
}

void CSTElement2D::setRestShapeFromCurrentConfiguration(){
	//edge vectors
	V3D V1(n[0]->getCurrentPosition(), n[1]->getCurrentPosition());
	V3D V2(n[0]->getCurrentPosition(), n[2]->getCurrentPosition());
	//matrix that holds three edge vectors
	Matrix2x2 dX;
	dX << V1[0], V2[0], 
		  V1[1], V2[1];

	dXInv = dX.inverse();

	//compute the area of the element...
	restShapeArea = computeRestShapeArea(this->simMesh->X);
//	Logger::logPrint("CSTElement2D Element area: %lf\n", restShapeArea);
}


double CSTElement2D::getMass() {
	return restShapeArea * massDensity;
}

double CSTElement2D::computeRestShapeArea(const dVector& X) {
	P3D p1 = n[0]->getCoordinates(X);
	P3D p2 = n[1]->getCoordinates(X);
	P3D p3 = n[2]->getCoordinates(X);
	V3D V1(p1, p2), V2(p1, p3);
	//now compute the area of the element...
	return 1 / 2.0 * fabs(V1.cross(V2).length());
}

void CSTElement2D::addEnergyGradientTo(const dVector &x, const dVector &, dVector& grad) {
	//compute the gradient, and write it out
	computeGradientComponents(x, dVector());
	for (int i = 0;i<3;i++)
		for (int j = 0;j<2;j++)
		grad[n[i]->dataStartIndex + j] += dEdx[i][j];
}

void CSTElement2D::addEnergyHessianTo(const dVector& x, const dVector &, std::vector<MTriplet>& hesEntries) {
    //compute the hessian blocks and 
    computeHessianComponents(x, dVector());
    for (int i = 0;i<3;i++)
        for (int j = 0;j < 3;j++)
            addSparseMatrixDenseBlockToTriplet(hesEntries, n[i]->dataStartIndex, n[j]->dataStartIndex, ddEdxdx[i][j], true);
}

void CSTElement2D::draw(const dVector& x) {
	// set_color(LIGHT_HENN1NK);

	double f = this->getEnergy(x, dVector()) / .05;
	// set_color(map_swirl(f, COOL));
	// set_color(map_swirl(f, VIRIDIS));
	set_color(LIGHT_CLAY);

	glBegin(GL_TRIANGLES);
	for (auto node : n) {
		glP3D(node->getCoordinates(x));
	}
	glEnd();

	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glLineWidth(2.);
	// set_color(HENN1NK);
	set_color(CLAY);
	glLineWidth(1);
	glBegin(GL_LINE_LOOP);
	for (auto node : n) {
		glP3Dz(node->getCoordinates(x), 0);
	}
	glEnd();
	glPopAttrib();
}

void CSTElement2D::drawRestConfiguration(const dVector& X) {
	set_color(WHITE);
	glBegin(GL_LINE_STRIP);
	for (auto node : n) {
		glP3D(node->getCoordinates(X));
	}
	glEnd();
}

/**
	F maps deformed vectors dx to undeformed coords dX: dx = F*dX (or, in general F = dx/dX. By writing x as a weighted combination of 
	node displacements, where the weights are computed using basis/shape functions, F can be computed anywhere inside the element).
	For linear basis functions, F is constant throughout the element so an easy way to compute it is by looking at the matrix  that 
	maps deformed traingle/tet edges to their underformed counterparts: v = FV, where v and V are matrices containing edge vectors 
	in deformed and undeformed configurations
*/
void CSTElement2D::computeDeformationGradient(const dVector& x, const dVector&, Matrix2x2& dxdX){
	//edge vectors
	V3D v1(n[0]->getCoordinates(x), n[1]->getCoordinates(x)), v2(n[0]->getCoordinates(x), n[2]->getCoordinates(x));
	dx << v1[0], v2[0],
		  v1[1], v2[1];
	dxdX = dx * dXInv;

//	print("../out/v1.m", v1);
//	print("../out/v2.m", v2);
//	print("../out/dx.m", dx);
//	print("../out/dXInv.m", dXInv);
//	print("../out/dxdX.m", dxdX);
}

//implements the StVK material model
double CSTElement2D::getEnergy(const dVector& x, const dVector&){
	//compute the deformation gradient
	computeDeformationGradient(x, dVector(), F);
	double energyDensity = 0;

	double normF2 = F(0,0)*F(0,0) + F(0,1)*F(0,1) + F(1,0)*F(1,0) + F(1,1)*F(1,1);
	double detF = F.determinant();

	energyDensity += shearModulus/2 * (normF2-2) - shearModulus * log(detF) + bulkModulus/2 * log(detF) * log(detF);

	return energyDensity * restShapeArea;

}

void CSTElement2D::computeGradientComponents(const dVector& x, const dVector&){
	//compute the gradient of the energy using the chain rule: dE/dx = dE/dF * dF/dx. dE/dF is the first Piola-Kirchoff stress sensor, for which nice expressions exist.

	//compute the deformation gradient
	computeDeformationGradient(x, dVector(), F);
	dEdF.setZero();
	Finv = F.inverse();
	FinvT = Finv.transpose();

	double normF2 = F(0,0)*F(0,0) + F(0,1)*F(0,1) + F(1,0)*F(1,0) + F(1,1)*F(1,1);
	double detF = F.determinant();

	dEdF = F * shearModulus + FinvT * (-shearModulus + bulkModulus*log(detF));

	//dF/dx is going to be some +/- Xinv terms. The forces on nodes 1,2 can be writen as: dE/dF * XInv', while the force on node 0 is -f1-f2;
	dEdx[1] = V3D(dEdF(0,0) * dXInv(0,0) + dEdF(0,1) * dXInv(0,1), dEdF(1,0) * dXInv(0,0) + dEdF(1,1) * dXInv(0,1), 0) * restShapeArea;
	dEdx[2] = V3D(dEdF(0,0) * dXInv(1,0) + dEdF(0,1) * dXInv(1,1), dEdF(1,0) * dXInv(1,0) + dEdF(1,1) * dXInv(1,1), 0) * restShapeArea;
	dEdx[0] = -dEdx[1]-dEdx[2];
}

void CSTElement2D::computeHessianComponents(const dVector& x, const dVector&) {
    /*
    H = dfdx = ddEdxdx.
    H = restShapeArea * dPdx(F; dFdx) * transpose(dXInv)
    There are different formula of dPdx in different models. See below.
    dFdx = dDsdx * dXInv
    dDs = [	dx1 - dx0, dx2 - dx0
    dy1 - dy0, dy2 - dy0 ]
    let dx0,dy0,dx1,dy1,dx2,dy2 be 1 respectively (while others keep 0 so that we get dDsd(xk)),
    we can calculate 6 elements of H in each turn ( {l = 0..5}ddEd(xk)d(xl) ), and finally
    fill out whole H matrix. (3*3=9 small 2x2 matrices)
    */
	/*
	dPdx(F; dFdx) = shearModulus * dFdx +
	(shearModulus - bulkModulus * log(det(F))) * FinvT * transpose(dFdx) * FinvT +
	bulkModulus * trace(Finv * dFdx) * FinvT
	Finv = inverse(F)
	FinvT = transpose(Finv)
	*/
	computeDeformationGradient(x, dVector(), F);
	Finv = F.inverse();
	FinvT = Finv.transpose();
	Matrix2x2 dF, dP, tmpM, dH;
	const double dDs[6][4] = { { -1,-1,0,0 },{ 0,0,-1,-1 },{ 1,0,0,0 },{ 0,0,1,0 },{ 0,1,0,0 },{ 0,0,0,1 } };
	for (int i = 0; i < 6; ++i)
	{
		for (int x = 0; x < 4; ++x)
			dF(x / 2, x % 2) = dDs[i][x];
		dF = dF * dXInv;
		dP = shearModulus * dF;
		double J = F.determinant();
		dP = dP + (shearModulus - bulkModulus * log(J)) * FinvT * dF.transpose() * FinvT;
		tmpM = Finv * dF;
		dP = dP + bulkModulus * (tmpM(0, 0) + tmpM(1, 1)) * FinvT;
		dH = restShapeArea * dP * dXInv.transpose();
		int row = i / 2, subrow = i % 2;
		ddEdxdx[row][1](subrow, 0) = dH(0, 0); ddEdxdx[row][1](subrow, 1) = dH(1, 0);
		ddEdxdx[row][2](subrow, 0) = dH(0, 1); ddEdxdx[row][2](subrow, 1) = dH(1, 1);
		ddEdxdx[row][0](subrow, 0) = -(dH(0, 0) + dH(0, 1));
		ddEdxdx[row][0](subrow, 1) = -(dH(1, 0) + dH(1, 1));
	}
}