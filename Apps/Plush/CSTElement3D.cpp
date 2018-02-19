#include <GUILib/GLUtils.h>
#include "CSTElement3D.h"
#include "SimulationMesh.h"

#define SCALE_FACTOR 10000000.0

CSTElement3D::CSTElement3D(SimulationMesh* simMesh, Node* n1, Node* n2, Node* n3, Node* n4) : SimMeshElement(simMesh) {
    //	shearModulus = 0.3 * 10e9 / SCALE_FACTOR;
    //	bulkModulus = 1.5 * 10e9 / SCALE_FACTOR;

    this->n[0] = n1;
    this->n[1] = n2;
    this->n[2] = n3;
    this->n[3] = n4;

	this->i_vec_.resize(4, -1);
	this->i_vec_[0] = n1->nodeIndex;
	this->i_vec_[1] = n2->nodeIndex;
	this->i_vec_[2] = n3->nodeIndex;
	this->i_vec_[3] = n4->nodeIndex;
 
    setRestShapeFromCurrentConfiguration();

    //distribute the mass of this element to the nodes that define it...
    for (int i = 0;i<4;i++)
        n[i]->addMassContribution(getMass() / 4.0);

    matModel = MM_NEO_HOOKEAN;//MM_STVK;//MM_LINEAR_ISOTROPIC
}

CSTElement3D::~CSTElement3D() {
}

void CSTElement3D::setRestShapeFromCurrentConfiguration() {
    //edge vectors
    V3D V1(n[0]->getCurrentPosition(), n[1]->getCurrentPosition());
    V3D V2(n[0]->getCurrentPosition(), n[2]->getCurrentPosition());
    V3D V3(n[0]->getCurrentPosition(), n[3]->getCurrentPosition());
    //matrix that holds three edge vectors
    Matrix3x3 dX;
    dX << V1[0], V2[0], V3[0],
        V1[1], V2[1], V3[1],
        V1[2], V2[2], V3[2];

    dXInv = dX.inverse();

    //compute the volume of the element...
    restShapeVolume = computeRestShapeVolume(this->simMesh->X);
    //	Logger::logPrint("CSTElement2D Element volume: %lf\n", restShapeVolume);
}


double CSTElement3D::getMass() {
    return restShapeVolume * massDensity;
}
// should be checked
double CSTElement3D::computeRestShapeVolume(const dVector& X) {
    P3D p1 = n[0]->getCoordinates(X);
    P3D p2 = n[1]->getCoordinates(X);
    P3D p3 = n[2]->getCoordinates(X);
    P3D p4 = n[3]->getCoordinates(X);
    V3D V1(p1, p2), V2(p1, p3), V3(p1, p4);
    //now compute the volume of the element...
    return 1 / 6.0 * fabs(V1.cross(V2).dot(V3));
}

void CSTElement3D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
    //compute the gradient, and write it out
    computeGradientComponents(x, X);
    for (int i = 0;i<4;i++)
        for (int j = 0;j<3;j++)
        grad[n[i]->dataStartIndex + j] += dEdx[i][j];
}

void CSTElement3D::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
    //compute the hessian blocks and
    computeHessianComponents(x, X);
    for (int i = 0;i < 4;i++)
        for (int j = 0;j < 4;j++)
        addSparseMatrixDenseBlockToTriplet(hesEntries, n[i]->dataStartIndex, n[j]->dataStartIndex, ddEdxdx[i][j], true);
}

void CSTElement3D::draw(const dVector& x) {
    //	glColor3d(1, 1, 1);
	glPushAttrib(GL_ALL_ATTRIB_BITS); {
		set_color(LIGHT_CLAY);
		glLineWidth(3.);
		glPointSize(3.);
		
		glBegin(GL_LINES); {
			for (int i = 0; i < 3; i++) {
				for (int j = i + 1; j < 4; j++) {
					P3D pi = n[i]->getCoordinates(x);
					P3D pj = n[j]->getCoordinates(x);
					glVertex3d(pi[0], pi[1], pi[2]);
					glVertex3d(pj[0], pj[1], pj[2]);
				}
			}
		} glEnd();

	} glPopAttrib();
}

void CSTElement3D::drawRestConfiguration(const dVector& X) {
    glColor3d(1, 0, 0);
    glBegin(GL_LINES);
    for (int i = 0; i<3;i++)
        for (int j = i + 1;j<4;j++) {
            P3D pi = n[i]->getCoordinates(X);
            P3D pj = n[j]->getCoordinates(X);
            glVertex3d(pi[0], pi[1], pi[2]);
            glVertex3d(pj[0], pj[1], pj[2]);
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
void CSTElement3D::computeDeformationGradient(const dVector& x, const dVector& X, Matrix3x3& dxdX) {
    //edge vectors
    V3D v1(n[0]->getCoordinates(x), n[1]->getCoordinates(x));
    V3D v2(n[0]->getCoordinates(x), n[2]->getCoordinates(x));
    V3D v3(n[0]->getCoordinates(x), n[3]->getCoordinates(x));
    dx << v1[0], v2[0], v3[0],
        v1[1], v2[1], v3[1],
        v1[2], v2[2], v3[2];
    dxdX = dx * dXInv;

    //	print("../out/v1.m", v1);
    //	print("../out/v2.m", v2);
    //	print("../out/dx.m", dx);
    //	print("../out/dXInv.m", dXInv);
    //	print("../out/dxdX.m", dxdX);
}

//implements the StVK material model
double CSTElement3D::getEnergy(const dVector& x, const dVector& X) {
    //compute the deformation gradient
    computeDeformationGradient(x, X, F);
    double energyDensity = 0;

    if (matModel == MM_STVK) {
        //compute the Green Strain = 1/2 * (F'F-I)
        strain = F * F.transpose(); strain(0, 0) -= 1; strain(1, 1) -= 1; strain(2, 2) -= 1; strain *= 0.5;

        //add the deviatoric part of the energy, which penalizes the change in the shape of the element - the frobenius norm of E [tr (E'E)] measures just that
        // should be checked
        energyDensity += shearModulus * strain.squaredNorm();

        //and the volumetric/hydrostatic part, which is approximated as the trace of E and aims to maintain a constant volume
        energyDensity += bulkModulus / 2 * (strain(0, 0) + strain(1, 1) + strain(2, 2)) * (strain(0, 0) + strain(1, 1) + strain(2, 2));
    }
    else if (matModel == MM_LINEAR_ISOTROPIC) {
        //compute the Cauchy strain: 1/2 (F+F') - I
        strain = (F + F.transpose()) * 0.5; strain(0, 0) -= 1; strain(1, 1) -= 1; strain(2, 2) -= 1;

        //add the deviatoric part of the energy, which penalizes the change in the shape of the element - the frobenius norm of E [tr (E'E)] measures just that
        energyDensity += shearModulus * strain.squaredNorm();

        //and the volumetric/hydrostatic part, which is approximated as the trace of E and aims to maintain a constant volume
        energyDensity += bulkModulus / 2 * (strain(0, 0) + strain(1, 1) + strain(2, 2)) * (strain(0, 0) + strain(1, 1) + strain(2, 2));
    }
    else if (matModel == MM_NEO_HOOKEAN) {
        double normF2 = F.squaredNorm();
        double detF = F.determinant();
        // here are some diff
        energyDensity += shearModulus / 2 * (normF2 - 2) - shearModulus * log(detF) + bulkModulus / 2 * log(detF) * log(detF);
    }

    return energyDensity * restShapeVolume;

}

void CSTElement3D::computeGradientComponents(const dVector& x, const dVector& X) {
    //compute the gradient of the energy using the chain rule: dE/dx = dE/dF * dF/dx. dE/dF is the first Piola-Kirchoff stress sensor, for which nice expressions exist.

    //compute the deformation gradient
    computeDeformationGradient(x, X, F);
    dEdF.setZero();
    Finv = F.inverse();
    FinvT = Finv.transpose();
    if (matModel == MM_STVK) {
        strain = F.transpose() * F; strain(0, 0) -= 1; strain(1, 1) -= 1; strain(2, 2) -= 1;
        strain *= 0.5;
        dEdF = strain;
        dEdF *= 2.0 * shearModulus;
        dEdF(0, 0) += bulkModulus * (strain(0, 0) + strain(1, 1) + strain(2, 2));
        dEdF(1, 1) += bulkModulus * (strain(0, 0) + strain(1, 1) + strain(2, 2));
        dEdF(2, 2) += bulkModulus * (strain(0, 0) + strain(1, 1) + strain(2, 2));
        dEdF = F * dEdF;
    }
    else if (matModel == MM_LINEAR_ISOTROPIC) {
        //compute the Cauchy strain: 1/2 (F+F') - I
        strain = (F + F.transpose()) * 0.5; strain(0, 0) -= 1; strain(1, 1) -= 1; strain(2, 2) -= 1;
        dEdF = strain; dEdF *= 2 * shearModulus;
        dEdF(0, 0) += (strain(0, 0) + strain(1, 1) + strain(2, 2)) * bulkModulus;
        dEdF(1, 1) += (strain(0, 0) + strain(1, 1) + strain(2, 2)) * bulkModulus;
        dEdF(2, 2) += (strain(0, 0) + strain(1, 1) + strain(2, 2)) * bulkModulus;
    }
    else if (matModel == MM_NEO_HOOKEAN) {
        double normF2 = F.squaredNorm();
        double detF = F.determinant();
        // here are some diff
        dEdF = F * shearModulus + FinvT * (-shearModulus + bulkModulus*log(detF));
    }

    //dF/dx is going to be some +/- Xinv terms. The forces on nodes 1,2 can be writen as: dE/dF * XInv', while the force on node 0 is -f1-f2;
    dEdx[1] = V3D(dEdF(0, 0) * dXInv(0, 0) + dEdF(0, 1) * dXInv(0, 1) + dEdF(0, 2) * dXInv(0, 2), dEdF(1, 0) * dXInv(0, 0) + dEdF(1, 1) * dXInv(0, 1) + dEdF(1, 2) * dXInv(0, 2), dEdF(2, 0) * dXInv(0, 0) + dEdF(2, 1) * dXInv(0, 1) + dEdF(2, 2) * dXInv(0, 2)) * restShapeVolume;
    dEdx[2] = V3D(dEdF(0, 0) * dXInv(1, 0) + dEdF(0, 1) * dXInv(1, 1) + dEdF(0, 2) * dXInv(1, 2), dEdF(1, 0) * dXInv(1, 0) + dEdF(1, 1) * dXInv(1, 1) + dEdF(1, 2) * dXInv(1, 2), dEdF(2, 0) * dXInv(1, 0) + dEdF(2, 1) * dXInv(1, 1) + dEdF(2, 2) * dXInv(1, 2)) * restShapeVolume;
    dEdx[3] = V3D(dEdF(0, 0) * dXInv(2, 0) + dEdF(0, 1) * dXInv(2, 1) + dEdF(0, 2) * dXInv(2, 2), dEdF(1, 0) * dXInv(2, 0) + dEdF(1, 1) * dXInv(2, 1) + dEdF(1, 2) * dXInv(2, 2), dEdF(2, 0) * dXInv(2, 0) + dEdF(2, 1) * dXInv(2, 1) + dEdF(2, 2) * dXInv(2, 2)) * restShapeVolume;
    dEdx[0] = -dEdx[1] - dEdx[2] - dEdx[3];
}

void CSTElement3D::computeHessianComponents(const dVector& x, const dVector& X) {
    Matrix3x3 dFdXij[4][3];
    for (int i = 0;i < 4;++i)
        for (int j = 0;j < 3;++j)
        {
            dFdXij[i][j].setZero();
            if (i > 0)
                dFdXij[i][j](j, i - 1) = 1;
            else
                dFdXij[i][j](j, 0) = dFdXij[i][j](j, 1) = dFdXij[i][j](j, 2) = -1;
            dFdXij[i][j] = dFdXij[i][j] * dXInv;
        }
    if (matModel == MM_STVK)
    {
        /*
        dPdx(F; dFdx) = dFdx * (2 * shearModulus * E + bulkModulus * trace(E) * I) +
        F * (2 * shearModulus * dEdx + bulkModulus * trace(dEdx) * I)
        dEdx = 0.5 * (transpose(dFdx) * F + transpose(F) * dFdx)
        */
        computeDeformationGradient(x, X, F);
        Matrix3x3 FT = F.transpose();
        Matrix3x3 E = 0.5 * (FT * F);
        E(0, 0) -= 0.5; E(1, 1) -= 0.5; E(2, 2) -= 0.5;
        Matrix3x3 I;
        I.setIdentity();
        for (int i = 0;i < 4;++i)
            for (int j = 0;j < 3;++j)
            {
                Matrix3x3 dEdXij = 0.5 * (dFdXij[i][j].transpose() * F + FT * dFdXij[i][j]);
                Matrix3x3 dPdXij = dFdXij[i][j] * (2.0 * shearModulus * E + bulkModulus * E.trace() * I);
                dPdXij += F * (2.0 * shearModulus * dEdXij + bulkModulus * dEdXij.trace() * I);
                Matrix3x3 dHdXij = restShapeVolume * dPdXij * dXInv.transpose();
                for (int ii = 0;ii < 3;++ii)
                    for (int jj = 0;jj < 3;++jj)
                    ddEdxdx[ii + 1][i](jj, j) = dHdXij(jj, ii);
                ddEdxdx[0][i](0, j) = -dHdXij(0, 2) - dHdXij(0, 1) - dHdXij(0, 0);
                ddEdxdx[0][i](1, j) = -dHdXij(1, 2) - dHdXij(1, 1) - dHdXij(1, 0);
                ddEdxdx[0][i](2, j) = -dHdXij(2, 2) - dHdXij(2, 1) - dHdXij(2, 0);
            }
    }
    else if (matModel == MM_LINEAR_ISOTROPIC)
    {
        //dPdx(F; dFdx) = shearModulus * (dFdx + transpose(dFdx)) + bulkModulus * trace(dFdx) * I
        Matrix3x3 dSdX[6], dFdX, dPdX, dHdX;
        Matrix3x3 MI;
        MI.setIdentity();

        for (int i = 0; i < 12; i++) {
            dFdX = dFdXij[i % 4][i / 4];
            dPdX = shearModulus*(dFdX + dFdX.transpose()) + bulkModulus * dFdX.trace() *MI;
            dHdX = restShapeVolume * dPdX * dXInv.transpose();
            for (int ii = 0;ii < 3;++ii)
                for (int jj = 0;jj < 3;++jj)
                    ddEdxdx[ii + 1][i % 4](jj, i / 4) = dHdX(jj, ii);
            ddEdxdx[0][i % 4](0, i / 4) = -dHdX(0, 2) - dHdX(0, 1) - dHdX(0, 0);
            ddEdxdx[0][i % 4](1, i / 4) = -dHdX(1, 2) - dHdX(1, 1) - dHdX(1, 0);
            ddEdxdx[0][i % 4](2, i / 4) = -dHdX(2, 2) - dHdX(2, 1) - dHdX(2, 0);
        }
    }
    else if (matModel == MM_NEO_HOOKEAN)
    {
        /*
        dPdx(F; dFdx) = shearModulus * dFdx +
        (shearModulus - bulkModulus * log(det(F))) * FinvT * transpose(dFdx) * FinvT +
        bulkModulus * trace(Finv * dFdx) * FinvT
        Finv = inverse(F)
        FinvT = transpose(Finv)
        */
        computeDeformationGradient(x, X, F);
        Finv = F.inverse();
        FinvT = Finv.transpose();
        Matrix3x3 dF, dP, tmpM, dH;
        for (int i = 0; i < 12; ++i)
        {
            dF = dFdXij[i / 3][i % 3];
            dP = shearModulus * dF;
            double J = F.determinant();
            dP = dP + (shearModulus - bulkModulus * log(J)) * FinvT * dF.transpose() * FinvT;
            tmpM = Finv * dF;
            dP = dP + bulkModulus * tmpM.trace() *FinvT;
            dH = restShapeVolume * dP * dXInv.transpose();
            for (int ii = 0;ii < 3;++ii)
                for (int jj = 0;jj < 3;++jj)
                ddEdxdx[ii + 1][i / 3](jj, i % 3) = dH(jj, ii);
            ddEdxdx[0][i / 3](0, i % 3) = -dH(0, 2) - dH(0, 1) - dH(0, 0);
            ddEdxdx[0][i / 3](1, i % 3) = -dH(1, 2) - dH(1, 1) - dH(1, 0);
            ddEdxdx[0][i / 3](2, i % 3) = -dH(2, 2) - dH(2, 1) - dH(2, 0);
        }
    }
}