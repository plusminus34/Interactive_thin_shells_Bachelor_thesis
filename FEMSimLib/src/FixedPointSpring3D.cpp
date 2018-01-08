#include <FEMSimLib/FixedPointSpring3D.h>
#include <GUILib/GLUtils.h>
#include <FEMSimLib/SimulationMesh.h>

FixedPointSpring3D::FixedPointSpring3D(SimulationMesh* simMesh, Node* node, P3D targetPosition, double K)
	: node(node), targetPosition(targetPosition), K(K)
{
}

FixedPointSpring3D::~FixedPointSpring3D()
{
}

double FixedPointSpring3D::getEnergy(const dVector & x, const dVector & X)
{
    //E = 0.5K xTx
    return 0.5 * K * (node->getCoordinates(x) - targetPosition).dot(node->getCoordinates(x) - targetPosition);
}


void FixedPointSpring3D::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
    //compute the gradient, and write it out
    //dEdx = Kx
    for (int j = 0; j < 3; j++) {
        grad[node->dataStartIndex + j] += K * (node->getCoordinates(x)[j] - targetPosition[j]);
    }
}

void FixedPointSpring3D::addEnergyHessianTo(const dVector & x, const dVector & X, std::vector<MTriplet>& hesEntries)
{
    //ddEdxdx = I;
    Matrix3x3 ddEdxdx;
    ddEdxdx.setIdentity();
    addSparseMatrixDenseBlockToTriplet(hesEntries, node->dataStartIndex, node->dataStartIndex, K * ddEdxdx, true);
}

void FixedPointSpring3D::draw(const dVector & x){
    glColor3d(1, 0, 0);
    P3D pi = (node->getCoordinates(x));
    P3D pj = targetPosition;
    glBegin(GL_LINES);
    glVertex3d(pi[0], pi[1], pi[2]);
    glVertex3d(pj[0], pj[1], pj[2]);
    glEnd();
}
