#include "Pin.h"

Pin::Pin(SimulationMesh* simMesh, BarycentricZeroLengthSpring* s1, BarycentricZeroLengthSpring* s2, BarycentricZeroLengthSpring* s3) : SimMeshElement(simMesh) {

    this->springs[0] = s1;
    this->springs[1] = s2;
	this->springs[2] = s3;

}

Pin::~Pin() {
}

double Pin::getMass() {
	return 0;
}

double Pin::getEnergy(const dVector& x, const dVector& X) {
	double sum = 0.0;
	for (int i = 0; i < 3; ++i) sum += springs[i]->getEnergy(x, X);
	return sum;
}

void Pin::addEnergyGradientTo(const dVector& x, const dVector& X, dVector& grad) {
	for (int i = 0; i < 3; ++i) springs[i]->addEnergyGradientTo(x, X, grad);
}

void Pin::addEnergyHessianTo(const dVector& x, const dVector& X, std::vector<MTriplet>& hesEntries) {
	for (int i = 0; i < 3; ++i) springs[i]->addEnergyHessianTo(x, X, hesEntries);
}

void Pin::draw(const dVector& x) {
}

void Pin::drawRestConfiguration(const dVector& X) {
}
