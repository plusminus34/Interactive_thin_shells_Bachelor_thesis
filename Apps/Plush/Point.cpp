#include "Point.h"
#include <PlushHelpers\error.h>

// Point is a convex combination of nodal positions.
// https://en.wikipedia.org/wiki/Convex_combination

Point::Point(vector<Node *> nodes, vector<double> weights) {
	// // General constructor
	// Point::nodes->size() == 1 -> NodalPoint
	// Point::nodes->size() == 3 -> BarycentricPoint

	if (nodes.size() != weights.size())      { error("Point mis-spec'd."); }
	if (true) { //abs(vecDouble_sum(weights) - 1.) > TINY) {
		// error("Renormalizing barycentric weights..." );
		double S = 0.;
		for (const auto &w : weights) { S += w; }
		for (auto &w : weights) { w /= S; } 
		if (!approx_equal(vecDouble_sum(weights), 1.)) {
			error("Renormalization failed.");
		}
	}
	this->nodes = nodes;
	this->weights = weights;
}
 
Point::Point(Node *node) {
	// // Special constructor
	// Point::nodes->size() == 1 -> NodalPoint
	this->nodes   = { node };
	this->weights = { 1. };
}

Point::~Point() {}

P3D Point::getCurrentPosition() {
	// return node->getCurrentPosition();
	P3D ret = P3D();
	for (size_t i = 0; i < nodes.size(); ++i) {
		ret += V3D(weights[i] * nodes[i]->getCurrentPosition());
	}
	return ret;
}

P3D Point::getTargetPosition() {
	// return node->getTargetPosition();
	P3D ret = P3D();
	for (size_t i = 0; i < nodes.size(); ++i) {
		ret += V3D(weights[i] * nodes[i]->getTargetPosition());
	}
	return ret;
}

P3D Point::getUndeformedPosition() {
	// return node->getUndeformedPosition();
	P3D ret = P3D();
	for (size_t i = 0; i < nodes.size(); ++i) {
		ret += V3D(weights[i] * nodes[i]->getUndeformedPosition());
	}
	return ret;
}

P3D Point::getCoordinates(const dVector& y) {
	// return node->getCoordinates(y);
	P3D ret = P3D();
	for (size_t i = 0; i < nodes.size(); ++i) {
		ret += V3D(weights[i] * nodes[i]->getCoordinates(y));
	}
	return ret;
}
 
