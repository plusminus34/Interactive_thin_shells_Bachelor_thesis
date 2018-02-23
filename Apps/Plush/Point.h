#pragma once
// --
#include "Point.h"
// --
#include <PlushHelpers/helpers_star.h>
// --
#include "Node.h"

class Point {

public:
	Point(vector<Node *> nodes, vector<double> weights);
	Point(Node *node);
	~Point();
 
public:
	vector<Node *> nodes;
	vector<double> weights;
	
public:
	P3D getCoordinates(const dVector& y);
	P3D getCurrentPosition();
	P3D getTargetPosition();
	P3D getUndeformedPosition();
 
};

