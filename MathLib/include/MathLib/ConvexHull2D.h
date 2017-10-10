#pragma once
#include "Matrix.h"

using namespace std;

struct Point2D {
	double x;
	double y;

	Point2D(double xIn, double yIn) : x(xIn), y(yIn) { }
};

static double ccw(const Point2D& a, const Point2D& b, const Point2D& c) {
	return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

struct ccwSorter {
	const Point2D& pivot;

	ccwSorter(const Point2D& inPivot) : pivot(inPivot) { }

	bool operator()(const Point2D& a, const Point2D& b) {
		return ccw(pivot, a, b) < 0;
	}
};

// Returns true if a is lexicographically before b.
static bool isLeftOf(const Point2D& a, const Point2D& b) {
	return (a.x < b.x || (a.x == b.x && a.y < b.y));
}

// The length of segment (a, b).
static double len(const Point2D& a, const Point2D& b) {
	return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
}

// The unsigned distance of p from segment (a, b).
static double dist(const Point2D& a, const Point2D& b, const Point2D& p) {
	return fabs((b.x - a.x) * (a.y - p.y) - (b.y - a.y) * (a.x - p.x)) / len(a, b);
}

class ConvexHull2D
{
public:
	ConvexHull2D();
	~ConvexHull2D();

public:

	static vector<Point2D> GrahamScan(vector<Point2D> v);
};

