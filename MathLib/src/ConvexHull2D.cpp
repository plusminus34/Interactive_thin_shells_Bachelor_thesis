#include "../include/MathLib/ConvexHull2D.h"



ConvexHull2D::ConvexHull2D()
{
}


ConvexHull2D::~ConvexHull2D()
{
}


vector<Point2D> ConvexHull2D::GrahamScan(vector<Point2D> v) {
	// Put our leftmost point at index 0
	swap(v[0], *min_element(v.begin(), v.end(), isLeftOf));

	// Sort the rest of the points in counter-clockwise order
	// from our leftmost point.
	sort(v.begin() + 1, v.end(), ccwSorter(v[0]));

	// Add our first three points to the hull.
	vector<Point2D> hull;
	auto it = v.begin();
	hull.push_back(*it++);
	hull.push_back(*it++);
	hull.push_back(*it++);

	while (it != v.end()) {
		// Pop off any points that make a convex angle with *it
		while (ccw(*(hull.rbegin() + 1), *(hull.rbegin()), *it) >= 0) {
			hull.pop_back();
		}
		hull.push_back(*it++);
	}

	return hull;
}