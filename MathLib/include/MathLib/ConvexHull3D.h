#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cassert>
#include "P3D.h"
#include "V3D.h"

using namespace std;

struct ConvexHull_Face {
	DynamicArray<int> vIndices;
};


struct ConvexHull2D_Vertex {
	P3D projectedPoint;
	int index;

	ConvexHull2D_Vertex(const P3D& p, int index) {
		this->projectedPoint = p;
		this->index = index;
	}
};

class GLMesh;

class ConvexHull3D{
public:
	static void computeConvexHullForMesh(GLMesh* meshInput, GLMesh* convexHullMesh, bool duplicateVertices = false);
	static void computeConvexHullFromSetOfPoints(const DynamicArray<P3D>& pointList, GLMesh* convexHullMesh, bool duplicateVertices = false);
	static void computeConvexHullFromSetOfPoints(const DynamicArray<P3D> &originalPoints, DynamicArray<P3D> &convexHullPoints, DynamicArray<ConvexHull_Face> &convexHullFaces);

	static void planarConvexHullFromSetOfPoints(const DynamicArray<P3D>& points, const V3D& n, DynamicArray<ConvexHull2D_Vertex> &vertices, bool duplicateVertices = false);
};
