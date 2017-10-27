#include "../include/MathLib/ConvexHull3D.h"
#define MAXN 1010

// TODO: re-add bullet!
//#include "../BulletCollision/LinearMath/btConvexHullComputer.h"

void ConvexHull3D::computeConvexHullFromSetOfPoints(const DynamicArray<P3D> &originalPoints, DynamicArray<P3D> &convexHullPoints, DynamicArray<ConvexHull_Face> &convexHullFaces){

	// TODO: re-add bullet!

	//DynamicArray<double> rawData;
	//for (uint i = 0; i < originalPoints.size(); i++) {
	//	rawData.push_back(originalPoints[i][0]);
	//	rawData.push_back(originalPoints[i][1]);
	//	rawData.push_back(originalPoints[i][2]);
	//}

	//btConvexHullComputer convexHullComputer;
	//convexHullComputer.compute(rawData.data(), 3 * sizeof(double), originalPoints.size(), 0, 0);

	////read off the vertices in the convex hull
	//for (int i = 0; i < convexHullComputer.vertices.size(); i++) {
	//	P3D p(convexHullComputer.vertices[i].x(), convexHullComputer.vertices[i].y(), convexHullComputer.vertices[i].z());
	//	convexHullPoints.push_back(p);
	//}

	////and now read off the faces...
	//for (int i = 0; i < convexHullComputer.faces.size();i++) {
	//	const btConvexHullComputer::Edge *curEdge = &convexHullComputer.edges[convexHullComputer.faces[i]];
	//	const btConvexHullComputer::Edge *startEdge = curEdge;
	//	convexHullFaces.push_back(ConvexHull_Face());
	//	assert(curEdge->getSourceVertex() < (int)convexHullPoints.size());
	//	convexHullFaces[i].vIndices.push_back(curEdge->getSourceVertex());
	//	while (curEdge->getNextEdgeOfFace() != startEdge) {
	//		curEdge = curEdge->getNextEdgeOfFace();
	//		assert(curEdge->getSourceVertex() < (int)convexHullPoints.size());
	//		convexHullFaces[i].vIndices.push_back(curEdge->getSourceVertex());
	//	}
	//}

	//all done...
}

class GLMesh;
void ConvexHull3D::computeConvexHullForMesh(GLMesh* meshInput, GLMesh *convexHullMesh, bool duplicateVertices) {
	// TODO: put outside of MatLib
	
	//DynamicArray<P3D> pointList;
	//for (int i = 0; i < meshInput->getVertexCount(); i++)
	//	pointList.push_back(meshInput->getVertex(i));

	//computeConvexHullFromSetOfPoints(pointList, convexHullMesh, duplicateVertices);
}

void ConvexHull3D::computeConvexHullFromSetOfPoints(const DynamicArray<P3D>& pointList, GLMesh* convexHullMesh, bool duplicateVertices) {
	// TODO: put outside of MatLib

	//	int zeroIndex = convexHullMesh->getVertexCount();
	//
	//	DynamicArray<P3D> CHVertices;
	//	DynamicArray<ConvexHull_Face> CHFaces;
	//
	//	ConvexHull3D::computeConvexHullFromSetOfPoints(pointList, CHVertices, CHFaces);
	//
	////	P3D midPoint;
	////	for (uint i = 0; i<pointList.size(); i++)
	////		midPoint += pointList[i] / pointList.size();
	//
	//	for (uint i = 0; i < CHVertices.size(); i++)
	//		convexHullMesh->addVertex(CHVertices[i]);
	//
	//	for (uint i = 0; i < CHFaces.size(); i++) {
	//		GLIndexedPoly poly;
	//		for (uint j = 0; j < CHFaces[i].vIndices.size(); j++)
	//			poly.addVertexIndex(CHFaces[i].vIndices[j] + zeroIndex);
	//
	//		convexHullMesh->addPoly(poly, duplicateVertices);
	//
	////		P3D p1 = pointList[CHFaces[i].vIndices[0]];
	////		P3D p2 = pointList[CHFaces[i].vIndices[1]];
	////		P3D p3 = pointList[CHFaces[i].vIndices[2]];
	//
	////		V3D n = V3D(p1, p2).cross(V3D(p1, p3));
	////		if (V3D(midPoint, p1).dot(n) > 0)
	////			convexHullMesh->addPoly(GLIndexedTriangle(faces[i].I[0] + zeroIndex, faces[i].I[1] + zeroIndex, faces[i].I[2] + zeroIndex), duplicateVertices);
	////		else
	////			convexHullMesh->addPoly(GLIndexedTriangle(faces[i].I[0] + zeroIndex, faces[i].I[2] + zeroIndex, faces[i].I[1] + zeroIndex), duplicateVertices);
	//	}
	//	convexHullMesh->computeNormals();
}


void ConvexHull3D::planarConvexHullFromSetOfPoints(const DynamicArray<P3D>& pointSet, const V3D& n, DynamicArray<ConvexHull2D_Vertex> &convexHullBoundaryVertices, bool duplicateVertices) {
	V3D t1, t2;
	n.getOrthogonalVectors(t1, t2);

	DynamicArray<int> ordering;
	DynamicArray<P3D> points = pointSet;

	ordering.clear();

	//find the point that is "left-most", and start from there...
	double minVal = DBL_MAX;
	double min2ndVal = DBL_MAX;
	int index = 0;
	for (uint i = 0; i<points.size(); i++) {
		ordering.push_back(i);
		double val = points[i].getComponentAlong(t1);
		double val2 = points[i].getComponentAlong(t2);
		if (minVal > val || (fabs(minVal - val)<0.000001 && val2 < min2ndVal)) {
			minVal = val;
			index = i;
			min2ndVal = val2;
		}
	}

	if (points.size() == 0) return;

	//now, make sure we start from this point...
	P3D p = points[index]; points[index] = points[0]; points[0] = p;
	int tmpI = ordering[index]; ordering[index] = ordering[0]; ordering[0] = tmpI;
	V3D axis = t2;
	//now, determine which point to add to the list next...
	for (uint i = 1; i<points.size(); i++) {
		double minAngle = DBL_MAX;
		double maxLength = -DBL_MAX;
		int index = 0;
		for (uint j = 0; j<points.size(); j++) {
			if (V3D(points[i - 1], points[j]).length() < 0.0001) continue;
			double angle = V3D(points[i - 1], points[j]).angleWith(axis, n);
			//we really don't expect to have negative angles here... if we do, it's just numerical errors...
			if (angle < -PI / 2) angle += 2 * PI;
			double len = V3D(points[i - 1], points[j]).length();
			if (minAngle - angle >= 0.000001 || (fabs(minAngle - angle) < 0.000001 && len > maxLength)) {
				minAngle = angle;
				index = j;
				maxLength = len;
			}
		}

		//check to see if we're back at the start... 
		if (V3D(points[0], points[index]).length() < 0.000001) {
			//if so, we're done... remove everything else from ordering now...
			ordering.resize(i);
			i = points.size();
		}
		else {
			P3D p = points[index]; points[index] = points[i]; points[i] = p;
			int tmpI = ordering[index]; ordering[index] = ordering[i]; ordering[i] = tmpI;
			axis = V3D(points[i - 1], points[i]);
		}

	}

	//now, based on the ordering, get the structure that will define the support polygon
	convexHullBoundaryVertices.clear();
	for (uint i = 0; i < ordering.size(); i++)
        convexHullBoundaryVertices.push_back(ConvexHull2D_Vertex(points[ordering[i]], ordering[i]));

	//and done...
}

