#include "CSTSimulationMesh2D.h"
#include <PlushHelpers\error.h>
#include <fstream> 
#include <sstream>
#include <stdio.h> 
#include <string.h> 


CSTSimulationMesh2D::CSTSimulationMesh2D() {
}

CSTSimulationMesh2D::~CSTSimulationMesh2D() {
}

////////////////////////////////////////////////////////////////////////////////
// 2D specs ////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int CSTSimulationMesh2D::D() {
	return 2;
}

void CSTSimulationMesh2D::add_simplices(const vector<vector<int>> &simplices_as_vecVecInt) {
	for (auto &i : simplices_as_vecVecInt) {
		CSTElement2D* triangle = new CSTElement2D(this, nodes[i[0]],
			                                            nodes[i[1]],
			                                            nodes[i[2]]);
		add_simplex(triangle);
	}
}

void CSTSimulationMesh2D::spawnSimplexMesh() {
	const P3D p0 = P3D( 0.00, 0.87);
	const P3D p1 = P3D( 0.50, 0.00);
	const P3D p2 = P3D(-0.50, 0.00);
	const vector<int> t0 = { 0, 1, 2, 3 };
	// --
	const vector<P3D> nodes_as_vecP3D = { p0, p1, p2 };
	const vector<vector<int>> simplices_as_vecVecInt = { t0 };
	const vector<int> pins_as_vecInt = { 1, 2 };
	// --
	spawnMesh(nodes_as_vecP3D, simplices_as_vecVecInt);
}

void CSTSimulationMesh2D::spawnSavedMesh(const char *prefix, bool loadTendons) {
	vector<P3D> nodes_as_vecP3D;
	vector<vector<int>> simplices_as_vecVecInt;
	vector<vector< pair<vector<int>, vector<double>> >> tendons_as_vecVecPairVecIntVecDouble;

	char   X_filename[128];
	char tri_filename[128];

	apply_suffix(prefix,   "X",     X_filename);
	apply_suffix(prefix, "tri",     tri_filename);

	FILE* X_fp = fopen(X_filename, "r");
	FILE* tri_fp = fopen(tri_filename, "r");

	{
		if (X_fp == NULL || tri_fp == NULL) {
			error("Some files not found:");
			if (  X_fp == NULL) { cout << "// " <<   X_filename << " not found." << endl; }
			if (tri_fp == NULL) { cout << "// " << tri_filename << " not found." << endl; }
			throw 123;
		}

		P3D p = P3D(0., 0., 0.);
		while (fscanf(X_fp, "%lf %lf", &p[0], &p[1]) != EOF) {
			nodes_as_vecP3D.push_back(p);
		}

		vector<int> i = { -1, -1, -1 };
		while (fscanf(tri_fp, "%d %d %d", &i[0], &i[1], &i[2]) != EOF) {
			simplices_as_vecVecInt.push_back(i);
		}
	}

	fclose(tri_fp);
	fclose(X_fp);

	spawnMesh(nodes_as_vecP3D, simplices_as_vecVecInt);
	if (loadTendons) { loadSavedTendons(prefix); }
}

////////////////////////////////////////////////////////////////////////////////
// modifications ///////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void CSTSimulationMesh2D::setPinnedNode(int ID, const P3D& point){
	P3D tp = point;
	tp[2] = 0;
	for (auto it = pins.begin(); it != pins.end(); ++it) {
		FixedPointSpring2D* fps = dynamic_cast<FixedPointSpring2D*>(*it);
		if (fps->node == nodes[ID]) {
			fps->targetPosition = tp;
			return;
		}
	}
	// --
	auto spring = new FixedPointSpring2D(this, nodes[ID], point);
	elements.push_back(spring);
	pins.push_back(spring);
}

int CSTSimulationMesh2D::getSelectedNodeID(Ray ray){
	int ID = -1;
	double dis = 2e9;
	for (uint i = 0; i < nodes.size(); i++) {
		P3D tp = nodes[i]->getCoordinates(x);
		tp.z() = 0;
		double tDis = ray.getDistanceToPoint(tp) / (sqrt((tp-ray.origin).dot(tp - ray.origin)));
		//Logger::consolePrint("%lf %lf %lf %lf\n", tp.x(), tp.y(), tp.z(), tDis);
		if (tDis < 0.01 && tDis < dis) {
			dis = tDis;
			ID = i;
		}
	}
	return ID;
}
 