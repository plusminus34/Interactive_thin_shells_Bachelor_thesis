#include "CSTSimulationMesh3D.h"
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include "CSTElement3D.h"
#include "FixedPointSpring3D.h"
#include <PlushHelpers/tetgen.h>
#include <PlushHelpers\error.h>
// #include <Plush\NodalTendon3D.h>

CSTSimulationMesh3D::CSTSimulationMesh3D() {
}


CSTSimulationMesh3D::~CSTSimulationMesh3D() {
}

////////////////////////////////////////////////////////////////////////////////
// 3D specifics ////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int CSTSimulationMesh3D::D() {
	return 3;
}

void CSTSimulationMesh3D::add_simplices(const vector<vector<int>> &simplices_as_vecVecInt) {
	for (auto &i : simplices_as_vecVecInt) {
		CSTElement3D* tetrahedron = new CSTElement3D(this, nodes[i[0]],
			                                               nodes[i[1]],
			                                               nodes[i[2]],
			                                               nodes[i[3]]);
		add_simplex(tetrahedron);
	}
} 

void CSTSimulationMesh3D::analyze_lower_simplices() {
	{};
}

void CSTSimulationMesh3D::spawnSimplexMesh() {
	double f = .87 / 1.41;
	const P3D p0 = P3D( 0.00*f, 1.41*f,  0.00*f);
	const P3D p1 = P3D(-0.87*f, 0.00*f, -0.50*f);
	const P3D p2 = P3D( 0.87*f, 0.00*f, -0.50*f);
	const P3D p3 = P3D( 0.00*f, 0.00*f,  1.00*f);
	const vector<int> t0 = { 0, 1, 2, 3 };
	// --
	const vector<P3D> nodes_as_vecP3D = { p0, p1, p2, p3 };
	const vector<vector<int>> simplices_as_vecVecInt = { t0 };
	const vector<int> pins_as_vecInt = { 0 };
	// --
	spawnMesh(nodes_as_vecP3D, simplices_as_vecVecInt);
}

void CSTSimulationMesh3D::spawnSavedMesh(const char *prefix) {
	vector<P3D> nodes_as_vecP3D;
	vector<vector<int>> simplices_as_vecVecInt;

	char   X_filename[128];
	char tet_filename[128];

	apply_suffix(prefix,   "X",     X_filename);
	apply_suffix(prefix, "tet",     tet_filename);

	FILE* X_fp   = fopen(X_filename, "r");
	FILE* tet_fp = fopen(tet_filename, "r");

	{
		if (X_fp == NULL || tet_fp == NULL) {
			error("Some files not found:");
			if (  X_fp == NULL) { cout << "// " <<   X_filename << " not found." << endl; }
			if (tet_fp == NULL) { cout << "// " << tet_filename << " not found." << endl; }
			throw 123;
		}

		P3D p;
		while (fscanf(X_fp, "%lf %lf %lf", &p[0], &p[1], &p[2]) != EOF) {
			nodes_as_vecP3D.push_back(p);
		}

		vector<int> i = { -1, -1, -1, -1 };
		while (fscanf(tet_fp, "%d %d %d %d", &i[0], &i[1], &i[2], &i[3]) != EOF) {
			simplices_as_vecVecInt.push_back(i);
		}
	}

	fclose(tet_fp);
	fclose(X_fp);

	spawnMesh(nodes_as_vecP3D, simplices_as_vecVecInt);
	loadSavedTendons(prefix);
}

int CSTSimulationMesh3D::getSelectedNodeID(Ray ray){
    int ID = -1;
    double dis = 2e9;
    for (uint i = 0; i < nodes.size(); i++) {
        P3D tp = nodes[i]->getCoordinates(x);
        double tDis = ray.getDistanceToPoint(tp) / (sqrt((tp - ray.origin).dot(tp - ray.origin)));
        //Logger::consolePrint("%lf %lf %lf %lf\n", tp.x(), tp.y(), tp.z(), tDis);
        if (tDis < 0.01 && tDis < dis) {
            dis = tDis;
            ID = i;
        }
    }
    return ID;
}


void CSTSimulationMesh3D::setPinnedNode(int ID, const P3D& point) {
	for (auto it = pins.begin(); it != pins.end(); ++it){
		FixedPointSpring3D* fps = dynamic_cast<FixedPointSpring3D*>(*it);
		if (fps->node == nodes[ID]){
			fps->targetPosition = point;
			return;
		}
	}
	// --
	auto spring = new FixedPointSpring3D(this, nodes[ID], point);
	elements.push_back(spring);
	pins.push_back(spring);
}

