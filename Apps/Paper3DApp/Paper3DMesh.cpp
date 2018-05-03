#include "Paper3DMesh.h"
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <FEMSimLib/CSTriangle3D.h>
#include <FEMSimLib/FixedPointSpring3D.h>
#include "BendingEdge.h"

Paper3DMesh::Paper3DMesh(){
}

Paper3DMesh::~Paper3DMesh(){
}

void Paper3DMesh::generateTestSystem(char* fName, int num_nodes) {
	FILE* fp = fopen(fName, "w");
	fprintf(fp, "%d %d %d\n\n", num_nodes, num_nodes - 2, num_nodes - 3);

	//generate nodes
	for (int j = 0; j<num_nodes; j++)
		fprintf(fp, "%lf %lf %lf\n", j*0.2, 0.4*cos(j*PI), -0.01*j*j);

	fprintf(fp, "\n\n");

	//generate triangles
	for (int j = 2; j<num_nodes; j++)
		fprintf(fp, "%li %li %li\n", j, j - 1, j - 2);

	//generate bending edges
	for (int j = 3; j < num_nodes; j++)
		fprintf(fp, "%li %li %li %li\n", j-2, j - 1, j - 3, j);

	fclose(fp);
}

void Paper3DMesh::generateRectangleSystem(char* fName, int nodes_x, int nodes_y, double length_x, double length_y) {
	if (nodes_x < 2 || nodes_y < 2) return;//at least 4 nodes
	int num_nodes = nodes_x * nodes_y;
	int num_triangles = 2 * (nodes_x - 1)*(nodes_y - 1);
	int num_edges = num_triangles / 2 + (nodes_x - 2) * (nodes_y-1) + (nodes_x-1) * (nodes_y - 2);//that right? nooo

	FILE* fp = fopen(fName, "w");
	fprintf(fp, "%d %d %d\n\n", num_nodes, num_triangles, num_edges);

	double h_x = length_x / (nodes_x - 1);
	double h_y = length_y / (nodes_y - 1);

	// node positions
	for (int i = 0; i < nodes_x; ++i)
		for (int j = 0; j < nodes_y;++j)
			fprintf(fp, "%lf %lf %lf\n", i*h_x, j*h_y, 0.0);

	fprintf(fp, "\n\n");

	// triangles
	for (int i = 1; i < nodes_x; ++i)
		for (int j = 1; j < nodes_y;++j) {
			int n = i * nodes_y + j;//index of node (i,j)
			fprintf(fp, "%li %li %li\n", n, n - 1, n - nodes_y - 1);
			fprintf(fp, "%li %li %li\n", n, n - nodes_y -1, n - nodes_y);
		}

	fprintf(fp, "\n\n");

	//connect triangle pairs
	for (int i = 1; i < nodes_x; ++i)
		for (int j = 1; j < nodes_y;++j) {
			int n = i * nodes_y + j;//index of node (i,j)
			fprintf(fp, "%li %li %li %li\n", n, n - nodes_y - 1, n - nodes_y, n - 1);
		}
	//connect rows
	for (int i = 1;i<nodes_x;++i)
		for (int j = 2;j < nodes_y;++j) {
			int n = i * nodes_y + j;
			fprintf(fp, "%li %li %li %li\n", n - nodes_y - 1, n - 1, n - nodes_y - 2, n);
		}
	//connect columns
	for(int i=2; i<nodes_x;++i)
		for (int j = 1; j < nodes_y;++j) {
			int n = i * nodes_y + j;
			fprintf(fp, "%li %li %li %li\n", n - nodes_y, n - nodes_y - 1, n - 2 * nodes_y - 1, n);
		}

	fclose(fp);
}

//
void Paper3DMesh::readMeshFromFile(const char* fName){
	clear();
	FILE* fp = fopen(fName, "r");

	int nodeCount = 0, triangleCount = 0, bendingEdgeCount;
	fscanf(fp, "%d %d %d", &nodeCount, &triangleCount, &bendingEdgeCount);

	x.resize(3 * nodeCount);
	X.resize(3 * nodeCount);
	v.resize(3 * nodeCount);
	f_ext.resize(3 * nodeCount);
	m.resize(3 * nodeCount);

	for (int i = 0; i < nodeCount; i++) {
		Node* newNode = new Node(this, i, 3 * i, 3);
		P3D p;
		fscanf(fp, "%lf %lf %lf", &p[0], &p[1], &p[2]);
		for (int j = 0; j < 3; j++) {
			x[3 * i + j] = p[j];
			X[3 * i + j] = p[j];
			v[3 * i + j] = 0;
			f_ext[3 * i + j] = 0;
			//the masses for the node are obtained by lumping together/distributing the mass of the elements that share the nodes...
			m[3 * i + j] = 0;
		}
		nodes.push_back(newNode);
	}

	for (int i = 0; i<triangleCount; i++) {
		int i1, i2, i3;
		fscanf(fp, "%d %d %d", &i1, &i2, &i3);

		CSTriangle3D* newElem = new CSTriangle3D(this, nodes[i1], nodes[i2], nodes[i3]);
		elements.push_back(newElem);
	}

	for (int i = 0; i<bendingEdgeCount; i++) {
		int i1, i2, i3, i4;
		fscanf(fp, "%d %d %d %d", &i1, &i2, &i3, & i4);

		BendingEdge* newElem = new BendingEdge(this, nodes[i1], nodes[i2], nodes[i3], nodes[i4]);
		elements.push_back(newElem);
	}

	fclose(fp);

	energyFunction = new FEMEnergyFunction();
	energyFunction->initialize(this);

}

int Paper3DMesh::getSelectedNodeID(Ray ray){
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


void Paper3DMesh::setPinnedNode(int ID, const P3D& point) {
	for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it) {
		FixedPointSpring3D* fps = dynamic_cast<FixedPointSpring3D*>(*it);
		if (fps->node == nodes[ID]) {
			fps->targetPosition = point;
			return;
		}
	}
	pinnedNodeElements.push_back(new FixedPointSpring3D(this, nodes[ID], point));
}

