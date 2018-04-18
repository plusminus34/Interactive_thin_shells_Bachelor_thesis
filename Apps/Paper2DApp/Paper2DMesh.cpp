#include "Paper2DMesh.h"
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <FEMSimLib/BilateralSpring3D.h>
#include <FEMSimLib/FixedPointSpring3D.h>
#include "AngleSpring.h"

Paper2DMesh::Paper2DMesh(){
}

Paper2DMesh::~Paper2DMesh(){
}

void Paper2DMesh::generateSinMassSpringSystem(char* fName, int num_nodes) {
	FILE* fp = fopen(fName, "w");
	fprintf(fp, "%d %d %d\n\n", num_nodes, num_nodes - 1, num_nodes - 2);

	std::vector<double> xpos(num_nodes);
	std::vector<double> ypos(num_nodes);

	xpos[0] = ypos[0] = 0.0;

	/*
	//Silly way to get equidistant nodes
	double dx = 0.1;
	double nnn = 0.5;
	double target_length = 3.0;
	double eps = 0.01;
	for (int i = 1;i < num_nodes;++i) {
		xpos[i] = xpos[i - 1] + dx;
		ypos[i] = sin(xpos[i] * nnn * 2 * PI);
		double l2 = (xpos[i] - xpos[i - 1])*(xpos[i] - xpos[i - 1]) + (ypos[i] - ypos[i - 1])*(ypos[i] - ypos[i - 1]);
		double target_l2 = target_length / (num_nodes - 1);
		if ((l2 - target_l2)*(l2 - target_l2) > eps) {
			if (l2 > target_l2) {
				dx *= 0.9;
			}
			else {
				dx *= 1.1;
			}
			--i;
		}
	}
	*/

	//the boring way, absolute distance between nodes varies
	double h = 1.0 / (num_nodes - 1);
	for (int i = 1;i < num_nodes;++i) {
		xpos[i] = h*i;
		ypos[i] = sin(2 * PI*h*i);
	}

	//generate particles
	for (int j = 0; j<num_nodes; j++)
		fprintf(fp, "%lf %lf %lf\n", xpos[j], ypos[j], 0.0);

	fprintf(fp, "\n\n");

	//generate springs between particles
	for (int j = 1; j<num_nodes; j++)
		fprintf(fp, "%li %li\n", j - 1, j);

	//generate angle springs
	for (int j = 1; j<num_nodes - 1; j++)
		fprintf(fp, "%li %li %li\n", j - 1, j, j + 1);

	fclose(fp);
}

//
void Paper2DMesh::readMeshFromFile(const char* fName){
	clear();
	FILE* fp = fopen(fName, "r");

	int nodeCount = 0, edgeCount = 0, angleCount = 0;
	fscanf(fp, "%d %d %d", &nodeCount, &edgeCount, &angleCount);

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

	for (int i = 0; i<edgeCount; i++) {
		int i1, i2;
		fscanf(fp, "%d %d", &i1, &i2);

		BilateralSpring3D* newElem = new BilateralSpring3D(this, nodes[i1], nodes[i2]);
		elements.push_back(newElem);
	}

	for (int i = 0; i<angleCount; i++) {
		int i1, i2, i3;
		fscanf(fp, "%d %d %d", &i1, &i2, &i3);

		AngleSpring* newElem = new AngleSpring(this, nodes[i1], nodes[i2], nodes[i3]);
		elements.push_back(newElem);
	}

	fclose(fp);

	energyFunction = new FEMEnergyFunction();
	energyFunction->initialize(this);

}

int Paper2DMesh::getSelectedNodeID(Ray ray){
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


void Paper2DMesh::setPinnedNode(int ID, const P3D& point) {
	for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it) {
		FixedPointSpring3D* fps = dynamic_cast<FixedPointSpring3D*>(*it);
		if (fps->node == nodes[ID]) {
			fps->targetPosition = point;
			return;
		}
	}
	pinnedNodeElements.push_back(new FixedPointSpring3D(this, nodes[ID], point));
}

