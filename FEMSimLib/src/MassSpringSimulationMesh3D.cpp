#include <FEMSimLib/MassSpringSimulationMesh3D.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <FEMSimLib/BilateralSpring3D.h>
#include <FEMSimLib/FixedPointSpring3D.h>

MassSpringSimulationMesh3D::MassSpringSimulationMesh3D(){
}

MassSpringSimulationMesh3D::~MassSpringSimulationMesh3D(){
}

void MassSpringSimulationMesh3D::generateTestMassSpringSystem(char* fName) {
	FILE* fp = fopen(fName, "w");
	fprintf(fp, "%d %d\n\n", 3, 2);

	//generate particles
	for (int j = 0; j<3; j++)
		fprintf(fp, "%lf %lf %lf\n", 0.01*j, 1 - 0.3*j, 0.1*j);

	fprintf(fp, "\n\n");

	//generate springs between particles
	fprintf(fp, "0 1\n");
	fprintf(fp, "1 2\n");

	fclose(fp);
}

//
void MassSpringSimulationMesh3D::readMeshFromFile(const char* fName){
	clear();
	FILE* fp = fopen(fName, "r");

	int nodeCount = 0, edgeCount = 0;
	fscanf(fp, "%d %d", &nodeCount, &edgeCount);

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

	fclose(fp);

	energyFunction = new FEMEnergyFunction();
	energyFunction->initialize(this);

}

int MassSpringSimulationMesh3D::getSelectedNodeID(Ray ray){
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


void MassSpringSimulationMesh3D::setPinnedNode(int ID, const P3D& point) {
	for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it) {
		FixedPointSpring3D* fps = dynamic_cast<FixedPointSpring3D*>(*it);
		if (fps->node == nodes[ID]) {
			fps->targetPosition = point;
			return;
		}
	}
	pinnedNodeElements.push_back(new FixedPointSpring3D(this, nodes[ID], point));
}

