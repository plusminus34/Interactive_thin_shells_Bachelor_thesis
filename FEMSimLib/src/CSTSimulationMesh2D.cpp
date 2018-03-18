#include <FEMSimLib/CSTSimulationMesh2D.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <FEMSimLib/CSTElement2D.h>

CSTSimulationMesh2D::CSTSimulationMesh2D(){
}

CSTSimulationMesh2D::~CSTSimulationMesh2D(){
}

void CSTSimulationMesh2D::generateSquareTriMesh(char* fName, double startX, double startY, double dX, double dY, int xSize, int ySize){
	FILE* fp = fopen(fName, "w");
	fprintf(fp, "%d %d\n\n", xSize * ySize, 2 * (xSize-1)* (ySize-1));

	for (int i=0; i<xSize; i++)
		for (int j=0; j<ySize;j++)
			fprintf(fp, "%lf %lf\n", startX + i * dX, startY + j * dY);

	fprintf(fp, "\n\n");

	for (int i=0; i<xSize-1; i++)
		for (int j=0; j<ySize-1;j++){
			fprintf(fp, "%d %d %d\n", i * ySize + j, i * ySize + (j + 1), (i+1)*ySize + j);
			fprintf(fp, "%d %d %d\n", i * ySize + (j+1), (i+1) * ySize + (j + 1), (i+1)*ySize + j);
		}

	fclose(fp);
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

void CSTSimulationMesh2D::setPinnedNode(int ID, const P3D& point){
	P3D tp = point;
	tp[2] = 0;
	for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it) {
		FixedPointSpring2D* fps = dynamic_cast<FixedPointSpring2D*>(*it);
		if (fps->node == nodes[ID]) {
			fps->targetPosition = tp;
			return;
		}
	}
	pinnedNodeElements.push_back(new FixedPointSpring2D(this, nodes[ID], tp));
}

void CSTSimulationMesh2D::readMeshFromFile(const char* fName){
	clear();
	FILE* fp = fopen(fName, "r");

	int nodeCount = 0, triCount = 0;
	fscanf(fp, "%d %d", &nodeCount, &triCount);

	x.resize(2 * nodeCount);
	X.resize(2 * nodeCount);
	v.resize(2 * nodeCount);
	f_ext.resize(2 * nodeCount);
	m.resize(2 * nodeCount);

	for (int i=0; i<nodeCount; i++){
		Node* newNode = new Node(this, i, 2 * i, 2);
		P3D p;
		fscanf(fp, "%lf %lf", &p[0], &p[1]);
		x[2 * i + 0] = p[0]; x[2 * i + 1] = p[1];
		X[2 * i + 0] = p[0]; X[2 * i + 1] = p[1];
		v[2 * i + 0] = 0; v[2 * i + 1] = 0;
		f_ext[2 * i + 0] = 0; f_ext[2 * i + 1] = 0;
		//the masses for the node are obtained by lumping together/distributing the mass of the elements that share the nodes...
		m[2 * i + 0] = 0; m[2 * i + 1] = 0;
		nodes.push_back(newNode);
	}

	for (int i=0;i<triCount; i++){
		int i1, i2, i3;
		fscanf(fp, "%d %d %d", &i1, &i2, &i3);

		CSTElement2D* newElem = new CSTElement2D(this, nodes[i1], nodes[i2], nodes[i3]);
		elements.push_back(newElem);
	}

	fclose(fp);


	energyFunction = new FEMEnergyFunction();
	energyFunction->initialize(this);
}

