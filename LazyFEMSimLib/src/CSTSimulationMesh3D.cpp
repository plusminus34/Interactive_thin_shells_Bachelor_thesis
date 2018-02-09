#include <LazyFEMSimLib/CSTSimulationMesh3D.h>
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <LazyFEMSimLib/CSTElement3D.h>
#include <LazyFEMSimLib/FixedPointSpring3D.h>
#include <tetgen/tetgen.h>

CSTSimulationMesh3D::CSTSimulationMesh3D()
{
}


CSTSimulationMesh3D::~CSTSimulationMesh3D()
{
}

void read(FILE *fp, int &x, char &ch)
{
	x = 0;
	while (ch > '9' || ch < '0')
		fscanf(fp, "%c", &ch);
	do
	{
		x = x * 10 + ch - '0';
		if (feof(fp))
			return;
		fscanf(fp, "%c", &ch);
	} while (ch <= '9' && ch >= '0');
	while (ch != ' ' && ch != '\n')
	{
		if (feof(fp))
			return;
		if (fscanf(fp, "%c", &ch) != 1)
			return;
	}
}



//
void CSTSimulationMesh3D::readMeshFromFile(const char* fName)
{
	FILE *fp = fopen(fName, "r");
	tetgenio input;
	input.mesh_dim = 3;
	char ch1, ch2;
	std::vector<double> coor;
	std::vector<int> faces;
	int &n = input.numberofpoints;
	int &f = input.numberoffacets;
	while (1)
	{
		if (feof(fp))
			break;
		if (fscanf(fp, "%c%c", &ch1, &ch2) != 2)
			break;
		if (ch1 == 'v' && ch2 == ' ')
		{
			double x, y, z;
			fscanf(fp, "%lf%lf%lf", &x, &y, &z);
			++n;
			coor.push_back(x);
			coor.push_back(y);
			coor.push_back(z);
		}
		else if (ch1 == 'f' && ch2 == ' ')
		{
			int x0, x1, x2;
			read(fp, x0, ch2);
			read(fp, x1, ch2);
			read(fp, x2, ch2);
			++f;
			faces.push_back(x0 - 1);
			faces.push_back(x1 - 1);
			faces.push_back(x2 - 1);
			//printf("--%c--\n", ch2);
		}
		if (feof(fp))
			break;
		while (ch2 != '\n')
		{
			if (feof(fp))
				break;
			if (fscanf(fp, "%c", &ch2) != 1)
				break;
		}
	}
	fclose(fp);
	input.pointlist = new REAL[n * 3];
	input.pointmarkerlist = new int[n];
	//printf("%d %d\n", n, f);
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			input.pointlist[i * 3 + j] = coor[i * 3 + j];
		}
		input.pointmarkerlist[i] = 1;
	}
	input.facetlist = new tetgenio::facet[f];
	input.facetmarkerlist = new int[f];
	for (int i = 0; i < f; ++i)
	{
		tetgenio::facet &cur = input.facetlist[i];
		cur.numberofholes = 0;
		cur.holelist = NULL;
		cur.numberofpolygons = 1;
		cur.polygonlist = new tetgenio::polygon[1];
		tetgenio::polygon &p = cur.polygonlist[0];
		p.numberofvertices = 3;
		p.vertexlist = new int[3];
		for (int j = 0; j < 3; ++j)
		{
			p.vertexlist[j] = faces[i * 3 + j];
		}
		input.facetmarkerlist[i] = 1;
	}
	tetgenio output;
	tetgenbehavior b;
	input.save_poly("../data/FEM/3d/cube");
	input.save_nodes("../data/FEM/3d/cube");
	//b.nobisect = 1;
	b.plc = 1;
	b.coarsen = 1;
	//freopen("../data/FEM/3d/cube.out", "w", stdout);
	tetrahedralize(&b, &input, &output);
	//output.save_nodes("../data/FEM/3d/cube");
	//output.save_elements("../data/FEM/3d/cube");
	//system("pause");
	int &nodeCount = output.numberofpoints;
	int &tetCount = output.numberoftetrahedra;
	x.resize(3 * nodeCount);
	X.resize(3 * nodeCount);
	v.resize(3 * nodeCount);
	f_ext.resize(3 * nodeCount);
	m.resize(3 * nodeCount);
	for (int i = 0; i<nodeCount; i++) {
		Node* newNode = new Node(this, i, 3 * i, 3);
		double *p = output.pointlist + 3 * i;
		x[3 * i + 0] = p[0]; x[3 * i + 1] = p[1]; x[3 * i + 2] = p[2];
		X[3 * i + 0] = p[0]; X[3 * i + 1] = p[1]; X[3 * i + 2] = p[2];
		v[3 * i + 0] = 0; v[3 * i + 1] = 0; v[3 * i + 2] = 0;
		f_ext[3 * i + 0] = 0; f_ext[3 * i + 1] = 0; f_ext[3 * i + 2] = 0;
		//the masses for the node are obtained by lumping together/distributing the mass of the elements that share the nodes...
		m[3 * i + 0] = 0; m[3 * i + 1] = 0; m[3 * i + 2] = 0;
		nodes.push_back(newNode);
	}
	for (int i = 0;i<tetCount; i++) {
		CSTElement3D* newElem = new CSTElement3D(this, nodes[output.tetrahedronlist[i * 4]], nodes[output.tetrahedronlist[i * 4 + 1]],
			nodes[output.tetrahedronlist[i * 4 + 2]], nodes[output.tetrahedronlist[i * 4 + 3]]);
		elements.push_back(newElem);
	}
	energyFunction = new FEMEnergyFunction();
	energyFunction->initialize(this);
}


void CSTSimulationMesh3D::readMeshFromFile_ply(char* fName, DynamicArray<P3D> const * add_input_points,
											   double massDensity, double shearModulus, double bulkModulus,
											   double scale, V3D const & offset,
											   double maxTetVolume)
{
	// input objects for tetget
	tetgenio input, addinput;
	input.mesh_dim = 3;
	// load model
	input.load_ply(fName);

	// set behavior for tetgen
	tetgenbehavior b;
	b.plc = 1;
	b.coarsen = 1;
	//b.refine = 1;
	b.quality = 1;

	if(maxTetVolume > 0.0) {
		b.fixedvolume = 1;
		b.maxvolume = maxTetVolume;//1.35e-6;
	}


	//b.minratio = 2.0;
	//b.mindihedral = 1.0;
	b.verbose = 1;

	// set additional points
	if(add_input_points && add_input_points->size() > 0) {
		int m = add_input_points->size();
		b.insertaddpoints = 1;
		addinput.pointlist = new REAL[m * 3];
		addinput.pointmarkerlist = new int[m];
		addinput.numberofpoints = m;
		for(int i = 0; i < m; ++i) {
			for(int j = 0; j < 3; ++j) {
				addinput.pointlist[i*3+j] = (*add_input_points)[i][j];
				addinput.pointmarkerlist[i] = 1;
			}
		}
	}

	// output
	tetgenio output;
	// create mesh (tetrahedons)
	tetrahedralize(&b, &input, &output, &addinput);

	// create CST Mesh 
	int &nodeCount = output.numberofpoints;
	int &tetCount = output.numberoftetrahedra;
	x.resize(3 * nodeCount);
	X.resize(3 * nodeCount);
	v.resize(3 * nodeCount);
	f_ext.resize(3 * nodeCount);
	m.resize(3 * nodeCount);

	// apply scaling and offset
	for(int i = 0; i < nodeCount; ++i) {
		for(int j = 0; j < 3; ++j) {
			output.pointlist[i*3 + j] *= scale;
			output.pointlist[i*3 + j] += offset(j);
		}
	}

	for (int i = 0; i<nodeCount; i++) {
		Node* newNode = new Node(this, i, 3 * i, 3);
		double *p = output.pointlist + 3 * i;
		x[3 * i + 0] = p[0]; x[3 * i + 1] = p[1]; x[3 * i + 2] = p[2];
		X[3 * i + 0] = p[0]; X[3 * i + 1] = p[1]; X[3 * i + 2] = p[2];
		v[3 * i + 0] = 0; v[3 * i + 1] = 0; v[3 * i + 2] = 0;
		f_ext[3 * i + 0] = 0; f_ext[3 * i + 1] = 0; f_ext[3 * i + 2] = 0;
		//the masses for the node are obtained by lumping together/distributing the mass of the elements that share the nodes...
		m[3 * i + 0] = 0; m[3 * i + 1] = 0; m[3 * i + 2] = 0;
		nodes.push_back(newNode);
	}
	for (int i = 0;i<tetCount; i++) {
		CSTElement3D* newElem = new CSTElement3D(this, nodes[output.tetrahedronlist[i * 4]], nodes[output.tetrahedronlist[i * 4 + 1]],
												 nodes[output.tetrahedronlist[i * 4 + 2]], nodes[output.tetrahedronlist[i * 4 + 3]],
												 massDensity, shearModulus, bulkModulus);
		elements.push_back(newElem);
	}
	energyFunction = new FEMEnergyFunction();
	energyFunction->initialize(this);
}

void add(std::map<int, int> &num, int k, int &cnt, std::vector<int> &arr)
{
	//	std::cout << k << std::endl;
	num[k] = ++cnt;
	arr.push_back(k);
}

void genPoints(std::map<int, int> &num, int sx, int sy, int sz, std::vector<int> &arr)
{
	int cnt = 0;
	num.clear();
	for (int i = 0; i <= sx; ++i)
	{
		bool faceX = (i == 0) || (i == sx);
		for (int j = 0; j <= sy; ++j)
		{
			bool faceY = (j == 0) || (j == sy);
			add(num, (i*(sx + 1) + j)*(sy + 1) + 0, cnt, arr);
			if (faceX || faceY)
				for (int k = 1; k < sz; ++k)
				add(num, (i*(sx + 1) + j)*(sy + 1) + k, cnt, arr);
			add(num, (i*(sx + 1) + j)*(sy + 1) + sz, cnt, arr);
		}
	}
}

void CSTSimulationMesh3D::generateCubeTriMesh(char* fName)
{
	const int sizeX = 5, sizeY = 5, sizeZ = 5;
	const double offsetX = 0, offsetY = 0, offsetZ = 0;
	const double lenX = 1, lenY = 1, lenZ = 1;
	FILE* fp = fopen(fName, "w");
	std::map<int, int> pointNum;
	std::vector<int> arr;
	arr.clear();
	arr.push_back(0);
	genPoints(pointNum, sizeX, sizeY, sizeZ, arr);
	for (uint i = 1; i < arr.size(); ++i)
	{
		int num = arr[i];
		int z = num % (sizeZ + 1); num /= sizeZ + 1;
		int y = num % (sizeY + 1); num /= sizeY + 1;
		int x = num;
		fprintf(fp, "v %.6lf %.6lf %.6lf\n", offsetX + x * lenX, offsetY + y * lenY, offsetZ + z * lenZ);
	}
	int x, y, z;
#define get(x,y,z) (((x)*(sizeY+1)+(y))*(sizeZ+1)+(z))
	//x
	for (x = 0; x <= sizeX; x += sizeX)
		for (y = 0; y < sizeY; ++y)
		for (z = 0; z < sizeZ; ++z)
		{
			fprintf(fp, "f %d", pointNum[get(x, y, z)]);
			fprintf(fp, " %d", pointNum[get(x, y + (x != 0), z + 1)]);
			fprintf(fp, " %d\n", pointNum[get(x, y + (x == 0), z + 1)]);
			fprintf(fp, "f %d", pointNum[get(x, y, z)]);
			fprintf(fp, " %d", pointNum[get(x, y + 1, z + (x == 0))]);
			fprintf(fp, " %d\n", pointNum[get(x, y + 1, z + (x != 0))]);
		}
	//y
	for (y = 0; y <= sizeY; y += sizeY)
		for (z = 0; z < sizeZ; ++z)
		for (x = 0; x < sizeX; ++x)
		{
			fprintf(fp, "f %d", pointNum[get(x, y, z)]);
			fprintf(fp, " %d", pointNum[get(x + 1, y, z + (y != 0))]);
			fprintf(fp, " %d\n", pointNum[get(x + 1, y, z + (y == 0))]);
			fprintf(fp, "f %d", pointNum[get(x, y, z)]);
			fprintf(fp, " %d", pointNum[get(x + (y == 0), y, z + 1)]);
			fprintf(fp, " %d\n", pointNum[get(x + (y != 0), y, z + 1)]);
		}
	//z
	for (z = 0; z <= sizeZ; z += sizeZ)
		for (x = 0; x < sizeX; ++x)
		for (y = 0; y < sizeY; ++y)
		{
			fprintf(fp, "f %d", pointNum[get(x, y, z)]);
			fprintf(fp, " %d", pointNum[get(x + (z != 0), y + 1, z)]);
			fprintf(fp, " %d\n", pointNum[get(x + (z == 0), y + 1, z)]);
			fprintf(fp, "f %d", pointNum[get(x, y, z)]);
			fprintf(fp, " %d", pointNum[get(x + 1, y + (z == 0), z)]);
			fprintf(fp, " %d\n", pointNum[get(x + 1, y + (z != 0), z)]);
		}
	fclose(fp);
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
	for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it){
		FixedPointSpring3D* fps = dynamic_cast<FixedPointSpring3D*>(*it);
		if (fps->node == nodes[ID]){
			fps->targetPosition = point;
			return;
		}
	}
	pinnedNodeElements.push_back(new FixedPointSpring3D(this, nodes[ID], point));
}

