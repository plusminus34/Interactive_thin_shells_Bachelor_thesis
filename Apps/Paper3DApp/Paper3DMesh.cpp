#include "Paper3DMesh.h"
#include <OptimizationLib/NewtonFunctionMinimizer.h>
#include <FEMSimLib/CSTriangle3D.h>
#include <FEMSimLib/FixedPointSpring3D.h>
#include "BendingEdge.h"

Paper3DMesh::Paper3DMesh(){
}

Paper3DMesh::~Paper3DMesh(){
}

void Paper3DMesh::init() {
	setEdgesFromTriangles();

	//Generate triangle elements
	for (int i = 0; i < triangles.rows(); ++i) {
		CSTriangle3D* newElem = new CSTriangle3D(this, nodes[triangles(i,0)], nodes[triangles(i, 1)], nodes[triangles(i, 2)]);
		elements.push_back(newElem);
		for (int j = 0; j < 3; ++j)
			nodes[triangles(i, j)]->adjacentElements.push_back(newElem);
	}
	//Generate edge elements
	for (int i = 0; i < edges.rows(); ++i) {
		int n0, n1, n2, n3;
		//figure out how the two triangles are connected
		int t0 = edges(i, 0), t1 = edges(i, 1);
		int j,k;
		for (int j1 = 0; j1 < 3; ++j1) {
			int j2 = (j1 + 1) % 3;
			for (int k1 = 0; k1 < 3; ++k1) {
				int k2 = (k1 - 1) % 3;
				if (triangles(t0, j1) == triangles(t1, k1) && triangles(t0, j2) == triangles(t1, k2)) {
					j = j1;
					k = k1;
					n0 = triangles(t0, j1);
					n1 = triangles(t0, j2);
				}
			}
		}
		n2 = triangles(t0, (j + 2) % 3);
		n3 = triangles(t1, (k + 1) % 3);
		BendingEdge* newElem = new BendingEdge(this, nodes[n0], nodes[n1], nodes[n2], nodes[n3]);
		elements.push_back(newElem);

		nodes[n0]->adjacentElements.push_back(newElem);
		nodes[n1]->adjacentElements.push_back(newElem);
		nodes[n2]->adjacentElements.push_back(newElem);
		nodes[n3]->adjacentElements.push_back(newElem);
	}

	// helper data structure: stores edges between nodes adjacent to node i (aka edges on the 1-ring of node i)
	std::vector<std::vector<std::vector<int>>> ring_parts;
	ring_parts.resize(nodes.size());
	for (int i = 0; i < triangles.rows(); ++i) {
		for (int j = 0; j < 3; ++j) {
			int j1 = (j + 1) % 3;
			int j2 = (j + 2) % 3;
			int n = triangles(i, j);
			std::vector<int> r(2);
			r[0] = triangles(i,j2); r[1] = triangles(i,j1);
			ring_parts[n].push_back(r);
		}
	}

	boundary.resize(nodes.size());
	orderedAdjacentNodes.resize(nodes.size());
	for (uint i = 0; i < nodes.size(); ++i) {
		boundary[i] = false;
		if (ring_parts[i].size() == 0) continue;

		int n_segments = ring_parts[i].size();
		orderedAdjacentNodes[i].resize(n_segments + 1);

		// arbitrary starting point
		orderedAdjacentNodes[i][0] = ring_parts[i][0][1];
		//find forward path
		for (int j = 1; j < n_segments + 1; ++j) {
			orderedAdjacentNodes[i][j] = -1;
			for (uint k = 0; k < ring_parts[i].size(); ++k) {
				if (orderedAdjacentNodes[i][j - 1] == ring_parts[i][k][0])
					orderedAdjacentNodes[i][j] = ring_parts[i][k][1];
			}
			if (orderedAdjacentNodes[i][j] == -1) {
				boundary[i] = true;
				//current neighbour must be at the end
				orderedAdjacentNodes[i][n_segments] = orderedAdjacentNodes[i][j - 1];
				break;
			}
		}

		if (boundary[i]) {
			//On boundary
			// Find entire path by going backwards from the end
			for (int j = n_segments - 1; j >= 0; --j) {
				orderedAdjacentNodes[i][j] = -1;
				for (uint k = 0; k < ring_parts[i].size(); ++k) {
					if (orderedAdjacentNodes[i][j + 1] == ring_parts[i][k][1])
						orderedAdjacentNodes[i][j] = ring_parts[i][k][0];
				}
			}
			
		}
		else {
			//Not on boundary
			// Last element is the same as the first one -> remove it
			orderedAdjacentNodes[i].pop_back();
		}
	}

	//Debugging output
	/*
	for (uint i = 0; i < orderedAdjacentNodes.size(); ++i) {
		printf("Node %d\thas %d\tneighbours",i, orderedAdjacentNodes[i].size());
		for (uint j = 0; j < orderedAdjacentNodes[i].size(); ++j) {
			printf("\t%d", orderedAdjacentNodes[i][j]);
		}
		printf("\n");
	}
	*/

	//TODO do curved initial configuration elsewhere
	bool curved_initial = true;
	if (curved_initial) {
		for (uint i = 0; i < nodes.size(); ++i) {
			P3D p = nodes[i]->getWorldPosition();
			double angle = (-2*p[0]*p[0] + p[0]) * 0.07 * PI;
			p[2] = p[0] * sin(angle);
			p[0] *= cos(angle);
			nodes[i]->setWorldPosition(p);
		}
	}
}

void Paper3DMesh::setEdgesFromTriangles() {
	//helper data structure 1: stores edges and the triangle they belong to
	std::vector< std::vector<int> > n0n1tr;
	n0n1tr.reserve(3 * triangles.rows());
	for (int i = 0; i < triangles.rows(); ++i) {
		for (int j = 0; j < 3; ++j) {
			int j2 = (j + 1) % 3;
			int n0 = triangles(i, j);
			int n1 = triangles(i, j2);
			if (n0 > n1) std::swap(n0, n1);//order: n0 < n1
			std::vector<int> r(3);
			r[0] = n0; r[1] = n1; r[2] = i;
			n0n1tr.push_back(r);
		}
	}
	std::sort(n0n1tr.begin(), n0n1tr.end());

	//helper data structure 2: stores triangles with an adjacent edge
	std::vector< std::pair<int, int> > t0t1;
	for (int i = 1; i < (int)n0n1tr.size(); ++i) {
		if (n0n1tr[i][0] == n0n1tr[i - 1][0] && n0n1tr[i][1] == n0n1tr[i - 1][1]) {
			t0t1.push_back(std::pair<int, int>(n0n1tr[i - 1][2], n0n1tr[i][2]));
		}
	}

	//fill edges matrix
	edges.resize(t0t1.size(), 2);
	for (int i = 0; i < (int)t0t1.size(); ++i) {
		edges.row(i) << t0t1[i].first, t0t1[i].second;
	}
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

	fclose(fp);
}

void Paper3DMesh::generateRectangleSystem(char* fName, int nodes_x, int nodes_y, double length_x, double length_y) {
	int num_nodes = nodes_x * nodes_y;
	int num_triangles = 2 * (nodes_x - 1)*(nodes_y - 1);

	FILE* fp = fopen(fName, "w");
	fprintf(fp, "%d %d\n\n", num_nodes, num_triangles);

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

	fclose(fp);
}

//
void Paper3DMesh::readMeshFromFile(const char* fName){
	clear();
	FILE* fp = fopen(fName, "r");

	int nodeCount = 0, triangleCount = 0;
	fscanf(fp, "%d %d", &nodeCount, &triangleCount);

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

	triangles.resize(triangleCount, 3);
	for (int i = 0; i<triangleCount; i++) {
		int i1, i2, i3;
		fscanf(fp, "%d %d %d", &i1, &i2, &i3);
		triangles.row(i) << i1, i2, i3;
	}

	fclose(fp);

	init();

	energyFunction = new FEMEnergyFunction();
	energyFunction->initialize(this);

}

void Paper3DMesh::getSaveData(MatrixNxM &xX, Eigen::MatrixXi &T, VectorXT<int> &fixNode, MatrixNxM &fixPos) {
	xX.resize(nodes.size(), 6);
	for (uint i = 0; i < nodes.size(); ++i)
		xX.row(i) << x[3 * i], x[3 * i + 1], x[3 * i + 2], X[3 * i], X[3 * i + 1], X[3 * i + 2];

	T = triangles;

	fixNode.resize(pinnedNodeElements.size());
	fixPos.resize(pinnedNodeElements.size(), 3);
	for (uint i = 0; i < pinnedNodeElements.size(); ++i) {
		FixedPointSpring3D* fps = dynamic_cast<FixedPointSpring3D*>(pinnedNodeElements[i]);
		fixNode[i] = fps->node->nodeIndex;
		fixPos.row(i) << fps->targetPosition[0], fps->targetPosition[1], fps->targetPosition[2];
	}
}

void Paper3DMesh::applyLoadData(MatrixNxM &xX, Eigen::MatrixXi &T, VectorXT<int> &fixNode, MatrixNxM &fixPos) {

	clear();

	int num_nodes = xX.rows();

	dVector x_temp(3 * num_nodes);
	x.resize(3 * num_nodes);
	X.resize(3 * num_nodes);
	v.resize(3 * num_nodes);
	f_ext.resize(3 * num_nodes);
	m.resize(3 * num_nodes);

	for (int i = 0; i < num_nodes; ++i) {
		Node* newNode = new Node(this, i, 3 * i, 3);
		P3D p(xX(i, 0), xX(i, 1), xX(i, 2));
		P3D P(xX(i, 3), xX(i, 4), xX(i, 5));
		for (int j = 0; j < 3; j++) {
			x_temp[3 * i + j] = p[j];
			X[3 * i + j] = P[j];
			v[3 * i + j] = 0;
			f_ext[3 * i + j] = 0;
			m[3 * i + j] = 0;
		}
		nodes.push_back(newNode);
	}
	x = X;

	triangles = T;

	init();

	energyFunction = new FEMEnergyFunction();
	energyFunction->initialize(this);

	x = x_temp;

	for (int i = 0; i < fixNode.size(); ++i)
		setPinnedNode(fixNode[i], P3D(fixPos(i, 0), fixPos(i, 1), fixPos(i, 2)));
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

void Paper3DMesh::cornersOfTriangle(int t, int &c0, int &c1, int &c2) {
	if (t < 0 || t >= triangles.rows()) return;
	c0 = triangles(t, 0);
	c1 = triangles(t, 1);
	c2 = triangles(t, 2);
}

bool Paper3DMesh::areNodesAdjacent(int n0, int n1) {
	for (uint i = 0; i < orderedAdjacentNodes[n0].size(); ++i) {
		if (orderedAdjacentNodes[n0][i] == n1)
			return true;
	}
	return false;
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

void Paper3DMesh::unpinNode(int ID) {//why was this not implemented in the simMesh?
	for (auto it = pinnedNodeElements.begin(); it != pinnedNodeElements.end(); ++it) {
		FixedPointSpring3D* fps = dynamic_cast<FixedPointSpring3D*>(*it);
		if (fps->node == nodes[ID]) {
			delete *it;
			pinnedNodeElements.erase(it);
			return;
		}
	}
}

void Paper3DMesh::replacePin(int ID, Pin* replacement) {
	for (uint i = 0; i < elements.size(); i++) {
		if (Pin* e = dynamic_cast<Pin*>(elements[i])) {
			if (e->getID() == ID) {
				delete elements[i];
				elements[i] = replacement;
			}
		}
	}
}

void Paper3DMesh::deletePin(int ID) {
	for (uint i = 0; i < elements.size(); i++) {
		if (Pin* e = dynamic_cast<Pin*>(elements[i])) {
			if (e->getID() == ID) {
				delete elements[i];
				elements[i] = elements[elements.size() - 1];
				elements.pop_back();
				--i;
			}
		}
	}
}

void Paper3DMesh::makeCut(const DynamicArray<uint>& path) {
	if (path.size() < 2) return;
	// remove edges along path
	for (uint i = 0; i < elements.size(); ++i) {
		if (BendingEdge* e = dynamic_cast<BendingEdge*>(elements[i])) {
			for(uint j = 1; j < path.size(); ++j)
				if ( (e->n[0] == nodes[path[j]] && e->n[1] == nodes[path[j - 1]]) 
					|| (e->n[1] == nodes[path[j]] && e->n[0] == nodes[path[j - 1]])){
					delete e;
					elements[i] = elements[elements.size() - 1];
					elements.pop_back();
					--i;
				}
		}
	}
	//duplicate and reconnect nodes
	int copy_index = -1;
	cutAtNode(-1, path[0], path[1], copy_index);
	for (int i = 1; i < (int)path.size() - 1; ++i) cutAtNode(path[i - 1], path[i], path[i + 1], copy_index);
	cutAtNode(path[path.size() - 2], path[path.size() - 1], -1, copy_index);
	// everything along the cut is now on the boundary
	for (uint i = 0; i < path.size(); ++i) boundary[path[i]] = true;
	// update edges matrix(not that it really matters)
	setEdgesFromTriangles();
}

void Paper3DMesh::cutAtNode(int n_prev, int n, int n_next, int &copy_index) {
	int num_adjacent = orderedAdjacentNodes[n].size();
	bool is_endpoint = (n_prev == -1 || n_next == -1);

	/*
	how to use copy_index
	as input, it is either the index of a copy of n_prev(if node n_prev was copied and its copy is adjacent to n)
	or -1 (if n_prev wasn't copied, or n is the first node on the path)
	as output, it is n_copy (if node n gets copied and an_copy is adjacent to n_next) or -1 (otherwise)
	*/

	int num_copies;

	if (!boundary[n] && is_endpoint) num_copies = 0;
	else if (boundary[n] && !is_endpoint && boundary[n_prev] && boundary[n_next]) num_copies = 0;
	else if (boundary[n] && n_prev == -1 && boundary[n_next]) num_copies = 0;
	else if (boundary[n] && n_next == -1 && boundary[n_prev]) num_copies = 0;

	else if (!boundary[n] && !is_endpoint) num_copies = 1;
	else if (boundary[n] && n_prev == -1 && !boundary[n_next]) num_copies = 1;
	else if (boundary[n] && n_next == -1 && !boundary[n_prev]) num_copies = 1;
	else if (boundary[n] && !is_endpoint && boundary[n_prev] != boundary[n_next]) num_copies = 1;
	//TODO case where n_prev, n, n_next are all on the boundary and the same triangle, but not adjacent along the boundary

	else if (boundary[n] && !is_endpoint && !boundary[n_prev] && !boundary[n_next]) num_copies = 2;

	int num_regions = num_copies + 1;// = num_copies + 1, 3 at most
	// which node (n or a copy of n) belongs to each region
	int n_after[3];
	// indices of each region's start- and endpoint in orderedAdjacentNodes[n]
	int i_region_end[3][2];
	//and those of n_prev and n_next
	int i_prev = -1, i_next = -1;
	//need some kind of label: connects to copy_index instead of n_prev (could probably be improved)
	int region_of_n_prev_copy;
	DynamicArray<int> adjacent_to_n_before = orderedAdjacentNodes[n];
	DynamicArray<int> adjacent_to_n_after[3];

	//figuring out the local structure
	if (!boundary[n]) {
		int current_region = num_regions - 1;
		for (int i = 0; i < num_adjacent; ++i) {
			int n_i = adjacent_to_n_before[i];
			if (n_i == n_prev || n_i == n_next) {
				if (num_regions == 1) {
					i_region_end[0][0] = i_region_end[0][1] = i;
				}
				else {
					i_region_end[current_region][1] = i;
					current_region = (current_region + 1) % num_regions;
					i_region_end[current_region][0] = i;
				}
				if (n_i == n_prev) {
					i_prev = i;
				}
				else if (n_i == n_next) {
					i_next = i;
				}
			}
		}
		if (num_regions == 2 && i_prev > i_next) {
			std::swap(i_region_end[0][0], i_region_end[1][0]);
			std::swap(i_region_end[0][1], i_region_end[1][1]);
		}
	}
	else {// n on boundary
		int current_region = -1;
		for (int i = 0; i < num_adjacent; ++i) {
			int n_i = adjacent_to_n_before[i];
			if (i == 0 || n_i == n_prev || n_i == n_next || i == num_adjacent - 1) {
				if (current_region > -1)
					i_region_end[current_region][1] = i;
				++current_region;
				if (current_region < 3)
					i_region_end[current_region][0] = i;
				if (n_i == n_prev) {
					i_prev = i;
				}
				else if (n_i == n_next) {
					i_next = i;
				}
			}
		}
	}

	// use copy_index? and in which region?
	region_of_n_prev_copy = -1;
	if (copy_index != -1) {
		if (num_regions == 1)
			region_of_n_prev_copy = 0;
		else if (num_regions == 2)
			region_of_n_prev_copy = 1;
		else if (num_regions == 3 && i_prev < i_next)
			region_of_n_prev_copy = 0;
		else if (num_regions == 3 && i_prev > i_next)
			region_of_n_prev_copy = 1;
	}

	//create copies of node n
	n_after[0] = n;
	for (int i = 1; i < num_regions; ++i) {
		n_after[i] = nodes.size();
		Node* newNode = new Node(this, n_after[i], 3 * n_after[i], 3);
		nodes.push_back(newNode);
	}
	// also need to copy data for the node
	int nodeCount = nodes.size();
	x.conservativeResize(3 * nodeCount);
	X.conservativeResize(3 * nodeCount);
	v.conservativeResize(3 * nodeCount);
	f_ext.conservativeResize(3 * nodeCount);
	m.conservativeResize(3 * nodeCount);
	for (int i = 1; i < num_regions; ++i)
		for (int j = 0; j < 3; ++j) {
			x[3 * n_after[i] + j] = x[3 * n + j];
			X[3 * n_after[i] + j] = X[3 * n + j];
			v[3 * n_after[i] + j] = v[3 * n + j];
			f_ext[3 * n_after[i] + j] = f_ext[3 * n + j];
			m[3 * n_after[i] + j] = m[3 * n + j];
			boundary.push_back(true);
		}
	if (num_regions == 3) {
		// reorder so when going in the direction of n_next,
		//  n is to the right of the cut, and the first copy is to the left
		if (i_prev < i_next) {
			n_after[0] = n_after[2];
			n_after[2] = n_after[1];
			n_after[1] = n;
		}
	}

	//update n_adjacent_after
	for (int i = 0; i < num_regions; ++i) {
		adjacent_to_n_after[i].push_back(adjacent_to_n_before[i_region_end[i][0]]);
		for (int j = (i_region_end[i][0] + 1) % num_adjacent; j != i_region_end[i][1]; ++j) {
			adjacent_to_n_after[i].push_back(adjacent_to_n_before[j]);
			if (j + 1 >= num_adjacent) j -= num_adjacent;
		}
		adjacent_to_n_after[i].push_back(adjacent_to_n_before[i_region_end[i][1]]);
	}
	if (num_regions == 2 && boundary[n] && (n_next == -1 || boundary[n_next])) {
		std::swap(adjacent_to_n_after[0], adjacent_to_n_after[1]);
	}
	if (region_of_n_prev_copy != -1)
		adjacent_to_n_after[region_of_n_prev_copy][adjacent_to_n_after[region_of_n_prev_copy].size() - 1] = copy_index;

	//update orderedAdjacentNodes for n and its copies
	orderedAdjacentNodes.resize(orderedAdjacentNodes.size() + num_copies);
	for (int i = 0; i < num_regions; ++i)
		orderedAdjacentNodes[n_after[i]] = adjacent_to_n_after[i];
	// and for the nodes adjacent to n
	for (uint i = 0; i < adjacent_to_n_before.size(); ++i) {
		int n_i = adjacent_to_n_before[i];
		if (n_i == n_next) continue;
		int j = -1;
		for (uint jj = 0; jj < orderedAdjacentNodes[n_i].size(); ++jj)
			if (orderedAdjacentNodes[n_i][jj] == n) {
				j = jj;
				break;
			}
		for (int k = 0; k < num_regions; ++k) {
			if (n_after[k] == n) continue;
			for (uint l = 0; l < adjacent_to_n_after[k].size(); ++l)
				if (adjacent_to_n_after[k][l] == n_i)
					orderedAdjacentNodes[n_i][j] = n_after[k];
		}
	}
	// and also the special case where n_prev was not copied and isn't on the boundary
	if (copy_index == -1 && n_prev != -1 && !boundary[n_prev] && num_regions > 1) {
		orderedAdjacentNodes[n_prev][0] = n_after[num_copies];
		int last = orderedAdjacentNodes[n_prev].size() - 1;
		orderedAdjacentNodes[n_prev][last] = n;
	}

	//update triangles
	for (int i = 0; i < triangles.rows(); ++i) {
		for (int j = 0; j < 3; ++j)
			if (triangles(i, j) == n) {
				int t_k = triangles(i, (j + 1) % 3);
				if (t_k == n_next) {
					// replace n with n, so...
				}
				else if (t_k == n_prev || (copy_index != -1 && t_k == copy_index)) {
					if (num_regions < 3)
						triangles(i, j) = n_after[num_copies];
					else if (num_regions == 3)
						if (i_prev < i_next)
							triangles(i, j) = n_after[0];
						else
							triangles(i, j) = n_after[1];
				}
				else {
					for (int r = 0; r < num_regions; ++r) {
						if (n_after[r] == n) continue;
						for (uint l = 0; l < adjacent_to_n_after[r].size(); ++l)
							if (t_k == adjacent_to_n_after[r][l])
								triangles(i, j) = n_after[r];
					}
				}
			}
	}

	//update adjacentElements
	DynamicArray<SimMeshElement*> adjacentElements = nodes[n]->adjacentElements;
	nodes[n]->adjacentElements.clear();
	for (uint i = 0; i < adjacentElements.size(); ++i) {
		if (CSTriangle3D* e = dynamic_cast<CSTriangle3D*>(adjacentElements[i])) {
			// find out which side of the cut the triangle belongs to
			int replacement = n;
			for (uint j = 0; j < 3; ++j) {
				if (e->n[j] == nodes[n]) {
					int k = (j + 1) % 3;
					if (n_next != -1 && e->n[k] == nodes[n_next]) {
						if (num_regions == 3 && i_prev > i_next) {
							//	replacement = n;
						}
					}
					else if ((n_prev != -1 && e->n[k] == nodes[n_prev]) || (copy_index != -1 && e->n[k] == nodes[copy_index])) {
						if (num_regions < 3)
							replacement = n_after[num_copies];
						else if (num_regions == 3)
							if (i_prev < i_next)
								replacement = n_after[0];
							else
								replacement = n_after[1];
					}
					else {
						for (int r = 0; r < num_regions; ++r) {
							if (n_after[r] == n) continue;
							for (uint l = 0; l < adjacent_to_n_after[r].size(); ++l)
								if (e->n[k] == nodes[adjacent_to_n_after[r][l]])
									replacement = n_after[r];
						}
					}
				}
			}
			if (replacement != n) {
				for (uint j = 0; j < 3; ++j)
					if (e->n[j] == nodes[n])
						e->n[j] = nodes[replacement];
			}
			nodes[replacement]->adjacentElements.push_back(e);
		}
		else if (BendingEdge* e = dynamic_cast<BendingEdge*>(adjacentElements[i])) {
			// find out which side of the cut the edge belongs to
			int replacement = n;
			int j;
			for (uint jj = 0; jj < 4; ++jj)
				if (e->n[jj] == nodes[n]) j = jj;
			if (j < 2) {
				// n is actually on the edge
				//no trouble here because the edges n_prev-n and n-n_next have already been deleted
				int k = (j + 1) % 2;
				for (int r = 0; r < num_regions; ++r) {
					if (n_after[r] == n) continue;
					for (uint l = 0; l < adjacent_to_n_after[r].size(); ++l)
						if (e->n[k] == nodes[adjacent_to_n_after[r][l]])
							replacement = n_after[r];
				}
			}
			else {
				// n is a corner of a triangle adjacent to te edge
				int k = j % 2;
				if (n_next != -1 && e->n[k] == nodes[n_next]) {
					if (num_regions == 3 && i_prev > i_next) {
					}
				}
				else if ((n_prev != -1 && e->n[k] == nodes[n_prev]) || (copy_index != -1 && e->n[k] == nodes[copy_index])) {
					if (num_regions < 3) {
						replacement = n_after[num_copies];
					}
					if (num_regions == 3)
						if (i_prev < i_next)
							replacement = n_after[0];
						else
							replacement = n_after[1];
				}
				else {
					for (int r = 0; r < num_regions; ++r) {
						if (n_after[r] == n) continue;
						for (uint l = 0; l < adjacent_to_n_after[r].size(); ++l)
							if (e->n[k] == nodes[adjacent_to_n_after[r][l]])
								replacement = n_after[r];
					}
				}
			}
			if (replacement != n) {
				e->n[j] = nodes[replacement];
			}
			nodes[replacement]->adjacentElements.push_back(e);
		}
	}

	//set copy_index
	bool next_needs_copy = false;
	if (num_copies == 1) {
		copy_index = -1;
		for (uint i = 0; i < adjacent_to_n_after[1].size(); ++i)
			if (adjacent_to_n_after[1][i] == n_next)
				copy_index = n_after[1];
	}
	else if (num_copies == 2) {
		if (i_prev < i_next)
			copy_index = n_after[2];
		else
			copy_index = n_after[1];
	}
}