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
	for (int i = 1; i < (int) n0n1tr.size(); ++i) {
		if (n0n1tr[i][0] == n0n1tr[i - 1][0] && n0n1tr[i][1] == n0n1tr[i - 1][1]) {
			t0t1.push_back(std::pair<int, int>(n0n1tr[i - 1][2], n0n1tr[i][2]));
		}
	}

	//fill edges matrix
	edges.resize(t0t1.size(), 2);
	for (int i = 0; i < (int) t0t1.size(); ++i) {
		edges.row(i) << t0t1[i].first, t0t1[i].second;
	}

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
		bool n0found = false;
		for (int j = 0; j < 3; ++j)
			for (int k = 0; k < 3; ++k)
				if (triangles(t0, j) == triangles(t1, k)) {
					if (n0found) {
						n1 = triangles(t0, j);
					}
					else {
						n0 = triangles(t0, j);
						n0found = true;
					}
				}
		for (int j = 0; j < 3; ++j) {
			if (triangles(t0, j) != n0 && triangles(t0, j) != n1) n2 = triangles(t0, j);
			if (triangles(t1, j) != n0 && triangles(t1, j) != n1) n3 = triangles(t1, j);
		}
		BendingEdge* newElem = new BendingEdge(this, nodes[n0], nodes[n1], nodes[n2], nodes[n3]);
		elements.push_back(newElem);

		nodes[n0]->adjacentElements.push_back(newElem);
		nodes[n1]->adjacentElements.push_back(newElem);
		nodes[n2]->adjacentElements.push_back(newElem);
		nodes[n3]->adjacentElements.push_back(newElem);
	}

	// helper data structure 3: stores edges between nodes adjacent to node i (aka edges on the 1-ring of node i)
	std::vector<std::vector<std::vector<int>>> hjg;//TODO rename this, hjg is not a good name
	hjg.resize(nodes.size());
	for (int i = 0; i < triangles.rows(); ++i) {
		for (int j = 0; j < 3; ++j) {
			int j1 = (j + 1) % 3;
			int j2 = (j + 2) % 3;
			int n = triangles(i, j);
			std::vector<int> r(2);
			r[0] = triangles(i,j2); r[1] = triangles(i,j1);
			hjg[n].push_back(r);
		}
	}

	boundary.resize(nodes.size());
	orderedAdjacentNodes.resize(nodes.size());
	for (uint i = 0; i < nodes.size(); ++i) {
		boundary[i] = false;
		if (hjg[i].size() == 0) continue;// avoid problems with unconnected nodes

		int n_segments = hjg[i].size();
		orderedAdjacentNodes[i].resize(n_segments + 1);

		// arbitrary starting point
		orderedAdjacentNodes[i][0] = hjg[i][0][1];
		//find forward path
		for (uint j = 1; j < n_segments + 1; ++j) {
			orderedAdjacentNodes[i][j] = -1;
			for (uint k = 0; k < hjg[i].size(); ++k) {
				if (orderedAdjacentNodes[i][j - 1] == hjg[i][k][0])
					orderedAdjacentNodes[i][j] = hjg[i][k][1];
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
				for (uint k = 0; k < hjg[i].size(); ++k) {
					if (orderedAdjacentNodes[i][j + 1] == hjg[i][k][1])
						orderedAdjacentNodes[i][j] = hjg[i][k][0];
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

	//TODO do this elsewhere
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
			}
		}
	}
}

void Paper3DMesh::makeCut(const DynamicArray<uint>& path) {
	if (path.size() < 2) return;
	/*
	how to implement?
	"duplicate node" is not that simple: have to connect stuff correctly too!
	possibility: during cutting, store a "left" and "right" node (or "original" and "duplicate")
		or not:
		remove edges along path
		duplicate first node in path if needed
		for each node n, decide how many duplicates there need to be, then create them
		  for each neighbour node(except those on the path), assign one new node(original node or one of up to two duplicates)
		  for each adjacent element, map it to one adjacent node and replace n in the element by the node assigned previously
	*/
	// remove edges along path
	for (int i = 0; i < elements.size(); ++i) {
		if (BendingEdge* e = dynamic_cast<BendingEdge*>(elements[i])) {
			for(int j=1;j<path.size();++j)
				if ( (e->n[0] == nodes[path[j]] && e->n[1] == nodes[path[j - 1]]) 
					|| (e->n[1] == nodes[path[j]] && e->n[0] == nodes[path[j - 1]])){
					delete e;
					elements[i] = elements[elements.size() - 1];
					elements.pop_back();
				}
		}
	}
	//Plan
	printf("cut path:");
	for (int i = 0; i < path.size(); ++i)printf(" %d", path[i]);
	printf("\n");
	int nNodes = nodes.size();
	printf("(start with %d nodes)\n", nNodes);
	for (int i = 0; i < path.size(); ++i) {
		bool is_endpoint = (i == 0) || (i == path.size() - 1);
		bool on_boundary = boundary[path[i]];
		int n = path[i];
		if (is_endpoint && on_boundary) {
			if (i == 0 && boundary[path[i + 1]]){
				printf("do nothing to node %d\t(first cut is along boundary)\n", n);
			}
			else if (i == path.size() - 1 && boundary[path[i - 1]]) {
				printf("do nothing to node %d\t(last cut is along boundary)\n", n);
			}
			else {
				printf("duplicate node %d\t(endpoint on boundary)\n", n);
				++nNodes;
			}
		}
		else if (is_endpoint && !on_boundary) {
			printf("do nothing to node %d\t(endpoint inside)\n", n);
		}
		else if (!is_endpoint && !on_boundary) {
			printf("duplicate node %d\t(ordinary point inside mesh)\n", n);
			++nNodes;
		}
		else if (!is_endpoint && on_boundary) {
			if (boundary[path[i - 1]] && boundary[path[i + 1]]) {
				printf("do nothing to node %d\t(cut along boundary)\n", n);
			}
			else if(boundary[path[i - 1]] && !boundary[path[i + 1]]) {
				printf("duplicate node %d\t(next is no longer on boundary)\n", n);
				++nNodes;
			}
			else if (!boundary[path[i - 1]] && boundary[path[i + 1]]) {
				printf("duplicate node %d\t(previous wasn't on boundary)\n", n);
				++nNodes;
			}
			else if (!boundary[path[i - 1]] && !boundary[path[i + 1]]) {
				printf("duplicate node %d twice\t(previous and next are inside)\n", n);
				++nNodes;
				++nNodes;
			}
		}
	}
	printf("(end with %d nodes)\n", nNodes);
	//TODO cut_at_node
	int copindex = -1;
	cutAtNode(-1, path[0], path[1], copindex);
	for (int i = 1; i < path.size() - 1; ++i) cutAtNode(path[i - 1], path[i], path[i + 1], copindex);
	cutAtNode(path[path.size() - 2], path[path.size() - 1], -1, copindex);
	for (int i = 0; i < path.size(); ++i) boundary[path[i]] = true;
	//TODO update edges matrix(not that it really matters)
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

	else if (boundary[n] && !is_endpoint && !boundary[n_prev] && !boundary[n_next]) num_copies = 2;

	printf("cutAtNode %d - %d - %d (copy_index %d)\n", n_prev, n, n_next, copy_index);
	if (num_copies == 0) {
		printf("\tmake no copy\n");
		if (copy_index != -1) {
			//only happens if n is an endpoint and not on the boundary
			int start_i = 0;
			for (int i = 0; i < num_adjacent; ++i) {
				if (orderedAdjacentNodes[n][i] == n_prev) {
					start_i = i;
					break;
				}
			}
			bool in_first_region = true;
			DynamicArray<int> adjacent_to_n;
			//n_prev-x-y-z-copy_index
			for (int i = 0; i < num_adjacent; ++i) {
				adjacent_to_n.push_back(orderedAdjacentNodes[n][(start_i + 1) % num_adjacent]);
			}
			adjacent_to_n.push_back(copy_index);
			orderedAdjacentNodes[n] = adjacent_to_n;
		}
		copy_index = -1;
	}
	else if (num_copies == 1) {
		printf("\tmake one copy\n");//TODO problem at |-b-i
		// copy node n and reconnect mesh parts so the copy will be on the left side of the cut

		// create a new node
		int n_copy = nodes.size();
		Node* newNode = new Node(this, n_copy, 3 * n_copy, 3);
		nodes.push_back(newNode);
		// also need to copy data for the node
		int nodeCount = n_copy + 1;
		x.conservativeResize(3 * nodeCount);
		X.conservativeResize(3 * nodeCount);
		v.conservativeResize(3 * nodeCount);
		f_ext.conservativeResize(3 * nodeCount);
		m.conservativeResize(3 * nodeCount);
		for (int i = 0; i < 3; ++i) {
			x[3 * n_copy + i] = x[3 * n + i];
			X[3 * n_copy + i] = X[3 * n + i];
			v[3 * n_copy + i] = v[3 * n + i];
			f_ext[3 * n_copy + i] = f_ext[3 * n + i];
			m[3 * n_copy + i] = m[3 * n + i];
		}
		//boundary[n] = true;//set this later to avoid trouble with boundary[n_prev] in the next step
		boundary.push_back(true);
		printf("\tnew node created\n");
		int start_i = 0;
		for (int i = 0; i < num_adjacent; ++i) {
			if (orderedAdjacentNodes[n][i] == n_prev) {
				start_i = i;
				break;
			}
		}
		bool in_first_region = true;
		printf("\twill update orderedAdjacent\n");
		// update orderedAdjacentNodes
		DynamicArray<int> adjacent_to_n, adjacent_to_n_copy;
		if (n_next != -1) {
			for (int ii = 0; ii < num_adjacent; ++ii) {
				int i = (start_i + ii) % num_adjacent;
				if (orderedAdjacentNodes[n][i] == n_next) {
					in_first_region = false;
					adjacent_to_n.push_back(orderedAdjacentNodes[n][i]);
					if (!(boundary[n] && boundary[n_next]))
						adjacent_to_n_copy.push_back(orderedAdjacentNodes[n][i]);
				}
				else if (in_first_region) {
					adjacent_to_n.push_back(orderedAdjacentNodes[n][i]);
				}
				else {
					adjacent_to_n_copy.push_back(orderedAdjacentNodes[n][i]);
				}
			}
			if (copy_index == -1 && n_prev != -1) {
				adjacent_to_n_copy.push_back(orderedAdjacentNodes[n][start_i]);
			}
			else if (n_prev != -1) {
				adjacent_to_n_copy.push_back(copy_index);
			}
			printf("\tn:");
			for (int i = 0; i < adjacent_to_n.size(); ++i)printf("\t%d", adjacent_to_n[i]);
			printf("\n\tn_c:");
			for (int i = 0; i < adjacent_to_n_copy.size(); ++i)printf("\t%d", adjacent_to_n_copy[i]);
			printf("\n");
		}
		else {
			//only happens if n_prev is inside the mesh and n (the last node on the path) is on the boundary
			for (int i = 0; i < num_adjacent; ++i) {
				if (orderedAdjacentNodes[n][i] == n_prev) {
					in_first_region = false;
					adjacent_to_n.push_back(orderedAdjacentNodes[n][i]);
					adjacent_to_n_copy.push_back(orderedAdjacentNodes[n][i]);
					printf("\t\t%d to both\n", orderedAdjacentNodes[n][i]);
				}
				else if (in_first_region) {
					adjacent_to_n_copy.push_back(orderedAdjacentNodes[n][i]);
					printf("\t\t%d to copy\n", orderedAdjacentNodes[n][i]);
				}
				else {
					adjacent_to_n.push_back(orderedAdjacentNodes[n][i]);
					printf("\t\t%d to n\n", orderedAdjacentNodes[n][i]);
				}
			}
		}
		orderedAdjacentNodes[n] = adjacent_to_n;
		orderedAdjacentNodes.push_back(adjacent_to_n_copy);

		printf("\tupdated orderedAdjacent\n");
		//replace n in triangles matrix
		for (int i = 0; i < triangles.rows(); ++i) {
			if (triangles(i,0) == n || triangles(i,1) == n || triangles(i,2) == n) {
				// find out which side of the cut the triangle belongs to
				int replacement = -1;
				for(int j = 0; j < 3; ++j){
					if (triangles(i, j) == n_prev || triangles(i, j) == n_next) continue;
					for (int k = 0; k < orderedAdjacentNodes[n].size(); ++k)
						if (triangles(i, j) == orderedAdjacentNodes[n][k])
							replacement = n;
					for (int k = 0; k < orderedAdjacentNodes[n_copy].size(); ++k)
						if (triangles(i, j) == orderedAdjacentNodes[n_copy][k])
							replacement = n_copy;
				}
				for (int j = 0; j < 3; ++j)
					if (triangles(i, j) == n)
						triangles(i, j) = replacement;
			}
		}
		printf("\tupdated triangles\n");
		// update adjacentElements (array in nodes as well as the elements themselves)
		DynamicArray<SimMeshElement*> adjacentElements = nodes[n]->adjacentElements;
		nodes[n]->adjacentElements.clear();
		for (int i = 0; i < adjacentElements.size(); ++i) {
			if (CSTriangle3D* e = dynamic_cast<CSTriangle3D*>(adjacentElements[i])) {
				// find out which side of the cut the triangle belongs to
				int replacement = n;
				for (int j = 0; j < 3; ++j) {
					if (e->n[j] == nodes[n_prev] || e->n[j] == nodes[n_next]) continue;
					for (int k = 0; k < orderedAdjacentNodes[n_copy].size(); ++k)
						if (e->n[j] == nodes[orderedAdjacentNodes[n_copy][k]])
							replacement = n_copy;
				}
				if (replacement != n) {
					for (int j = 0; j < 3; ++j)
						if (e->n[j] == nodes[n])
							e->n[j] = nodes[replacement];
				}
				nodes[replacement]->adjacentElements.push_back(e);
			}
			else if (BendingEdge* e = dynamic_cast<BendingEdge*>(adjacentElements[i])) {
				// find out which side of the cut the edge belongs to
				int replacement = n;
				for (int j = 0; j < 4; ++j) {
					if (e->n[j] == nodes[n_prev] || e->n[j] == nodes[n_next]) continue;
					for (int k = 0; k < orderedAdjacentNodes[n_copy].size(); ++k)
						if (e->n[j] == nodes[orderedAdjacentNodes[n_copy][k]])
							replacement = n_copy;
				}
				if (replacement != n) {
					for (int j = 0; j < 4; ++j)
						if (e->n[j] == nodes[n])
							e->n[j] = nodes[replacement];
				}
				nodes[replacement]->adjacentElements.push_back(e);
			}
		}
		printf("\tupdated adjacentElements\n");
		// update orderedAdjacentNodes[not_n]
		for (int i = 0; i < orderedAdjacentNodes[n_copy].size(); ++i) {
			int n_adj = orderedAdjacentNodes[n_copy][i];
			printf("\t\tupdating orderedAdj[%d]\n", n_adj);
			if (n_adj == n_next || (n_adj==n_prev && copy_index == -1)) continue;
			for (int j = 0; j < orderedAdjacentNodes[n_adj].size(); ++j)
				if (orderedAdjacentNodes[n_adj][j] == n)
					orderedAdjacentNodes[n_adj][j] = n_copy;
		}
		// n_next will do its own updates
		// if n_prev was duplicated, its copies can be treated like any other nodes
		//attention: if n_prev wasn't on the boundary was not duplicated, this has to be done
		if (n_prev != -1 && copy_index == -1 && !boundary[n_prev]) {
			int n_index = -1;
			for (int i = 0; i < orderedAdjacentNodes[n_prev].size(); ++i)
				if (orderedAdjacentNodes[n_prev][i] == n) {
					n_index = i;
					break;
				}
			DynamicArray<int> adjacent_to_n_prev;
			adjacent_to_n_prev.push_back(n_copy);
			for (int ii = 0; ii < orderedAdjacentNodes[n_prev].size(); ++ii) {
				int i = (n_index + ii + 1) % orderedAdjacentNodes[n_prev].size();
				adjacent_to_n_prev.push_back(orderedAdjacentNodes[n_prev][i]);
			}
			orderedAdjacentNodes[n_prev] = adjacent_to_n_prev;
			printf("\tn_prev:");
			for (int i = 0; i < adjacent_to_n_prev.size(); ++i)printf("\t%d", adjacent_to_n_prev[i]);
			printf("\n");
		}
		printf("\tupdated more orderedAdjacent\n");

		bool next_needs_copy = false;
		for (int i = 0; i < adjacent_to_n_copy.size(); ++i)
			if (adjacent_to_n_copy[i] == n_next)
				next_needs_copy = true;
		if (next_needs_copy) {
			copy_index = n_copy;
		}
		else {
			copy_index = -1;
		}
	}
	else if (num_copies == 2) {
		printf("\tmake two copies\n");
		// this happens only if n is on the boundary and n_prev and n_next aren't
		// make 2 copies of n
		//  the first connects to n_prev or its copy
		//  the second connects to n_next (or maybe later it copy)
		// connecting everything correctly might be a nightmare
		// but n being on the boundary makes some parts simpler
		/*
		connection before (a and c are at least 1 node, b is probably 0 nodes)
			a	-	n	-	c
			|	  / | \		|
			n_prev -b - n_next
		connections after adding n_copy_1 and n_copy_2
		a - n_copy_1			 n			n_copy_2 - c
		|    /				  /	 |	\			   \   |
		n_prev*			n_prev - b - n_next			n_next
		*/
		int index_prev = -1, index_next = -1;

		// create 2 new nodes
		int n_copy_1 = nodes.size();
		Node* newNode = new Node(this, n_copy_1, 3 * n_copy_1, 3);
		nodes.push_back(newNode);
		int n_copy_2 = nodes.size();
		newNode = new Node(this, n_copy_2, 3 * n_copy_2, 3);
		nodes.push_back(newNode);

		int nodeCount = n_copy_2 + 1;
		x.conservativeResize(3 * nodeCount);
		X.conservativeResize(3 * nodeCount);
		v.conservativeResize(3 * nodeCount);
		f_ext.conservativeResize(3 * nodeCount);
		m.conservativeResize(3 * nodeCount);
		for (int i = 0; i < 3; ++i) {
			x[3 * n_copy_1 + i] = x[3 * n + i];
			X[3 * n_copy_1 + i] = X[3 * n + i];
			v[3 * n_copy_1 + i] = v[3 * n + i];
			f_ext[3 * n_copy_1 + i] = f_ext[3 * n + i];
			m[3 * n_copy_1 + i] = m[3 * n + i];

			x[3 * n_copy_2 + i] = x[3 * n + i];
			X[3 * n_copy_2 + i] = X[3 * n + i];
			v[3 * n_copy_2 + i] = v[3 * n + i];
			f_ext[3 * n_copy_2 + i] = f_ext[3 * n + i];
			m[3 * n_copy_2 + i] = m[3 * n + i];
		}
		boundary.push_back(true);
		boundary.push_back(true);

		// update orderedAdjacentNodes
		DynamicArray<int> adjacent_to_n, adjacent_to_n_copy_1, adjacent_to_n_copy_2;
		for (int i = 0; i < num_adjacent; ++i) {
			int n_i = orderedAdjacentNodes[n][i];
			if (index_prev == -1 && n_i != n_prev) {
				adjacent_to_n_copy_1.push_back(n_i);
			}
			else if (n_i == n_prev) {
				if (copy_index == -1) {
					adjacent_to_n_copy_1.push_back(n_i);
				}
				else {
					adjacent_to_n_copy_1.push_back(copy_index);
				}
				index_prev = i;
				adjacent_to_n.push_back(n_i);
			}
			else if (index_next == -1 && n_i != n_next) {
				adjacent_to_n.push_back(n_i);
			}
			else if (n_i == n_next) {
				adjacent_to_n.push_back(n_i);
				index_next = i;
				adjacent_to_n_copy_2.push_back(n_i);
			}
			else {
				adjacent_to_n_copy_2.push_back(n_i);
			}
		}
		orderedAdjacentNodes[n] = adjacent_to_n;
		orderedAdjacentNodes.push_back(adjacent_to_n_copy_1);
		orderedAdjacentNodes.push_back(adjacent_to_n_copy_2);

		// update triangles matrix
		for (int i = 0; i < triangles.rows(); ++i) {
			if (triangles(i, 0) == n || triangles(i, 1) == n || triangles(i, 2) == n) {
				int replacement = n;
				for (int j = 0; j < 3; ++j) {
					if (triangles(i, j) == n_prev || triangles(i,j) == n_next) continue;
					for (int k = 0; k < orderedAdjacentNodes[n_copy_1].size(); ++k)
						if (triangles(i, j) == orderedAdjacentNodes[n_copy_1][k])
							replacement = n_copy_1;
					for (int k = 0; k < orderedAdjacentNodes[n_copy_2].size(); ++k)
						if (triangles(i, j) == orderedAdjacentNodes[n_copy_2][k])
							replacement = n_copy_2;
				}
				if (replacement != n)
					for (int j = 0; j < 3; ++j)
						if (triangles(i, j) == n)
							triangles(i, j) = replacement;
			}
		}

		// update adjacentElements
		DynamicArray<SimMeshElement*> adjacentElements = nodes[n]->adjacentElements;
		nodes[n]->adjacentElements.clear();
		for (int i = 0; i < adjacentElements.size(); ++i) {
			if (CSTriangle3D* e = dynamic_cast<CSTriangle3D*>(adjacentElements[i])) {
				int replacement = n;
				for (int j = 0; j < 3; ++j) {
					if (e->n[j] == nodes[n_prev] || e->n[j] == nodes[n_next]) continue;
					for (int k = 0; k < orderedAdjacentNodes[n_copy_1].size(); ++k)
						if (e->n[j] == nodes[orderedAdjacentNodes[n_copy_1][k]])
							replacement = n_copy_1;
					for (int k = 0; k < orderedAdjacentNodes[n_copy_2].size(); ++k)
						if (e->n[j] == nodes[orderedAdjacentNodes[n_copy_2][k]])
							replacement = n_copy_2;
				}
				if (replacement != n) {
					for (int j = 0; j < 3; ++j)
						if (e->n[j] == nodes[n])
							e->n[j] = nodes[replacement];
				}
				nodes[replacement]->adjacentElements.push_back(e);
			}
			else if (BendingEdge* e = dynamic_cast<BendingEdge*>(adjacentElements[i])) {
				// find out which side of the cut the edge belongs to
				int replacement = n;
				for (int j = 0; j < 4; ++j) {
					if (e->n[j] == nodes[n_prev] || e->n[j] == nodes[n_next]) continue;
					for (int k = 0; k < orderedAdjacentNodes[n_copy_1].size(); ++k)
						if (e->n[j] == nodes[orderedAdjacentNodes[n_copy_1][k]])
							replacement = n_copy_1;
					for (int k = 0; k < orderedAdjacentNodes[n_copy_2].size(); ++k)
						if (e->n[j] == nodes[orderedAdjacentNodes[n_copy_2][k]])
							replacement = n_copy_2;
				}
				if (replacement != n) {
					for (int j = 0; j < 4; ++j)
						if (e->n[j] == nodes[n])
							e->n[j] = nodes[replacement];
				}
				nodes[replacement]->adjacentElements.push_back(e);
			}
		}

		//TODO update orderedAdjacent not n
		// update orderedAdjacentNodes[not_n]
		for (int i = 0; i < orderedAdjacentNodes[n_copy_1].size(); ++i) {
			int n_adj = orderedAdjacentNodes[n_copy_1][i];
			if (n_adj == n_prev && copy_index == -1) continue;
			for (int j = 0; j < orderedAdjacentNodes[n_adj].size(); ++j)
				if (orderedAdjacentNodes[n_adj][j] == n)
					orderedAdjacentNodes[n_adj][j] = n_copy_1;
		}
		for (int i = 0; i < orderedAdjacentNodes[n_copy_2].size(); ++i) {
			int n_adj = orderedAdjacentNodes[n_copy_2][i];
			if (n_adj == n_next) continue;
			for (int j = 0; j < orderedAdjacentNodes[n_adj].size(); ++j)
				if (orderedAdjacentNodes[n_adj][j] == n)
					orderedAdjacentNodes[n_adj][j] = n_copy_2;
		}
		if (copy_index == -1) {
			//if n_prev was not copied (i.e. it is the starting point of the cut), there needs to be a new order
			int start_i;
			for(int i=0;i<orderedAdjacentNodes[n_prev].size();++i)
				if (orderedAdjacentNodes[n_prev][i] == n) {
					start_i = i;
					break;
				}
			DynamicArray<int> adjacent_to_n_prev;
			adjacent_to_n_prev.push_back(n_copy_1);
			for (int ii = 0; ii < orderedAdjacentNodes[n_prev].size(); ++ii) {
				int i = (start_i + i + 1) % orderedAdjacentNodes[n_prev].size();
				adjacent_to_n_prev.push_back(orderedAdjacentNodes[n_prev][i]);
			}
			orderedAdjacentNodes[n_prev] = adjacent_to_n_prev;
		}
		copy_index = n_copy_2;
	}
}