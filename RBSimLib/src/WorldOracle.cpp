#include <RBSimLib/WorldOracle.h>
#include <MathLib/Segment.h>


WorldOracle::WorldOracle(const V3D& worldUp, const Plane& groundPlane){
	this->worldUp = worldUp;
	this->groundPlane = groundPlane;
}

WorldOracle::~WorldOracle(void){
}

double WorldOracle::getWorldHeightAt(const P3D& worldLoc){
	// Throw a ray from the given world location downward and intersect it with all world objects.
	P3D p;
	Ray ray;
	Segment seg;

	ray.origin = worldLoc; ray.origin.setComponentAlong(worldUp, 1000.0); ray.direction = -worldUp;

	ray.getDistanceToPlane(groundPlane, &p);
	double highest = p.getComponentAlong(worldUp);

	for (uint i=0; i<boxes.size(); i++)	{
		if (!boxes[i].canBeSteppedOn) continue;

		if (boxes[i].intersectWithRay(ray, &seg)) {
			double lowerHeight = seg.b.getComponentAlong(worldUp);
			double upperHeight = seg.a.getComponentAlong(worldUp);
			double boxHeight = MAX(lowerHeight, upperHeight);
			highest = MAX(highest, boxHeight);
		}
	}

	return highest;
}


void WorldOracle::writeRBSFile(const char* fName){
	FILE* fp = fopen(fName, "w");
	//we'll create an RB with the correct CDP, mesh and position
	for (uint i = 0;i<boxes.size();i++) {
		fprintf(fp, "RigidBody\n");
		fprintf(fp, "\tfrozen\n");
		fprintf(fp, "\tname worldBox%d\n", i);
		V3D n = boxes.at(i).orientation.v.unit();
		fprintf(fp, "\torientation %lf %lf %lf %lf\n", boxes.at(i).orientation.getRotationAngle(n), n[0], n[1], n[2]);
		fprintf(fp, "\tposition %lf %lf %lf\n", boxes.at(i).position[0], boxes.at(i).position[1], boxes.at(i).position[2]);
		fprintf(fp, "\tCDP Box %2.3lf %2.3lf %2.3lf %2.3lf %2.3lf %2.3lf\n", boxes.at(i).c1[0], boxes.at(i).c1[1], boxes.at(i).c1[2], boxes.at(i).c2[0], boxes.at(i).c2[1], boxes.at(i).c2[2]);
		fprintf(fp, "/End\n\n\n");
	}

	fprintf(fp, "\n\nRigidBody\n");
	fprintf(fp, "\tname ground\n");
	fprintf(fp, "\tfrozen\n");
	fprintf(fp, "\tCDP Plane %lf %lf %lf %lf %lf %lf\n", groundPlane.n[0], groundPlane.n[1], groundPlane.n[2], groundPlane.p[0], groundPlane.p[1], groundPlane.p[2]);
//	fprintf(fp, "\tCDP Box -2 -1 -2 2 0 2\n");
	fprintf(fp, "\tfrictionCoefficient 2.5\n");
	fprintf(fp, "\trestitutionCoefficient 0.35\n");
	fprintf(fp, "/End\n");

	fclose(fp);
}
