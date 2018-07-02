#include "KineSimLib/KS_MultiLinkBar.h"
#include "KineSimLib/KS_LoaderUtils.h"

#define LOCAL_COORDINATES_PLANE_NORMAL (getLocalCordinatesPlaneNormal())

KS_MultiLinkBar::KS_MultiLinkBar(char* rName) : KS_MechanicalComponent(rName) {
	width = 0.2;
	thickness = 0.05;
}

KS_MultiLinkBar* KS_MultiLinkBar::clone() const{
	KS_MultiLinkBar* comp = new KS_MultiLinkBar(*this);
	for(uint i=0;i<meshes.size();i++)
		comp->meshes[i]=this->meshes[i]->clone();
	for(uint i=0;i<this->tracerParticles.size();i++)
		comp->tracerParticles[i].mc=dynamic_cast<KS_MechanicalComponent*>(comp);
	return comp;
}

KS_MultiLinkBar::~KS_MultiLinkBar(void){

}

Point3d KS_MultiLinkBar::sample(double t, Vector3d& derivative) {
	int nPoints = (int)midLinePoints.size();
	int currentStartPoint = (int) (t * (nPoints-1));
	int currentEndPoint = currentStartPoint + 1;
	if (currentStartPoint >= nPoints) {
		currentStartPoint = nPoints - 1;
	}
	if (currentEndPoint >= nPoints) {
		currentEndPoint = nPoints - 1;
	}
	double local_t_width = 1.0 / (nPoints-1);
	double local_t = (t - currentStartPoint * local_t_width) / local_t_width;
	derivative = Vector3d(midLinePoints[currentEndPoint] - midLinePoints[currentStartPoint]);
	return midLinePoints[currentStartPoint] + derivative * local_t;
}

bool KS_MultiLinkBar::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_MULTI_LINK_BAR);
	fprintf(f, "%s\n", str);

	writeBaseComponentToFile(f);

	str = getKSString(KS_THICKNESS);
	fprintf(f, "\t%s %lf\n", str, thickness);

	str = getKSString(KS_WIDTH);
	fprintf(f, "\t%s %lf\n", str, width);

	str = getKSString(KS_BAR_END_POINTS);
	for (uint i=0; i<midLinePoints.size();i++)
		fprintf(f, "\t%s %lf %lf\n", str, midLinePoints[i].x, midLinePoints[i].y);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}


bool KS_MultiLinkBar::loadFromFile(FILE* f){
	if (f == NULL){
		logPrint("KS_MultiLinkBar: Cannot load input file.\n");
		return false;
	}
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	//this is where it happens.
	while (!feof(f)){
		//get a line from the file...
		readValidLine(buffer, f, sizeof(buffer));
		char *line = lTrim(buffer);

		if (KS_MechanicalComponent::processInputLine(line))
			continue;

		int lineType = getKSLineType(line);
		switch (lineType) {
			case KS_THICKNESS:
				if(sscanf(line, "%lf", &thickness) != 1) assert(false);
				break;
			case KS_END:
				setupGeometry();
				return true;
				break;
			case KS_WIDTH:
				if(sscanf(line, "%lf", &width) != 1) assert(false);
				break;
			case KS_BAR_END_POINTS:{
				Point3d p;
				sscanf(line, "%lf %lf\n", &p.x, &p.y);
				midLinePoints.push_back(p);
				}break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_MultiLinkBar: Warning - end of file met before END primitive\n");
	return false;
}



void KS_MultiLinkBar::setupGeometry(){
	if (midLinePoints.size() < 2) return;
	GLMesh* tmpMesh = new GLMesh();
	//the start, middle and end points define the KS_MultiLinkBar's center line, so extrude these about the normal to get the points that define the contour of the KS_MultiLinkBar


	double length = 0;

	for (uint i=0;i<midLinePoints.size();i++){
		if(i>0) length += Vector3d(midLinePoints[i], midLinePoints[i-1]).length();
		//we need to estimate the normal at this point by looking at consecutive points along the mid line
		Vector3d normal;
		if (i > 0) normal += Vector3d(midLinePoints[i-1], midLinePoints[i]).unit();
		if (i < midLinePoints.size()-1) normal += Vector3d(midLinePoints[i], midLinePoints[i+1]).unit();
		normal.z = 0; double tmp = normal.x; normal.x = -normal.y; normal.y = tmp;
		normal.toUnit();

		Point3d p = midLinePoints[i] + normal * width/2.0;
		p.z = -thickness/2;tmpMesh->addVertex(p);
		p.z =  thickness/2; tmpMesh->addVertex(p);

		p = midLinePoints[i] + normal * -width/2.0;
		p.z = -thickness/2; tmpMesh->addVertex(p);
		p.z =  thickness/2; tmpMesh->addVertex(p);
	}

//	Logger::printStatic("MultiLinkBar %s: total length: %lf\n", this->m_name, length);
//	if (midLinePoints.size() == 3){
//		Logger::printStatic("MultiLinkBar %s: l1 %lf, l2: %lf, angle betwen: %lf\n", this->m_name, Vector3d(midLinePoints[1], midLinePoints[0]).length(), Vector3d(midLinePoints[1], midLinePoints[2]).length(), DEG(Vector3d(midLinePoints[1], midLinePoints[0]).angleWith(Vector3d(midLinePoints[1], midLinePoints[2]))));
//	}
//	Logger::printStatic("MultiLinkBar %s: total length: %lf\n", this->m_name, Vector3d(midLinePoints[midLinePoints.size()-1], midLinePoints[0]).length());

	int n = (int)midLinePoints.size();
	//get the end points
	tmpMesh->addPoly(GLIndexedQuad(0, 1, 3, 2, true));
	tmpMesh->addPoly(GLIndexedQuad(4*n-4, 4*n-2, 4*n-1, 4*n-3, true));

	//now connect the surfaces...
	for (int i=0;i<n-1;i++){
		tmpMesh->addPoly(GLIndexedQuad(4*i+0, 4*i+2, 4*i+6, 4*i+4, true));
		tmpMesh->addPoly(GLIndexedQuad(4*i+0, 4*i+4, 4*i+5, 4*i+1, true));
		tmpMesh->addPoly(GLIndexedQuad(4*i+1, 4*i+5, 4*i+7, 4*i+3, true));
		tmpMesh->addPoly(GLIndexedQuad(4*i+2, 4*i+3, 4*i+7, 4*i+6, true));
	}


	tmpMesh->setColour(meshColor[0], meshColor[1], meshColor[2], 1);
	tmpMesh->computeNormals();
	meshes.push_back(tmpMesh);
}

void KS_MultiLinkBar::setMidLinePoints(const DynamicArray<Point3d> &pts){
	midLinePoints.clear();
	midLinePoints = pts;
	for(uint i=0;i<meshes.size();i++) delete meshes[i];
	meshes.clear();
	setupGeometry();
}

void KS_MultiLinkBar::getMidLinePoints(DynamicArray<Point3d> &pts) {
	pts.assign(midLinePoints.begin(), midLinePoints.end());
}

