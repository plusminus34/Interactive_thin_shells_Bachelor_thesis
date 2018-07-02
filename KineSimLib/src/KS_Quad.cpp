#include "KineSimLib/KS_Quad.h"
#include "KineSimLib/KS_LoaderUtils.h"

#define LOCAL_COORDINATES_PLANE_NORMAL (getLocalCordinatesPlaneNormal())

KS_Quad::KS_Quad(char* rName) : KS_MechanicalComponent(rName) {
	thickness = 0.05;
}

KS_Quad* KS_Quad::clone() const {
	KS_Quad* comp = new KS_Quad(*this);
	for(uint i=0;i<meshes.size();i++)
		comp->meshes[i]=this->meshes[i]->clone();
	for(uint i=0;i<this->tracerParticles.size();i++)
		comp->tracerParticles[i].mc=dynamic_cast<KS_MechanicalComponent*>(comp);
	return comp;
}

KS_Quad::~KS_Quad(void){

}

bool KS_Quad::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_QUAD);
	fprintf(f, "%s\n", str);

	writeBaseComponentToFile(f);

	str = getKSString(KS_THICKNESS);
	fprintf(f, "\t%s %lf\n", str, thickness);

	str = getKSString(KS_QUAD_POINTS);
	fprintf(f, "\t%s %lf %lf %lf %lf %lf %lf %lf %lf\n", str, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}


bool KS_Quad::loadFromFile(FILE* f){
	if (f == NULL){
		logPrint("KS_Quad: Cannot load input file.\n");
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
			case KS_QUAD_POINTS:
				if(sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p2.x, &p2.y, &p3.x, &p3.y, &p4.x, &p4.y) != 8) assert(false);
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_Quad: Warning - end of file met before END primitive\n");
	return false;
}

void KS_Quad::setupGeometry(){
	GLMesh* tmpMesh = new GLMesh();

	p1.z = p2.z = p3.z = p4.z = -thickness/2;
	tmpMesh->addVertex(p1);
	tmpMesh->addVertex(p2);
	tmpMesh->addVertex(p3);
	tmpMesh->addVertex(p4);

//	Logger::printStatic("g: %lf\n",Vector3d(p1, p2).length());
//	Logger::printStatic("i: %lf\n",Vector3d(p2, p3).length());
//	Logger::printStatic("h: %lf\n",Vector3d(p3, p1).length());

	p1.z = p2.z = p3.z = p4.z = thickness/2;
	tmpMesh->addVertex(p1);
	tmpMesh->addVertex(p2);
	tmpMesh->addVertex(p3);
	tmpMesh->addVertex(p4);

	tmpMesh->addPoly(GLIndexedQuad(0, 1, 2, 3,true));
	tmpMesh->addPoly(GLIndexedQuad(4, 7, 6, 5,true));
	tmpMesh->addPoly(GLIndexedQuad(0, 3, 7, 4,true));
	tmpMesh->addPoly(GLIndexedQuad(3, 2, 6, 7,true));
	tmpMesh->addPoly(GLIndexedQuad(2, 1, 5, 6,true));
	tmpMesh->addPoly(GLIndexedQuad(1, 0, 4, 5,true));




	tmpMesh->setColour(meshColor[0], meshColor[1], meshColor[2], 1);
	tmpMesh->computeNormals();
	meshes.push_back(tmpMesh);
}

