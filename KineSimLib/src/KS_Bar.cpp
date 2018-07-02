#include "KineSimlib/KS_Bar.h"
#include "KineSimlib/KS_LoaderUtils.h"

#define LOCAL_COORDINATES_PLANE_NORMAL (getAlphaAxis())

KS_Bar::KS_Bar(char* rName) : KS_MechanicalComponent(rName) {
	width = 0.2;
	thickness = 0.05;
	start = V3D(-1,0,0);
	//the middle point will always be at the origin
	middle = V3D(0,0,0);
	end = V3D(1,0,0);
}

KS_Bar* KS_Bar::clone() const{
	KS_Bar* comp = new KS_Bar(*this);
	for(uint i=0;i<meshes.size();i++)
		comp->meshes[i]=this->meshes[i]->clone();
	for(uint i=0;i<this->tracerParticles.size();i++)
		comp->tracerParticles[i].mc=dynamic_cast<KS_MechanicalComponent*>(comp);
	return comp;
}

KS_Bar::~KS_Bar(void){

}

bool KS_Bar::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_BAR);
	fprintf(f, "%s\n", str);

	writeBaseComponentToFile(f);

	str = getKSString(KS_THICKNESS);
	fprintf(f, "\t%s %lf\n", str, thickness);

	str = getKSString(KS_WIDTH);
	fprintf(f, "\t%s %lf\n", str, width);

	str = getKSString(KS_BAR_END_POINTS);
	fprintf(f, "\t%s %lf %lf %lf %lf\n", str, start[0], start[1], end[0], end[1]);

	for (uint i=0; i<hollowIntervals.size()/2;i++){
		str = getKSString(KS_HOLLOW_INTERVAL);
		fprintf(f, "\t%s %lf %lf\n", str, hollowIntervals[2*i], hollowIntervals[2*i+1]);
	}

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}


bool KS_Bar::loadFromFile(FILE* f){
	if (f == NULL){
		//logPrint("KS_Bar: Cannot load input file.\n");
		Logger::print("KS_Bar: Cannot load input file.\n");
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
			case KS_BAR_END_POINTS:
				if(sscanf(line, "%lf %lf %lf %lf", &start.x,&start.y, &end.x,&end.y) != 4) assert(false);
				break;
			case KS_HOLLOW_INTERVAL:{
				double start, end;
				if(sscanf(line, "%lf %lf", &start, &end) != 2) assert(false);
				assert(start > 0);
				assert(end < 1);
				if (hollowIntervals.size() > 0)
					assert(hollowIntervals[hollowIntervals.size()-1] < start);
				hollowIntervals.push_back(start);
				hollowIntervals.push_back(end);
				}break;
			default:
				//logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				Logger::print("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	//logPrint("KS_Bar: Warning - end of file met before END primitive\n");
	Logger::print("KS_Bar: Warning - end of file met before END primitive\n");
	return false;
}


void getWeights(double t, double &w1, double &w2, double &w3){
	if (t < 0.5){
		w1 = 1-mapTo01Range(t, 0, 0.5);
		w2 = 1 - w1;
		w3 = 0;
	}else{
		w1 = 0;
		w2 = 1-mapTo01Range(t, 0.5, 1.0);
		w3 = 1-w2;
	}
}

void KS_Bar::addBarVertices(GLMesh* tmpMesh, double progress, const V3D& startUp, const V3D& middleUp, const V3D& endUp, const V3D& startDown, const V3D& middleDown, const V3D& endDown, bool createQuadLower, bool createQuadMiddle, bool createQuadUpper, bool flipNormals){
	double w1, w2, w3;
	getWeights(progress, w1, w2, w3);

	int startIndex = (int)tmpMesh->getVertexCount();

	V3D s = startDown * w1 + middleDown * w2 + endDown * w3;
	V3D e = startUp * w1 + middleUp * w2 + endUp * w3;
	//start by creating the geometry for the start face...
	for (int i=0; i< 4;i++){
		tmpMesh->addVertex(P3D() + (s - LOCAL_COORDINATES_PLANE_NORMAL * thickness/2.0 + V3D(e-s) * i * 0.33));
		tmpMesh->addVertex(P3D() + (s + LOCAL_COORDINATES_PLANE_NORMAL * thickness/2.0 + V3D(e-s) * i * 0.33));
	}

	if (createQuadLower)
		tmpMesh->addPoly(GLIndexedQuad(startIndex+0, startIndex+1, startIndex+3, startIndex+2, flipNormals));
	if (createQuadMiddle)
		tmpMesh->addPoly(GLIndexedQuad(startIndex+2, startIndex+3, startIndex+5, startIndex+4, flipNormals));
	if (createQuadUpper)
		tmpMesh->addPoly(GLIndexedQuad(startIndex+4, startIndex+5, startIndex+7, startIndex+6, flipNormals));

}

void KS_Bar::setupGeometry(){
	GLMesh* tmpMesh = new GLMesh();
	//the start, middle and end points define the KS_Bar's center line, so extrude these about the normal to get the points that define the contour of the KS_Bar
	V3D v1(middle-start); v1.normalize();
	V3D v2(end-middle); v2.normalize();
	V3D startNormal(-v1.y, v1.x, 0); startNormal.normalize();
	V3D middleNormal(-(v1.y+v2.y), (v1.x+v2.x), 0); middleNormal.normalize();
	V3D endNormal(-v2.y, v2.x, 0); endNormal.normalize();

	V3D startUp = start + startNormal * width/2;
	V3D startDown = start - startNormal * width/2;

	V3D middleUp = middle + middleNormal * width/2;
	V3D middleDown = middle - middleNormal * width/2;

	V3D endUp = end + endNormal * width/2;
	V3D endDown = end - endNormal * width/2;

	DynamicArray<double> intervalPoints;
	DynamicArray<bool> hollow;

	intervalPoints.push_back(0);

	for (uint i=0; i<hollowIntervals.size() / 2;i++){
		intervalPoints.push_back(hollowIntervals[2 * i + 0]);
		hollow.push_back(false);
		intervalPoints.push_back(hollowIntervals[2 * i + 1]);
		hollow.push_back(true);
	}
	intervalPoints.push_back(1);
	hollow.push_back(false);

	//now we want to make sure that the mid point of the KS_Bar is also in, since it's important for non-straight Bars
	for (uint i=0;i<intervalPoints.size();i++){
		if (intervalPoints[i] == 0.5) break;
		if (intervalPoints[i] > 0.5){
			bool intervalIsHollow = hollow[i-1];
			intervalPoints.insert(intervalPoints.begin() + i, 0.5);
			hollow.insert(hollow.begin() + i, intervalIsHollow);
			break;
		}
	}

	//now add all the correct vertices for the KS_Bar
	for (uint i=0;i<intervalPoints.size();i++){
		if (i == 0 || i == intervalPoints.size()-1)
			addBarVertices(tmpMesh, intervalPoints[i], startUp, middleUp, endUp, startDown, middleDown, endDown, true, true, true, i != 0);
		else if (hollow[i] != hollow[i-1])
			addBarVertices(tmpMesh, intervalPoints[i], startUp, middleUp, endUp, startDown, middleDown, endDown, false, true, false, hollow[i]);
		else
			addBarVertices(tmpMesh, intervalPoints[i], startUp, middleUp, endUp, startDown, middleDown, endDown, false, false, false);
	}

	//now connect the surfaces...
	int numberOfFaces = (int)tmpMesh->getVertexCount() / 8;
	for (int i=0;i<numberOfFaces-1;i++){
		tmpMesh->addPoly(GLIndexedQuad(i*8 + 7, i*8 + 6, (i+1)*8 + 6, (i+1)*8 + 7, true));
		tmpMesh->addPoly(GLIndexedQuad(i*8 + 1, i*8 + 0, (i+1)*8 + 0, (i+1)*8 + 1));

		tmpMesh->addPoly(GLIndexedQuad(i*8 + 5, i*8 + 7, (i+1)*8 + 7, (i+1)*8 + 5, true));
		tmpMesh->addPoly(GLIndexedQuad(i*8 + 4, i*8 + 6, (i+1)*8 + 6, (i+1)*8 + 4));

		tmpMesh->addPoly(GLIndexedQuad(i*8 + 1, i*8 + 3, (i+1)*8 + 3, (i+1)*8 + 1, true));
		tmpMesh->addPoly(GLIndexedQuad(i*8 + 0, i*8 + 2, (i+1)*8 + 2, (i+1)*8 + 0));
		if (hollow[i] == false){
			tmpMesh->addPoly(GLIndexedQuad(i*8 + 5, i*8 + 3, (i+1)*8 + 3, (i+1)*8 + 5));
			tmpMesh->addPoly(GLIndexedQuad(i*8 + 2, i*8 + 4, (i+1)*8 + 4, (i+1)*8 + 2));
		}else{
			tmpMesh->addPoly(GLIndexedQuad(i*8 + 4, i*8 + 5, (i+1)*8 + 5, (i+1)*8 + 4, true));
			tmpMesh->addPoly(GLIndexedQuad(i*8 + 2, i*8 + 3, (i+1)*8 + 3, (i+1)*8 + 2));
		}
	}

	tmpMesh->setColour(meshColor[0], meshColor[1], meshColor[2], 1);
	tmpMesh->computeNormals();
	meshes.push_back(tmpMesh);
}

void KS_Bar::setDimensions( V3D s, V3D m, V3D e )
{
	start = s;

	middle = m;

	end = e;
}