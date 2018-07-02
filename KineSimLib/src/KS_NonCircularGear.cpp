#include "KineSimLib/KS_NonCircularGear.h"
#include "KineSimLib/KS_LoaderUtils.h"

#define LOCAL_COORDINATES_GEAR_AXIS (getAlphaAxis())

KS_NonCircularGear::KS_NonCircularGear(char* gearName) : KS_MechanicalComponent(gearName){
	setKS_NonCircularGearParameters(0.1, 20, RAD(20), 0.07);
}

KS_NonCircularGear* KS_NonCircularGear::clone() const {
	KS_NonCircularGear* comp = new KS_NonCircularGear(*this);
	for(uint i=0;i<meshes.size();i++)
		comp->meshes[i]=this->meshes[i]->clone();
	for(uint i=0;i<this->tracerParticles.size();i++)
		comp->tracerParticles[i].mc=dynamic_cast<KS_MechanicalComponent*>(comp);
	return comp;
}

KS_NonCircularGear::~KS_NonCircularGear(void){
}

void KS_NonCircularGear::setKS_NonCircularGearParameters(double pthickness, int pNumberOfTeeth, double pPressureAngle, double pTeethHeight){
	this->thickness = pthickness;
	this->numberOfTeeth = pNumberOfTeeth;
	this->pressureAngle = pPressureAngle;
	this->teethHeight = pTeethHeight;
}

void KS_NonCircularGear::createGearGeometry(Trajectory3D* gearProfile, double gearCircumference, double teethOffset, bool clockwise){
/*
	char fName[200];
	sprintf(fName, "%s.gear", this->m_name);
	FILE* fp = fopen(fName,"w");

	fprintf(fp, "%lf %lf %lf\n%d\n", posWorld.x, posWorld.y, posWorld.z, gearProfile->getKnotCount());

	for (int i=0;i<gearProfile->getKnotCount();i++)
		fprintf(fp, "%lf %lf %lf %lf\n", gearProfile->getKnotPosition(i), gearProfile->getKnotValue(i).x, gearProfile->getKnotValue(i).y,  gearProfile->getKnotValue(i).z);

	fclose(fp);
*/
	int numberOfTeeth = getNumTeeth();
	double teethThickness = gearCircumference / (2 * numberOfTeeth) * 0.8;

	GLMesh* tmpMesh = new GLMesh();

	tmpMesh->addVertex(Point3d(0,0, -thickness/2.0));
	tmpMesh->addVertex(Point3d(0,0, +thickness/2.0));

	for (int i=0; i < numberOfTeeth; i++){
		double t = ((double)(i+teethOffset) / (numberOfTeeth));
//		t = 1-t;
		//t is a percentage that tells us how far along the circumference of the gear (length wise) we are supposed to be - figure out where along the trajectory we should be...
		double desiredLen = t * gearCircumference;

		int j=0;
		double totalCurrentLen = 0;
		for (j=0;j<gearProfile->getKnotCount()-1;j++){
			totalCurrentLen += (gearProfile->getKnotValue(j) - gearProfile->getKnotValue(j+1)).length();
			if (totalCurrentLen > desiredLen) break;
		}

		t = (double)j / (gearProfile->getKnotCount()-1);

		Vector3d p = gearProfile->evaluate_linear(t);
		double h = 0.001;
		Vector3d pPh = gearProfile->evaluate_linear(t+h);
		Vector3d pMh = gearProfile->evaluate_linear(t-h);
		Vector3d tangent = (pPh - pMh) / (2*h);
	
		if (t < h){
			pPh = gearProfile->evaluate_linear(t+h);
			pMh = gearProfile->evaluate_linear(t);
			tangent = (pPh - pMh) / (h);
		}
		if (t > 1-h){
			pPh = gearProfile->evaluate_linear(t);
			pMh = gearProfile->evaluate_linear(t-h);
			tangent = (pPh - pMh) / (h);			
		}

		//if (clockwise == true) 
		tangent *= -1;

		tangent.normalize();
		Vector3d normal(-tangent.y, tangent.x, 0);

		if (clockwise == false) normal *= -1;

		double offsetP = 1.1;
		double offsetM = 0.8;

		Vector3d n = rotate(normal, pressureAngle * ((!clockwise)?(1):(-1)), Vector3d(0,0,1));

		tmpMesh->addVertex(Point3d() + Vector3d(p.x - n.x*teethHeight/2.0 * offsetP - tangent.x * teethThickness/2, p.y - n.y*teethHeight/2.0 * offsetP - tangent.y * teethThickness/2, -thickness/2.0));
		tmpMesh->addVertex(Point3d() + Vector3d(p.x - n.x*teethHeight/2.0 * offsetP - tangent.x * teethThickness/2, p.y - n.y*teethHeight/2.0 * offsetP - tangent.y * teethThickness/2, +thickness/2.0));

		tmpMesh->addVertex(Point3d() + Vector3d(p.x + n.x*teethHeight/2.0 * offsetM - tangent.x * teethThickness/2, p.y + n.y*teethHeight/2.0 * offsetM - tangent.y * teethThickness/2, -thickness/2.0));
		tmpMesh->addVertex(Point3d() + Vector3d(p.x + n.x*teethHeight/2.0 * offsetM - tangent.x * teethThickness/2, p.y + n.y*teethHeight/2.0 * offsetM - tangent.y * teethThickness/2, +thickness/2.0));

		n = rotate(normal, -pressureAngle * ((!clockwise)?(1):(-1)), Vector3d(0,0,1));
		tmpMesh->addVertex(Point3d() + Vector3d(p.x + n.x*teethHeight/2.0 * offsetM + tangent.x * teethThickness/2, p.y + n.y*teethHeight/2.0 * offsetM + tangent.y * teethThickness/2, -thickness/2.0));
		tmpMesh->addVertex(Point3d() + Vector3d(p.x + n.x*teethHeight/2.0 * offsetM + tangent.x * teethThickness/2, p.y + n.y*teethHeight/2.0 * offsetM + tangent.y * teethThickness/2, +thickness/2.0));

		tmpMesh->addVertex(Point3d() + Vector3d(p.x - n.x*teethHeight/2.0 * offsetP + tangent.x * teethThickness/2, p.y - n.y*teethHeight/2.0 * offsetP + tangent.y * teethThickness/2, -thickness/2.0));
		tmpMesh->addVertex(Point3d() + Vector3d(p.x - n.x*teethHeight/2.0 * offsetP + tangent.x * teethThickness/2, p.y - n.y*teethHeight/2.0 * offsetP + tangent.y * teethThickness/2, +thickness/2.0));

	}

	for (int i=0; i < numberOfTeeth; i++){
		tmpMesh->addPoly(GLIndexedTriangle(0, 2 + i*8 + 0, 2 + i*8+6, !clockwise));
		tmpMesh->addPoly(GLIndexedTriangle(1, 2 + i*8 + 1, 2 + i*8+7, clockwise));

		tmpMesh->addPoly(GLIndexedQuad(2 + i*8 + 0, 2 + i*8 + 2, 2 + i*8 + 4, 2 + i*8 + 6, !clockwise));
		tmpMesh->addPoly(GLIndexedQuad(2 + i*8 + 1, 2 + i*8 + 3, 2 + i*8 + 5, 2 + i*8 + 7, clockwise));

		tmpMesh->addPoly(GLIndexedQuad(2 + i*8 + 0, 2 + i*8 + 1, 2 + i*8 + 3, 2 + i*8 + 2, !clockwise));
		tmpMesh->addPoly(GLIndexedQuad(2 + i*8 + 2, 2 + i*8 + 3, 2 + i*8 + 5, 2 + i*8 + 4, !clockwise));
		tmpMesh->addPoly(GLIndexedQuad(2 + i*8 + 4, 2 + i*8 + 5, 2 + i*8 + 7, 2 + i*8 + 6, !clockwise));

		int indexAfter = (i-1);
		if (indexAfter == -1) indexAfter = numberOfTeeth-1;

		tmpMesh->addPoly(GLIndexedTriangle(0, 2 + i*8 + 6, 2 + indexAfter*8+0, !clockwise));
		tmpMesh->addPoly(GLIndexedTriangle(1, 2 + i*8 + 7, 2 + indexAfter*8+1, clockwise));

		tmpMesh->addPoly(GLIndexedQuad(2 + i*8 + 6, 2 + i*8 + 7, 2 + indexAfter*8 + 1, 2 + indexAfter*8 + 0, !clockwise));
	}

	tmpMesh->computeNormals();
	tmpMesh->setColour(meshColor[0], meshColor[1], meshColor[2], 1);
	meshes.push_back(tmpMesh);
}

bool KS_NonCircularGear::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_NON_CIRCULAR_GEAR);
	fprintf(f, "%s\n", str);

	writeBaseComponentToFile(f);

	str = getKSString(KS_THICKNESS);
	fprintf(f, "\t%s %lf\n", str, thickness);

	str = getKSString(KS_NUMBER_OF_TEETH);
	fprintf(f, "\t%s %d\n", str, numberOfTeeth);

	str = getKSString(KS_TEETH_HEIGHT);
	fprintf(f, "\t%s %lf\n", str, teethHeight);

	str = getKSString(KS_PRESSURE_ANGLE);
	fprintf(f, "\t%s %lf\n", str, pressureAngle);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}


bool KS_NonCircularGear::loadFromFile(FILE* f){
	if (f == NULL){
		logPrint("Gear: Cannot load input file.\n");
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
			case KS_NUMBER_OF_TEETH:
				if(sscanf(line, "%d", &numberOfTeeth) != 1) assert(false);
				break;
			case KS_TEETH_HEIGHT:
				if(sscanf(line, "%lf", &teethHeight) != 1) assert(false);
				break;
			case KS_PRESSURE_ANGLE:
				if(sscanf(line, "%lf", &pressureAngle) != 1) assert(false);
				break;
			case KS_NOT_IMPORTANT:
				logPrint("Gear warning: Ignoring input line: \'%s\'\n", line);
				break;
			case KS_COMMENT:
				break;
			case KS_END:
				setupGeometry();
				return true;
				break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("Gear: Warning - end of file met before END primitive\n");
	return false;
}
