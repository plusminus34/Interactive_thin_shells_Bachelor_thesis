#include "KineSimLib/KS_HermiteSplineLinkBar.h"
#include "KineSimLib/KS_LoaderUtils.h"

KS_HermiteSplineLinkBar::KS_HermiteSplineLinkBar(char* rName) : KS_MultiLinkBar(rName){
	nSamples = 0;
}

KS_HermiteSplineLinkBar::~KS_HermiteSplineLinkBar(void){
}

bool KS_HermiteSplineLinkBar::writeToFile(FILE* f){
	char* str;

	str = getKSString(KS_MULTI_LINK_BAR);
	fprintf(f, "%s\n", str);

	writeBaseComponentToFile(f);

	str = getKSString(KS_THICKNESS);
	fprintf(f, "\t%s %lf\n", str, thickness);

	str = getKSString(KS_WIDTH);
	fprintf(f, "\t%s %lf\n", str, width);

	str = getKSString(KS_BAR_END_POINTS);
	fprintf(f, "\t%s %lf %lf\n", str, midLinePoints.front().x, midLinePoints.front().y);
	fprintf(f, "\t%s %lf %lf\n", str, midLinePoints.back().x, midLinePoints.back().y);

	str = getKSString(KS_HERMITE_SPLINE_ANGLES);
	for (uint i=0; i<angles.size();i++)
		fprintf(f, "\t%s %lf\n", str, angles[i]);

	str = getKSString(KS_HERMITE_SPLINE_MAGNITUDES);
	for (uint i=0; i<magnitudes.size();i++)
		fprintf(f, "\t%s %lf\n", str, magnitudes[i]);

	str = getKSString(KS_HERMITE_SPLINE_N_SAMPLES);
	fprintf(f, "\t%s %d\n", str, nSamples);

	str = getKSString(KS_END);
	fprintf(f, "%s\n\n\n", str);

	return true;
}


bool KS_HermiteSplineLinkBar::loadFromFile(FILE* f){
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
			case KS_END:{
				generatePointsFromSpline();
				setupGeometry();
				return true;
				}break;
			case KS_WIDTH:
				if(sscanf(line, "%lf", &width) != 1) assert(false);
				break;
			case KS_BAR_END_POINTS:{
				Point3d p;
				sscanf(line, "%lf %lf\n", &p.x, &p.y);
				midLinePoints.push_back(p);
				}break;
			case KS_HERMITE_SPLINE_N_SAMPLES:
				if(sscanf(line, "%d", &nSamples) != 1) assert(false);
				break;
			case KS_HERMITE_SPLINE_ANGLES:{
				double angle;
				sscanf(line, "%lf\n", &angle);
				angles.push_back(angle);
				}break;
			case KS_HERMITE_SPLINE_MAGNITUDES:{
				double magnitude;
				sscanf(line, "%lf\n", &magnitude);
				magnitudes.push_back(magnitude);
				}break;
			default:
				logPrint("Incorrect KS input file. Unexpected line: %s\n", buffer);
				return false;
		}
	}

	logPrint("KS_MultiLinkBar: Warning - end of file met before END primitive\n");
	return false;
}

void KS_HermiteSplineLinkBar::generatePointsFromSpline(){

	nSamples = (nSamples < 3) ? 3 : nSamples;

	double dt = 1.0/(double)(nSamples-1);

	int pointIndex;

	for(int i = 0; i < ((int)angles.size())-1; i++){
		pointIndex = (nSamples-1)*i;

		Point3d P0 = midLinePoints[pointIndex], P1 = midLinePoints[pointIndex+1];
		Vector3d R0 = Vector3d(cos(angles[i]),sin(angles[i]),0)*magnitudes[i], R1 = Vector3d(cos(angles[i+1]),sin(angles[i+1]),0)*magnitudes[i+1];
		
		double t2, t3;
		double coeffP0, coeffR0, coeffP1, coeffR1;

		int k = 0;
		for(double t = dt; k < nSamples-2; t += dt, k++)
		{
			t2 = t*t;
			t3 = t2*t;

			coeffP0 = 2.0*t3-3.0*t2+1;
			coeffR0 = t3-2.0*t2+t;
			coeffP1 = -2.0*t3+3.0*t2;
			coeffR1 = t3-t2;		

			Point3d newPoint = P0*coeffP0 + R0*coeffR0 + P1*coeffP1 + R1*coeffR1;
			midLinePoints.insert(midLinePoints.begin()+pointIndex+1+k,newPoint);
		}
	}
}