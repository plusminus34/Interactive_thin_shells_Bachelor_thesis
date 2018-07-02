#pragma once
#include "KS_MechanicalComponent.h"


class KS_Quad : public KS_MechanicalComponent{
public:
	KS_Quad(char* rName);
	~KS_Quad(void);

	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);
	virtual KS_Quad* clone() const;

	double getThickness(){return thickness;}

private:
/**
	properties of Bars:
*/
	//the width and thickness of the Bar
	double thickness;

	Point3d p1, p2, p3, p4;

	void setupGeometry();
};

