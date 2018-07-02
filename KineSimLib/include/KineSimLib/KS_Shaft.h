#pragma once
#include "KS_MechanicalComponent.h"

class KS_Shaft : public KS_MechanicalComponent {
public:
	KS_Shaft(char* sName);
	~KS_Shaft(void);
	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);
	virtual KS_Shaft* clone() const;

	void setLength(double l){length = l;}
	void setRadius(double r){radius = r;}

	
private:
/**
	properties of shafts:
*/
	//the radius of the KS_Shaft
	double radius;
	//the length of the KS_Shaft
	double length;

	void setupGeometry();
};

