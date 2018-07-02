#pragma once
#include "KS_MechanicalComponent.h"


class KS_MultiLinkBar : public KS_MechanicalComponent{
public:
	KS_MultiLinkBar(char* rName);
	~KS_MultiLinkBar(void);

	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);
	virtual KS_MultiLinkBar* clone() const;

	void setMidLinePoints(const DynamicArray<Point3d> &pts);
	void getMidLinePoints(DynamicArray<Point3d> &pts);
	void setWidth(double w) { width = w; }
	Point3d sample(double t, Vector3d& derivative);


	double getThickness(){return thickness;}

protected:
/**
	properties of Bars:
*/
	//the width and thickness of the Bar
	double thickness, width;
	DynamicArray<Point3d> midLinePoints;

	void setupGeometry();
};

