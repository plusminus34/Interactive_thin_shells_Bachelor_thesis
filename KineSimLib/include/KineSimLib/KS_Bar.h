#pragma once
#include "KS_MechanicalComponent.h"


class KS_Bar : public KS_MechanicalComponent{
public:
	KS_Bar(char* rName);
	~KS_Bar(void);

	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);
	virtual KS_Bar* clone() const;

	void setWidth(double w) { width = w; }
	void setDimensions(V3D s, V3D m, V3D e);
	void setHollowIntervals( const DynamicArray<double> v) { hollowIntervals.clear(); hollowIntervals.resize(v.size()); std::copy(v.begin(), v.end(), hollowIntervals.begin());}

	double getThickness(){return thickness;}
private:
/**
	properties of Bars:
*/
	//the width and thickness of the Bar
	double thickness, width;
	//we assume that the KS_Bar is made of two piece-wise linear parts, in order to be able to generate arbitrary shapes. For this reason,
	//we store (y,z) coordinates for the start, middle and end point of the KS_Bar
	V3D start, middle, end;
	//we allow the KS_Bar to be hollowed out so that it can slide around pins. The intervals stored here correspond to the start and end
	//locations of the hollowed out intervals
	DynamicArray<double> hollowIntervals;

	void setupGeometry();
	void addBarVertices(GLMesh* tmpMesh, double progress, const V3D& startUp, const V3D& middleUp, const V3D& endUp, const V3D& startDown, const V3D& middleDown, const V3D& endDown, bool createQuadLower, bool createQuadMiddle, bool createQuadUpper, bool flipNormals = false);
};

