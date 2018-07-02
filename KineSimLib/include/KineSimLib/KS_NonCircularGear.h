#pragma once

#include "KS_MechanicalComponent.h"
#include <MathLib/Trajectory.h>


class KS_NonCircularGear : public KS_MechanicalComponent {
friend class KS_NonCircularGearsConnection;
private:
/**
	properties of non-circular gears (NOTE: many of them are inferred from the connection, which also stores very important information...)
*/
	//thickness of the component
	double thickness;
	//number of teeth
	int numberOfTeeth;
	//slope of the teeth, relative to the normal direction of the gear at the middle of the tooth (also known as pressure angle)
	double pressureAngle;
	//this is the height of each tooth
	double teethHeight;

	void createGearGeometry(Trajectory3D* gearProfile, double gearCircumference, double teethOffset, bool clockwise);

public:
	KS_NonCircularGear(char* name);
	~KS_NonCircularGear (void);

	void setKS_NonCircularGearParameters(double pthickness, int pNumberOfTeeth, double pPressureAngle, double pTeethHeight);

	//access methods
	int getNumTeeth() const { return  numberOfTeeth; }
	void setNumTeeth(int n) { numberOfTeeth = n; }
	void setTeethHeight(double h) { teethHeight = h; }
	void setPressureAngle(double p) { pressureAngle = p; }

	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);
	virtual KS_NonCircularGear* clone() const;

	KS_NonCircularGear* getCopy() const;

};


