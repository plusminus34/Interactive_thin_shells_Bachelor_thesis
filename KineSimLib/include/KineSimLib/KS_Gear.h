#pragma once

#include "KS_MechanicalComponent.h"

/**
==================================================================================================
 	Computational Design of Mechanical Assemblies
	---------------------------------------------

  These gears need to be mounted on a shaft (input or output), they can be co-axial with other 
  gears (compound KS_Gear trains), they mesh with other toothed devices (spur KS_Gear, worm KS_Gear, rack, 
  bevel gears) and they can be directly driven by their shaft, by an attached rod, or by a KS_Gear.

  These gears are parameterizable: spur gears, bevel gears and crown gears can all be achieved
  using this component.
==================================================================================================
*/

class KS_Gear : public KS_MechanicalComponent {
private:
/**
	properties of gears:
*/
	//thickness.
	double thickness;
	//effective radius of the KS_Gear. Two gears that are meshing have their pitch circles touching at one point (point of tangency).
	double pitchRadius;
	//this is the angle between the axis of the KS_Gear and the pitch surface. When this is 0, the KS_Gear is a spur KS_Gear. At 45 it is the typical bevel KS_Gear, and at 90 it is a crown KS_Gear.
	double pitchSurfaceAngle;
	//number of teeth
	int numberOfTeeth;
	//slope of the teeth, relative to the normal direction of the KS_Gear at the middle of the tooth (also known as pressure angle)
	double pressureAngle;
	//this is the height of each tooth
	double teethHeight;

	//this parameter is purely cosmetic. It ensures that gears mesh properly with each other when all offsets are 0.
	double meshOffsetAngle;


	void setThickness(double t) { thickness = t; }

	double phaseDriverOffset;

public:
	KS_Gear(char* name);
	~KS_Gear(void);

	void setSpurGearParameters(double pthickness, double pPitchRadius, double pPitchSurfaceAngle, int pNumberOfTeeth, double pPressureAngle, double pTeethHeight);
	void setupGeometry();

	//access methods
	int getNumTeeth() const { return  numberOfTeeth; }
	void setNumTeeth(int n) { numberOfTeeth = n; }
	void setTeethHeight(double h) { teethHeight = h; }
	double getTeethHeight() const { return teethHeight; }
	double getPitchRadius()const { return pitchRadius; }
	void setPitchRadius(double r) { pitchRadius = r; }
	double getThickness(){return thickness;}

	virtual bool loadFromFile(FILE* f);
	virtual bool writeToFile(FILE* f);
	virtual KS_Gear* clone() const;
	void setPressureAngle( double pressureAngle );
	void setMeshOffsetAngle( double meshOffsetAngle );
	void setPitchSurfaceAngle( double pitchSurfaceAngle );
	
	
	void setPhaseDriverShift( double phaseShift ) {phaseDriverOffset = phaseShift;}
	double getPhaseDriverShift() { return phaseDriverOffset; }


};


