#pragma once

#include <stdio.h>
#include <MathLib/MathLib.h>
#include "KineSimLib\KS_MechanicalAssembly.h"

class KS_MechanismController{

public:
	KS_MechanismController(KS_MechanicalAssembly* mech);
	virtual ~KS_MechanismController(void);
	
	virtual void setMotorAngleValues() = 0; //each controller will have its own way of specifying the target motor angle values
	virtual void updateMotorConnections();
	virtual void activateMechanismController(); // updates the motor angles in the connections;

	virtual void initialize(KS_MechanicalAssembly* mech); // initializes the members

	//calculates the number of actuated connections mechanism controller operates on
	virtual void setActuatedConnectionCount();
	virtual int getActuatedConnectionCount();

public:
	KS_MechanicalAssembly* mechanism;
	//dVector desiredState; in the specific instance of the IK controller
	dVector motorAngleValues;
	dVector startingMechState; //saves the starting mechanism state for restart

protected:
	int sizeActuatedConnections;
	
};

