#pragma once

#include <stdio.h>
#include <MathLib/MathLib.h>
#include "..\include\KineSimLib\KS_MechanismController.h"


KS_MechanismController::KS_MechanismController(KS_MechanicalAssembly * mech)
{
	this->mechanism = mech;
	initialize(mech);
	motorAngleValues.resize(getActuatedConnectionCount()); motorAngleValues.setZero();

}

KS_MechanismController::~KS_MechanismController(void)
{
}

void KS_MechanismController::updateMotorConnections()
{
	for (int i = 0; i <getActuatedConnectionCount(); i++) {
		mechanism->actuated_connections[i]->setOffset(motorAngleValues[i]);
	}
	mechanism->updateActuatedConnections();
}

void KS_MechanismController::activateMechanismController()
{
	
}

void KS_MechanismController::initialize(KS_MechanicalAssembly * mech)
{
	setActuatedConnectionCount();
	motorAngleValues.resize(sizeActuatedConnections);
	motorAngleValues.setZero();
	startingMechState = mech->s;
}

void KS_MechanismController::setActuatedConnectionCount()
{
	sizeActuatedConnections = mechanism->actuated_connections.size();
}

int KS_MechanismController::getActuatedConnectionCount()
{
	setActuatedConnectionCount();
	return sizeActuatedConnections;
}
