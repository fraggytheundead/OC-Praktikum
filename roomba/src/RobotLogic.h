/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/

/*
 * RobotLogic.h
 *
 *  Created on: Jun 4, 2009
 *      Author: Alexander Böcken, Michael Brehler, Stephan Lohse, Tobias Witt
 */

#ifndef __OCPROJ_ROBOTLOGIC_H
#define __OCPROJ_ROBOTLOGIC_H

#include <isense/os.h>
#include <isense/util/util.h>
//#include <isense/platforms/jennic/jennic_os.h>
#include <isense/util/pseudo_random_number_generator.h>
#include "CreateRobot.h"
#include "Communication.h"
#include <isense/isense_memory.h>
#include <isense/protocols/routing/neighborhood_monitor.h>

using namespace isense;

// Task IDs:
#define ROBOT_ACTION_STOP		1

// struct for tasks - contains the ID of the task to be executed
// and the time at which it was scheduled.
struct taskStruct {
	uint8 id;
	Time time;
};

class RobotLogic: public RobotHandler,
public isense::TimeoutHandler,
public isense::Task
{
public:
	RobotLogic(Os& os, Uart *pUart, Communication *pCommunication);
	virtual ~RobotLogic();
	void doTask(const char* taskName, uint8 paramLength, const uint16 *parameters);
	void getCapabilities();
	virtual void onStateChanged(PCROBOTSTATE pState);
	virtual void onChecksumError();

	///From isense::TimeoutHandler
	virtual void timeout(void *userdata);
	///From isense::Task
	virtual void execute(void *userdata);

protected:
	PseudoRandomNumberGenerator m_randOmat;
	void turn(int16 angle, uint8 randomComponent, Time actionTime);
	void turnInfinite(int16 direction);
	void stop();
	void spread(uint16 tempID,uint8 tempThreshold);
	void gather(uint16 tempID,uint8 tempThreshold);
	void randomDrive();
	void usedemo(int demoNr);
	void driveDistance(uint16 speed, uint16 radius, uint16 distance, Time actionTime);
	Communication *m_pCommunication;
	Os& m_pOs;
	Robot m_Robot;
	NeighborhoodMonitor m_neighborhoodMonitor;
	void miTheme();
	const static int8 cSPREAD=1;
	const static int8 cGATHER=2;
	const static int8 cRANDOMDRIVE=3;
	const static int8 cMITHEME=4;
	int8 activeTask;
	uint16 centerID;
	int timeoutCounter;
	uint8 songNumber;
	bool taskBool;
	bool timeoutBool;
	NeighborhoodMonitor::neighbor* neighbors;
	NeighborhoodMonitor::neighbor* neighborsCopy;
	uint16 centerQualityID[20];
	uint8 centerQuality[20];
	uint8 centerCounter;
	uint8 centerThreshold;
	const static uint8 maxCenterCounter=6;
	Time lastAction;

};

#endif /* __OCPROJ_ROBOTLOGIC_H */
