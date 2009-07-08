/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/

/*
 * RobotLogic.cpp
 *
 *  Created on: Jun 4, 2009
 *      Author: Alexander Böcken, Stephan Lohse, Tobias Witt
 */

#include "RobotLogic.h"
#include "roombatest.h"

//#define DEBUG_GET_CAPABILITIES

RobotLogic::RobotLogic(Os& os, Uart *pUart, Communication *pCommunication) :
	m_pOs(os),
	m_Robot(os)
{
  m_Robot.initialize(pUart);
  m_Robot.setRobotHandler(this);
  m_randOmat.srand((uint32) (m_pOs.time()).ms());

  m_pCommunication = pCommunication;
}

RobotLogic::~RobotLogic()
{
  // TODO Auto-generated destructor stub
}

void RobotLogic::doTask(const char* taskName, uint8 paramLength, const uint16 *parameters)
{
	m_pOs.debug("doTask STRING, ID: %s, paramLength: %i", taskName, paramLength);
	if(strcmp(taskName, "drive") == 0)
	{
		if(paramLength == 2)
		{
			m_pOs.debug("doTask: drive  Param0:%i  Param1:%i",parameters[0],parameters[1]);
			m_Robot.drive((uint16) parameters[0], (uint16) parameters[1]);
		}
	}

	if(strcmp(taskName, "turn") == 0)
	{
		if(paramLength == 2)
		{
			m_pOs.debug("doTask: turnParam0:%i  Param1:%i",parameters[0],parameters[1]);
			turn((int16) parameters[0], (uint8) (parameters[1] & 0xff));
		}
	}

	if(strcmp(taskName, "turnInfinite") == 0)
	{
		if(paramLength == 1)
		{
			m_pOs.debug("doTask: turnInfinite");
			turnInfinite((int16) parameters[0]);
		}
	}

	if(strcmp(taskName, "stop") == 0)
	{
		m_pOs.debug("doTask: stop");
		stop();
	}

	if(strcmp(taskName, "drveDist") == 0)
		{
			if(paramLength == 3)
			{
				m_pOs.debug("doTask: drveDist  Param0: %i  Param1: %i Param2: %i",parameters[0],parameters[1],parameters[2]);
				driveDistance((uint16) parameters[0], (uint16) parameters[1], (uint16) parameters[2]);
			}
		}
}

void RobotLogic::getCapabilities()
{
#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities start");
#endif
	uint8 taskListLength = 4;
	const char* taskList[]={"drive","turn","drveDist","stop"};
	const char*** paramList;
	const uint8 paramListLength[]={2,2,3,0};

	// TODO
	uint8 sensorLength = 0;
	char* sensors[]={};
	uint8 sensorRange[]={};

	paramList = ((const char ***)isense::malloc(sizeof (const char **) * taskListLength));
	for (int i = 0; i < taskListLength; ++i)
	{
		int bytesNeeded = (sizeof (const char *) * paramListLength[i]);
//		m_pOs.debug("multiplikation: %i, sizeof (const char **): %i, paramListLength: %i", bytesNeeded, sizeof (const char *), paramListLength[i]);
		if(bytesNeeded > 0) {
			paramList[i] = ((const char **)isense::malloc(bytesNeeded));
		}
	}
#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities after mallocs");
#endif

	paramList[0][0] = "speed";
	paramList[0][1] = "rad";
	paramList[1][0] = "angl";
	paramList[1][1] = "random";
	paramList[2][0] = "spd";
	paramList[2][1] = "rad";
	paramList[2][2] = "dist";

	uint16 nodeID = m_pOs.id();

#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities after assigning paramlist members");
#endif

	Communication *m_pComm;
	m_pComm = ((roombatest *) m_pOs.application())->getCommunication();

#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities after getting communication");
#endif

//	uint8 buf[128];
	m_pComm->sendFeatures(nodeID, taskListLength,
			taskList, paramListLength, paramList,
			sensorLength, sensors, sensorRange);

//#ifdef DEBUG_GET_CAPABILITIES
//	m_pOs.debug("getCapabilities after sendfeatures");
//	m_Robot.setLeds(0x02, 16, 255);
//#endif
//
//	Flooding& flooding = ((roombatest *) m_pOs.application())->getFlooding();
//	flooding.send(len,  buf);
//
//#ifdef DEBUG_GET_CAPABILITIES
//	m_pOs.debug("getCapabilities after flooding");
//	m_Robot.setLeds(0x08, 16, 255);
//#endif

	for (int i = 0; i < taskListLength; ++i)
	{
//		m_pOs.debug("getCapabilities: freeing paramlist[%i]", i);
		isense::free (paramList[i]);
	}

#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities after freeing paramlist members");
#endif

	isense::free (paramList);

#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities end");
	m_Robot.setLeds(0x0A, 16, 255);
#endif
}

void RobotLogic::turn(int16 angle, uint8 randomComponent)
{
  m_pOs.debug("turn, angle: %i, randomComp: %i", angle, randomComponent);
  int8 random = (int8) (m_randOmat.rand(randomComponent) & 0xff);
  angle += (int16) random;
  m_pOs.debug("turn, angle incl. random: %i", angle);

  uint8 script[] = { CMD_DRIVE,  0, 255, 0xff, 0xff,
            CMD_WAIT_ANGLE, (uint8) ((angle & 0xff00) >> 8), (uint8) (angle & 0xff),
            CMD_DRIVE, 0, 0, 0, 0};
  if(angle > 0)
  {
    script[3] = 0;
    script[4] = 1;
  }
  m_Robot.setScript(script, sizeof(script));
  m_Robot.executeScript();
  m_pOs.debug("turn end");
}

void RobotLogic::turnInfinite(int16 turnVelocity)
{
	m_Robot.driveDirect(turnVelocity,-turnVelocity);
}

void RobotLogic::stop()
{
	m_pOs.debug("RobotLogic stop");
	m_Robot.driveDirect(0,0);
}

void RobotLogic::driveDistance(uint16 speed, uint16 radius, uint16 distance) {
	static uint8 task = ROBOT_ACTION_STOP;
	int seconds = distance / speed;
	int msecs = ((1000 * distance) / speed) - 1000 * seconds;
	m_pOs.debug("RobotLogic driveDistance, distance: %i, speed: %i, seconds: %i, msecs: %i", distance, speed, seconds, msecs);
	Time distanceTime =  Time(seconds, msecs);
	m_Robot.drive(speed, radius);
	m_pOs.add_timeout_in(distanceTime, this, &task);

}

void RobotLogic::timeout(void *userdata)
{
	m_pOs.debug("RobotLogic timeout, ID: %i", *(uint8*) userdata);
	m_pOs.add_task(this, userdata);
}

void RobotLogic::execute(void *userdata)
{
	uint8 taskID = *(uint8*) userdata;
	m_pOs.debug("RobotLogic execute, taskID: %i", taskID);
	if(taskID == ROBOT_ACTION_STOP)
	{
		stop();
	}
}

void RobotLogic::onStateChanged(PCROBOTSTATE pState)
{
	static bool b = false;

//	if(b)
//		CoreModule(*g_pOS).led_on();
//	else
//		CoreModule(*g_pOS).led_off();
	m_pOs.debug("bumpAndWheelDrop: %x, batteryCharge: %i, batteryTemperature: %i",
			pState->bumpAndWheelDrop, pState->batteryCapacity, pState->batteryTemperature);

	b = !b;
}

void RobotLogic::onChecksumError()
{
	m_pOs.debug("ChecksumError");
}

