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
#include "roombatest.cpp"

RobotLogic::RobotLogic(Uart *pUart, Communication *pCommunication)
{
  // TODO Auto-generated constructor stub
  m_ourRobot.initialize(pUart);
  m_randOmat.srand((uint32) (JennicOs::os_pointer()->time()).ms());

//  m_pCommunication = ((roombatest *) JennicOs::os_pointer()->application())->getCommunication();
  m_pCommunication = pCommunication;
}

RobotLogic::~RobotLogic()
{
  // TODO Auto-generated destructor stub
}

void RobotLogic::doTask(const char* taskName, uint8 paramLength, const uint16 *parameters)
{
//	JennicOs::os_pointer()->debug("doTask STRING, ID: %s, paramLength: %i", taskName, paramLength);
	if(strcmp(taskName, "drive") == 0)
	{
		if(paramLength == 2)
		{
//			JennicOs::os_pointer()->debug("doTask: drive  Param0:%i  Param1:%i",parameters[0],parameters[1]);
			m_ourRobot.drive((uint16) parameters[0], (uint16) parameters[1]);
		}
	}

	if(strcmp(taskName, "turn") == 0)
	{
		if(paramLength == 2)
		{
//			JennicOs::os_pointer()->debug("doTask: turn");
			turn((int16) parameters[0], (uint8) (parameters[1] & 0xff));
		}
	}

	if(strcmp(taskName, "turnInfinite") == 0)
	{
		if(paramLength == 1)
		{
//			JennicOs::os_pointer()->debug("doTask: turnInfinite");
			turnInfinite((int16) parameters[0]);
		}
	}

	if(strcmp(taskName, "stop") == 0)
	{
//		JennicOs::os_pointer()->debug("doTask: stop");
		stop();
	}
}

void RobotLogic::doTask(uint8 taskID, uint8 paramLength, int16 *parameters)
{
//  JennicOs::os_pointer()->debug("doTask ID, ID: %i, paramLength: %i", taskID, paramLength);
  if(taskID == 1)
  {
    if(paramLength == 2)
    {
//      JennicOs::os_pointer()->debug("doTask: drive");
      m_ourRobot.drive((uint16) parameters[0], (uint16) parameters[1]);
    }
  }

  if(taskID == 2)
  {
    if(paramLength == 2)
    {
//      JennicOs::os_pointer()->debug("doTask: turn");
      turn(parameters[0], (uint8) (parameters[1] & 0xff));
    }
  }

  if(taskID == 3)
  {
	  if(paramLength == 1)
	  {
//		JennicOs::os_pointer()->debug("doTask: turn forever");
		turnInfinite(parameters[0]);
	  }
  }

  if(taskID == 4)
  {
//		JennicOs::os_pointer()->debug("doTask: stop");
		stop();
  }
}

void RobotLogic::getCapabilities()
{
	JennicOs::os_pointer()->debug("getCapabilities start");
	uint8 taskListLength = 4;
	const char* taskList[]={"drive","turn","turnInfinite","stop"};
	const char*** paramList;

	const uint8    paramListLength[]={2,2,1,0};

	JennicOs::os_pointer()->debug("multiplikation: %i, sizeof (const char **): %i, len: %i", sizeof (const char **) * taskListLength, sizeof (const char **), taskListLength);

//#define STRING_MATRIX_NEW(len) ((const char ***)isense::malloc(sizeof (const char **) * len))
//#define STRING_ARRAY_NEW(len) ((const char **)isense::malloc(sizeof (const char *) * len))

	paramList = ((const char ***)isense::malloc(sizeof (const char **) * taskListLength));
	for (int i = 0; i < taskListLength; ++i)
	{
		int bytesNeeded = (sizeof (const char *) * paramListLength[i]);
		JennicOs::os_pointer()->debug("multiplikation: %i, sizeof (const char **): %i, paramListLength: %i", bytesNeeded, sizeof (const char *), paramListLength[i]);
		if(bytesNeeded > 0) {
			paramList[i] = ((const char **)isense::malloc(bytesNeeded));
		}
	}
	JennicOs::os_pointer()->debug("getCapabilities after mallocs");

	paramList[0][0] = "velocity";
	paramList[0][1] = "radius";
	paramList[1][0] = "angle";
	paramList[1][1] = "randomComponent";
	paramList[2][0] = "direction";

	JennicOs::os_pointer()->debug("getCapabilities after writing strings");

	uint16 nodeID = JennicOs::os_pointer()->id();

	JennicOs::os_pointer()->debug("getCapabilities, nodeID: %i", nodeID);

	m_pCommunication->sendFeatures(nodeID, taskListLength, taskList, paramListLength, paramList);

	JennicOs::os_pointer()->debug("getCapabilities after sendFeatures");

	for (int i = 0; i < taskListLength; ++i)
	{
//		JennicOs::os_pointer()->debug("getCapabilities: paramlist[%i]: %s", i, paramList[i]);
		isense::free (paramList[i]);
	}
	isense::free (paramList);

	JennicOs::os_pointer()->debug("getCapabilities end");
}

void RobotLogic::turn(int16 angle, uint8 randomComponent)
{
  JennicOs::os_pointer()->debug("turn, angle: %i, randomComp: %i", angle, randomComponent);
//  int8 random = (int8) (m_randOmat.rand(randomComponent) & 0xff);
//  angle += (int16) random;
  JennicOs::os_pointer()->debug("turn, angle incl. random: %i", angle);
  uint8 script[] = { CMD_DRIVE,  0, 255, 0xff, 0xff,
            CMD_WAIT_ANGLE, (uint8) ((angle & 0xff00) >> 8), (uint8) (angle & 0xff),
            CMD_DRIVE, 0, 0, 0, 0};
  if(angle > 0)
  {
    script[3] = 0;
    script[4] = 1;
  }
  m_ourRobot.setScript(script, sizeof(script));
  m_ourRobot.executeScript();
  JennicOs::os_pointer()->debug("turn end");
}

void RobotLogic::turnInfinite(int16 turnVelocity)
{
	m_ourRobot.driveDirect(turnVelocity,-turnVelocity);
}

void RobotLogic::stop()
{
	m_ourRobot.driveDirect(0,0);
}
