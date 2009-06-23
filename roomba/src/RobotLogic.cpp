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

RobotLogic::RobotLogic(Os& os, Uart *pUart, Communication *pCommunication) :
	m_pOs(os),
	m_Robot(os)
{
  // TODO Auto-generated constructor stub
  m_Robot.initialize(pUart);
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
//			m_pOs.debug("doTask: drive  Param0:%i  Param1:%i",parameters[0],parameters[1]);
			m_Robot.drive((uint16) parameters[0], (uint16) parameters[1]);
		}
	}

	if(strcmp(taskName, "turn") == 0)
	{
		if(paramLength == 2)
		{
//			m_pOs.debug("doTask: turn");
			turn((int16) parameters[0], (uint8) (parameters[1] & 0xff));
		}
	}

	if(strcmp(taskName, "turnInfinite") == 0)
	{
		if(paramLength == 1)
		{
//			m_pOs.debug("doTask: turnInfinite");
			turnInfinite((int16) parameters[0]);
		}
	}

	if(strcmp(taskName, "stop") == 0)
	{
//		m_pOs.debug("doTask: stop");
		stop();
	}
}

void RobotLogic::doTask(uint8 taskID, uint8 paramLength, int16 *parameters)
{
//  m_pOs.debug("doTask ID, ID: %i, paramLength: %i", taskID, paramLength);
  if(taskID == 1)
  {
    if(paramLength == 2)
    {
//      m_pOs.debug("doTask: drive");
      m_Robot.drive((uint16) parameters[0], (uint16) parameters[1]);
    }
  }

  if(taskID == 2)
  {
    if(paramLength == 2)
    {
//      m_pOs.debug("doTask: turn");
      turn(parameters[0], (uint8) (parameters[1] & 0xff));
    }
  }

  if(taskID == 3)
  {
	  if(paramLength == 1)
	  {
//		m_pOs.debug("doTask: turn forever");
		turnInfinite(parameters[0]);
	  }
  }

  if(taskID == 4)
  {
//		m_pOs.debug("doTask: stop");
		stop();
  }
}

void RobotLogic::getCapabilities()
{
	m_pOs.debug("getCapabilities start");
	uint8 taskListLength = 4;
	const char* taskList[]={"drive","turn","turnInfinite","stop"};
	const char*** paramList;
	const uint8 paramListLength[]={2,2,1,0};

	paramList = ((const char ***)isense::malloc(sizeof (const char **) * taskListLength));
	for (int i = 0; i < taskListLength; ++i)
	{
		int bytesNeeded = (sizeof (const char *) * paramListLength[i]);
		m_pOs.debug("multiplikation: %i, sizeof (const char **): %i, paramListLength: %i", bytesNeeded, sizeof (const char *), paramListLength[i]);
		if(bytesNeeded > 0) {
			paramList[i] = ((const char **)isense::malloc(bytesNeeded));
		}
	}
	m_pOs.debug("getCapabilities after mallocs");

	paramList[0][0] = "velocity";
	paramList[0][1] = "radius";
	paramList[1][0] = "angle";
	paramList[1][1] = "randomComponent";
	paramList[2][0] = "direction";

	uint16 nodeID = m_pOs.id();

	Communication *m_pComm;
	m_pComm = ((roombatest *) m_pOs.application())->getCommunication();

	uint8 buf[128];
	uint8 len = m_pComm->sendFeatures(nodeID, taskListLength, taskList, paramListLength, paramList, buf);

	Flooding& flooding = ((roombatest *) m_pOs.application())->getFlooding();
	flooding.send(len,  buf);

	for (int i = 0; i < taskListLength; ++i)
	{
//		m_pOs.debug("getCapabilities: freeing paramlist[%i]", i);
		isense::free (paramList[i]);
	}
	m_pOs.debug("getCapabilities after freeing paramlist members");
	isense::free (paramList);

	m_pOs.debug("getCapabilities end");
}

void RobotLogic::turn(int16 angle, uint8 randomComponent)
{
  m_pOs.debug("turn, angle: %i, randomComp: %i", angle, randomComponent);
//  int8 random = (int8) (m_randOmat.rand(randomComponent) & 0xff);
//  angle += (int16) random;
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
	m_Robot.driveDirect(0,0);
}
