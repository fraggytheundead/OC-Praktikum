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

RobotLogic::RobotLogic(Uart *pUart)
{
	// TODO Auto-generated constructor stub
	m_ourRobot.initialize(pUart);
	m_randOmat.srand((uint32) (JennicOs::os_pointer()->time()).ms());
}

RobotLogic::~RobotLogic()
{
	// TODO Auto-generated destructor stub
}

//void RobotLogic::doTask(string taskName, uint8 paramLength, int16 parameters[])
//{
//	if(strcmp(taskname, "drive"))
//	{
//		if(paramLength == 2)
//		{
//			m_ourRobot.drive((uint16) parameters[0], (uint16) parameters[1]);
//		}
//	}
//
//	if(strcmp(taskname, "turn"))
//	{
//		if(paramLength == 2)
//		{
//			turn(parameters[0], (uint8) (parameters[1] & 0xff));
//		}
//	}
//}

void RobotLogic::doTask(uint8 taskID, uint8 paramLength, int16 *parameters)
{
	JennicOs::os_pointer()->debug("doTask, ID: %i, paramLength: %i", taskID, paramLength);
	if(taskID == 1)
	{
		if(paramLength == 2)
		{
			JennicOs::os_pointer()->debug("doTask: drive");
			m_ourRobot.drive((uint16) parameters[0], (uint16) parameters[1]);
		}
	}

	if(taskID == 2)
	{
		if(paramLength == 2)
		{
			JennicOs::os_pointer()->debug("doTask: turn");
			turn(parameters[0], (uint8) (parameters[1] & 0xff));
		}
	}
}

string RobotLogic::getCapabilities()
{
	return NULL;
}

void RobotLogic::turn(int16 angle, uint8 randomComponent)
{
	JennicOs::os_pointer()->debug("turn, angle: %i, randomComp: %i", angle, randomComponent);
//	int8 random = (int8) (m_randOmat.rand(randomComponent) & 0xff);
//	angle += (int16) random;
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
