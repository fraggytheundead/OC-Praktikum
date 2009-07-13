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
 *      Author: Alexander Böcken, Stephan Lohse, Tobias Witt
 */

#ifndef __OCPROJ_ROBOTLOGIC_H
#define __OCPROJ_ROBOTLOGIC_H

#include <isense/os.h>
#include <isense/util/util.h>
//#include <isense/platforms/jennic/jennic_os.h>
#include <isense/util/pseudo_random_number_generator.h>
#include "CreateRobot.h"
#include "Communication.h"

using namespace isense;

// Task IDs:
#define ROBOT_ACTION_STOP		1

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
	void turn(int16 angle, uint8 randomComponent);
	void turnBetter(int16 angle, uint8 randomComponent);
	void turnInfinite(int16 direction);
	void stop();
	void driveDistance(uint16 speed, uint16 radius, uint16 distance);
	Communication *m_pCommunication;
	Os& m_pOs;
	Robot m_Robot;

};

#endif /* __OCPROJ_ROBOTLOGIC_H */
