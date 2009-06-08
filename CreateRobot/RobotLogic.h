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

#include <isense/util/util.h>
#include <isense/platforms/jennic/jennic_os.h>
#include <isense/util/pseudo_random_number_generator.h>
#include "CreateRobot.h"

using namespace isense;

#ifndef ROBOTLOGIC_H_
#define ROBOTLOGIC_H_

class RobotLogic {
public:
	RobotLogic(Uart *pUart);
	virtual ~RobotLogic();
	void doTask(string taskName, uint8 paramLength, int16 *parameters);
	void doTask(uint8 taskID, uint8 paramLength, int16 *parameters);
	string getCapabilities();

protected:
	Robot m_ourRobot;
	PseudoRandomNumberGenerator m_randOmat;
	void turn(int16 angle, uint8 randomComponent);

};

#endif /* ROBOTLOGIC_H_ */
