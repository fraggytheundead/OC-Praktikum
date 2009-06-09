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
 *      Author: Alexander B�cken, Stephan Lohse, Tobias Witt
 */

#include <isense/util/util.h>
//#include <isense/platforms/jennic/jennic_os.h>
#include <isense/util/pseudo_random_number_generator.h>
#include "CreateRobot.h"
#include "Communication.h"

using namespace isense;

#ifndef ROBOTLOGIC_H_
#define ROBOTLOGIC_H_

class RobotLogic {
public:
  RobotLogic(Uart *pUart);
  virtual ~RobotLogic();
  void doTask(const char* taskName, uint8 paramLength, const uint16 *parameters);
  void doTask(uint8 taskID, uint8 paramLength, int16 *parameters);
  void getCapabilities();

protected:
  Robot m_ourRobot;
  PseudoRandomNumberGenerator m_randOmat;
  void turn(int16 angle, uint8 randomComponent);
  void turnInfinite(int16 direction);
  void stop();
  Communication* m_pCommunication;
};

#endif /* ROBOTLOGIC_H_ */
