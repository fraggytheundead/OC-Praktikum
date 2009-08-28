/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/

/*
 * roombatest.h
 *
 *  Created on: Jun 18, 2009
 *      Author: fraggy
 */

#ifndef __OCPROJ_ROOMBATEST_H_
#define __OCPROJ_ROOMBATEST_H_

#include <isense/application.h>
#include <isense/os.h>
#include <isense/dispatcher.h>
#include <isense/radio.h>
#include <isense/hardware_radio.h>
#include <isense/task.h>
#include <isense/timeout_handler.h>
#include <isense/isense.h>
#include <isense/uart.h>
#include <isense/dispatcher.h>
#include <isense/time.h>
#include <isense/sleep_handler.h>
#include <isense/modules/pacemate_module/pacemate_module.h>
#include <isense/util/util.h>
#include <isense/protocols/routing/flooding.h>
#include "Robot.h"

#define MILLISECONDS 1000

// Packet types
#define ROOMBA_REMOTE_START_COMMUNICATION		1
#define ROOMBA_REMOTE_END_COMMUNICATION			2
#define ROOMBA_REMOTE_PING						3
#define ROOMBA_REMOTE_PONG						4
#define ROOMBA_REMOTE_DRIVE						11

// Tasks
#define TASK_CONNECTCAND_TIMEOUT				1
#define TASK_PINGPONG							2
#define TASK_CONNECTION_TIMEDOUT				3

#define TASK_CONNECT_BLINK						5


using namespace isense;

class roombatest: public isense::Application,
		public isense::Receiver,
		public isense::Sender,
		public isense::Task,
		public isense::TimeoutHandler,
		public isense::SleepHandler,
		public RobotHandler
		{
public:
	roombatest(isense::Os& os);

	virtual ~roombatest();

	///From isense::Application
	virtual void boot(void);

	///From isense::SleepHandler
	virtual bool stand_by(void); // Memory held

	///From isense::SleepHandler
	virtual bool hibernate(void); // Memory not held

	///From isense::SleepHandler
	virtual void wake_up(bool memory_held);

	///From isense::Receiver
	virtual void receive(uint8 len, const uint8 * buf, uint16 src_addr,
			uint16 dest_addr, uint16 lqi, uint8 seq_no, uint8 interface);

	///From isense::Sender
	virtual void confirm(uint8 state, uint8 tries, isense::Time time);

	///From isense::Task
	virtual void execute(void* userdata);

	///From isense::TimeoutHandler
	virtual void timeout(void* userdata);

	/// from RobotHandler
	virtual void onIoModeChanged(uint8 ioMode);
	virtual void onPowerStateChanged(PCPOWERSTATE pState);
	virtual void onCliffStateChanged(PCCLIFFSTATE pState);
	virtual void onMovementStateChanged(PCMOVEMENTSTATE pState);
	virtual void onButtonChanged(uint8 buttons);
	virtual void onWallSensorChanged(uint8 wall, uint8 virtualWall);
	virtual void onBumpAndWheelDrop(uint8 bumpsAndWheelDrop);
	virtual void onSongStateChanged(uint8 songNumber, uint8 songPlaying);

private:
	Uart& ourUart_;
	Robot m_robot;
	uint16 connectedTo;
	uint16 connectCandidate;
	void interpretMessage(uint8 len, const uint8 * buf);
	Time lastContact;
	bool bumped;
	CoreModule *cm_;
	bool connectLed;
};

#endif /* __OCPROJ_ROOMBATEST_H_ */
