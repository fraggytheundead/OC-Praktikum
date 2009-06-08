/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/
#include <isense/application.h>
#include <isense/os.h>
#include <isense/dispatcher.h>
#include <isense/radio.h>
#include <isense/task.h>
#include <isense/timeout_handler.h>
#include <isense/isense.h>
#include <isense/uart.h>
#include <isense/dispatcher.h>
#include <isense/time.h>
#include <isense/button_handler.h>
#include <isense/sleep_handler.h>
#include <isense/modules/pacemate_module/pacemate_module.h>
#include <isense/util/util.h>
#include "RobotLogic.h"

#define MILLISECONDS 1000
//----------------------------------------------------------------------------
/**
 */

using namespace isense;

//Os& globalOs;

class roombatest :
	public isense::Application,
	public isense::Receiver,
	public isense::Sender,
	public isense::Task,
	public isense::TimeoutHandler,
	public isense::SleepHandler,
	public ButtonHandler,
	public RobotHandler
{
public:
	roombatest(isense::Os& os);

	virtual ~roombatest() ;

	///From isense::Application
	virtual void boot (void) ;

	///From isense::SleepHandler
	virtual bool stand_by (void) ; 	// Memory held

	///From isense::SleepHandler
	virtual bool hibernate (void) ;  // Memory not held

	///From isense::SleepHandler
	virtual void wake_up (bool memory_held) ;

	///From isense::ButtonHandler
	virtual void button_down( uint8 button );

	///From isense::Receiver
	virtual void receive (uint8 len, const uint8 * buf, uint16 src_addr, uint16 dest_addr, uint16 lqi, uint8 seq_no, uint8 interface) ;

	///From isense::Sender
	virtual void confirm (uint8 state, uint8 tries, isense::Time time) ;

	///From isense::Task
	virtual void execute( void* userdata ) ;

	///From isense::TimeoutHandler
	virtual void timeout( void* userdata ) ;

	virtual void onStateChanged(PCROBOTSTATE pState) ;

	virtual void onChecksumError() ;
private:
	Uart& ourUart_;
	char* ourWriteBuffer;
	bool driving;
	RobotLogic ourRobot;

};

//----------------------------------------------------------------------------
roombatest::
	roombatest(isense::Os& os)
	: isense::Application(os),
	ourUart_(os_.uart(1)),
	ourRobot(&ourUart_)
	{
	}

//----------------------------------------------------------------------------
roombatest::
	~roombatest()

	{
	}

//----------------------------------------------------------------------------
void
	roombatest::
	boot(void)
	{
        os_.debug("App::boot");
        os_.allow_sleep(false);
        os_.set_log_mode(ISENSE_LOG_MODE_RADIO);
        os_.debug("Debug over Radio");
//        globalOs = os_;
//        ourRobot.initialize(&ourUart_);
//        ourRobot.startDemo(4);
//        char *script = (char*) malloc(24*sizeof(char));
//        uint8 script[] = {137, 1, 44, 128, 0, 156, 1, 144, 137, 1, 44, 0, 1, 157, 0, 90, 153};
//        ourRobot.setScript(script, sizeof(script));
//        ourRobot.executeScript();
//        ourRobot.setRobotHandler(this);
//        uint8 packets[] = {9, 25, 7, STREAM_BATTERY_TEMPERATURE};
//        uint8 packets[] = {9};
//        ourRobot.streamPackets(1, packets);
        int16 parameter[] = {45, 0};
        ourRobot.doTask(2, 2, parameter);
        parameter[0] = 90;
        ourRobot.doTask(2, 2, parameter);
        parameter[0] = 180;
        ourRobot.doTask(2, 2, parameter);
        parameter[0] = -315;
        ourRobot.doTask(2, 2, parameter);
//        parameter[0] = 180;
//        parameter[1] = 45;
//        ourRobot.doTask(2, (uint8) sizeof(parameter), parameter);

 	}

//----------------------------------------------------------------------------
bool
	roombatest::
	stand_by (void)
	{
		os_.debug("App::sleep");
		return true;
	}

//----------------------------------------------------------------------------
bool
	roombatest::
	hibernate (void)
	{
		os_.debug("App::hibernate");
		return false;
	}

//----------------------------------------------------------------------------
void
	roombatest::
	wake_up (bool memory_held)
	{
		os_.debug("App::Wakeup");
	}

void
	roombatest::
	button_down( uint8 button )
	{

	}
//----------------------------------------------------------------------------
void
	roombatest::
	execute( void* userdata )
	{
		//os_.debug("exe");

	}

//----------------------------------------------------------------------------
void
	roombatest::
	receive (uint8 len, const uint8 * buf, uint16 src_addr, uint16 dest_addr, uint16 lqi, uint8 seq_no, uint8 interface)
	{
	}

//----------------------------------------------------------------------------
void
	roombatest::
	confirm (uint8 state, uint8 tries, isense::Time time)
	{
	}

//----------------------------------------------------------------------------
void
	roombatest::
	timeout( void* userdata )
	{
		os_.add_task( this, NULL);
		os_.add_timeout_in(Time(MILLISECONDS), this, NULL);
	}

//----------------------------------------------------------------------------
/**
  */
isense::Application* application_factory(isense::Os& os)
{
	return new roombatest(os);
}


/*-----------------------------------------------------------------------
* Source  $Source: $
* Version $Revision: 1.24 $
* Date    $Date: 2006/10/19 12:37:49 $
*-----------------------------------------------------------------------
* $Log$
*-----------------------------------------------------------------------*/

void roombatest::onStateChanged(PCROBOTSTATE pState) {
	os_.debug("LeftCliff: %i, batteryTemperature: %i, batteryCharge: %i, bumpAndWheelDrop: %i", pState->cliff, pState->batteryTemperature, pState->batteryCharge, pState->bumpAndWheelDrop);
}

void roombatest::onChecksumError() {
	os_.debug("Checksum Error");
}
