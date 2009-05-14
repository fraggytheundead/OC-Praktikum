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

#define MILLISECONDS 1000
//----------------------------------------------------------------------------
/**
 */

using namespace isense;

class roombatest :
	public isense::Application,
	public isense::Receiver,
	public isense::Sender,
	public isense::Task,
	public isense::TimeoutHandler,
	public isense::SleepHandler,
	public ButtonHandler
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
private:
	Uart& ourUart_;
	char* ourWriteBuffer;
	bool driving;

};

//----------------------------------------------------------------------------
roombatest::
	roombatest(isense::Os& os)
	: isense::Application(os),
	ourUart_(os_.uart(1))
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
        if(!ourUart_.enabled()) ourUart_.enable();
        ourUart_.set_baudrate(19200);
        // 8 Databits, no flowcontrol, set Stoppbit to 1
        ourUart_.set_control( 8, 'N', 1 );
        // send roomba start command
        ourUart_.put( 0x80 );
        // put roomba into safe mode
        ourUart_.put( 0x83 );

        driving = false;

        ourWriteBuffer = (char*) malloc(5*(sizeof(char*)));
        os_.add_timeout_in(Time(MILLISECONDS), this, NULL);
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
		if(driving) {
			// stop
			*ourWriteBuffer = 137;
			*(ourWriteBuffer+1) = 0;
			*(ourWriteBuffer+2) = 0;
			*(ourWriteBuffer+3) = 0x80;
			*(ourWriteBuffer+4) = 0x00;
			ourUart_.write_buffer(ourWriteBuffer, 5);
		} else {
			// go straight with a speed of 10 mm/s
			*ourWriteBuffer = 137;
			*(ourWriteBuffer+1) = 0;
			*(ourWriteBuffer+2) = 100;
			*(ourWriteBuffer+3) = 0x80;
			*(ourWriteBuffer+4) = 0x00;
			ourUart_.write_buffer(ourWriteBuffer, 5);
		}

		driving = !driving;
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
