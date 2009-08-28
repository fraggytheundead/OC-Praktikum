/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/

#include "roombatest.h"

//----------------------------------------------------------------------------
/**
 */


//----------------------------------------------------------------------------
roombatest::roombatest(isense::Os& os) :
	isense::Application(os),
	ourUart_(os_.uart(1)),
	m_robot(os)
	,cm_(new CoreModule(os))
{
	os_.dispatcher().add_receiver(this);
	m_robot.initialize(&ourUart_);
	m_robot.setRobotHandler(this);
	connectCandidate = 0;
	connectedTo = 0;
}

//----------------------------------------------------------------------------
roombatest::~roombatest()
{
}

//----------------------------------------------------------------------------
void roombatest::boot(void) {
	os_.allow_sleep(false);
//	os_.add_timeout_in(Time(2 * MILLISECONDS), this, NULL);
    os_.set_log_mode(ISENSE_LOG_MODE_RADIO);
	os_.radio().hardware_radio().set_channel(20);
    os_.debug("Boot");
//    m_robotLogic.getCapabilities();
//    uint16 temp[] = {0x1bfc,70};
//    m_robotLogic.doTask("spread",2,temp);
    cm_->led_off();
    connectLed = false;
    os_.add_timeout_in(Time(500), this, (int*) TASK_CONNECT_BLINK);
}

//----------------------------------------------------------------------------
bool roombatest::stand_by(void) {
	return true;
}

//----------------------------------------------------------------------------
bool roombatest::hibernate(void) {
	return false;
}

//----------------------------------------------------------------------------
void roombatest::wake_up(bool memory_held) {
}

void roombatest::execute(void* userdata) {
//	os_.debug("exe");
	switch((int) userdata) {
	case TASK_CONNECTCAND_TIMEOUT:
		connectCandidate = 0;
		break;
	case TASK_CONNECTION_TIMEDOUT:
//		os_.debug("Connection timed out");
		connectedTo = 0;
		m_robot.setLeds(8,16,255);
		break;
	case TASK_PINGPONG:
//		os_.debug("Ping Pong");
		// if the remote hasn't sent anything in 2 seconds we ask if it's still there
		if(os_.time().operator-(lastContact).operator>(Time(2000))) {
//			os_.debug("Ping!");
			uint8 buffer[] = {ROOMBA_REMOTE_PING};
			os_.radio().send(connectedTo, 1, buffer, 0, NULL);
		}
		// if the remote can't be reached within 6 seconds, we consider the connection lost
		if(os_.time().operator-(lastContact).operator>(Time(6000))) {
//			os_.debug("Ping Pong timed out");
			os_.add_task(this, (uint8*) TASK_CONNECTION_TIMEDOUT);
			m_robot.stop();
			// end the PingPong Task
			break;
		}
		os_.add_timeout_in(Time(3000), this, (uint8*) TASK_PINGPONG);
		break;
	case TASK_CONNECT_BLINK:
		if(connectedTo) {
			connectLed = true;
			cm_->led_on();
		} else {
			if(connectLed) {
				connectLed = false;
				cm_->led_off();
			} else {
				connectLed = true;
				cm_->led_on();
			}
		}
		os_.add_timeout_in(Time(500), this, (int*) TASK_CONNECT_BLINK);
		break;
	default:
		//nothing
		break;
	}
}

void roombatest::receive(uint8 len, const uint8 * buf, uint16 src_addr,
		uint16 dest_addr, uint16 lqi, uint8 seq_no, uint8 interface)
{
//	os_.debug("receive, len: %i, src: %h, dest: %h", len, src_addr, dest_addr);
	if(len > 0) {
//		os_.debug("receive, buf[0]: %i", buf[0]);
	}
	if(!connectedTo && dest_addr == 0xffff) {
		if(len > 0 && buf[0] == ROOMBA_REMOTE_START_COMMUNICATION && connectCandidate == 0) {
			uint8 buffer[] = {ROOMBA_REMOTE_PING};
			os_.radio().send(src_addr, 1, buffer, 0, NULL);
			connectCandidate = src_addr;
			os_.add_timeout_in(Time(2000), this, (uint8*) TASK_CONNECTCAND_TIMEOUT);
		}
	}
	if(!connectedTo && dest_addr == os_.id() && connectCandidate == src_addr) {
		if(len > 0 && buf[0] == ROOMBA_REMOTE_PONG) {
			connectCandidate = 0;
			connectedTo = src_addr;
			m_robot.setLeds(8,16,255);
			os_.add_timeout_in(Time(3000), this, (uint8*) TASK_PINGPONG);
			os_.debug("connected to %h", connectedTo);
		}
	}
	if(connectedTo == src_addr && dest_addr == os_.id()) {
		interpretMessage(len, buf);
	}

}

void roombatest::interpretMessage(uint8 len, const uint8 * buf)
{
	lastContact = os_.time();

	if(len > 0 && buf[0] == ROOMBA_REMOTE_PING)
	{
		uint8 buffer[] = {ROOMBA_REMOTE_PONG};
		os_.radio().send(connectedTo, 1, buffer , 0, NULL);
	}
	if(len >= 5 && buf[0] == ROOMBA_REMOTE_DRIVE)
	{
		// we will only drive, if we haven't bumped into anything lately
		if(!bumped) {
			os_.debug("Drive speed: %i, radius: %i", buf[1] << 8 | buf[2], buf[3] << 8 | buf[4]);
			m_robot.drive(buf[1] << 8 | buf[2], buf[3] << 8 | buf[4]);
		} else {
			// having the roomba stop is the way to reset bumped
			if(!(buf[1] << 8 | buf[2])) {
				bumped = false;
			}
		}
		uint8 buffer[] = {ROOMBA_REMOTE_PONG};
		os_.radio().send(connectedTo, 1, buffer, 0, NULL);
	}

}

void roombatest::confirm(uint8 state, uint8 tries, isense::Time time)
{

}

void roombatest::timeout(void* userdata) {
//	os_.debug("timeout");
	os_.add_task(this, userdata);
}

isense::Application* application_factory(isense::Os& os) {
	return new roombatest(os);
}



void roombatest::onIoModeChanged(uint8 ioMode)
{

}

void roombatest::onPowerStateChanged(PCPOWERSTATE pState)
{

}

void roombatest::onCliffStateChanged(PCCLIFFSTATE pState)
{

}

void roombatest::onMovementStateChanged(PCMOVEMENTSTATE pState)
{

}

void roombatest::onButtonChanged(uint8 buttons)
{

}

void roombatest::onWallSensorChanged(uint8 wall, uint8 virtualWall)
{

}

void roombatest::onBumpAndWheelDrop(uint8 bumpsAndWheelDrop)
{
	// oops, ran into something
	if(bumpsAndWheelDrop & 0x03) {
		m_robot.driveDistance(-200, 32768, 200);
		bumped = true;
	}
}

void roombatest::onSongStateChanged(uint8 songNumber, uint8 songPlaying)
{

}
