/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/

/*
 * CreateRobot.cpp
 *
 *  Created on: May 25, 2009
 *      Author: Alexander Böcken, Stephan Lohse, Tobias Witt
 */

#include "CreateRobot.h"

#define DEBUG_UINT8_DATA_HANDLER

// Stream definitions
#define STREAM_UNKNOWN					255
#define STREAM_CHECKSUM					254
#define STREAM_BYTES_TO_READ			253
#define STREAM_HEADER					19

// Task IDs
#define CREATEROBOT_TASK_INIT						1

#define SET_16BIT_VALUE(name, data) \
	{ \
		if(name == 0) \
		{ \
			name = data; \
			goto skipSetStateToUnknown; \
		} \
		else \
			name = (name << 8) | data; \
	}

Robot::Robot(Os& os) :
	m_pUart(NULL),
	m_pHandler(NULL),
	m_pOs(os)
{
}

Robot::~Robot()
{

}

/**
 * initializes the UART and starts communication between iSense and the Create Robot.
 * Puts the Robot in SAFE mode
 * @param pUart a pointer to the UART that is to be used
 * @return true if the initialization was successful
 */
void Robot::initialize(Uart *pUart){
	m_pOs.debug("CreateRobot init");
	m_pUart = pUart;
	Time currentTime = m_pOs.time();
	// wait for the iCreate to boot up - 4 seconds should do the trick
	if(currentTime.sec() < 6)
	{
		static uint8 task = CREATEROBOT_TASK_INIT;
		Time initTime =  Time(6, 0);
//		m_pOs.debug("CreateRobot init: fuege timeout in %i Sekunden hinzu", initTime.sec());
		m_pOs.add_timeout_in(initTime, this, &task);
	}
	else
	{
		initPart2();
	}
}

void Robot::initPart2()
{
	// init UART
	if(!m_pUart->enabled()) m_pUart->enable();
	m_pUart->set_baudrate(57600);
	// 8 Databits, no flowcontrol, set Stoppbit to 1
	m_pUart->set_control( 8, 'N', 1 );
	// set data handler
	m_pUart->set_uint8_handler(this);
	m_pUart->enable_interrupt(true);
	// send roomba start command
	m_pUart->put( CMD_START );
	// turn power LED on (green, full intensity)
	setLeds(0, 0, 255);
	// put roomba into safe mode
	changeModeSafe();

	// request streaming all packets
//	uint8 buf[] = {STREAM_BUMP_WHEELDROP, STREAM_WALL,
//			STREAM_CLIFF_LEFT, STREAM_CLIFF_FRONT_LEFT,
//			STREAM_CLIFF_FRONT_RIGHT, STREAM_CLIFF_RIGHT,
//			STREAM_VIRTUAL_WALL, STREAM_INFRARED,
//			STREAM_BUTTONS, STREAM_DISTANCE,
//			STREAM_ANGLE, STREAM_CHARGING_STATE,
//			STREAM_VOLTAGE, STREAM_BATTERY_TEMPERATURE,
//			STREAM_BATTERY_CHARGE, STREAM_BATTERY_CAPACITY,
//			STREAM_WALL_SIGNAL, STREAM_CLIFF_LEFT_SIGNAL,
//			STREAM_CLIFF_FRONT_LEFT_SIGNAL, STREAM_CLIFF_FRONT_RIGHT_SIGNAL,
//			STREAM_CLIFF_RIGHT_SIGNAL, STREAM_CARGO_BAY_DIGITAL_INPUTS,
//			STREAM_CARGO_BAY_ANALOG_SIGNAL, STREAM_CHARGING_SOURCES,
//			STREAM_OI_MODE, STREAM_SONG_NUMBER,
//			STREAM_SONG_PLAYING, STREAM_NUMBER_OF_STREAM_STREAMS,
//			STREAM_REQUESTED_VELOCITY, STREAM_REQUESTED_RADIUS,
//			STREAM_REQUESTED_RIGHT_VELOCITY, STREAM_REQUESTED_LEFT_VELOCITY};
//	streamPackets(32, buf);

//	uint8 buf[] = {STREAM_BUMP_WHEELDROP, STREAM_BATTERY_TEMPERATURE, STREAM_BATTERY_CHARGE };
//	streamPackets(3, buf);
}

void Robot::changeModeSafe()
{
	m_pUart->put( CMD_ENTER_SAFE_MODE );
	// make the power LED yellow
	setLeds(0, 16, 255);
}

void Robot::changeModeFull()
{
	m_pUart->put( CMD_ENTER_FULL_MODE );
	// make the Power LED red
	setLeds(0, 255, 255);
}


/**
 * Starts a demo on the Create Robot
 * @param demo the demo to be started
 */
void Robot::startDemo(int demo){
	char buff[2];
	buff[0] = CMD_START_DEMO;
	buff[1] = (char) demo;
	m_pUart->write_buffer(buff, 2);
}

/**
 * Makes the Create Robot drive
 * @param velocity the velocity in mm/s. Valid values are between -500 mm/s and 500 mm/s
 * @param radius the radius of the turn. Valid values are between -2000 mm and 2000 mm.
 *		Positive values mean turning to the left, positive values to the right.
 */
/* TODO: evtl herausfinden, was passiert, wenn man hier
 * ungültige Werte verwendet (zB velocity > 500) und ggf abfangen
 * angeblich fängt der Roboter das von selbst ab
 */
void Robot::drive(uint16 velocity, uint16 radius){
	char buff[5];
	buff[0] = CMD_DRIVE;
	buff[1] = 0xff & (velocity >> 8);
	buff[2] = 0xff & velocity;
	buff[3] = 0xff & (radius >> 8);
	buff[4] = 0xff & radius;
	m_pUart->write_buffer(buff, 5);
}

/**
 * Makes the Create Robot drive, turning the two wheels at the given speeds
 * @param leftVelocity speed of the left wheel. Valid values are between -500 mm/s and 500 mm/s
 * @param rightVelocity speed of the right wheel. Valid values are between -500 mm/s and 500 mm/s
 */
void Robot::driveDirect(uint16 leftVelocity, uint16 rightVelocity){
	char buff[5];
	buff[0] = CMD_DRIVE_DIRECT;
	buff[1] = 0xff & (rightVelocity >> 8);
	buff[2] = 0xff & rightVelocity;
	buff[3] = 0xff & (leftVelocity >> 8);
	buff[4] = 0xff & leftVelocity;
	m_pUart->write_buffer(buff, 5);
}

void Robot::setLeds(uint8 ledMask, uint8 powerLedColor, uint8 powerLedIntensity){
//	JennicOs::os_pointer()->debug("setLeds, ledMask:%i, LedColor: %i, LedIntensity: %i", ledMask, powerLedColor, powerLedIntensity);
	char buff[4];
	buff[0] = CMD_SET_LEDS;
	buff[1] = ledMask;
	buff[2] = powerLedColor;
	buff[3] = powerLedIntensity;
	m_pUart->write_buffer(buff, 4);
}

void Robot::setOutputs(uint8 mask){
	char buff[2];
	buff[0] = CMD_SET_OUTPUTS;
	buff[1] = mask;
	m_pUart->write_buffer(buff, 2);
}

void Robot::setLowSideDriver(uint8 mask){
	char buff[2];
	buff[0] = CMD_LOW_SIDE_DRIVER;
	buff[1] = mask;
	m_pUart->write_buffer(buff, 2);
}

void Robot::setSong(int slot, uint8 *pBuffer, uint8 len){
	char buff[3];
	buff[0] = CMD_SET_SONG;
	buff[1] = (char) slot;
	buff[2] = len;
	m_pUart->write_buffer(buff, 3);
	m_pUart->write_buffer((char*) pBuffer, 2*len);
}

void Robot::playSong(int slot){
	char buff[2];
	buff[0] = CMD_PLAY_SONG;
	buff[1] = (char) slot;
	m_pUart->write_buffer(buff, 2);
}

void Robot::requestPacket(int type){
	char buff[2];
	buff[0] = CMD_REQUEST_PACKET;
	buff[1] = (char) type;
	m_pUart->write_buffer(buff, 2);
}

void Robot::streamPackets(int numPackets, uint8 *pPackets){
	char buff[2];
	buff[0] = CMD_STREAM_PACKETS;
	buff[1] = (char) numPackets;
	m_pUart->write_buffer(buff, 2);
	m_pUart->write_buffer((char*) pPackets, numPackets);
}

void Robot::setStreamState(bool bPaused){
	char buff[2];
	buff[0] = CMD_SET_STREAM_STATE;
	buff[1] = bPaused?0:1;
	m_pUart->write_buffer(buff, 2);
}

void Robot::setScript(uint8 *pScript, uint8 len){
	char buff[2];
	buff[0] = CMD_SET_SCRIPT;
	buff[1] = len;
	m_pUart->write_buffer(buff, 2);
	m_pUart->write_buffer((char*) pScript, len);
}


void Robot::executeScript(){
	m_pUart->put( CMD_EXECUTE_SCRIPT );
}

// TODO
uint8 Robot::getScript(uint8 *pScript, uint8 len){
	return 0;
}

void Robot::wait(uint8 time){
	char buff[2];
	buff[0] = CMD_WAIT;
	buff[1] = time;
	m_pUart->write_buffer(buff, 2);
}

	// in 15ms units
void Robot::waitForDistance(int16 distance){
	char buff[3];
	buff[0] = CMD_WAIT_DISTANCE;
	buff[1] = (char) 0xff & (distance >> 8);
	buff[2] = (char) 0xff & distance;
	m_pUart->write_buffer(buff, 2);
}

void Robot::waitForAngle(int16 angle){
	char buff[3];
	buff[0] = CMD_WAIT_ANGLE;
	buff[1] = (char) 0xff & (angle >> 8);
	buff[2] = (char) 0xff & angle;
	m_pUart->write_buffer(buff, 2);
}

void Robot::waitForEvent(int event){
	char buff[2];
	buff[0] = CMD_WAIT_EVENT;
	buff[1] = (char) event;
	m_pUart->write_buffer(buff, 2);
}

void Robot::handle_uint8_data(uint8 data)
{

}

void Robot::timeout(void *userdata)
{
//	m_pOs.debug("CreateRobot timeout, ID: %i", *(uint8*) userdata);
	m_pOs.add_task(this, userdata);
}

void Robot::execute(void *userdata)
{
	uint8 taskID = *(uint8*) userdata;
//	m_pOs.debug("execute, taskID: %i", taskID);
	if(taskID == CREATEROBOT_TASK_INIT)
	{
		initPart2();
	}
}

