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
 *      Author: Alexander Böcken, Michael Brehler, Stephan Lohse, Tobias Witt
 */

#include "Robot.h"

#define DEBUG_UINT8_DATA_HANDLER

// Stream definitions
#define STREAM_UNKNOWN					255
#define STREAM_CHECKSUM					254
#define STREAM_BYTES_TO_READ			253
#define STREAM_HEADER					19

// Task IDs
#define CREATEROBOT_TASK_INIT						1
#define CREATEROBOT_TASK_PROCESS_DATA				2
#define ROBOT_ACTION_STOP							3

struct RobotState_t
{
	uint8  bumpsAndWheelDrop;
	uint8  wall;
	uint8  cliffLeft;
	uint8  cliffFrontLeft;
	uint8  cliffFrontRight;
	uint8  cliffRight;
	uint8  virtualWall;
	uint8  overcurrents;
	uint16 unused;
	uint8  infrared;
	uint8  buttons;
	int16  distance;
	int16  angle;
	uint8  chargingState;
	uint16 voltage;
	int16  current;
	int8   batteryTemperature;
	uint16 batteryCharge;
	uint16 batteryCapacity;
	uint16 wallSignal;
	uint16 cliffLeftSignal;
	uint16 cliffFrontLeftSignal;
	uint16 cliffFrontRightSignal;
	uint16 cliffRightSignal;
	uint8  userDigitalInputs;
	uint16 userAnalogInputs;
	uint8  chargingSourcesAvailable;
	uint8  IOMode;
	uint8  songNumber;
	uint8  songPlaying;
	uint8  numberOfStreamPackets;
	int16  velocity;
	int16  radius;
	int16  rightVelocity;
	int16  leftVelocity;
} __attribute__((packed));

typedef struct RobotState_t	ROBOTSTATE;
typedef ROBOTSTATE*			PROBOTSTATE;
typedef const ROBOTSTATE*	PCROBOTSTATE;


Robot::Robot(Os& os) :
	m_pUart(NULL),
	m_pHandler(NULL),
	m_pOs(os),
	m_sensorBufferWriteIndex(0)
{
	memset(m_sensorBuffer, 0, SENSOR_BUFFER_SIZE);

	m_randOmat.srand((uint32) (m_pOs.time()).ms());

	lastAction = Time(0,0);
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
	// wait for the iCreate to boot up - 5 seconds should do the trick
	if(currentTime.sec() < 5)
	{
		Time initTime =  Time(5, 0);
//		m_pOs.debug("CreateRobot init: fuege timeout in %i Sekunden hinzu", initTime.sec());
		taskStruct *task = new taskStruct();
		(*task).id = CREATEROBOT_TASK_INIT;
		(*task).time = m_pOs.time();
		m_pOs.add_timeout_in(initTime, this, task);
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

	taskStruct *task = new taskStruct();
	(*task).id = CREATEROBOT_TASK_PROCESS_DATA;
	(*task).time = m_pOs.time();
	m_pOs.add_task(this, task);
//	m_pOs.debug("init 2 Ende");
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
void Robot::startDemo(int demo)
{
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
void Robot::drive(uint16 velocity, uint16 radius)
{
	drive_notime(velocity, radius);
	lastAction = m_pOs.time();
}

void Robot::drive_notime(uint16 velocity, uint16 radius)
{
	char buff[5];
	buff[0] = CMD_DRIVE;
	buff[1] = 0xff & (velocity >> 8);
	buff[2] = 0xff & velocity;
	buff[3] = 0xff & (radius >> 8);
	buff[4] = 0xff & radius;
	m_pUart->write_buffer(buff, 5);
}

void Robot::driveStraight(uint16 velocity) {
	driveStraight_notime(velocity);
	lastAction = m_pOs.time();
}

void Robot::driveStraight_notime(uint16 velocity) {
	char buff[5];
	buff[0] = CMD_DRIVE;
	buff[1] = 0xff & (velocity >> 8);
	buff[2] = 0xff & velocity;
	buff[3] = 0x80;
	buff[4] = 0x00;
	m_pUart->write_buffer(buff, 5);

	lastAction = m_pOs.time();
}

/**
 * Makes the Create Robot drive, turning the two wheels at the given speeds
 * @param leftVelocity speed of the left wheel. Valid values are between -500 mm/s and 500 mm/s
 * @param rightVelocity speed of the right wheel. Valid values are between -500 mm/s and 500 mm/s
 */
void Robot::driveDirect(uint16 leftVelocity, uint16 rightVelocity)
{
	driveDirect_notime(leftVelocity, rightVelocity);
	lastAction = m_pOs.time();
}

void Robot::driveDirect_notime(uint16 leftVelocity, uint16 rightVelocity)
{
	char buff[5];
	buff[0] = CMD_DRIVE_DIRECT;
	buff[1] = 0xff & (rightVelocity >> 8);
	buff[2] = 0xff & rightVelocity;
	buff[3] = 0xff & (leftVelocity >> 8);
	buff[4] = 0xff & leftVelocity;
	m_pUart->write_buffer(buff, 5);

	lastAction = m_pOs.time();
}

void Robot::setLeds(uint8 ledMask, uint8 powerLedColor, uint8 powerLedIntensity)
{
//	JennicOs::os_pointer()->debug("setLeds, ledMask:%i, LedColor: %i, LedIntensity: %i", ledMask, powerLedColor, powerLedIntensity);
	char buff[4];
	buff[0] = CMD_SET_LEDS;
	buff[1] = ledMask;
	buff[2] = powerLedColor;
	buff[3] = powerLedIntensity;
	m_pUart->write_buffer(buff, 4);
}

void Robot::setOutputs(uint8 mask)
{
	char buff[2];
	buff[0] = CMD_SET_OUTPUTS;
	buff[1] = mask;
	m_pUart->write_buffer(buff, 2);
}

void Robot::setLowSideDriver(uint8 mask)
{
	char buff[2];
	buff[0] = CMD_LOW_SIDE_DRIVER;
	buff[1] = mask;
	m_pUart->write_buffer(buff, 2);
}

void Robot::setSong(int slot, uint8 *pBuffer, uint8 len)
{
	char buff[3];
	buff[0] = CMD_SET_SONG;
	buff[1] = (char) slot;
	buff[2] = len;
	m_pUart->write_buffer(buff, 3);
	m_pUart->write_buffer((char*) pBuffer, 2*len);
}

void Robot::playSong(int slot)
{
	char buff[2];
	buff[0] = CMD_PLAY_SONG;
	buff[1] = (char) slot;
	m_pUart->write_buffer(buff, 2);
}

void Robot::requestPacket(int type)
{
	char buff[2];
	buff[0] = CMD_REQUEST_PACKET;
	buff[1] = (char) type;
	m_pUart->write_buffer(buff, 2);
}

void Robot::streamPackets(int numPackets, uint8 *pPackets)
{
	char buff[2];
	buff[0] = CMD_STREAM_PACKETS;
	buff[1] = (char) numPackets;
	m_pUart->write_buffer(buff, 2);
	m_pUart->write_buffer((char*) pPackets, numPackets);
}

void Robot::setStreamState(bool bPaused)
{
	char buff[2];
	buff[0] = CMD_SET_STREAM_STATE;
	buff[1] = bPaused?0:1;
	m_pUart->write_buffer(buff, 2);
}

void Robot::setScript(uint8 *pScript, uint8 len)
{
	char buff[2];
	buff[0] = CMD_SET_SCRIPT;
	buff[1] = len;
	m_pUart->write_buffer(buff, 2);
	m_pUart->write_buffer((char*) pScript, len);
}


void Robot::executeScript()
{
	m_pUart->put( CMD_EXECUTE_SCRIPT );
}

// TODO
uint8 Robot::getScript(uint8 *pScript, uint8 len)
{
	return 0;
}

void Robot::wait(uint8 time)
{
	char buff[2];
	buff[0] = CMD_WAIT;
	buff[1] = time;
	m_pUart->write_buffer(buff, 2);
}

void Robot::waitForDistance(int16 distance)
{
	char buff[3];
	buff[0] = CMD_WAIT_DISTANCE;
	buff[1] = (char) 0xff & (distance >> 8);
	buff[2] = (char) 0xff & distance;
	m_pUart->write_buffer(buff, 2);
}

void Robot::waitForAngle(int16 angle)
{
	char buff[3];
	buff[0] = CMD_WAIT_ANGLE;
	buff[1] = (char) 0xff & (angle >> 8);
	buff[2] = (char) 0xff & angle;
	m_pUart->write_buffer(buff, 2);
}

void Robot::waitForEvent(int event)
{
	char buff[2];
	buff[0] = CMD_WAIT_EVENT;
	buff[1] = (char) event;
	m_pUart->write_buffer(buff, 2);
}

void Robot::turn(int16 angle, uint8 randomComponent)
{
	// 206mm/s == 90° / s
	uint16 turnSpeed = 206;

	Time actionTime = m_pOs.time();
	lastAction = actionTime;


	if(randomComponent > 0)
	{
		int8 random = (int8) (m_randOmat.rand(randomComponent) & 0xff);
		angle += (int16) random;
	}

	uint16 turnDirection = 0x0001;
	if(angle < 0)
	{
		turnDirection = 0xffff;
		angle = angle * -1;
	}

	drive_notime(turnSpeed, turnDirection);

	taskStruct *task = new taskStruct();
	(*task).id = ROBOT_ACTION_STOP;
	(*task).time = actionTime;
	uint32 seconds = angle / 90;
	uint16 msecs = (1000 * angle / 90) - 1000 * seconds;
	Time turnTime =  Time(seconds, msecs);

//	m_pOs.debug("RobotLogic turn, angle: %i, seconds: %i, msecs: %i", angle, seconds, msecs);
//	m_pOs.debug("RobotLogic turn, taskID: %i, TaskTime: %i s %i ms", (*task).id, (*task).time.sec(), (*task).time.ms());
	m_pOs.add_timeout_in(turnTime, this,(void*) task);
//	m_pOs.debug("RobotLogic turn, taskadresse %x", task);
}

void Robot::driveDistance(uint16 speed, uint16 radius, uint16 distance)
{
	Time actionTime = m_pOs.time();
	lastAction = actionTime;

	taskStruct *task = new taskStruct();
	(*task).id = ROBOT_ACTION_STOP;
	(*task).time = actionTime;
	uint32 seconds = distance / speed;
	uint16 msecs = ((1000 * distance) / speed) - 1000 * seconds;
//	m_pOs.debug("RobotLogic driveDistance, distance: %i, speed: %i, seconds: %i, msecs: %i", distance, speed, seconds, msecs);
//	m_pOs.debug("RobotLogic turn, taskID: %i, TaskTime: %i s %i ms", (*task).id, (*task).time.sec(), (*task).time.ms());
	Time distanceTime =  Time(seconds, msecs);
	drive_notime(speed, radius);
	m_pOs.add_timeout_in(distanceTime, this, (void*) task);
//	m_pOs.debug("RobotLogic turn, taskadresse %x", task);
}

void Robot::driveStraightDistance(uint16 speed, uint16 distance)
{
	Time actionTime = m_pOs.time();
	lastAction = actionTime;

	taskStruct *task = new taskStruct();
	(*task).id = ROBOT_ACTION_STOP;
	(*task).time = actionTime;
	uint32 seconds = distance / speed;
	uint16 msecs = ((1000 * distance) / speed) - 1000 * seconds;
	Time distanceTime =  Time(seconds, msecs);
	driveStraight_notime(speed);
	m_pOs.add_timeout_in(distanceTime, this, (void*) task);
}

void Robot::turnInfinite(int16 turnVelocity)
{
	driveDirect(turnVelocity,-turnVelocity);
}

void Robot::stop()
{
	lastAction = m_pOs.time();
//	m_pOs.debug("RobotLogic stop");
	driveDirect(0,0);
}

void Robot::handle_uint8_data(uint8 data)
{
	if(m_sensorBufferWriteIndex >= SENSOR_BUFFER_SIZE)
		m_sensorBufferWriteIndex = 0;

	m_sensorBuffer[m_sensorBufferWriteIndex++] = data;
}

void Robot::timeout(void *userdata)
{
//	m_pOs.debug("CreateRobot timeout, ID: %i", *(uint8*) userdata);
	m_pOs.add_task(this, userdata);
}

void Robot::execute(void *userdata)
{
	taskStruct *task = (taskStruct*) userdata;
//	m_pOs.debug("robot exe, %i", (int) (*task).id);
	switch((*task).id)
	{
	case CREATEROBOT_TASK_PROCESS_DATA:
		executeSensorData();
		break;
	case CREATEROBOT_TASK_INIT:
//		m_pOs.debug("execute: Init");
		initPart2();
		break;
	case ROBOT_ACTION_STOP:
		if((*task).time.sec() == lastAction.sec() && (*task).time.ms() == lastAction.ms())
		{
			stop();
		}
		break;
	}

	delete (taskStruct*) userdata;
}

void Robot::executeSensorData() {
	processSensorData();
	requestPacket(STREAM_ALL);
	taskStruct *newtask = new taskStruct();
	(*newtask).id = CREATEROBOT_TASK_PROCESS_DATA;
	(*newtask).time = m_pOs.time();
	m_pOs.add_task_in(Time(SENSOR_INTERVAL), this, newtask);
}

void Robot::processSensorData()
{
	static uint8         ioMode;
	static POWERSTATE    powerState;
	static CLIFFSTATE    cliffState;
	static MOVEMENTSTATE movementState;
	static uint8         buttonState;
	static uint8         wallState;
	static uint8         virtualWallState;
	static uint8         bumpAndWheelState;
	static uint8         songNumber;
	static uint8         songPlaying;


	if(m_sensorBufferWriteIndex == sizeof(ROBOTSTATE))
	{
		PCROBOTSTATE pRobotState = (PCROBOTSTATE)m_sensorBuffer;

		// call the handlers
		if(ioMode != pRobotState->IOMode)
			m_pHandler->onIoModeChanged(ioMode = pRobotState->IOMode);

		if(powerState.batteryCapacity          != pRobotState->batteryCapacity          ||
		   powerState.batteryCharge            != pRobotState->batteryCharge            ||
		   powerState.batteryTemperature       != pRobotState->batteryTemperature       ||
		   powerState.chargingSourcesAvailable != pRobotState->chargingSourcesAvailable ||
		   powerState.chargingState            != pRobotState->chargingState            ||
		   powerState.current                  != pRobotState->current                  ||
		   powerState.voltage                  != pRobotState->voltage)
		{
			powerState.batteryCapacity          = pRobotState->batteryCapacity;
			powerState.batteryCharge            = pRobotState->batteryCharge;
			powerState.batteryTemperature       = pRobotState->batteryTemperature;
			powerState.chargingSourcesAvailable = pRobotState->chargingSourcesAvailable;
			powerState.chargingState            = pRobotState->chargingState;
			powerState.current                  = pRobotState->current;
			powerState.voltage                  = pRobotState->voltage;

			m_pHandler->onPowerStateChanged(&powerState);
		}

		if(cliffState.cliffFrontLeft        != pRobotState->cliffFrontLeft        ||
		   cliffState.cliffFrontLeftSignal  != pRobotState->cliffFrontLeftSignal  ||
		   cliffState.cliffFrontRight       != pRobotState->cliffFrontRight       ||
		   cliffState.cliffFrontRightSignal != pRobotState->cliffFrontRightSignal ||
		   cliffState.cliffLeft             != pRobotState->cliffLeft             ||
		   cliffState.cliffLeftSignal       != pRobotState->cliffLeftSignal       ||
		   cliffState.cliffRight            != pRobotState->cliffRight            ||
		   cliffState.cliffRightSignal      != pRobotState->cliffRightSignal)
		{
			cliffState.cliffFrontLeft        = pRobotState->cliffFrontLeft;
			cliffState.cliffFrontLeftSignal  = pRobotState->cliffFrontLeftSignal;
			cliffState.cliffFrontRight       = pRobotState->cliffFrontRight;
			cliffState.cliffFrontRightSignal = pRobotState->cliffFrontRightSignal;
			cliffState.cliffLeft             = pRobotState->cliffLeft;
			cliffState.cliffLeftSignal       = pRobotState->cliffLeftSignal;
			cliffState.cliffRight            = pRobotState->cliffRight;
			cliffState.cliffRightSignal      = pRobotState->cliffRightSignal;

			m_pHandler->onCliffStateChanged(&cliffState);
		}

		if(movementState.angle         != pRobotState->angle         ||
		   movementState.distance      != pRobotState->distance      ||
		   movementState.leftVelocity  != pRobotState->leftVelocity  ||
		   movementState.radius        != pRobotState->radius        ||
		   movementState.rightVelocity != pRobotState->rightVelocity ||
		   movementState.velocity      != pRobotState->velocity)
		{
			movementState.angle         = pRobotState->angle;
			movementState.distance      = pRobotState->distance;
			movementState.leftVelocity  = pRobotState->leftVelocity;
			movementState.radius        = pRobotState->radius;
			movementState.rightVelocity = pRobotState->rightVelocity;
			movementState.velocity      = pRobotState->velocity;

			m_pHandler->onMovementStateChanged(&movementState);
		}

		if(wallState != pRobotState->wall || virtualWallState != pRobotState->virtualWall)
		{
			wallState        = pRobotState->wall;
			virtualWallState = pRobotState->virtualWall;

			m_pHandler->onWallSensorChanged(wallState, virtualWallState);
		}

		if(bumpAndWheelState != pRobotState->bumpsAndWheelDrop)
			m_pHandler->onBumpAndWheelDrop(bumpAndWheelState = pRobotState->bumpsAndWheelDrop);

		if(buttonState != pRobotState->buttons)
			m_pHandler->onButtonChanged(buttonState = pRobotState->buttons);

		if(songNumber != pRobotState->songNumber || songPlaying != pRobotState->songPlaying)
		{
			songNumber  = pRobotState->songNumber;
			songPlaying = pRobotState->songPlaying;

			m_pHandler->onSongStateChanged(songNumber, songPlaying);
		}
	}

	m_sensorBufferWriteIndex = 0;
}

