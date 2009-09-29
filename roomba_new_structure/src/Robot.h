/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/


#ifndef __OCPROJ_CREATEROBOT_H
#define __OCPROJ_CREATEROBOT_H

#include <isense/timeout_handler.h>
#include <isense/task.h>
#include <isense/time.h>
#include <isense/uart.h>
#include <isense/platforms/jennic/jennic_os.h>
#include <isense/gpio.h>

using namespace isense;

// IRobot Create commands
#define CMD_START				128
#define CMD_SET_BAUT_RATE		129
#define CMD_ENTER_SAFE_MODE		131
#define CMD_ENTER_FULL_MODE		132
#define CMD_START_DEMO			136
#define CMD_DRIVE				137
#define CMD_DRIVE_DIRECT		145
#define CMD_SET_LEDS			139
#define CMD_SET_OUTPUTS			147
#define CMD_LOW_SIDE_DRIVER		138
#define CMD_SET_SONG			140
#define CMD_PLAY_SONG			141
#define CMD_REQUEST_PACKET		142
//#define CMD_QUERY_PACKET		149
#define CMD_STREAM_PACKETS		148
#define CMD_SET_STREAM_STATE	150
#define CMD_SET_SCRIPT			152
#define CMD_EXECUTE_SCRIPT		153
#define CMD_GET_SCRIPT			154
#define CMD_WAIT				155
#define CMD_WAIT_DISTANCE		156
#define CMD_WAIT_ANGLE			157
#define CMD_WAIT_EVENT			158

// wait events
#define WAIT_WHEEL_DROP			1
#define WAIT_FRONT_WHEEL_DROP	2
#define WAIT_LEFT_WHEEL_DROP	3
#define WAIT_RIGHT_WHEEL_DROP	4
#define WAIT_BUMP				5
#define WAIT_LEFT_BUMP			6
#define WAIT_RIGHT_BUMP			7
#define WAIT_VIRTUAL_WALL		8
#define WAIT_WALL				9
#define WAIT_CLIFF				10
#define WAIT_LEFT_CLIFF			11
#define WAIT_FRONT_LEFT_CLIFF	12
#define WAIT_FRONT_RIGHT_CLIFF	13
#define WAIT_RIGHT_CLIFF		14
#define WAIT_HOME_BASE			15
#define WAIT_ADVANCE_BUTTON		16
#define WAIT_PLAY_BUTTON		17
#define WAIT_DIGITAL_INPUT_0	18
#define WAIT_DIGITAL_INPUT_1	19
#define WAIT_DIGITAL_INPUT_2	20
#define WAIT_DIGITAL_INPUT_3	21
#define WAIT_PASSIVE_MOVE		22

// Packet Types
#define STREAM_ALL						6	// 52 byte
#define STREAM_BUMP_WHEELDROP			7	// 1 byte
#define STREAM_WALL						8	// 1 byte
#define STREAM_CLIFF_LEFT				9	// 1 byte
#define STREAM_CLIFF_FRONT_LEFT			10	// 1 byte
#define STREAM_CLIFF_FRONT_RIGHT		11	// 1 byte
#define STREAM_CLIFF_RIGHT				12	// 1 byte
#define STREAM_VIRTUAL_WALL				13	// 1 byte
#define STREAM_OVERCURRENTS				14	// 1 byte
#define STREAM_UNUSED_1					14	// 1 byte
#define STREAM_UNUSED_2					16	// 1 byte
#define STREAM_INFRARED					17	// 1 byte
#define STREAM_BUTTONS					18	// 1 byte
#define STREAM_DISTANCE					19	// 2 bytes
#define STREAM_ANGLE					20	// 2 bytes
#define STREAM_CHARGING_STATE			21	// 1 byte
#define STREAM_VOLTAGE					22	// 2 bytes
#define STREAM_CURRENT					23	// 2 bytes
#define STREAM_BATTERY_TEMPERATURE		24	// 1 byte
#define STREAM_BATTERY_CHARGE			25	// 2 bytes
#define STREAM_BATTERY_CAPACITY			26	// 2 bytes
#define STREAM_WALL_SIGNAL				27	// 2 bytes
#define STREAM_CLIFF_LEFT_SIGNAL		28	// 2 bytes
#define STREAM_CLIFF_FRONT_LEFT_SIGNAL	29	// 2 bytes
#define STREAM_CLIFF_FRONT_RIGHT_SIGNAL	30	// 2 bytes
#define STREAM_CLIFF_RIGHT_SIGNAL		31	// 2 bytes
#define STREAM_CARGO_BAY_DIGITAL_INPUTS	32	// 1 byte
#define STREAM_CARGO_BAY_ANALOG_SIGNAL	33	// 2 bytes
#define STREAM_CHARGING_SOURCES			34	// 1 byte
#define STREAM_OI_MODE					35	// 1 byte
#define STREAM_SONG_NUMBER				36	// 1 byte
#define STREAM_SONG_PLAYING				37	// 1 byte
#define STREAM_NUMBER_OF_STREAM_STREAMS	38	// 1 byte
#define STREAM_REQUESTED_VELOCITY		39	// 2 bytes
#define STREAM_REQUESTED_RADIUS			40	// 2 bytes
#define STREAM_REQUESTED_RIGHT_VELOCITY	41	// 2 bytes
#define STREAM_REQUESTED_LEFT_VELOCITY	42	// 2 bytes

typedef struct PowerState_t
{
	uint8  chargingState;
	uint16 voltage;
	int16  current;
	int8   batteryTemperature;
	uint16 batteryCharge;
	uint16 batteryCapacity;
	uint8  chargingSourcesAvailable;
} POWERSTATE, *PPOWERSTATE;

typedef const POWERSTATE*	PCPOWERSTATE;


typedef struct CliffState_t
{
	uint8  cliffLeft;
	uint8  cliffFrontLeft;
	uint8  cliffFrontRight;
	uint8  cliffRight;
	uint16 cliffLeftSignal;
	uint16 cliffFrontLeftSignal;
	uint16 cliffFrontRightSignal;
	uint16 cliffRightSignal;
} CLIFFSTATE, *PCLIFFSTATE;

typedef const CLIFFSTATE*	PCCLIFFSTATE;


typedef struct MovementState_t
{
	int16 distance;
	int16 angle;
	int16 velocity;
	int16 radius;
	int16 rightVelocity;
	int16 leftVelocity;
} MOVEMENTSTATE, *PMOVEMENTSTATE;

typedef const MOVEMENTSTATE*	PCMOVEMENTSTATE;

class RobotHandler
{
public:
	virtual void onIoModeChanged(uint8 ioMode) = 0;
	virtual void onPowerStateChanged(PCPOWERSTATE pState) = 0;
	virtual void onCliffStateChanged(PCCLIFFSTATE pState) = 0;
	virtual void onMovementStateChanged(PCMOVEMENTSTATE pState) = 0;
	virtual void onButtonChanged(uint8 buttons) = 0;
	virtual void onWallSensorChanged(uint8 wall, uint8 virtualWall) = 0;
	virtual void onBumpAndWheelDrop(uint8 bumpsAndWheelDrop) = 0;
	virtual void onSongStateChanged(uint8  songNumber, uint8  songPlaying) = 0;
};

#define SENSOR_BUFFER_SIZE	52	// in bytes
#define SENSOR_INTERVAL		66	// in milliseconds (equals approx. 15 times per second)

class MovementDoneHandler
{
public:
	virtual void movementDone();
};

// struct for tasks - contains the ID of the task to be executed
// and the time at which it was scheduled.
struct taskStruct {
	int id;
	Time time;
	MovementDoneHandler *doneHandler;
};


class Robot: public Uint8DataHandler,
	public isense::TimeoutHandler,
	public isense::Task
{

protected:
	Uart       *m_pUart;
	RobotHandler *m_pHandler;

private:
	Os& m_pOs;

protected:
	void initPart2();
	uint8         m_sensorBuffer[SENSOR_BUFFER_SIZE];
	int           m_sensorBufferWriteIndex;

	PseudoRandomNumberGenerator m_randOmat;
	Time lastAction;
	void executeSensorData();
	void processSensorData();

public:
	Robot(Os& os);
	virtual ~Robot();
	void initialize(Uart *pUart);
	void startDemo(int demo);
	void drive(uint16 velocity, uint16 radius);
	void drive_notime(uint16 velocity, uint16 radius);
	void driveStraight(uint16 velocity);
	void driveStraight_notime(uint16 velocity);
	void driveDirect(uint16 leftVelocity, uint16 rightVelocity);
	void driveDirect_notime(uint16 leftVelocity, uint16 rightVelocity);
	void setLeds(uint8 ledMask, uint8 powerLedColor, uint8 powerLedIntensity);
	void setOutputs(uint8 mask);
	void setLowSideDriver(uint8 mask);
	void setSong(int slot, uint8 *pBuffer, uint8 len);
	void playSong(int slot);
	void requestPacket(int type);
	void streamPackets(int numPackets, uint8 *pPackets);
	void setStreamState(bool bPaused);
	void setScript(uint8 *pScript, uint8 len);
	void executeScript();
	uint8 getScript(uint8 *pScript, uint8 len);
	void wait(uint8 time);// in 15ms units
	void waitForDistance(int16 distance);
	void waitForAngle(int16 angle);
	void waitForEvent(int event);
	void changeModeSafe();
	void changeModeFull();
	void turn(int16 angle, uint8 randomComponent, MovementDoneHandler *pDoneHandler);
	void turnInfinite(int16 direction);
	void stop();
	void driveDistance(uint16 speed, uint16 radius, uint16 distance, MovementDoneHandler *pDoneHandler);
	void driveStraightDistance(uint16 speed, uint16 distance, MovementDoneHandler *pDoneHandler);

	virtual void handle_uint8_data(uint8 data);
	///From isense::TimeoutHandler
	virtual void timeout(void *userdata);
	///From isense::Task
	virtual void execute(void *userdata);

	inline void setRobotHandler(RobotHandler *pHandler)	{ m_pHandler = pHandler; }
	inline RobotHandler* getRobotHandler()				{ return m_pHandler; }
};
#endif /* __OCPROJ_CREATEROBOT_H */
