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

	/**
	 * initializes the UART and starts communication between iSense and the Create Robot.
	 * Puts the Robot in SAFE mode.
	 */
	void initPart2();
	uint8         m_sensorBuffer[SENSOR_BUFFER_SIZE];
	int           m_sensorBufferWriteIndex;

	PseudoRandomNumberGenerator m_randOmat;
	Time lastAction;
	void executeSensorData();
	void processSensorData();

	/**
	 * Drives without resetting the lastAction variable. For internal use only.
	 * Long explanation: The commands turn, driveDistance and driveStraightDistance work by calculating the time that
	 * is needed to perform the action and then scheduling a task that calls stop at the time it is completed. However,
	 * if in the meantime another movement was started we don't want to stop that new movement, so we use lastAction
	 * to remember at what time the last movement command was scheduled.
	 * @param velocity the velocity in mm/s. Valid values are between -500 mm/s and 500 mm/s
	 * @param radius the radius of the turn. Valid values are between -2000 mm and 2000 mm.
	 *		Positive values mean turning to the left, positive values to the right.
	 */
	void drive_notime(uint16 velocity, uint16 radius);

	/**
	 * Drives in a straight line without resetting the lastAction variable. For internal use only.
	 * (see long explanation in drive_notime)
	 * @param velocity the velocity in mm/s. Valid values are between -500 mm/s and 500 mm/s
	 */
	void driveStraight_notime(uint16 velocity);

	/**
	* Makes the Create Robot drive, turning the two wheels at the given speeds without
	* resetting the lastAction variable. For internal use only. (see long explanation in drive_notime)
	* @param leftVelocity speed of the left wheel. Valid values are between -500 mm/s and 500 mm/s
	* @param rightVelocity speed of the right wheel. Valid values are between -500 mm/s and 500 mm/s
	*/
	void driveDirect_notime(uint16 leftVelocity, uint16 rightVelocity);

public:
	Robot(Os& os);
	virtual ~Robot();

	/**
	 * Checks if 5 seconds have passed since turning on and calls
	 * initPart2() or schedules it to be called in 5 seconds.
	 * @param pUart a pointer to the UART that is to be used
	 */
	void initialize(Uart *pUart);

	/**
	 * Starts a demo on the Create Robot.
	 * @param demo the demo to be started. Please refer to the Create Open Interface Manual for valid Demos.
	 */
	void startDemo(int demo);

	/**
	 * Makes the Create Robot drive
	 * @param velocity the velocity in mm/s. Valid values are between -500 mm/s and 500 mm/s
	 * @param radius the radius of the turn. Valid values are between -2000 mm and 2000 mm.
	 *		Positive values mean turning to the left, positive values to the right.
	 */
	void drive(uint16 velocity, uint16 radius);

	/**
	 * Makes the Create Robot drive in a straight line
	 * @param velocity the velocity in mm/s. Valid values are between -500 mm/s and 500 mm/s
	 */
	void driveStraight(uint16 velocity);

	/**
	 * Makes the Create Robot drive, turning the two wheels at the given speeds
	 * @param leftVelocity speed of the left wheel. Valid values are between -500 mm/s and 500 mm/s
	 * @param rightVelocity speed of the right wheel. Valid values are between -500 mm/s and 500 mm/s
	 */
	void driveDirect(uint16 leftVelocity, uint16 rightVelocity);

	/**
	 * Controls the LEDs of the iCreate.
	 * @param ledMask bitmask that controls the Play LED and the Advance LED. Set 0x02 for Play and 0x08 for Advance.
	 * @param powerLedColor controls the green and red share in the LED color. 0 is green and 255 is red, values inbetween mix the two.
	 * @param powerLedIntensity controls the brightness of the power LED, where 0 is off and 255 is full intensity.
	 */
	void setLeds(uint8 ledMask, uint8 powerLedColor, uint8 powerLedIntensity);

	/**
	 * controls the state of the 3 digital output pins on the 25 pin Cargo Bay Connector.
	 * @param mask bitmask that sets the digital output pins on high or low.
	 * 	0x01 for digital-out-0 (pin 19)
	 * 	0x02 for digital-out-1 (pin 7)
	 *	0x04 for digital-out-2 (pin 20)
	 */
	void setOutputs(uint8 mask);

	/**
	 * This command lets you control the three low side drivers. The state of each driver is
	 * specified by one bit in the data byte.
	 * @param mask bitmask that turns the low side drivers on or off.
	 * 	0x01 for Low Side Driver 0 (pin 23)
	 * 	0x02 for Low Side Driver 1 (pin 22)
	 *	0x04 for Low Side Driver 2 (pin 24)
	 */
	void setLowSideDriver(uint8 mask);

	/**
	 * Writes a song of up to 16 notes to the given slot of the iRobot. Please refer to the Create Open Interface Manual
	 * for instructions on how to specify notes.
	 * @param slot the slot (or song number) in which the song is to be stored, valid slots are 0-15.
	 * @param pBuffer the notes and duration of the song. The first byte is a note, the second byte the
	 * 	duration (in 1/64ths of a second) of that note. Please refer to the Create Open Interface Manual for details.
	 * @param len the length of the song. (NOT the size of the array, but rather half the size of the array)
	 */
	void setSong(int slot, uint8 *pBuffer, uint8 len);

	/**
	 * Plays the song that is stored in the given slot
	 * @param slot the slot (or song number) of the song that should be played.
	 */
	void playSong(int slot);

	/**
	 * Requests the iRobot to send back the requested sensor data. (Which will be handled by processSensorData() which in turn
	 * calls the associated handlers. So without serious modification of this class, you won't be able to directly get sensor
	 * data)
	 * @param packetId the packet ID of the requested sensor data. Valid values are 0-42, please refer to the Create Open
	 * Interface Manual for details
	 */
	void requestPacket(int packetId);

	/**
	 * This command starts a continuous stream of data packets.
	 * The list of packets requested is sent every 15 ms, which is the rate iRobot Create uses to update data. As it turns out that
	 * doesn't seem to work all that well over the serial connection. Also it interferes with the current implementation that polls
	 * the iCreate in the interval specified in SENSOR_INTERVAL. So streaming is not recommended!
	 * @param numPackets the number of packets that are requested
	 * @param pPackets list of packets that are to be streamed. Please refer to the Create Open
	 * Interface Manual for packet IDs
	 */
	void streamPackets(int numPackets, uint8 *pPackets);

	/**
	 * Pauses or unpauses the streaming of packets
	 * @param bPaused stream is paused if true, stream is active if false.
	 */
	void setStreamState(bool bPaused);

	/**
	 * Writes a script to the iCreate. A script consists of a series of opcodes and arguments to the opcodes. Please refer
	 * to the Create Open Interface Manual for opcodes.
	 * @param pScript a list of opcodes and arguments. Can be up to 100 bytes long
	 * @param len size of pScript
	 */
	void setScript(uint8 *pScript, uint8 len);

	/**
	 * Executes the script that has last been written by setScript.
	 */
	void executeScript();

	/**
	 * Causes the iCreate to wait for the specified time. During the wait iCreate will not react to ANY inputs.
	 * @param time wait time in 1/10ths of a second. However, the iCreate has a timer resolution of 15ms only.
	 */
	void wait(uint8 time);

	/**
	 * Causes the iCreate to wait until it has traveled the specified distance.
	 * During the wait iCreate will not react to ANY inputs.
	 * @param distance wait distance in Millimeters.
	 */
	void waitForDistance(int16 distance);

	/**
	 * This command causes Create to wait until it has rotated through specified angle in degrees. When Create turns
	 * counterclockwise, the angle is incremented. When Create turns clockwise, the angle is decremented.
	 * During the wait iCreate will not react to ANY inputs.
	 * @param angle wait angle in degrees.
	 */
	void waitForAngle(int16 angle);

	/**
	 * This command causes Create to wait until it detects the specified event. During the wait
	 * iCreate will not react to ANY inputs.
	 * @param event the event to wait for. Send the negative event number to
	 * wait for the inverse event (like the release of a bumper). Please refer to the Create Open Interface Manual
	 * for event codes.
	 */
	void waitForEvent(int event);

	/**
	 * Puts the iCreate into Safe mode, enabling user control of Create. It turns off all LEDs.
	 *
	 */
	void changeModeSafe();

	/**
	 * Puts the iCreate in Full mode, turning off all safety measures of the iCreate (you'll have to make sure
	 * yourself, that it doesn't tumble down the stairs)
	 */
	void changeModeFull();

	/**
	 * Turns the iCreate through the specified angle.
	 * @param angle angle through which the iCreate is to be turned in degrees. Use positive angles for
	 * counter-clockwise turns and negative angles for clockwise turns.
	 * @param randomComponent specifies a range of degrees by which the actual turn can deviate from the angle. The
	 * idea is to avoid getting stuck in corners because of being caught in a loop of the same movements over and over.
	 * A randomComponent of 30 means the turn can deviate by 15 degrees to the left or the right.
	 * @param pDoneHandler handler that is called when the turn is finished.
	 */
	void turn(int16 angle, uint8 randomComponent, MovementDoneHandler *pDoneHandler);

	/**
	 * Turns the iCreate until it either receives a new movement command or runs out of battery.
	 * @param turnVelocity the velocity of the turn, positive velocity causes a counter-clockwise turn.
	 */
	void turnInfinite(int16 turnVelocity);

	/**
	 * Stops all movement of the iCreate.
	 */
	void stop();

	/**
	 * Stops all movement of the iCreate.
	 */
	void stop_notime();

	/**
	 * Makes the Create Robot drive a specified distance
	 * @param velocity the velocity in mm/s. Valid values are between -500 mm/s and 500 mm/s
	 * @param radius the radius of the turn. Valid values are between -2000 mm and 2000 mm.
	 *		Positive values mean turning to the left, positive values to the right.
	 * @param distance the distance before the iCreate stops in Millimeters.
	 * @param pDoneHandler handler that is called when the turn is finished.
	 */
	void driveDistance(uint16 speed, uint16 radius, uint16 distance, MovementDoneHandler *pDoneHandler);

	/**
	 * Makes the Create Robot drive a specified distance in a straight line
	 * @param velocity the velocity in mm/s. Valid values are between -500 mm/s and 500 mm/s
	 * @param distance the distance before the iCreate stops in Millimeters.
	 * @param pDoneHandler handler that is called when the turn is finished.
	 */
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
