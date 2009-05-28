#ifndef _CREATEROBOT_H_
#define _CREATEROBOT_H_

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


struct RobotState_t
{
	int16  distance;
	int16  angle;
	uint16 voltage;
	int16  current;
	uint16 batteryCharge;
	uint16 batteryCapacity;
	uint16 wallSignal;
	uint16 cliffLeftSignal;
	uint16 cliffFrontLeftSignal;
	uint16 cliffFrontRightSignal;
	uint16 cliffRightSignal;
	uint16 cargoBayAnalogSignal;
	int16  requestedVelocity;
	int16  requestedRadius;
	int16  requestedRightVelocity;
	int16  requestedLeftVelocity;
	uint8  bumpAndWheelDrop;
	uint8  wall;
	uint8  cliff;
	uint8  virtualWall;
	uint8  overcurrents;
	uint8  infrared;
	uint8  buttons;
	uint8  charginState;
	int8   batteryTemperature;
	uint8  cargoBayDigitalInputs;
	uint8  chargingSources;
	uint8  songNumber;
	uint8  songPlaying;
};

typedef struct RobotState_t	ROBOTSTATE;
typedef ROBOTSTATE*			PROBOTSTATE;
typedef const ROBOTSTATE*	PCROBOTSTATE;

// Values for cliff
#define CLIFF_LEFT			0x1
#define CLIFF_FRONT_LEFT	0x2
#define CLIFF_FRONT_RIGHT	0x4
#define CLIFF_RIGHT			0x8


class RobotHandler
{
	virtual onStateChanged(PCROBOTSTATE pState);
	virtual onChecksumError();
};


class Robot: public Uint8DataHandler
{
protected:
	Uart       *m_pUart;
	RobotState *m_pHandler;

public:
	Robot();
	bool initialize(Uart *pUart);
	void startDemo(int demo);
	void drive(uint16 velocity, uint16 radius);
	void driveDirect(uint16 leftVelocity, uint16 rightVelocity);
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

	virtual void handle_uint8_data(uint8 data);

	inline void setRobotHandler(RobotHandler *pHandler)	{ m_pHandler = pHandler; }
	inline RobotHandler* getRobotHandler()				{ return m_pHandler; }
};
#endif