// Stream definitions
#define STREAM_UNKNOWN					255
#define STREAM_CHECKSUM					254
#define STREAM_BYTES_TO_READ			253
#define STREAM_HEADER					19

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
#define STREAM_FRONT_CLIFF_LEFT_SIGNAL	29	// 2 bytes
#define STREAM_FRONT_CLIFF_RIGHT_SIGNAL	30	// 2 bytes
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

#define SET_16BIT_VALUE(name, data) \
	{ \
		if(stream.distance == 0) \
		{ \
			name = data; \
			goto skipSetStateToUnknown; \
		} \
		else \
			name = (name << 8) | data; \
	}



Robot::Robot() :
	m_pUart(NULL),
	m_pHandler(NULL)
{
}


void Robot::handle_uint8_data(uint8 data)
{
	static ROBOTSTATE oldRobotState, robotState;
	static int        checksum;
	static int        bytesToRead = 0;
	static uint8      streamState = STREAM_UNKNOWN;

	switch(streamState)
	{
	case STREAM_UNKNOWN:
		if(data == STREAM_HEADER && bytesToRead == 0)
		{
			memset(&robotState, 0, sizeof(ROBOTSTATE));
			checksum     = 0;
			streamState  = STREAM_BYTES_TO_READ;
		}
		else
			streamState = data;
		return;	// do not decrement bytesToRead

	case STREAM_BYTES_TO_READ:
		bytesToRead = data;
		streamState = STREAM_UNKNOWN;
		return;	// do not decrement bytesToRead

	case STREAM_CHECKSUM:
		if(checksum + (int)data == 0)
		{
			if(memcmp(&robotState, &oldRobotState, sizeof(ROBOTSTATE)) != 0)
			{
				m_pHandler->onStateChanged(&robotState);
				memcpy(&oldRobotState, &robotState, sizeof(ROBOTSTATE));
			}
		}
		else
		{
			m_pHandler->onChecksumError();
			checksum = 0;
		}

		streamState = STREAM_UNKNOWN;
		return;	// do not decrement bytesToRead

	case STREAM_BUMP_WHEELDROP:
		state.bumpAndWheelDrop = data;
		break;

	case STREAM_WALL:
		state.wall  = data;
		break;

	case STREAM_CLIFF_LEFT:
		if(data)
			state.cliff |= CLIFF_LEFT;
		break;

	case STREAM_CLIFF_FRONT_LEFT:
		if(data)
			state.cliff |= CLIFF_FRONT_LEFT;
		break;

	case STREAM_CLIFF_FRONT_RIGHT:
		if(data)
			state.cliff |= CLIFF_FRONT_RIGHT;
		break;

	case STREAM_CLIFF_RIGHT:
		if(data)
			state.cliff |= CLIFF_RIGHT;
		break;

	case STREAM_VIRTUAL_WALL:
		state.virtualWall = data;
		break;

	case STREAM_OVERCURRENTS
		state.virtualWall = data;
		break;

	case STREAM_INFRARED:
		state.infrared = data;
		break;

	case STREAM_BUTTONS:
		state.buttons = data;
		break;

	case STREAM_DISTANCE:
		SET_16BIT_VALUE(stream.distance, data);
		break;

	case STREAM_ANGLE:
		SET_16BIT_VALUE(stream.angle, data);
		break;

	case STREAM_CHARGING_STATE:
		stream.charginState = data;
		break;

	case STREAM_VOLTAGE:
		SET_16BIT_VALUE(stream.voltage, data);
		break;

	case STREAM_CURRENT:
		SET_16BIT_VALUE(stream.current, data);
		break;

	case STREAM_BATTERY_TEMPERATURE:
		stream.batteryTemperature = data;
		break;

	case STREAM_BATTERY_CHARGE:
		SET_16BIT_VALUE(stream.batteryCharge, data);
		break;

	case STREAM_BATTERY_CAPACITY:
		SET_16BIT_VALUE(stream.batteryCapacity, data);
		break;

	case STREAM_WALL_SIGNAL:
		SET_16BIT_VALUE(stream.wallSignal, data);
		break;

	case STREAM_CLIFF_LEFT_SIGNAL:
		SET_16BIT_VALUE(stream.cliffLeftSignal, data);
		break;

	case STREAM_CLIFF_FRONT_LEFT_SIGNAL:
		SET_16BIT_VALUE(stream.cliffFrontLeftSignal, data);
		break;

	case STREAM_CLIFF_FRONT_RIGHT_SIGNAL:
		SET_16BIT_VALUE(stream.cliffFrontRightSignal, data);
		break;

	case STREAM_CLIFF_RIGHT_SIGNAL:
		SET_16BIT_VALUE(stream.cliffRightSignal, data);
		break;

	case STREAM_CARGO_BAY_DIGITAL_INPUTS:
		stream.cargoBayDigitalInputs = data;
		break;

	case STREAM_CARGO_BAY_ANALOG_SIGNAL:
		SET_16BIT_VALUE(stream.cargoBayAnalogSignal, data);
		break;

	case STREAM_CHARGIN_SOURCES:
		stream.charginSources = data;
		break;

	case STREAM_SONG_NUMBER:
		stream.songNumber = data;
		break;

	case STREAM_SONG_PLAYING:
		stream.songPlaying = data;
		break;

	case STREAM_REQUESTED_VELOCITY:
		SET_16BIT_VALUE(stream.requestedVelocity, data);
		break;

	case STREAM_REQUESTED_RADIUS:
		SET_16BIT_VALUE(stream.requestedRadius, data);
		break;

	case STREAM_REQUESTED_RIGHT_VELOCITY:
		SET_16BIT_VALUE(stream.requestedRightVelocity, data);
		break;

	case STREAM_REQUESTED_LEFT_VELOCITY:
		SET_16BIT_VALUE(stream.requestedLeftVelocity, data);
		break;
	}

	streamState = STREAM_UNKNOWN;

skipSetStateToUnknown;
	if(--bytesToRead == 0)
		streamState = STREAM_CHECKSUM;
}