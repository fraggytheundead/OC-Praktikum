/************************************************************************
 ** This file is part of the the iSense project.
 ** Copyright (C) 2006 coalesenses GmbH (http://www.coalesenses.com)
 ** ALL RIGHTS RESERVED.
 ************************************************************************/
#include <isense/application.h>
#include <isense/os.h>
#include <isense/dispatcher.h>
#include <isense/radio.h>
#include <isense/hardware_radio.h>
#include <isense/task.h>
#include <isense/timeout_handler.h>
#include <isense/isense.h>
#include <isense/uart.h>
#include <isense/gpio.h>
#include <isense/dispatcher.h>
#include <isense/time.h>
#include <isense/button_handler.h>
#include <isense/sleep_handler.h>
#include <isense/modules/core_module/core_module.h>
#include <isense/modules/pacemate_module/pacemate_module.h>
#include <isense/modules/environment_module/environment_module.h>
#include <isense/modules/environment_module/temp_sensor.h>
#include <isense/modules/environment_module/light_sensor.h>
#include <isense/modules/security_module/lis_accelerometer.h>
#include <isense/modules/security_module/pir_sensor2.h>
#include <isense/data_handlers.h>
#include <isense/util/util.h>

#define MILLISECONDS 1000
#define TASK_SET_LIGHT_THRESHOLD 1

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
#define TASK_START_COMMUNICATION				4
#define TASK_CONNECT_BLINK						5


//#define EM
#define SM
//----------------------------------------------------------------------------
/**
 */

using namespace isense;

class SmartHomeApp :
	public isense::Application
	,public isense::Receiver
	,public isense::Sender
	,public isense::Task
	,public isense::TimeoutHandler
	,public isense::SleepHandler
	,public ButtonHandler
#ifdef EM
	,public Int8DataHandler
	,public Uint32DataHandler
#endif
#ifdef SM
	,public BufferDataHandler
	,public SensorHandler2
#endif
{
public:
	SmartHomeApp(isense::Os& os);

	virtual ~SmartHomeApp() ;

	///From isense::Application
	virtual void boot (void) ;

	// boots the EnvironmentModule
	void boot_EM();

	//boots the SecurityModule
	void boot_SM();

	///From isense::SleepHandler
	virtual bool stand_by (void) ; 	// Memory held

	///From isense::SleepHandler
	virtual bool hibernate (void) ;  // Memory not held

	///From isense::SleepHandler
	virtual void wake_up (bool memory_held) ;

	///From isense::ButtonHandler
	virtual void button_down( uint8 button );

	// Starts the connected Device and sets the LED to on
	void startDevice();

	// Stops the connected Device and sets the LED to off
	void stopDevice();

	///From isense::Receiver
	virtual void receive (uint8 len, const uint8 * buf, uint16 src_addr, uint16 dest_addr, uint16 lqi, uint8 seq_no, uint8 interface) ;

	// Send a MSG to activate or deactivate devices
	void send(bool temp, bool light, bool acc, bool pir, bool on);

	///From isense::Sender
	virtual void confirm (uint8 state, uint8 tries, isense::Time time) ;

	///From isense::Task
	virtual void execute( void* userdata ) ;

	///From isense::TimeoutHandler
	virtual void timeout( void* userdata ) ;

#ifdef EM
	// inherited from Int8DataHandler, called by the temperature
	// sensor if the temperature threshold is exceeded
	void handle_int8_data( int8 value );

	// inherited from UInt32DataHandler, called by the light
	// sensor upon luminance changes
	void handle_uint32_data( uint32 value );
#endif
#ifdef SM
	// inherited from BufferDataHandler, called when
	// accelerometer data is available
	virtual void handle_buffer_data( BufferData* buf_data );

	// inherited from SensorHandler, called when a
	// passive infrared sensor event occurs
	virtual void handle_sensor(bool this_is_the_end, uint16 interrupt_count);
#endif
	void drive(int16 speed, int16 radius);

private:
	CoreModule *cm_;
#ifdef EM
	// pointer to the Environmental Board
	EnvironmentModule* em_;
#endif
#ifdef SM
	// pointer to the accelerometer
	LisAccelerometer* acc_;
	// pointer to the passive infrared (PIR) sensor
	PirSensor2* pir_;
#endif
	uint16 connectedTo;
	Time lastContact;
	bool connectLed;
	bool starting_communication;
};

//----------------------------------------------------------------------------
SmartHomeApp::
	SmartHomeApp(isense::Os& os)
	: isense::Application(os)
	,cm_(new CoreModule(os))
	{
#ifdef EM
	// create EnvironmentModule instance
	em_ = new EnvironmentModule(os);
#endif
#ifdef SM
	// create LisAccelerometer instance
	acc_ = new LisAccelerometer(os);
	// create PriSensor2 instance
	pir_ = new PirSensor2(os);
#endif

    boot_EM();
#ifdef EM
        if (em_->temp_sensor() != NULL)
			{
				// Der Temperatursensor sagt bescheid, wenn es w�rmer als 31 Grad oder k�lter als 29 Grad wird.
				em_->temp_sensor()->set_threshold(31,29);
            }
#endif

    boot_SM();
#ifdef SM
        if (acc_ != NULL)
			{
			    // Der Beschleunigungssensor sagt bescheid wenn er eine Ersch�tterung von 500mg feststellt
			    acc_->set_threshold(50);
            }
#endif
	}

//----------------------------------------------------------------------------
SmartHomeApp::
	~SmartHomeApp()

	{
	}

//----------------------------------------------------------------------------
void
	SmartHomeApp::
	boot(void)
	{
        // Die Applikation startet
        os_.debug("App::boot %h",os_.id());
        os_.allow_sleep(false);
		os_.allow_doze( false );

		os_.dispatcher().add_receiver(this);

		os_.radio().hardware_radio().set_channel(20);
		starting_communication = true;
        os_.add_timeout_in(Time(300), this, (int*) TASK_START_COMMUNICATION);

        cm_->led_off();
        connectLed = false;
        os_.add_timeout_in(Time(500), this, (int*) TASK_CONNECT_BLINK);
 	}

void
	SmartHomeApp::
	button_down( uint8 button )
	{
        // Einer der beiden Kn�pfe wurde gedr�ckt

	}
//----------------------------------------------------------------------------
void
	SmartHomeApp::
	execute( void* userdata )
	{
	switch((int) userdata) {
		case TASK_CONNECTION_TIMEDOUT:
			os_.debug("Connection Timed out");
			connectedTo = 0;
			starting_communication = true;
			cm_->led_off();
			os_.add_timeout_in(Time(300), this, (int*) TASK_START_COMMUNICATION);
			break;
		case TASK_PINGPONG:
			os_.debug("Ping Pong");
			// if the robot hasn't sent anything in 2 seconds we ask if it's still there
			if(os_.time().operator-(lastContact).operator>(Time(2000))) {
				os_.debug("Ping!");
				uint8 buf[] = {ROOMBA_REMOTE_PING};
				os_.radio().send(connectedTo, 1, buf, 0, NULL);
			}
			// if the remote can't be reached within 6 seconds, we consider the connection lost
			if(os_.time().operator-(lastContact).operator>(Time(6000))) {
				os_.debug("Ping Pong timed out");
				os_.add_task(this, (uint8*) TASK_CONNECTION_TIMEDOUT);
				break;
			}
			os_.add_timeout_in(Time(3000), this, (uint8*) TASK_PINGPONG);
			break;
		case TASK_START_COMMUNICATION:
			if(starting_communication) {
				uint8 buf[] = {ROOMBA_REMOTE_START_COMMUNICATION};
				os_.radio().send(0xffff, 1, buf, 0, NULL);
				os_.add_timeout_in(Time(1000), this, (int*) TASK_START_COMMUNICATION);
			}
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

#ifdef EM
		if ( (uint32)userdata == TASK_SET_LIGHT_THRESHOLD)
		{
			// Der Lichtsensor sagt bescheid wenn sich die Helligkeit um 60% ge�ndert hat
			em_->light_sensor()->enable_threshold_interrupt(true, 60);
		}
#endif
	}

void
	SmartHomeApp::
	timeout( void* userdata )
	{
		os_.add_task( this, userdata);
	}

//--------------------------------------------------------------
#ifdef EM
void
	SmartHomeApp::
	handle_uint32_data( uint32 value )
	{
		// Der Lichtsensor hat eine �nderung der Helligkeit festgestellt.
	}
#endif

#ifdef EM
void
	SmartHomeApp::
	handle_int8_data( int8 value )
	{
		// Der Temperatursensor hat festgestellt, dass der Schwellwert �ber oder unterschitten wurde
	}
#endif
//--------------------------------------------------------------
#ifdef SM
void
	SmartHomeApp::
	handle_buffer_data( BufferData* buf_data )
	{
		// Der Beschleunigungssensor hat eine Bewegung wahrgenommen
		acc_->set_mode(MODE_THRESHOLD);
		int16 xAchse = buf_data->buf[0];
		int16 yAchse = buf_data->buf[1];
//		int16 zAchse = buf_data->buf[2];
//		os_.debug("x: %i, y: %i", xAchse, yAchse);
		int16 speed_factor = 0;
		int16 radius_factor = 0;
		int16 speed;
		int16 radius;
		if(xAchse < 200 && xAchse > -200) {
//			os_.debug("speed_factor = 0");
			speed_factor = 0;
		} else if(xAchse >= 200) {
//			os_.debug("xAchse >= 200");
			speed_factor = (xAchse - 200);
			if(speed_factor > 600) {
//				os_.debug("xAchse > 600");
				speed_factor = 100;
			} else {
				speed_factor = (speed_factor * 100 / 600);
//				os_.debug("xAchse <= 600, speed_factor: %i", speed_factor);
			}
		} else {
//			os_.debug("xAchse <= -200");
			speed_factor = (xAchse + 200);
			if(speed_factor < -600) {
//				os_.debug("xAchse < -600, speed_factor = -100");
				speed_factor = -100;
			} else {
				speed_factor = (speed_factor * 100 / 600);
//				os_.debug("xAchse <= -600, speed_factor: %i", speed_factor);
			}
		}

		if(yAchse < 200 && yAchse > -200) {
//			os_.debug("radius = 0");
			radius_factor = 0;
		} else if(yAchse >= 200) {
//			os_.debug("yAchse >= 200");
			radius_factor = (yAchse - 200);
			if(radius_factor > 600) {
//				os_.debug("yAchse > 600");
				radius_factor = 100;
			} else {
				radius_factor = (radius_factor * 100 / 600);
//				os_.debug("yAchse <= 600, radius_factor: %i", radius_factor);
			}
		} else {
//			os_.debug("yAchse <= -200");
			radius_factor = (yAchse + 200);
			if(radius_factor < -600) {
//				os_.debug("yAchse < -600, radius_factor = -100");
				radius_factor = -100;
			} else {
				radius_factor = (radius_factor * 100 / 600);
//				os_.debug("yAchse <= -600, radius_factor: %i", radius_factor);
			}
		}

//		os_.debug("speed_factor: %i, radius_factor: %i", speed_factor, radius_factor);

		speed = (speed_factor * 500) / 100;

		if(radius_factor==0) {
			radius = 32768;
		} else if (radius_factor == 100){
			// auf der Stelle drehen
			radius = 0;
			speed = 250;
		} else if (radius_factor == -100) {
			radius = 0;
			speed = -250;
		} else if (radius_factor > 0){
			radius = 2000 - ((radius_factor * 2000) / 100);
		} else {
			radius = -2000 - ((radius_factor * 2000) / 100);
		}

		drive(speed, radius);
	}
#endif

#ifdef SM
void
	SmartHomeApp::
	handle_sensor(bool this_is_the_end, uint16 interrupt_count)
	{
		// Der PIR Sensor hat eine oder mehrere Bewegungen wahrgenommen
        // this_is_the_end == false Erste Bewegung
        //                 == false keine Bewegung mehr seit ein paar Millisekunden
	}
#endif
//----------------------------------------------------------------------------
void
	SmartHomeApp::
	receive (uint8 len, const uint8 * buf, uint16 src_addr, uint16 dest_addr, uint16 lqi, uint8 seq_no, uint8 interface)
	{
        // Der Sensorknoten hat eine Nachricht empfangen
		if(!connectedTo && dest_addr == os_.id()) {
			if(len > 0 && buf[0] == ROOMBA_REMOTE_PING ) {
				uint8 buffer[] = {ROOMBA_REMOTE_PONG};
				os_.radio().send(src_addr, 1, buffer, 0, NULL);
				connectedTo = src_addr;
				starting_communication = false;
				os_.add_timeout_in(Time(2000), this, (uint8*) TASK_PINGPONG);
				lastContact = os_.time();
			}
		}
		if(connectedTo == src_addr && len > 0 && buf[0] == ROOMBA_REMOTE_PING) {
			uint8 buffer[] = {ROOMBA_REMOTE_PONG};
			os_.radio().send(src_addr, 1, buffer, 0, NULL);
			lastContact = os_.time();
		}
		if(connectedTo == src_addr && len > 0 && buf[0] == ROOMBA_REMOTE_PONG) {
			lastContact = os_.time();
		}
	}

void
	SmartHomeApp::
	send(bool temp, bool light, bool acc, bool pir, bool on)
	{
        // Benutze diese Methode um eine Nachricht zu versenden
	}

//----------------------------------------------------------------------------
void
	SmartHomeApp::
	confirm (uint8 state, uint8 tries, isense::Time time)
	{
	}

//----------------------------------------------------------------------------

void
	SmartHomeApp::
	boot_EM()
	{
#ifdef EM
		// if allocation of EnvironmentModule was successful
		if (em_!=NULL)
		{
			if (em_->light_sensor() != NULL)
			{
				//--------- configure the light sensor ----------
				// set this application as the data handler called upon
				// luminance changes beyond the set percentage
				em_->light_sensor()->set_data_handler(this);
				// add a task to be called in a second. Then, the
				// threshold mode is set one second after boot, to give the
				// sensor time for its first conversion
				os().add_task_in(Time(1,0), this, (void*)TASK_SET_LIGHT_THRESHOLD);
			} else
				os().fatal("Could not allocate light sensor");


			if (em_->temp_sensor() != NULL)
			{
				//--------- configure the light sensor ----------
				// set the temperature threshold to 35�C and the hysteresis
				// value to 30�C
				em_->temp_sensor()->set_threshold(31,29);
				// set this application as the data handler called if
				// temperature threshold is exceed or falls back below
				// the hysteresis value
				em_->temp_sensor()->set_data_handler(this);
			} else
				os().fatal("Could not allocate temp sensor");

			// enable the Environmental Sensor Module,
			// including its sensors
			em_->enable(true);

			// register task to be called in a minute for periodic sensor readings
			//os().add_task_in(Time(60,0), this, (void*)TASK_READ_SENSORS);
		} else
			os().fatal("Could not allocate EnvironmentModule");
#endif
	}

void
	SmartHomeApp::
	boot_SM()
	{
#ifdef SM
		if ((acc_ != NULL) && (pir_!= NULL))
			{
			// ----- configure accelerometer ------
			// set threshold mode, i.e. the sensor will start to
			// deliver data after the set threshold was exceeded
			acc_->set_mode(MODE_THRESHOLD);

			// set the threshold to 5 mg
			acc_->set_threshold(500);

			// set this application as the sensor event handler
			// --> handle_buffer_data will be called if
			// sensor data is available
			acc_->set_handler(this);

			// switch accelerometer
			acc_->enable();

			// ----- configure PIR sensor -------------
			// set this application as the sensor event handler
			// --> handle_sensor will be called upon a PIR event
			pir_->set_sensor_handler(this);
			//set the PIR event duration to 2 secs
			pir_->set_pir_sensor_int_interval( 2000 );
			// switch on the PIR sensor
			pir_->enable();
		} else
			os().fatal("Could not allocate PIR or ACC sensors");
#endif
	}

//----------------------------------------------------------------------------
bool
	SmartHomeApp::
	stand_by (void)
	{
		os_.debug("App::sleep");
		return true;
	}

//----------------------------------------------------------------------------
bool
	SmartHomeApp::
	hibernate (void)
	{
		os_.debug("App::hibernate");
		return false;
	}

//----------------------------------------------------------------------------
void
	SmartHomeApp::
	wake_up (bool memory_held)
	{
		os_.debug("App::Wakeup");
	}

//----------------------------------------------------------------------------
/**
  */
isense::Application* application_factory(isense::Os& os)
{
	return new SmartHomeApp(os);
}

void
	SmartHomeApp::
	drive (int16 speed, int16 radius)
	{
		os_.debug("Speed: %i, Radius: %i", speed, radius);
		uint8 buf[] = { ROOMBA_REMOTE_DRIVE, (speed & 0xff00) >> 8, speed & 0xff,
				(radius & 0xff00) >> 8, radius & 0xff};
		os_.radio().send(connectedTo, 5, buf, 0, NULL);

	}

/*-----------------------------------------------------------------------
* Source  $Source: $
* Version $Revision: 1.24 $
* Date    $Date: 2006/10/19 12:37:49 $
*-----------------------------------------------------------------------
* $Log$
*-----------------------------------------------------------------------*/
