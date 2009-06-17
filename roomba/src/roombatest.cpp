/************************************************************************
 ** This file is part of the the iSense project.
 ** Copyright (C) 2006 coalesenses GmbH (http://www.coalesenses.com)
 ** ALL RIGHTS RESERVED.
 ************************************************************************/
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
#include <isense/sleep_handler.h>
#include <isense/modules/pacemate_module/pacemate_module.h>
#include <isense/util/util.h>
#include <isense/protocols/routing/flooding.h>
#include "Communication.h"
#include "RobotLogic.h"

#define MILLISECONDS 1000

//----------------------------------------------------------------------------
/**
 */
static uint16 BROADCAST = 0;

using namespace isense;

class roombatest: public isense::Application,
		public isense::Receiver,
		public isense::Sender,
		public isense::Task,
		public isense::TimeoutHandler,
		public isense::SleepHandler,
		public isense::UartPacketHandler {
public:
	roombatest(isense::Os& os);

	virtual ~roombatest();

	///From isense::Application
	virtual void boot(void);

	///From isense::SleepHandler
	virtual bool stand_by(void); // Memory held

	///From isense::SleepHandler
	virtual bool hibernate(void); // Memory not held

	///From isense::SleepHandler
	virtual void wake_up(bool memory_held);

	///From isense::Receiver
	virtual void receive(uint8 len, const uint8 * buf, uint16 src_addr,
			uint16 dest_addr, uint16 lqi, uint8 seq_no, uint8 interface);

	///From isense::Sender
	virtual void confirm(uint8 state, uint8 tries, isense::Time time);

	///From isense::Task
	virtual void execute(void* userdata);

	///From isense::TimeoutHandler
	virtual void timeout(void* userdata);

	///From isense::UartPacketHandler
	virtual void handle_uart_packet(uint8 type, uint8* buf, uint8 length);

	RobotLogic* getRobotLogic();

	Communication* getCommunication();

	Flooding* getFlooding();

private:
	Flooding flooding;
	Communication m_comModule;
	Uart& ourUart_;
	RobotLogic m_robotLogic;
};

//----------------------------------------------------------------------------
roombatest::roombatest(isense::Os& os) :
	isense::Application(os),
	flooding(os),
//	m_skeleton(os),
	m_comModule(os),
	ourUart_(os_.uart(1)),
	m_robotLogic(&ourUart_, &m_comModule)
{
	os_.dispatcher().add_receiver(this);
	os_.uart(0).set_packet_handler(isense::Uart::MESSAGE_TYPE_CUSTOM_IN_1, this);
	os_.uart(0).enable_interrupt(true);
}

//----------------------------------------------------------------------------
roombatest::~roombatest()
{
}

//----------------------------------------------------------------------------
void roombatest::boot(void) {
	os_.allow_sleep(false);
//	os_.add_timeout_in(Time(5 * MILLISECONDS), this, NULL);
//    os_.set_log_mode(ISENSE_LOG_MODE_RADIO);
    os_.debug("Boot");
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
//	m_robotLogic.getCapabilities();
}

void roombatest::receive(uint8 len, const uint8 * buf, uint16 src_addr,
		uint16 dest_addr, uint16 lqi, uint8 seq_no, uint8 interface) {
	os_.debug("receive:");
	for (int i = 0; i < len; ++i) {
		os_.debug("buf[%d] = %d", i, buf[i]);
	}
	uint16 destination = (buf[1] << 8) | buf[2];
	if (destination == os_.id() || destination == BROADCAST) {
		m_comModule.decodeMessage(len, buf);
	}
}

void roombatest::confirm(uint8 state, uint8 tries, isense::Time time) {
}

void roombatest::timeout(void* userdata) {
	os_.add_task(this, NULL);
	os_.add_timeout_in(Time(MILLISECONDS), this, NULL);
}

void roombatest::handle_uart_packet(uint8 type, uint8* buf, uint8 length) {
	os_.debug("handle_uart_packet:");
	for (int i = 0; i < length; ++i) {
		os_.debug("  buf[%d] = %d", i, buf[i]);
	}
	os_.debug("");
	flooding.send(length, buf);

}

RobotLogic* roombatest::getRobotLogic()
{
	return &m_robotLogic;
}

Communication* roombatest::getCommunication()
{
	return &m_comModule;
}

Flooding* roombatest::getFlooding()
{
	return &flooding;
}

isense::Application* application_factory(isense::Os& os) {
	return new roombatest(os);
}
