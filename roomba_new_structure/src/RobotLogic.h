/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/

/* @class RobotLogic
 * @author Alexander BÃ¶cken, Michael Brehler, Stephan Lohse, Tobias Witt
 * @brief Interface between GUI and Robot, also implements the swarm logic
 * @detailed This class implements the gather and spread function as well as some basic robot control.
 * To execute any task you need to call the doTask method
 */

/*
 * RobotLogic.h
 *
 *  Created on: Jun 4, 2009
 *      Author: Alexander Böcken, Michael Brehler, Stephan Lohse, Tobias Witt
 */

#ifndef __OCPROJ_ROBOTLOGIC_H
#define __OCPROJ_ROBOTLOGIC_H

#include <isense/os.h>
#include <isense/util/util.h>
//#include <isense/platforms/jennic/jennic_os.h>
#include <isense/util/pseudo_random_number_generator.h>
#include "Robot.h"
#include "Communication.h"
#include <isense/isense_memory.h>
#include <isense/protocols/routing/neighborhood_monitor.h>

using namespace isense;

class RobotLogic: public RobotHandler,
public isense::TimeoutHandler,
public isense::Task,
public MovementDoneHandler
{
public:
	RobotLogic(Os& os, Uart *pUart, Communication *pCommunication);
	virtual ~RobotLogic();

	/**
	 * This method receives all the tasks given to the robot and calls the specific functions
	 * for them.
	 * Example: uint16 temp[]={32768,200}; doTask("drive",2,temp)
	 * 			The robot drives straight with 200mm/s
	 * @param taskName  The name of the task to be called
	 * @param paramLength  The number of parameters the task uses
	 * @param parameters  The actual values of the parameters
	 */
	void doTask(const char* taskName, uint8 paramLength, const uint16 *parameters);

	/**
	 * This method gets called by the Communication and than calls a function in the Communication class
	 * which sends all available tasks back to the GUI.
	 *
	 */
	void getCapabilities();

	/**
	 * This gets called via Interrupt by the robot class if the status of the IO mode of the robot changes
	 * @param ioMode
	 */
	virtual void onIoModeChanged(uint8 ioMode);

	/**
	 * This gets called via Interrupt by the robot class if anything related to the power state of the robot changes
	 * @param pState  the struct containing all data related to the power state
	 */
	virtual void onPowerStateChanged(PCPOWERSTATE pState);

	/**
	 * This gets called via Interrupt by the robot class if the status of the cliff sensors changes.
	 * This includes cliffFrontLeft, cliffFrontLeftSignal, cliffFrontRight, cliffFrontRightSignal,
	 * cliffLeft, cliffLeftSignal, cliffRight, cliffRightSignal
	 * See roomba manual for more info.
	 * @param pState the struct containing the cliff sensors data
	 */
	virtual void onCliffStateChanged(PCCLIFFSTATE pState);

	/**
	 * This gets called via Interrupt by the robot class if anything related to the movement state changes.
	 * This includes angle, distance, leftVelocity, radius, rightVelocity, velocity.
	 * See roomba manual for more info
	 * @param pState the struct containing the movement data
	 */
	virtual void onMovementStateChanged(PCMOVEMENTSTATE pState);

	/**
	 * This gets called via Interrupt by the robot class if the status of the buttons changes
	 * @param buttons  The buttons of the robot
	 */
	virtual void onButtonChanged(uint8 buttons);

	/**
	 * This gets called via Interrupt by the robot class if the status of the wall sensor changes
	 * @param wall  The value of the wall sensor
	 * @param virtualWall  The value of the virtual wall sensor
	 */
	virtual void onWallSensorChanged(uint8 wall, uint8 virtualWall);

	/**
	 * This gets called via Interrupt by the robot class if the status of the bumper or wheel drops changes.
	 * @param bumpsAndWheelDrop  The state of the bumper and wheel drops
	 */
	virtual void onBumpAndWheelDrop(uint8 bumpsAndWheelDrop);

	/**
	 * This gets called via Interrupt by the robot class if the status of a song changes
	 * @param songNumber the current number of the song playing
	 * @param songPlaying the status of the song 0:stopped 1:playing
	 */
	virtual void onSongStateChanged(uint8 songNumber, uint8 songPlaying);

	///From isense::TimeoutHandler
	virtual void timeout(void *userdata);
	///From isense::Task
	virtual void execute(void *userdata);

	/**
	 * This gets called by the robot class if any movement is done. Used by the swarm functions.
	 */
	virtual void movementDone();

protected:
	/**
	 * This starts the behavior spread. While in spread mode the Robot will try to reach
	 * a maximum distance away from a centerID given by a link quality threshold.
	 * After getting below the threshold the robot will turn 180° and drive back until
	 * the threshold is reached again then stop.
	 * The actual logic is implemented via the timeout and task system.
	 * @param tempID  The centerID
	 * @param tempThreshold  The minimum threshold
	 */
	void spread(uint16 tempID,uint8 tempThreshold);

	/**
	 * This starts the behavior gather. While in gather mode the Robot will search around
	 * for a given centerID and tries to reach a certain threshold for the link quality.
	 * After reaching that threshold it will play a short sound and stop.
	 * The actual logic is implemented via the timeout and task system.
	 * @param tempID The centerID
	 * @param tempThreshold The threshold to reach
	 */
	void gather(uint16 tempID,uint8 tempThreshold);

	/**
	 * This is not implemented. All it does is get the link qualities of all neighbors.
	 * The actual logic is implemented via the timeout and task system.
	 */
	void randomDrive();

	/**
	 * This plays the first seconds of the Monkey Island opening music.
	 */
	void miTheme();
	Communication *m_pCommunication;
	Os& m_pOs;
	Robot m_Robot;
	NeighborhoodMonitor m_neighborhoodMonitor;
	const static int8 cSPREAD=1;
	const static int8 cGATHER=2;
	const static int8 cRANDOMDRIVE=3;
	const static int8 cMITHEME=4;
	int8 activeTask;
	uint16 centerID;
	int timeoutCounter;
	uint8 songNumber;
	bool taskBool;
	bool timeoutBool;
	bool noNeighborsDetected;
	bool centerFound;
	NeighborhoodMonitor::neighbor* neighbors;
	NeighborhoodMonitor::neighbor* neighborsCopy;
	uint16 linkQualityID[20];
	uint8 linkQualityArray[20];
	uint16 centerQualityID[20];
	uint8 centerQuality[20];
	uint8 centerConnected[20];
	uint8 centerTimeoutCounter[20];
	uint16 gatherSum;
	uint16 oldGatherSum;
	uint16 gatherDistance;
	const static uint8 maxTimeout=40;
	uint8 centerCounter;
	uint8 centerThreshold;
	uint8 hops;
	const static uint8 maxCenterCounter=7;
	uint8 bumpsAndWheeldrop;
	int turnAngle;
	int swarmState;
	const static int cNONE=0;
	const static int cTURNWAIT=1;
	const static int cBUMPSWAIT=2;
	const static int cGATHERDISTANCEWAIT=3;
	const static int cGATHERTURNWAIT=4;
	// TODO: Check if this is correct
	uint8 m_bumpsAndWheelDrop;

};

#endif /* __OCPROJ_ROBOTLOGIC_H */
