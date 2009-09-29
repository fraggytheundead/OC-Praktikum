/* ----------------------------------------------------------------------
 * This file is part of the WISEBED project.
 * Copyright (C) 2009 by the Institute of Telematics, University of Luebeck
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the BSD License. Refer to bsd-licence.txt
 * file in the root of the source tree for further details.
------------------------------------------------------------------------*/

/*
 * RobotLogic.cpp
 *
 *  Created on: Jun 4, 2009
 *      Author: Alexander Böcken, Michael Brehler, Stephan Lohse, Tobias Witt
 */

#include "RobotLogic.h"
#include "roombatest.h"

#define DEBUG_GET_CAPABILITIES
//#define DEBUG_DOTASK

RobotLogic::RobotLogic(Os& os, Uart *pUart, Communication *pCommunication) :
	m_pOs(os),
	m_Robot(os),
	m_neighborhoodMonitor(os, 1800),
	neighbors(NULL)
{
	m_Robot.initialize(pUart);
	m_Robot.setRobotHandler(this);

	int c=0;
	for (c=0; c<20; c++)
	{
		centerQualityID[c]=BROADCAST;
	}

	m_pCommunication = pCommunication;

	m_pOs.add_timeout_in(Time(MILLISECONDS), this, NULL);

	noNeighborsDetected = false;

	bumpsAndWheeldrop = 0;
}

RobotLogic::~RobotLogic()
{
	// TODO Auto-generated destructor stub
}

void RobotLogic::doTask(const char* taskName, uint8 paramLength, const uint16 *parameters)
{
#ifdef DEBUG_DOTASK
	m_pOs.debug("doTask STRING, ID: %s, paramLength: %i", taskName, paramLength);
#endif
	if(strcmp(taskName, "drive") == 0)
	{
		if(paramLength == 2)
		{
#ifdef DEBUG_DOTASK
			m_pOs.debug("doTask: drive  Param0:%i  Param1:%i",parameters[0],parameters[1]);
#endif
			m_Robot.drive((uint16) parameters[0], (uint16) parameters[1]);
		}
	}

	if(strcmp(taskName, "turn") == 0)
	{
		if(paramLength == 2)
		{
			activeTask=-1;
#ifdef DEBUG_DOTASK
			m_pOs.debug("doTask: turnParam0:%i  Param1:%i",parameters[0],parameters[1]);
#endif
			m_Robot.turn((int16) parameters[0], (uint8) (parameters[1] & 0xff), NULL);
		}
	}

	if(strcmp(taskName, "turnInfinite") == 0)
	{
		if(paramLength == 1)
		{
			activeTask=-1;
#ifdef DEBUG_DOTASK
			m_pOs.debug("doTask: turnInfinite");
			turnInfinite((int16) parameters[0]);
#endif
		}
	}

	if(strcmp(taskName, "stop") == 0)
	{
		activeTask=-1;
#ifdef DEBUG_DOTASK
		m_pOs.debug("doTask: stop");
#endif
		m_Robot.stop();
	}

	if(strcmp(taskName, "driveDistance") == 0)
	{
		if(paramLength == 3)
		{
			activeTask=-1;
#ifdef DEBUG_DOTASK
			m_pOs.debug("doTask: drveDist  Param0: %i  Param1: %i Param2: %i",parameters[0],parameters[1],parameters[2]);
#endif
			m_Robot.driveDistance((uint16) parameters[0], (uint16) parameters[1], (uint16) parameters[2], NULL);

		}
	}

	if (strcmp(taskName, "spread") == 0)
	{
#ifdef DEBUG_DOTASK
		m_pOs.debug("doTask: spread");
#endif
		spread(parameters[0],parameters[1]);
	}

	if (strcmp(taskName, "gather") == 0)
	{
		gather(parameters[0],parameters[1]);
	}

	if (strcmp(taskName, "randomDrive") == 0)
	{
		randomDrive();
	}

	if (strcmp(taskName, "patapatapata") == 0)
	{
		//m_pOs.debug("patapatapata erhalten in ID: %i",m_pOs.id());
		m_pCommunication->sendMessage(m_pOs.id(),BROADCAST,"pon",0,NULL);
	}

	if (strcmp(taskName, "pon") == 0)
	{
		//m_pCommunication->sendMessage(0,"pon",0,NULL);
		m_pOs.debug("pon erhalten in ID: %i",m_pOs.id());
	}

	if (strcmp(taskName, "cquality") == 0)
	{
		//m_pOs.debug("Centerquality Msgtest von %x ist %i %i",parameters[0],parameters[1],parameters[2]);
		uint16 tempID;
		tempID=parameters[0];
		int i=0;
		for (i=0; i<20; i++) {
			if (centerQualityID[i]==tempID)
			{
				//m_pOs.debug("Centerquality[%i] von %x ist %i %i",i,tempID,parameters[1],parameters[2]);
				centerConnected[i]=parameters[1];
				centerQuality[i]=parameters[2];
				centerTimeoutCounter[i]=0;
				return;
			}
			if (centerQualityID[i]==BROADCAST)
			{
				centerQualityID[i]=tempID;
				centerConnected[i]=parameters[1];
				centerQuality[i]=parameters[2];
				//m_pOs.debug("Centerquality[%i] von %x ist %i %i ",i,tempID,parameters[1],parameters[2]);
				break;
			}
		}
	}

	if (strcmp(taskName, "usedemo") == 0)
	{
		activeTask=-1;
		m_Robot.startDemo(parameters[0]);
	}

	if(strcmp(taskName, "mitheme") == 0)
	{
		miTheme();
	}

	if(strcmp(taskName, "driveStraight") == 0)
	{
		if(paramLength == 1)
		{
			activeTask=-1;
			m_Robot.driveStraight(parameters[0]);
		}
	}

	if(strcmp(taskName, "driveStraightDistance") == 0)
	{
		if(paramLength == 1)
		{
			activeTask=-1;
			m_Robot.driveStraightDistance(parameters[0], parameters[1], NULL);
		}
	}

//	m_pOs.debug("ActionTime: %i s %i ms", lastAction.sec(), lastAction.ms());
}

void RobotLogic::getCapabilities()
{
#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities start");
#endif
	//uint8 taskListLength = 12;
	uint8 taskListLength=10;
	//const char* taskList[]={"drive","turn","driveDistance","turnInfinite","stop","spread","gather","randomDrive","mitheme","usedemo", "driveStraight", "driveStraightDistance"};
	const char* taskList[]={"drive","turn","driveDistance","turnInfinite","stop","spread","gather","randomDrive","mitheme","usedemo"};
	const char*** paramList;
	//const uint8 paramListLength[]={2,2,3,1,0,2,2,0,0,1,1,2};
	const uint8 paramListLength[]={2,2,3,1,0,2,2,0,0,1};

	// TODO
	uint8 sensorLength = 3;
	char* sensors[]={"battery", "bumperLeft", "bumperRight"};
	uint8 sensorRange[]={0,100,0,1,0,1};

	paramList = ((const char ***)isense::malloc(sizeof (const char **) * taskListLength));
	for (int i = 0; i < taskListLength; ++i)
	{
		int bytesNeeded = (sizeof (const char *) * paramListLength[i]);
		//		m_pOs.debug("multiplikation: %i, sizeof (const char **): %i, paramListLength: %i", bytesNeeded, sizeof (const char *), paramListLength[i]);
		if(bytesNeeded > 0) {
			paramList[i] = ((const char **)isense::malloc(bytesNeeded));
		}
	}
#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities after mallocs");
#endif

	paramList[0][0] = "speed";
	paramList[0][1] = "radius";
	paramList[1][0] = "angle";
	paramList[1][1] = "random";
	paramList[2][0] = "speed";
	paramList[2][1] = "radius";
	paramList[2][2] = "distance";
	paramList[3][0] = "direction";
	paramList[5][0] = "centerID";
	paramList[5][1] = "threshold";
	paramList[6][0] = "centerID";
	paramList[6][1] = "threshold";
	paramList[9][0] = "number";
	/*paramList[10][0] = "speed";
	paramList[11][0] = "speed";
	paramList[11][1] = "distance";*/

	uint16 nodeID = m_pOs.id();

#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities after assigning paramlist members");
#endif

	Communication *m_pComm;
	m_pComm = ((roombatest *) m_pOs.application())->getCommunication();

#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities after getting communication");
#endif

	//	uint8 buf[128];
	m_pComm->sendFeatures(nodeID, taskListLength,
			taskList, paramListLength, paramList,
			sensorLength, sensors, sensorRange);

	for (int i = 0; i < taskListLength; ++i)
	{
		//		m_pOs.debug("getCapabilities: freeing paramlist[%i]", i);
		isense::free (paramList[i]);
	}

#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities after freeing paramlist members");
#endif

	isense::free (paramList);

#ifdef DEBUG_GET_CAPABILITIES
	m_pOs.debug("getCapabilities end");
	m_Robot.setLeds(0x0A, 16, 255);
#endif
}

void RobotLogic::spread(uint16 tempID,uint8 tempThreshold)
{
	activeTask=cSPREAD;
	hops=255;
	centerID = tempID;
	centerThreshold=tempThreshold;
	noNeighborsDetected=false;
	for (int i=0; i<20; i++)
	{
		centerQualityID[i] = BROADCAST;
		centerQuality[i] = 0;
		centerConnected[i] = 0;
	}
}

void RobotLogic::gather(uint16 tempID,uint8 tempThreshold)
{
	activeTask = cGATHER;
	centerID = tempID;
	centerThreshold=tempThreshold;
	for (int i = 0; i<20; i++)
	{
		centerQualityID[i] = BROADCAST;
		centerQuality[i] = 0;
	}
	centerCounter=0;
	if (centerID!=m_pOs.id())
	{
		uint16 temp[] = {200,32768};
		doTask("drive",2,temp);
	}
}

void RobotLogic::randomDrive()
{
	activeTask=cRANDOMDRIVE;
	uint8 linkQuality;
	NeighborhoodMonitor::neighbor* neighbors;
	neighbors = m_neighborhoodMonitor.get_neighbors(
			m_neighborhoodMonitor.SIGNAL_STRENGTH);
	while (neighbors->addr != 0xFFFF) {
		linkQuality = neighbors->value;
		m_pOs.debug("randomDrive, linkQuality: %i", linkQuality);
		*(neighbors++);
	}
	m_Robot.turn(1,180, this);
	uint16 temp[] = {200,32768};
	doTask("drive",2,temp);
}

void RobotLogic::miTheme()
{
	activeTask = cMITHEME;

	uint8 song1[]={88,8,1,8,88,8,91,8,90,8,88,8,86,16,88,32,1,8,86,8,1,8,86,8,84,8,83,8,86,8,84,8};
	m_Robot.setSong(0,song1,16);
	uint8 song2[]={1,8,84,8,1,8,83,8,83,8,1,8,1,8,88,8,1,8,88,24,91,8,90,8,88,8,86,8,86,8,88,24};
	m_Robot.setSong(1,song2,16);
	uint8 song3[]={88,8,1,8,90,8,91,8,1,8,91,8,1,8,93,8,93,8,1,8,1,8,90,8,89,8,1,8,91,8,89,8};
	m_Robot.setSong(2,song3,16);
	uint8 song4[]={88,8,86,8,90,8,91,8,1,8,91,8,1,8,89,8,89,8,1,8,1,16,1,32};
	m_Robot.setSong(3,song4,12);

	songNumber=0;
	m_Robot.playSong(songNumber);
}

void RobotLogic::timeout(void *userdata)
{
//	m_pOs.debug("RobotLogic timeout, ID: %i", *(uint8*) userdata);

	if(userdata != NULL)
	{
		m_pOs.add_task(this, userdata);
	}
	else
	{
		taskBool=m_pOs.add_task(this, NULL);
		//m_pOs.debug("ROBOTLOGIC taskbool:%i",taskBool);
		//m_pOs.debug("ROBOTLOGIC timeoutbool:%i",timeoutBool);
		timeoutCounter++;
		//m_pOs.debug("TimeoutCounter:%i",timeoutCounter);
	}
}

void RobotLogic::execute(void *userdata)
{
	//m_pOs.debug("TIMEOUT ROBOTLOGIC");
	if (activeTask==cSPREAD)
	{
		bool bcenterConnected;
		uint8 linkQuality;
		uint8 neighborCount = 0;
		uint8 ownCenterQuality = 0;
		m_pOs.debug("spread: GetNeighbors ID: %x",m_pOs.id());
		neighbors = m_neighborhoodMonitor.get_neighbors(m_neighborhoodMonitor.SIGNAL_STRENGTH);
		neighborsCopy=neighbors;
		if (neighbors!=NULL)
		{
			while (neighbors->addr != 0xFFFF)
			{
				linkQuality = neighbors->value;
				//m_pOs.debug("spread, Nachbar addr: %x  linkQuality: %i", neighbors->addr, linkQuality);
				neighborCount++;
				linkQualityArray[neighborCount]=linkQuality;
				linkQualityID[neighborCount]=neighbors->addr;
				if (neighbors->addr == centerID) {
					ownCenterQuality=linkQuality;
				}
				*(neighbors++);
			}
			isense::free(neighborsCopy);
		}
		else
		{
			neighborCount=0;
		}

		for (int i=0; i<20; i++)
		{
			centerTimeoutCounter[i]++;
			if (centerTimeoutCounter[i]>maxTimeout)
			{
				centerQualityID[i]=BROADCAST;
				centerQuality[i]=0;
				centerConnected[i]=0;
			}
		}

		if (m_pOs.id()!=centerID)
		{
			bcenterConnected=true;
			uint16 templinkQuality=0;
			if (ownCenterQuality<centerThreshold)
			{
				m_Robot.stop();
				bcenterConnected=false;
				for (int i=0; i<20; i++)
				{
					for (int n=0; n<20; n++)
					{
						if (centerQualityID[i]==linkQualityID[n])
						{
							templinkQuality=linkQualityArray[n];
						}
					}
					if ((centerConnected[i]>0)&&(templinkQuality>centerThreshold))
					{
						if ((centerConnected[i]<=hops+1))//||(hops==0))
							{
							m_pOs.debug("Connected over ID:%x    Hop:%i",centerQualityID[i],centerConnected[i]);
							bcenterConnected=true;
							hops=centerConnected[i];
							}
						else
						{
							m_pOs.debug("Illegaler Hop ID:%x    Hops:%i",centerQualityID[i],centerConnected[i]);
							bcenterConnected=false;
						}
					}
				}
			}
			else
			{
				hops=0;
				m_pOs.debug("Directly Connected:%i",ownCenterQuality);
			}
			uint16 temp[3];
			temp[0]=m_pOs.id();
			if (bcenterConnected)
			{
				temp[1]=hops+1;
			}
			else
			{
				m_pOs.debug("Not Connected");
				temp[1]=0;
			}
			m_pOs.debug("Hops:%i",hops);
			temp[2]=ownCenterQuality;
			m_pCommunication->sendMessage(m_pOs.id(),BROADCAST,"cquality",3,temp);
			//TODO richtige Bezeichnung
			if (bumpsAndWheeldrop&1)
			{
				m_Robot.turn(90,10, this);
			}
			else if (bumpsAndWheeldrop&2)
			{
				m_Robot.turn(-90,10, this);
			}
			else if (bcenterConnected)
			{
				if (noNeighborsDetected)
				{
					uint8 turnscript[] = {137, 0, 200, 0, 0, 157, 0, 180};
					turnAngle=0;
					m_Robot.setScript(turnscript, 8);
					m_Robot.executeScript();
					uint16 temp[] = {200,32768};
					doTask("drive",2,temp);
					noNeighborsDetected = false;
				}
				else
				{
					uint16 temp[] = {200,32768};
					doTask("drive",2,temp);
				}
			}
			//TODO was sinnvolleres als umzudrehen
			else if ((!bcenterConnected)&&(!noNeighborsDetected))
			{
				uint8 turnscript[] = {137, 0, 200, 0, 0, 157, 0, 180};
				turnAngle=0;
				m_Robot.setScript(turnscript, 8);
				m_Robot.executeScript();
				uint16 temp[] = {200,32768};
				doTask("drive",2,temp);
				noNeighborsDetected = true;
			}
		}
		m_pOs.debug("Ende von spread");
	}
	if (activeTask==cGATHER)
	{
		m_pOs.debug("cGATHER");
		uint8 linkQuality;
		timeoutCounter=((timeoutCounter+1)%10);
		if (timeoutCounter==0)
		{
			neighbors = m_neighborhoodMonitor.get_neighbors(m_neighborhoodMonitor.SIGNAL_STRENGTH);
			neighborsCopy=neighbors;
			if (neighbors!=NULL)
			{
				while (neighbors->addr != 0xFFFF)
				{
					linkQuality = neighbors->value;
					//m_pOs.debug("Nachbar addr: %x",neighbors->addr);
					//m_pOs.debug("gather, linkQuality: %i", linkQuality);
					if (neighbors->addr == centerID) {
						centerQuality[centerCounter]=linkQuality;
						m_pOs.debug("centerQuality[%i]: %i",centerCounter,centerQuality[centerCounter]);
						centerCounter=(centerCounter+1)%maxCenterCounter;
					}
					*(neighbors++);
				}
				isense::free(neighborsCopy);
				//m_pCommunication->sendMessage(m_pOs.id(),BROADCAST,"dummytraffic",0,NULL);
				if (m_pOs.id()!=centerID)
				{
					if (centerCounter==0)
					{
						oldGatherSum=gatherSum;
						gatherSum=0;
						for (int i=0;i<maxCenterCounter;i++)
						{
							gatherSum=gatherSum+centerQuality[centerCounter];
						}
						if (gatherSum/maxCenterCounter>centerThreshold)
						{
							m_Robot.setLeds(0, 0, 0);
							m_pOs.debug("Center erreicht");
							m_Robot.stop();
							activeTask=0;
						}

						if ((oldGatherSum/maxCenterCounter)>(gatherSum/maxCenterCounter)+5)
						{
							m_Robot.setLeds(2, 255, 255);
							m_pOs.debug("Gather moved away from %i to %i",(oldGatherSum/maxCenterCounter),(gatherSum/maxCenterCounter));
							uint8 turnscript[] = {137, 0, 200, 0, 0, 157, 0, 90};
							m_Robot.setScript(turnscript, 8);
							m_Robot.executeScript();
							gatherDistance=500;
							m_Robot.driveDistance(200,32768,gatherDistance, this);
							/*uint8 movescript[] = {137, 1, 44, 128, 0, 156, uint8(gatherDistance>>8), uint8(gatherDistance&0xff), 137, 0, 0, 0, 0};
								m_Robot.setScript(movescript, 13);
								m_Robot.executeScript();*/
						}
						else
						{
							m_Robot.setLeds(8, 16, 255);
							gatherDistance=gatherDistance-10;
							if (gatherDistance<200)
							{
								gatherDistance=200;
							}
							uint8 movescript[] = {137, 1, 44, 128, 0, 156, uint8(gatherDistance>>8), uint8(gatherDistance&0xff), 137, 0, 0, 0, 0};
							m_Robot.setScript(movescript, 13);
							m_Robot.executeScript();
						}
					}
				}
			}
		}
	}
	if (activeTask==cRANDOMDRIVE)
	{
		m_pOs.debug("cRANDOMDRIVE");
		uint8 linkQuality;
		uint8 neighborCount = 0;
		m_pOs.debug("randomDrive: GetNeighbors ID: %x",m_pOs.id());
		neighbors = m_neighborhoodMonitor.get_neighbors(m_neighborhoodMonitor.SIGNAL_STRENGTH);
		neighborsCopy=neighbors;
		if (neighbors!=NULL)
		{
			while (neighbors->addr != 0xFFFF)
			{
				linkQuality = neighbors->value;
				m_pOs.debug("Nachbar addr: %x",neighbors->addr);
				m_pOs.debug("randomDrive, linkQuality: %i", linkQuality);
				if (linkQuality > 50)
				{
					neighborCount++;
				}
				*(neighbors++);
			}
			isense::free(neighborsCopy);
		}
		if (neighborCount==0)
		{
			uint8 turnscript[] = {137, 0, 200, 0, 0, 157, 0, 180};
			m_Robot.setScript(turnscript, 8);
			m_Robot.executeScript();
			uint16 temp[] = {200,32768};
			doTask("drive",2,temp);
		}
		if (bumpsAndWheeldrop&1)
		{
			uint8 turnscript[] = {137, 0, 200, 0, 0, 157, 0, 90};
			m_Robot.setScript(turnscript, 8);
			m_Robot.executeScript();
			uint16 temp[] = {200,32768};
			doTask("drive",2,temp);
		}
	}
	timeoutBool=m_pOs.add_timeout_in(Time(MILLISECONDS/10), this, NULL);
}

void RobotLogic::onIoModeChanged(uint8 ioMode)
{
}

void RobotLogic::onPowerStateChanged(PCPOWERSTATE pState)
{
}

void RobotLogic::onCliffStateChanged(PCCLIFFSTATE pState)
{
}

void RobotLogic::onMovementStateChanged(PCMOVEMENTSTATE pState)
{
	turnAngle=turnAngle+pState->angle;
}

void RobotLogic::onButtonChanged(uint8 buttons)
{
	m_pOs.debug("Button changed: %d", buttons);
}

void RobotLogic::onWallSensorChanged(uint8 wall, uint8 virtualWall)
{
}

void RobotLogic::onBumpAndWheelDrop(uint8 bumpsAndWheelDrop)
{
	m_pOs.debug("Bump and wheel drop: %d", bumpsAndWheelDrop);
	bumpsAndWheeldrop = bumpsAndWheelDrop;
}

void RobotLogic::onSongStateChanged(uint8 songNumber, uint8 songPlaying)
{
	if (activeTask==cMITHEME)
	{
		if (songPlaying==0)
		{
			songNumber=songNumber+1;
			m_pOs.debug("MITHEME %i",songNumber);
			m_Robot.playSong(songNumber);
		}
		if (songNumber>3)
		{
			activeTask=-1;
		}
	}

}

void RobotLogic::movementDone()
{

}
