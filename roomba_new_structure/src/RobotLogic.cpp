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
//#define DEBUG_SWARM

// Sensor names
#define BATTERY				"battery"
#define BUMP_LEFT			"BumpL"
#define BUMP_RIGHT			"BumpR"
#define WHEEL_DROP_LEFT		"DropL"
#define WHEEL_DROP_RIGHT	"DropR"

// task name
#define TASK_DRIVE				"drive"
#define TASK_TURN				"turn"
#define TASK_DRIVE_DISTANCE		"driveDistance"
#define TASK_TURN_INFINITE		"turnInfinite"
#define TASK_STOP				"stop"
#define TASK_SPREAD				"spread"
#define TASK_GATHER				"gather"
#define TASK_MITHEME			"miTheme"
#define TASK_DEMO				"demo"
#define TASK_DRIVE_STRAIGHT		"driveStraight"
#define TASK_DRIVE_STRAIGHT_DISTANCE		"driveStraightDistance"


#define GUI_ID				0


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
	if(strcmp(taskName, TASK_DRIVE) == 0)
	{
		if(paramLength == 2)
		{
#ifdef DEBUG_DOTASK
			m_pOs.debug("doTask: drive  Param0:%i  Param1:%i",parameters[0],parameters[1]);
#endif
			m_Robot.drive((uint16) parameters[0], (uint16) parameters[1]);
		}
	}

	if(strcmp(taskName, TASK_TURN) == 0)
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

	if(strcmp(taskName, TASK_TURN_INFINITE) == 0)
	{
		if(paramLength == 1)
		{
			activeTask=-1;
#ifdef DEBUG_DOTASK
			m_pOs.debug("doTask: turnInfinite");
#endif
			m_Robot.turnInfinite((uint16) parameters[0]);
		}
	}

	if(strcmp(taskName, TASK_STOP) == 0)
	{
		activeTask=-1;
#ifdef DEBUG_DOTASK
		m_pOs.debug("doTask: stop");
#endif
		m_Robot.stop();
	}

	if(strcmp(taskName, TASK_DRIVE_DISTANCE) == 0)
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

	if (strcmp(taskName, TASK_SPREAD) == 0)
	{
#ifdef DEBUG_DOTASK
		m_pOs.debug("doTask: spread");
#endif
		spread(parameters[0],parameters[1]);
	}

	if (strcmp(taskName, TASK_GATHER) == 0)
	{
		gather(parameters[0],parameters[1]);
	}

	//this task creates dummy traffic for the neighborhoodmonitor
	if (strcmp(taskName, "patapatapata") == 0)
	{
		//m_pOs.debug("patapatapata erhalten in ID: %i",m_pOs.id());
		m_pCommunication->sendMessage(m_pOs.id(),BROADCAST,"pon",0,NULL);
	}

	//this task creates dummy traffic for the neighborhoodmonitor
	if (strcmp(taskName, "pon") == 0)
	{
		//m_pCommunication->sendMessage(0,"pon",0,NULL);
		m_pOs.debug("pon erhalten in ID: %i",m_pOs.id());
		if (activeTask==cGATHER)
		{
			if ((swarmState!=cGATHERDISTANCEWAIT)&&(swarmState!=cGATHERTURNWAIT))
			{
				m_pOs.add_task(this, NULL);
			}
		}
	}

	//this task manages the link qualities sent by other robots during the swarm behaviors
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
				centerTimeoutCounter[i]=0;
				//m_pOs.debug("Centerquality[%i] von %x ist %i %i ",i,tempID,parameters[1],parameters[2]);
				break;
			}
		}
	}

	if (strcmp(taskName, TASK_DEMO) == 0)
	{
		activeTask=-1;
		m_Robot.startDemo(parameters[0]);
	}

	if(strcmp(taskName, TASK_MITHEME) == 0)
	{
		miTheme();
	}

	if(strcmp(taskName, TASK_DRIVE_STRAIGHT) == 0)
	{
		if(paramLength == 1)
		{
			activeTask=-1;
			m_Robot.driveStraight(parameters[0]);
		}
	}

	if(strcmp(taskName, TASK_DRIVE_STRAIGHT_DISTANCE) == 0)
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
	uint8 taskListLength=9;
	//const char* taskList[]={"drive","turn","driveDistance","turnInfinite","stop","spread","gather","randomDrive","mitheme","usedemo", "driveStraight", "driveStraightDistance"};
	const char* taskList[]={TASK_DRIVE,TASK_TURN,TASK_DRIVE_DISTANCE,TASK_TURN_INFINITE
			,TASK_STOP,TASK_SPREAD,TASK_GATHER,TASK_MITHEME,TASK_DEMO};
	const char*** paramList;
	//const uint8 paramListLength[]={2,2,3,1,0,2,2,0,0,1,1,2};
	const uint8 paramListLength[]={2,2,3,1,0,2,2,0,1};

	// TODO
	uint8 sensorLength = 5;
	char* sensors[]={ BATTERY, BUMP_LEFT, BUMP_RIGHT, WHEEL_DROP_LEFT, WHEEL_DROP_RIGHT };
	uint8 sensorRange[]={0,100,0,1,0,1, 0, 1, 0, 1};

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
	paramList[8][0] = "number";
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
	centerFound=false;
}

void RobotLogic::gather(uint16 tempID,uint8 tempThreshold)
{
	activeTask = cGATHER;
	centerID = tempID;
	centerThreshold=tempThreshold;
	gatherDistance=1000;
	for (int i = 0; i<20; i++)
	{
		centerQualityID[i] = BROADCAST;
		centerQuality[i] = 0;
	}
	centerCounter=0;
	if (centerID!=m_pOs.id())
	{
		//uint16 temp[] = {200,32768};
		//doTask("drive",2,temp);
		//m_Robot.driveStraightDistance(200,500,this);
		//swarmState=cGATHERDISTANCEWAIT;
	}
	uint8 song1[]={88,8,1,8};
	m_Robot.setSong(0,song1,2);
	m_Robot.playSong(0);
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
#ifdef DEBUG_SWARM
		m_pOs.debug("randomDrive, linkQuality: %i", linkQuality);
#endif
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
		//timeoutCounter++;
		//m_pOs.debug("TimeoutCounter:%i",timeoutCounter);
	}
}

void RobotLogic::execute(void *userdata)
{

	if (activeTask!=-1) {
		if (bumpsAndWheeldrop&1)
		{
			m_Robot.turn(90,10,this);
			swarmState=cBUMPSWAIT;
			return;
		}
		else if (bumpsAndWheeldrop&2)
		{
			m_Robot.turn(-90,10,this);
			swarmState=cBUMPSWAIT;
			return;
		}

		if (swarmState==cTURNWAIT)
		{
			//uint16 temp[] = {200,32768};
			//doTask("drive",2,temp);
			m_Robot.driveStraight(200);
			swarmState=cNONE;
		}
		if (swarmState==cGATHERTURNWAIT)
		{
			m_Robot.driveStraightDistance(200,gatherDistance, this);
			swarmState=cNONE;
		}
		if ((swarmState==cBUMPSWAIT)||(swarmState==cGATHERDISTANCEWAIT))
		{
			swarmState=cNONE;
		}
	}

	//m_pOs.debug("TIMEOUT ROBOTLOGIC");
	if (activeTask==cSPREAD)
	{
		bool bcenterConnected;
		uint8 linkQuality;
		uint8 neighborCount = 0;
		uint8 ownCenterQuality = 0;
		uint16* linkArray;
		linkArray=((uint16*)isense::malloc(40));
#ifdef DEBUG_SWARM
		m_pOs.debug("spread: GetNeighbors ID: %x",m_pOs.id());
#endif
		neighbors = m_neighborhoodMonitor.get_neighbors(m_neighborhoodMonitor.SIGNAL_STRENGTH);
		neighborsCopy=neighbors;
		if (neighbors!=NULL)
		{
			while (neighbors->addr != 0xFFFF)
			{
				linkQuality = neighbors->value;
#ifdef DEBUG_SWARM
				m_pOs.debug("spread, Nachbar addr: %x  linkQuality: %i", neighbors->addr, linkQuality);
#endif
				neighborCount++;
				linkQualityArray[neighborCount]=linkQuality;
				linkQualityID[neighborCount]=neighbors->addr;
				if (neighbors->addr == centerID)
				{
					ownCenterQuality=linkQuality;
				}
				*(neighbors++);
				linkArray[neighborCount*2]=(uint16)neighbors->addr;
				linkArray[neighborCount*2+1]=(uint16)linkQuality;
			}
			m_pCommunication->sendMessage(m_pOs.id(), GUI_ID, "linkStatus", neighborCount*2, linkArray);
			isense::free(neighborsCopy);
			isense::free(linkArray);
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

		if ((m_pOs.id()!=centerID)&&(neighborCount>0))
		{
			bcenterConnected=true;
//			uint16 templinkQuality=0;
			if (ownCenterQuality<centerThreshold)
			{
				/*m_Robot.stop();
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
				}*/
				bcenterConnected=false;
				for (int i=0; i<20; i++)
				{
					if ((centerConnected[i]==1)&&(centerQualityID[i]<m_pOs.id()))
					{
						for (int n=0; n<20; n++)
						{
							if (linkQualityID[n]==centerQualityID[i])
							{
								if (linkQualityArray[n]>=centerThreshold)
								{
									bcenterConnected=true;
									break;
								}
							}
						}
					}
					if (bcenterConnected)
					{
						break;
					}
				}
			}
			else
			{
				bcenterConnected=true;
				//hops=0;
#ifdef DEBUG_SWARM
				m_pOs.debug("Directly Connected:%i",ownCenterQuality);
#endif
			}
			uint16 temp[3];
			temp[0]=m_pOs.id();
			if ((bcenterConnected)||(centerFound))
			{
				temp[1]=1;
			}
			else
			{
#ifdef DEBUG_SWARM
				m_pOs.debug("Not Connected");
#endif
				temp[1]=0;
			}
			//m_pOs.debug("Hops:%i",hops);
			temp[2]=ownCenterQuality;
			m_pCommunication->sendMessage(m_pOs.id(),BROADCAST,"cquality",3,temp);
			//TODO richtige Bezeichnung
			if (bcenterConnected)
			{
				if (noNeighborsDetected)
				{
					if (ownCenterQuality>centerThreshold+5)
					{
						centerFound=true;
						m_Robot.stop();
					}
					//m_Robot.turn(180,0,this);
					//swarmState=cTURNWAIT;
					//noNeighborsDetected = false;
					//return
				}
				else
				{
					m_Robot.driveStraight(200);
				}
			}
			//TODO was sinnvolleres als umzudrehen
			else if ((!bcenterConnected)&&(!noNeighborsDetected))
			{
				/*uint8 turnscript[] = {137, 0, 200, 0, 0, 157, 0, 180};
				m_Robot.setScript(turnscript, 8);
				m_Robot.executeScript();*/
				m_Robot.turn(180,0,this);
				swarmState=cTURNWAIT;
				noNeighborsDetected = true;
				return;
				/*uint16 temp[] = {200,32768};
				doTask("drive",2,temp);*/
			}
		}
#ifdef DEBUG_SWARM
		m_pOs.debug("Ende von spread");
#endif
	}
	if (activeTask==cGATHER)
	{
		uint8 linkQuality;
		uint16* linkArray;
		linkArray=((uint16*)isense::malloc(2));
#ifdef DEBUG_SWARM
		m_pOs.debug("cGATHER");
#endif
		neighbors = m_neighborhoodMonitor.get_neighbors(m_neighborhoodMonitor.SIGNAL_STRENGTH);
		neighborsCopy=neighbors;
		if (neighbors!=NULL)
		{
			while (neighbors->addr != 0xFFFF)
			{
				linkQuality = neighbors->value;
#ifdef DEBUG_SWARM
				m_pOs.debug("gather, Nachbar addr: %x   linkQuality: %i",neighbors->addr, linkQuality);
#endif
				if (neighbors->addr == centerID)
				{
					centerQuality[centerCounter]=linkQuality;
#ifdef DEBUG_SWARM
					m_pOs.debug("centerQuality[%i]: %i",centerCounter,centerQuality[centerCounter]);
#endif
					centerCounter=(centerCounter+1)%maxCenterCounter;
					linkArray[0]=centerID;
					linkArray[1]=linkQuality;
				}
				m_pCommunication->sendMessage(m_pOs.id(), GUI_ID, "linkStatus", 2, linkArray);
				*(neighbors++);
			}
			isense::free(neighborsCopy);
			isense::free(linkArray);
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
#ifdef DEBUG_SWARM
						m_pOs.debug("Center erreicht");
#endif
						m_Robot.stop();
						activeTask=0;
						swarmState=cNONE;
						uint8 song1[]={88,8,1,8,88,8,91,8};
						m_Robot.setSong(0,song1,4);
						m_Robot.playSong(0);
					}
					else if ((oldGatherSum/maxCenterCounter)>(gatherSum/maxCenterCounter)+5)
					{
#ifdef DEBUG_SWARM
						m_pOs.debug("Gather moved away from %i to %i",(oldGatherSum/maxCenterCounter),(gatherSum/maxCenterCounter));
#endif
						m_Robot.turn(90,0,this);
						swarmState=cGATHERTURNWAIT;
						gatherDistance=1000;
						return;
						//m_Robot.driveDistance(200,32768,gatherDistance, this);
						/*uint8 movescript[] = {137, 1, 44, 128, 0, 156, uint8(gatherDistance>>8), uint8(gatherDistance&0xff), 137, 0, 0, 0, 0};
								m_Robot.setScript(movescript, 13);
								m_Robot.executeScript();*/
					}
					else
					{
						//m_Robot.setLeds(8, 16, 255);
						gatherDistance=gatherDistance-100;
						if (gatherDistance<200)
						{
							gatherDistance=200;
						}
#ifdef DEBUG_SWARM
						m_pOs.debug("Gather fahr weiter GatherDistance: %i",gatherDistance);
#endif
						m_Robot.driveStraightDistance(200,gatherDistance,this);
						swarmState=cGATHERDISTANCEWAIT;
						return;
					}
				}
			}
		}
	}
	if (activeTask==cRANDOMDRIVE)
	{
#ifdef DEBUG_SWARM
		m_pOs.debug("cRANDOMDRIVE");
#endif
		uint8 linkQuality;
		uint8 neighborCount = 0;
#ifdef DEBUG_SWARM
		m_pOs.debug("randomDrive: GetNeighbors ID: %x",m_pOs.id());
#endif
		neighbors = m_neighborhoodMonitor.get_neighbors(m_neighborhoodMonitor.SIGNAL_STRENGTH);
		neighborsCopy=neighbors;
		if (neighbors!=NULL)
		{
			while (neighbors->addr != 0xFFFF)
			{
				linkQuality = neighbors->value;
#ifdef DEBUG_SWARM
				m_pOs.debug("Nachbar addr: %x",neighbors->addr);
				m_pOs.debug("randomDrive, linkQuality: %i", linkQuality);
#endif
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
	}
	if (activeTask!=cGATHER)
	{
		timeoutBool=m_pOs.add_timeout_in(Time(MILLISECONDS/10), this, NULL);
	}
}

void RobotLogic::onIoModeChanged(uint8 ioMode)
{
}

void RobotLogic::onPowerStateChanged(PCPOWERSTATE pState)
{
	uint16 charge_percentage;
	charge_percentage = (uint16)(((uint32)pState->batteryCharge)*100 / pState->batteryCapacity);
	m_pCommunication->sendMessage(m_pOs.id(), GUI_ID, "sensor_" BATTERY, 1, &charge_percentage);
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
	uint16 value;

	m_bumpsAndWheelDrop = bumpsAndWheelDrop;

	value = (bumpsAndWheelDrop & 1) ? 1 : 0;
	m_pCommunication->sendMessage(m_pOs.id(), GUI_ID, "sensor_" BUMP_RIGHT, 1, &value);
	value = (bumpsAndWheelDrop & 2) ? 1 : 0;
	m_pCommunication->sendMessage(m_pOs.id(), GUI_ID, "sensor_" BUMP_LEFT, 1, &value);
	value = (bumpsAndWheelDrop & 4) ? 1 : 0;
	m_pCommunication->sendMessage(m_pOs.id(), GUI_ID, "sensor_" WHEEL_DROP_RIGHT, 1, &value);
	value = (bumpsAndWheelDrop & 8) ? 1 : 0;
	m_pCommunication->sendMessage(m_pOs.id(), GUI_ID, "sensor_" WHEEL_DROP_LEFT, 1, &value);


	bumpsAndWheeldrop = bumpsAndWheelDrop;
	m_pOs.add_task(this, NULL);
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
	m_pOs.debug("MovementDone in %x",m_pOs.id());
	if ((swarmState==cTURNWAIT)||(swarmState==cBUMPSWAIT)||(swarmState==cGATHERTURNWAIT)||(swarmState==cGATHERDISTANCEWAIT))
	{
		m_pOs.add_task(this, NULL);
	}
}
