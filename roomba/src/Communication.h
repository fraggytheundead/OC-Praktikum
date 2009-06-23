/*
 * Communication.h
 *
 *  Created on: 04.06.2009
 *      Author: Administrator
 */
//ueberlegen, die methoden statisch zu machen

#ifndef __OCPROJ_COMMUNICATION_H
#define __OCPROJ_COMMUNICATION_H

#include <isense/os.h>
#include <isense/protocols/routing/flooding.h>

class Communication {
public:
	Communication(isense::Os& os);
	virtual ~Communication();

	uint8 sendFeatures(uint16 robotId, uint8 taskListLength, const char ** taskList,
			const uint8 * paramListLength, const char ***  paramList, uint8 *buf);
	uint8 sendMessage(uint16 robotId, const char * taskName, uint8 valueLength,
			const uint16 * values, uint8* buf);
	void decodeMessage(uint8 len, const uint8 * buf);

private:
	isense::Os& m_os;
};

#endif /* __OCPROJ_COMMUNICATION_H */
