/*
 * common.h
 *
 *  Created on: 07-Dec-2022
 *      Author: ajayk
 */

#ifndef COMMON_H_
#define COMMON_H_


#define DEBUG_LOGS 0

#define TOF_PROXIMITY_SLOTS		(5)

typedef enum {
	PROXIMITY_QUITE_CLOSE = 0,
	PROXIMITY_CLOSE,
	PROXIMITY_MID,
	PROXIMITY_FAR,
	PROXIMITY_QUITE_FAR,
	PROXIMITY_SAFE
} e_proximity_t;

static const uint16_t proximity_slots[TOF_PROXIMITY_SLOTS] = {100, 130, 150, 180, 200};


#endif /* COMMON_H_ */
