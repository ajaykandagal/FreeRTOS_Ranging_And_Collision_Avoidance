/*******************************************************************************
 * @file	common.h
 * @brief 	This file contains common definitions.
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Dec 09, 2022
 ******************************************************************************/
#ifndef COMMON_H_
#define COMMON_H_


#include "MKL25Z4.h"


#define DEBUG_LOGS 	(0)
#define TESTING		(0)


// Contains number of number of enums in e_proximity_t except PROXIMITY_SAFE
#define TOF_PROXIMITY_SLOTS		(5)


typedef enum {
	PROXIMITY_TOO_CLOSE = 0,
	PROXIMITY_CLOSE,
	PROXIMITY_MID,
	PROXIMITY_FAR,
	PROXIMITY_QUITE_FAR,
	PROXIMITY_SAFE
} e_proximity_t;


static const uint16_t proximity_slots[TOF_PROXIMITY_SLOTS] = {100, 150, 200, 250, 300};


#endif /* COMMON_H_ */
