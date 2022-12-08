/*
 * common.h
 *
 *  Created on: 07-Dec-2022
 *      Author: ajayk
 */

#ifndef COMMON_H_
#define COMMON_H_


#define DEBUG_LOGS 0


typedef enum {
	PROXIMITY_SAFE = 0,
	PROXIMITY_QUITE_FAR,
	PROXIMITY_FAR,
	PROXIMITY_MID,
	PROXIMITY_CLOSE,
	PROXIMITY_QUITE_CLOSE
} e_proximity_t;


#endif /* COMMON_H_ */
