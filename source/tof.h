/*******************************************************************************
 * @file	tof.h
 * @brief 	This file contains functions to initialize ToF sensors and reads
 * 			the range value from sensors and returns the value.
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Dec 07, 2022
 ******************************************************************************/
#ifndef TOF_H_
#define TOF_H_


/* Number of ToF sensors */
#define VL53L0X_SENSORS_COUNT 1


/* Id of each connected ToF sensors */
typedef enum
{
	VL53L0X_IDX_FIRST = 0,
	VL53L0X_IDX_SECOND
} vl53l0x_idx_t;



/*******************************************************************************
 * Configures the ToF sensors and initializes their Load Power Management pins
 * and turns on the sensors.
 ******************************************************************************/
void tof_init();


/*******************************************************************************
 * Gets the range value from the ToF sensor by doing single ranging measurement.
 *
 * @param
 *  idx		Id of ToF sensor.
 *
 * @return
 *  Returns the range value. If range is invalid then returns
 *  VL53L0X_OUT_OF_RANGE defined in vl53l0x.h.
 ******************************************************************************/
uint16_t tof_get_range(vl53l0x_idx_t idx);

#endif /* TOF_H_ */
