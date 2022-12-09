/*******************************************************************************
 * @file	vl53l0x.h
 * @brief	This code is heavily inspired from Niklas' code. STM doesn't provide
 * 			register mapping documentation for vl5310x ToF sensor and instead
 * 			they have given API library which has to be modified for platform
 * 			specific I2C bus implementation. The library is results in quite
 * 			large memory footprint and uses C++ approach. I have decided to use
 * 			Niklas' code which only implements basic configuration and single
 * 			ranging.
 *
 * @author	Niklas Nilsson
 * @link	https://github.com/artfulbytes/vl6180x_vl53l0x_msp430.git
 *
 * @editor	Dec 01, 2022, Ajay Kandagal, ajaykandagal94@gmail.com
 * @change	Functions have been modified to handle a single ToF sensor based on
 * 			address passed. A bug was observed while taking single measurement
 * 			and has been corrected as per STM's API documentation.
 *
 ******************************************************************************/
#ifndef VL53L0X_H
#define VL53L0X_H


#include <stdbool.h>
#include <stdint.h>


#define VL53L0X_OUT_OF_RANGE (8190)
#define VL53L0X_DEVICEMODE_SINGLE_RANGING (0)
#define VL53L0X_DEVICEMODE_CONTINUOUS_RANGING (1)
#define VL53L0X_ERROR_NONE (1)


/**
 * Initializes the sensors to the given address.
 *
 * @note
 *  Each sensor must have its XSHUT pin connected.
 */
bool vl53l0x_init(const uint8_t addr);


/**
 * Does a single range measurement.
 *
 * @param
 *  addr 	Selects sensor of given address.
 *  range 	Contains the measured range or VL53L0X_OUT_OF_RANGE
 *        	if out of range.
 *
 * @return
 *  True if success, False if error
 *
 * @note
 *  Polling-based
 */
bool vl53l0x_read_range_single(const uint8_t addr, uint16_t *range);


#endif /* VL53L0X_H */
