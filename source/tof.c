/*******************************************************************************
 * @file	tof.c
 * @brief 	This file contains functions to initialize ToF sensors and reads
 * 			the range value from sensors and returns the value.
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Dec 07, 2022
 ******************************************************************************/
#include <stdlib.h>
#include "MKL25Z4.h"

#include "tof.h"
#include "i2c.h"
#include "vl53l0x.h"
#include "fsl_debug_console.h"


/* Data format of ToF sensor info */
typedef struct vl53l0x_info
{
	uint8_t addr;
	uint8_t lpm_gpio_pin;
} vl53l0x_info_t;



/* Data a each connected ToF sensor */
static vl53l0x_info_t vl53l0x_infos[] =
{
		{
				.addr = 0x30,
				.lpm_gpio_pin = 4
		},
		{
				.addr = 0x31,
				.lpm_gpio_pin = 5
		}
};


/*******************************************************************************
 * Returns the ToF sensor data of given id.
 *
 * @param
 *  idx		Id of ToF sensor.
 *
 * @return
 *  Returns the pointer to valid ToF data if the sensor is found else returns
 *  NULL.
 *
 ******************************************************************************/
vl53l0x_info_t* const get_tof_sensor(vl53l0x_idx_t idx)
{
	switch (idx)
	{
#if VL53L0X_SENSORS_COUNT >= 1
	case VL53L0X_IDX_FIRST:
		return &vl53l0x_infos[0];
#endif
#if VL53L0X_SENSORS_COUNT >= 2
	case VL53L0X_IDX_SECOND:
		return &vl53l0x_infos[1];
#endif
	default:
		return NULL;
	}
}


/*******************************************************************************
 * PUBLIC FUNCTION
 *
 * Configures the ToF sensors and initializes their Load Power Management pins
 * and turns on the sensors.
 ******************************************************************************/
void tof_init()
{
	vl53l0x_info_t* tof_sensor;

	i2c_init();

	for (uint8_t i = 0; i < VL53L0X_SENSORS_COUNT; i++)
	{
		tof_sensor = get_tof_sensor(i);

		if (tof_sensor != NULL)
		{
			SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
			PORTE->PCR[tof_sensor->lpm_gpio_pin] &= ~((uint32_t) PORT_PCR_MUX_MASK);
			PORTE->PCR[tof_sensor->lpm_gpio_pin] |= ((uint32_t) PORT_PCR_MUX(1));

			GPIOE->PDDR |= ((uint32_t)( 1 << tof_sensor->lpm_gpio_pin));

			GPIOE->PDOR |= ((uint32_t)( 1 << tof_sensor->lpm_gpio_pin));

			if (vl53l0x_init(tof_sensor->addr))
				PRINTF("\n\rToF Sensor %u initialized", i);
		}
	}
}


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FOR FULL DESCRIPTION
 *
 * Gets the range value from the ToF sensor by doing single ranging measurement.
 ******************************************************************************/
uint16_t tof_get_range(vl53l0x_idx_t idx)
{
	vl53l0x_info_t* tof_sensor = get_tof_sensor(idx);

	if (tof_sensor != NULL)
	{
		uint16_t range;
		if (vl53l0x_read_range_single(tof_sensor->addr, &range))
			return range;
		else
			return VL53L0X_OUT_OF_RANGE;
	}
	return VL53L0X_OUT_OF_RANGE;
}

