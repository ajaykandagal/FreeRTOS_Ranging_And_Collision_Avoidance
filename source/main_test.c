/*******************************************************************************
 * @file	main_test.c
 * @brief 	This file contains functions to test ToF sensor and the buzzer.
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @data 	Dec 07, 2022
 ******************************************************************************/
#include "MKL25Z4.h"
#include "vl53l0x.h"
#include "fsl_debug_console.h"

#include "main_test.h"


// Set this value to the distance from the sensor
#define OBSTACLE_DISTANE_MM		(300)

#define ALLOWED_RANGE_ERROR		(10)
#define UPPER_RANGE_OFFSET		(OBSTACLE_DISTANE_MM + ALLOWED_RANGE_ERROR)
#define LOWER_RANGE_OFFSET		(OBSTACLE_DISTANE_MM - ALLOWED_RANGE_ERROR)
#define TEST_TOF_SAMPLES   		(100)


/*******************************************************************************
 * PUBLIC FUNCTION: SEE HEADER FILE FOR FULL DESCRIPTION
 *
 * This test will check the ToF sensor working. To carry out this test ToF
 * sensor should be placed at a known distance from the obstacle. Set the macro
 * OBSTACLE_DISTANE_MM with known object distance.
 ******************************************************************************/
void test_tof_sensor(vl53l0x_idx_t idx)
{
	uint16_t range_average = 0;
	uint16_t failed_count = 0;
	uint16_t passed_count = 0;
	uint16_t out_of_range_count = 0;
	uint16_t range = 0;

	for(uint16_t i = 0; i < TEST_TOF_SAMPLES; i++)
	{
		range = tof_get_range(VL53L0X_IDX_FIRST);

		if (range != VL53L0X_OUT_OF_RANGE)
		{
			if (range >= LOWER_RANGE_OFFSET && range <= UPPER_RANGE_OFFSET)
			{
				range_average = (range_average + range) / 2;
				passed_count++;
			}
			else
			{
				failed_count++;
			}
		}
		else
		{
			out_of_range_count++;
		}
	}

	PRINTF("\n\rTesting Completed!");
	PRINTF("\n\r\tPassed samples = %u\n\r\tFailed samples = %u"
			"\n\r\tOut of range samples = %u", passed_count, failed_count, out_of_range_count);

	uint16_t range_error = 0;
	float accuracy = 0;

	if (range_average > OBSTACLE_DISTANE_MM)
		range_error = (range_average - OBSTACLE_DISTANE_MM) / TEST_TOF_SAMPLES;
	else
		range_error = (OBSTACLE_DISTANE_MM - range_average) / TEST_TOF_SAMPLES;

	accuracy = 100 - (((float)range_error / OBSTACLE_DISTANE_MM) * 100);

	PRINTF("\n\r\tAccuracy of sensor = %f%", accuracy);

	if (out_of_range_count > passed_count)
	{
		PRINTF("\n\r\tSensor has too many out of range values. "
				"\n\r\t\t This could be because of"
				"\n\r\t\t\t*Ambient light"
				"\n\r\t\t\t*The obstacle surface reflects light poorly"
				"\n\r\t\t\t*The sensor is damaged");
	}

	if (failed_count > passed_count)
	{
		PRINTF("\n\r\tSensor readings are unreliable."
				"\n\r\t\t This could be because of"
				"\n\r\t\t\t*Ambient light"
				"\n\r\t\t\t*The obstacle surface reflects light poorly"
				"\n\r\t\t\t*The sensor is damaged");
	}
}


/*******************************************************************************
 * PUBLIC FUNCTION
 *
 * This is a observation based test. The test will verify the buzzer on period
 * by passing various proximity values and then plays the tone by passing the
 * valid proximity value.
 ******************************************************************************/
void test_buzzer()
{

}
