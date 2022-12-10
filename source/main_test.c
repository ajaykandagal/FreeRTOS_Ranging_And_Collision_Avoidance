/*******************************************************************************
 * @file	main_test.c
 * @brief 	This file contains functions to test ToF sensor and the buzzer.
 *
 * @author 	Ajaykumar Kandagal, ajka9053@colorado.edu
 * @date 	Dec 07, 2022
 *
 * @editor	Dec 09, 2022, Ajay Kandagal, ajka9053@colorado.edu
 * @change	Added test cases for buzzer.
 ******************************************************************************/
#include "MKL25Z4.h"
#include "fsl_debug_console.h"

#include "common.h"
#include "tof.h"
#include "vl53l0x.h"
#include "buzzer.h"
#include "main_test.h"


// Set this value to the distance from the sensor
#define OBSTACLE_DISTANE_MM		(300)

#define ALLOWED_RANGE_ERROR		(10)
#define UPPER_RANGE_OFFSET		(OBSTACLE_DISTANE_MM + ALLOWED_RANGE_ERROR)
#define LOWER_RANGE_OFFSET		(OBSTACLE_DISTANE_MM - ALLOWED_RANGE_ERROR)
#define TEST_TOF_SAMPLES   		(100)
#define COUNTER_1MS_VALUE 		(3800)


typedef struct{
	e_proximity_t in_proximity;
	uint16_t exp_buzzer_on_period;
}buzzer_test_t;


/*******************************************************************************
 * This test will check the ToF sensor working. To carry out this test ToF
 * sensor should be placed at a known distance from the obstacle. Set the macro
 * OBSTACLE_DISTANE_MM with known object distance.
 *
 * @param
 *  idx		Id of ToF sensor to test.
 *
 ******************************************************************************/
void test_tof_sensor(vl53l0x_idx_t idx)
{
	uint16_t range_average = 0;
	uint16_t failed_count = 0;
	uint16_t passed_count = 0;
	uint16_t out_of_range_count = 0;
	uint16_t range = 0;

	PRINTF("\n\n\r***");
	PRINTF("\n\rTesting ToF Sensor."
			"\n\rMake sure the sensor is placed at specified distance from obstacle");

	for(uint16_t i = 0; i < TEST_TOF_SAMPLES; i++)
	{
		range = tof_get_range(idx);

		if (range != VL53L0X_OUT_OF_RANGE)
		{
			range_average = (range_average + range) / 2;

			if (range >= LOWER_RANGE_OFFSET && range <= UPPER_RANGE_OFFSET)
				passed_count++;
			else
				failed_count++;
		}
		else
		{
			out_of_range_count++;
		}
	}

	PRINTF("\n\n\rTesting Completed!");
	PRINTF("\n\rPassed samples = %u"
			"\n\rFailed samples = %u"
			"\n\rOut of range samples = %u", passed_count, failed_count, out_of_range_count);

	if (passed_count > failed_count && passed_count > out_of_range_count)
	{
		uint16_t range_error = 0;
		uint16_t accuracy = 0;

		if (range_average > OBSTACLE_DISTANE_MM)
			range_error = (range_average - OBSTACLE_DISTANE_MM);
		else
			range_error = (OBSTACLE_DISTANE_MM - range_average);

		accuracy = 100 - ((range_error * 100 / OBSTACLE_DISTANE_MM));

		PRINTF("\n\rAccuracy of sensor (in percent) = %u", accuracy);
	}

	if (out_of_range_count > passed_count)
	{
		PRINTF("\n\rSensor has too many out of range values. "
				"\n\r This could be because of"
				"\n\r\t*Ambient light"
				"\n\r\t*The obstacle surface reflects light poorly"
				"\n\r\t*The sensor is damaged");
	}

	if (failed_count > passed_count)
	{
		PRINTF("\n\rSensor readings are unreliable."
				"\n\r This could be because of"
				"\n\r\t*Ambient light"
				"\n\r\t*The obstacle surface reflects light poorly"
				"\n\r\t*The sensor is damaged");
	}
}



/*******************************************************************************
 * Creates delay in milliseconds using for loops.
 *
 * @param
 *  time_ms		Amount of delay to be created in milliseconds.
 *
 ******************************************************************************/
void loop_delay_ms(uint32_t time_ms)
{
	// Creates required amount of delay by doing nothing
	for (unsigned long int i = 0; i < time_ms; i++)
	{
		for (unsigned long int j = 0; j < COUNTER_1MS_VALUE; j++)
			__asm volatile ("NOP");
	}
}


/*******************************************************************************
 * The test will verify the buzzer on period by passing various proximity values.
 * Next will be hearing based test, the buzzer is played by passing the valid
 * proximity value. You should be able to able to hear the buzzer with different
 * on periods.
 ******************************************************************************/
void test_buzzer()
{
	uint16_t failed_cases = 0;
	uint16_t passed_cases = 0;

	buzzer_test_t buzzer_test[] = {
			{PROXIMITY_SAFE, 0},
			{PROXIMITY_QUITE_FAR, 100},
			{PROXIMITY_FAR, 200},
			{PROXIMITY_MID, 300},
			{PROXIMITY_CLOSE, 400},
			{PROXIMITY_TOO_CLOSE, 500},
			{200, 0},
			{-1, 0},
	};

	PRINTF("\n\n\r***");
	PRINTF("\n\rTesting Buzzer."
			"\n\rThis tests the buzzer setup function");

	for(uint16_t i = 0; i < (sizeof(buzzer_test) / sizeof(buzzer_test_t)); i++)
	{
		buzzer_setup(buzzer_test[i].in_proximity);

		if(get_buzzer_on_period() == buzzer_test[i].exp_buzzer_on_period)
		{
			passed_cases++;
			PRINTF("\n\r\tTest case %u passed", i);
		}
		else
		{
			failed_cases++;
			PRINTF("\n\r\tTest case %u failed", i);
		}
	}

	PRINTF("\n\n\rTesting Completed!");

	if (passed_cases == (sizeof(buzzer_test) / sizeof(buzzer_test_t)))
		PRINTF("\n\rAll buzzer test cases passed!");
	else
		PRINTF("\n\r%u test cases passed and %u test cases failed", passed_cases, failed_cases);


	PRINTF("\n\n\r***");
	PRINTF("\n\n\rNow the buzzer will be played at different on/off periods."
			"\n\rPlease check if you can hear the sound");

	for(uint16_t i = 0; i < PROXIMITY_SAFE; i++)
	{
		buzzer_setup(i);

		for (uint8_t j = 0; j < 3; j++)
		{
			buzzer_start();
			loop_delay_ms(get_buzzer_on_period());
			buzzer_stop();
			loop_delay_ms(get_buzzer_off_period());
		}
	}
}

/*******************************************************************************
 * PUBLIC FUNCTION
 *
 * A common call to test both ToF sensor and the buzzer.
 ******************************************************************************/
void main_test()
{
	test_tof_sensor(VL53L0X_IDX_FIRST);
	test_buzzer();
}
